# import queue
import numpy as np
import CVFunc
import signal
import math
import xlwt
import binascii
import serial
import time
import cv2
import datetime
import multiprocessing as mp
import os
import JY61

# 读取模型参数
file_model = open('Model.txt', 'r', encoding='utf-8')
para_lines = file_model.readlines()
file_model.close()

# Canny参数初始化模式
cali_type = 3
minCan = 40
maxCan = 100

# 前后单目、IMU串口、超声波串口编号
front_id = 0
back_id = 1
imu_com = 'COM12'

# 主板通信串口
se = serial.Serial('/dev/ttyTHS1', 115200, timeout=0.1)

# 前后测距阈值
laser_TH_F = 200
laser_TH_B = 150
laser_TH_LR = 80

# 机身尺寸
J1_width = 600
column_move = 300

# 左右单目阈值
camera_TH_L = 100
camera_TH_R = 100

cmd_0_head = 'aa 01'
cmd_1_stop = '00'
cmd_1_moveFront = '01'
cmd_1_moveBack = '02'
cmd_1_rotateLeft = '03'
cmd_1_rotateRight = '04'
cmd_2_speed0 = '00'
cmd_2_slow = '0a'
cmd_2_max = '64'
cmd_3_stop = '00'
cmd_3_front = '01'
cmd_3_back = '02'
cmd_4_stop = '00'
cmd_4_start = '01'


# 调用相机获取图片进行测距
def distance_get(q_s, q_f, q_l, q_r, q_i, lock_ser, cap_id, file_address):
    # 根据相机编号分配模型参数
    global para_lines
    global minCan, maxCan

    if int(cap_id) == front_id:
        model_F = float(para_lines[0].strip('\n'))
        model_W = float(para_lines[1].strip('\n'))
        model_a = float(para_lines[2].strip('\n'))
        model_b = float(para_lines[3].strip('\n'))
        principal_x = int(para_lines[4].strip('\n'))
        principal_y = int(para_lines[5].strip('\n'))
    else:
        model_F = float(para_lines[6].strip('\n'))
        model_W = float(para_lines[7].strip('\n'))
        model_a = float(para_lines[8].strip('\n'))
        model_b = float(para_lines[9].strip('\n'))
        principal_x = int(para_lines[10].strip('\n'))
        principal_y = int(para_lines[11].strip('\n'))

    # 尝试连接相机
    cap = cv2.VideoCapture(cap_id)
    cap.set(6, 1196444237)
    cap.set(3, 1920)
    cap.set(4, 1080)
    cap.set(5, 30)
    cap.set(11, 80)
    cap.set(12, 80)
    if cap.isOpened():
        print('Camera ', cap_id, 'start')
    else:
        cap = cv2.VideoCapture(cap_id)
        cap.set(6, 1196444237)
        cap.set(3, 1920)
        cap.set(4, 1080)
        cap.set(5, 30)
        print('Camera ', cap_id, 'start')

    # 循环处理图像
    loop_num = 0
    angle_set = 0.0

    while cap.isOpened():
        start_time_total = time.time()
        str_Time = datetime.datetime.now().strftime('%H%M%S-%f')
        loop_num += 1
        ret_mess = ''  # 数据信息
        time_mess = ''  # 时间信息

        # 获取图像及参数
        start_time = time.time()
        ret, frame = cap.read()
        if not ret:
            cap = cv2.VideoCapture(cap_id)
            ret, frame = cap.read()
        if cap_id == back_id:
            rgb_frame = cv2.rotate(frame, cv2.ROTATE_180)
        else:
            rgb_frame = frame.copy()
        img_height = int(rgb_frame.shape[0])
        img_width = int(rgb_frame.shape[1])
        mid_height = int(img_height / 2)
        mid_width = int(img_width / 2)
        rgb_rot = rgb_frame.copy()
        rgb_show = rgb_frame.copy()
        # 获取偏航角
        if not q_i.empty():
            jy_list = q_i.get()
            angle_set = jy_list[0]
        end_time = time.time()
        time_mess += 'Cap:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # Canny提取边界，保留下半部分
        start_time = time.time()
        rgb_half = rgb_frame.copy()
        rgb_half[0:principal_y - 50, :] = (0, 0, 0)
        gra_edge = cv2.Canny(rgb_half, minCan, maxCan)
        end_time = time.time()
        time_mess += 'Can:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 获取水平和垂直线
        start_time = time.time()
        gra_edge_rot = gra_edge.copy()
        gra_edge_rot[0:principal_y, :] = 0
        lines = cv2.HoughLinesP(gra_edge_rot, rho=1.0, theta=np.pi / 180, threshold=100, minLineLength=100,
                                maxLineGap=5)
        end_time = time.time()
        time_mess += 'Hou:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 旋转校正、拉平、融合
        start_time = time.time()
        num_line = 0
        num_ver = 0
        num_hor = 0
        num_front = 0
        num_left = 0
        num_right = 0
        data_ver = np.zeros((0, 3), dtype=float)  # 垂直线数据，0是最接近中轴的w（x）值，1是最接近相机的h（y）值，2是最远离相机的h（y）值
        data_hor = np.zeros((0, 3), dtype=float)  # 水平线数据，0是最接近相机的h（y）值，1是最左侧（最小）的w（x）值，2是最右侧（最大）的w（x）值
        dis_f = [0]
        dis_l = [0]
        dis_r = [0]
        try:
            if str(type(lines)) != "<class 'NoneType'>":
                if len(lines) > 0:
                    for line in lines:
                        for x1_p, y1_p, x2_p, y2_p in line:
                            if CVFunc.getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 50.0:
                                # 根据偏航角进行旋转
                                h1 = CVFunc.calc_horizontal(y1_p, model_F, model_W, model_a, model_b)
                                h2 = CVFunc.calc_horizontal(y2_p, model_F, model_W, model_a, model_b)
                                w1 = CVFunc.calc_vertical((x1_p - principal_x), y1_p, model_F, model_W, model_a,
                                                          model_b)
                                w2 = CVFunc.calc_vertical((x2_p - principal_x), y2_p, model_F, model_W, model_a,
                                                          model_b)
                                # 在2米范围内
                                if h1 < 2000 and h2 < 2000 and w1 < 2000 and w2 < 2000:
                                    num_line += 1
                                    x1_imu, y1_imu = CVFunc.points_rotate(angle_set, w1, h1)
                                    x2_imu, y2_imu = CVFunc.points_rotate(angle_set, w2, h2)

                                    # 调整后处理不为水平线或者垂直线的线段，进行拉平拉直，小于±15认为是水平线，大于±75认为是垂直线
                                    if x1_imu != x2_imu and y1_imu != y2_imu:
                                        temp_tan = np.arctan((y1_imu - y2_imu) / (x1_imu - x2_imu)) * 57.29577
                                        if abs(temp_tan) <= 15:  # 判断是水平线,按照最接近相机来拉平
                                            if abs(x1_imu) > abs(x2_imu):
                                                y1_imu = y2_imu
                                            else:
                                                y2_imu = y1_imu
                                        elif abs(temp_tan) >= 75:  # 判断是垂直线,按照最接近中轴来拉直
                                            if abs(x1_imu) > abs(x2_imu):
                                                x1_imu = x2_imu
                                            else:
                                                x2_imu = x1_imu

                                    # 如果是同一条线，进行融合
                                    if y1_imu == y2_imu:  # 水平线
                                        num_hor += 1
                                        # 如果距离中轴的差值不超过50，则认为是同一条，按照最接近中轴的值进行融合。否则新增为新一条。
                                        temp_left = min(x1_imu, x2_imu)
                                        temp_right = max(x1_imu, x2_imu)
                                        if len(data_hor) == 0:  # 第1条水平线，直接保存
                                            data_hor = np.append(data_hor, [[y1_imu, temp_left, temp_right]], axis=0)
                                        else:
                                            new_hor = True
                                            for values_hor in data_hor:
                                                if abs(values_hor[0] - y1_imu) < 50:
                                                    if abs((values_hor[2] + values_hor[1]) / 2) > abs(
                                                            (temp_right + temp_left) / 2):
                                                        values_hor[0] = y1_imu
                                                    values_hor[1] = min(values_hor[1], temp_left)
                                                    values_hor[2] = max(values_hor[2], temp_right)
                                                    new_hor = False
                                                    break
                                            if new_hor:
                                                data_hor = np.append(data_hor, [[y1_imu, temp_left, temp_right]],
                                                                     axis=0)
                                    elif x1_imu == x2_imu:  # 垂直线
                                        num_ver += 1
                                        # 如果距离中轴的差值不超过50，则认为是同一条，按最接近中轴进行融合。否则新增为新一条。
                                        temp_button = min(y1_imu, y2_imu)
                                        temp_top = max(y1_imu, y2_imu)
                                        if len(data_ver) == 0:
                                            data_ver = np.append(data_ver, [[x1_imu, temp_button, temp_top]], axis=0)
                                        else:
                                            new_ver = True
                                            for values_ver in data_ver:
                                                if abs(values_ver[0] - x1_imu) < 50:
                                                    if abs(values_ver[0]) > abs(x1_imu):
                                                        values_ver[0] = x1_imu
                                                    temp_v = min(values_ver[1], temp_button)
                                                    values_ver[1] = temp_v
                                                    temp_v = max(values_ver[2], temp_top)
                                                    values_ver[2] = temp_v
                                                    new_ver = False
                                                    break
                                            if new_ver:
                                                data_ver = np.append(data_ver, [[x1_imu, temp_button, temp_top]],
                                                                     axis=0)
                                    else:
                                        pass
                    end_time = time.time()
                    time_mess += 'Rot:' + str(round((end_time - start_time) * 1000, 4)) + ';'

                    # 反推图像上的直线位置，找出最近的水平线和左右垂直线
                    start_time = time.time()
                    if len(data_hor) > 0:
                        for values_hor in data_hor:
                            w1_b, h1_b = CVFunc.points_rotate(-angle_set, values_hor[1], values_hor[0])
                            w2_b, h2_b = CVFunc.points_rotate(-angle_set, values_hor[2], values_hor[0])
                            y1_b = CVFunc.calc_h2y(h1_b, model_F, model_W, model_a, model_b)
                            y2_b = CVFunc.calc_h2y(h2_b, model_F, model_W, model_a, model_b)
                            x1_b = CVFunc.calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
                            x2_b = CVFunc.calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
                            x_mid = int((x1_b + x2_b) / 2)
                            y_mid = int((y1_b + y2_b) / 2)
                            cv2.line(rgb_rot, (x1_b, y1_b), (x2_b, y2_b), (255, 0, 0), 1)
                            cv2.putText(rgb_rot, str(round(values_hor[0], 0)), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.8,
                                        (255, 0, 0), 1)
                            cv2.line(rgb_show, (x1_b, y1_b), (x2_b, y2_b), (255, 0, 0), 6)
                            num_front += 1
                            if dis_f[0] == 0:
                                dis_f[0] = int(values_hor[0])
                            else:
                                dis_f.append(int(values_hor[0]))

                    if len(data_ver) > 0:
                        for values_ver in data_ver:
                            w1_b, h1_b = CVFunc.points_rotate(-angle_set, values_ver[0], values_ver[1])
                            w2_b, h2_b = CVFunc.points_rotate(-angle_set, values_ver[0], values_ver[2])
                            y1_b = CVFunc.calc_h2y(h1_b, model_F, model_W, model_a, model_b)
                            y2_b = CVFunc.calc_h2y(h2_b, model_F, model_W, model_a, model_b)
                            x1_b = CVFunc.calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
                            x2_b = CVFunc.calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
                            x_mid = int((x1_b + x2_b) / 2)
                            y_mid = int((y1_b + y2_b) / 2)
                            cv2.line(rgb_rot, (x1_b, y1_b), (x2_b, y2_b), (0, 255, 255), 1)
                            cv2.putText(rgb_rot, str(round(values_ver[0], 0)), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.8,
                                        (0, 255, 255), 1)
                            cv2.line(rgb_show, (x1_b, y1_b), (x2_b, y2_b), (0, 255, 255), 6)
                            if values_ver[0] < 0:
                                num_left += 1
                                if dis_l[0] == 0:
                                    dis_l[0] = int(abs(values_ver[0]))
                                else:
                                    dis_l.append(int(abs(values_ver[0])))
                            else:
                                num_right += 1
                                if dis_r[0] == 0:
                                    dis_r[0] = int(values_ver[0])
                                else:
                                    dis_r.append(int(values_ver[0]))
                    end_time = time.time()
                    time_mess += 'Dra:' + str(round((end_time - start_time) * 1000, 4)) + ';'

            # if cap_id == front_id:
            #     print('F', str(dis_f[0]), str(dis_l[0]), str(dis_r[0]))
            # else:
            #     print('B', str(dis_f[0]), str(dis_l[0]), str(dis_r[0]))
        except Exception as e:
            print(e)
            print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
            print(f"error line:{e.__traceback__.tb_lineno}")

        # 画相机中心十字
        start_time = time.time()
        cv2.line(rgb_rot, (principal_x, 0), (principal_x, img_height), (255, 255, 0), 1)
        cv2.line(rgb_rot, (0, principal_y), (img_width, principal_y), (255, 255, 0), 1)
        cv2.circle(rgb_rot, (principal_x, principal_y), 5, (255, 255, 0), 3)
        # 显示及保存图片
        # rgb_show = cv2.resize(rgb_rot, (960, 540))
        # cv2.imshow('Cap' + str(cap_id), rgb_show)
        cv2.imwrite(file_address + 'C' + str(cap_id) + '-' + str_Time + '.jpg', rgb_frame)
        cv2.imwrite(file_address + 'D' + str(cap_id) + '-' + str_Time + '.jpg', rgb_rot)

        # 保存txt，传输数据
        q_s.put(rgb_show)
        if q_s.qsize() > 1:
            q_s.get()
        ret_mess += 'Tim:' + str_Time + ';Yaw:' + str(angle_set) + ';all:' + str(num_line)
        ret_mess += ';frt:' + str(num_front) + ';lft:' + str(num_left) + ';rgt:' + str(num_right)
        if dis_f[0] != 0:
            dis_f.sort()
            q_f.put(dis_f)
            if q_f.qsize() > 1:
                q_f.get()
            for i in range(0, len(dis_f), 1):
                ret_mess += ';f_' + str(i) + ':' + str(dis_f[i])
        if dis_l[0] != 0:
            dis_l.sort()
            q_l.put(dis_l)
            if q_l.qsize() > 1:
                q_l.get()
            for i in range(0, len(dis_l), 1):
                ret_mess += ';l_' + str(i) + ':' + str(dis_l[i])
        if dis_r[0] != 0:
            dis_r.sort()
            q_r.put(dis_r)
            if q_r.qsize() > 1:
                q_r.get()
            for i in range(0, len(dis_r), 1):
                ret_mess += ';r_' + str(i) + ':' + str(dis_r[i])
        ret_mess += ';\n'
        file_rec = open(file_address + str(cap_id) + '.txt', 'a')
        if len(ret_mess) > 0:
            file_rec.write('Tpy:Date;' + ret_mess)

        end_time = time.time()
        time_mess += 'Shw:' + str(round((end_time - start_time) * 1000, 4)) + ';'
        end_time_total = time.time()
        time_mess += 'All:' + str(round((end_time_total - start_time_total) * 1000, 4)) + ';\n'

        if len(time_mess) > 0:
            file_rec.write('Tpy:Timer;' + time_mess)
        file_rec.close()


# 读取IMU数据
def imu_get(q_i0, q_i1, q_im, q_ia, lock_ser, file_address):
    cv2.waitKey(500)
    se_i = serial.Serial(imu_com, 9600, timeout=0.05)
    # 释放串口积攒的数据
    se_i.flushInput()
    se_i.flushOutput()
    print('IMU start')
    while True:
        try:
            # 串口j采集JY61的IMU数据
            # lock_ser.acquire()
            imu_rec = se_i.read(33)
            # lock_ser.release()
            if imu_rec:
                str_imu = binascii.b2a_hex(imu_rec).decode()
                file_rec = open(file_address + 'JY61.txt', 'a')
                str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
                if len(str_imu) == 66 and str_imu[0:4] == '5551':
                    jy_list = JY61.DueData(imu_rec)
                    if str(type(jy_list)) != "<class 'NoneType'>":
                        sav_mess = ("normal;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;\n" % jy_list)
                        send_list = [round(jy_list[6], 2), round(jy_list[7], 2), round(jy_list[8], 2)]
                        # print(str(round(jy_list[6], 2)))
                        q_i0.put(send_list)
                        q_i0.get() if q_i0.qsize() > 1 else time.sleep(0.005)
                        q_i1.put(send_list)
                        q_i1.get() if q_i1.qsize() > 1 else time.sleep(0.005)
                        q_im.put(send_list)
                        q_im.get() if q_im.qsize() > 1 else time.sleep(0.005)
                        q_ia.put(send_list)
                        q_ia.get() if q_ia.qsize() > 1 else time.sleep(0.005)
                    else:
                        sav_mess = ('NoneType;' + str_imu + ';\n')
                        se_i.flushOutput()
                else:
                    sav_mess = ('error;' + str_imu + ';\n')
                    se_i.flushOutput()
                file_rec.write(str_time + ';' + sav_mess)
                file_rec.close()
        except Exception as e:
            print(e)
            print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
            print(f"error line:{e.__traceback__.tb_lineno}")


# 根据图像调整Canny参数
def camera_init():
    global para_lines
    global minCan, maxCan

    model_F = float(para_lines[0].strip('\n'))
    model_W = float(para_lines[1].strip('\n'))
    model_a = float(para_lines[2].strip('\n'))
    model_b = float(para_lines[3].strip('\n'))
    principal_x = int(para_lines[4].strip('\n'))
    principal_y = int(para_lines[5].strip('\n'))

    # 尝试连接相机
    cap = cv2.VideoCapture(front_id)
    cap.set(6, 1196444237)
    cap.set(3, 1920)
    cap.set(4, 1080)
    cap.set(5, 30)
    cap.set(11, 80)
    cap.set(12, 80)
    if not cap.isOpened():
        cap = cv2.VideoCapture(front_id)
        cap.set(6, 1196444237)
        cap.set(3, 1920)
        cap.set(4, 1080)
        cap.set(5, 30)
        if not cap.isOpened():
            print('打开摄像头失败')
            return 40, 100

    loop_num = 0
    while cap.isOpened() and loop_num < 15:
        loop_num += 1
        ret, frame = cap.read()
        if loop_num > 20:
            break

    loop_num = 0
    while cap.isOpened() and loop_num < 5:
        ret, frame = cap.read()
        rgb_frame = frame.copy()

        # 设置各项参数
        angle_set = 0.0
        width_threshold = 1050
        img_height = int(rgb_frame.shape[0])
        img_width = int(rgb_frame.shape[1])
        mid_height = int(img_height / 2)
        mid_width = int(img_width / 2)
        rgb_rot = rgb_frame.copy()
        str_Time = datetime.datetime.now().strftime('%H%M%S')
        x = 0
        xls = xlwt.Workbook()
        sheet = xls.add_sheet('sheet1', cell_overwrite_ok=True)

        temp_ver = 0
        temp_hor = 0
        temp_minCan = 0
        temp_maxCan = 0

        for min_value in range(20, 220, 20):
            for max_value in range(50, 450, 50):
                find_width = False
                rec_width = 0.0
                rec_left = 0.0
                rec_right = 0.0

                # Canny提取边界，保留下半部分
                start_time = time.time()
                rgb_half = rgb_frame.copy()
                rgb_half[0:principal_y - 50, :] = (0, 0, 0)
                gra_edge = cv2.Canny(rgb_half, min_value, max_value)
                gra_edge_rot = gra_edge.copy()
                gra_edge_rot[0:principal_y, :] = 0
                end_time = time.time()
                time_Can = round((end_time - start_time) * 1000, 4)

                # 获取水平和垂直线
                start_time = time.time()
                lines = cv2.HoughLinesP(gra_edge_rot, rho=1.0, theta=np.pi / 180, threshold=100, minLineLength=100,
                                        maxLineGap=5)
                end_time = time.time()
                time_Hough = round((end_time - start_time) * 1000, 4)

                # 旋转校正、拉平、融合
                num_line = 0
                num_ver = 0
                num_L = 0
                num_R = 0
                num_hor = 0
                data_ver = np.zeros((0, 3), dtype=float)  # 垂直线数据，0是最接近中轴的w（x）值，1是最接近相机的h（y）值，2是最远离相机的h（y）值
                data_L = np.zeros((0, 5), dtype=int)  # 左垂线数据，0是中轴距离，1-4是x1,y1,x2,y2
                data_R = np.zeros((0, 5), dtype=int)  # 右垂线数据，0是中轴距离，1-4是x1,y1,x2,y2
                data_hor = np.zeros((0, 3), dtype=float)  # 水平线数据，0是最接近相机的h（y）值，1是最左侧（最小）的w（x）值，2是最右侧（最大）的w（x）值
                gra_lines = np.zeros((gra_edge.shape[0], gra_edge.shape[1]), np.uint8)  # 创建个全0的黑背景
                try:
                    if len(lines) > 0:
                        for line in lines:
                            for x1_p, y1_p, x2_p, y2_p in line:
                                if CVFunc.getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 50.0:
                                    # 根据偏航角进行旋转
                                    h1 = CVFunc.calc_horizontal(y1_p, model_F, model_W, model_a, model_b)
                                    h2 = CVFunc.calc_horizontal(y2_p, model_F, model_W, model_a, model_b)
                                    w1 = CVFunc.calc_vertical((x1_p - principal_x), y1_p, model_F, model_W, model_a,
                                                              model_b)
                                    w2 = CVFunc.calc_vertical((x2_p - principal_x), y2_p, model_F, model_W, model_a,
                                                              model_b)
                                    # 在2米范围内
                                    if h1 < 2000 and h2 < 2000 and w1 < 2000 and w2 < 2000:
                                        num_line += 1
                                        x1_imu, y1_imu = CVFunc.points_rotate(angle_set, w1, h1)
                                        x2_imu, y2_imu = CVFunc.points_rotate(angle_set, w2, h2)

                                        # 调整后处理不为水平线或者垂直线的线段，进行拉平拉直，小于±15认为是水平线，大于±75认为是垂直线
                                        if x1_imu != x2_imu and y1_imu != y2_imu:
                                            temp_tan = np.arctan((y1_imu - y2_imu) / (x1_imu - x2_imu)) * 57.29577
                                            if abs(temp_tan) <= 15:  # 判断是水平线,按照最接近相机来拉平
                                                cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                                                if abs(x1_imu) > abs(x2_imu):
                                                    y1_imu = y2_imu
                                                else:
                                                    y2_imu = y1_imu
                                            elif abs(temp_tan) >= 75:  # 判断是垂直线,按照最接近中轴来拉直
                                                cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                                                if abs(x1_imu) > abs(x2_imu):
                                                    x1_imu = x2_imu
                                                else:
                                                    x2_imu = x1_imu
                                            else:
                                                cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)

                                        # 如果是同一条线，进行融合
                                        if y1_imu == y2_imu:  # 水平线
                                            num_hor += 1
                                            # 如果距离中轴的差值不超过50，则认为是同一条，按照最接近中轴的值进行融合。否则新增为新一条。
                                            temp_left = min(x1_imu, x2_imu)
                                            temp_right = max(x1_imu, x2_imu)
                                            if len(data_hor) == 0:  # 第1条水平线，直接保存
                                                data_hor = np.append(data_hor, [[y1_imu, temp_left, temp_right]],
                                                                     axis=0)
                                            else:
                                                new_hor = True
                                                for values_hor in data_hor:
                                                    if abs(values_hor[0] - y1_imu) < 50:
                                                        if abs((values_hor[2] + values_hor[1]) / 2) > abs(
                                                                (temp_right + temp_left) / 2):
                                                            values_hor[0] = y1_imu
                                                        values_hor[1] = min(values_hor[1], temp_left)
                                                        values_hor[2] = max(values_hor[2], temp_right)
                                                        new_hor = False
                                                        break
                                                if new_hor:
                                                    data_hor = np.append(data_hor, [[y1_imu, temp_left, temp_right]],
                                                                         axis=0)
                                        elif x1_imu == x2_imu:  # 垂直线
                                            num_ver += 1
                                            # 如果距离中轴的差值不超过50，则认为是同一条，按最接近中轴进行融合。否则新增为新一条。
                                            temp_button = min(y1_imu, y2_imu)
                                            temp_top = max(y1_imu, y2_imu)
                                            if len(data_ver) == 0:
                                                data_ver = np.append(data_ver, [[x1_imu, temp_button, temp_top]],
                                                                     axis=0)
                                            else:
                                                new_ver = True
                                                for values_ver in data_ver:
                                                    if abs(values_ver[0] - x1_imu) < 50:
                                                        if abs(values_ver[0]) > abs(x1_imu):
                                                            values_ver[0] = x1_imu
                                                        temp_v = min(values_ver[1], temp_button)
                                                        values_ver[1] = temp_v
                                                        temp_v = max(values_ver[2], temp_top)
                                                        values_ver[2] = temp_v
                                                        new_ver = False
                                                        break
                                                if new_ver:
                                                    data_ver = np.append(data_ver, [[x1_imu, temp_button, temp_top]],
                                                                         axis=0)

                    # 反推图像上的直线位置，找出最近的水平线和左右垂直线
                    start_time = time.time()

                    if len(data_hor) > 0:
                        for values_hor in data_hor:
                            w1_b, h1_b = CVFunc.points_rotate(-angle_set, values_hor[1], values_hor[0])
                            w2_b, h2_b = CVFunc.points_rotate(-angle_set, values_hor[2], values_hor[0])
                            y1_b = CVFunc.calc_h2y(h1_b, model_F, model_W, model_a, model_b)
                            y2_b = CVFunc.calc_h2y(h2_b, model_F, model_W, model_a, model_b)
                            x1_b = CVFunc.calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
                            x2_b = CVFunc.calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
                            x_mid = int((x1_b + x2_b) / 2)
                            y_mid = int((y1_b + y2_b) / 2)
                            cv2.line(rgb_rot, (x1_b, y1_b), (x2_b, y2_b), (255, 0, 0), 1)
                            cv2.putText(rgb_rot, str(round(values_hor[0], 0)), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.6, (255, 0, 0), 1)
                    if len(data_ver) > 0:
                        for values_ver in data_ver:
                            w1_b, h1_b = CVFunc.points_rotate(-angle_set, values_ver[0], values_ver[1])
                            w2_b, h2_b = CVFunc.points_rotate(-angle_set, values_ver[0], values_ver[2])
                            y1_b = CVFunc.calc_h2y(h1_b, model_F, model_W, model_a, model_b)
                            y2_b = CVFunc.calc_h2y(h2_b, model_F, model_W, model_a, model_b)
                            x1_b = CVFunc.calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
                            x2_b = CVFunc.calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
                            x_mid = int((x1_b + x2_b) / 2)
                            y_mid = int((y1_b + y2_b) / 2)
                            cv2.line(rgb_rot, (x1_b, y1_b), (x2_b, y2_b), (0, 0, 255), 1)
                            cv2.putText(rgb_rot, str(round(values_ver[0], 0)), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.6, (0, 0, 255), 1)
                            if values_ver[0] < 0:
                                num_L += 1
                                data_L = np.append(data_L, [[int(abs(values_ver[0])), x1_b, y1_b, x2_b, y2_b]], axis=0)
                                if num_R > 0:
                                    for data_R_value in data_R:
                                        width_value = int(abs(values_ver[0])) + int(data_R_value[0])
                                        if abs(width_value - width_threshold) <= 200:
                                            find_width = True
                                            rec_width = width_value
                                            rec_left = str(abs(values_ver[0]))
                                            rec_right = str(data_R_value[0])
                            else:
                                num_R += 1
                                data_R = np.append(data_R, [[int(abs(values_ver[0])), x1_b, y1_b, x2_b, y2_b]], axis=0)
                                if num_L > 0:
                                    for data_L_value in data_L:
                                        width_value = int(abs(values_ver[0])) + int(data_L_value[0])
                                        if abs(width_value - width_threshold) <= 200:
                                            find_width = True
                                            rec_width = width_value
                                            rec_left = str(data_L_value[0])
                                            rec_right = str(values_ver[0])

                except Exception as e:
                    print(e)
                    print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
                    print(f"error line:{e.__traceback__.tb_lineno}")

                # 画相机中心十字，读取超声数据，反推图像位置，画出水平线
                start_time = time.time()
                cv2.line(rgb_rot, (principal_x, 0), (principal_x, img_height), (255, 255, 0), 1)
                cv2.line(rgb_rot, (0, principal_y), (img_width, principal_y), (255, 255, 0), 1)
                cv2.circle(rgb_rot, (principal_x, principal_y), 5, (255, 255, 0), 3)

                cv2.imwrite('./TestData/Test/C-' + str_Time + '.jpg', rgb_frame)
                cv2.imwrite('./TestData/Test/D-' + str_Time + '.jpg', rgb_rot)

                # 显示及保存图片
                if find_width:
                    if num_hor > 0:
                        # 比较左右垂线和前向水平线数量，选最多的作为参数
                        if temp_ver == 0:
                            temp_ver = num_L + num_R
                            temp_hor = num_hor
                            temp_minCan = min_value
                            temp_maxCan = max_value
                        else:
                            if temp_ver < (num_L + num_R):
                                temp_ver = num_L + num_R
                                temp_hor = num_hor
                                temp_minCan = min_value
                                temp_maxCan = max_value
                            elif temp_ver == (num_L + num_R):
                                if temp_hor < num_hor:
                                    temp_ver = num_L + num_R
                                    temp_hor = num_hor
                                    temp_minCan = min_value
                                    temp_maxCan = max_value

                        sheet.write(x, 0, min_value)
                        sheet.write(x, 1, max_value)
                        sheet.write(x, 2, time_Can)
                        sheet.write(x, 3, time_Hough)
                        sheet.write(x, 4, num_line)
                        sheet.write(x, 5, num_hor)
                        sheet.write(x, 6, num_L)
                        sheet.write(x, 7, num_R)
                        sheet.write(x, 8, rec_width)
                        sheet.write(x, 9, rec_left)
                        sheet.write(x, 10, rec_right)
                        x += 1
        # 如果检测到玻璃两边，保存校准数据，返回左、右识别线段最多；前向识别线段最多的参数
        if x > 0:
            xls.save('./TestData/Test/Cali-' + str_Time + '.xls')
            minCan = temp_minCan
            maxCan = temp_maxCan
            print('Get New', minCan, maxCan)
            return temp_minCan, temp_maxCan
        else:
            print('Not Found')
            return 40, 100
    print('Run Out')
    return minCan, maxCan


# 融合前后单目、超声和IMU，快速更新四向和偏航角
def multi_calc(q_s0, q_s1, q_f, q_b, q_lf, q_rf, q_lb, q_rb, q_i, q_d, q_c, q_s, file_address):
    cv2.waitKey(2000)
    print('calc start')

    imu_yaw = 0.0

    cam_front = 0
    cam_back = 0
    cam_left_f = 0
    cam_left_b = 0
    cam_right_f = 0
    cam_right_b = 0
    cam_left = 0
    cam_right = 0

    dis_f = [0]
    dis_lf = [0]
    dis_rf = [0]
    dis_b = [0]
    dis_lb = [0]
    dis_rb = [0]

    ac_sensors = [0, 0, 0, 0, 0, 0]
    ac_ctl = [0]

    start_time = time.time()
    end_time = time.time()
    time_mess = round((end_time - start_time) * 1000, 0)

    # show_width = 640
    # show_height = 360
    show_width = 480
    show_height = 270
    half_width = int(show_width / 2)
    half_height = int(show_height / 2)

    rgb_c0 = np.zeros((show_height, show_width, 3), np.uint8)
    rgb_c1 = np.zeros((show_height, show_width, 3), np.uint8)

    while True:
        str_Time = datetime.datetime.now().strftime('%H:%M:%S.%f')
        # 收到数据标志位
        is_imu = False
        is_c0 = False
        is_c1 = False
        is_f = False
        is_lf = False
        is_rf = False
        is_b = False
        is_lb = False
        is_rb = False
        is_s = False
        is_ctl = False

        # 主板的传感器，前后测距和4向防跌落
        if not q_s.empty():
            is_s = True
            ac_sensors = q_s.get()

        # 发往主板的控制命令
        if not q_c.empty():
            is_ctl = True
            ac_ctl = q_c.get()

        # IMU
        if not q_i.empty():
            is_imu = True
            imu_list = q_i.get()
            imu_yaw = imu_list[0]

        # 前向收到数据
        if not q_s0.empty():
            # 前向图像，调整尺寸
            is_c0 = True
            rgb_tmp = q_s0.get()
            rgb_c0 = cv2.resize(rgb_tmp, (show_width, show_height))

            # 前向水平线
            if not q_f.empty():
                is_f = True
                dis_f = q_f.get()
            else:
                dis_f = [0]
            cam_front = dis_f[0]

            # 前向左垂线
            if not q_lf.empty():
                is_lf = True
                dis_lf = q_lf.get()
            else:
                dis_lf = [0]
            # cam_left_f = dis_lf[0]

            # 前向右垂线
            if not q_rf.empty():
                is_rf = True
                dis_rf = q_rf.get()
            else:
                dis_rf = [0]
            # cam_right_f = dis_rf[0]

        # 后向收到数据
        if not q_s1.empty():
            # 后向图像，调整尺寸和方向
            is_c1 = True
            rgb_t1 = q_s1.get()
            rgb_t2 = cv2.resize(rgb_t1, (show_width, show_height))
            rgb_c1 = cv2.rotate(rgb_t2, cv2.ROTATE_180)

            # 后向水平线
            if not q_b.empty():
                is_b = True
                dis_b = q_b.get()
            else:
                dis_b = [0]
            cam_back = dis_b[0]

            # 后向左垂线
            if not q_lb.empty():
                is_lb = True
                dis_lb = q_lb.get()
            else:
                dis_lb = [0]
            # cam_left_b = dis_lb[0]

            # 后向右垂线
            if not q_rb.empty():
                is_rb = True
                dis_rb = q_rb.get()
            else:
                dis_rb = [0]
            # cam_right_b = dis_rb[0]

        # 如果前后单目的任意一个完成处理，则更新显示
        if is_c0 or is_c1 or is_s:
            end_time = time.time()
            time_mess = round((end_time - start_time) * 1000, 0)

            # 上下单目的左侧融合
            if dis_lf[0] != 0 and dis_lb[0] != 0:
                is_fb_left = False
                for i in range(0, len(dis_lf), 1):
                    for j in range(0, len(dis_lb), 1):
                        if abs(dis_lf[i] - dis_lb[j]) < 20:
                            cam_left = int((dis_lf[i] + dis_lb[j]) / 2)
                            cam_left_f = dis_lf[i]
                            cam_left_b = dis_lb[j]
                            is_fb_left = True
                            break
                    if is_fb_left:
                        break
            elif dis_lf[0] != 0 and cam_left != 0:
                for i in range(0, len(dis_lf), 1):
                    if abs(dis_lf[i] - cam_left) < 20:
                        cam_left = int((cam_left * 2 + dis_lf[i]) / 3)
                        cam_left_f = dis_lf[i]
                        break
            elif dis_lb[0] != 0 and cam_left != 0:
                for i in range(0, len(dis_lb), 1):
                    if abs(dis_lb[i] - cam_left) < 20:
                        cam_left = int((cam_left * 2 + dis_lb[i]) / 3)
                        cam_left_b = dis_lb[i]
                        break

            # 上下单目的右侧融合
            if dis_rf[0] != 0 and dis_rb[0] != 0:
                is_fb_right = False
                for i in range(0, len(dis_rf), 1):
                    for j in range(0, len(dis_rb), 1):
                        if abs(dis_rf[i] - dis_rb[j]) < 20:
                            cam_right = int((dis_rf[i] + dis_rb[j]) / 2)
                            cam_right_f = dis_rf[i]
                            cam_right_b = dis_rb[j]
                            is_fb_right = True
                            break
                    if is_fb_right:
                        break
            elif dis_rf[0] != 0 and cam_right != 0:
                for i in range(0, len(dis_rf), 1):
                    if abs(dis_rf[i] - cam_right) < 20:
                        cam_right = int((cam_right + dis_rf[i]) / 2)
                        cam_right_f = dis_rf[i]
                        break
            elif dis_rb[0] != 0 and cam_right != 0:
                for i in range(0, len(dis_rb), 1):
                    if abs(dis_rb[i] - cam_right) < 20:
                        cam_right = int((cam_right + dis_rb[i]) / 2)
                        cam_right_b = dis_rb[i]
                        break

            send_list = [cam_front, cam_back, cam_left, cam_right]
            q_d.put(send_list)
            q_d.get() if q_d.qsize() > 1 else time.sleep(0.005)

            # 左上和左下展示图像
            rgb_show_0 = rgb_c0
            rgb_show_1 = rgb_c1

            # 右上的距离展示区
            rgb_show_line = np.zeros((show_height, show_width, 3), np.uint8)
            if dis_f[0] != 0:
                for i in range(0, len(dis_f), 1):
                    temp_y = int(half_height - (half_height * dis_f[i] / 2000))
                    cv2.line(rgb_show_line, (0, temp_y), (show_width, temp_y), (255, 0, 0), 1)
            if dis_b[0] != 0:
                for i in range(0, len(dis_b), 1):
                    temp_y = int(half_height + (half_height * dis_b[i] / 2000))
                    cv2.line(rgb_show_line, (0, temp_y), (show_width, temp_y), (255, 0, 0), 1)
            if dis_lf[0] != 0:
                for i in range(0, len(dis_lf), 1):
                    temp_x = int(half_width - (half_width * dis_lf[i] / 2000))
                    cv2.line(rgb_show_line, (temp_x, 0), (temp_x, half_height), (255, 0, 0), 1)
            if dis_lb[0] != 0:
                for i in range(0, len(dis_lb), 1):
                    temp_x = int(half_width - (half_width * dis_lb[i] / 2000))
                    cv2.line(rgb_show_line, (temp_x, half_height), (temp_x, show_height), (255, 0, 0), 1)
            if dis_rf[0] != 0:
                for i in range(0, len(dis_rf), 1):
                    temp_x = int(half_width + (half_width * dis_rf[i] / 2000))
                    cv2.line(rgb_show_line, (temp_x, 0), (temp_x, half_height), (255, 0, 0), 1)
            if dis_rb[0] != 0:
                for i in range(0, len(dis_rb), 1):
                    temp_x = int(half_width + (half_width * dis_rb[i] / 2000))
                    cv2.line(rgb_show_line, (temp_x, half_height), (temp_x, show_height), (255, 0, 0), 1)
            if cam_left != 0:
                temp_x = int(half_width - (half_width * cam_left / 2000))
                cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (0, 0, 255), 1)
            if cam_right != 0:
                temp_x = int(half_width + (half_width * cam_right / 2000))
                cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (0, 0, 255), 1)
            if cam_front != 0:
                temp_y = int(half_height - (half_height * cam_front / 2000))
                cv2.line(rgb_show_line, (0, temp_y), (show_width, temp_y), (0, 0, 255), 1)
            if cam_back != 0:
                temp_y = int(half_height + (half_height * cam_back / 2000))
                cv2.line(rgb_show_line, (0, temp_y), (show_width, temp_y), (0, 0, 255), 1)
            if cam_left_f != 0:
                temp_x = int(half_width - (half_width * cam_left_f / 2000))
                cv2.line(rgb_show_line, (temp_x, 0), (temp_x, half_height), (0, 0, 255), 1)
            if cam_left_b != 0:
                temp_x = int(half_width - (half_width * cam_left_b / 2000))
                cv2.line(rgb_show_line, (temp_x, half_height), (temp_x, show_height), (0, 0, 255), 1)
            if cam_right_f != 0:
                temp_x = int(half_width + (half_width * cam_right_f / 2000))
                cv2.line(rgb_show_line, (temp_x, 0), (temp_x, half_height), (0, 0, 255), 1)
            if cam_right_b != 0:
                temp_x = int(half_width + (half_width * cam_right_b / 2000))
                cv2.line(rgb_show_line, (temp_x, half_height), (temp_x, show_height), (0, 0, 255), 1)

            yaw_sin = math.sin(math.radians(abs(imu_yaw)))
            yaw_cos = math.cos(math.radians(abs(imu_yaw)))
            if -90 <= imu_yaw < 0:
                temp_x = int(half_width - yaw_sin * (half_height * 0.5))
                temp_y = int(half_height - yaw_cos * (half_height * 0.5))
            elif 0 <= imu_yaw <= 90:
                temp_x = int(half_width + yaw_sin * (half_height * 0.5))
                temp_y = int(half_height - yaw_cos * (half_height * 0.5))
            elif -180 <= imu_yaw < -90:
                temp_x = int(half_width - yaw_sin * (half_height * 0.5))
                temp_y = int(half_height - yaw_cos * (half_height * 0.5))
            elif 90 < imu_yaw <= 180:
                temp_x = int(half_width + yaw_sin * (half_height * 0.5))
                temp_y = int(half_height - yaw_cos * (half_height * 0.5))
            else:
                temp_x = half_width
                temp_y = half_height - (half_height * 0.5)
            cv2.line(rgb_show_line, (half_width, half_height), (temp_x, temp_y), (160, 0, 240), 3)

            if 0 < ac_sensors[4] < 200:
                temp_y = int(half_height - (half_height * ac_sensors[4] / 200))
                cv2.line(rgb_show_line, (0, temp_y), (show_width, temp_y), (0, 255, 0), 1)
            if 0 < ac_sensors[5] < 200:
                temp_y = int(half_height + (half_height * ac_sensors[5] / 200))
                cv2.line(rgb_show_line, (0, temp_y), (show_width, temp_y), (0, 255, 0), 1)
            if ac_sensors[0] == 0:
                cv2.rectangle(rgb_show_line, (half_width - 5, half_height - 10), (half_width, half_height),
                              (0, 255, 0), -1)
            else:
                cv2.rectangle(rgb_show_line, (half_width - 5, half_height - 10), (half_width, half_height),
                              (0, 0, 255), -1)
            if ac_sensors[1] == 0:
                cv2.rectangle(rgb_show_line, (half_width, half_height - 10), (half_width + 5, half_height),
                              (0, 255, 0), -1)
            else:
                cv2.rectangle(rgb_show_line, (half_width, half_height - 10), (half_width + 5, half_height),
                              (0, 0, 255), -1)
            if ac_sensors[2] == 0:
                cv2.rectangle(rgb_show_line, (half_width - 5, half_height), (half_width, half_height + 10),
                              (0, 255, 0), -1)
            else:
                cv2.rectangle(rgb_show_line, (half_width - 5, half_height), (half_width, half_height + 10),
                              (0, 0, 255), -1)
            if ac_sensors[3] == 0:
                cv2.rectangle(rgb_show_line, (half_width, half_height), (half_width + 5, half_height + 10),
                              (0, 255, 0), -1)
            else:
                cv2.rectangle(rgb_show_line, (half_width, half_height), (half_width + 5, half_height + 10),
                              (0, 0, 255), -1)

            cv2.line(rgb_show_line, (0, half_height), (show_width, half_height), (0, 255, 0), 1)
            cv2.line(rgb_show_line, (half_width, 0), (half_width, show_height), (0, 255, 0), 1)

            # 右下的数据展示区
            rgb_show_data = np.zeros((show_height, show_width, 3), np.uint8)
            str_0 = str_Time + '    ' + str(time_mess) + '  Yaw' + str(imu_yaw)
            str_1 = 'L:' + str(cam_left) + '  R:' + str(cam_right)
            str_1 += '  F:' + str(cam_front) + '  B:' + str(cam_back)
            str_2 = 'LaF:' + str(ac_sensors[4]) + '  LaB:' + str(ac_sensors[5]) + '  Fall:'
            if ac_sensors[0] + ac_sensors[1] + ac_sensors[2] + ac_sensors[3] == 0:
                str_2 += 'None'
            else:
                if ac_sensors[0] == 1:
                    str_2 += ' FL '
                if ac_sensors[1] == 1:
                    str_2 += ' FR '
                if ac_sensors[2] == 1:
                    str_2 += ' BL '
                if ac_sensors[3] == 1:
                    str_2 += ' BR '

            str_3 = str(ac_ctl[1])
            if ac_ctl[0] == 0:
                str_3 += '  Stop'
            elif ac_ctl[0] == 1:
                str_3 += '  Front'
            elif ac_ctl[0] == 2:
                str_3 += '  Back'
            elif ac_ctl[0] == 3:
                str_3 += '  Left'
            elif ac_ctl[0] == 4:
                str_3 += '  Right'
            if ac_ctl[2] == 0:
                str_3 += '  Stop'
            elif ac_ctl[2] == 1:
                str_3 += '  Front'
            elif ac_ctl[2] == 2:
                str_3 += '  Back'
            if ac_ctl[3] == 0:
                str_3 += '  Stop'
            elif ac_ctl[3] == 1:
                str_3 += '  Start'

            cv2.putText(rgb_show_data, str_0, (0, int(show_height / 4) - 5), cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 255, 255),
                        1)
            cv2.putText(rgb_show_data, str_1, (0, int(show_height / 2) - 5), cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 255, 255),
                        1)
            cv2.putText(rgb_show_data, str_2, (0, int(show_height * 3 / 4) - 5), cv2.FONT_HERSHEY_COMPLEX, 0.6,
                        (255, 255, 255), 1)
            cv2.putText(rgb_show_data, str_3, (0, show_height - 5), cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 255, 255), 1)

            # 4图拼接
            rgb_mix = np.zeros(((show_height * 2), (show_width * 2), 3), np.uint8)
            rgb_mix[0:show_height, 0:show_width] = rgb_show_0
            rgb_mix[0:show_height, show_width:(show_width * 2)] = rgb_show_line
            rgb_mix[show_height:(show_height * 2), 0:show_width] = rgb_show_1
            rgb_mix[show_height:(show_height * 2), show_width:(show_width * 2)] = rgb_show_data
            cv2.imshow('Show', rgb_mix)

            start_time = time.time()

        cv2.waitKey(1)


def quit_all():
    print('Exit ALL')
    os.kill(os.getpid(), signal.SIGTERM)


# 读取各个传感器信息，返回判断后的状态
def read_feedback(hex_rec, i_yaw):
    rec_mess = binascii.b2a_hex(hex_rec).decode()
    sensor_state = 0
    # 拆分数据包的4个防跌落
    if len(rec_mess) >= 20:
        if rec_mess[0:4] == 'aa02':
            int_fall_FL = int(rec_mess[4:6])
            int_fall_FR = int(rec_mess[6:8])
            int_fall_BL = int(rec_mess[8:10])
            int_fall_BR = int(rec_mess[10:12])
            hex_f_h = hex_rec[6]
            hex_f_l = hex_rec[7]
            hex_b_h = hex_rec[8]
            hex_b_l = hex_rec[9]
            laser_F = hex_f_h << 8 | hex_f_l
            laser_B = hex_b_h << 8 | hex_b_l
        else:
            int_fall_FL = int_fall_FR = int_fall_BL = int_fall_BR = 0
            laser_F = laser_B = 0
    else:
        int_fall_FL = int_fall_FR = int_fall_BL = int_fall_BR = 0
        laser_F = laser_B = 0
    # 多传感器判断
    if (int_fall_FL + int_fall_FR + int_fall_BL + int_fall_BR) > 0:  # 优先判断防跌落
        if (int_fall_FL + int_fall_FR + int_fall_BL + int_fall_BR) > 2:
            sensor_state = 99
        elif int_fall_FL + int_fall_FR == 2:
            sensor_state = 1
        elif int_fall_BL + int_fall_BR == 2:
            sensor_state = 2
        elif int_fall_FL + int_fall_BL == 2:
            sensor_state = 3
        elif int_fall_FR + int_fall_BR == 2:
            sensor_state = 4
        elif int_fall_FL == 1:
            if i_yaw <= -2.0:
                sensor_state = 3
            else:
                sensor_state = 1
        elif int_fall_FR == 1:
            if i_yaw >= 2.0:
                sensor_state = 4
            else:
                sensor_state = 1
        elif int_fall_BL == 1:
            if i_yaw >= 2.0:
                sensor_state = 3
            else:
                sensor_state = 2
        elif int_fall_BR == 1:
            if i_yaw <= -2.0:
                sensor_state = 4
            else:
                sensor_state = 2
    elif laser_F < laser_TH_F or laser_B < laser_TH_B:  # 无跌落风险，判断前后测距
        if laser_F < laser_TH_F and laser_B < laser_TH_B:
            sensor_state = 99
        elif abs(i_yaw) <= 5:
            if laser_F < laser_TH_F:
                sensor_state = 1
            elif laser_B < laser_TH_B:
                sensor_state = 2
        elif i_yaw < -5.0:
            if laser_F < laser_TH_LR:
                sensor_state = 3
            elif laser_B < laser_TH_LR:
                sensor_state = 4
        elif i_yaw > 5.0:
            if laser_F < laser_TH_LR:
                sensor_state = 4
            elif laser_B < laser_TH_LR:
                sensor_state = 3
    elif abs(i_yaw) > 5:  # 没到边沿，判断偏航
        sensor_state = 5
    return sensor_state


# 读取主板的反馈信息，返回各个传感器数据
def read_sensors(hex_rec):
    rec_mess = binascii.b2a_hex(hex_rec).decode()
    fall_FL = str(int(rec_mess[4:6]))
    fall_FR = str(int(rec_mess[6:8]))
    fall_BL = str(int(rec_mess[8:10]))
    fall_BR = str(int(rec_mess[10:12]))
    hex_f_h = hex_rec[6]
    hex_f_l = hex_rec[7]
    hex_b_h = hex_rec[8]
    hex_b_l = hex_rec[9]
    laser_F = hex_f_h << 8 | hex_f_l
    laser_B = hex_b_h << 8 | hex_b_l
    show_mess = 'Fall:' + fall_FL + ';' + fall_FR + ';' + fall_BL + ';' + fall_BR + '. Laser:' + str(
        laser_F) + '-' + str(laser_B)
    return show_mess


# 根据发送的数据，拆分控制信号和主板信号
def read_message(hex_rec):
    str_rec = binascii.b2a_hex(hex_rec).decode()
    is_normal = True
    if len(str_rec) >= 12 and str_rec[0:4] == 'aa01':
        int_move = int(str_rec[4:6])
        int_speed = int(str_rec[6:8], 16)
        int_board = int(str_rec[8:10])
        int_water = int(str_rec[10:12])
        is_normal = True
        ret_list = [int_move, int_speed, int_board, int_water]
    elif len(str_rec) >= 20 and str_rec[0:4] == 'aa02':
        int_fall_FL = int(str_rec[4:6])
        int_fall_FR = int(str_rec[6:8])
        int_fall_BL = int(str_rec[8:10])
        int_fall_BR = int(str_rec[10:12])
        hex_f_h = hex_rec[6]
        hex_f_l = hex_rec[7]
        hex_b_h = hex_rec[8]
        hex_b_l = hex_rec[9]
        laser_F = hex_f_h << 8 | hex_f_l
        laser_B = hex_b_h << 8 | hex_b_l
        is_normal = True
        ret_list = [int_fall_FL, int_fall_FR, int_fall_BL, int_fall_BR, laser_F, laser_B]
    else:
        is_normal = False
        ret_list = [0]
    return is_normal, ret_list


# 根据动作组进行执行，list的0是动作的16进制命令，1是执行多少次
def func_action(list_action, q_i, q_c, q_s, lock_ser, file_address):
    sensor_state = 0  # 传感器状态，99是传感器异常，98是无反馈，97是运行错误

    imu_yaw = 0.0
    imu_pitch = 0.0
    imu_roll = 0.0

    for id_action in range(0, len(list_action), 1):
        loop_nofeedback = 0  # 累加无反馈次数
        threshold_nofeedback = 10  # 累计无反馈上限值
        hex_action = list_action[id_action][0]
        time_action = list_action[id_action][1]
        for num_action in range(0, time_action, 1):
            no_feedback = True
            while no_feedback:
                try:
                    se.write(hex_action)
                    is_read, ret_list = read_message(hex_action)
                    if is_read:
                        q_c.put(ret_list)
                        q_c.get() if q_c.qsize() > 1 else time.sleep(0.001)
                    str_send = binascii.b2a_hex(hex_action).decode('utf-8')
                    str_Time = datetime.datetime.now().strftime('%H:%M:%S.%f')
                    file_rec = open(file_address + 'Control.txt', 'a')
                    file_rec.write(str_Time + ';auto;s;' + str_send + ';\n')
                    file_rec.close()

                    cv2.waitKey(60)

                    # 获取偏航角
                    if not q_i.empty():
                        imu_list = q_i.get()
                        imu_yaw = imu_list[0]
                        imu_pitch = imu_list[1]
                        imu_roll = imu_list[2]

                    hex_rec = se.readline()
                    if hex_rec:
                        # 收到反馈，跳出反馈循环
                        no_feedback = False
                        str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
                        if len(str_rec) >= 12:
                            if str_rec[0:4] == 'aa02':
                                is_read, ret_list = read_message(hex_rec)
                                if is_read:
                                    q_s.put(ret_list)
                                    q_s.get() if q_s.qsize() > 1 else time.sleep(0.001)
                                sensor_mess = read_sensors(hex_rec)
                                print(sensor_mess, str(imu_yaw))
                            else:
                                print('头部异常', str_rec)
                        else:
                            print('长度异常', str_rec)
                        file_rec = open(file_address + 'Control.txt', 'a')
                        file_rec.write(str_Time + ';auto;r;' + str_rec + ';\n')
                        file_rec.close()
                        # if sensor_state > 0:
                        #     return sensor_state, id_action
                    else:
                        # 重新发送命令，并累计无反馈次数
                        loop_nofeedback += 1
                        print('无反馈', str(loop_nofeedback))
                        if loop_nofeedback >= threshold_nofeedback:
                            sensor_state = 98
                            return sensor_state, id_action
                except Exception as e_fa:
                    print(e_fa)
                    print(f'error file:{e_fa.__traceback__.tb_frame.f_globals["__file__"]}')
                    print(f"error line:{e_fa.__traceback__.tb_lineno}")
                    sensor_state = 97
                    return sensor_state, id_action

    return sensor_state, 0


# 多步执行动作
def multi_action(hex_action, times_action, set_yaw, cmd_34, q_i, q_c, q_s, lock_ser, file_address):
    sensor_state = 0  # 传感器状态，99是传感器异常，98是无反馈，97是运行报错
    loop_nofeedback = 0  # 累加无反馈次数
    threshold_nofeedback = 10  # 累计无反馈上限值
    imu_yaw = 0.0
    imu_pitch = 0.0
    imu_roll = 0.0

    for id_action in range(0, times_action, 1):
        no_feedback = True
        while no_feedback:
            try:
                se.write(hex_action)
                is_read, ret_list = read_message(hex_action)
                if is_read:
                    q_c.put(ret_list)
                    q_c.get() if q_c.qsize() > 1 else time.sleep(0.001)
                str_send = binascii.b2a_hex(hex_action).decode('utf-8')
                str_Time = datetime.datetime.now().strftime('%H:%M:%S.%f')
                file_rec = open(file_address + 'Control.txt', 'a')
                file_rec.write(str_Time + ',multi,s,' + str_send + '\r\n')
                file_rec.close()
                # 获取偏航角
                if not q_i.empty():
                    imu_list = q_i.get()
                    imu_yaw = imu_list[0]
                    imu_pitch = imu_list[1]
                    imu_roll = imu_list[2]

                cv2.waitKey(40)
                hex_rec = se.readline()
                if hex_rec:
                    # 收到反馈，跳出反馈循环
                    no_feedback = False
                    str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
                    file_rec = open(file_address + 'Control.txt', 'a')
                    file_rec.write(str_Time + ',multi,r,' + str_rec + '\r\n')
                    file_rec.close()
                    is_read, ret_list = read_message(hex_rec)
                    if is_read:
                        q_s.put(ret_list)
                        q_s.get() if q_s.qsize() > 1 else time.sleep(0.001)
                    sensor_state = read_feedback(hex_rec, imu_yaw)
                    if sensor_state != 0 and sensor_state != 5:
                        break
                    if abs(imu_yaw - set_yaw) > 5.0:
                        correct_yaw(imu_yaw, set_yaw, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                else:
                    # 重新发送命令，并累计无反馈次数
                    loop_nofeedback += 1
                    print('无反馈', str(loop_nofeedback))
                    if loop_nofeedback >= threshold_nofeedback:
                        sensor_state = 98
                        no_feedback = False
                        break
            except Exception as e_sa:
                print(e_sa)
                print(f'error file:{e_sa.__traceback__.tb_frame.f_globals["__file__"]}')
                print(f"error line:{e_sa.__traceback__.tb_lineno}")
                sensor_state = 97
                no_feedback = False
                break
        if sensor_state != 0 and sensor_state != 5:
            break
    if sensor_state == 5:
        sensor_state = 0
    return sensor_state


# 校正偏航
def correct_yaw(now_yaw, target_yaw, cmd_34, q_i, q_c, q_s, lock_ser, file_address):
    get_correct_err = 0
    correct_speed = 100
    imu_yaw = now_yaw
    rot_yaw = now_yaw - target_yaw

    loop_nofeedback = 0  # 累加无反馈次数
    threshold_nofeedback = 10  # 累计无反馈上限值

    while abs(rot_yaw) > 1.0:
        last_yaw = imu_yaw - target_yaw
        # 设置旋转动作
        cmd_2_corSpeed = CVFunc.trans_speed(str(correct_speed))
        if rot_yaw < 0:
            hex_correctYaw = CVFunc.set_order(cmd_0_head + cmd_1_rotateRight + cmd_2_corSpeed + cmd_34)
        else:
            hex_correctYaw = CVFunc.set_order(cmd_0_head + cmd_1_rotateLeft + cmd_2_corSpeed + cmd_34)

        # 发送动作命令
        se.write(hex_correctYaw)
        is_read, ret_list = read_message(hex_correctYaw)
        if is_read:
            q_c.put(ret_list)
            q_c.get() if q_c.qsize() > 1 else time.sleep(0.001)
        str_send = binascii.b2a_hex(hex_correctYaw).decode('utf-8')
        str_Time_cy = datetime.datetime.now().strftime('%H:%M:%S.%f')
        file_rec = open(file_address + 'Control.txt', 'a')
        file_rec.write(str_Time_cy + ';CorrectYaw;s;' + str_send + ';\n')
        file_rec.close()
        # 等待串口发送
        cv2.waitKey(40)

        # 获取偏航角
        if not q_i.empty():
            imu_list = q_i.get()
            imu_yaw = imu_list[0]
            imu_pitch = imu_list[1]
            imu_roll = imu_list[2]

        # 读取主板反馈
        hex_rec = se.readline()
        if hex_rec:
            # 收到反馈
            str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
            file_rec = open(file_address + 'Control.txt', 'a')
            file_rec.write(str_Time_cy + ';CorrectYaw;r;' + str_rec + ';\n')
            file_rec.close()
            is_read, ret_list = read_message(hex_rec)
            if is_read:
                q_s.put(ret_list)
                q_s.get() if q_s.qsize() > 1 else time.sleep(0.001)
            # sensor_state = read_feedback(hex_rec, imu_yaw)
        else:
            # 累计无反馈次数，继续执行
            loop_nofeedback += 1
            print('无反馈', str(loop_nofeedback))
            if loop_nofeedback >= threshold_nofeedback:
                get_correct_err = 98
                print('无反馈，结束运行')
                break

        # 获取偏航角
        if not q_i.empty():
            imu_list = q_i.get()
            imu_yaw = imu_list[0]
            imu_pitch = imu_list[1]
            imu_roll = imu_list[2]

        rot_yaw = imu_yaw - target_yaw
        if abs(rot_yaw) <= 1.0:  # 如果与目标角度相差不大于1，则认为已完成，
            get_correct_err = 0
            break
        else:
            if (abs(last_yaw) > abs(rot_yaw)) and (
                    (last_yaw < 0 and rot_yaw < 0) or (last_yaw > 0 and rot_yaw > 0)):  # 相差变小并且同向
                pass
            elif (last_yaw < 0 and rot_yaw > 0) or (last_yaw > 0 and rot_yaw < 0):  # 偏航角反向了，降低旋转速度
                if correct_speed > 25:
                    correct_speed = correct_speed - 25
    return get_correct_err


# 前后清洗
def go_wash(is_front, cmd_2, cmd_34, q_i, q_c, q_s, q_d, lock_ser, file_address, loop_times):
    sensor_state = 0  # 传感器状态，99是传感器异常，98是无反馈，97是运行报错
    loop_nofeedback = 0  # 累加无反馈次数
    threshold_nofeedback = 10  # 累计无反馈上限值
    imu_yaw = 0.0
    imu_pitch = 0.0
    imu_roll = 0.0
    loop_num = 0

    if is_front:
        hex_wash = CVFunc.set_order(cmd_0_head + cmd_1_moveFront + cmd_2 + cmd_34)
        hex_slow = CVFunc.set_order(cmd_0_head + cmd_1_moveFront + cmd_2_slow + cmd_34)
    else:
        hex_wash = CVFunc.set_order(cmd_0_head + cmd_1_moveBack + cmd_2 + cmd_34)
        hex_slow = CVFunc.set_order(cmd_0_head + cmd_1_moveBack + cmd_2_slow + cmd_34)

    dis_f = 0
    dis_b = 0
    dis_l = 0
    dis_r = 0
    if not q_d.empty():
        dis_list = q_d.get()
        dis_f = dis_list[0]
        dis_b = dis_list[1]
        dis_l = dis_list[2]
        dis_r = dis_list[3]
    before_f = dis_f
    before_b = dis_b
    before_l = dis_l
    before_r = dis_r

    while sensor_state == 0 and (loop_num <= loop_times or loop_times == -1):
        no_feedback = True
        while no_feedback:
            try:
                se.write(hex_wash)
                # 传输主板控制命令
                is_read, ret_list = read_message(hex_wash)
                if is_read:
                    q_c.put(ret_list)
                    q_c.get() if q_c.qsize() > 1 else time.sleep(0.001)
                # 主板控制命令存入txt
                str_send = binascii.b2a_hex(hex_wash).decode('utf-8')
                str_Time_wash = datetime.datetime.now().strftime('%H:%M:%S.%f')
                file_rec = open(file_address + 'Control.txt', 'a')
                file_rec.write(str_Time_wash + ';wash;s;' + str_send + ';\n')
                file_rec.close()

                # 等待串口发送
                cv2.waitKey(40)

                # 获取偏航角
                if not q_i.empty():
                    imu_list = q_i.get()
                    imu_yaw = imu_list[0]
                    imu_pitch = imu_list[1]
                    imu_roll = imu_list[2]
                # 获取单目测距
                if not q_d.empty():
                    dis_list = q_d.get()
                    dis_f = dis_list[0]
                    dis_b = dis_list[1]
                    dis_l = dis_list[2]
                    dis_r = dis_list[3]

                # 读取串口反馈
                hex_rec = se.readline()

                # 获取偏航角
                if not q_i.empty():
                    imu_list = q_i.get()
                    imu_yaw = imu_list[0]
                    imu_pitch = imu_list[1]
                    imu_roll = imu_list[2]
                # 获取单目测距
                if not q_d.empty():
                    dis_list = q_d.get()
                    dis_f = dis_list[0]
                    dis_b = dis_list[1]
                    dis_l = dis_list[2]
                    dis_r = dis_list[3]

                if hex_rec:
                    # 收到反馈，跳出反馈循环，累加次数
                    loop_num += 1
                    no_feedback = False
                    # 主板传感数据存入txt
                    str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
                    file_rec = open(file_address + 'Control.txt', 'a')
                    file_rec.write(str_Time_wash + ';wash;r;' + str_rec + ';\n')
                    file_rec.close()
                    # 传输主板传感数据
                    is_read, ret_list = read_message(hex_rec)
                    if is_read:
                        q_s.put(ret_list)
                        q_s.get() if q_s.qsize() > 1 else time.sleep(0.001)
                    sensor_state = read_feedback(hex_rec, imu_yaw)
                    # 主板感知数据、IMU数据、单目测距数据异常处理
                    if sensor_state == 0 and (abs(before_l - dis_l) > camera_TH_L and abs(before_r - dis_r) > camera_TH_R):     # 如果传感没问题但左右都移动超过阈值，旋转斜向移动后拉直
                        # 根据清洗方向和距离右边情况，执行相应动作
                        if is_front and (before_r - dis_r) < 0:     # 清洗前进时偏向左
                            # 向右旋转10度
                            get_correct = correct_yaw(imu_yaw, 10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                            if get_correct == 98:
                                return get_correct
                            # 斜向移动直至回到距离右边的初始距离
                            while before_r - dis_r < -50:
                                get_correct = multi_action(hex_slow, 1, 10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                                if get_correct == 98:
                                    return get_correct
                                # 获取单目测距
                                if not q_d.empty():
                                    dis_list = q_d.get()
                                    dis_f = dis_list[0]
                                    dis_b = dis_list[1]
                                    dis_l = dis_list[2]
                                    dis_r = dis_list[3]
                        elif is_front and (before_r - dis_r) > 0:   # 清洗前进时偏向右
                            # 向左旋转10度
                            get_correct = correct_yaw(imu_yaw, -10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                            if get_correct == 98:
                                return get_correct
                            # 斜向移动直至回到距离右边的初始距离
                            while before_r - dis_r > 50:
                                get_correct = multi_action(hex_slow, 1, -10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                                if get_correct == 98:
                                    return get_correct
                                # 获取单目测距
                                if not q_d.empty():
                                    dis_list = q_d.get()
                                    dis_f = dis_list[0]
                                    dis_b = dis_list[1]
                                    dis_l = dis_list[2]
                                    dis_r = dis_list[3]
                        elif not is_front and (before_r - dis_r) < 0:   # 清洗后退时偏向左
                            # 向左旋转10度
                            get_correct = correct_yaw(imu_yaw, -10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                            if get_correct == 98:
                                return get_correct
                            # 斜向移动直至回到距离右边的初始距离
                            while before_r - dis_r < -50:
                                get_correct = multi_action(hex_slow, 1, -10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                                if get_correct == 98:
                                    return get_correct
                                # 获取单目测距
                                if not q_d.empty():
                                    dis_list = q_d.get()
                                    dis_f = dis_list[0]
                                    dis_b = dis_list[1]
                                    dis_l = dis_list[2]
                                    dis_r = dis_list[3]
                        elif not is_front and (before_r - dis_r) > 0:   # 清洗后退时偏向右
                            # 向右旋转10度
                            get_correct = correct_yaw(imu_yaw, 10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                            if get_correct == 98:
                                return get_correct
                            # 斜向移动直至回到距离右边的初始距离
                            while before_r - dis_r > 50:
                                get_correct = multi_action(hex_slow, 1, 10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                                if get_correct == 98:
                                    return get_correct
                                # 获取单目测距
                                if not q_d.empty():
                                    dis_list = q_d.get()
                                    dis_f = dis_list[0]
                                    dis_b = dis_list[1]
                                    dis_l = dis_list[2]
                                    dis_r = dis_list[3]
                        # 旋转回直
                        get_correct = correct_yaw(imu_yaw, 0.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)  # 偏航校正
                        if get_correct == 98:
                            return get_correct
                    elif 2 < sensor_state < 5:        # 到左右边，移动远离边界后偏航校正
                        # 停止移动，旋转角度
                        if is_front:
                            hex_correct = CVFunc.set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_34)
                            get_correct = multi_action(hex_correct, 2, imu_yaw, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                            if get_correct == 98:
                                return get_correct

                            if sensor_state == 3:
                                get_correct = correct_yaw(imu_yaw, 10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                                if get_correct == 98:
                                    return get_correct
                                # 旋转后直行，远离边界
                                get_correct = multi_action(hex_wash, 10, 10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                                if get_correct == 98:
                                    return get_correct
                            elif sensor_state == 4:
                                get_correct = correct_yaw(imu_yaw, -10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                                if get_correct == 98:
                                    return get_correct
                                # 旋转后直行，远离边界
                                get_correct = multi_action(hex_wash, 10, -10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                                if get_correct == 98:
                                    return get_correct
                        else:
                            hex_correct = CVFunc.set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_34)
                            get_correct = multi_action(hex_correct, 2, imu_yaw, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                            if get_correct == 98:
                                return get_correct

                            if sensor_state == 3:
                                get_correct = correct_yaw(imu_yaw, -10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                                if get_correct == 98:
                                    return get_correct
                                # 旋转后直行，远离边界
                                get_correct = multi_action(hex_wash, 10, -10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                                if get_correct == 98:
                                    return get_correct

                            elif sensor_state == 4:
                                get_correct = correct_yaw(imu_yaw, 10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                                if get_correct == 98:
                                    return get_correct
                                # 旋转后直行，远离边界
                                get_correct = multi_action(hex_wash, 10, 10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                                if get_correct == 98:
                                    return get_correct

                        # 旋转回直
                        get_correct = correct_yaw(imu_yaw, 0.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)  # 偏航校正
                        if get_correct == 98:
                            return get_correct
                        sensor_state = 0
                    elif sensor_state == 5:     # 清洗偏航，执行偏航校正
                        get_correct = correct_yaw(imu_yaw, 0.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)  # 偏航校正
                        if get_correct == 98:
                            return get_correct
                        sensor_state = 0

                else:
                    # 重新发送命令，并累计无反馈次数
                    loop_nofeedback += 1
                    print('无反馈', str(loop_nofeedback))
                    if loop_nofeedback >= threshold_nofeedback:
                        sensor_state = 98
                        no_feedback = False
            except Exception as e_sa:
                print(e_sa)
                print(f'error file:{e_sa.__traceback__.tb_frame.f_globals["__file__"]}')
                print(f"error line:{e_sa.__traceback__.tb_lineno}")
                sensor_state = 97
                no_feedback = False

    return sensor_state


# 左右平移
def change_column(is_top, to_left, cmd_34, q_i, q_c, q_s, q_d, lock_ser, file_address):
    re_state = 0
    hex_moveFront = CVFunc.set_order(cmd_0_head + cmd_1_moveFront + cmd_2_max + cmd_34)
    hex_moveBack = CVFunc.set_order(cmd_0_head + cmd_1_moveBack + cmd_2_max + cmd_34)
    hex_moveStop = CVFunc.set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_34)
    hex_slowFront = CVFunc.set_order(cmd_0_head + cmd_1_moveFront + cmd_2_slow + cmd_34)
    hex_slowBack = CVFunc.set_order(cmd_0_head + cmd_1_moveBack + cmd_2_slow + cmd_34)

    dis_f = 0
    dis_b = 0
    dis_l = 0
    dis_r = 0
    if not q_d.empty():
        dis_list = q_d.get()
        dis_f = dis_list[0]
        dis_b = dis_list[1]
        dis_l = dis_list[2]
        dis_r = dis_list[3]
    before_f = dis_f
    before_b = dis_b
    before_l = dis_l
    before_r = dis_r

    if is_top:  # 如果在上边
        # 后退1秒，离开上边
        move_state = multi_action(hex_moveBack, 5, 0.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state

        # 如果动作前没有识别到两边，继续提取
        if not q_d.empty():
            dis_list = q_d.get()
            dis_f = dis_list[0]
            dis_b = dis_list[1]
            dis_l = dis_list[2]
            dis_r = dis_list[3]
        before_f = dis_f
        before_b = dis_b
        before_l = dis_l
        before_r = dis_r

        # 左旋或右旋30度，后退形成横向距离，等待确认平移距离，旋转回直
        if to_left:
            # 旋转
            move_state = correct_yaw(0.0, 30.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
            if move_state == 98:
                return move_state
            elif 2 < move_state < 5:
                re_state = move_state
            # 斜向后退
            move_state = multi_action(hex_moveBack, 10, 30.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
            if move_state == 98:
                return move_state
            elif 2 < move_state < 5:
                re_state = move_state
            # 停稳判断平移距离
            move_state = multi_action(hex_moveStop, 2, 30.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
            if move_state == 98:
                return move_state
            elif 2 < move_state < 5:
                re_state = move_state
            # 读取平移距离
            if not q_d.empty():
                dis_list = q_d.get()
                dis_f = dis_list[0]
                dis_b = dis_list[1]
                dis_l = dis_list[2]
                dis_r = dis_list[3]
            after_f = dis_f
            after_b = dis_b
            after_l = dis_l
            after_r = dis_r
            # 左右都判断未到指定距离则进行微调。顶端左移，后向单目的左边为主判断
            while not ((column_move - 100) < abs(before_l - after_l) < (column_move + 100) or (column_move - 100) < abs(before_r - after_r) < (column_move + 100)):
                # 如果移动距离不足，减速继续斜向后退；如果移动距离过多，减速斜向前进。
                if (before_l - after_l - column_move) < 0:
                    move_state = multi_action(hex_slowBack, 1, 30.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    # 读取平移距离
                    if not q_d.empty():
                        dis_list = q_d.get()
                        after_f = dis_list[0]
                        after_b = dis_list[1]
                        after_l = dis_list[2]
                        after_r = dis_list[3]
                else:
                    move_state = multi_action(hex_slowFront, 1, 30.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    # 读取平移距离
                    if not q_d.empty():
                        dis_list = q_d.get()
                        after_f = dis_list[0]
                        after_b = dis_list[1]
                        after_l = dis_list[2]
                        after_r = dis_list[3]
            # 旋转回直
            move_state = correct_yaw(30.0, 0.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
            if move_state == 98:
                return move_state
            elif 2 < move_state < 5:
                re_state = move_state
        else:
            # 旋转
            move_state = correct_yaw(0.0, -30.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
            if move_state == 98:
                return move_state
            elif 2 < move_state < 5:
                re_state = move_state
            # 斜向后退
            move_state = multi_action(hex_moveBack, 10, -30.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
            if move_state == 98:
                return move_state
            elif 2 < move_state < 5:
                re_state = move_state
            # 停稳判断平移距离
            move_state = multi_action(hex_moveStop, 2, -30.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
            if move_state == 98:
                return move_state
            elif 2 < move_state < 5:
                re_state = move_state
            # 读取平移距离
            if not q_d.empty():
                dis_list = q_d.get()
                dis_f = dis_list[0]
                dis_b = dis_list[1]
                dis_l = dis_list[2]
                dis_r = dis_list[3]
            after_f = dis_f
            after_b = dis_b
            after_l = dis_l
            after_r = dis_r
            # 左右都判断未到指定距离则进行微调。顶端右移，后向单目的左边为主判断
            while not ((column_move - 100) < abs(before_l - after_l) < (column_move + 100) or (column_move - 100) < abs(before_r - after_r) < (column_move + 100)):
                # 如果移动距离不足，减速继续斜向后退；如果移动距离过多，减速斜向前进。
                if (before_r - after_r - column_move) < 0:
                    move_state = multi_action(hex_slowBack, 1, -30.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    # 读取平移距离
                    if not q_d.empty():
                        dis_list = q_d.get()
                        after_f = dis_list[0]
                        after_b = dis_list[1]
                        after_l = dis_list[2]
                        after_r = dis_list[3]
                else:
                    move_state = multi_action(hex_slowFront, 1, -30.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    # 读取平移距离
                    if not q_d.empty():
                        dis_list = q_d.get()
                        after_f = dis_list[0]
                        after_b = dis_list[1]
                        after_l = dis_list[2]
                        after_r = dis_list[3]

            # 旋转回直
            move_state = correct_yaw(-30.0, 0.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
            if move_state == 98:
                return move_state
            elif 2 < move_state < 5:
                re_state = move_state

        # 前进2秒，回到上边
        move_state = multi_action(hex_moveFront, 10, 0.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state

    else:  # 如果在下边
        # 前进1秒，离开下边
        move_state = multi_action(hex_moveFront, 5, 0.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state

        # 左旋或右旋30度，前进形成横向距离，等待确认平移距离，旋转回直
        if to_left:
            # 旋转
            move_state = correct_yaw(0.0, -30.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
            if move_state == 98:
                return move_state
            elif 2 < move_state < 5:
                re_state = move_state
            # 斜向前进
            move_state = multi_action(hex_moveFront, 10, -30.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
            if move_state == 98:
                return move_state
            elif 2 < move_state < 5:
                re_state = move_state
            # 停稳判断平移距离
            move_state = multi_action(hex_moveStop, 2, -30.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
            if move_state == 98:
                return move_state
            elif 2 < move_state < 5:
                re_state = move_state
            # 读取平移距离
            if not q_d.empty():
                dis_list = q_d.get()
                dis_f = dis_list[0]
                dis_b = dis_list[1]
                dis_l = dis_list[2]
                dis_r = dis_list[3]
            after_f = dis_f
            after_b = dis_b
            after_l = dis_l
            after_r = dis_r
            # 左右都判断未到指定距离则进行微调。底部左移，前向单目的左边为主判断
            while not ((column_move - 100) < abs(before_l - after_l) < (column_move + 100) or (column_move - 100) < abs(before_r - after_r) < (column_move + 100)):
                # 如果移动距离不足，减速继续斜向前进；如果移动距离过多，减速斜向后退。
                if (before_l - after_l - column_move) < 0:
                    move_state = multi_action(hex_slowFront, 1, -30.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    # 读取平移距离
                    if not q_d.empty():
                        dis_list = q_d.get()
                        after_f = dis_list[0]
                        after_b = dis_list[1]
                        after_l = dis_list[2]
                        after_r = dis_list[3]
                else:
                    move_state = multi_action(hex_slowBack, 1, -30.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    # 读取平移距离
                    if not q_d.empty():
                        dis_list = q_d.get()
                        after_f = dis_list[0]
                        after_b = dis_list[1]
                        after_l = dis_list[2]
                        after_r = dis_list[3]

            # 旋转回直
            move_state = correct_yaw(-30.0, 0.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
            if move_state == 98:
                return move_state
            elif 2 < move_state < 5:
                re_state = move_state
        else:
            # 旋转
            move_state = correct_yaw(0.0, 30.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
            if move_state == 98:
                return move_state
            elif 2 < move_state < 5:
                re_state = move_state
            # 斜向前进
            move_state = multi_action(hex_moveFront, 10, 30.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
            if move_state == 98:
                return move_state
            elif 2 < move_state < 5:
                re_state = move_state
            # 停稳判断平移距离
            move_state = multi_action(hex_moveStop, 2, 30.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
            if move_state == 98:
                return move_state
            elif 2 < move_state < 5:
                re_state = move_state
            # 读取平移距离
            if not q_d.empty():
                dis_list = q_d.get()
                dis_f = dis_list[0]
                dis_b = dis_list[1]
                dis_l = dis_list[2]
                dis_r = dis_list[3]
            after_f = dis_f
            after_b = dis_b
            after_l = dis_l
            after_r = dis_r
            # 左右都判断未到指定距离则进行微调。底部右移，前向单目的右边为主判断
            while not ((column_move - 100) < abs(before_l - after_l) < (column_move + 100) or (column_move - 100) < abs(before_r - after_r) < (column_move + 100)):
                # 如果移动距离不足，减速继续斜向前进；如果移动距离过多，减速斜向后退。
                if (before_r - after_r - column_move) < 0:
                    move_state = multi_action(hex_slowFront, 1, 30.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    # 读取平移距离
                    if not q_d.empty():
                        dis_list = q_d.get()
                        after_f = dis_list[0]
                        after_b = dis_list[1]
                        after_l = dis_list[2]
                        after_r = dis_list[3]
                else:
                    move_state = multi_action(hex_slowBack, 1, 30.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    # 读取平移距离
                    if not q_d.empty():
                        dis_list = q_d.get()
                        after_f = dis_list[0]
                        after_b = dis_list[1]
                        after_l = dis_list[2]
                        after_r = dis_list[3]
            # 旋转回直
            move_state = correct_yaw(30.0, 0.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
            if move_state == 98:
                return move_state
            elif 2 < move_state < 5:
                re_state = move_state
        # 后退2秒，回到下边
        move_state = multi_action(hex_moveBack, 10, q_i, q_c, q_s, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state

    return re_state


# 执行自动控制
def autocontrol_run(q_i, q_d, q_c, q_s, lock_ser, file_address):
    # 设置上下次数
    loop_time = 2  # 上行+下行算作1次
    # 设置平移方向及次数
    move_side = 0  # 0不平移，1左移，2右移
    move_times = 0  # 设置平移次数
    move_num = 0  # 累计平移次数

    wash_loops = 20     # 设置上下清洗的移动距离限定，-1代表无限定。每次200ms，5次相当于1秒

    # 设置清洗速度
    cmd_2_washSpeed = CVFunc.trans_speed('50')

    # 启动水系统
    hex_sprayStart = CVFunc.set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_stop + cmd_4_start)
    # 全停止
    hex_allStop = CVFunc.set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_stop + cmd_4_stop)
    # 刮板向前
    hex_boardFront = CVFunc.set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_front + cmd_4_stop)
    # 刮板向前，启动水系统
    hex_sprayFront = CVFunc.set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_front + cmd_4_start)
    # 清洗前进，移动停止
    hex_stopFront = CVFunc.set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_front + cmd_4_start)
    # 刮板向前，关闭水系统
    hex_sprayStop_boardFront = CVFunc.set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_front + cmd_4_stop)
    # 刮板向后
    hex_boardBack = CVFunc.set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_back + cmd_4_stop)
    # 刮板向下，启动水系统
    hex_sprayBack = CVFunc.set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_back + cmd_4_start)
    # 清洗下行，移动停止
    hex_stopBack = CVFunc.set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_back + cmd_4_start)
    # 刮板向下，关闭水系统
    hex_sprayStop_boardBack = CVFunc.set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_back + cmd_4_stop)

    # 动作组-准备清洗向前
    list_prepareFront = [[hex_boardFront, 15],  # 刮板向前，15次约3秒
                         [hex_sprayFront, 1]]  # 启动水系统，1次

    # 动作组-准备清洗向后
    list_prepareBack = [[hex_boardBack, 15],  # 刮板后，15次约3秒
                        [hex_sprayBack, 1]]  # 启动水系统，1次

    # 动作组-清洗向前到边
    list_edgeFront = [[hex_stopFront, 2],  # 移动停止，2次
                      [hex_sprayStop_boardFront, 1],  # 关闭水系统，1次
                      [hex_boardBack, 7],  # 刮板由前，改为向后，7次约1.4秒，预计到中间位置
                      [hex_allStop, 1]]  # 全停止，进行下一指令
    # 动作组-清洗向后到边
    list_edgeBack = [[hex_stopBack, 2],  # 移动停止，2次
                     [hex_sprayStop_boardBack, 1],  # 关闭水系统，1次
                     [hex_boardFront, 7],  # 刮板由前，改为向后，7次与1.4秒，预计到中间位置
                     [hex_allStop, 1]]  # 全停止，进行下一指令

    imu_yaw = 0.0
    imu_pitch = 0.0
    imu_roll = 0.0
    board_cmd = cmd_3_stop + cmd_4_stop

    # 无反馈重新循环标志位
    no_feedBack = False
    while True:
        # 测试通信
        in_init = True
        while in_init:
            str_init = 'aa 01 00 00 00 00 a8'
            hex_init_send = bytes.fromhex(str_init)
            str_Time_ac = datetime.datetime.now().strftime('%H:%M:%S.%f')
            try:
                # 获取偏航角
                if not q_i.empty():
                    imu_list = q_i.get()
                    imu_yaw = imu_list[0]
                    imu_pitch = imu_list[1]
                    imu_roll = imu_list[2]

                se.write(hex_init_send)
                is_read, ret_list = read_message(hex_init_send)
                if is_read:
                    q_c.put(ret_list)
                    q_c.get() if q_c.qsize() > 1 else time.sleep(0.001)
                file_rec = open(file_address + 'Control.txt', 'a')
                file_rec.write(str_Time_ac + ';init;s;' + str_init + ';\n')
                file_rec.close()
                # 发送后，延时读取
                cv2.waitKey(30)

                # 获取偏航角
                if not q_i.empty():
                    imu_list = q_i.get()
                    imu_yaw = imu_list[0]
                    imu_pitch = imu_list[1]
                    imu_roll = imu_list[2]

                hex_init_rec = se.readline()
                if hex_init_rec:
                    str_init_rec = binascii.b2a_hex(hex_init_rec).decode('utf-8')
                    if len(str_init_rec) >= 12:
                        if str_init_rec[0:4] == 'aa02':
                            is_read, ret_list = read_message(hex_init_rec)
                            if is_read:
                                q_s.put(ret_list)
                                q_s.get() if q_s.qsize() > 1 else time.sleep(0.001)
                            sensor_init_mess = read_sensors(hex_init_rec)
                            print(sensor_init_mess, str(imu_yaw))
                            in_init = False
                            no_feedBack = False
                        else:
                            print('串口数据异常')
                    else:
                        print('串口数据异常')
                    file_rec = open(file_address + 'Control.txt', 'a')
                    file_rec.write(str_Time_ac + ';init;r;' + str_init_rec + ';\n')
                    file_rec.close()
            except Exception as e:
                print(e)
                print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
                print(f"error line:{e.__traceback__.tb_lineno}")

        # 初始偏航校正
        print('初始偏航校正')
        # 获取偏航角
        if not q_i.empty():
            imu_list = q_i.get()
            imu_yaw = imu_list[0]
            imu_pitch = imu_list[1]
            imu_roll = imu_list[2]

        if abs(imu_yaw) > 3.0:
            board_cmd = cmd_3_stop + cmd_4_stop
            get_state = correct_yaw(imu_yaw, 0.0, board_cmd, q_i, q_c, q_s, lock_ser, file_address)
            if get_state == 98:
                no_feedBack = True
        if no_feedBack:
            continue

        # 进入自动控制
        print('进入自动控制')
        get_state = 0
        # 建立通信，开始执行
        for loop_num in range(0, loop_time, 1):
            # 准备清洗前进
            print('刮板向前，启动水系统')
            get_state = func_action(list_prepareFront, q_i, q_c, q_s, lock_ser, file_address)
            board_cmd = cmd_3_front + cmd_4_start
            if get_state == 98:
                no_feedBack = True
                break

            # 清洗前进
            print('清洗前行，次数', str(wash_loops))
            get_state = go_wash(True, cmd_2_washSpeed, board_cmd, q_i, q_c, q_s, q_d, lock_ser, file_address, wash_loops)
            if get_state == 98:
                no_feedBack = True
                break
            elif get_state == 2:
                print('后向误判')
            elif get_state > 90:
                print('故障')

            # 上行到上边
            print('到达顶部')
            get_state = func_action(list_edgeFront, q_i, q_c, q_s, lock_ser, file_address)
            board_cmd = cmd_3_stop + cmd_4_stop
            if get_state == 98:
                no_feedBack = True
                break

            # 到上边平移
            if move_side == 0:  # 直行，不平移
                pass
            elif move_side == 1:  # 上边左移
                print('向左平移，已移动', str(move_num))
                get_err_T2L = change_column(True, True, board_cmd, q_i, q_c, q_s, q_d, lock_ser, file_address)
                move_num += 1
                if move_num >= move_times:
                    move_num = 0
                    move_side = 2
                if get_err_T2L == 98:
                    no_feedBack = True
                    break
                elif get_err_T2L == 3:
                    move_num = 0
                    move_side = 2
            elif move_side == 2:  # 上边右移
                print('向右平移，已移动', str(move_num))
                get_err_T2R = change_column(True, False, board_cmd, q_i, q_c, q_s, q_d, lock_ser, file_address)
                move_num += 1
                if move_num >= move_times:
                    move_num = 0
                    move_side = 1
                if get_err_T2R == 98:
                    no_feedBack = True
                    break
                elif get_err_T2R == 4:
                    move_num = 0
                    move_side = 1

            # 准备清洗后退
            print('刮板向后，启动水系统')
            get_state = func_action(list_prepareBack, q_i, q_c, q_s, lock_ser, file_address)
            board_cmd = cmd_3_back + cmd_4_start
            if get_state == 98:
                no_feedBack = True
                break

            # 清洗后退
            print('清洗后退，次数', str(wash_loops))
            get_state = go_wash(False, cmd_2_washSpeed, board_cmd, q_i, q_c, q_s, q_d, lock_ser, file_address, wash_loops)
            if get_state == 98:
                no_feedBack = True
                break
            elif get_state == 1:
                print('前向误判')
            elif get_state > 90:
                print('故障')

            # 下行到下边
            print('到达底部')
            get_state = func_action(list_edgeBack, q_i, q_c, q_s, lock_ser, file_address)
            board_cmd = cmd_3_stop + cmd_4_stop
            if get_state == 98:
                no_feedBack = True
                break

            # 到下边，如果不是最后一轮，则平移
            if loop_num < (loop_time - 1):
                if move_side == 0:  # 直行，不平移
                    pass
                elif move_side == 1:  # 左移
                    print('向左平移，已移动', str(move_num))
                    get_err_B2L = change_column(False, True, board_cmd, q_i, q_c, q_s, q_d, lock_ser, file_address)
                    move_num += 1
                    if move_num >= move_times:
                        move_num = 0
                        move_side = 2
                    if get_err_B2L == 98:
                        no_feedBack = True
                        break
                    elif get_err_B2L == 3:
                        move_num = 0
                        move_side = 2
                elif move_side == 2:  # 右移
                    print('向右平移，已移动', str(move_num))
                    get_err_B2R = change_column(False, False, board_cmd, q_i, q_c, q_s, q_d, lock_ser, file_address)
                    move_num += 1
                    if move_num >= move_times:
                        move_num = 0
                        move_side = 1
                    if get_err_B2R == 98:
                        no_feedBack = True
                        break
                    elif get_err_B2R == 4:
                        move_num = 0
                        move_side = 1
            print('循环剩余', str(loop_time - loop_num))
        if no_feedBack:
            continue


if __name__ == '__main__':

    if cali_type == 0:
        minCan, maxCan = camera_init()
        # pass
    elif cali_type == 1:
        minCan = int(input('minCan:'))
        maxCan = int(input('maxCan:'))
    else:
        minCan = 40
        maxCan = 100

    # 新建文件夹,读取时间作为文件名
    str_fileAddress = './TestData/'
    str_Time = datetime.datetime.now().strftime('%Y%m%d-%H%M')
    # file_rec = open(str_fileHome + str_Time + '.txt', 'w', encoding='utf-8')
    str_fileAddress += str_Time
    if not os.path.exists(str_fileAddress):
        os.makedirs(str_fileAddress)
    str_fileAddress += '/'

    mp.set_start_method(method='spawn')  # init
    processes = []
    lock = mp.Lock()
    # 前向相机，图像及3向线段组
    queue_image_0 = mp.Queue(maxsize=2)
    queue_front_0 = mp.Queue(maxsize=2)
    queue_left_0 = mp.Queue(maxsize=2)
    queue_right_0 = mp.Queue(maxsize=2)
    # 后向相机，图像及3向线段组
    queue_image_1 = mp.Queue(maxsize=2)
    queue_back_1 = mp.Queue(maxsize=2)
    queue_left_1 = mp.Queue(maxsize=2)
    queue_right_1 = mp.Queue(maxsize=2)
    # IMU，偏航、俯仰和翻滚，分发前后相机、左右计算和自动控制
    queue_imu_0 = mp.Queue(maxsize=2)
    queue_imu_1 = mp.Queue(maxsize=2)
    queue_imu_main = mp.Queue(maxsize=2)
    queue_imu_auto = mp.Queue(maxsize=2)
    # 测距、自控交互，测距的4向距离，自控的控制命令和传感器数据
    queue_distance = mp.Queue(maxsize=2)
    queue_control = mp.Queue(maxsize=2)
    queue_sensors = mp.Queue(maxsize=2)

    # 前向视觉测距
    processes.append(
        mp.Process(target=distance_get, args=(
            queue_image_0, queue_front_0, queue_left_0, queue_right_0, queue_imu_0, lock, front_id, str_fileAddress)))
    # 后向视觉测距
    processes.append(
        mp.Process(target=distance_get, args=(
            queue_image_1, queue_back_1, queue_right_1, queue_left_1, queue_imu_1, lock, back_id, str_fileAddress)))
    # IMU测姿态
    processes.append(
        mp.Process(target=imu_get, args=(
            queue_imu_0, queue_imu_1, queue_imu_main, queue_imu_auto, lock, str_fileAddress)))
    # 视觉感知融合
    processes.append(
        mp.Process(target=multi_calc, args=(
            queue_image_0, queue_image_1, queue_front_0, queue_back_1, queue_left_0, queue_right_0,
            queue_left_1, queue_right_1, queue_imu_main, queue_distance, queue_control, queue_sensors,
            str_fileAddress)))
    # 自动运行
    processes.append(
        mp.Process(target=autocontrol_run, args=(
            queue_imu_auto, queue_distance, queue_control, queue_sensors, lock, str_fileAddress)))

    for process in processes:
        process.daemon = True
        process.start()
    for process in processes:
        process.join()
