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
import crcmod
import keyboard
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

# 初始化状态标志位
glo_is_init = True

# 相机编号和IMU串口编号
camera_id = 0
imu_com = '/dev/ttyTHS1'

# 主板通信串口
se = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

# 机身尺寸
vehicle_left = 118
vehicle_right = 127
vehicle_front = 148

# 左右单目阈值
camera_TH_L = 100
camera_TH_R = 100

cmd_0_head = 'aa 01'
cmd_1_stop = '00'
cmd_1_moveFront = '01'
cmd_1_moveBack = '02'
cmd_1_rotateLeft = '04'
cmd_1_rotateRight = '03'
cmd_2_speed0 = '00'
cmd_2_slow = '0a'
cmd_2_max = '64'
cmd_3_stop = '00'
cmd_3_front = '01'
cmd_3_back = '02'
cmd_4_stop = '00'
cmd_4_start = '01'


# 抓取图片，确认视频流的读入
def image_put(q, c_id):
    cap = cv2.VideoCapture(c_id)
    cap.set(6, 1196444237)
    cap.set(3, 1920)
    cap.set(4, 1080)
    cap.set(5, 30)
    if cap.isOpened():
        print('Get1', c_id)
    else:
        cap = cv2.VideoCapture(c_id)
        cap.set(6, 1196444237)
        cap.set(3, 1920)
        cap.set(4, 1080)
        cap.set(5, 30)
        print('Get2', c_id)

    while cap.isOpened():
        ret, frame = cap.read()
        # 抓取图片不成功再重新抓取
        if not ret:
            cap = cv2.VideoCapture(c_id)
            print('Get3', c_id)
            ret, frame = cap.read()
        q.put(frame)
        # print('q.qsize():', q.qsize())
        q.get() if q.qsize() > 1 else time.sleep(0.01)


# 调用相机获取图片进行测距
def distance_get(q_c, q_i, q_yi, q_ym, q_img, q_f, q_l, q_r, lock_ser, cap_id, file_address, q_temp):
    # 根据相机编号分配模型参数
    global para_lines
    global minCan, maxCan

    model_F = float(para_lines[0].strip('\n'))
    model_W = float(para_lines[1].strip('\n'))
    model_a = float(para_lines[2].strip('\n'))
    model_b = float(para_lines[3].strip('\n'))
    principal_x = int(para_lines[4].strip('\n'))
    principal_y = int(para_lines[5].strip('\n'))

    # 循环处理图像
    loop_num = 0
    imu_roll = 0.0
    imu_pitch = 0.0
    imu_yaw = 0.0
    while True:
        start_time_total = time.time()
        str_Time = datetime.datetime.now().strftime('%H%M%S-%f')
        ret_mess = ''  # 数据信息
        time_mess = ''  # 时间信息

        # 获取图像及参数
        start_time = time.time()
        if not q_c.empty():
            rgb_frame = q_c.get()
            loop_num += 1
        else:
            continue
        img_height = int(rgb_frame.shape[0])
        img_width = int(rgb_frame.shape[1])
        mid_height = int(img_height / 2)
        mid_width = int(img_width / 2)
        rgb_rot = rgb_frame.copy()
        rgb_show = rgb_frame.copy()
        # 获取偏航角
        if not q_i.empty():
            jy_list = q_i.get()
            imu_roll = jy_list[0]
            imu_pitch = jy_list[1]
            imu_yaw = jy_list[2]
        end_time = time.time()
        time_mess += 'Cap:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 保留下半部分的红色区域（Canny提取边界，保留下半部分）
        start_time = time.time()
        rgb_half = rgb_frame.copy()
        rgb_half[0:principal_y - 50, :] = (0, 0, 0)
        hsv_half = cv2.cvtColor(rgb_half, cv2.COLOR_BGR2HSV)
        low_range = np.array([0, 123, 100])
        high_range = np.array([5, 255, 255])
        gra_edge = cv2.inRange(hsv_half, low_range, high_range)
        gra_canny = gra_edge.copy()
        # gra_edge = cv2.Canny(rgb_half, minCan, maxCan)
        # gra_canny = gra_edge.copy()
        end_time = time.time()
        time_mess += 'Can:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 获取水平和垂直线
        start_time = time.time()
        gra_edge_rot = gra_edge.copy()
        gra_edge_rot[0:principal_y, :] = 0
        lines = cv2.HoughLinesP(gra_edge_rot, rho=1.0, theta=np.pi / 180, threshold=100, minLineLength=100, maxLineGap=5)
        end_time = time.time()
        time_mess += 'Hou:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 旋转校正、拉平、融合
        start_time = time.time()
        gra_hough = np.zeros((img_height, img_width), np.uint8)  # 创建个全0的黑背景
        # 偏航角
        yaw_avg = 0.0
        yaw_weight = 0.0
        other_avg = 0.0
        other_weight = 0.0
        # 垂直、水平线坐标中继
        num_ver = 0
        lines_ver = [[0, 0, 0, 0]]
        num_hor = 0
        lines_hor = [[0, 0, 0, 0]]
        data_ver = np.zeros((0, 3), dtype=float)  # 垂直线数据，0是最接近中轴的w（x）值，1是最接近相机的h（y）值，2是最远离相机的h（y）值
        data_hor = np.zeros((0, 3), dtype=float)  # 水平线数据，0是最接近相机的h（y）值，1是最左侧（最小）的w（x）值，2是最右侧（最大）的w（x）值
        # 左边、右边垂直线数据
        num_left = 0
        num_right = 0
        dis_l = [0]
        dis_r = [0]
        num_front = 0
        dis_f = [0]

        # 车身边界数据
        try:
            if str(type(lines)) != "<class 'NoneType'>":
                if len(lines) > 0:
                    # 计算偏航角
                    for line in lines:
                        for x1_p, y1_p, x2_p, y2_p in line:
                            if CVFunc.getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 100.0:
                                cv2.line(gra_hough, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                                # 根据偏航角进行旋转
                                h1 = CVFunc.calc_horizontal(y1_p, model_F, model_W, model_a, model_b)
                                h2 = CVFunc.calc_horizontal(y2_p, model_F, model_W, model_a, model_b)
                                w1 = CVFunc.calc_vertical((x1_p - principal_x), y1_p, model_F, model_W, model_a,
                                                          model_b)
                                w2 = CVFunc.calc_vertical((x2_p - principal_x), y2_p, model_F, model_W, model_a,
                                                          model_b)

                                # 在2米范围内
                                if h1 < 2000 and h2 < 2000 and w1 < 2000 and w2 < 2000:
                                    if h1 != h2:
                                        angle_tan = np.arctan((w1 - w2) / (h2 - h1)) * 57.29577
                                    else:
                                        angle_tan = 90.0
                                    # x_mid = int((x1_p + x2_p) / 2)
                                    # y_mid = int((y1_p + y2_p) / 2)
                                    # cv2.line(rgb_rot, (x1_p, y1_p), (x2_p, y2_p), (255, 0, 0), 1)
                                    # cv2.putText(rgb_rot, str(round(angle_tan, 0)), (x_mid, y_mid),
                                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)
                                    # cv2.putText(rgb_rot, str(h1) + ',' + str(w1), (x1_p, y1_p),
                                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)
                                    # cv2.putText(rgb_rot, str(h2) + ',' + str(w2), (x2_p, y2_p),
                                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)
                                    if abs(angle_tan) < 45:
                                        temp_weight = math.sqrt(np.square(h1 - h2) + np.square(w1 - w2))
                                        temp_sum = yaw_avg * yaw_weight + angle_tan * temp_weight
                                        yaw_weight = yaw_weight + temp_weight
                                        yaw_avg = temp_sum / yaw_weight
                                        if num_ver == 0:
                                            num_ver = 1
                                            lines_ver[0] = [w1, h1, w2, h2]
                                        else:
                                            num_ver += 1
                                            lines_ver.append([w1, h1, w2, h2])
                                    else:
                                        temp_weight = math.sqrt(np.square(h1 - h2) + np.square(w1 - w2))
                                        temp_sum = other_avg * other_weight + angle_tan * temp_weight
                                        other_weight = other_weight + temp_weight
                                        other_avg = temp_sum / other_weight
                                        if num_hor == 0:
                                            num_hor = 1
                                            lines_hor[0] = [w1, h1, w2, h2]
                                        else:
                                            num_hor += 1
                                            lines_hor.append([w1, h1, w2, h2])
                    # 如果计算了偏航角，则反馈给IMU进程和计算进程
                    if yaw_weight != 0:
                        q_yi.put(yaw_avg)
                        q_yi.get() if q_yi.qsize() > 1 else time.sleep(0.005)
                        q_ym.put(round(yaw_avg, 2))
                        q_ym.get() if q_ym.qsize() > 1 else time.sleep(0.005)
                    # 融合和画出垂直线
                    if num_ver != 0:
                        # 根据偏航角调整融合垂直线
                        for w1, h1, w2, h2 in lines_ver:
                            x1_imu, y1_imu = CVFunc.points_rotate(yaw_avg, w1, h1)
                            x2_imu, y2_imu = CVFunc.points_rotate(yaw_avg, w2, h2)
                            # 垂直线,按照最接近中轴来拉直
                            if x1_imu != x2_imu and y1_imu != y2_imu:
                                if abs(x1_imu) > abs(x2_imu):
                                    x1_imu = x2_imu
                                else:
                                    x2_imu = x1_imu
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
                                    data_ver = np.append(data_ver, [[x1_imu, temp_button, temp_top]], axis=0)
                        # 反推图像位置
                        for values_ver in data_ver:
                            w1_b, h1_b = CVFunc.points_rotate(-yaw_avg, values_ver[0], values_ver[1])
                            w2_b, h2_b = CVFunc.points_rotate(-yaw_avg, values_ver[0], values_ver[2])
                            y1_b = CVFunc.calc_h2y(h1_b, model_F, model_W, model_a, model_b)
                            y2_b = CVFunc.calc_h2y(h2_b, model_F, model_W, model_a, model_b)
                            x1_b = CVFunc.calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
                            x2_b = CVFunc.calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
                            x_mid = int((x1_b + x2_b) / 2)
                            y_mid = int((y1_b + y2_b) / 2)
                            cv2.line(rgb_rot, (x1_b, y1_b), (x2_b, y2_b), (255, 0, 0), 1)
                            cv2.putText(rgb_rot, str(round(values_ver[0], 0)), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.8,
                                        (255, 0, 0), 1)
                            cv2.line(rgb_show, (x1_b, y1_b), (x2_b, y2_b), (255, 0, 0), 6)
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
                    # 融合和画出水平线
                    if num_hor != 0:
                        # 根据偏航角调整融合垂直线
                        for w1, h1, w2, h2 in lines_hor:
                            x1_imu, y1_imu = CVFunc.points_rotate(yaw_avg, w1, h1)
                            x2_imu, y2_imu = CVFunc.points_rotate(yaw_avg, w2, h2)
                            # 水平线,按照最接近相机来拉平
                            if x1_imu != x2_imu and y1_imu != y2_imu:
                                if abs(x1_imu) > abs(x2_imu):
                                    y1_imu = y2_imu
                                else:
                                    y2_imu = y1_imu
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
                        # 反推图像位置
                        for values_hor in data_hor:
                            w1_b, h1_b = CVFunc.points_rotate(-yaw_avg, values_hor[1], values_hor[0])
                            w2_b, h2_b = CVFunc.points_rotate(-yaw_avg, values_hor[2], values_hor[0])
                            y1_b = CVFunc.calc_h2y(h1_b, model_F, model_W, model_a, model_b)
                            y2_b = CVFunc.calc_h2y(h2_b, model_F, model_W, model_a, model_b)
                            x1_b = CVFunc.calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
                            x2_b = CVFunc.calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
                            x_mid = int((x1_b + x2_b) / 2)
                            y_mid = int((y1_b + y2_b) / 2)
                            cv2.line(rgb_rot, (x1_b, y1_b), (x2_b, y2_b), (0, 255, 0), 1)
                            cv2.putText(rgb_rot, str(round(values_hor[0], 0)), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.8, (0, 255, 0), 1)
                            cv2.line(rgb_show, (x1_b, y1_b), (x2_b, y2_b), (0, 255, 0), 6)
                            num_front += 1
                            if dis_f[0] == 0:
                                dis_f[0] = int(values_hor[0])
                            else:
                                dis_f.append(int(values_hor[0]))

                    if num_left != 0:
                        dis_l.sort()
                        q_l.put(dis_l)
                        if q_l.qsize() > 1:
                            q_l.get()
                    if num_right != 0:
                        dis_r.sort()
                        q_r.put(dis_r)
                        if q_r.qsize() > 1:
                            q_r.get()
                    if num_front != 0:
                        dis_f.sort()
                        q_f.put(dis_f)
                        if q_f.qsize() > 1:
                            q_f.get()

            end_time = time.time()
            time_mess += 'Cal:' + str(round((end_time - start_time) * 1000, 4)) + ';'
        except Exception as e:
            print(e)
            print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
            print(f"error line:{e.__traceback__.tb_lineno}")

        # 画相机中心十字
        start_time = time.time()
        cv2.line(rgb_rot, (principal_x, 0), (principal_x, img_height), (255, 255, 0), 1)
        cv2.line(rgb_rot, (0, principal_y), (img_width, principal_y), (255, 255, 0), 1)
        cv2.circle(rgb_rot, (principal_x, principal_y), 5, (255, 255, 0), 3)
        cv2.line(rgb_show, (principal_x, 0), (principal_x, img_height), (255, 255, 0), 1)
        cv2.line(rgb_show, (0, principal_y), (img_width, principal_y), (255, 255, 0), 1)
        cv2.circle(rgb_show, (principal_x, principal_y), 5, (255, 255, 0), 3)
        q_img.put(rgb_show)
        if q_img.qsize() > 1:
            q_img.get()
        q_temp.put(gra_canny)
        if q_temp.qsize() > 1:
            q_temp.get()
        # cv2.imshow('Test', rgb_show)
        # cv2.waitKey(1)

        # cv2.imwrite(file_address + 'C' + str(cap_id) + '-' + str_Time + '.jpg', rgb_frame)
        # cv2.imwrite(file_address + 'D' + str(cap_id) + '-' + str_Time + '.jpg', rgb_rot)

        # 保存txt，传输数据
        # file_rec = open(file_address + str(cap_id) + '.txt', 'a')
        # end_time = time.time()
        # time_mess += 'Shw:' + str(round((end_time - start_time) * 1000, 4)) + ';'
        # end_time_total = time.time()
        # time_mess += 'All:' + str(round((end_time_total - start_time_total) * 1000, 4)) + ';\n'
        #
        # if len(time_mess) > 0:
        #     file_rec.write('Tpy:Timer;' + time_mess)
        # file_rec.close()


# 读取IMU数据
def imu_get(q_ia, q_id, q_im, q_y, q_c, lock_ser, file_address):
    global glo_is_init

    cv2.waitKey(500)
    se_i = serial.Serial(imu_com, 9600, timeout=0.05)
    # 释放串口积攒的数据
    se_i.flushInput()
    se_i.flushOutput()
    print('IMU start')
    while True:
        try:
            # 控制信号标志位
            is_ctl = False
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
                        # print(str(round(jy_list[6], 2)), str(round(jy_list[7], 2)), str(round(jy_list[8], 2)))
                        q_id.put(send_list)
                        q_id.get() if q_id.qsize() > 1 else time.sleep(0.005)
                        q_im.put(send_list)
                        q_im.get() if q_im.qsize() > 1 else time.sleep(0.005)
                        q_ia.put(send_list)
                        q_ia.get() if q_ia.qsize() > 1 else time.sleep(0.005)
                    else:
                        sav_mess = ('NoneType;' + str_imu + ';\n')
                        print(sav_mess)
                        se_i.flushOutput()
                else:
                    sav_mess = ('error;' + str_imu + ';\n')
                    print(sav_mess)
                    se_i.flushOutput()
                file_rec.write(str_time + ';' + sav_mess)
                file_rec.close()

            if not q_y.empty() and not q_c.empty() and glo_is_init:
                yaw_c = q_y.get()
                ac_ctl = q_c.get()
                if abs(yaw_c) <= 0.1 and ac_ctl[0] == 0:
                    str_zero = 'ff aa 52'
                    hex_zero = bytes.fromhex(str_zero)
                    se_i.write(hex_zero)
                    cv2.waitKey(50)

        except Exception as e:
            print(e)
            print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
            print(f"error line:{e.__traceback__.tb_lineno}")


# 融合前后单目、超声和IMU，快速更新四向和偏航角
def multi_calc(q_img, q_f, q_l, q_r, q_y, q_i, q_d, q_c, file_address, q_temp):
    cv2.waitKey(500)
    print('calc start')

    imu_yaw = 0.0
    imu_roll = 0.0
    imu_pitch = 0.0

    cam_front = 0
    cam_left = 0
    cam_right = 0
    cam_yaw = 0.0

    dis_f = [0]
    dis_l = [0]
    dis_r = [0]

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

    rgb_cam = np.zeros((show_height, show_width, 3), np.uint8)

    # 车身边界数据
    side_left = 0.0
    side_right = 0.0
    side_front = 0.0

    while True:
        if not q_img.empty():
            print('Get Img')
            break
        else:
            print('No Image')
            cv2.waitKey(100)

    while True:
        str_Time = datetime.datetime.now().strftime('%H:%M:%S.%f')
        # 收到数据标志位
        is_imu = False
        is_cam = False
        is_f = False
        is_l = False
        is_r = False
        is_y = False
        is_ctl = False

        # 发往主板的控制命令
        if not q_c.empty():
            is_ctl = True
            ac_ctl = q_c.get()

        # IMU
        if not q_i.empty():
            is_imu = True
            imu_list = q_i.get()
            imu_roll = imu_list[0]
            imu_pitch = imu_list[1]
            imu_yaw = -imu_list[2]

        # 单目收到数据
        if not q_img.empty():
            end_time = time.time()
            time_mess = round((end_time - start_time) * 1000, 0)

            # 临时缓存，提取红色或Canny变换图像
            gra_temp = q_temp.get()
            gra_canny = cv2.resize(gra_temp, (show_width, show_height))
            rgb_show_1 = cv2.cvtColor(gra_canny, cv2.COLOR_GRAY2BGR)

            # 左上展示图像
            is_cam = True
            rgb_tmp = q_img.get()
            rgb_show_0 = cv2.resize(rgb_tmp, (show_width, show_height))

            # 视觉偏航角
            if not q_y.empty():
                is_y = True
                cam_yaw = q_y.get()
            else:
                cam_yaw = -999.9

            # 前向水平线
            if not q_f.empty():
                is_f = True
                dis_f = q_f.get()
                cam_front = dis_f[0]
                side_front = cam_front + 20
            else:
                side_front = -999.9

            # 前向左垂线
            if not q_l.empty():
                is_l = True
                dis_l = q_l.get()
                cam_left = dis_l[0]
                if is_y:
                    temp_yaw = cam_yaw
                else:
                    temp_yaw = imu_yaw
                if temp_yaw >= 0:
                    side_left = round((cam_left - vehicle_front * math.sin(math.radians(abs(temp_yaw))) - vehicle_left * math.sin(math.radians(90.0 - abs(temp_yaw)))), 1)
                else:
                    side_left = round((cam_left + vehicle_front * math.sin(math.radians(abs(temp_yaw))) - vehicle_left * math.sin(math.radians(90.0 - abs(temp_yaw)))), 1)
            else:
                side_left = -999.9

            # 前向右垂线
            if not q_r.empty():
                is_r = True
                dis_r = q_r.get()
                cam_right = dis_r[0]
                if is_y != 0:
                    temp_yaw = cam_yaw
                else:
                    temp_yaw = -imu_yaw
                if temp_yaw >= 0:
                    side_right = round((cam_right + vehicle_front * math.sin(math.radians(abs(temp_yaw))) - vehicle_right * math.sin(math.radians(90.0 - abs(temp_yaw)))), 1)
                else:
                    side_right = round((cam_right - vehicle_front * math.sin(math.radians(abs(temp_yaw))) - vehicle_right * math.sin(math.radians(90.0 - abs(temp_yaw)))), 1)
            else:
                side_right = -999.9

            send_list = [side_front, side_left, side_right, cam_yaw]
            q_d.put(send_list)
            q_d.get() if q_d.qsize() > 1 else time.sleep(0.005)

            # 右上的距离展示区
            rgb_show_line = np.zeros((show_height, show_width, 3), np.uint8)
            if is_f:
                for i in range(0, len(dis_f), 1):
                    temp_y = int(show_height - (show_height * dis_f[i] / 2000))
                    if i == 0:
                        cv2.line(rgb_show_line, (0, temp_y), (show_width, temp_y), (0, 0, 255), 1)
                    else:
                        cv2.line(rgb_show_line, (0, temp_y), (show_width, temp_y), (255, 0, 0), 1)
            if is_l:
                for i in range(0, len(dis_l), 1):
                    temp_x = int(half_width - (half_width * dis_l[i] / 2000))
                    if i == 0:
                        cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (0, 0, 255), 1)
                    else:
                        cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (255, 0, 0), 1)
            if is_r:
                for i in range(0, len(dis_r), 1):
                    temp_x = int(half_width + (half_width * dis_r[i] / 2000))
                    if i == 0:
                        cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (0, 0, 255), 1)
                    else:
                        cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (255, 0, 0), 1)

            # 视觉的偏航角显示
            if is_y:
                yaw_sin = math.sin(math.radians(abs(cam_yaw)))
                yaw_cos = math.cos(math.radians(abs(cam_yaw)))
                if -90 <= cam_yaw < 0:
                    temp_x = int(half_width - yaw_sin * (half_height * 0.5))
                    temp_y = int(show_height - yaw_cos * (half_height * 0.5))
                    cv2.line(rgb_show_line, (half_width, show_height), (temp_x, temp_y), (160, 0, 240), 3)
                elif 0 <= cam_yaw <= 90:
                    temp_x = int(half_width + yaw_sin * (half_height * 0.5))
                    temp_y = int(show_height - yaw_cos * (half_height * 0.5))
                    cv2.line(rgb_show_line, (half_width, show_height), (temp_x, temp_y), (160, 0, 240), 3)

            # IMU的偏航角显示
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

            temp_x = int(half_width - (half_width * vehicle_left / 2000))
            cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (255, 255, 255), 1)
            temp_x = int(half_width + (half_width * vehicle_right / 2000))
            cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (255, 255, 255), 1)

            # 右下的数据展示区
            rgb_show_data = np.zeros((show_height, show_width, 3), np.uint8)
            str_0 = str_Time + '    ' + str(time_mess) + '  F:'
            if is_f:
                str_0 += str(side_front)
            else:
                str_0 += 'N/A'
            if is_l:
                str_1 = 'cL:' + str(cam_left) + '  sL:' + str(side_left)
            else:
                str_1 = 'cL:N/A  sL:N/A'
            if is_r:
                str_1 += '  sR:' + str(side_right) + '  cR:' + str(cam_right)
            else:
                str_1 += '  sR:N/A  cR:N/A'
            if is_y:
                str_2 = 'cYaw:' + str(cam_yaw)
            else:
                str_2 = 'cYaw:N/A'
            str_2 += '  iYaw:' + str(imu_yaw) + '  Roll:' + str(imu_roll) + '  Pitch:' + str(imu_pitch)

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


            cv2.putText(rgb_show_data, str_0, (0, int(show_height / 4) - 5), cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255),
                        1)
            cv2.putText(rgb_show_data, str_1, (0, int(show_height / 2) - 5), cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255),
                        1)
            cv2.putText(rgb_show_data, str_2, (0, int(show_height * 3 / 4) - 5), cv2.FONT_HERSHEY_COMPLEX, 0.4,
                        (255, 255, 255), 1)
            cv2.putText(rgb_show_data, str_3, (0, show_height - 5), cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)

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
# def func_action(list_action, q_i, q_c, q_s, lock_ser, file_address):
#     sensor_state = 0  # 传感器状态，99是传感器异常，98是无反馈，97是运行错误
#
#     imu_yaw = 0.0
#     imu_pitch = 0.0
#     imu_roll = 0.0
#
#     for id_action in range(0, len(list_action), 1):
#         loop_nofeedback = 0  # 累加无反馈次数
#         threshold_nofeedback = 10  # 累计无反馈上限值
#         hex_action = list_action[id_action][0]
#         time_action = list_action[id_action][1]
#         for num_action in range(0, time_action, 1):
#             no_feedback = True
#             while no_feedback:
#                 try:
#                     se.write(hex_action)
#                     is_read, ret_list = read_message(hex_action)
#                     if is_read:
#                         q_c.put(ret_list)
#                         q_c.get() if q_c.qsize() > 1 else time.sleep(0.001)
#                     str_send = binascii.b2a_hex(hex_action).decode('utf-8')
#                     str_Time = datetime.datetime.now().strftime('%H:%M:%S.%f')
#                     file_rec = open(file_address + 'Control.txt', 'a')
#                     file_rec.write(str_Time + ';auto;s;' + str_send + ';\n')
#                     file_rec.close()
#
#                     cv2.waitKey(60)
#
#                     # 获取偏航角
#                     if not q_i.empty():
#                         imu_list = q_i.get()
#                         imu_yaw = imu_list[0]
#                         imu_pitch = imu_list[1]
#                         imu_roll = imu_list[2]
#
#                     hex_rec = se.readline()
#                     if hex_rec:
#                         # 收到反馈，跳出反馈循环
#                         no_feedback = False
#                         str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
#                         if len(str_rec) >= 12:
#                             if str_rec[0:4] == 'aa02':
#                                 is_read, ret_list = read_message(hex_rec)
#                                 if is_read:
#                                     q_s.put(ret_list)
#                                     q_s.get() if q_s.qsize() > 1 else time.sleep(0.001)
#                                 sensor_mess = read_sensors(hex_rec)
#                                 print(sensor_mess, str(imu_yaw))
#                             else:
#                                 print('头部异常', str_rec)
#                         else:
#                             print('长度异常', str_rec)
#                         file_rec = open(file_address + 'Control.txt', 'a')
#                         file_rec.write(str_Time + ';auto;r;' + str_rec + ';\n')
#                         file_rec.close()
#                         # if sensor_state > 0:
#                         #     return sensor_state, id_action
#                     else:
#                         # 重新发送命令，并累计无反馈次数
#                         loop_nofeedback += 1
#                         print('无反馈', str(loop_nofeedback))
#                         if loop_nofeedback >= threshold_nofeedback:
#                             sensor_state = 98
#                             return sensor_state, id_action
#                 except Exception as e_fa:
#                     print(e_fa)
#                     print(f'error file:{e_fa.__traceback__.tb_frame.f_globals["__file__"]}')
#                     print(f"error line:{e_fa.__traceback__.tb_lineno}")
#                     sensor_state = 97
#                     return sensor_state, id_action
#
#     return sensor_state, 0


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
                    # is_read, ret_list = read_message(hex_rec)
                    # if is_read:
                    #     q_s.put(ret_list)
                    #     q_s.get() if q_s.qsize() > 1 else time.sleep(0.001)
                    # sensor_state = read_feedback(hex_rec, imu_yaw)
                    # if sensor_state != 0 and sensor_state != 5:
                    #     break
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


# 校正偏航角
def correct_yaw(now_yaw, target_yaw, cmd_34, q_i, q_d, q_c, q_ci, file_address):
    get_correct_err = 0
    imu_yaw = now_yaw
    rot_yaw = now_yaw - target_yaw

    loop_nofeedback = 0  # 累加无反馈次数
    threshold_nofeedback = 10  # 累计无反馈上限值

    target_count = 0

    # while abs(rot_yaw) > 1.0:
    while target_count <= 5:
        last_yaw = imu_yaw - target_yaw
        # 根据偏航角设定初始旋转速度
        if abs(rot_yaw) > 30:
            correct_speed = 50
        elif abs(rot_yaw) > 20:
            correct_speed = 30
        elif abs(rot_yaw) > 10:
            correct_speed = 20
        else:
            correct_speed = 10
        # 设置旋转动作
        cmd_2_corSpeed = CVFunc.trans_speed(str(correct_speed))
        if target_count == 0:
            if rot_yaw < 0:
                hex_correctYaw = CVFunc.set_order(cmd_0_head + cmd_1_rotateRight + cmd_2_corSpeed + cmd_34)
            else:
                hex_correctYaw = CVFunc.set_order(cmd_0_head + cmd_1_rotateLeft + cmd_2_corSpeed + cmd_34)
        else:
            hex_correctYaw = CVFunc.set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_34)

        # 发送动作命令
        se.write(hex_correctYaw)
        is_read, ret_list = read_message(hex_correctYaw)
        if is_read:
            q_c.put(ret_list)
            q_c.get() if q_c.qsize() > 1 else time.sleep(0.001)
            q_ci.put(ret_list)
            q_ci.get() if q_ci.qsize() > 1 else time.sleep(0.001)
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
            imu_yaw = -imu_list[2]

        # 读取主板反馈
        hex_rec = se.readline()
        if hex_rec:
            # 收到反馈
            str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
            file_rec = open(file_address + 'Control.txt', 'a')
            file_rec.write(str_Time_cy + ';CorrectYaw;r;' + str_rec + ';\n')
            file_rec.close()
            # is_read, ret_list = read_message(hex_rec)
            # if is_read:
            #     q_s.put(ret_list)
            #     q_s.get() if q_s.qsize() > 1 else time.sleep(0.001)
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
            imu_yaw = -imu_list[2]

        rot_yaw = imu_yaw - target_yaw
        if abs(rot_yaw) <= 1.0:  # 如果与目标角度相差不大于1，则认为已完成，
            target_count += 1
        else:
            target_count = 0
            # if (abs(last_yaw) > abs(rot_yaw)) and (
            #         (last_yaw < 0 and rot_yaw < 0) or (last_yaw > 0 and rot_yaw > 0)):  # 相差变小并且同向
            #     pass
            # elif (last_yaw < 0 and rot_yaw > 0) or (last_yaw > 0 and rot_yaw < 0):  # 偏航角反向了，降低旋转速度
            #     if correct_speed > 10:
            #         correct_speed = correct_speed - 10
    return get_correct_err


# 校正偏移距离
def correct_deviation(now_dis, target_dis, cmd_34, q_i, q_d, q_c, q_ci, file_address):
    get_correct_err = 0
    side_l = now_dis
    move_dis = now_dis - target_dis

    loop_nofeedback = 0  # 累加无反馈次数
    threshold_nofeedback = 10  # 累计无反馈上限值

    imu_yaw = 0.0
    side_l = -999.9

    target_count = 0

    # 获取偏航角
    if not q_i.empty():
        imu_list = q_i.get()
        imu_yaw = -imu_list[2]

    # 左旋30度
    get_correct_err = correct_yaw(imu_yaw, -30.0, cmd_34, q_i, q_d, q_c, q_ci, file_address)
    # 无反馈退出
    if get_correct_err == 98:
        return get_correct_err

    correct_speed = 25
    cmd_2_corSpeed = CVFunc.trans_speed(str(correct_speed))
    if move_dis < 0:
        hex_correctDis = CVFunc.set_order(cmd_0_head + cmd_1_moveBack + cmd_2_corSpeed + cmd_34)
    else:
        hex_correctDis = CVFunc.set_order(cmd_0_head + cmd_1_moveFront + cmd_2_corSpeed + cmd_34)
    move_times = int(abs(move_dis) * 2 / 10)    # 30度角，移动距离需要是偏移量的2倍
    # 根据距离，发送动作命令
    for i in range(0, move_times, 1):
        se.write(hex_correctDis)
        is_read, ret_list = read_message(hex_correctDis)
        if is_read:
            q_c.put(ret_list)
            q_c.get() if q_c.qsize() > 1 else time.sleep(0.001)
            q_ci.put(ret_list)
            q_ci.get() if q_ci.qsize() > 1 else time.sleep(0.001)
        str_send = binascii.b2a_hex(hex_correctDis).decode('utf-8')
        str_Time_cy = datetime.datetime.now().strftime('%H:%M:%S.%f')
        file_rec = open(file_address + 'Control.txt', 'a')
        file_rec.write(str_Time_cy + ';CorrectYaw;s;' + str_send + ';\n')
        file_rec.close()
        # 等待串口发送
        cv2.waitKey(40)
        # 读取主板反馈
        hex_rec = se.readline()
        if hex_rec:
            # 收到反馈
            str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
            file_rec = open(file_address + 'Control.txt', 'a')
            file_rec.write(str_Time_cy + ';CorrectYaw;r;' + str_rec + ';\n')
            file_rec.close()
        else:
            # 累计无反馈次数，继续执行
            loop_nofeedback += 1
            print('无反馈', str(loop_nofeedback))
            if loop_nofeedback >= threshold_nofeedback:
                get_correct_err = 98
                print('无反馈，结束运行')
                # 获取视觉数据
        if not q_d.empty():
            dis_list = q_d.get()
            side_l = dis_list[1]

    # 等待视觉测距稳定
    for i in range(0, 3, 1):
        hex_correctDis = CVFunc.set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_34)
        se.write(hex_correctDis)
        is_read, ret_list = read_message(hex_correctDis)
        if is_read:
            q_c.put(ret_list)
            q_c.get() if q_c.qsize() > 1 else time.sleep(0.001)
            q_ci.put(ret_list)
            q_ci.get() if q_ci.qsize() > 1 else time.sleep(0.001)
        str_send = binascii.b2a_hex(hex_correctDis).decode('utf-8')
        str_Time_cy = datetime.datetime.now().strftime('%H:%M:%S.%f')
        file_rec = open(file_address + 'Control.txt', 'a')
        file_rec.write(str_Time_cy + ';CorrectYaw;s;' + str_send + ';\n')
        file_rec.close()
        # 等待串口发送
        cv2.waitKey(40)
        # 读取主板反馈
        hex_rec = se.readline()
        if hex_rec:
            # 收到反馈
            str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
            file_rec = open(file_address + 'Control.txt', 'a')
            file_rec.write(str_Time_cy + ';CorrectYaw;r;' + str_rec + ';\n')
            file_rec.close()
        else:
            # 累计无反馈次数，继续执行
            loop_nofeedback += 1
            print('无反馈', str(loop_nofeedback))
            if loop_nofeedback >= threshold_nofeedback:
                get_correct_err = 98
                print('无反馈，结束运行')
        # 获取视觉数据
        if not q_d.empty():
            dis_list = q_d.get()
            side_temp = dis_list[1]
            if side_temp != -999.9:
                side_l = side_temp
        # 获取偏航角
        if not q_i.empty():
            imu_list = q_i.get()
            imu_yaw = -imu_list[2]

    move_dis = side_l - target_dis
    while abs(move_dis) > 10:
        print(move_dis)
        correct_speed = 10
        cmd_2_corSpeed = CVFunc.trans_speed(str(correct_speed))
        if move_dis < 0:
            hex_correctDis = CVFunc.set_order(cmd_0_head + cmd_1_moveBack + cmd_2_corSpeed + cmd_34)
        else:
            hex_correctDis = CVFunc.set_order(cmd_0_head + cmd_1_moveFront + cmd_2_corSpeed + cmd_34)
        # 根据距离，发送动作命令
        se.write(hex_correctDis)
        is_read, ret_list = read_message(hex_correctDis)
        if is_read:
            q_c.put(ret_list)
            q_c.get() if q_c.qsize() > 1 else time.sleep(0.001)
            q_ci.put(ret_list)
            q_ci.get() if q_ci.qsize() > 1 else time.sleep(0.001)
        str_send = binascii.b2a_hex(hex_correctDis).decode('utf-8')
        str_Time_cy = datetime.datetime.now().strftime('%H:%M:%S.%f')
        file_rec = open(file_address + 'Control.txt', 'a')
        file_rec.write(str_Time_cy + ';CorrectYaw;s;' + str_send + ';\n')
        file_rec.close()
        # 等待串口发送
        cv2.waitKey(40)
        # 读取主板反馈
        hex_rec = se.readline()
        if hex_rec:
            # 收到反馈
            str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
            file_rec = open(file_address + 'Control.txt', 'a')
            file_rec.write(str_Time_cy + ';CorrectYaw;r;' + str_rec + ';\n')
            file_rec.close()
        else:
            # 累计无反馈次数，继续执行
            loop_nofeedback += 1
            print('无反馈', str(loop_nofeedback))
            if loop_nofeedback >= threshold_nofeedback:
                get_correct_err = 98
                print('无反馈，结束运行')
        # 获取偏航角
        if not q_i.empty():
            imu_list = q_i.get()
            imu_yaw = -imu_list[2]
        # 获取视觉数据，更新剩余距离
        if not q_d.empty():
            dis_list = q_d.get()
            side_temp = dis_list[1]
            if side_temp != -999.9:
                side_l = side_temp
                move_dis = side_l - target_dis
            else:
                if move_dis < 0:
                    move_dis = move_dis + 2
                else:
                    move_dis = move_dis - 2
        else:
            if move_dis < 0:
                move_dis = move_dis + 2
            else:
                move_dis = move_dis - 2

    get_correct_err = correct_yaw(imu_yaw, 0.0, cmd_34, q_i, q_d, q_c, q_ci, file_address)
    # 无反馈退出
    if get_correct_err == 98:
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
                    # # 传输主板传感数据
                    # is_read, ret_list = read_message(hex_rec)
                    # if is_read:
                    #     q_s.put(ret_list)
                    #     q_s.get() if q_s.qsize() > 1 else time.sleep(0.001)
                    # sensor_state = read_feedback(hex_rec, imu_yaw)
                    # # 主板感知数据、IMU数据、单目测距数据异常处理
                    # if sensor_state == 0 and (abs(before_l - dis_l) > camera_TH_L and abs(before_r - dis_r) > camera_TH_R):     # 如果传感没问题但左右都移动超过阈值，旋转斜向移动后拉直
                    #     # 根据清洗方向和距离右边情况，执行相应动作
                    #     if is_front and (before_r - dis_r) < 0:     # 清洗前进时偏向左
                    #         # 向右旋转10度
                    #         get_correct = correct_yaw(imu_yaw, 10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    #         if get_correct == 98:
                    #             return get_correct
                    #         # 斜向移动直至回到距离右边的初始距离
                    #         while before_r - dis_r < -50:
                    #             get_correct = multi_action(hex_slow, 1, 10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    #             if get_correct == 98:
                    #                 return get_correct
                    #             # 获取单目测距
                    #             if not q_d.empty():
                    #                 dis_list = q_d.get()
                    #                 dis_f = dis_list[0]
                    #                 dis_b = dis_list[1]
                    #                 dis_l = dis_list[2]
                    #                 dis_r = dis_list[3]
                    #     elif is_front and (before_r - dis_r) > 0:   # 清洗前进时偏向右
                    #         # 向左旋转10度
                    #         get_correct = correct_yaw(imu_yaw, -10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    #         if get_correct == 98:
                    #             return get_correct
                    #         # 斜向移动直至回到距离右边的初始距离
                    #         while before_r - dis_r > 50:
                    #             get_correct = multi_action(hex_slow, 1, -10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    #             if get_correct == 98:
                    #                 return get_correct
                    #             # 获取单目测距
                    #             if not q_d.empty():
                    #                 dis_list = q_d.get()
                    #                 dis_f = dis_list[0]
                    #                 dis_b = dis_list[1]
                    #                 dis_l = dis_list[2]
                    #                 dis_r = dis_list[3]
                    #     elif not is_front and (before_r - dis_r) < 0:   # 清洗后退时偏向左
                    #         # 向左旋转10度
                    #         get_correct = correct_yaw(imu_yaw, -10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    #         if get_correct == 98:
                    #             return get_correct
                    #         # 斜向移动直至回到距离右边的初始距离
                    #         while before_r - dis_r < -50:
                    #             get_correct = multi_action(hex_slow, 1, -10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    #             if get_correct == 98:
                    #                 return get_correct
                    #             # 获取单目测距
                    #             if not q_d.empty():
                    #                 dis_list = q_d.get()
                    #                 dis_f = dis_list[0]
                    #                 dis_b = dis_list[1]
                    #                 dis_l = dis_list[2]
                    #                 dis_r = dis_list[3]
                    #     elif not is_front and (before_r - dis_r) > 0:   # 清洗后退时偏向右
                    #         # 向右旋转10度
                    #         get_correct = correct_yaw(imu_yaw, 10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    #         if get_correct == 98:
                    #             return get_correct
                    #         # 斜向移动直至回到距离右边的初始距离
                    #         while before_r - dis_r > 50:
                    #             get_correct = multi_action(hex_slow, 1, 10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    #             if get_correct == 98:
                    #                 return get_correct
                    #             # 获取单目测距
                    #             if not q_d.empty():
                    #                 dis_list = q_d.get()
                    #                 dis_f = dis_list[0]
                    #                 dis_b = dis_list[1]
                    #                 dis_l = dis_list[2]
                    #                 dis_r = dis_list[3]
                    #     # 旋转回直
                    #     get_correct = correct_yaw(imu_yaw, 0.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)  # 偏航校正
                    #     if get_correct == 98:
                    #         return get_correct
                    # elif 2 < sensor_state < 5:        # 到左右边，移动远离边界后偏航校正
                    #     # 停止移动，旋转角度
                    #     if is_front:
                    #         hex_correct = CVFunc.set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_34)
                    #         get_correct = multi_action(hex_correct, 2, imu_yaw, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    #         if get_correct == 98:
                    #             return get_correct
                    #
                    #         if sensor_state == 3:
                    #             get_correct = correct_yaw(imu_yaw, 10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    #             if get_correct == 98:
                    #                 return get_correct
                    #             # 旋转后直行，远离边界
                    #             get_correct = multi_action(hex_wash, 10, 10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    #             if get_correct == 98:
                    #                 return get_correct
                    #         elif sensor_state == 4:
                    #             get_correct = correct_yaw(imu_yaw, -10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    #             if get_correct == 98:
                    #                 return get_correct
                    #             # 旋转后直行，远离边界
                    #             get_correct = multi_action(hex_wash, 10, -10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    #             if get_correct == 98:
                    #                 return get_correct
                    #     else:
                    #         hex_correct = CVFunc.set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_34)
                    #         get_correct = multi_action(hex_correct, 2, imu_yaw, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    #         if get_correct == 98:
                    #             return get_correct
                    #
                    #         if sensor_state == 3:
                    #             get_correct = correct_yaw(imu_yaw, -10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    #             if get_correct == 98:
                    #                 return get_correct
                    #             # 旋转后直行，远离边界
                    #             get_correct = multi_action(hex_wash, 10, -10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    #             if get_correct == 98:
                    #                 return get_correct
                    #
                    #         elif sensor_state == 4:
                    #             get_correct = correct_yaw(imu_yaw, 10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    #             if get_correct == 98:
                    #                 return get_correct
                    #             # 旋转后直行，远离边界
                    #             get_correct = multi_action(hex_wash, 10, 10.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)
                    #             if get_correct == 98:
                    #                 return get_correct
                    #
                    #     # 旋转回直
                    #     get_correct = correct_yaw(imu_yaw, 0.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)  # 偏航校正
                    #     if get_correct == 98:
                    #         return get_correct
                    #     sensor_state = 0
                    # elif sensor_state == 5:     # 清洗偏航，执行偏航校正
                    #     get_correct = correct_yaw(imu_yaw, 0.0, cmd_34, q_i, q_c, q_s, lock_ser, file_address)  # 偏航校正
                    #     if get_correct == 98:
                    #         return get_correct
                    #     sensor_state = 0

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


def single_action(hex_action, q_c, q_ci, str_fileAddress):
    sensor_state = 0  # 传感器状态，99是传感器异常，98是无反馈，97是运行报错
    loop_nofeedback = 0  # 累加无反馈次数
    threshold_nofeedback = 10  # 累计无反馈上限值
    no_feedback = True
    while no_feedback:
        try:
            se.write(hex_action)
            is_read, ret_list = read_message(hex_action)
            if is_read:
                q_c.put(ret_list)
                q_c.get() if q_c.qsize() > 1 else time.sleep(0.001)
                q_ci.put(ret_list)
                q_ci.get() if q_ci.qsize() > 1 else time.sleep(0.001)
            str_send = binascii.b2a_hex(hex_action).decode('utf-8')
            # str_Time = datetime.datetime.now().strftime('%H:%M:%S.%f')
            # file_rec = open(str_fileAddress, 'a')
            # file_rec.write(str_Time + ',single,s,' + str_send + '\r\n')
            # file_rec.close()
            # print(str_Time, 's', str_send)
            cv2.waitKey(40)
            hex_rec = se.readline()
            if hex_rec:
                # 收到反馈，跳出反馈循环
                no_feedback = False
                str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
                if len(str_rec) >= 12:
                    if str_rec[0:4] == 'aa02':
                        pass
                        # sensor_mess = read_sensors(hex_rec)
                        # print(sensor_mess)
                    else:
                        print('头部异常', str_rec)
                else:
                    print('长度异常', str_rec)
                # file_rec = open(str_fileAddress, 'a')
                # file_rec.write(str_Time + ',single,r,' + str_rec + '\r\n')
                # file_rec.close()
            else:
                # 重新发送命令，并累计无反馈次数
                loop_nofeedback += 1
                print('无反馈', str(loop_nofeedback))
                if loop_nofeedback >= threshold_nofeedback:
                    sensor_state = 98
                    no_feedback = False
        except Exception as e_sa:
            print(e_sa)
            sensor_state = 97
            no_feedback = False
    return sensor_state


# 执行自动控制
def autocontrol_run(q_i, q_d, q_c, q_ci, lock_ser, file_address):
    global glo_is_init

    # 设置移动速度
    cmd_2_moveSpeed = CVFunc.trans_speed('50')
    # 设置旋转速度
    cmd_2_rotatePalstance = CVFunc.trans_speed('20')
    # 移动向前
    hex_moveFront = CVFunc.set_order(cmd_0_head + cmd_1_moveFront + cmd_2_moveSpeed + cmd_3_stop + cmd_4_stop)
    # 移动向后
    hex_moveBack = CVFunc.set_order(cmd_0_head + cmd_1_moveBack + cmd_2_moveSpeed + cmd_3_stop + cmd_4_stop)
    # 右旋
    hex_rotateRight = CVFunc.set_order(cmd_0_head + cmd_1_rotateRight + cmd_2_rotatePalstance + cmd_3_stop + cmd_4_stop)
    # 左旋
    hex_rotateLeft = CVFunc.set_order(cmd_0_head + cmd_1_rotateLeft + cmd_2_rotatePalstance + cmd_3_stop + cmd_4_stop)
    # 全停止
    hex_allStop = CVFunc.set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_stop + cmd_4_stop)


    # 设置上下次数
    loop_time = 2  # 前进+后退算作1次
    wash_loops = 10     # 设置前后清洗的移动距离限定，-1代表无限定。每次200ms，5次相当于1秒
    # 设置清洗速度
    cmd_2_washSpeed = CVFunc.trans_speed('25')

    imu_yaw = 0.0
    imu_pitch = 0.0
    imu_roll = 0.0
    board_cmd = cmd_3_stop + cmd_4_stop

    cam_yaw = -999.9
    side_f = -999.9
    side_l = -999.9
    side_r = -999.9

    # 无反馈重新循环标志位
    no_feedBack = False
    while True:
        glo_is_init = True
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
                    imu_roll = imu_list[0]
                    imu_pitch = imu_list[1]
                    imu_yaw = -imu_list[2]

                se.write(hex_init_send)
                is_read, ret_list = read_message(hex_init_send)
                if is_read:
                    q_c.put(ret_list)
                    q_c.get() if q_c.qsize() > 1 else time.sleep(0.001)
                    q_ci.put(ret_list)
                    q_ci.get() if q_ci.qsize() > 1 else time.sleep(0.001)
                file_rec = open(file_address + 'Control.txt', 'a')
                file_rec.write(str_Time_ac + ';init;s;' + str_init + ';\n')
                file_rec.close()
                # 发送后，延时读取
                cv2.waitKey(30)

                # 获取偏航角
                if not q_i.empty():
                    imu_list = q_i.get()
                    imu_roll = imu_list[0]
                    imu_pitch = imu_list[1]
                    imu_yaw = -imu_list[2]

                hex_init_rec = se.readline()
                if hex_init_rec:
                    str_init_rec = binascii.b2a_hex(hex_init_rec).decode('utf-8')
                    if len(str_init_rec) >= 20:
                        if str_init_rec[0:4] == 'aa02':
                            is_read, ret_list = read_message(hex_init_rec)
                            # print('翻滚：' + str(imu_roll) + '，俯仰：' + str(imu_pitch) + '，偏航：' + str(imu_yaw))
                            # in_init = False
                            no_feedBack = False
                        else:
                            print('串口异常')
                    else:
                        print('串口异常')
                if keyboard.is_pressed('n'):  # 结束运行，全部停止
                    print('进入单步动作测试')
                    in_init = False
            except Exception as e:
                print(e)
                print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
                print(f"error line:{e.__traceback__.tb_lineno}")

        # 单个动作测试
        is_oneAction = True
        glo_is_init = True
        while is_oneAction:
            get_one_err = 0
            get_stop_err = 0
            # 输入单步命令
            if keyboard.is_pressed('w') or keyboard.is_pressed('8'):  # 向前移动，刮板和水系统停止
                print('前进')
                get_one_err = single_action(hex_moveFront, q_c, q_ci, file_address)
            elif keyboard.is_pressed('s') or keyboard.is_pressed('2'):  # 向后移动，刮板和水系统停止
                print('后退')
                get_one_err = single_action(hex_moveBack, q_c, q_ci, file_address)
            elif keyboard.is_pressed('a') or keyboard.is_pressed('4'):  # 向左旋转，刮板和水系统停止
                print('左旋')
                get_one_err = single_action(hex_rotateLeft, q_c, q_ci, file_address)
            elif keyboard.is_pressed('d') or keyboard.is_pressed('6'):  # 向右旋转，刮板和水系统停止
                print('右旋')
                get_one_err = single_action(hex_rotateRight, q_c, q_ci, file_address)
            elif keyboard.is_pressed('q'):  # 结束运行，全部停止
                print('结束运行')
                get_stop_err = single_action(hex_allStop, q_c, q_ci, file_address)
                is_oneAction = False
            else:  # 无输入，停止移动，刮板和水系统停止
                # print('停止')
                get_stop_err = single_action(hex_allStop, q_c, q_ci, file_address)
            # 无反馈退出
            if get_one_err == 98 or get_stop_err == 98:
                print('无反馈，结束运行')
                is_oneAction = False
                no_feedBack = True
        if no_feedBack:
            continue

        glo_is_init = False
        # 获取偏航角
        if not q_i.empty():
            imu_list = q_i.get()
            imu_roll = imu_list[0]
            imu_pitch = imu_list[1]
            imu_yaw = -imu_list[2]
        # 如果角度有偏航，进行校正
        if abs(imu_yaw) > 1.0:
            print('校正偏航')
            get_state = correct_yaw(imu_yaw, 0.0, board_cmd, q_i, q_d, q_c, q_ci, file_address)
            # 无反馈退出
            if get_state == 98:
                print('无反馈，结束运行')
                no_feedBack = True
            print('校正完成')
        else:
            print('无偏航')
        if no_feedBack:
            continue
        # 获取视觉数据
        if not q_d.empty():
            dis_list = q_d.get()
            side_f = dis_list[0]
            side_l = dis_list[1]
            side_r = dis_list[2]
            cam_yaw = dis_list[3]
        else:
            cv2.waitKey(100)
            if not q_d.empty():
                dis_list = q_d.get()
                side_f = dis_list[0]
                side_l = dis_list[1]
                side_r = dis_list[2]
                cam_yaw = dis_list[3]
        # 如果距离轨迹有偏移，进行校正
        if side_l != -999.9:
            if abs(side_l - 100) > 10:
                print('校正偏移')
                get_state = correct_deviation(side_l, 100.0, board_cmd, q_i, q_d, q_c, q_ci, file_address)
                # 无反馈退出
                if get_state == 98:
                    print('无反馈，结束运行')
                    no_feedBack = True
                print('校正完成')
            else:
                print('无偏移')
        else:
            print('未找到左边界')
        if no_feedBack:
            continue
        # # 进入自动控制
        # print('进入自动控制')
        # get_state = 0
        # # 建立通信，开始执行
        # for loop_num in range(0, loop_time, 1):
        #     # 清洗前进
        #     print('清洗前行，次数', str(wash_loops))
        #     get_state = go_wash(True, cmd_2_washSpeed, board_cmd, q_i, q_c, q_s, q_d, lock_ser, file_address, wash_loops)
        #     if get_state == 98:
        #         no_feedBack = True
        #         break
        #     elif get_state == 2:
        #         print('后向误判')
        #     elif get_state > 90:
        #         print('故障')
        #
        #     # 清洗后退
        #     print('清洗后退，次数', str(wash_loops))
        #     get_state = go_wash(False, cmd_2_washSpeed, board_cmd, q_i, q_c, q_s, q_d, lock_ser, file_address, wash_loops)
        #     if get_state == 98:
        #         no_feedBack = True
        #         break
        #     elif get_state == 1:
        #         print('前向误判')
        #     elif get_state > 90:
        #         print('故障')
        #
        #     print('循环剩余', str(loop_time - loop_num))
        # if no_feedBack:
        #     continue


if __name__ == '__main__':
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
    # 单目相机测距，相机进程图像、视觉偏航角（给IMU和给计算进程）、标注后图像、前水平线、左垂直线、右垂直线
    queue_camera = mp.Queue(maxsize=2)
    queue_yaw_imu = mp.Queue(maxsize=2)
    queue_yaw_main = mp.Queue(maxsize=2)
    queue_image = mp.Queue(maxsize=2)
    queue_front = mp.Queue(maxsize=2)
    queue_left = mp.Queue(maxsize=2)
    queue_right = mp.Queue(maxsize=2)
    queue_temp = mp.Queue(maxsize=2)
    # IMU，偏航、俯仰和翻滚，分发前后相机、左右计算和自动控制
    queue_imu_dis = mp.Queue(maxsize=2)
    queue_imu_main = mp.Queue(maxsize=2)
    queue_imu_auto = mp.Queue(maxsize=2)
    # 自动控制交互，计算给自控距离数据，自控给计算控制命令，自控给IMU控制命令
    queue_distance = mp.Queue(maxsize=2)
    queue_control = mp.Queue(maxsize=2)
    queue_control_imu = mp.Queue(maxsize=2)

    # 单目视觉测距
    processes.append(mp.Process(target=image_put, args=(queue_camera, camera_id)))
    processes.append(
        mp.Process(target=distance_get, args=(
            queue_camera, queue_imu_dis, queue_yaw_imu, queue_yaw_main, queue_image, queue_front, queue_left,
            queue_right, lock, camera_id, str_fileAddress, queue_temp)))
    # IMU测姿态
    processes.append(
        mp.Process(target=imu_get, args=(
            queue_imu_auto, queue_imu_dis, queue_imu_main, queue_yaw_imu, queue_control_imu, lock, str_fileAddress)))
    # 视觉感知融合
    processes.append(
        mp.Process(target=multi_calc, args=(
            queue_image, queue_front, queue_left, queue_right, queue_yaw_main, queue_imu_main, queue_distance, queue_control,
            str_fileAddress, queue_temp)))
    # 自动运行
    processes.append(
        mp.Process(target=autocontrol_run, args=(
            queue_imu_auto, queue_distance, queue_control, queue_control_imu, lock, str_fileAddress)))

    for process in processes:
        process.daemon = True
        process.start()
    for process in processes:
        process.join()
