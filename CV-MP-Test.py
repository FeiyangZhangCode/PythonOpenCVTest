# import queue
import cv2
import time
import datetime
import multiprocessing as mp
import numpy as np
import os
import CVFunc
# import INA219
import sys
import signal
import serial
import binascii
# import smbus
# import MPU6050
import JY61
import math
import xlwt


# 读取模型参数
file_model = open('Model.txt', 'r', encoding='utf-8')
para_lines = file_model.readlines()
file_model.close()

# 初始化Canny参数
minCan = 40
maxCan = 100

# 前后单目、IMU串口、超声波串口编号
front_id = 0
back_id = 1
imu_com = 'COM12'


# 返回摄像头格式
def get_camera_data(cap, num):
    str_Info = 'C' + str(num) + ':' + str(cap.get(3)) + '*' + str(cap.get(4)) + '; FPS' + str(cap.get(5)) + '\n'
    str_fourcc = str("".join([chr((int(cap.get(cv2.CAP_PROP_FOURCC)) >> 8 * k) & 0xFF) for k in range(4)])) + '\n'
    str_Info = str_Info + str_fourcc
    return str_Info


# 计算两点间距离
def getDist_P2P(x1_d, y1_d, x2_d, y2_d):
    distance = math.pow((x1_d - x2_d), 2) + math.pow((y1_d - y2_d), 2)
    distance = math.sqrt(distance)
    return distance


# 根据偏航角角度进行坐标旋转
def points_rotate(angle, org_x, org_y):
    cos_angle = math.cos(math.radians(angle))
    sin_angle = math.sin(math.radians(angle))
    new_x = org_x * cos_angle + org_y * sin_angle
    new_y = org_y * cos_angle - org_x * sin_angle
    return new_x, new_y


# 根据水平线距离反推图像像素高度
def calc_h2y(flo_h, f, w, a, b):
    y_back = (f * w / flo_h - b) / a
    return int(y_back)


# 根据垂直线距离反推图像像素宽度
def calc_w2x(flo_w, y, f, w, a, b, p_x):
    temp_x = abs(flo_w) * (a * y + b) / w
    if flo_w < 0:
        x_back = p_x - temp_x
    else:
        x_back = p_x + temp_x
    return int(x_back)


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
        lines = cv2.HoughLinesP(gra_edge_rot, rho=1.0, theta=np.pi / 180, threshold=100, minLineLength=100, maxLineGap=5)
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
                            if getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 50.0:
                                # 根据偏航角进行旋转
                                h1 = CVFunc.calc_horizontal(y1_p, model_F, model_W, model_a, model_b)
                                h2 = CVFunc.calc_horizontal(y2_p, model_F, model_W, model_a, model_b)
                                w1 = CVFunc.calc_vertical((x1_p - principal_x), y1_p, model_F, model_W, model_a, model_b)
                                w2 = CVFunc.calc_vertical((x2_p - principal_x), y2_p, model_F, model_W, model_a, model_b)
                                # 在2米范围内
                                if h1 < 2000 and h2 < 2000 and w1 < 2000 and w2 < 2000:
                                    num_line += 1
                                    x1_imu, y1_imu = points_rotate(angle_set, w1, h1)
                                    x2_imu, y2_imu = points_rotate(angle_set, w2, h2)

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
                                                    if abs((values_hor[2] + values_hor[1]) / 2) > abs((temp_right + temp_left) / 2):
                                                        values_hor[0] = y1_imu
                                                    values_hor[1] = min(values_hor[1], temp_left)
                                                    values_hor[2] = max(values_hor[2], temp_right)
                                                    new_hor = False
                                                    break
                                            if new_hor:
                                                data_hor = np.append(data_hor, [[y1_imu, temp_left, temp_right]], axis=0)
                                    elif x1_imu == x2_imu:      # 垂直线
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
                                                data_ver = np.append(data_ver, [[x1_imu, temp_button, temp_top]], axis=0)
                                    else:
                                        pass
                    end_time = time.time()
                    time_mess += 'Rot:' + str(round((end_time - start_time) * 1000, 4)) + ';'

                    # 反推图像上的直线位置，找出最近的水平线和左右垂直线
                    start_time = time.time()
                    if len(data_hor) > 0:
                        for values_hor in data_hor:
                            w1_b, h1_b = points_rotate(-angle_set, values_hor[1], values_hor[0])
                            w2_b, h2_b = points_rotate(-angle_set, values_hor[2], values_hor[0])
                            y1_b = calc_h2y(h1_b, model_F, model_W, model_a, model_b)
                            y2_b = calc_h2y(h2_b, model_F, model_W, model_a, model_b)
                            x1_b = calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
                            x2_b = calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
                            x_mid = int((x1_b + x2_b) / 2)
                            y_mid = int((y1_b + y2_b) / 2)
                            cv2.line(rgb_rot, (x1_b, y1_b), (x2_b, y2_b), (255, 0, 0), 1)
                            cv2.putText(rgb_rot, str(round(values_hor[0], 0)), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                        (255, 0, 0), 1)
                            cv2.line(rgb_show, (x1_b, y1_b), (x2_b, y2_b), (255, 0, 0), 6)
                            num_front += 1
                            if dis_f[0] == 0:
                                dis_f[0] = int(values_hor[0])
                            else:
                                dis_f.append(int(values_hor[0]))

                    if len(data_ver) > 0:
                        for values_ver in data_ver:
                            w1_b, h1_b = points_rotate(-angle_set, values_ver[0], values_ver[1])
                            w2_b, h2_b = points_rotate(-angle_set, values_ver[0], values_ver[2])
                            y1_b = calc_h2y(h1_b, model_F, model_W, model_a, model_b)
                            y2_b = calc_h2y(h2_b, model_F, model_W, model_a, model_b)
                            x1_b = calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
                            x2_b = calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
                            x_mid = int((x1_b + x2_b) / 2)
                            y_mid = int((y1_b + y2_b) / 2)
                            cv2.line(rgb_rot, (x1_b, y1_b), (x2_b, y2_b), (0, 255, 255), 1)
                            cv2.putText(rgb_rot, str(round(values_ver[0], 0)), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
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


# 读取超声波和IMU数据
# def sensor_get(q_u0, q_u1, q_um, q_i0, q_i1, q_im, lock_ser, file_address):
#     se_u = serial.Serial(ultra_com, 9600, timeout=0.02)
#     se_i = serial.Serial(imu_com, 9600, timeout=0.02)
#     is_u_send = True
#
#     # 释放串口积攒的数据
#     # cv2.waitKey(300)
#     # release_mess = se_u.readline()
#     # cv2.waitKey(300)
#     release_mess = se_i.readline()
#
#     print('Sensor start')
#     while True:
#         try:
#             # 串口u发出超声波采样指令
#             if is_u_send:
#                 se_u.write('1'.encode())
#                 is_u_send = False
#
#             # 串口j采集JY61的IMU数据
#             imu_rec = se_i.read(33)
#             if imu_rec:
#                 str_imu = binascii.b2a_hex(imu_rec).decode()
#                 file_rec = open(file_address + 'JY61.txt', 'a')
#                 str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
#                 if len(str_imu) == 66 and str_imu[0:4] == '5551':
#                     jy_list = JY61.DueData(imu_rec)
#                     if str(type(jy_list)) != "<class 'NoneType'>":
#                         sav_mess = ("normal;%10.2f;%10.2f;%10.2f;%10.2f;%10.2f;%10.2f;%10.2f;%10.2f;%10.2f;\n" % jy_list)
#                         send_list = [round(jy_list[6], 2), round(jy_list[7], 2), round(jy_list[8], 2)]
#                         # print(str(round(jy_list[6], 2)))
#                         q_i0.put(send_list)
#                         q_i0.get() if q_i0.qsize() > 1 else time.sleep(0.005)
#                         q_i1.put(send_list)
#                         q_i1.get() if q_i1.qsize() > 1 else time.sleep(0.005)
#                         q_im.put(send_list)
#                         q_im.get() if q_im.qsize() > 1 else time.sleep(0.005)
#                     else:
#                         sav_mess = ('NoneType;' + str_imu + ';\n')
#                         cv2.waitKey(50)
#                         release_mess = se_i.readline()
#                 else:
#                     sav_mess = ('error;' + str_imu + ';\n')
#                     cv2.waitKey(50)
#                     release_mess = se_i.readline()
#                 file_rec.write(str_time + ';' + sav_mess)
#                 file_rec.close()
#
#             # 串口u接收超声波数据
#             ult_rec = se_u.readline()
#             # lock_ser.acquire()
#             if ult_rec:
#                 is_u_send = True
#                 file_serial = open(file_address + 'Ultra.txt', 'a')
#                 str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
#                 str_ult = binascii.b2a_hex(ult_rec).decode()
#                 if str_ult[0:2] == 'ff' and len(str_ult) > 18:
#                     s1 = str_ult[2:6]
#                     s2 = str_ult[6:10]
#                     u0 = int(s1, 16)
#                     u1 = int(s2, 16)
#                     ultra_list = [u0, u1]
#                     q_u0.put(u0)
#                     q_u0.get() if q_u0.qsize() > 1 else time.sleep(0.005)
#                     q_u1.put(u1)
#                     q_u1.get() if q_u1.qsize() > 1 else time.sleep(0.005)
#                     q_um.put(ultra_list)
#                     q_um.get() if q_um.qsize() > 1 else time.sleep(0.005)
#                     file_serial.write(str_time + ';normal;' + str(u0) + ';' + str(u1) + ';\n')
#                     # print('Ultra', str(u0), str(u1))
#                 else:
#                     file_serial.write(str_time + ';error;' + str(str_ult) + ';\n')
#                 file_serial.close()
#         except Exception as e:
#             print(e)
#             print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
#             print(f"error line:{e.__traceback__.tb_lineno}")
#         # finally:
#         #     lock_ser.release()


# 读取IMU数据
def imu_get(q_i0, q_i1, q_im, lock_ser, file_address):
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
                        sav_mess = (
                                    "normal;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;\n" % jy_list)
                        send_list = [round(jy_list[6], 2), round(jy_list[7], 2), round(jy_list[8], 2)]
                        # print(str(round(jy_list[6], 2)))
                        q_i0.put(send_list)
                        q_i0.get() if q_i0.qsize() > 1 else time.sleep(0.005)
                        q_i1.put(send_list)
                        q_i1.get() if q_i1.qsize() > 1 else time.sleep(0.005)
                        q_im.put(send_list)
                        q_im.get() if q_im.qsize() > 1 else time.sleep(0.005)
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


# 读取超声波数据
# def ultra_get(q_u0, q_u1, q_um, lock_ser, file_address):
#     cv2.waitKey(1100)
#     se_u = serial.Serial(ultra_com, 9600, timeout=0.05)
#     is_u_send = True
#     print('Ultra start')
#     while True:
#         try:
#             # 串口u发出超声波采样指令
#             if is_u_send:
#                 se_u.write('1'.encode())
#                 cv2.waitKey(50)
#                 is_u_send = False
#             # 串口u接收超声波数据
#             ult_rec = se_u.readline()
#             # lock_ser.acquire()
#             if ult_rec:
#                 is_u_send = True
#                 file_serial = open(file_address + 'Ultra.txt', 'a')
#                 str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
#                 str_ult = binascii.b2a_hex(ult_rec).decode()
#                 if str_ult[0:2] == 'ff' and len(str_ult) > 18:
#                     s1 = str_ult[2:6]
#                     s2 = str_ult[6:10]
#                     u0 = int(s1, 16)
#                     u1 = int(s2, 16)
#                     ultra_list = [u0, u1]
#                     q_u0.put(u0)
#                     q_u0.get() if q_u0.qsize() > 1 else time.sleep(0.005)
#                     q_u1.put(u1)
#                     q_u1.get() if q_u1.qsize() > 1 else time.sleep(0.005)
#                     q_um.put(ultra_list)
#                     q_um.get() if q_um.qsize() > 1 else time.sleep(0.005)
#                     file_serial.write(str_time + ';normal;' + str(u0) + ';' + str(u1) + ';\n')
#                     # print('Ultra', str(u0), str(u1))
#                 else:
#                     file_serial.write(str_time + ';error;' + str(str_ult) + ';\n')
#                 file_serial.close()
#         except Exception as e:
#             print(e)
#             print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
#             print(f"error line:{e.__traceback__.tb_lineno}")


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
                                if getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 50.0:
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
                                        x1_imu, y1_imu = points_rotate(angle_set, w1, h1)
                                        x2_imu, y2_imu = points_rotate(angle_set, w2, h2)

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
                            w1_b, h1_b = points_rotate(-angle_set, values_hor[1], values_hor[0])
                            w2_b, h2_b = points_rotate(-angle_set, values_hor[2], values_hor[0])
                            y1_b = calc_h2y(h1_b, model_F, model_W, model_a, model_b)
                            y2_b = calc_h2y(h2_b, model_F, model_W, model_a, model_b)
                            x1_b = calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
                            x2_b = calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
                            x_mid = int((x1_b + x2_b) / 2)
                            y_mid = int((y1_b + y2_b) / 2)
                            cv2.line(rgb_rot, (x1_b, y1_b), (x2_b, y2_b), (255, 0, 0), 1)
                            cv2.putText(rgb_rot, str(round(values_hor[0], 0)), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.6, (255, 0, 0), 1)
                    if len(data_ver) > 0:
                        for values_ver in data_ver:
                            w1_b, h1_b = points_rotate(-angle_set, values_ver[0], values_ver[1])
                            w2_b, h2_b = points_rotate(-angle_set, values_ver[0], values_ver[2])
                            y1_b = calc_h2y(h1_b, model_F, model_W, model_a, model_b)
                            y2_b = calc_h2y(h2_b, model_F, model_W, model_a, model_b)
                            x1_b = calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
                            x2_b = calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
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
def multi_calc(q_s0, q_s1, q_f, q_b, q_lf, q_rf, q_lb, q_rb, q_i, file_address):
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

    rgb_c0 = np.zeros((540, 960, 3), np.uint8)
    rgb_c1 = np.zeros((540, 960, 3), np.uint8)

    start_time = time.time()
    end_time = time.time()
    time_mess = round((end_time - start_time) * 1000, 2)

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

        # IMU
        if not q_i.empty():
            is_imu = True
            imu_list = q_i.get()
            imu_yaw = imu_list[0]

        # 前向图像
        if not q_s0.empty():
            is_c0 = True
            rgb_c0 = q_s0.get()

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

        # 后向图像
        if not q_s1.empty():
            is_c1 = True
            rgb_c1 = q_s1.get()

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
        if is_c0 or is_c1:
            end_time = time.time()
            time_mess = round((end_time - start_time) * 1000, 4)

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
                        cam_right = int((cam_right * 2 + dis_rf[i]) / 3)
                        cam_right_f = dis_rf[i]
                        break
            elif dis_rb[0] != 0 and cam_right != 0:
                for i in range(0, len(dis_rb), 1):
                    if abs(dis_rb[i] - cam_right) < 20:
                        cam_right = int((cam_right * 2 + dis_rb[i]) / 3)
                        cam_right_b = dis_rb[i]
                        break

            show_width = 640
            show_height = 360
            half_width = int(show_width / 2)
            half_height = int(show_height / 2)
            # 调整原图尺寸便于在左上和左下展示
            rgb_show_0 = cv2.resize(rgb_c0, (show_width, show_height))
            rgb_1 = cv2.resize(rgb_c1, (show_width, show_height))
            rgb_show_1 = cv2.rotate(rgb_1, cv2.ROTATE_180)

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

            cv2.line(rgb_show_line, (0, half_height), (show_width, half_height), (0, 255, 255), 1)
            cv2.line(rgb_show_line, (half_width, 0), (half_width, show_height), (0, 255, 255), 1)
            cv2.rectangle(rgb_show_line, (half_width - 1, half_height - 1), (half_width + 1, half_height + 1), (0, 255, 255), 1)

            # 右下的数据展示区
            rgb_show_data = np.zeros((show_height, show_width, 3), np.uint8)
            str_0 = str_Time + '    ' + str(time_mess) + ' ms'
            str_1 = 'Yaw:' + str(imu_yaw) + '  Left:' + str(cam_left) + '  Right:' + str(cam_right)
            str_2 = 'L:' + str(cam_left_f) + '-' + str(cam_left_b) + '  R:' + str(cam_right_f) + '-' + str(cam_right_b)
            str_2 += '  F:' + str(cam_front) + '  B:' + str(cam_back)
            str_3 = ''
            if is_f:
                str_3 += 'nF:' + str(len(dis_f))
            else:
                str_3 += 'nF:0'
            if is_lf:
                str_3 += '  nLF:' + str(len(dis_lf))
            else:
                str_3 += '  nLF:0'
            if is_rf:
                str_3 += '  nRF:' + str(len(dis_rf))
            else:
                str_3 += '  nRF:0'
            if is_b:
                str_3 += '  nB:' + str(len(dis_b))
            else:
                str_3 += '  nB:0'
            if is_lb:
                str_3 += '  nLB:' + str(len(dis_lb))
            else:
                str_3 += '  nLB:0'
            if is_rb:
                str_3 += '  nRB:' + str(len(dis_rb))
            else:
                str_3 += '  nRB:0'
            cv2.putText(rgb_show_data, str_0, (0, int(show_height / 4)), cv2.FONT_HERSHEY_COMPLEX, 0.8, (255, 255, 255), 1)
            cv2.putText(rgb_show_data, str_1, (0, int(show_height / 2)), cv2.FONT_HERSHEY_COMPLEX, 0.8, (255, 255, 255), 1)
            cv2.putText(rgb_show_data, str_2, (0, int(show_height * 3 / 4)), cv2.FONT_HERSHEY_COMPLEX, 0.8, (255, 255, 255), 1)
            cv2.putText(rgb_show_data, str_3, (0, show_height), cv2.FONT_HERSHEY_COMPLEX, 0.8, (255, 255, 255), 1)

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


# 解决进程问题
def run_multi_camera():
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
    queue_s0 = mp.Queue(maxsize=2)
    queue_front_0 = mp.Queue(maxsize=2)
    queue_left_0 = mp.Queue(maxsize=2)
    queue_right_0 = mp.Queue(maxsize=2)

    queue_s1 = mp.Queue(maxsize=2)
    queue_back_1 = mp.Queue(maxsize=2)
    queue_left_1 = mp.Queue(maxsize=2)
    queue_right_1 = mp.Queue(maxsize=2)

    queue_imu_0 = mp.Queue(maxsize=2)
    queue_imu_1 = mp.Queue(maxsize=2)
    queue_imu_main = mp.Queue(maxsize=2)

    processes.append(
        mp.Process(target=distance_get, args=(queue_s0, queue_front_0, queue_left_0, queue_right_0, queue_imu_0, lock, front_id, str_fileAddress)))
    processes.append(
        mp.Process(target=distance_get, args=(queue_s1, queue_back_1, queue_right_1, queue_left_1, queue_imu_1, lock, back_id, str_fileAddress)))
    processes.append(
        mp.Process(target=imu_get, args=(queue_imu_0, queue_imu_1, queue_imu_main, lock, str_fileAddress)))
    processes.append(
        mp.Process(target=multi_calc, args=(queue_s0, queue_s1, queue_front_0, queue_back_1, queue_left_0, queue_right_0, queue_left_1, queue_right_1, queue_imu_main, str_fileAddress)))

    for process in processes:
        process.daemon = True
        process.start()
    for process in processes:
        process.join()


if __name__ == '__main__':

    cali_type = input('参数校准模式（0：自动，1：手动，2：默认）：')
    if cali_type == '0':
        minCan, maxCan = camera_init()
        # pass
    elif cali_type == '1':
        minCan = int(input('minCan:'))
        maxCan = int(input('maxCan:'))
    else:
        minCan = 40
        maxCan = 100

    run_multi_camera()  # 调用主函数
