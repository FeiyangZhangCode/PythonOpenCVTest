import crcmod
import cv2
import time
import datetime
import multiprocessing as mp
import numpy as np
import os
import CVFunc
import INA219
import sys
import signal
import serial
import binascii
import MPU6050
import math

se = serial.Serial('/dev/ttyTHS1', 115200, timeout=0.1)

ultra_TH_F = 200  # 前向超声波阈值
ultra_TH_B = 40  # 后向超声波阈值
ultra_TH_LR = 30

cmd_0_head = 'aa 01'
cmd_1_stop = '00'
cmd_1_moveFront = '01'
cmd_1_moveBack = '02'
cmd_1_rotateLeft = '03'
cmd_1_rotateRight = '04'
cmd_2_speed0 = '00'
cmd_2_max = '64'
cmd_3_stop = '00'
cmd_3_front = '01'
cmd_3_back = '02'
cmd_4_stop = '00'
cmd_4_start = '01'

# 读取模型参数
file_model = open('Model.txt', 'r', encoding='utf-8')
para_lines = file_model.readlines()
file_model.close()


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


# 将命令更改为串口输出的16进制，并加上crc8校验
def set_order(str_order):
    hex_order = bytes.fromhex(str_order)
    crc8 = crcmod.predefined.Crc('crc-8')
    crc8.update(hex_order)
    hex_crc8 = bytes.fromhex(hex(crc8.crcValue)[2:])
    hex_order = hex_order + hex_crc8
    return hex_order


# 速度改16进制
def trans_speed(str_speed):
    int_speed = int(str_speed)
    cmd_speed = hex(int_speed)[2:]
    if int_speed > 100:
        cmd_speed = '64'
    elif int_speed < 16:
        cmd_speed = '0' + cmd_speed
    return cmd_speed


# 读取各个传感器信息，返回判断后的状态
def read_feedback(rec_mess, u_front, u_back, i_yaw):
    sensor_state = 0
    # 拆分数据包的4个防跌落
    if len(rec_mess) >= 12:
        if rec_mess[0:4] == 'aa02':
            int_fall_FL = int(rec_mess[4:6])
            int_fall_FR = int(rec_mess[6:8])
            int_fall_BL = int(rec_mess[8:10])
            int_fall_BR = int(rec_mess[10:12])
        else:
            int_fall_FL = int_fall_FR = int_fall_BL = int_fall_BR = 0
    else:
        int_fall_FL = int_fall_FR = int_fall_BL = int_fall_BR = 0
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
    elif u_front < ultra_TH_F or u_back < ultra_TH_B:  # 无跌落风险，判断超声波
        if u_front < ultra_TH_F and u_back < ultra_TH_B:
            sensor_state = 99
        elif abs(i_yaw) <= 5:
            if u_front < ultra_TH_F:
                sensor_state = 1
            elif u_back < ultra_TH_B:
                sensor_state = 2
        elif i_yaw < -5.0:
            if u_front < ultra_TH_LR:
                sensor_state = 3
            elif u_back < ultra_TH_LR:
                sensor_state = 4
        elif i_yaw > 5.0:
            if u_front < ultra_TH_LR:
                sensor_state = 4
            elif u_back < ultra_TH_LR:
                sensor_state = 3
    elif abs(i_yaw) > 5:  # 没到边沿，判断偏航
        sensor_state = 5
    return sensor_state


# 读取主板的反馈信息，返回各个传感器数据
def read_sensors(rec_mess):
    fall_FL = int(rec_mess[4:6])
    fall_FR = int(rec_mess[6:8])
    fall_BL = int(rec_mess[8:10])
    fall_BR = int(rec_mess[10:12])

    return fall_FL, fall_FR, fall_BL, fall_BR


# 读取超声波和IMU数据
def sensor_get(q_u0, q_u1, q_i0, q_i1, q_ua, q_ia, lock_ser, file_address):
    se_u = serial.Serial('/dev/ttyTHS1', 9600, timeout=0.1)
    while True:
        try:
            # 串口发出超声波采样指令
            se_u.write('1'.encode())

            # I2C采集IMU数据
            cv2.waitKey(50)
            temp_out, rot_x, rot_y, sav_mess = MPU6050.get_imu_data()
            file_rec = open(file_address + 'MPU.txt', 'a')
            file_rec.write(sav_mess)
            file_rec.close()
            str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
            send_imu = [str_time, round(temp_out, 2), round(rot_x, 2), round(rot_y, 2)]
            q_i0.put(round(rot_x, 2))
            q_i0.get() if q_i0.qsize() > 1 else time.sleep(0.005)
            q_i1.put(round(rot_x, 2))
            q_i1.get() if q_i1.qsize() > 1 else time.sleep(0.005)
            q_ia.put(send_imu)
            q_ia.get() if q_ia.qsize() > 1 else time.sleep(0.005)

            # 串口接收超声波数据
            ult_rec = se_u.readline()
            lock_ser.acquire()
            if ult_rec:
                file_serial = open(file_address + 'Ultra.txt', 'a')
                str_ult = binascii.b2a_hex(ult_rec).decode()
                if str_ult[0:2] == 'ff' and len(str_ult) > 18:
                    s1 = str_ult[2:6]
                    s2 = str_ult[6:10]
                    u0 = int(s1, 16)
                    u1 = int(s2, 16)
                    q_u0.put(u0)
                    q_u1.put(u1)
                    q_u0.get() if q_u0.qsize() > 1 else time.sleep(0.005)
                    q_u1.get() if q_u1.qsize() > 1 else time.sleep(0.005)
                    str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
                    send_ultra = [str_time, u0, u1]
                    q_ua.put(send_ultra)
                    q_ua.get() if q_ua.qsize() > 1 else time.sleep(0.005)

                    file_serial.write(str_time + ';' + str(str_ult) + ';' + str(u0) + ';' + str(u1) + ';\n')
                else:
                    file_serial.write(str_time + ';' + str(str_ult) + ';\n')
                file_serial.close()
        except Exception as e:
            print(e)
        finally:
            lock_ser.release()


# 调用相机获取图片进行测距
def distance_get(q_s, q_u, q_i, lock_ser, cap_id, file_address):
    # 根据相机编号分配模型参数
    global para_lines
    if int(cap_id) == 0:
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
    # 获取视频帧率
    print(get_camera_data(cap, cap_id))
    if cap.isOpened():
        print('Get1', cap_id)
    else:
        cap = cv2.VideoCapture(cap_id)
        cap.set(6, 1196444237)
        cap.set(3, 1920)
        cap.set(4, 1080)
        cap.set(5, 30)
        print(get_camera_data(cap, cap_id))
        print('Get2', cap_id)

    # 循环处理图像
    loop_num = 0
    imu_yaw = 0.0
    while cap.isOpened():
        start_time_total = time.time()
        str_Time = datetime.datetime.now().strftime('%H%M%S-%f')
        loop_num += 1
        ret_mess = ''  # 数据信息
        err_mess = ''  # 报错信息
        time_mess = ''  # 时间信息
        ret_value = [0.0] * 5  # 0是水平线，1是左垂线，2是右垂线，3是超声波，4是偏航角

        # 获取图像及参数
        start_time = time.time()
        ret, frame = cap.read()
        if not ret:
            cap = cv2.VideoCapture(cap_id)
            ret, frame = cap.read()
        if cap_id == 1:
            rgb_frame = cv2.rotate(frame, cv2.ROTATE_180)
        else:
            rgb_frame = frame.copy()
        img_height = int(rgb_frame.shape[0])
        img_width = int(rgb_frame.shape[1])
        mid_height = int(img_height / 2)
        mid_width = int(img_width / 2)
        rgb_rot = rgb_frame.copy()
        end_time = time.time()
        time_mess += 'Cap:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # Canny提取边界，保留下半部分
        start_time = time.time()
        rgb_half = rgb_frame.copy()
        rgb_half[0:principal_y - 50, :] = (0, 0, 0)
        # gra_edge = CVFunc.find_edge_light(rgb_frame)
        minCan = 40
        maxCan = 100
        gra_edge = cv2.Canny(rgb_half, minCan, maxCan)
        end_time = time.time()
        time_mess += 'Can:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 截取下半部分
        start_time = time.time()
        gra_edge_rot = gra_edge.copy()
        gra_edge_rot[0:principal_y, :] = 0
        end_time = time.time()
        time_mess += 'Hal:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 获取水平和垂直线
        start_time = time.time()
        lines = cv2.HoughLinesP(gra_edge_rot, rho=1.0, theta=np.pi / 180, threshold=20, minLineLength=20, maxLineGap=5)
        end_time = time.time()
        time_mess += 'Hou:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 识别线提取偏航角
        start_time = time.time()
        num_line = 0
        hor_angle_avg = 0.0
        hor_angle_weight = 0
        ver_angle_avg = 0.0
        ver_angle_weight = 0

        # 获取机身IMU的偏航角
        if not q_i.empty():
            imu_yaw = q_i.get()
        if len(lines) > 0:
            for line in lines:
                for x1_p, y1_p, x2_p, y2_p in line:
                    line_length = getDist_P2P(x1_p, y1_p, x2_p, y2_p)
                    if line_length > 50.0:
                        h1 = CVFunc.calc_horizontal(y1_p, model_F, model_W, model_a, model_b)
                        h2 = CVFunc.calc_horizontal(y2_p, model_F, model_W, model_a, model_b)
                        w1 = CVFunc.calc_vertical((x1_p - principal_x), y1_p, model_F, model_W, model_a, model_b)
                        w2 = CVFunc.calc_vertical((x2_p - principal_x), y2_p, model_F, model_W, model_a, model_b)
                        # 获取玻璃平面上的角度
                        if w1 == w2:
                            tan_line = 90
                        else:
                            tan_line = np.arctan((h1 - h2) / (w1 - w2)) * 57.29577
                        if h1 < 5000 and h2 < 5000 and w1 < 5000 and w2 < 5000 and (
                                abs(tan_line) <= 10 or abs(tan_line) >= 80):
                            num_line += 1
                            # 大于60度为左右垂直线，小于30度为水平线。后期调整为IMU获得偏航角的±10度
                            if abs(tan_line) > 80:
                                if tan_line > 0:
                                    tan_line = tan_line - 90.0
                                else:
                                    tan_line = 90.0 + tan_line
                                ver_angle_avg = (ver_angle_avg * ver_angle_weight + tan_line * line_length) / (
                                            ver_angle_weight + line_length)
                                ver_angle_weight = ver_angle_weight + line_length
                            elif abs(tan_line) < 10:
                                hor_angle_avg = (hor_angle_avg * hor_angle_weight + tan_line * line_length) / (
                                            hor_angle_weight + line_length)
                                hor_angle_weight = hor_angle_weight + line_length
                            cv2.line(rgb_rot, (x1_p, y1_p), (x2_p, y2_p), (255, 0, 0), 1)
            angle_set = ver_angle_avg
        else:
            angle_set = imu_yaw

        ret_value[4] = angle_set
        end_time = time.time()
        time_mess += 'Yaw:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 旋转校正、拉平、融合
        start_time = time.time()
        num_ver = 0
        num_hor = 0
        data_ver = np.zeros((0, 3), dtype=float)  # 垂直线数据，0是最接近中轴的w（x）值，1是最接近相机的h（y）值，2是最远离相机的h（y）值
        data_hor = np.zeros((0, 3), dtype=float)  # 水平线数据，0是最接近相机的h（y）值，1是最左侧（最小）的w（x）值，2是最右侧（最大）的w（x）值
        if len(lines) > 0:
            for line in lines:
                for x1_p, y1_p, x2_p, y2_p in line:
                    if getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 50.0:
                        # 根据偏航角进行旋转
                        h1 = CVFunc.calc_horizontal(y1_p, model_F, model_W, model_a, model_b)
                        h2 = CVFunc.calc_horizontal(y2_p, model_F, model_W, model_a, model_b)
                        w1 = CVFunc.calc_vertical((x1_p - principal_x), y1_p, model_F, model_W, model_a, model_b)
                        w2 = CVFunc.calc_vertical((x2_p - principal_x), y2_p, model_F, model_W, model_a, model_b)
                        # 获取玻璃平面上的角度
                        if w1 == w2:
                            tan_line = 90
                        else:
                            tan_line = np.arctan((h1 - h2) / (w1 - w2)) * 57.29577
                        if h1 < 5000 and h2 < 5000 and w1 < 5000 and w2 < 5000 and (
                                abs(tan_line) <= 10 or abs(tan_line) >= 80):
                            x1_imu, y1_imu = points_rotate(angle_set, w1, h1)
                            x2_imu, y2_imu = points_rotate(angle_set, w2, h2)

                            # 如果调整后不为水平线或者垂直线，进行拉平拉直
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
                            temp_show = ''
                            if y1_imu == y2_imu:  # 水平线
                                num_hor += 1
                                temp_show = 'Y' + str(round(y1_imu, 0))
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
                                temp_show = 'X' + str(round(x1_imu, 0))
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
                                temp_show = str(round(x1_imu, 0)) + ',' + str(round(y1_imu, 0)) + '\n' + str(
                                    round(x2_imu, 0)) + ',' + str(round(y2_imu, 0))
        end_time = time.time()
        time_mess += 'Rot:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 反推图像上的直线位置，找出最近的水平线和左右垂直线
        start_time = time.time()
        dis_temp_f = 9999.0
        dis_temp_r = 9999.0
        dis_temp_l = -9999.0

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
                cv2.putText(rgb_rot, str(round(values_hor[0], 0)), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 0, 0), 1)
                if dis_temp_f > values_hor[0]:
                    dis_temp_f = values_hor[0]
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
                cv2.putText(rgb_rot, str(round(values_ver[0], 0)), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 0, 255), 1)
                if values_ver[0] < 0:
                    if abs(dis_temp_l) > abs(values_ver[0]):
                        dis_temp_l = values_ver[0]
                else:
                    if dis_temp_r > values_ver[0]:
                        dis_temp_r = values_ver[0]
        ret_value[0] = dis_temp_f
        ret_value[1] = dis_temp_l * -1
        ret_value[2] = dis_temp_r
        end_time = time.time()
        time_mess += 'Dra:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 画相机中心十字，读取超声数据，反推图像位置，画出水平线
        start_time = time.time()
        cv2.line(rgb_rot, (principal_x, 0), (principal_x, img_height), (255, 255, 0), 1)
        cv2.line(rgb_rot, (0, principal_y), (img_width, principal_y), (255, 255, 0), 1)
        cv2.circle(rgb_rot, (principal_x, principal_y), 5, (255, 255, 0), 3)

        ultra_value = 0.0
        if not q_u.empty():
            ultra_value = q_u.get()
        if ultra_value > 0 and (ultra_value - model_b) != 0:
            height_ultra = int(((model_F * model_W) / ultra_value - model_b) / model_a)
            cv2.line(rgb_rot, (0, height_ultra), (img_width, height_ultra), (255, 255, 255), 1)
            cv2.putText(rgb_rot, str(ultra_value), (mid_width, height_ultra), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 1)
        ret_value[3] = ultra_value
        end_time = time.time()
        time_mess += 'Ult:' + str(round((end_time - start_time) * 1000, 4)) + ';'
        # 显示及保存图片
        # start_time = time.time()
        # rgb_show = cv2.resize(rgb_rot, (480, 270))
        # cv2.imshow('Cap' + str(cap_id), rgb_show)
        # # cv2.imwrite(file_address + 'C' + str(cap_id) + '-' + str_Time + '.jpg', rgb_frame)
        cv2.imwrite(file_address + 'D' + str(cap_id) + '-' + str_Time + '.jpg', rgb_rot)
        # end_time = time.time()
        # time_mess += 'Show:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

        end_time_total = time.time()
        time_total = str(round((end_time_total - start_time_total) * 1000, 4)) + 'ms\n'
        time_mess += 'All:' + str(round((end_time_total - start_time_total) * 1000, 4)) + ';'
        # 显示数据
        if cap_id == 0:
            print(str(int(ret_value[1])), str(int(ret_value[2])), 'F' + str(int(ret_value[3])), str(round(imu_yaw, 2)))
        else:
            print(str(int(ret_value[2])), str(int(ret_value[1])), 'B' + str(int(ret_value[3])), str(round(imu_yaw, 2)))

        # 保存txt
        ret_mess += 'all:' + str(num_line) + ';hor:' + str(num_hor) + ';ver:' + str(num_ver)
        ret_mess += ';Fro:' + str(round(dis_temp_f, 0)) + ';Lef:' + str(round(dis_temp_l, 0))
        ret_mess += ';Rig:' + str(round(dis_temp_r, 0)) + ';Ult:' + str(ultra_value)
        ret_mess += ';v_y:' + str(round(angle_set, 2)) + ';i_y:' + str(round(imu_yaw, 2))
        ret_mess += ';Tim:' + str_Time + ';\n'

        time_mess += 'Tim:' + str_Time + ';\n'

        file_rec = open(file_address + str(cap_id) + '.txt', 'a')
        if len(ret_mess) > 0:
            file_rec.write('Tpy:Date;' + ret_mess)
        # if len(err_mess) > 0:
        #     file_rec.write('Error:\n' + err_mess)
        if len(time_mess) > 0:
            file_rec.write('Tpy:Timer;' + time_mess)
        file_rec.close()
        # cv2.waitKey(1)
        q_s.put(ret_value)
        q_s.get() if q_s.qsize() > 1 else time.sleep(0.005)


# 根据动作组进行执行，list的0是动作的16进制命令，1是执行多少次
def func_action(list_action, q_u, q_i, lock_ser, file_address):
    sensor_state = 0  # 传感器状态，99是传感器异常，98是无反馈，97是运行错误

    ultra_front = 0
    ultra_back = 0
    ultra_time = ''
    imu_yaw = 0.0
    imu_pitch = 0.0
    imu_temp = 0.0
    imu_time = ''

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
                    str_send = binascii.b2a_hex(hex_action).decode('utf-8')
                    str_Time = datetime.datetime.now().strftime('%H:%M:%S.%f')
                    file_rec = open(file_address + 'Control.txt', 'a')
                    file_rec.write(str_Time + ';auto;s;' + str_send + ';\n')
                    file_rec.close()
                    # print(str_Time, 's', str_send)

                    cv2.waitKey(60)

                    # 获取偏航角
                    if not q_i.empty():
                        imu_list = q_i.get()
                        imu_time = imu_list[0]
                        imu_temp = imu_list[1]
                        imu_yaw = imu_list[2]
                        imu_pitch = imu_list[3]
                    # 获取前后超声波
                    if not q_u.empty():
                        ultra_list = q_u.get()
                        ultra_time = ultra_list[0]
                        ultra_front = ultra_list[1]
                        ultra_back = ultra_list[2]
                    print('超声波', ultra_front, ultra_back, '偏航角', imu_yaw, '机内温度', imu_temp)

                    hex_rec = se.readline()
                    if hex_rec:
                        # 收到反馈，跳出反馈循环
                        no_feedback = False
                        str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
                        if len(str_rec) >= 12:
                            if str_rec[0:4] == 'aa02':
                                int_fall_FL, int_fall_FR, int_fall_BL, int_fall_BR = read_sensors(str_rec)
                                print(int_fall_FL, int_fall_FR, int_fall_BL, int_fall_BR)
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
                    sensor_state = 97
                    return sensor_state, id_action

    return sensor_state, 0


# 多步执行动作
def multi_action(hex_action, times_action, q_u, q_i, lock_ser, file_address):
    sensor_state = 0  # 传感器状态，99是传感器异常，98是无反馈，97是运行报错
    loop_nofeedback = 0  # 累加无反馈次数
    threshold_nofeedback = 10  # 累计无反馈上限值
    ultra_front = 0
    ultra_back = 0
    ultra_time = ''
    imu_yaw = 0.0
    imu_pitch = 0.0
    imu_temp = 0.0
    imu_time = ''

    for id_action in range(0, times_action, 1):
        no_feedback = True
        while no_feedback:
            try:
                se.write(hex_action)
                str_send = binascii.b2a_hex(hex_action).decode('utf-8')
                str_Time = datetime.datetime.now().strftime('%H:%M:%S.%f')
                file_rec = open(file_address + 'Control.txt', 'a')
                file_rec.write(str_Time + ',multi,s,' + str_send + '\r\n')
                file_rec.close()
                # print(str_Time, 's', str_send)
                # 获取偏航角
                if not q_i.empty():
                    imu_list = q_i.get()
                    imu_time = imu_list[0]
                    imu_temp = imu_list[1]
                    imu_yaw = imu_list[2]
                    imu_pitch = imu_list[3]
                # 获取前后超声波
                if not q_u.empty():
                    ultra_list = q_u.get()
                    ultra_time = ultra_list[0]
                    ultra_front = ultra_list[1]
                    ultra_back = ultra_list[2]

                cv2.waitKey(40)
                hex_rec = se.readline()
                if hex_rec:
                    # 收到反馈，跳出反馈循环
                    no_feedback = False
                    str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
                    file_rec = open(file_address + 'Control.txt', 'a')
                    file_rec.write(str_Time + ',multi,r,' + str_rec + '\r\n')
                    file_rec.close()
                    sensor_state = read_feedback(str_rec, ultra_front, ultra_back, imu_yaw)
                    if sensor_state != 0 and sensor_state != 5:
                        break
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
                sensor_state = 97
                no_feedback = False
                break
        if sensor_state != 0 and sensor_state != 5:
            break
    if sensor_state == 5:
        sensor_state = 0
    return sensor_state


# 校正偏航
def correct_yaw(now_yaw, target_yaw, cmd_34, q_u, q_i, lock_ser, file_address):
    get_correct_err = 0
    correct_speed = 100
    imu_yaw = now_yaw
    rot_yaw = now_yaw - target_yaw

    loop_nofeedback = 0  # 累加无反馈次数
    threshold_nofeedback = 10  # 累计无反馈上限值

    while abs(rot_yaw) > 1.0:
        last_yaw = imu_yaw - target_yaw
        # 设置旋转动作
        cmd_2_corSpeed = trans_speed(str(correct_speed))
        if rot_yaw < 0:
            hex_correctYaw = set_order(cmd_0_head + cmd_1_rotateRight + cmd_2_corSpeed + cmd_34)
        else:
            hex_correctYaw = set_order(cmd_0_head + cmd_1_rotateLeft + cmd_2_corSpeed + cmd_34)

        # 发送动作命令
        se.write(hex_correctYaw)
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
            imu_time = imu_list[0]
            imu_temp = imu_list[1]
            imu_yaw = imu_list[2]
            imu_pitch = imu_list[3]

        # 读取主板反馈
        hex_rec = se.readline()
        if hex_rec:
            # 收到反馈
            str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
            file_rec = open(file_address + 'Control.txt', 'a')
            file_rec.write(str_Time_cy + ';CorrectYaw;r;' + str_rec + ';\n')
            file_rec.close()
            # sensor_state = read_feedback(str_rec, ultra_front, ultra_back, imu_yaw)
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
            imu_time = imu_list[0]
            imu_temp = imu_list[1]
            imu_yaw = imu_list[2]
            imu_pitch = imu_list[3]

        rot_yaw = imu_yaw - target_yaw
        if abs(rot_yaw) <= 1.0:     # 如果与目标角度相差不大于1，则认为已完成，
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
def go_wash(is_front, cmd_2, cmd_34, q_u, q_i, lock_ser, file_address):
    sensor_state = 0  # 传感器状态，99是传感器异常，98是无反馈，97是运行报错
    loop_nofeedback = 0  # 累加无反馈次数
    threshold_nofeedback = 10  # 累计无反馈上限值
    ultra_front = 0
    ultra_back = 0
    ultra_time = ''
    imu_yaw = 0.0
    imu_pitch = 0.0
    imu_temp = 0.0
    imu_time = ''

    if is_front:
        hex_wash = set_order(cmd_0_head + cmd_1_moveFront + cmd_2 + cmd_34)
    else:
        hex_wash = set_order(cmd_0_head + cmd_1_moveBack + cmd_2 + cmd_34)

    while sensor_state == 0:
        no_feedback = True
        while no_feedback:
            try:
                se.write(hex_wash)
                str_send = binascii.b2a_hex(hex_wash).decode('utf-8')
                str_Time_wash = datetime.datetime.now().strftime('%H:%M:%S.%f')
                file_rec = open(file_address + 'Control.txt', 'a')
                file_rec.write(str_Time_wash + ';wash;s;' + str_send + ';\n')
                file_rec.close()
                # print(str_Time, 's', str_send)

                # 等待串口发送
                cv2.waitKey(40)

                # 获取偏航角
                if not q_i.empty():
                    imu_list = q_i.get()
                    imu_time = imu_list[0]
                    imu_temp = imu_list[1]
                    imu_yaw = imu_list[2]
                    imu_pitch = imu_list[3]
                # 获取前后超声波
                if not q_u.empty():
                    ultra_list = q_u.get()
                    ultra_time = ultra_list[0]
                    ultra_front = ultra_list[1]
                    ultra_back = ultra_list[2]

                # 读取串口反馈
                hex_rec = se.readline()

                # 获取偏航角
                if not q_i.empty():
                    imu_list = q_i.get()
                    imu_time = imu_list[0]
                    imu_temp = imu_list[1]
                    imu_yaw = imu_list[2]
                    imu_pitch = imu_list[3]
                # 获取前后超声波
                if not q_u.empty():
                    ultra_list = q_u.get()
                    ultra_time = ultra_list[0]
                    ultra_front = ultra_list[1]
                    ultra_back = ultra_list[2]

                if hex_rec:
                    # 收到反馈，跳出反馈循环
                    no_feedback = False
                    str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
                    file_rec = open(file_address + 'Control.txt', 'a')
                    file_rec.write(str_Time_wash + ';wash;r;' + str_rec + ';\n')
                    file_rec.close()
                    sensor_state = read_feedback(str_rec, ultra_front, ultra_back, imu_yaw)
                    # 到左右边，移动远离边界
                    if 2 < sensor_state < 5:
                        # 单次停止信号
                        hex_correct = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_34)
                        se.write(hex_correct)
                        str_send = binascii.b2a_hex(hex_correct).decode('utf-8')
                        str_Time_wash = datetime.datetime.now().strftime('%H:%M:%S.%f')
                        file_rec = open(file_address + 'Control.txt', 'a')
                        file_rec.write(str_Time_wash + ';stop;s;' + str_send + '\r\n')
                        file_rec.close()
                        cv2.waitKey(40)
                        hex_rec = se.readline()
                        if hex_rec:
                            str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
                            file_rec = open(file_address + 'Control.txt', 'a')
                            file_rec.write(str_Time + ';single;r;' + str_rec + '\r\n')
                            file_rec.close()
                        # 停止移动，根据上下行和到左右边旋转角度
                        if is_front:
                            # 如果上行到左边，则右旋；如果到右边，则左旋
                            if sensor_state == 3:
                                get_correct = correct_yaw(imu_yaw, 10.0, cmd_34, q_u, q_i, lock_ser, file_address)
                                if get_correct == 98:
                                    return get_correct
                            elif sensor_state == 4:
                                get_correct = correct_yaw(imu_yaw, -10.0, cmd_34, q_u, q_i, lock_ser, file_address)
                                if get_correct == 98:
                                    return get_correct
                        else:
                            # 如果下行到左边，则左旋；如果到右边，则右旋
                            if sensor_state == 3:
                                get_correct = correct_yaw(imu_yaw, -10.0, cmd_34, q_u, q_i, lock_ser, file_address)
                                if get_correct == 98:
                                    return get_correct
                            elif sensor_state == 4:
                                get_correct = correct_yaw(imu_yaw, 10.0, cmd_34, q_u, q_i, lock_ser, file_address)
                                if get_correct == 98:
                                    return get_correct
                        # 旋转后直行，远离边界
                        get_correct = multi_action(hex_wash, 10, q_u, q_i, lock_ser, file_address)
                        if get_correct == 98:
                            return get_correct
                    # 执行远离边界或清洗偏航，偏航校正
                    if 2 < sensor_state < 6:
                        get_correct = correct_yaw(imu_yaw, 0.0, cmd_34, q_u, q_i, lock_ser, file_address)  # 偏航校正
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
                sensor_state = 97
                no_feedback = False

    return sensor_state


# 左右平移
def change_column(is_top, to_left, cmd_34, q_u, q_i, lock_ser, file_address):
    re_state = 0
    hex_moveFront = set_order(cmd_0_head + cmd_1_moveFront + cmd_2_max + cmd_34)
    hex_moveBack = set_order(cmd_0_head + cmd_1_moveBack + cmd_2_max + cmd_34)
    if is_top:  # 如果在上边
        # 后退1秒，离开上边
        move_state = multi_action(hex_moveBack, 5, q_u, q_i, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state
        # 左旋或右旋30度
        if to_left:
            move_state = correct_yaw(0.0, 30.0, cmd_34, q_u, q_i, lock_ser, file_address)
        else:
            move_state = correct_yaw(0.0, -30.0, cmd_34, q_u, q_i, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state

        # 后退2秒，斜向行驶形成横向距离
        move_state = multi_action(hex_moveBack, 10, q_u, q_i, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state

        # 旋转回垂直
        if to_left:
            move_state = correct_yaw(30.0, 0.0, cmd_34, q_u, q_i, lock_ser, file_address)
        else:
            move_state = correct_yaw(-30.0, 0.0, cmd_34, q_u, q_i, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state

        # 前进2秒，回到上边
        move_state = multi_action(hex_moveFront, 10, q_u, q_i, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state

    else:  # 如果在下边
        # 前进1秒，离开下边
        move_state = multi_action(hex_moveFront, 5, q_u, q_i, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state

        # 左旋或右旋30度
        if to_left:
            move_state = correct_yaw(0.0, -30.0, cmd_34, q_u, q_i, lock_ser, file_address)
        else:
            move_state = correct_yaw(0.0, 30.0, cmd_34, q_u, q_i, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state

        # 前进2秒，斜向行驶形成横向距离
        move_state = multi_action(hex_moveFront, 10, q_u, q_i, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state

        # 旋转回垂直
        if to_left:
            move_state = correct_yaw(-30.0, 0.0, cmd_34, q_u, q_i, lock_ser, file_address)
        else:
            move_state = correct_yaw(30.0, 0.0, cmd_34, q_u, q_i, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state

        # 后退2秒，回到下边
        move_state = multi_action(hex_moveBack, 10, q_u, q_i, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state

    return re_state


# 执行自动控制
def autocontrol_run(q_u, q_i, lock_ser, file_address):
    # 设置上下次数
    loop_time = 2  # 上行+下行算作1次
    # 设置平移方向及次数
    move_side = 0  # 0不平移，1左移，2右移
    move_times = 0  # 设置平移次数
    move_num = 0  # 累计平移次数

    # 设置清洗速度
    cmd_2_washSpeed = trans_speed('50')
    # 设置移动速度
    cmd_2_moveSpeed = trans_speed('100')

    # 全停止
    hex_allStop = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_stop + cmd_4_stop)
    # 刮板向前
    hex_boardFront = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_front + cmd_4_stop)
    # 刮板向前，启动水系统
    hex_sprayFront = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_front + cmd_4_start)
    # 清洗前进，移动停止
    hex_stopFront = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_front + cmd_4_start)
    # 刮板向前，关闭水系统
    hex_sprayStop_boardFront = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_front + cmd_4_stop)
    # 刮板向后
    hex_boardBack = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_back + cmd_4_stop)
    # 刮板向下，启动水系统
    hex_sprayBack = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_back + cmd_4_start)
    # 清洗下行，移动停止
    hex_stopBack = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_back + cmd_4_start)
    # 刮板向下，关闭水系统
    hex_sprayStop_boardBack = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_back + cmd_4_stop)

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

    ultra_front = 0
    ultra_back = 0
    ultra_time = ''
    imu_yaw = 0.0
    imu_pitch = 0.0
    imu_temp = 0.0
    imu_time = ''
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
                se.write(hex_init_send)
                file_rec = open(file_address + 'Control.txt', 'a')
                file_rec.write(str_Time_ac + ';init;s;' + str_init + ';\n')
                file_rec.close()
                # print(str_Time_ac, 'i', str_init)
                # 获取偏航角
                if not q_i.empty():
                    imu_list = q_i.get()
                    imu_time = imu_list[0]
                    imu_temp = imu_list[1]
                    imu_yaw = imu_list[2]
                    imu_pitch = imu_list[3]
                # 获取前后超声波
                if not q_u.empty():
                    ultra_list = q_u.get()
                    ultra_time = ultra_list[0]
                    ultra_front = ultra_list[1]
                    ultra_back = ultra_list[2]

                # 发送后，延时读取
                cv2.waitKey(30)

                # 获取偏航角
                if not q_i.empty():
                    imu_list = q_i.get()
                    imu_time = imu_list[0]
                    imu_temp = imu_list[1]
                    imu_yaw = imu_list[2]
                    imu_pitch = imu_list[3]
                # 获取前后超声波
                if not q_u.empty():
                    ultra_list = q_u.get()
                    ultra_time = ultra_list[0]
                    ultra_front = ultra_list[1]
                    ultra_back = ultra_list[2]
                print('超声波', str(ultra_front), str(ultra_back), '偏航角', str(imu_yaw), '机内温度', str(imu_temp))

                hex_init_rec = se.readline()
                if hex_init_rec:
                    str_init_rec = binascii.b2a_hex(hex_init_rec).decode('utf-8')
                    if len(str_init_rec) >= 12:
                        if str_init_rec[0:4] == 'aa02':
                            int_fall_FL, int_fall_FR, int_fall_BL, int_fall_BR = read_sensors(str_init_rec)
                            print(int_fall_FL, int_fall_FR, int_fall_BL, int_fall_BR)
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

        # 初始偏航校正
        print('初始偏航校正')
        # 获取偏航角
        if not q_i.empty():
            imu_list = q_i.get()
            imu_time = imu_list[0]
            imu_temp = imu_list[1]
            imu_yaw = imu_list[2]
            imu_pitch = imu_list[3]

        if abs(imu_yaw) > 5.0:
            board_cmd = cmd_3_stop + cmd_4_stop
            get_state = correct_yaw(imu_yaw, 0.0, board_cmd, q_u, q_i, lock_ser, file_address)
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
            get_state = func_action(list_prepareFront, q_u, q_i, lock_ser, file_address)
            board_cmd = cmd_3_front + cmd_4_start
            if get_state == 98:
                no_feedBack = True
                break

            # 清洗前进
            get_state = go_wash(True, cmd_2_washSpeed, q_u, q_i)
            if get_state == 98:
                no_feedBack = True
                break
            elif get_state == 2:
                print('后向误判')
            elif get_state > 90:
                print('故障')

            # 上行到上边
            get_state = func_action(list_edgeFront, q_u, q_i, lock_ser, file_address)
            board_cmd = cmd_3_stop + cmd_4_stop
            if get_state == 98:
                no_feedBack = True
                break

            # 到上边平移
            if move_side == 0:  # 直行，不平移
                pass
            elif move_side == 1:  # 上边左移
                get_err_T2L = change_column(True, True, board_cmd, q_u, q_i, lock_ser, file_address)
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
                get_err_T2R = change_column(True, False, board_cmd, q_u, q_i, lock_ser, file_address)
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
            get_state = func_action(list_prepareBack, q_u, q_i, lock_ser, file_address)
            board_cmd = cmd_3_back + cmd_4_start
            if get_state == 98:
                no_feedBack = True
                break

            # 清洗后退
            get_state = go_wash(False, cmd_2_washSpeed, board_cmd, q_u, q_i, lock_ser, file_address)
            if get_state == 98:
                no_feedBack = True
                break
            elif get_state == 1:
                print('前向误判')
            elif get_state > 90:
                print('故障')

            # 下行到下边
            get_state = func_action(list_edgeBack, q_u, q_i, lock_ser, file_address)
            board_cmd = cmd_3_stop + cmd_4_stop
            if get_state == 98:
                no_feedBack = True
                break

            # 到下边，如果不是最后一轮，则平移
            if loop_num < (loop_time - 1):
                if move_side == 0:  # 直行，不平移
                    pass
                elif move_side == 1:  # 左移
                    get_err_B2L = change_column(False, True, board_cmd, q_u, q_i, lock_ser, file_address)
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
                    get_err_B2R = change_column(False, False, board_cmd, q_u, q_i, lock_ser, file_address)
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

        if no_feedBack:
            continue


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
    queue_s0 = mp.Queue(maxsize=2)
    queue_s1 = mp.Queue(maxsize=2)
    queue_ultra_0 = mp.Queue(maxsize=2)
    queue_ultra_1 = mp.Queue(maxsize=2)
    queue_ultra_a = mp.Queue(maxsize=2)
    queue_imu_0 = mp.Queue(maxsize=2)
    queue_imu_1 = mp.Queue(maxsize=2)
    queue_imu_a = mp.Queue(maxsize=2)

    processes.append(
        mp.Process(target=distance_get, args=(queue_s0, queue_ultra_0, queue_imu_0, lock, 0, str_fileAddress)))
    processes.append(
        mp.Process(target=distance_get, args=(queue_s1, queue_ultra_1, queue_imu_1, lock, 1, str_fileAddress)))
    processes.append(
        mp.Process(target=sensor_get, args=(queue_ultra_0, queue_ultra_1, queue_imu_0, queue_imu_1, queue_ultra_a, queue_imu_a, lock, str_fileAddress)))
    processes.append(
        mp.Process(target=autocontrol_run, args=(queue_ultra_a, queue_imu_a, lock, str_fileAddress)))

    for process in processes:
        process.daemon = True
        process.start()
    for process in processes:
        process.join()
