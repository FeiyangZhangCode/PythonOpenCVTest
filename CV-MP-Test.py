# import queue
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
import smbus
import MPU6050
import JY61
import math

se = serial.Serial('/dev/ttyTHS1', 9600, timeout=0.15)

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


# 调用相机获取图片进行测距
def distance_get(q_s, q_u, q_i, q_j, lock_ser, cap_id, file_address):
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
    angle_set = 0.0

    while cap.isOpened():
        start_time_total = time.time()
        str_Time = datetime.datetime.now().strftime('%H%M%S-%f')
        loop_num += 1
        ret_mess = ''  # 数据信息
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
        lines = cv2.HoughLinesP(gra_edge_rot, rho=1.0, theta=np.pi / 180, threshold=100, minLineLength=100, maxLineGap=5)
        end_time = time.time()
        time_mess += 'Hou:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 获取偏航角
        jy_yaw = 999.0
        gy_yaw = 999.0
        gy_tmp = 0.0

        if not q_j.empty():
            jy_list = q_j.get()
            jy_yaw = jy_list[0]
        if not q_i.empty():
            gy_list = q_i.get()
            gy_yaw = gy_list[1]
            gy_tmp = gy_list[0]
        if jy_yaw != 999.0:
            angle_set = jy_yaw
        elif gy_yaw != 999.0:
            angle_set = gy_yaw

        ret_value[4] = round(angle_set, 2)
        # 旋转校正、拉平、融合
        start_time = time.time()
        num_line = 0
        num_ver = 0
        num_hor = 0
        data_ver = np.zeros((0, 3), dtype=float)  # 垂直线数据，0是最接近中轴的w（x）值，1是最接近相机的h（y）值，2是最远离相机的h（y）值
        data_hor = np.zeros((0, 3), dtype=float)  # 水平线数据，0是最接近相机的h（y）值，1是最左侧（最小）的w（x）值，2是最右侧（最大）的w（x）值
        try:
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
            ret_value[0] = round(dis_temp_f, 0)
            ret_value[1] = round((dis_temp_l * -1), 0)
            ret_value[2] = round(dis_temp_r, 0)
            end_time = time.time()
            time_mess += 'Dra:' + str(round((end_time - start_time) * 1000, 4)) + ';'
        except Exception as e:
            print(e)

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
        ret_value[3] = round(ultra_value, 0)
        end_time = time.time()
        time_mess += 'Ult:' + str(round((end_time - start_time) * 1000, 4)) + ';'
        # 显示及保存图片
        start_time = time.time()
        # rgb_show = cv2.resize(rgb_rot, (960, 540))
        # cv2.imshow('Cap' + str(cap_id), rgb_show)
        cv2.imwrite(file_address + 'C' + str(cap_id) + '-' + str_Time + '.jpg', rgb_frame)
        cv2.imwrite(file_address + 'D' + str(cap_id) + '-' + str_Time + '.jpg', rgb_rot)
        end_time = time.time()
        time_mess += 'Shw:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        end_time_total = time.time()
        # time_total = str(round((end_time_total - start_time_total) * 1000, 4)) + 'ms\n'
        time_mess += 'All:' + str(round((end_time_total - start_time_total) * 1000, 4)) + ';\n'
        # 显示数据
        if cap_id == 0:
            print('F', str(int(ret_value[1])), str(int(ret_value[2])), str(int(ret_value[3])), str(ret_value[4]))
        else:
            print('B', str(int(ret_value[2])), str(int(ret_value[1])), str(int(ret_value[3])), str(ret_value[4]))

        # 保存txt
        ret_mess += 'Tim:' + str_Time + ';all:' + str(num_line) + ';hor:' + str(num_hor) + ';ver:' + str(num_ver)
        ret_mess += ';Fro:' + str(ret_value[0]) + ';Lef:' + str(ret_value[1]) + ';Rig:' + str(ret_value[2])\
                    + ';Ult:' + str(ret_value[3]) + ';Yaw:' + str(ret_value[4]) + ';Tmp:' + str(gy_tmp)\
                    + ';iJY:' + str(jy_yaw) + ';iGY:' + str(gy_yaw) + ';\n'

        file_rec = open(file_address + str(cap_id) + '.txt', 'a')
        if len(ret_mess) > 0:
            file_rec.write('Tpy:Date;' + ret_mess)
        if len(time_mess) > 0:
            file_rec.write('Tpy:Timer;' + time_mess)
        file_rec.close()
        # cv2.waitKey(1)
        q_s.put(ret_value)
        if q_s.qsize() > 1:
            q_s.get()


# 读取超声波和IMU数据
def sensor_get(q_u0, q_u1, q_i0, q_i1, q_j0, q_j1, lock_ser, file_address):
    se_u = serial.Serial('/dev/ttyTHS1', 9600, timeout=0.02)
    se_j = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.02)
    is_u_send = True

    # 释放串口积攒的数据
    # cv2.waitKey(300)
    # release_mess = se_u.readline()
    cv2.waitKey(300)
    release_mess = se_j.readline()

    while True:
        try:
            # 串口u发出超声波采样指令
            if is_u_send:
                se_u.write('1'.encode())
                is_u_send = False

            # I2C采集GY521的IMU数据
            # cv2.waitKey(50)
            temp_out, rot_x, rot_y, sav_mess = MPU6050.get_imu_data()
            file_rec = open(file_address + 'MPU.txt', 'a')
            file_rec.write(sav_mess)
            file_rec.close()
            send_list = [round(temp_out, 2), round(rot_x, 2), round(rot_y, 2)]
            # print('GY521', str(round(rot_x, 2)))
            q_i0.put(send_list)
            q_i0.get() if q_i0.qsize() > 1 else time.sleep(0.005)
            q_i1.put(send_list)
            q_i1.get() if q_i1.qsize() > 1 else time.sleep(0.005)

            # 串口j采集JY61的IMU数据
            datahex = se_j.read(33)
            if datahex:
                jy_list = JY61.DueData(datahex)
                file_rec = open(file_address + 'JY61.txt', 'a')
                str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
                sav_mess = ("%10.2f;%10.2f;%10.2f;%10.2f;%10.2f;%10.2f;%10.2f;%10.2f;%10.2f;\n" % jy_list)
                file_rec.write(str_time + ';' + sav_mess)
                file_rec.close()
                send_list = [round(jy_list[6], 2), round(jy_list[7], 2), round(jy_list[8], 2)]
                # print(str(round(jy_list[6], 2)))
                q_j0.put(send_list)
                q_j0.get() if q_j0.qsize() > 1 else time.sleep(0.005)
                q_j1.put(send_list)
                q_j1.get() if q_j1.qsize() > 1 else time.sleep(0.005)

            # 串口u接收超声波数据
            ult_rec = se_u.readline()
            # lock_ser.acquire()
            if ult_rec:
                is_u_send = True
                file_serial = open(file_address + 'Ultra.txt', 'a')
                str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
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
                    file_serial.write(str_time + ';' + str(str_ult) + ';' + str(u0) + ';' + str(u1) + ';\n')
                    # print('Ultra', str(u0), str(u1))
                else:
                    file_serial.write(str_time + ';' + str(str_ult) + ';\n')
                file_serial.close()
        except Exception as e:
            print(e)
        # finally:
        #     lock_ser.release()


# 融合前后单目、超声和IMU，快速更新四向和偏航角
# def perception_fusion(q_c0, q_c1, q_u, q_i, q_p, lock_ser, file_address):
#     ultra_front = 0
#     ultra_back = 0
#     ultra_time = ''
#     imu_yaw = 0.0
#     imu_pitch = 0.0
#     imu_temp = 0.0
#     imu_time = ''
#     c0_front = 0.0
#     c0_left = 0.0
#     c0_right = 0.0
#     c0_yaw = 0.0
#     c1_back = 0.0
#     c1_left = 0.0
#     c1_right = 0.0
#     c1_yaw = 0.0
#
#     while True:
#         has_c0 = not q_c0.empty()
#         has_c1 = not q_c1.empty()
#         has_ultra = not q_u.empty()
#         has_imu = not q_i.empty()
#
#         # 有新IMU，直接录入
#         if has_imu:
#             imu_list = q_i.get()
#             imu_time = imu_list[0]
#             imu_temp = imu_list[1]
#             imu_yaw = imu_list[2]
#             imu_pitch = imu_list[3]
#
#         # 有新超声波，直接录入
#         if has_ultra:
#             ultra_list = q_u.get()
#             ultra_time = ultra_list[0]
#             ultra_front = ultra_list[1]
#             ultra_back = ultra_list[2]
#
#         # 有新相机0，判断情况
#         if has_c0:
#             c0_list = q_c0.get()
#             new_front = c0_list[0]
#             new_left = c0_list[1]
#             new_right = c0_list[2]
#
#             # 首轮执行，直接录入数据
#             if c0_left == c0_right == c0_front == 0.0:
#                 c0_front = new_front
#                 c0_left = new_left
#                 c0_right = new_right
#                 c0_yaw = c0_list[4]
#             else:



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
    queue_s1 = mp.Queue(maxsize=2)
    queue_ultra_0 = mp.Queue(maxsize=2)
    queue_ultra_1 = mp.Queue(maxsize=2)
    queue_imu_0 = mp.Queue(maxsize=2)
    queue_imu_1 = mp.Queue(maxsize=2)

    queue_JY61_0 = mp.Queue(maxsize=2)
    queue_JY61_1 = mp.Queue(maxsize=2)

    processes.append(
        mp.Process(target=distance_get, args=(queue_s0, queue_ultra_0, queue_imu_0, queue_JY61_0, lock, 0, str_fileAddress)))
    processes.append(
        mp.Process(target=distance_get, args=(queue_s1, queue_ultra_1, queue_imu_1, queue_JY61_1, lock, 1, str_fileAddress)))

    processes.append(
        mp.Process(target=sensor_get, args=(queue_ultra_0, queue_ultra_1, queue_imu_0, queue_imu_1, queue_JY61_0, queue_JY61_1, lock, str_fileAddress)))

    for process in processes:
        process.daemon = True
        process.start()
    for process in processes:
        process.join()


if __name__ == '__main__':
    run_multi_camera()  # 调用主函数
