# import queue
import cv2
import time
import datetime
import multiprocessing as mp
import os
import CVFunc
import INA219
import sys
import signal
import serial
import binascii
import smbus
import MPU6050

se = serial.Serial('/dev/ttyTHS1', 9600, timeout=0.15)

# 读取模型参数
file_model = open('Model.txt', 'r', encoding='utf-8')
para_lines = file_model.readlines()
file_model.close()


# 确定摄像头数量和编号，提取从0-6
def get_camera_id():
    list_ID = [int] * 7
    int_num = 0
    for k in range(0, 7, 1):
        cap_test = cv2.VideoCapture(k)
        if cap_test.isOpened():
            list_ID[int_num] = k
            int_num += 1
            cap_test.release()
    return list_ID, int_num


# 返回摄像头格式
def get_camera_data(cap, num):
    str_Info = 'C' + str(num) + ':' + str(cap.get(3)) + '*' + str(cap.get(4)) + '; FPS' + str(cap.get(5)) + '\n'
    str_fourcc = str("".join([chr((int(cap.get(cv2.CAP_PROP_FOURCC)) >> 8 * k) & 0xFF) for k in range(4)])) + '\n'
    str_Info = str_Info + str_fourcc
    return str_Info


# 抓取图片，确认视频流的读入
def image_put(q, c_id, file_address):
    cap = cv2.VideoCapture(c_id)
    cap.set(6, 1196444237)
    cap.set(3, 1920)
    cap.set(4, 1080)
    cap.set(5, 30)
    cap.set(11, 80)
    cap.set(12, 80)
    # 获取视频帧率
    print(get_camera_data(cap, c_id))
    if cap.isOpened():
        print('Get1', c_id)
    else:
        cap = cv2.VideoCapture(c_id)
        cap.set(6, 1196444237)
        cap.set(3, 1920)
        cap.set(4, 1080)
        cap.set(5, 30)
        print(get_camera_data(cap, c_id))
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


# 获得视频流帧数图片，调用测距
def distance_get(q_c, q_s, q_u, lock_ser, cap_id, file_address):
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

    loop_num = 0
    while True:
        loop_num += 1
        str_Time = datetime.datetime.now().strftime('%H%M%S-%f')

        ret_mess = ''  # 数据信息
        err_mess = ''  # 报错信息
        time_mess = ''  # 时间信息
        ret_value = [0] * 4  # 0是水平线，1是左垂线，2是右垂线，3是超声波

        # 获取图像及参数
        rgb_frame = q_c.get()
        img_height = int(rgb_frame.shape[0])
        img_width = int(rgb_frame.shape[1])
        mid_height = int(img_height / 2)
        mid_width = int(img_width / 2)

        # Canny提取边界，保留下半部分
        start_time = time.time()
        # gra_edge = CVFunc.find_edge_light(rgb_frame)
        if cap_id == 0:
            minCan = 20
            maxCan = 50
        else:
            minCan = 40
            maxCan = 100
        gra_edge = cv2.Canny(rgb_frame, minCan, maxCan)
        rgb_rot = rgb_frame.copy()
        gra_edge_rot = gra_edge.copy()
        gra_edge_rot[0:principal_y, :] = 0
        end_time = time.time()
        time_mess += 'Can:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

        # 获取水平和垂直线
        start_time = time.time()
        hor_lines_points, left_lines, right_lines, err_mess_hough = CVFunc.get_HoughLinesFLR(gra_edge_rot, principal_x,
                                                                                             principal_y)
        err_mess += err_mess_hough
        end_time = time.time()
        time_mess += 'Hou:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

        start_time = time.time()
        # 画相机中心十字
        cv2.line(rgb_rot, (principal_x, 0), (principal_x, img_height), (255, 255, 0), 1)
        cv2.line(rgb_rot, (0, principal_y), (img_width, principal_y), (255, 255, 0), 1)
        cv2.circle(rgb_rot, (principal_x, principal_y), 5, (255, 255, 0), 3)

        # 读取超声数据，反推图像位置，画出水平线
        ultra_value = q_u.get()
        if ultra_value > 0 and (ultra_value - model_b) != 0:
            height_ultra = int(((model_F * model_W) / ultra_value - model_b) / model_a)
            cv2.line(rgb_rot, (0, height_ultra), (img_width, height_ultra), (255, 255, 255), 1)
            cv2.putText(rgb_rot, str(ultra_value), (mid_width, height_ultra), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 1)
        ret_value[3] = int(ultra_value)
        # 计算水平线到相机的距离，输出最近距离，并画出水平线
        dis_temp = 9999.0
        height_edge = - model_b / model_a
        axis_temp_h = 0
        try:
            for hor_point in hor_lines_points:
                if float(hor_point[0]) > height_edge:
                    dis_hor = CVFunc.calc_horizontal(hor_point[0], model_F, model_W, model_a, model_b)
                    dis_hor = round(dis_hor, 2)
                    if dis_hor < dis_temp:
                        dis_temp = dis_hor
                        axis_temp_h = hor_point[0]
            cv2.line(rgb_rot, (0, axis_temp_h), (img_width, axis_temp_h), (255, 0, 0), 1)
            cv2.putText(rgb_rot, str(dis_temp), (mid_width, axis_temp_h), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 0, 0), 1)
        except Exception as e:
            err_mess += 'calc horizontal line\n' + str(e) + '\n'

        nearest_hor = int(dis_temp)
        # if 0 < ultra_value < nearest_hor:
        #     nearest_hor = ultra_value
        ret_value[0] = nearest_hor
        ret_mess += 'F,' + str(nearest_hor) + '\n'

        # 计算左垂直线到图像中轴的距离
        dis_temp_l = 9999.0
        ver_left = [[0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0]]  # 平均a，平均b，平均dis，累加权重，最小dis，最小x1，最小y1，最小x2，最小y2
        for l_line in left_lines:
            for x1, y1, x2, y2 in l_line:
                if float(y1) > height_edge and float(y2) > height_edge and min(y1, y2) > (
                        principal_y + 50):  # 剔除过于接近中线的数据
                    dis_ver_1 = CVFunc.calc_vertical(abs(x1 - principal_x), y1, model_F, model_W, model_a, model_b)
                    dis_ver_2 = CVFunc.calc_vertical(abs(x2 - principal_x), y2, model_F, model_W, model_a, model_b)
                    x1_f = float(x1)
                    x2_f = float(x2)
                    y1_f = float(y1)
                    y2_f = float(y2)
                    func_a = (y1_f - y2_f) / (x1_f - x2_f)
                    func_b = y1_f - func_a * x1_f
                    weight_ver = CVFunc.getDist_P2P(x1, y1, x2, y2)
                    avg_dis = (dis_ver_1 + dis_ver_2) / 2
                    temp_x = int((principal_y - func_b) / func_a)
                    if temp_x < (principal_x + 50):
                        if ver_left[0][0] == 0.0:
                            ver_left[0][0] = func_a
                            ver_left[0][1] = func_b
                            ver_left[0][2] = avg_dis
                            ver_left[0][3] = weight_ver
                            ver_left[0][4] = min(dis_ver_1, dis_ver_2)
                            ver_left[0][5] = x1
                            ver_left[0][6] = y1
                            ver_left[0][7] = x2
                            ver_left[0][8] = y2
                        else:
                            bool_newline = True
                            for i in range(0, len(ver_left), 1):
                                temp_a = ver_left[i][0]
                                temp_b = ver_left[i][1]
                                temp_dis = ver_left[i][2]
                                temp_weight = ver_left[i][3]
                                if abs(func_a - temp_a) <= 0.05 or abs(
                                        temp_dis - avg_dis) <= 10.0:  # 如果新线段与已有线段的斜率相近，或者距离相近，则累加进已有线段
                                    sum_weight = temp_weight + weight_ver
                                    ver_left[i][0] = (temp_a * temp_weight + func_a * weight_ver) / sum_weight
                                    ver_left[i][1] = (temp_b * temp_weight + func_b * weight_ver) / sum_weight
                                    ver_left[i][2] = (temp_dis * temp_weight + avg_dis * weight_ver) / sum_weight
                                    ver_left[i][3] = sum_weight
                                    if ver_left[i][4] > min(dis_ver_1, dis_ver_2):
                                        ver_left[i][4] = min(dis_ver_1, dis_ver_2)
                                        ver_left[i][5] = x1
                                        ver_left[i][6] = y1
                                        ver_left[i][7] = x2
                                        ver_left[i][8] = y2
                                    bool_newline = False
                                    break
                            if bool_newline:
                                temp_list = [func_a, func_b, avg_dis, weight_ver, min(dis_ver_1, dis_ver_2),
                                             x1, y1, x2, y2]
                                ver_left.append(temp_list)
        if abs(ver_left[0][0]) > 0.0:
            for ver_list in ver_left:
                temp_a = ver_list[0]
                temp_b = ver_list[1]
                temp_dis = ver_list[2]
                temp_weight = ver_list[3]
                y_m = principal_y
                y_b = img_height
                x_m = int((y_m - temp_b) / temp_a)
                x_b = int((y_b - temp_b) / temp_a)
                cv2.line(rgb_rot, (x_m, y_m), (x_b, y_b), (0, 255, 0), 1)
                cv2.line(rgb_rot, (ver_list[5], ver_list[6]), (ver_list[7], ver_list[8]), (0, 0, 255), 1)
                cv2.putText(rgb_rot, str(ver_list[4]) + '-' + str(round(temp_dis, 0)),
                            (int((ver_list[5] + ver_list[7]) / 2), int((ver_list[6] + ver_list[8]) / 2)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
                if dis_temp_l > temp_dis:
                    dis_temp_l = temp_dis
        else:
            dis_temp_l = 9999.0
        ret_value[1] = int(dis_temp_l)
        ret_mess += 'L,' + str(dis_temp_l) + '\n'

        # 计算右垂直线到图像中轴的距离
        dis_temp_r = 9999.0
        ver_right = [[0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0]]  # 平均a，平均b，平均dis，累加权重，最小dis，最小x1，最小y1，最小x2，最小y2
        for r_line in right_lines:
            for x1, y1, x2, y2 in r_line:
                if float(y1) > height_edge and float(y2) > height_edge:
                    dis_ver_1 = CVFunc.calc_vertical(abs(x1 - principal_x), y1, model_F, model_W, model_a, model_b)
                    dis_ver_2 = CVFunc.calc_vertical(abs(x2 - principal_x), y2, model_F, model_W, model_a, model_b)
                    x1_f = float(x1)
                    x2_f = float(x2)
                    y1_f = float(y1)
                    y2_f = float(y2)
                    func_a = (y1_f - y2_f) / (x1_f - x2_f)
                    func_b = y1_f - func_a * x1_f
                    weight_ver = CVFunc.getDist_P2P(x1, y1, x2, y2)
                    avg_dis = (dis_ver_1 + dis_ver_2) / 2
                    temp_x = int((principal_y - func_b) / func_a)
                    if temp_x > (principal_x - 50):
                        if ver_right[0][0] == 0.0:
                            ver_right[0][0] = func_a
                            ver_right[0][1] = func_b
                            ver_right[0][2] = avg_dis
                            ver_right[0][3] = weight_ver
                            ver_right[0][4] = min(dis_ver_1, dis_ver_2)
                            ver_right[0][5] = x1
                            ver_right[0][6] = y1
                            ver_right[0][7] = x2
                            ver_right[0][8] = y2
                        else:
                            bool_newline = True
                            for i in range(0, len(ver_right), 1):
                                temp_a = ver_right[i][0]
                                temp_b = ver_right[i][1]
                                temp_dis = ver_right[i][2]
                                temp_weight = ver_right[i][3]
                                if abs(func_a - temp_a) <= 0.05 or abs(
                                        temp_dis - avg_dis) <= 10.0:  # 如果新线段与已有线段的斜率相近，或者距离相近，则累加进已有线段
                                    sum_weight = temp_weight + weight_ver
                                    ver_right[i][0] = (temp_a * temp_weight + func_a * weight_ver) / sum_weight
                                    ver_right[i][1] = (temp_b * temp_weight + func_b * weight_ver) / sum_weight
                                    ver_right[i][2] = (temp_dis * temp_weight + avg_dis * weight_ver) / sum_weight
                                    ver_right[i][3] = sum_weight
                                    if ver_right[i][4] > min(dis_ver_1, dis_ver_2):
                                        ver_right[i][4] = min(dis_ver_1, dis_ver_2)
                                        ver_right[i][5] = x1
                                        ver_right[i][6] = y1
                                        ver_right[i][7] = x2
                                        ver_right[i][8] = y2
                                    bool_newline = False
                                    break
                            if bool_newline:
                                temp_list = [func_a, func_b, avg_dis, weight_ver, min(dis_ver_1, dis_ver_2),
                                             x1, y1, x2, y2]
                                ver_right.append(temp_list)
        if abs(ver_right[0][0]) > 0.0:
            for ver_list in ver_right:
                temp_a = ver_list[0]
                temp_b = ver_list[1]
                temp_dis = ver_list[2]
                temp_weight = ver_list[3]
                y_m = principal_y
                y_b = img_height
                x_m = int((y_m - temp_b) / temp_a)
                x_b = int((y_b - temp_b) / temp_a)
                cv2.line(rgb_rot, (x_m, y_m), (x_b, y_b), (0, 255, 0), 1)
                cv2.line(rgb_rot, (ver_list[5], ver_list[6]), (ver_list[7], ver_list[8]), (0, 0, 255), 1)
                cv2.putText(rgb_rot, str(ver_list[4]) + '-' + str(round(temp_dis, 0)),
                            (int((ver_list[5] + ver_list[7]) / 2), int((ver_list[6] + ver_list[8]) / 2)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
                if dis_temp_r > temp_dis:
                    dis_temp_r = temp_dis
        else:
            dis_temp_r = 9999.0
        ret_value[2] = int(dis_temp_r)
        ret_mess += 'R,' + str(dis_temp_r) + '\n'
        end_time = time.time()
        time_mess += 'Cal:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

        # 显示及保存图片
        # cv2.imshow('Cap', frame_distance)
        # cv2.imwrite(file_address + 'C' + str(cap_id) + '-' + str_Time + '.jpg', rgb_frame)
        # cv2.imwrite(file_address + 'D' + str(cap_id) + '-' + str_Time + '.jpg', frame_distance)
        # 保存txt
        file_rec = open(file_address + str(cap_id) + '.txt', 'a')
        file_rec.write(str_Time + '  ' + str(loop_num) + '\n')
        if len(ret_mess) > 0:
            file_rec.write('Data:\n' + ret_mess)
        if len(err_mess) > 0:
            file_rec.write('Error:\n' + err_mess)
        if len(time_mess) > 0:
            file_rec.write('Timer:\n' + time_mess)
        file_rec.close()
        q_s.put(ret_value)
        if q_s.qsize() > 1:
            q_s.get()


# 读取UPS和Uart
def uart_ups_get(q0, q1, qi, lock_ser, file_address):
    global se
    loop_num = 0
    i_last = 0.0
    file_rec = open(file_address + 'UPS.txt', 'a')
    str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
    ina_mess, i_value = INA219.get_ina219_data()
    i_last = i_value
    file_rec.write(str_time + '   ' + str(loop_num) + '\n' + ina_mess)
    file_rec.close()
    front_value = 0
    rear_value = 0
    left_value = 0
    right_value = 0
    imu_temp = 0
    imu_x = 0
    imu_y = 0
    while True:
        loop_num += 1
        start_time = time.time()
        # 读取两个摄像头测距进程的数据
        get_0 = False
        get_1 = False
        se_send = ''
        rec_mess = ''
        if not qi.empty():
            imu_list = qi.get()
            imu_temp = imu_list[0]
            imu_x = imu_list[1]
            imu_y = imu_list[2]

        if not q0.empty() and not q1.empty():
            get_0 = True
            get_1 = True
            t0_list = q0.get()
            t1_list = q1.get()
            front_0 = t0_list[0]
            left_0 = t0_list[1]
            right_0 = t0_list[2]
            ultra_0 = t0_list[3]
            front_1 = t1_list[0]
            left_1 = t1_list[1]
            right_1 = t1_list[2]
            ultra_1 = t1_list[3]
            front_value = min(front_0, ultra_0)
            rear_value = min(front_1, ultra_1)
            left_value = min(left_0, right_1)
            right_value = min(right_0, left_1)
            se_send = '2 ' + 'F' + str(front_value) + 'B' + str(rear_value) + 'L' + str(left_value) + 'R' + str(
                right_value) + 'X' + str(imu_x) + 'Y' + str(imu_y)
            rec_mess = '0;' + str(front_0) + ';' + str(left_0) + ';' + str(right_0) + ';' + str(ultra_0) + ';' + str(imu_temp) + ';' + str(imu_x) + ';' + str(imu_y) + ';\n'
            rec_mess += '1;' + str(front_1) + ';' + str(left_1) + ';' + str(right_1) + ';' + str(ultra_1) + ';' + str(imu_temp) + ';' + str(imu_x) + ';' + str(imu_y) + ';\n'

        elif not q0.empty() and q1.empty():
            get_0 = True
            get_1 = False
            temp_list = q0.get()
            front_0 = temp_list[0]
            left_0 = temp_list[1]
            right_0 = temp_list[2]
            ultra_0 = temp_list[3]
            front_value = min(front_0, ultra_0)
            left_value = left_0
            right_value = right_0
            se_send = '0 ' + 'F' + str(front_value) + 'B' + str(rear_value) + 'L' + str(left_value) + 'R' + str(
                right_value) + 'X' + str(imu_x) + 'Y' + str(imu_y)
            rec_mess = '0;' + str(front_0) + ';' + str(left_0) + ';' + str(right_0) + ';' + str(ultra_0) + ';' + str(imu_temp) + ';' + str(imu_x) + ';' + str(imu_y) + ';\n'
        elif q0.empty() and not q1.empty():
            get_0 = False
            get_1 = True
            temp_list = q1.get()
            front_1 = temp_list[0]
            left_1 = temp_list[1]
            right_1 = temp_list[2]
            ultra_1 = temp_list[3]
            rear_value = min(front_1, ultra_1)
            left_value = right_1
            right_value = left_1
            se_send = '1 ' + 'F' + str(front_value) + 'B' + str(rear_value) + 'L' + str(left_value) + 'R' + str(
                right_value) + 'X' + str(imu_x) + 'Y' + str(imu_y)
            rec_mess = '1;' + str(front_1) + ';' + str(left_1) + ';' + str(right_1) + ';' + str(ultra_1) + ';' + str(imu_temp) + ';' + str(imu_x) + ';' + str(imu_y) + ';\n'
        else:
            get_0 = False
            get_1 = False
            se_send = ''
            rec_mess = ''
        # 如果有C0或C1的数据，锁定后串口传输
        if get_0 or get_1:
            # lock_ser.acquire()
            # try:
            #     se.write(se_send.encode())
            # except Exception as e:
            #     print(e)
            # finally:
            #     str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
            #     print(str_time)
            #     print(se_send)
            # lock_ser.release()
            str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
            print(str_time, se_send)
            lock_ser.acquire()
            rec_mess = str_time + ';' + rec_mess
            file_data = open(file_address + 'Sensors.txt', 'a')
            file_data.write(rec_mess)
            file_data.close()
            lock_ser.release()

        # 读取UPS
        if loop_num % 2000 == 0:
            file_rec = open(file_address + 'UPS.txt', 'a')
            str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
            ina_mess, i_value = INA219.get_ina219_data()
            file_rec.write(str_time + '   ' + '\n' + ina_mess)
            file_rec.close()
            print('I=' + str(round(i_value, 4)) + 'A')
            if (i_last - i_value) > 1.0:
                # os.system('shutdown now')
                print('shutdown', i_last)
                i_last = i_value
            else:
                i_last = i_value
        end_time = time.time()
        # print('timer', str(loop_num), str(round((end_time - start_time) * 1000, 4)))


# 串口读取超声数据
def ultra_get(q0, q1, lock_ser, file_address):
    global se
    while True:
        try:
            se.write('1'.encode())
            time.sleep(0.1)
            line = se.readline()
            lock_ser.acquire()
            if line:
                ultra_list = [0] * 2
                file_serial = open(file_address + 'Serial.txt', 'a')
                str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
                rec_hex = binascii.b2a_hex(line)
                file_serial.write(str_time + str(rec_hex) + '\n')
                file_serial.close()
                if str(rec_hex[0:2].decode()) == 'ff':
                    s1 = str(rec_hex[2:6].decode())
                    s2 = str(rec_hex[6:10].decode())
                    u0 = int(s1, 16)
                    u1 = int(s2, 16)
                    q0.put(u0)
                    q1.put(u1)
                    q0.get() if q0.qsize() > 1 else time.sleep(0.005)
                    q1.get() if q1.qsize() > 1 else time.sleep(0.005)
        except Exception as e:
            print(e)
        lock_ser.release()


# I2C读取IMU数据
def imu_get(q_i, file_address):

    while True:
        cv2.waitKey(50)
        temp_out, rot_x, rot_y, sav_mess = MPU6050.get_imu_data()
        send_list = [round(temp_out, 2), round(rot_x, 2), round(rot_y, 2)]
        file_rec = open(file_address + 'MPU.txt', 'a')
        file_rec.write(sav_mess)
        file_rec.close()
        q_i.put(send_list)
        q_i.get() if q_i.qsize() > 1 else time.sleep(0.005)


def quit_all():
    print('Exit ALL')
    os.kill(os.getpid(), signal.SIGTERM)
    # try:
    #     sys.exit()
    # except Exception as e:
    #     print(e)


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

    # 摄像头编号
    cap_list, cap_num = get_camera_id()

    mp.set_start_method(method='spawn')  # init
    processes = []
    lock = mp.Lock()
    queue_cam0 = mp.Queue(maxsize=2)
    queue_cam1 = mp.Queue(maxsize=2)
    queue_s0 = mp.Queue(maxsize=2)
    queue_s1 = mp.Queue(maxsize=2)
    queue_u0 = mp.Queue(maxsize=2)
    queue_u1 = mp.Queue(maxsize=2)
    queue_imu = mp.Queue(maxsize=2)
    if cap_num == 1:
        processes.append(
            mp.Process(target=image_put, args=(queue_cam0, cap_list[0], str_fileAddress)))
        processes.append(
            mp.Process(target=distance_get, args=(queue_cam0, queue_s0, queue_u0, lock, 0, str_fileAddress)))
    elif cap_num > 1:
        processes.append(
            mp.Process(target=image_put, args=(queue_cam0, cap_list[0], str_fileAddress)))
        processes.append(
            mp.Process(target=distance_get, args=(queue_cam0, queue_s0, queue_u0, lock, 0, str_fileAddress)))
        processes.append(
            mp.Process(target=image_put, args=(queue_cam1, cap_list[1], str_fileAddress)))
        processes.append(
            mp.Process(target=distance_get, args=(queue_cam1, queue_s1, queue_u1, lock, 1, str_fileAddress)))
    else:
        quit()

    processes.append(mp.Process(target=uart_ups_get, args=(queue_s0, queue_s1, queue_imu, lock, str_fileAddress)))
    processes.append(mp.Process(target=ultra_get, args=(queue_u0, queue_u1, lock, str_fileAddress)))
    processes.append(mp.Process(target=imu_get, args=(queue_imu, str_fileAddress)))

    for process in processes:
        process.daemon = True
        process.start()
    for process in processes:
        process.join()


if __name__ == '__main__':
    run_multi_camera()  # 调用主函数
