import multiprocessing

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
file_model = open('./Model-360.txt', 'r', encoding='utf-8')
para_lines = file_model.readlines()
file_model.close()

# 初始化状态标志位
glo_is_init = True

# 相机编号、IMU串口信息、主板串口通信
camera_id = 0
imu_com = '/dev/ttyTHS1'
imu_baud = 9600
imu_timeout = 0.05
laser_com = '/dev/ttyUSB1'
laser_baud = 115200
laser_timeout = 0.005
se = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

# 机身尺寸、光伏板尺寸
vehicle_left = 118
vehicle_right = 127
vehicle_front = 117
vehicle_width = vehicle_left + vehicle_right
panel_width = 630

# 激光测距阈值
laser_threshold = 17

# 清洗速度及校正阈值
int_washSpeed = 50
wash_thre_yaw = 0.5
correct_thre_yaw = 30.0
wash_thre_deviation = 30.0
correct_thre_deviation = 120.0

# 侧边全局缓存
# manager = multiprocessing.Manager()
# rec_side_left = manager.list()
# rec_side_right = manager.list()
rec_side_left = [0.0]
rec_side_right = [0.0]

cmd_0_head = 'aa 01'
cmd_13_stop = '00'
cmd_13_front = '01'
cmd_13_back = '02'
cmd_24_stop = '00'
cmd_24_normal = CVFunc.trans_speed('50')
cmd_24_slow = CVFunc.trans_speed('30')
cmd_24_fast = CVFunc.trans_speed('70')
cmd_5_stop = '00'
cmd_5_front = '01'
cmd_5_back = '02'
cmd_6_stop = '00'
cmd_6_start = '01'

# 移动向前
hex_Front = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_normal + cmd_13_front + cmd_24_normal + cmd_5_stop + cmd_6_stop)
# 移动向后
hex_Back = CVFunc.set_order(cmd_0_head + cmd_13_back + cmd_24_normal + cmd_13_back + cmd_24_normal + cmd_5_stop + cmd_6_stop)
# 右旋
hex_rotateRight = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_slow + cmd_13_back + cmd_24_slow + cmd_5_stop + cmd_6_stop)
# 左旋
hex_rotateLeft = CVFunc.set_order(cmd_0_head + cmd_13_back + cmd_24_slow + cmd_13_front + cmd_24_slow + cmd_5_stop + cmd_6_stop)
# 右前转
hex_FrontRight = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_fast + cmd_13_front + cmd_24_slow + cmd_5_stop + cmd_6_stop)
# 左前转
hex_FrontLeft = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_slow + cmd_13_front + cmd_24_fast + cmd_5_stop + cmd_6_stop)
# 全停止
hex_allStop = CVFunc.set_order(cmd_0_head + cmd_13_stop + cmd_24_stop + cmd_13_stop + cmd_24_stop + cmd_5_stop + cmd_6_stop)


# 抓取图片，确认视频流的读入
def image_put(q, c_id):
    cap = cv2.VideoCapture(c_id)
    cap.set(6, 1196444237)
    cap.set(3, 640)
    cap.set(4, 360)
    cap.set(5, 30)
    if cap.isOpened():
        print('Get1', c_id)
    else:
        cap = cv2.VideoCapture(c_id)
        cap.set(6, 1196444237)
        cap.set(3, 640)
        cap.set(4, 360)
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
        rgb_show = rgb_frame.copy()
        # 获取偏航角
        # if not q_i.empty():
        #     jy_list = q_i.get()
        #     imu_roll = jy_list[0]
        #     imu_pitch = jy_list[1]
        #     imu_yaw = jy_list[2]
        end_time = time.time()
        time_mess += 'Cap:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 保留下半部分
        start_time = time.time()
        rgb_half = rgb_frame.copy()
        rgb_half[0:principal_y - 2, :] = (0, 0, 0)
        end_time = time.time()
        time_mess += 'Hal:' + str(round((end_time - start_time) * 1000, 4)) + ';'
        # 灰度并二值化
        start_time = time.time()
        gra_gray = cv2.cvtColor(rgb_half, cv2.COLOR_BGR2GRAY)
        thre_gray, gra_threshold = cv2.threshold(gra_gray, 140, 255, cv2.THRESH_BINARY)
        end_time = time.time()
        time_mess += 'Gra:' + str(round((end_time - start_time) * 1000, 4)) + ';'
        # Canny
        start_time = time.time()
        rgb_thres = cv2.cvtColor(gra_threshold, cv2.COLOR_GRAY2BGR)
        rgb_thres_hough = rgb_thres.copy()
        gra_canny = cv2.Canny(rgb_half, 100, 400)
        end_time = time.time()
        time_mess += 'Can:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 获取水平和垂直线
        start_time = time.time()
        gra_edge_rot = gra_canny.copy()
        gra_edge_rot[0:principal_y, :] = 0
        lines = cv2.HoughLinesP(gra_edge_rot, rho=1.0, theta=np.pi / 180, threshold=50, minLineLength=50,
                                maxLineGap=5)
        end_time = time.time()
        time_mess += 'Hou:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 旋转校正、拉平、融合
        gra_hough = np.zeros((img_height, img_width), np.uint8)  # 创建个全0的黑背景
        # 垂直线计算偏航角参数
        ver_avg = 0.0
        ver_weight = 0.0
        ver_sum = 0.0
        angles_ver = [0.0]
        num_yaw_l = 0
        num_yaw_r = 0
        # 水平线计算偏航角参数
        hor_avg = 0.0
        hor_weight = 0.0
        hor_sum = 0.0
        angles_hor = [0.0]

        num_ver = 0
        num_hor = 0
        # 提取浅色矩形参数
        rectangles_front = [[0.0, 0.0, 0.0, 0.0]]  # 垂直矩形，0是左边，1是右边，2是下边，3是上边
        rectangles_left = [[0.0, 0.0, 0.0, 0.0]]  # 垂直矩形，0是左边，1是右边，2是下边，3是上边
        rectangles_right = [[0.0, 0.0, 0.0, 0.0]]  # 垂直矩形，0是左边，1是右边，2是下边，3是上边
        lines_left = [[0.0, 0.0, 0.0]]  # 0是最接近中轴的w（x）值，1是最接近相机的h（y）值，2是最远离相机的h（y）值
        lines_right = [[0.0, 0.0, 0.0]]  # 0是最接近中轴的w（x）值，1是最接近相机的h（y）值，2是最远离相机的h（y）值
        num_dis_l = 0
        num_dis_r = 0
        lines_front = [[0.0, 0.0, 0.0]]  # 0是最接近相机的h（y）值，1是最左侧（最小）的w（x）值，2是最右侧（最大）的w（x）值
        num_dis_f = 0

        try:
            if lines is not None:
                start_time = time.time()
                # 初次计算偏航角
                for line in lines:
                    for x1_p, y1_p, x2_p, y2_p in line:
                        if CVFunc.getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 50.0:
                            # cv2.line(gra_hough, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                            # 根据偏航角进行旋转
                            h1 = CVFunc.calc_horizontal(y1_p, model_F, model_W, model_a, model_b)
                            h2 = CVFunc.calc_horizontal(y2_p, model_F, model_W, model_a, model_b)
                            w1 = CVFunc.calc_vertical((x1_p - principal_x), y1_p, model_F, model_W, model_a, model_b)
                            w2 = CVFunc.calc_vertical((x2_p - principal_x), y2_p, model_F, model_W, model_a, model_b)
                            # 水平距离200、垂直距离100以内
                            if (h1 < 200 or h2 < 200) and (abs(w1) < 200 or abs(w2) < 200):
                                # cv2.line(rgb_thres_hough, (x1_p, y1_p), (x2_p, y2_p), (0, 0, 255), 2)
                                if h1 != h2:
                                    angle_tan = np.arctan((w1 - w2) / (h2 - h1)) * 57.29577
                                else:
                                    angle_tan = 90.0

                                x_mid = int((x1_p + x2_p) / 2)
                                y_mid = int((y1_p + y2_p) / 2)
                                cv2.line(gra_hough, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                                cv2.putText(gra_hough, str(round(angle_tan, 2)), (x_mid, y_mid),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255, 1)

                                if abs(angle_tan) < 45:
                                    ver_sum += angle_tan
                                    if w1 < 0 and w2 < 0:
                                        num_yaw_l += 1
                                    elif w1 >= 0 and w2 >= 0:
                                        num_yaw_r += 1
                                    else:
                                        num_yaw_l += 1
                                    if num_ver == 0:
                                        num_ver = 1
                                        angles_ver[0] = angle_tan
                                    else:
                                        num_ver += 1
                                        angles_ver.append(angle_tan)
                                else:
                                    if angle_tan <= -45:
                                        angle_tan = 180 + angle_tan
                                    hor_sum += angle_tan
                                    if num_hor == 0:
                                        num_hor = 1
                                        angles_hor[0] = angle_tan
                                    else:
                                        num_hor += 1
                                        angles_hor.append(angle_tan)
                # 算数平均垂直线角度计算视觉偏航角
                if num_ver > 0:
                    temp_avg = ver_sum / num_ver
                    for angle_ver in angles_ver:
                        if abs(temp_avg - angle_ver) < 10.0:
                            ver_avg += angle_ver
                            ver_weight += 1
                    if ver_weight > 0:
                        ver_avg = ver_avg / ver_weight
                yaw_avg = ver_avg
                # 根据垂直线角度和水平线角度，输出偏航角yaw_avg，统一为偏左为负，偏右为正
                # 如果计算了偏航角，则反馈给IMU进程和计算进程
                if ver_avg != 0.0 and num_yaw_l != 0 and num_yaw_r != 0:
                    q_yi.put(yaw_avg)
                    q_yi.get() if q_yi.qsize() > 1 else time.sleep(0.005)
                    q_ym.put(round(yaw_avg, 2))
                    q_ym.get() if q_ym.qsize() > 1 else time.sleep(0.005)
                else:
                    yaw_avg = 0.0
                end_time = time.time()
                time_mess += 'Yaw:' + str(round((end_time - start_time) * 1000, 4)) + ';'

                # 如果识别到了偏航角，则进行旋转校正
                if yaw_avg != 0.0:
                    # 用均值的±10范围剔除误差数据，更新垂直线和水平线的角度。垂直线是-45至45，水平线是45至135
                    start_time = time.time()
                    for line in lines:
                        for x1_p, y1_p, x2_p, y2_p in line:
                            if CVFunc.getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 100.0:
                                h1 = CVFunc.calc_horizontal(y1_p, model_F, model_W, model_a, model_b)
                                h2 = CVFunc.calc_horizontal(y2_p, model_F, model_W, model_a, model_b)
                                w1 = CVFunc.calc_vertical((x1_p - principal_x), y1_p, model_F, model_W, model_a,
                                                          model_b)
                                w2 = CVFunc.calc_vertical((x2_p - principal_x), y2_p, model_F, model_W, model_a,
                                                          model_b)
                                # 水平距离1000、垂直距离600以内，根据偏航角进行旋转
                                if (h1 < 1000 and h2 < 1000) and (abs(w1) < 700 and abs(w2) < 700):
                                    if h1 != h2:
                                        angle_tan = np.arctan((w1 - w2) / (h2 - h1)) * 57.29577
                                    else:
                                        angle_tan = 90.0
                                    x1_imu, y1_imu = CVFunc.points_rotate(yaw_avg, w1, h1)
                                    x2_imu, y2_imu = CVFunc.points_rotate(yaw_avg, w2, h2)
                                    # 在yaw的均值±10内，认定为垂直线或水平线，拉直
                                    if abs(angle_tan) < 45.0:
                                        if abs(angle_tan - ver_avg) < 10.0:
                                            temp_value = str(x1_imu) + '-' + str(x2_imu)
                                            x_mid = int((x1_p + x2_p) / 2)
                                            y_mid = int((y1_p + y2_p) / 2)
                                            cv2.line(rgb_thres_hough, (x1_p, y1_p), (x2_p, y2_p), (255, 0, 0), 2)
                                            cv2.line(rgb_show, (x1_p, y1_p), (x2_p, y2_p), (255, 0, 0), 2)
                                            # cv2.putText(rgb_thres_hough, temp_value, (x_mid, y_mid),
                                            #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
                                            # print('Ver:' + temp_value)
                                            # 垂直线,按照最接近中轴来拉直
                                            if abs(x1_imu) > abs(x2_imu):
                                                x1_imu = x2_imu
                                            else:
                                                x2_imu = x1_imu
                                            temp_button = min(y1_imu, y2_imu)
                                            temp_top = max(y1_imu, y2_imu)
                                            # 分别存进左右线段数组
                                            if x1_imu < 0.0:
                                                if num_dis_l == 0:
                                                    lines_left[0] = [x1_imu, temp_button, temp_top]
                                                    num_dis_l = 1
                                                else:
                                                    lines_left.append([x1_imu, temp_button, temp_top])
                                                    num_dis_l += 1
                                            else:
                                                if num_dis_r == 0:
                                                    lines_right[0] = [x1_imu, temp_button, temp_top]
                                                    num_dis_r = 1
                                                else:
                                                    lines_right.append([x1_imu, temp_button, temp_top])
                                                    num_dis_r += 1
                                    # elif ((400 < h1 or h2 < 800) and abs(w1 - w2) < 200) or (
                                    #         h1 or h2 <= 400 and abs(w1 - w2) < 75):
                                    else:
                                        if angle_tan <= -45.0:
                                            angle_tan = 180.0 + angle_tan
                                        angle_tan = angle_tan - 90.0

                                        if abs(angle_tan - yaw_avg) < 10.0:
                                            temp_value = str(y1_imu) + '-' + str(y2_imu)
                                            x_mid = int((x1_p + x2_p) / 2)
                                            y_mid = int((y1_p + y2_p) / 2)
                                            cv2.line(rgb_thres_hough, (x1_p, y1_p), (x2_p, y2_p), (0, 255, 0), 2)
                                            cv2.line(rgb_show, (x1_p, y1_p), (x2_p, y2_p), (0, 255, 0), 2)
                                            # cv2.putText(rgb_thres_hough, temp_value, (x_mid, y_mid),
                                            #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                                            # print('Hor:' + temp_value)
                                            # 水平线,按照最接近相机来拉平
                                            if abs(y1_imu) > abs(y2_imu):
                                                y1_imu = y2_imu
                                            else:
                                                y2_imu = y1_imu
                                            temp_left = min(x1_imu, x2_imu)
                                            temp_right = max(x1_imu, x2_imu)
                                            # 存进前向线段数组
                                            if num_dis_f == 0:
                                                lines_front[0] = [y1_imu, temp_left, temp_right]
                                                num_dis_f = 1
                                            else:
                                                lines_front.append([y1_imu, temp_left, temp_right])
                                                num_dis_f += 1
                    end_time = time.time()
                    time_mess += 'Lin:' + str(round((end_time - start_time) * 1000, 4)) + ';'
                    start_time = time.time()
                    # 从左向右，根据左侧线段建立左侧矩形
                    if num_dis_l > 1:
                        lines_left.sort(reverse=False)
                        rectangles_left[0] = [lines_left[0][0], lines_left[0][0], lines_left[0][1], lines_left[0][2]]
                        for line_l in lines_left:
                            has_change = False
                            for rect_l in rectangles_left:
                                if 0.0 < line_l[0] - rect_l[1] <= 100.0:
                                    temp_left = rect_l[0]
                                    temp_right = line_l[0]
                                    temp_button = rect_l[2]
                                    temp_top = rect_l[3]
                                    temp_width = abs(temp_right - temp_left)
                                    temp_length = temp_top - temp_button

                                    num_list_255 = 0
                                    num_list_0 = 0
                                    w1_s = temp_left
                                    w2_s = temp_right
                                    for i in range(3, 6, 1):
                                        h_s = temp_top - temp_length * i / 6
                                        x1_s, y1_s = CVFunc.points_dis2xy(yaw_avg, w1_s, h_s, model_F, model_W,
                                                                          model_a, model_b, principal_x)
                                        x2_s, y2_s = CVFunc.points_dis2xy(yaw_avg, w2_s, h_s, model_F, model_W,
                                                                          model_a, model_b, principal_x)
                                        if y1_s >= img_height:
                                            y1_s = img_height - 1
                                        if x1_s <= 0:
                                            x1_s = 1
                                        elif x1_s >= img_width:
                                            x1_s = img_width - 1
                                        if y2_s >= img_height:
                                            y2_s = img_height - 1
                                        if x2_s <= 0:
                                            x2_s = 1
                                        elif x2_s >= img_width:
                                            x2_s = img_width - 1
                                        num_255 = 0
                                        num_0 = 0
                                        for x12_s in range(min(x1_s, x2_s), max(x1_s, x2_s), 1):
                                            if gra_threshold[y1_s, x12_s] == 255:
                                                num_255 += 1
                                            else:
                                                num_0 += 1
                                        if num_255 + num_0 > 0:
                                            if num_255 / (num_0 + num_255) > 0.8:  # 如果采样点中白色多于80%
                                                num_list_255 += 1
                                            else:
                                                num_list_0 += 1
                                    if num_list_255 > num_list_0:
                                        has_change = True
                                        rect_l[1] = line_l[0]
                                        if line_l[1] < rect_l[2]:
                                            rect_l[2] = line_l[1]
                                        if line_l[2] > rect_l[3]:
                                            rect_l[3] = line_l[2]

                                elif rect_l[1] == line_l[0]:
                                    has_change = True
                                    if line_l[1] < rect_l[2]:
                                        rect_l[2] = line_l[1]
                                    if line_l[2] > rect_l[3]:
                                        rect_l[3] = line_l[2]
                            if not has_change:
                                rectangles_left.append([line_l[0], line_l[0], line_l[1], line_l[2]])
                #     # 合并左侧相交的矩形或直线
                #     if len(rectangles_left) > 1:
                #         rectangles_left.sort(reverse=False)
                #         num_rect_l = len(rectangles_left)
                #         i = 0
                #         while i < num_rect_l - 2:
                #             j = i + 1
                #             while j < num_rect_l - 1 and j <= i + 2:
                #                 if rectangles_left[i][0] - 2 <= rectangles_left[j][0] <= rectangles_left[i][1] + 2:
                #                     rectangles_left[i][0] = min(rectangles_left[i][0], rectangles_left[j][0])
                #                     rectangles_left[i][1] = max(rectangles_left[i][1], rectangles_left[j][1])
                #                     rectangles_left[i][2] = min(rectangles_left[i][2], rectangles_left[j][2])
                #                     rectangles_left[i][3] = max(rectangles_left[i][3], rectangles_left[j][3])
                #                     del rectangles_left[j]
                #                     num_rect_l = num_rect_l - 1
                #                 else:
                #                     j += 1
                #             i += 1
                #
                    # 从右向左，根据右侧线段建立右侧矩形
                    if num_dis_r > 0:
                        lines_right.sort(reverse=True)
                        rectangles_right[0] = [lines_right[0][0], lines_right[0][0], lines_right[0][1],
                                               lines_right[0][2]]
                        for line_r in lines_right:
                            has_change = False
                            for rect_r in rectangles_right:
                                if 0.0 < rect_r[0] - line_r[0] <= 100.0:
                                    temp_left = line_r[0]
                                    temp_right = rect_r[1]
                                    temp_button = rect_r[2]
                                    temp_top = rect_r[3]
                                    temp_width = abs(temp_right - temp_left)
                                    temp_length = temp_top - temp_button

                                    num_list_255 = 0
                                    num_list_0 = 0
                                    w1_s = temp_left
                                    w2_s = temp_right
                                    for i in range(3, 6, 1):
                                        h_s = temp_top - temp_length * i / 6
                                        x1_s, y1_s = CVFunc.points_dis2xy(yaw_avg, w1_s, h_s, model_F, model_W,
                                                                          model_a, model_b, principal_x)
                                        x2_s, y2_s = CVFunc.points_dis2xy(yaw_avg, w2_s, h_s, model_F, model_W,
                                                                          model_a, model_b, principal_x)
                                        if y1_s >= img_height:
                                            y1_s = img_height - 1
                                        if x1_s <= 0:
                                            x1_s = 1
                                        elif x1_s >= img_width:
                                            x1_s = img_width - 1
                                        if y2_s >= img_height:
                                            y2_s = img_height - 1
                                        if x2_s <= 0:
                                            x2_s = 1
                                        elif x2_s >= img_width:
                                            x2_s = img_width - 1
                                        num_255 = 0
                                        num_0 = 0
                                        for x12_s in range(min(x1_s, x2_s), max(x1_s, x2_s), 1):
                                            if gra_threshold[y1_s, x12_s] == 255:
                                                num_255 += 1
                                            else:
                                                num_0 += 1
                                        if num_255 + num_0 > 0:
                                            if num_255 / (num_0 + num_255) > 0.8:  # 如果采样点中白色多于80%
                                                num_list_255 += 1
                                            else:
                                                num_list_0 += 1
                                    if num_list_255 > num_list_0:
                                        has_change = True
                                        rect_r[0] = line_r[0]
                                        rect_r[2] = min(line_r[1], rect_r[2])
                                        rect_r[3] = max(line_r[2], rect_r[3])
                                elif rect_r[0] == line_r[0]:
                                    has_change = True
                                    rect_r[2] = min(line_r[1], rect_r[2])
                                    rect_r[3] = max(line_r[2], rect_r[3])
                            if not has_change:
                                rectangles_right.append([line_r[0], line_r[0], line_r[1], line_r[2]])
                #     # 合并右侧相交的矩形或直线
                #     if len(rectangles_right) > 1:
                #         num_rect_r = len(rectangles_right)
                #         i = 0
                #         while i < num_rect_r - 2:
                #             j = i + 1
                #             while j < num_rect_r - 1 and j <= i + 2:
                #                 if rectangles_right[i][0] - 2 <= rectangles_right[j][1] <= rectangles_right[i][1] + 2:
                #                     rectangles_right[i][0] = min(rectangles_right[i][0], rectangles_right[j][0])
                #                     rectangles_right[i][1] = max(rectangles_right[i][1], rectangles_right[j][1])
                #                     rectangles_right[i][2] = min(rectangles_right[i][2], rectangles_right[j][2])
                #                     rectangles_right[i][3] = max(rectangles_right[i][3], rectangles_right[j][3])
                #                     del rectangles_right[j]
                #                     num_rect_r = num_rect_r - 1
                #                 else:
                #                     j += 1
                #             i += 1
                #
                    # 从近到远，根据最近线段建立前向矩形
                    if num_dis_f > 1:
                        lines_front.sort(reverse=False)
                        rectangles_front[0] = [lines_front[0][1], lines_front[0][2], lines_front[0][0],
                                               lines_front[0][0]]
                        for line_f in lines_front:
                            has_change = False
                            for rect_f in rectangles_front:
                                if 0.0 < line_f[0] - rect_f[3] <= 30.0:
                                    temp_left = rect_f[0]
                                    temp_right = rect_f[1]
                                    temp_button = rect_f[2]
                                    temp_top = line_f[0]
                                    temp_width = temp_right - temp_left
                                    temp_length = temp_top - temp_button

                                    num_list_255 = 0
                                    num_list_0 = 0
                                    h1_s = temp_top
                                    h2_s = temp_button
                                    for i in range(1, 4, 1):
                                        w_s = temp_left + temp_width * i / 4
                                        x1_s, y1_s = CVFunc.points_dis2xy(yaw_avg, w_s, h1_s, model_F, model_W,
                                                                          model_a, model_b, principal_x)
                                        x2_s, y2_s = CVFunc.points_dis2xy(yaw_avg, w_s, h2_s, model_F, model_W,
                                                                          model_a, model_b, principal_x)
                                        if y1_s >= img_height:
                                            y1_s = img_height - 1
                                        if x1_s <= 0:
                                            x1_s = 1
                                        elif x1_s >= img_width:
                                            x1_s = img_width - 1
                                        if y2_s >= img_height:
                                            y2_s = img_height - 1
                                        if x2_s <= 0:
                                            x2_s = 1
                                        elif x2_s >= img_width:
                                            x2_s = img_width - 1
                                        num_255 = 0
                                        num_0 = 0
                                        for y12_s in range(min(y1_s, y2_s), max(y1_s, y2_s), 1):
                                            if gra_threshold[y12_s, x1_s] == 255:
                                                num_255 += 1
                                            else:
                                                num_0 += 1
                                        if num_255 + num_0 > 0:
                                            if num_255 / (num_0 + num_255) > 0.8:  # 如果采样点中白色多于80%
                                                num_list_255 += 1
                                            else:
                                                num_list_0 += 1
                                    if num_list_255 > num_list_0:
                                        has_change = True
                                        rect_f[2] = line_f[0]
                                        rect_f[0] = min(rect_f[0], line_f[1])
                                        rect_f[1] = max(rect_f[1], line_f[2])
                                elif rect_f[2] <= line_f[0] <= rect_f[3]:
                                    has_change = True
                                    rect_f[0] = min(rect_f[0], line_f[1])
                                    rect_f[1] = max(rect_f[1], line_f[2])
                            if not has_change:
                                rectangles_front.append([line_f[1], line_f[2], line_f[0], line_f[0]])
                #     # 合并相交的矩形或直线
                #     if len(rectangles_front) > 1:
                #         num_rect_f = len(rectangles_front)
                #         i = 0
                #         while i < num_rect_f - 2:
                #             j = i + 1
                #             while j < num_rect_f - 1 and j <= i + 2:
                #                 if (rectangles_front[i][2] - 5 <= rectangles_front[j][2] <= rectangles_front[i][3] + 5) \
                #                         or (rectangles_front[i][2] - 5 <= rectangles_front[j][3] <= rectangles_front[i][
                #                     3] + 5):
                #                     rectangles_front[i][0] = min(rectangles_front[i][0], rectangles_front[j][0])
                #                     rectangles_front[i][1] = max(rectangles_front[i][1], rectangles_front[j][1])
                #                     rectangles_front[i][2] = min(rectangles_front[i][2], rectangles_front[j][2])
                #                     rectangles_front[i][3] = max(rectangles_front[i][3], rectangles_front[j][3])
                #                     del rectangles_front[j]
                #                     num_rect_f = num_rect_f - 1
                #                 else:
                #                     j += 1
                #             i += 1
                    end_time = time.time()
                    time_mess += 'Ret:' + str(round((end_time - start_time) * 1000, 4)) + ';'
        except Exception as e:
            print(e)
            print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
            print(f"error line:{e.__traceback__.tb_lineno}")
        finally:
            pass
            # print(yaw_avg)
            # cv2.imshow('canny', gra_canny)
            # cv2.imshow('thre', gra_threshold)
            # cv2.imshow('Hough', gra_hough)
            # cv2.imshow('HoughT', rgb_thres_hough)
            # cv2.waitKey(1)
        # 保存txt，传输数据
        start_time = time.time()
        if len(rectangles_left) > 1:
            q_l.put(rectangles_left)
            if q_l.qsize() > 1:
                q_l.get()
        if len(rectangles_right) > 1:
            q_r.put(rectangles_right)
            if q_r.qsize() > 1:
                q_r.get()
        if len(rectangles_front) > 1:
            q_f.put(rectangles_front)
            if q_f.qsize() > 1:
                q_f.get()
        q_img.put(rgb_show)
        if q_img.qsize() > 1:
            q_img.get()
        q_temp.put(rgb_thres_hough)
        if q_temp.qsize() > 1:
            q_temp.get()
        # cv2.imwrite(file_address + 'C' + str(cap_id) + '-' + str_Time + '.jpg', rgb_frame)
        # cv2.imwrite(file_address + 'D' + str(cap_id) + '-' + str_Time + '.jpg', rgb_rot)
        file_rec = open(file_address + 'Dis.txt', 'a')
        end_time = time.time()
        time_mess += 'Shw:' + str(round((end_time - start_time) * 1000, 4)) + ';'
        end_time_total = time.time()
        time_mess += 'All:' + str(round((end_time_total - start_time_total) * 1000, 4)) + ';\n'
        if len(time_mess) > 0:
            file_rec.write('Tpy:Timer;' + time_mess)
        file_rec.close()


# 读取IMU数据
def imu_get(q_ia, q_id, q_im, q_y, q_c, lock_ser, file_address):
    global glo_is_init

    cv2.waitKey(500)
    se_i = serial.Serial(imu_com, imu_baud, timeout=imu_timeout)
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
                # file_rec = open(file_address + 'JY61.txt', 'a')
                # str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
                if len(str_imu) == 66 and str_imu[0:4] == '5551':
                    jy_list = JY61.DueData(imu_rec)
                    if str(type(jy_list)) != "<class 'NoneType'>":
                        sav_mess = ("normal;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;\n" % jy_list)
                        send_list = [round(jy_list[6], 2), round(jy_list[7], 2), round(jy_list[8], 2)]
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
                # file_rec.write(str_time + ';' + sav_mess)
                # file_rec.close()

            if not q_y.empty() and not q_c.empty() and glo_is_init:
                yaw_c = q_y.get()
                ac_ctl = q_c.get()
                if abs(yaw_c) <= 0.5 and yaw_c != 0.00 and ac_ctl[0] == 0:
                    str_zero = 'ff aa 52'
                    hex_zero = bytes.fromhex(str_zero)
                    se_i.write(hex_zero)
                    cv2.waitKey(50)

        except Exception as e:
            print(e)
            print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
            print(f"error line:{e.__traceback__.tb_lineno}")


# 读取激光测距数据
def laser_get(q_la, q_lm, file_address):
    se_l = serial.Serial(laser_com, laser_baud, timeout=laser_timeout)
    # 释放串口积攒的数据
    se_l.flushInput()
    se_l.flushOutput()
    while True:
        try:
            # 串口采集TFminiPlus的测距数据
            laser_rec = se_l.read(18)
            if laser_rec:
                str_laser = binascii.b2a_hex(laser_rec).decode()
                # file_rec = open('./TestData/JY61.txt', 'a')
                str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
                if str_laser[0:4] == '5959':
                    hex_f_l = laser_rec[2]
                    hex_f_h = laser_rec[3]
                    hex_s_l = laser_rec[4]
                    hex_s_h = laser_rec[5]
                    laser_F = hex_f_h << 8 | hex_f_l
                    laser_S = hex_s_h << 8 | hex_s_l
                    # print(str(laser_F), str(laser_S))
                    q_la.put(laser_F)
                    q_la.get() if q_la.qsize() > 1 else time.sleep(0.005)
                    q_lm.put(laser_F)
                    q_lm.get() if q_lm.qsize() > 1 else time.sleep(0.005)
                else:
                    print('laser error:' + str_laser)
                    # se_l.flushOutput()
                # file_rec.write(str_time + ';' + sav_mess)
                # file_rec.close()

        except Exception as e:
            print(e)
            print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
            print(f"error line:{e.__traceback__.tb_lineno}")


# 融合前后单目、超声和IMU，快速更新四向和偏航角
def multi_calc(q_img, q_f, q_l, q_r, q_y, q_i, q_d, q_c, file_address, q_temp, q_las):
    cv2.waitKey(1000)
    print('calc start')
    # 根据相机编号分配模型参数
    global para_lines, rec_side_left, rec_side_right
    model_F = float(para_lines[0].strip('\n'))
    model_W = float(para_lines[1].strip('\n'))
    model_a = float(para_lines[2].strip('\n'))
    model_b = float(para_lines[3].strip('\n'))
    principal_x = int(para_lines[4].strip('\n'))
    principal_y = int(para_lines[5].strip('\n'))

    imu_yaw = 0.0
    imu_roll = 0.0
    imu_pitch = 0.0

    cam_front = 0
    cam_left = 0
    cam_right = 0
    cam_yaw = 0.0

    dis_f = [[0.0, 0.0]]
    dis_l = [[0.0, 0.0]]
    dis_r = [[0.0, 0.0]]

    ac_ctl = [0]

    dis_laser = laser_threshold

    start_time = time.time()
    end_time = time.time()
    time_mess = round((end_time - start_time) * 1000, 0)

    show_width = 640
    show_height = 360
    # show_width = 480
    # show_height = 270
    # show_width = 960
    # show_height = 540
    half_width = int(show_width / 2)
    half_height = int(show_height / 2)

    # rgb_cam = np.zeros((show_height, show_width, 3), np.uint8)

    while True:
        if not q_img.empty():
            print('Get Img')
            break
        else:
            print('No Image')
            cv2.waitKey(500)

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

        # Laser
        if not q_las.empty():
            dis_laser = q_las.get()

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
            start_time_calc = time.time()

            # 临时缓存，提取二值图
            gra_temp = q_temp.get()
            temp_height = int(gra_temp.shape[0])
            if temp_height != show_height:
                gra_threshold = cv2.resize(gra_temp, (show_width, show_height))
            else:
                gra_threshold = gra_temp.copy()
            # rgb_show_1 = cv2.cvtColor(gra_threshold, cv2.COLOR_GRAY2BGR)
            rgb_show_1 = gra_threshold.copy()

            # 左上展示图像
            is_cam = True
            rgb_tmp = q_img.get()

            # 视觉偏航角
            if not q_y.empty():
                is_y = True
                cam_yaw = q_y.get()
            else:
                cam_yaw = 0.0

            # 前向水平框
            if not q_f.empty():
                is_f = True
                rect_f = q_f.get()
                rgb_tmp, dis_f = CVFunc.draw_rectangles(False, rect_f, rgb_tmp, 15.0, cam_yaw, model_F, model_W, model_a, model_b, principal_x)
                if dis_f[0][0] != 0:
                    dis_f.sort(reverse=True)
                    side_front = dis_f[0][0]
                else:
                    side_front = -999.9
            else:
                side_front = -999.9

            # 前向左侧垂直框
            if not q_l.empty():
                is_l = True
                rect_l = q_l.get()
                rgb_tmp, dis_l = CVFunc.draw_rectangles(True, rect_l, rgb_tmp, 15.0, cam_yaw, model_F, model_W, model_a, model_b, principal_x)
                if dis_l[0][0] != 0.0:
                    dis_l.sort(reverse=False)
                    cam_left = abs(dis_l[0][1])
                    if cam_yaw >= 0.0:
                        side_left = round((cam_left - vehicle_front * math.sin(
                            math.radians(abs(cam_yaw))) - vehicle_left * math.sin(math.radians(90.0 - abs(cam_yaw)))),
                                          1)
                    else:
                        side_left = round((cam_left + vehicle_front * math.sin(
                            math.radians(abs(cam_yaw))) - vehicle_left * math.sin(math.radians(90.0 - abs(cam_yaw)))),
                                          1)
                else:
                    side_left = -999.9
            else:
                side_left = -999.9

            # 前向右侧垂直框
            if not q_r.empty():
                is_r = True
                rect_r = q_r.get()
                rgb_tmp, dis_r = CVFunc.draw_rectangles(True, rect_r, rgb_tmp, 15.0, cam_yaw, model_F, model_W, model_a, model_b, principal_x)
                if dis_r[0][0] != 0.0:
                    dis_r.sort(reverse=True)
                    cam_right = dis_r[0][0]
                    if cam_yaw >= 0:
                        side_right = round((cam_right + vehicle_front * math.sin(
                            math.radians(abs(cam_yaw))) - vehicle_right * math.sin(math.radians(90.0 - abs(cam_yaw)))), 1)
                    else:
                        side_right = round((cam_right - vehicle_front * math.sin(
                            math.radians(abs(cam_yaw))) - vehicle_right * math.sin(math.radians(90.0 - abs(cam_yaw)))), 1)
                else:
                    side_right = -999.9
            else:
                side_right = -999.9

            # 发送前向最远框的水平距离，左右侧最远框经过车宽换算后距离，视觉偏航角
            send_list = [side_front, side_left, side_right, cam_yaw]
            q_d.put(send_list)
            q_d.get() if q_d.qsize() > 1 else time.sleep(0.005)

            # 右上的距离展示区
            rgb_show_line = np.zeros((show_height, show_width, 3), np.uint8)
            if side_left != -999.9:
                temp_x = int(half_width - (half_width * (side_left + vehicle_left) / 500))
                cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (0, 0, 255), 1)
            if side_right != -999.9:
                temp_x = int(half_width + (half_width * (side_right + vehicle_right) / 500))
                cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (0, 0, 255), 1)

            if is_f and dis_f[0][0] != 0:
                dis_f.sort(reverse=True)
                for i in range(0, len(dis_f), 1):
                    if i == 0:
                        temp_y = int(show_height - (show_height * dis_f[i][0] / 1000))
                        cv2.line(rgb_show_line, (0, temp_y), (show_width, temp_y), (0, 0, 255), 1)
                        temp_y = int(show_height - (show_height * dis_f[i][1] / 1000))
                        cv2.line(rgb_show_line, (0, temp_y), (show_width, temp_y), (0, 0, 255), 1)
                    else:
                        temp_y = int(show_height - (show_height * dis_f[i][0] / 1000))
                        cv2.line(rgb_show_line, (0, temp_y), (show_width, temp_y), (255, 0, 0), 1)
                        temp_y = int(show_height - (show_height * dis_f[i][1] / 1000))
                        cv2.line(rgb_show_line, (0, temp_y), (show_width, temp_y), (255, 0, 0), 1)

            # 视觉的偏航角显示
            if is_y:
                yaw_sin = math.sin(math.radians(abs(cam_yaw)))
                yaw_cos = math.cos(math.radians(abs(cam_yaw)))
                if -90.0 <= cam_yaw < 0.0:
                    temp_x = int(half_width - yaw_sin * (half_height * 0.5))
                    temp_y = int(show_height - yaw_cos * (half_height * 0.5))
                    cv2.line(rgb_show_line, (half_width, show_height), (temp_x, temp_y), (160, 0, 240), 3)
                elif 0.0 < cam_yaw <= 90.:
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

            # 车身宽度
            temp_x = int(half_width - (half_width * vehicle_left / 500))
            cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (255, 255, 255), 1)
            temp_x = int(half_width + (half_width * vehicle_right / 500))
            cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (255, 255, 255), 1)

            # 右下的数据展示区
            rgb_show_data = np.zeros((show_height, show_width, 3), np.uint8)
            str_0 = 'Timer:' + str(time_mess)
            if is_y:
                str_0 += '  Y:' + str(cam_yaw)
            else:
                str_0 += '  Y:N/A'
            str_0 += '  Laser:' + str(dis_laser)
            if side_left != -999.9:
                str_1 = 'L:' + str(int(side_left))
            else:
                str_1 = 'L:N/A'

            for i in range(0, len(rec_side_left), 1):
                str_1 += ' ' + str(int(rec_side_left[i]))

            if is_r and dis_r[0][0] != 0.0:
                str_2 = 'R:' + str(int(side_right))
            else:
                str_2 = 'R:N/A'

            for i in range(0, len(rec_side_right), 1):
                str_2 += ' ' + str(int(rec_side_right[i]))

            # if is_f and dis_f[0][0] != 0.0:
            #     dis_f.sort(reverse=True)
            #     str_1 += '  F:' + str(int(dis_f[0][0]))
            # else:
            #     str_1 += '  F:N/A'

            if ac_ctl[0] == ac_ctl[2] == 1:
                if ac_ctl[1] == ac_ctl[3]:
                    str_3 = 'Front'
                elif ac_ctl[1] > ac_ctl[3]:
                    str_3 = 'Right'
                elif ac_ctl[1] < ac_ctl[3]:
                    str_3 = 'Left'
                else:
                    str_3 = 'error'
            elif ac_ctl[0] == ac_ctl[2] == 2:
                str_3 = 'Back'
            elif ac_ctl[0] == 1 and ac_ctl[2] == 2:
                str_3 = 'rotateRight'
            elif ac_ctl[0] == 2 and ac_ctl[2] == 1:
                str_3 = 'rotateLeft'
            elif ac_ctl[0] == ac_ctl[2] == 0:
                str_3 = 'Stop'
            else:
                str_3 = 'error'
            if ac_ctl[0] == 0:
                str_3 += ' '
            elif ac_ctl[0] == 1:
                str_3 += '+'
            elif ac_ctl[0] == 2:
                str_3 += '-'
            str_3 += str(ac_ctl[1])
            str_3 += '  '
            if ac_ctl[2] == 0:
                str_3 += ' '
            elif ac_ctl[2] == 1:
                str_3 += '+'
            elif ac_ctl[2] == 2:
                str_3 += '-'
            str_3 += str(ac_ctl[3])

            str_3  += '  Yaw:' + str(imu_yaw) + '  Roll:' + str(imu_roll) + '  Pitch:' + str(imu_pitch)

            cv2.putText(rgb_show_data, str_0, (0, int(show_height / 5) - 5), cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 255, 255),
                        1)
            cv2.putText(rgb_show_data, str_1, (0, int(show_height * 2 / 5) - 5), cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 255, 255),
                        1)
            cv2.putText(rgb_show_data, str_2, (0, int(show_height * 3 / 5) - 5), cv2.FONT_HERSHEY_COMPLEX, 0.6,
                        (255, 255, 255), 1)
            cv2.putText(rgb_show_data, str_3, (0, int(show_height * 4 / 5) - 5), cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 255, 255), 1)

            # 4图拼接
            temp_height = int(rgb_tmp.shape[0])
            if temp_height != show_height:
                rgb_show_0 = cv2.resize(rgb_tmp, (show_width, show_height))
            else:
                rgb_show_0 = rgb_tmp.copy()
            rgb_mix = np.zeros(((show_height * 2), (show_width * 2), 3), np.uint8)
            rgb_mix[0:show_height, 0:show_width] = rgb_show_0
            rgb_mix[0:show_height, show_width:(show_width * 2)] = rgb_show_line
            rgb_mix[show_height:(show_height * 2), 0:show_width] = rgb_show_1
            rgb_mix[show_height:(show_height * 2), show_width:(show_width * 2)] = rgb_show_data
            cv2.imshow('Show', rgb_mix)

            # cv2.imwrite(file_address + 'C' + str_Time + '.jpg', rgb_show_0)
            # cv2.imwrite(file_address + 'M' + str_Time + '.jpg', rgb_mix)

            end_time_calc = time.time()
            time_mess_calc = round((end_time_calc - start_time_calc) * 1000, 0)

            start_time = time.time()

        cv2.waitKey(1)


def quit_all():
    print('Exit ALL')
    os.kill(os.getpid(), signal.SIGTERM)


# 根据发送的数据，拆分控制信号和主板信号
def read_message(hex_rec):
    str_rec = binascii.b2a_hex(hex_rec).decode()
    is_normal = True
    if len(str_rec) >= 16 and str_rec[0:4] == 'aa01':
        int_move_left = int(str_rec[4:6])
        int_speed_left = int(str_rec[6:8], 16)
        int_move_right = int(str_rec[8:10])
        int_speed_right = int(str_rec[10:12], 16)
        int_board = int(str_rec[12:14])
        int_water = int(str_rec[14:16])
        is_normal = True
        ret_list = [int_move_left, int_speed_left, int_move_right, int_speed_right, int_board, int_water]
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


# 校正偏航角
def correct_yaw(now_yaw, target_yaw, cmd_56, q_i, q_d, q_c, q_ci, file_address):
    global glo_is_init, rec_side_left, rec_side_right

    get_correct_err = 0
    imu_yaw = 0.0
    cam_yaw = -999.9
    rot_yaw = now_yaw - target_yaw

    loop_nofeedback = 0  # 累加无反馈次数
    threshold_nofeedback = 10  # 累计无反馈上限值

    target_count = 0

    glo_is_init = True
    # 等待视觉测距稳定
    for i in range(0, 3, 1):
        se.write(hex_allStop)
        is_read, ret_list = read_message(hex_allStop)
        if is_read:
            q_c.put(ret_list)
            q_c.get() if q_c.qsize() > 1 else time.sleep(0.001)
            q_ci.put(ret_list)
            q_ci.get() if q_ci.qsize() > 1 else time.sleep(0.001)
        # str_send = binascii.b2a_hex(hex_correctDis).decode('utf-8')
        # str_Time_cy = datetime.datetime.now().strftime('%H:%M:%S.%f')
        # file_rec = open(file_address + 'Control.txt', 'a')
        # file_rec.write(str_Time_cy + ';CorrectYaw;s;' + str_send + ';\n')
        # file_rec.close()
        # 等待串口发送
        cv2.waitKey(40)
        # 读取主板反馈
        hex_rec = se.readline()
        if hex_rec:
            pass
            # 收到反馈
            # str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
            # file_rec = open(file_address + 'Control.txt', 'a')
            # file_rec.write(str_Time_cy + ';CorrectYaw;r;' + str_rec + ';\n')
            # file_rec.close()
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
            if dis_list[1] != -999.9:
                side_l = dis_list[1]
                rec_side_left.append(side_l)
            if dis_list[2] != -999.9:
                side_r = dis_list[2]
                rec_side_right.append(side_r)
            side_temp = dis_list[3]
            if side_temp != -999.9:
                cam_yaw = side_temp
        # 获取偏航角
        if not q_i.empty():
            imu_list = q_i.get()
            imu_yaw = -imu_list[2]

    if cam_yaw != -999.9:
        diff_cam_imu = cam_yaw - imu_yaw
    else:
        diff_cam_imu = 0.0

    # while abs(rot_yaw) > 1.0:
    while target_count <= 5:
        last_yaw = imu_yaw - target_yaw

        # 设置旋转动作
        if target_count == 0:
            # 根据偏航角设定初始旋转速度
            if abs(rot_yaw) > 20:
                correct_speed = 10
            elif abs(rot_yaw) > 10:
                correct_speed = 5
            else:
                correct_speed = 2
            cmd_24_corSpeed = CVFunc.trans_speed(str(correct_speed))
            if rot_yaw < 0:     # yaw负数，偏向左，向右旋转
                hex_correctYaw = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_corSpeed + cmd_13_back + cmd_24_corSpeed + cmd_56)
            else:       # yaw正数，偏向右，向左旋转
                hex_correctYaw = CVFunc.set_order(cmd_0_head + cmd_13_back + cmd_24_corSpeed + cmd_13_front + cmd_24_corSpeed + cmd_56)
        else:
            hex_correctYaw = hex_allStop

        # 发送动作命令
        se.write(hex_correctYaw)
        is_read, ret_list = read_message(hex_correctYaw)
        if is_read:
            q_c.put(ret_list)
            q_c.get() if q_c.qsize() > 1 else time.sleep(0.001)
            q_ci.put(ret_list)
            q_ci.get() if q_ci.qsize() > 1 else time.sleep(0.001)
        # str_send = binascii.b2a_hex(hex_correctYaw).decode('utf-8')
        # str_Time_cy = datetime.datetime.now().strftime('%H:%M:%S.%f')
        # file_rec = open(file_address + 'Control.txt', 'a')
        # file_rec.write(str_Time_cy + ';CorrectYaw;s;' + str_send + ';\n')
        # file_rec.close()
        # 等待串口发送
        cv2.waitKey(40)

        # 获取IMU偏航角
        if not q_i.empty():
            imu_list = q_i.get()
            imu_yaw = -imu_list[2]

        # 获取视觉偏航角
        if not q_d.empty():
            dis_list = q_d.get()
            if dis_list[1] != -999.9:
                side_l = dis_list[1]
                rec_side_left.append(side_l)
            if dis_list[2] != -999.9:
                side_r = dis_list[2]
                rec_side_right.append(side_r)
            cam_yaw = dis_list[3]

        # 读取主板反馈
        hex_rec = se.readline()
        if hex_rec:
            pass
            # 收到反馈
            # str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
            # file_rec = open(file_address + 'Control.txt', 'a')
            # file_rec.write(str_Time_cy + ';CorrectYaw;r;' + str_rec + ';\n')
            # file_rec.close()
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

        # 获取视觉偏航角
        if not q_d.empty():
            dis_list = q_d.get()
            if dis_list[1] != -999.9:
                side_l = dis_list[1]
                rec_side_left.append(side_l)
            if dis_list[2] != -999.9:
                side_r = dis_list[2]
                rec_side_right.append(side_r)
            cam_yaw = dis_list[3]

        if cam_yaw != -999.9:
            rot_yaw = cam_yaw - target_yaw
        else:
            rot_yaw = imu_yaw + diff_cam_imu - target_yaw
        if abs(rot_yaw) <= 1.0:  # 如果与目标角度相差不大于1，则认为已完成，
            target_count += 1
        else:
            target_count = 0
            # 键盘输入急停
        if keyboard.is_pressed('b'):
            glo_is_init = False
            get_correct_err = single_action(hex_allStop, q_c, q_ci, file_address)
            get_correct_err = 96
            return get_correct_err

    glo_is_init = False
    return get_correct_err


# 校正偏移距离
def correct_deviation(dis_dev, target_l, target_r, cmd_56, q_i, q_d, q_c, q_ci, file_address):
    global rec_side_left, rec_side_right
    is_right = False
    imu_yaw = 0.0
    side_l = -999.9
    side_r = -999.9
    cam_yaw = -999.9
    now_yaw = 0.0
    move_dis = 0.0

    # 1.更新调整距离
    num_timer = 0
    try_again = True
    while num_timer < 3 and try_again:
        num_timer += 1
        # 停止
        get_correct_err = single_action(hex_allStop, q_c, q_ci, file_address)
        if get_correct_err > 90:
            return get_correct_err
        # 获取视觉
        if not q_d.empty():
            dis_list = q_d.get()
            side_f = dis_list[0]
            side_l = dis_list[1]
            side_r = dis_list[2]
            cam_yaw = dis_list[3]
        if not q_i.empty():
            imu_list = q_i.get()
            imu_roll = imu_list[0]
            imu_pitch = imu_list[1]
            imu_yaw = -imu_list[2]
        if cam_yaw != -999.9 or 0.0:
            now_yaw = cam_yaw
        else:
            now_yaw = imu_yaw
        # 判断平移距离
        if target_l <= target_r:
            if side_l != -999.9:
                move_dis = side_l - target_l
                dis_dev = move_dis
                try_again = False
            elif side_r != -999.9:
                move_dis = target_r - side_r
            else:
                move_dis = dis_dev
        else:
            if side_r != -999.9:
                move_dis = target_r - side_r
                dis_dev = move_dis
                try_again = False
            elif side_l != -999.9:
                move_dis = side_l - target_l
            else:
                move_dis = dis_dev

    # 2.根据偏航距离，判断左旋还是右旋
    if move_dis < 0:
        is_right = True
        rot_angle = 30.0
    else:
        is_right = False
        rot_angle = -30.0
    get_correct_err = correct_yaw(now_yaw, rot_angle, cmd_56, q_i, q_d, q_c, q_ci, file_address)
    if get_correct_err > 90:
        return get_correct_err

    # 3.直线移动
    num_error_dis = 0
    target_count = 0
    dis_dev = abs(dis_dev)
    move_dis = abs(move_dis)
    while target_count < 5 and num_error_dis < 10:
        print('测量距离' + str(int(move_dis)) + '  估算距离' + str(int(dis_dev)))
        # 根据剩余距离设定速度和前进后退
        if target_count == 0:
            if -50.0 < move_dis < -10.0:
                cmd_24_corSpeed = CVFunc.trans_speed('10')
                hex_correctDis = CVFunc.set_order(cmd_0_head + cmd_13_back + cmd_24_corSpeed + cmd_13_back + cmd_24_corSpeed + cmd_56)
                int_move = 3
                num_error_dis = 0
            elif move_dis >= 10.0:
                if move_dis > 50.0:
                    cmd_24_corSpeed = CVFunc.trans_speed('20')
                    int_move = 2
                else:
                    cmd_24_corSpeed = CVFunc.trans_speed('10')
                    int_move = 1
                hex_correctDis = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_corSpeed + cmd_13_front + cmd_24_corSpeed + cmd_56)
                num_error_dis = 0
            else:
                hex_correctDis = hex_allStop
                int_move = 0
                num_error_dis += 1
        else:
            hex_correctDis = hex_allStop
            int_move = 0
        get_correct_err = single_action(hex_correctDis, q_c, q_ci, file_address)
        # 键盘输入急停
        if keyboard.is_pressed('b'):
            int_move = 0
            get_correct_err = single_action(hex_allStop, q_c, q_ci, file_address)
            get_correct_err = 96
            return get_correct_err
        if get_correct_err > 90:
            return get_correct_err

        # 更新剩余距离。
        if not q_d.empty():
            dis_list = q_d.get()
            side_l = dis_list[1]
            side_r = dis_list[2]
            cam_yaw = dis_list[3]
        # 估算的剩余行驶距离
        if int_move == 1:
            dis_dev = dis_dev - 2
        elif int_move == 2:
            dis_dev = dis_dev - 4
        elif int_move == 3:
            dis_dev = dis_dev + 2
        # 实际测得的剩余行驶距离，相差过大则直接使用估算距离
        if is_right:
            if side_r != -999.9:
                temp_dis = side_r - target_r
                if abs(temp_dis - dis_dev) < 50.0:
                    move_dis = temp_dis
                    dis_dev = move_dis
                    num_error_dis = 0
                else:
                    num_error_dis += 1
                    move_dis = dis_dev
            else:
                if int_move == 1:
                    move_dis = move_dis - 2
                elif int_move == 2:
                    move_dis = move_dis - 4
                elif int_move == 3:
                    move_dis = move_dis + 2
        else:
            if side_l != -999.9:
                temp_dis = side_l - target_l
                if abs(temp_dis - dis_dev) < 50.0:
                    move_dis = temp_dis
                    dis_dev = move_dis
                    num_error_dis = 0
                else:
                    num_error_dis += 1
                    move_dis = dis_dev
            else:
                if int_move == 1:
                    move_dis = move_dis - 2
                elif int_move == 2:
                    move_dis = move_dis - 4
                elif int_move == 3:
                    move_dis = move_dis + 2

        # 如果相差距离不大于10，开始累加5次后进入下一步；否则继续调整
        if abs(move_dis) <= 10 or abs(dis_dev) <= 10.0:
            target_count += 1
        else:
            target_count = 0
    if target_count < 5 and num_error_dis == 10:
        get_correct_err = 95

    # 3.旋转回直
    if not q_d.empty():
        dis_list = q_d.get()
        cam_yaw = dis_list[3]
    if not q_i.empty():
        imu_list = q_i.get()
        imu_yaw = -imu_list[2]
    # 提取偏航角
    if cam_yaw != -999.9 or 0.0:
        now_yaw = cam_yaw
    else:
        now_yaw = imu_yaw
    get_straight_err = correct_yaw(now_yaw, 0.0, cmd_56, q_i, q_d, q_c, q_ci, file_address)
    if get_straight_err > 90:
        return get_straight_err

    # 左右边全局记录清零
    rec_side_left = [target_l]
    rec_side_right = [target_r]
    return get_correct_err


# 前后清洗
def go_wash(wash_speed, cmd_56, q_i, q_c, q_ci, q_d, lock_ser, file_address, loop_times, wash_left, wash_right, q_l):
    global rec_side_left, rec_side_right
    # 传感器状态，0是正常，1是到边。99是传感器异常，98是无反馈，97是运行报错，96是手动急停
    get_state = 0
    loop_nofeedback = 0  # 累加无反馈次数
    threshold_nofeedback = 10  # 累计无反馈上限值
    imu_yaw = 0.0
    imu_pitch = 0.0
    imu_roll = 0.0
    loop_num = 0

    side_f = -999.9
    side_l = -999.9
    side_r = -999.9
    cam_yaw = -999.9

    dis_laser = 20
    # 视觉偏航与IMU偏航相符标识位。如果相符，无视觉时靠IMU；如果不符，无视觉时置零。
    cy_equal_iy = False
    # 偏航调整
    moveDis_dev = 0
    stage_dev = 0
    toLeft_dev = False

    while get_state == 0 and (loop_num <= loop_times or loop_times == -1):
        loop_num += 1
        # 1.获取感知数据
        # 获取激光测距
        if not q_l.empty():
            dis_laser = q_l.get()
        # 获取视觉
        if not q_d.empty():
            dis_list = q_d.get()
            side_f = dis_list[0]
            if panel_width > dis_list[1] > 0.0:
                side_l = dis_list[1]
                rec_side_left.append(side_l)
            if panel_width > dis_list[2] > 0.0:
                side_r = dis_list[2]
                rec_side_right.append(side_r)
            cam_yaw = dis_list[3]
        else:
            cv2.waitKey(50)
            if not q_d.empty():
                dis_list = q_d.get()
                side_f = dis_list[0]
                if panel_width > dis_list[1] > 0.0:
                    side_l = dis_list[1]
                    rec_side_left.append(side_l)
                if panel_width > dis_list[2] > 0.0:
                    side_r = dis_list[2]
                    rec_side_right.append(side_r)
                cam_yaw = dis_list[3]
            else:
                side_l = -999.9
                side_r = -999.9
                cam_yaw = -999.9
        # 获取IMU
        if not q_i.empty():
            imu_list = q_i.get()
            imu_roll = imu_list[0]
            imu_pitch = imu_list[1]
            imu_yaw = -imu_list[2]
        # 提取偏航角
        if cam_yaw != -999.9:
            now_yaw = cam_yaw
            if abs(cam_yaw - imu_yaw) < 2.0:
                cy_equal_iy = True
            else:
                cy_equal_iy = False
        else:
            if cy_equal_iy:
                now_yaw = imu_yaw
            else:
                now_yaw = 0.0
        # 有任意一边识别到距离，则开始提取偏移距离
        if side_l != -999.9 or side_r != -999.9:
            num_rec_right = len(rec_side_right)
            if num_rec_right > 2:
                last_right = (rec_side_right[num_rec_right - 2] + rec_side_right[num_rec_right - 3]) / 2
            else:
                last_right = wash_right

            num_rec_left = len(rec_side_left)
            if num_rec_left > 2:
                last_left = (rec_side_left[num_rec_left - 2] + rec_side_left[num_rec_left - 3]) / 2
            else:
                last_left = wash_left

            # if num_rec_left > 1 or num_rec_right > 1:
                # print(wash_left, rec_side_left, wash_right, rec_side_right)

            if (max(0.0, wash_right - 200) <= side_r <= wash_right + 200) and (
                    max(0.0, last_right - 10) <= side_r <= last_right + 10):
                dis_deviation_left = wash_right - side_r
            else:
                dis_deviation_left = -999.9

            if (max(0.0, wash_left - 200) <= side_l <= wash_left + 200) and (
                    max(0.0, last_left - 10) <= side_l <= last_left + 10):
                dis_deviation_right = side_l - wash_left
            else:
                dis_deviation_right = -999.9

            if dis_deviation_left != -999.9 and dis_deviation_right != -999.9:
                if abs(dis_deviation_left) < abs(dis_deviation_right):
                    dis_deviation = dis_deviation_left
                else:
                    dis_deviation = dis_deviation_right
            elif dis_deviation_left == -999.9 and dis_deviation_right != -999.9:
                dis_deviation = dis_deviation_right
            elif dis_deviation_left != -999.9 and dis_deviation_right == -999.9:
                dis_deviation = dis_deviation_left
            else:
                dis_deviation = 0.0
            # print(int(dis_deviation), int(dis_deviation_left), int(dis_deviation_right))
        else:
            dis_deviation = 0.0
            dis_deviation_left = 0.0
            dis_deviation_right = 0.0

        # 2.特殊状态调整
        if keyboard.is_pressed('b'):    # 急停按钮，标识报错并跳出
            get_error = single_action(hex_allStop, q_c, q_ci, file_address)
            get_state = 96
            break
        elif keyboard.is_pressed('r') or dis_laser <= laser_threshold:      # 手动或测距到边
            get_error = single_action(hex_allStop, q_c, q_ci, file_address)
            get_state = 1
            break
        elif abs(now_yaw) >= correct_thre_yaw:      # 偏航角过大，直接偏航校正
            print('偏航' + str(now_yaw))
            get_state = correct_yaw(now_yaw, 0.0, cmd_56, q_i, q_d, q_c, q_ci, file_address)
            if get_state > 90:
                get_error = single_action(hex_allStop, q_c, q_ci, file_address)
                return get_state
            cam_yaw = 0.0
            now_yaw = 0.0
            loop_num = 0
            continue
        elif abs(dis_deviation) >= correct_thre_deviation:      # 偏移量过大，直接偏移校正
            print('偏移' + str(dis_deviation) + '  左侧' + str(side_l) + '  右侧' + str(side_r))
            get_state = correct_deviation(dis_deviation, wash_left, wash_right, cmd_56, q_i, q_d, q_c, q_ci,
                                          file_address)
            if get_state > 90:
                get_error = single_action(hex_allStop, q_c, q_ci, file_address)
                return get_state
            cam_yaw = 0.0
            now_yaw = 0.0
            loop_num = 0
            continue
        elif wash_thre_deviation <= abs(dis_deviation) < correct_thre_deviation and stage_dev == 0:      # 偏移量较小，左右转后直行
            print('偏移调整', str(dis_deviation), str(dis_deviation_left), str(dis_deviation_right), rec_side_left, rec_side_right)
            moveDis_dev = int(round((abs(dis_deviation) - 10) / 4, 0)) + 4
            stage_dev = 0
            if dis_deviation < 0:
                toLeft_dev = False
            else:
                toLeft_dev = True
        elif abs(dis_deviation) < wash_thre_deviation < abs(moveDis_dev) and stage_dev > 0:     # 如果偏移调整过程中发现进入区域内，则跳出调整
            print('取消调整')
            if stage_dev == 1:
                stage_dev = moveDis_dev - 1
            elif stage_dev < moveDis_dev - 2:
                stage_dev = moveDis_dev - 2

        # 3.判断移动状态
        if loop_num * 5 < wash_speed:      # 启动阶段，每步5%加速
            cmd_24_speedUp = CVFunc.trans_speed(str(5 * loop_num))
            hex_washFront = CVFunc.set_order(
                cmd_0_head + cmd_13_front + cmd_24_speedUp + cmd_13_front + cmd_24_speedUp + cmd_56)
        elif moveDis_dev > 0:
            stage_dev += 1
            if 0 < stage_dev <= 2:
                if toLeft_dev:
                    hex_washFront = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_slow + cmd_13_front + cmd_24_fast + cmd_56)
                else:
                    hex_washFront = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_fast + cmd_13_front + cmd_24_slow + cmd_56)
            elif 2 < stage_dev <= (moveDis_dev - 2):
                print('剩余', str(moveDis_dev - stage_dev - 2))
                cmd_24_washSpeed = CVFunc.trans_speed(str(wash_speed))
                hex_washFront = CVFunc.set_order(
                    cmd_0_head + cmd_13_front + cmd_24_washSpeed + cmd_13_front + cmd_24_washSpeed + cmd_56)
            elif (moveDis_dev - 2) < stage_dev <= moveDis_dev:
                if toLeft_dev:
                    hex_washFront = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_fast + cmd_13_front + cmd_24_slow + cmd_56)
                else:
                    hex_washFront = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_slow + cmd_13_front + cmd_24_fast + cmd_56)
            else:
                print('完成调整')
                cmd_24_washSpeed = CVFunc.trans_speed(str(wash_speed))
                hex_washFront = CVFunc.set_order(
                    cmd_0_head + cmd_13_front + cmd_24_washSpeed + cmd_13_front + cmd_24_washSpeed + cmd_56)
                moveDis_dev = 0
                stage_dev = 0
        elif wash_thre_yaw < abs(now_yaw) < correct_thre_yaw:       # 偏航角在可控范围内，差速调整
            speed_left = wash_speed - int(round(now_yaw, 0))
            if speed_left < 0:
                speed_left = 0
            elif speed_left > 100:
                speed_left = 100
            cmd_2_corYaw = CVFunc.trans_speed(str(speed_left))
            speed_right = wash_speed + int(round(now_yaw, 0))
            if speed_right < 0:
                speed_right = 0
            elif speed_right > 100:
                speed_right = 100
            cmd_4_corYaw = CVFunc.trans_speed(str(speed_right))
            hex_washFront = CVFunc.set_order(
                cmd_0_head + cmd_13_front + cmd_2_corYaw + cmd_13_front + cmd_4_corYaw + cmd_56)
            print('差速调整', str(now_yaw), str(cam_yaw), str(imu_yaw), str(speed_left), str(speed_right) )
        else:
            cmd_24_washSpeed = CVFunc.trans_speed(str(wash_speed))
            hex_washFront = CVFunc.set_order(
                cmd_0_head + cmd_13_front + cmd_24_washSpeed + cmd_13_front + cmd_24_washSpeed + cmd_56)

        # 3.执行清洗移动
        get_state = single_action(hex_washFront, q_c, q_ci, file_address)
        if keyboard.is_pressed('b'):    # 急停按钮，标识报错并跳出
            get_error = single_action(hex_allStop, q_c, q_ci, file_address)
            get_state = 96
            break
        if get_state > 90:
            get_error = single_action(hex_allStop, q_c, q_ci, file_address)
            break

    return get_state


# 到边调头
def turn_around(cmd_56, q_i, q_c, q_ci, q_d, lock_ser, file_address):
    get_state = 0
    cmd_24_rotateSpeed = CVFunc.trans_speed('40')
    hex_turnAround = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_rotateSpeed + cmd_13_back + cmd_24_rotateSpeed + cmd_56)
    imu_yaw = 0.0
    cam_yaw = -999.9
    loop_times = 0

    # 向右不断旋转，直至转到180±20度
    while loop_times < 22:
        loop_times += 1
        get_state = single_action(hex_turnAround, q_c, q_ci, file_address)
        if keyboard.is_pressed('b'):    # 急停按钮，标识报错并跳出
            get_error = single_action(hex_allStop, q_c, q_ci, file_address)
            get_state = 96
            return get_state
        if get_state > 90:
            get_error = single_action(hex_allStop, q_c, q_ci, file_address)
            return get_state

        # 获取偏航角
        if not q_i.empty():
            imu_list = q_i.get()
            imu_yaw = -imu_list[2]

    # 获取视觉
    if not q_d.empty():
        dis_list = q_d.get()
        cam_yaw = dis_list[3]
    # 获取IMU
    if not q_i.empty():
        imu_list = q_i.get()
        imu_yaw = -imu_list[2]
    # 提取偏航角
    if cam_yaw != -999.9 or 0.0:
        now_yaw = cam_yaw
    else:
        now_yaw = imu_yaw
    get_state = correct_yaw(now_yaw, 0.0, cmd_56, q_i, q_d, q_c, q_ci, file_address)

    return get_state


# 到边后U型平移调头
def u_turn(cmd_56, q_i, q_c, q_ci, q_d, lock_ser, file_address):
    get_state = 0
    cmd_24_rotateSpeed = CVFunc.trans_speed('40')
    cmd_24_moveSpeed = CVFunc.trans_speed('40')
    hex_turnAround = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_rotateSpeed + cmd_13_back + cmd_24_rotateSpeed + cmd_56)
    hex_moveFront = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_moveSpeed + cmd_13_front + cmd_24_moveSpeed + cmd_56)
    imu_yaw = 0.0
    cam_yaw = -999.9
    loop_times = 0
    # 获取偏航角
    if not q_i.empty():
        imu_list = q_i.get()
        imu_yaw = -imu_list[2]
    else:
        cv2.waitKey(50)
        if not q_i.empty():
            imu_list = q_i.get()
            imu_yaw = -imu_list[2]
    # 1.向右不断旋转，直至转到90±20度
    while loop_times < 11:
        loop_times += 1
        get_state = single_action(hex_turnAround, q_c, q_ci, file_address)
        if keyboard.is_pressed('b'):  # 急停按钮，标识报错并跳出
            get_error = single_action(hex_allStop, q_c, q_ci, file_address)
            get_state = 96
            return get_state
        if get_state > 90:
            get_error = single_action(hex_allStop, q_c, q_ci, file_address)
            return get_state
    # 2.校准90度
    if not q_d.empty():
        dis_list = q_d.get()
        cam_yaw = dis_list[3]
    if not q_i.empty():
        imu_list = q_i.get()
        imu_yaw = -imu_list[2]
    if cam_yaw != -999.9 or 0.0:
        now_yaw = cam_yaw
    else:
        cv2.waitKey(50)
        if not q_d.empty():
            dis_list = q_d.get()
            cam_yaw = dis_list[3]
        if not q_i.empty():
            imu_list = q_i.get()
            imu_yaw = -imu_list[2]
        if cam_yaw != -999.9 or 0.0:
            now_yaw = cam_yaw
        else:
            now_yaw = imu_yaw
    get_state = correct_yaw(now_yaw, 0.0, cmd_56, q_i, q_d, q_c, q_ci, file_address)
    if keyboard.is_pressed('b'):  # 急停按钮，标识报错并跳出
        get_error = single_action(hex_allStop, q_c, q_ci, file_address)
        get_state = 96
        return get_state
    if get_state > 90:
        return get_state

    # 3.向前定次数移动
    for i in range(0, 12, 1):
        get_state = single_action(hex_moveFront, q_c, q_ci, file_address)
        if keyboard.is_pressed('b'):  # 急停按钮，标识报错并跳出
            get_error = single_action(hex_allStop, q_c, q_ci, file_address)
            get_state = 96
            return get_state
        # 报错退出
        if get_state > 90:
            return get_state

    # 4.向右不断旋转，直至转到90±20度
    loop_times = 0
    while loop_times < 11:
        loop_times += 1
        get_state = single_action(hex_turnAround, q_c, q_ci, file_address)
        if keyboard.is_pressed('b'):  # 急停按钮，标识报错并跳出
            get_error = single_action(hex_allStop, q_c, q_ci, file_address)
            get_state = 96
            return get_state
        if get_state > 90:
            get_error = single_action(hex_allStop, q_c, q_ci, file_address)
            return get_state
    # 5.校准90度
    if not q_d.empty():
        dis_list = q_d.get()
        cam_yaw = dis_list[3]
    if not q_i.empty():
        imu_list = q_i.get()
        imu_yaw = -imu_list[2]
    if cam_yaw != -999.9 or 0.0:
        now_yaw = cam_yaw
    else:
        now_yaw = imu_yaw
    get_state = correct_yaw(now_yaw, 0.0, cmd_56, q_i, q_d, q_c, q_ci, file_address)
    if get_state > 90:
        return get_state

    return get_state


# 到边后“弓”型转向
def z_turn(is_far, cmd_56, q_i, q_c, q_ci, q_d, q_l, lock_ser, file_address):
    get_state = 0
    cmd_24_rotateSpeed = CVFunc.trans_speed('40')
    cmd_24_moveSpeed = CVFunc.trans_speed('40')
    hex_turnRight = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_rotateSpeed + cmd_13_back + cmd_24_rotateSpeed + cmd_56)
    hex_turnLeft = CVFunc.set_order(cmd_0_head + cmd_13_back + cmd_24_rotateSpeed + cmd_13_front + cmd_24_rotateSpeed + cmd_56)
    hex_moveFront = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_moveSpeed + cmd_13_front + cmd_24_moveSpeed + cmd_56)

    imu_yaw = 0.0
    cam_yaw = -999.9
    loop_times = 0
    dis_laser = 999.9
    # 获取偏航角
    if not q_i.empty():
        imu_list = q_i.get()
        imu_yaw = -imu_list[2]
    else:
        cv2.waitKey(50)
        if not q_i.empty():
            imu_list = q_i.get()
            imu_yaw = -imu_list[2]
    # 1.根据弓型方向，向左或向右不断旋转，直至转到90±20度。如果已到边界，直接退出并返回1
    while loop_times < 11:
        loop_times += 1
        if is_far:
            get_state = single_action(hex_turnLeft, q_c, q_ci, file_address)
        else:
            get_state = single_action(hex_turnRight, q_c, q_ci, file_address)
        if keyboard.is_pressed('b'):  # 急停按钮，标识报错并跳出
            get_error = single_action(hex_allStop, q_c, q_ci, file_address)
            get_state = 96
            return get_state
        if get_state > 90:
            get_error = single_action(hex_allStop, q_c, q_ci, file_address)
            return get_state
    if not q_l.empty():
        dis_laser = q_l.get()
        if dis_laser <= laser_threshold:
            get_state = 1
            return get_state

    # 2.校准90度
    if not q_d.empty():
        dis_list = q_d.get()
        cam_yaw = dis_list[3]
    if not q_i.empty():
        imu_list = q_i.get()
        imu_yaw = -imu_list[2]
    if cam_yaw != -999.9 or 0.0:
        now_yaw = cam_yaw
    else:
        cv2.waitKey(50)
        if not q_d.empty():
            dis_list = q_d.get()
            cam_yaw = dis_list[3]
        if not q_i.empty():
            imu_list = q_i.get()
            imu_yaw = -imu_list[2]
        if cam_yaw != -999.9 or 0.0:
            now_yaw = cam_yaw
        else:
            now_yaw = imu_yaw
    get_state = correct_yaw(now_yaw, 0.0, cmd_56, q_i, q_d, q_c, q_ci, file_address)
    if keyboard.is_pressed('b'):  # 急停按钮，标识报错并跳出
        get_error = single_action(hex_allStop, q_c, q_ci, file_address)
        get_state = 96
        return get_state
    if get_state > 90:
        return get_state

    # 3.向前定次数移动。如果已到边界，直接退出并返回1
    for i in range(0, 41, 1):
        if not q_l.empty():
            dis_laser = q_l.get()
            if dis_laser <= laser_threshold:
                get_state = 1
                return get_state
        get_state = single_action(hex_moveFront, q_c, q_ci, file_address)
        if keyboard.is_pressed('b'):  # 急停按钮，标识报错并跳出
            get_error = single_action(hex_allStop, q_c, q_ci, file_address)
            get_state = 96
            return get_state
        if get_state > 90:
            return get_state
        if not q_l.empty():
            dis_laser = q_l.get()
            if dis_laser <= laser_threshold:
                get_state = 1
                return get_state

    # 4.向左或向右不断旋转，直至转到90±20度
    loop_times = 0
    while loop_times < 11:
        loop_times += 1
        if is_far:
            get_state = single_action(hex_turnLeft, q_c, q_ci, file_address)
        else:
            get_state = single_action(hex_turnRight, q_c, q_ci, file_address)
        if keyboard.is_pressed('b'):  # 急停按钮，标识报错并跳出
            get_error = single_action(hex_allStop, q_c, q_ci, file_address)
            get_state = 96
            return get_state
        if get_state > 90:
            get_error = single_action(hex_allStop, q_c, q_ci, file_address)
            return get_state

    # 5.校准90度
    if not q_d.empty():
        dis_list = q_d.get()
        cam_yaw = dis_list[3]
    if not q_i.empty():
        imu_list = q_i.get()
        imu_yaw = -imu_list[2]
    if cam_yaw != -999.9 or 0.0:
        now_yaw = cam_yaw
    else:
        now_yaw = imu_yaw
    get_state = correct_yaw(now_yaw, 0.0, cmd_56, q_i, q_d, q_c, q_ci, file_address)
    if get_state > 90:
        return get_state

    return get_state


# 到达终点后返回
def end_back(cmd_56, q_i, q_c, q_ci, q_d, q_l, lock_ser, file_address):
    get_state = 0
    cmd_24_rotateSpeed = CVFunc.trans_speed('40')
    cmd_24_moveSpeed = CVFunc.trans_speed('40')
    hex_turnRight = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_rotateSpeed + cmd_13_back + cmd_24_rotateSpeed + cmd_56)
    hex_turnLeft = CVFunc.set_order(cmd_0_head + cmd_13_back + cmd_24_rotateSpeed + cmd_13_front + cmd_24_rotateSpeed + cmd_56)
    hex_moveFront = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_moveSpeed + cmd_13_front + cmd_24_moveSpeed + cmd_56)
    imu_yaw = 0.0
    cam_yaw = -999.9
    loop_times = 0
    dis_laser = 999.9

    # 1.向右不断旋转，直至转到180±20度
    while loop_times < 22:
        loop_times += 1
        get_state = single_action(hex_turnRight, q_c, q_ci, file_address)
        if keyboard.is_pressed('b'):    # 急停按钮，标识报错并跳出
            get_error = single_action(hex_allStop, q_c, q_ci, file_address)
            get_state = 96
            return get_state
        if get_state > 90:
            get_error = single_action(hex_allStop, q_c, q_ci, file_address)
            return get_state

    # 2.校准90度
    if not q_d.empty():
        dis_list = q_d.get()
        cam_yaw = dis_list[3]
    if not q_i.empty():
        imu_list = q_i.get()
        imu_yaw = -imu_list[2]
    if cam_yaw != -999.9 or 0.0:
        now_yaw = cam_yaw
    else:
        cv2.waitKey(50)
        if not q_d.empty():
            dis_list = q_d.get()
            cam_yaw = dis_list[3]
        if not q_i.empty():
            imu_list = q_i.get()
            imu_yaw = -imu_list[2]
        if cam_yaw != -999.9 or 0.0:
            now_yaw = cam_yaw
        else:
            now_yaw = imu_yaw
    get_state = correct_yaw(now_yaw, 0.0, cmd_56, q_i, q_d, q_c, q_ci, file_address)
    if keyboard.is_pressed('b'):  # 急停按钮，标识报错并跳出
        get_error = single_action(hex_allStop, q_c, q_ci, file_address)
        get_state = 96
        return get_state
    if get_state > 90:
        return get_state

    # 3.向前移动直至到边界
    while dis_laser > laser_threshold:
        if not q_l.empty():
            dis_laser = q_l.get()
        get_state = single_action(hex_moveFront, q_c, q_ci, file_address)
        if keyboard.is_pressed('b'):  # 急停按钮，标识报错并跳出
            get_error = single_action(hex_allStop, q_c, q_ci, file_address)
            get_state = 96
            return get_state
        if get_state > 90:
            return get_state
        if not q_l.empty():
            dis_laser = q_l.get()

    return get_state


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
def autocontrol_run(q_i, q_d, q_c, q_ci, lock_ser, file_address, q_l):
    global glo_is_init, panel_width, rec_side_left, rec_side_right

    print('Auto Start')

    # 设置清洗参数
    loop_time = -1  # 单向算作1次
    wash_loops = -1     # 设置前后清洗的移动距离限定，-1代表无限定。每次200ms，5次相当于1秒
    wash_right_dis = 190.0    # 右侧边界距离
    wash_left_dis = panel_width - vehicle_width - wash_right_dis       # 左侧边界距离
    need_back = False

    imu_yaw = 0.0
    imu_pitch = 0.0
    imu_roll = 0.0
    cmd_56 = cmd_5_stop + cmd_6_stop

    cam_yaw = -999.9
    side_f = -999.9
    side_l = -999.9
    side_r = -999.9

    # 无反馈重新循环标志位
    no_feedBack = False
    while True:
        glo_is_init = True
# 1.测试通信
        in_init = True
        while in_init:
            str_init = 'aa 01 00 00 00 00 00 00 b0'
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
                # file_rec = open(file_address + 'Control.txt', 'a')
                # file_rec.write(str_Time_ac + ';init;s;' + str_init + ';\n')
                # file_rec.close()
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
                            in_init = False
                            no_feedBack = False
                        else:
                            print('串口异常')
                    else:
                        print('串口异常')
                # if keyboard.is_pressed('n'):  # 结束运行，全部停止
                #     print('进入单步动作测试')
                #     in_init = False
            except Exception as e:
                print(e)
                print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
                print(f"error line:{e.__traceback__.tb_lineno}")

# 2.单个动作测试
        is_oneAction = True
        glo_is_init = True
        while is_oneAction:
            get_one_err = 0
            get_stop_err = 0
            # 输入单步命令
            if keyboard.is_pressed('w') or keyboard.is_pressed('8'):  # 向前移动，刮板和水系统停止
                print('前进')
                get_one_err = single_action(hex_Front, q_c, q_ci, file_address)
            elif keyboard.is_pressed('s') or keyboard.is_pressed('2'):  # 向后移动，刮板和水系统停止
                print('后退')
                get_one_err = single_action(hex_Back, q_c, q_ci, file_address)
            elif keyboard.is_pressed('a') or keyboard.is_pressed('4'):  # 向左旋转，刮板和水系统停止
                print('左旋')
                get_one_err = single_action(hex_rotateLeft, q_c, q_ci, file_address)
            elif keyboard.is_pressed('d') or keyboard.is_pressed('6'):  # 向右旋转，刮板和水系统停止
                print('右旋')
                get_one_err = single_action(hex_rotateRight, q_c, q_ci, file_address)
            elif keyboard.is_pressed('q') or keyboard.is_pressed('7'):  # 前行并左转，刮板和水系统停止
                print('左前转')
                get_one_err = single_action(hex_FrontLeft, q_c, q_ci, file_address)
            elif keyboard.is_pressed('e') or keyboard.is_pressed('9'):  # 前行并左转，刮板和水系统停止
                print('右前转')
                get_one_err = single_action(hex_FrontRight, q_c, q_ci, file_address)
            elif keyboard.is_pressed('r'):  # 结束运行，全部停止
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

# 3.进入自动控制
        print('进入自动控制')
        get_state = 0
        # 建立通信，开始执行
        loop_num = -1
        while (not need_back) and (loop_num < loop_time or loop_time == -1):
            loop_num += 1
            print('左边距' + str(wash_left_dis) + ' 右边距' + str(wash_right_dis))
            # 边距记录清零
            rec_side_right = [wash_right_dis]
            rec_side_left = [wash_left_dis]

            # 3.1.如果角度有偏航，进行校正
            if not q_d.empty():
                dis_list = q_d.get()
                side_f = dis_list[0]
                if dis_list[1] != -999.9:
                    side_l = dis_list[1]
                    rec_side_left.append(side_l)
                if dis_list[2] != -999.9:
                    side_r = dis_list[2]
                    rec_side_right.append(side_r)
                cam_yaw = dis_list[3]
            if not q_i.empty():
                imu_list = q_i.get()
                imu_roll = imu_list[0]
                imu_pitch = imu_list[1]
                imu_yaw = -imu_list[2]
            if cam_yaw != -999.9 or 0.0:
                now_yaw = cam_yaw
            else:
                now_yaw = imu_yaw
            # 偏航大于1.0就调整拉直
            if abs(now_yaw) > 1.0:
                print('偏航角度' + str(now_yaw))
                get_state = correct_yaw(now_yaw, 0.0, cmd_56, q_i, q_d, q_c, q_ci, file_address)
                # 无反馈退出
                if get_state == 98:
                    print('无反馈，结束运行')
                    no_feedBack = True
                elif get_state == 96:
                    print('手动急停')
                    no_feedBack = True
                elif get_state > 90:
                    print('故障')
                    no_feedBack = True
                    break
                print('校正完成')
            else:
                print('偏航在范围内')
            if no_feedBack:
                continue

            # 3.2. 如果距离轨迹有偏移，进行校正
            # 静止获取左右任意一边距离。如果同时获取到两边，则更新光伏板宽度。
            for i in range(0, 6, 1):
                if not q_d.empty():
                    dis_list = q_d.get()
                    side_f = dis_list[0]
                    if dis_list[1] != -999.9:
                        side_l = dis_list[1]
                        rec_side_left.append(side_l)
                    if dis_list[2] != -999.9:
                        side_r = dis_list[2]
                        rec_side_right.append(side_r)
                    cam_yaw = dis_list[3]
                get_state = single_action(hex_allStop, q_c, q_ci, file_address)
                if get_state == 98:
                    print('无反馈，结束运行')
                    no_feedBack = True
                    break
                if not q_d.empty():
                    dis_list = q_d.get()
                    side_f = dis_list[0]
                    if dis_list[1] != -999.9:
                        side_l = dis_list[1]
                        rec_side_left.append(side_l)
                    if dis_list[2] != -999.9:
                        side_r = dis_list[2]
                        rec_side_right.append(side_r)
                    cam_yaw = dis_list[3]
            # 累计左侧距离取平均
            if len(rec_side_left) > 1:
                temp_sum = 0.0
                for i in range(1, len(rec_side_left), 1):
                    temp_sum += rec_side_left[i]
                side_l = temp_sum / (len(rec_side_left) - 1)
            else:
                side_l = -999.9
            # 累计右侧距离取平均
            if len(rec_side_right) > 1:
                temp_sum = 0.0
                for i in range(1, len(rec_side_right), 1):
                    temp_sum += rec_side_right[i]
                side_r = temp_sum / (len(rec_side_right) - 1)
            else:
                side_r = -999.9
            print(rec_side_left, rec_side_right)
            # 根据识别情况进行处理
            if side_l != -999.9 or side_r != -999.9:
                # 如果同时识别到两边，则更新光伏板宽度
                if side_l != -999.9 and side_r != -999.9 and abs(panel_width - (side_l + side_r + vehicle_width)) < 50:
                    panel_width = side_l + side_r + vehicle_width
                    if wash_left_dis <= wash_right_dis:
                        wash_right_dis = panel_width - vehicle_width - wash_left_dis
                    else:
                        wash_left_dis = panel_width - vehicle_width - wash_right_dis
                    print('光伏板全长' + str(round(panel_width, 0)))

                # 优先判断距离近的边，如果没有再判断距离远的边
                dis_deviation = 0.0
                if wash_left_dis <= wash_right_dis:
                    if side_l != -999.9 and abs(side_l - wash_left_dis) > 20:
                        dis_deviation = side_l - wash_left_dis
                    elif side_r != -999.9 and abs(side_r - wash_right_dis) > 20:
                        dis_deviation = wash_right_dis - side_r
                else:
                    if side_r != -999.9 and abs(side_r - wash_right_dis) > 20:
                        dis_deviation = wash_right_dis - side_r
                    elif side_l != -999.9 and abs(side_l - wash_left_dis) > 20:
                        dis_deviation = side_l - wash_left_dis
                if abs(dis_deviation) > 50.0:
                    print('偏移距离' + str(dis_deviation))
                    get_state = correct_deviation(dis_deviation, wash_left_dis, wash_right_dis, cmd_56, q_i, q_d,
                                                  q_c, q_ci, file_address)
                    # 无反馈退出
                    if get_state == 98:
                        print('无反馈，结束运行')
                        no_feedBack = True
                    elif get_state == 96:
                        print('手动急停')
                        no_feedBack = True
                    elif get_state > 90:
                        print('故障')
                        no_feedBack = True
                        break
                    print('校正完成')
                else:
                    print('偏移在范围内')
            else:
                print('未找到边界')
            if no_feedBack:
                continue

            # 3.3.清洗前进
            print('清洗前行，次数', str(wash_loops))
            get_state = go_wash(int_washSpeed, cmd_56, q_i, q_c, q_ci, q_d, lock_ser, file_address, wash_loops, wash_left_dis, wash_right_dis, q_l)
            if get_state == 98:
                print('无反馈，结束运行')
                no_feedBack = True
                break
            elif get_state == 96:
                print('手动急停')
                no_feedBack = True
                break
            elif get_state == 95:
                print('偏移校正报错')
                break
            elif get_state > 90:
                print('故障')
                no_feedBack = True
                break
            elif get_state == 1:
                print('到边')

            # 3.4.到达位置，调头
            get_state = turn_around(cmd_56, q_i, q_c, q_ci, q_d, lock_ser, file_address)
            if get_state == 98:
                print('无反馈，结束运行')
                no_feedBack = True
                break
            elif get_state == 96:
                print('手动急停')
                no_feedBack = True
                break
            elif get_state > 90:
                print('故障')
                no_feedBack = True
                break
            print('完成调头')

            # # 3.4 转90度，直行一段距离后再转90度
            # get_state = u_turn(cmd_56, q_i, q_c, q_ci, q_d, lock_ser, file_address)
            # if get_state == 98:
            #     print('无反馈，结束运行')
            #     no_feedBack = True
            #     break
            # elif get_state == 96:
            #     print('手动急停')
            #     no_feedBack = True
            #     break
            # elif get_state > 90:
            #     print('故障')
            #     no_feedBack = True
            #     break
            # print('完成调头')

            # # 3.4 根据循环次数判断左移还是右移
            # if loop_num % 2 == 0:
            #     get_state = z_turn(True, cmd_56, q_i, q_c, q_ci, q_d, q_l, lock_ser, file_address)
            # else:
            #     get_state = z_turn(False, cmd_56, q_i, q_c, q_ci, q_d, q_l, lock_ser, file_address)
            # if get_state == 98:
            #     print('无反馈，结束运行')
            #     no_feedBack = True
            #     break
            # elif get_state == 96:
            #     print('手动急停')
            #     no_feedBack = True
            #     break
            # elif get_state > 90:
            #     print('故障')
            #     no_feedBack = True
            #     break
            # elif get_state == 1:
            #     need_back = True
            #
            # # 3.5 左右设定距离对调
            # temp_lr = wash_right_dis
            # wash_right_dis = wash_left_dis
            # wash_left_dis = temp_lr
            # # print('完成调头')
            #
            # # 3.6 如果到尽头，调头返回
            # if need_back:
            #     print('完成清洗，返回')
            #     get_state = end_back(cmd_56, q_i, q_c, q_ci, q_d, q_l, lock_ser, file_address)
            #     if get_state == 98:
            #         print('无反馈，结束运行')
            #         no_feedBack = True
            #         break
            #     elif get_state == 96:
            #         print('手动急停')
            #         no_feedBack = True
            #         break
            #     elif get_state > 90:
            #         print('故障')
            #         no_feedBack = True
            #         break

            if loop_time != -1:
                print('循环剩余' + str(loop_time - loop_num))
        print('自动循环结束')
        need_back = False
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
    # 单目相机测距，相机进程图像、视觉偏航角（给IMU和给计算进程）、标注后图像、前水平线、左垂直线、右垂直线
    queue_camera = mp.Queue(maxsize=2)
    queue_yaw_imu = mp.Queue(maxsize=2)
    queue_yaw_main = mp.Queue(maxsize=2)
    queue_image = mp.Queue(maxsize=2)
    queue_front = mp.Queue(maxsize=2)
    queue_left = mp.Queue(maxsize=2)
    queue_right = mp.Queue(maxsize=2)
    queue_temp = mp.Queue(maxsize=2)
    # IMU，偏航、俯仰和翻滚，分发前相机测距、计算和自动控制
    queue_imu_dis = mp.Queue(maxsize=2)
    queue_imu_main = mp.Queue(maxsize=2)
    queue_imu_auto = mp.Queue(maxsize=2)
    # 自动控制交互，计算给自控距离数据，自控给计算控制命令，自控给IMU控制命令
    queue_distance = mp.Queue(maxsize=2)
    queue_control = mp.Queue(maxsize=2)
    queue_control_imu = mp.Queue(maxsize=2)
    # 激光测距数据，发送至自动控制和计算
    queue_laser_auto = mp.Queue(maxsize=2)
    queue_laser_main = mp.Queue(maxsize=2)

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
            str_fileAddress, queue_temp, queue_laser_main)))
    # 自动运行
    processes.append(
        mp.Process(target=autocontrol_run, args=(
            queue_imu_auto, queue_distance, queue_control, queue_control_imu, lock, str_fileAddress, queue_laser_auto)))
    # 激光测距
    processes.append(mp.Process(target=laser_get, args=(queue_laser_auto, queue_laser_main, str_fileAddress)))

    for process in processes:
        process.daemon = True
        process.start()
    for process in processes:
        process.join()
