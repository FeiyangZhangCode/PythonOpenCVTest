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

# 初始化状态标志位
glo_is_init = True

# 相机编号、IMU串口信息、主板串口信息
camera_id = 0
imu_com = '/dev/ttyTHS1'
imu_baud = 9600
imu_timeout = 0.05

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
        # mid_height = int(img_height / 2)
        # mid_width = int(img_width / 2)
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
        rgb_half[0:principal_y - 50, :] = (0, 0, 0)
        end_time = time.time()
        time_mess += 'Hal:' + str(round((end_time - start_time) * 1000, 4)) + ';'
        # 灰度并二值化
        start_time = time.time()
        gra_gray = cv2.cvtColor(rgb_half, cv2.COLOR_BGR2GRAY)
        thre_gray, gra_threshold = cv2.threshold(gra_gray, 175, 255, cv2.THRESH_BINARY)
        end_time = time.time()
        time_mess += 'Gra:' + str(round((end_time - start_time) * 1000, 4)) + ';'
        # Canny
        start_time = time.time()
        # rgb_thres = np.zeros((img_height, img_width, 3), np.uint8)  # 创建个全0的黑背景
        # for j in range(0, img_width, 1):
        #     for i in range(0, img_height, 1):
        #         if gra_threshold[i, j] == 255:
        #             rgb_thres[i, j] = (255, 255, 255)
        #         else:
        #             rgb_thres[i, j] = (0, 0, 0)
        # rgb_thres_hough = rgb_thres.copy()
        gra_canny = cv2.Canny(rgb_half, 200, 500)
        end_time = time.time()
        time_mess += 'Can:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 获取水平和垂直线
        start_time = time.time()
        gra_edge_rot = gra_canny.copy()
        gra_edge_rot[0:principal_y, :] = 0
        lines = cv2.HoughLinesP(gra_edge_rot, rho=1.0, theta=np.pi / 180, threshold=150, minLineLength=150,
                                maxLineGap=10)
        end_time = time.time()
        time_mess += 'Hou:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 旋转校正、拉平、融合
        # gra_hough = np.zeros((img_height, img_width), np.uint8)  # 创建个全0的黑背景
        # 垂直线计算偏航角参数
        ver_avg = 0.0
        ver_weight = 0.0
        ver_sum = 0.0
        angles_ver = [0.0]
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
                        if CVFunc.getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 100.0:
                            # cv2.line(gra_hough, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                            # 根据偏航角进行旋转
                            h1 = CVFunc.calc_horizontal(y1_p, model_F, model_W, model_a, model_b)
                            h2 = CVFunc.calc_horizontal(y2_p, model_F, model_W, model_a, model_b)
                            w1 = CVFunc.calc_vertical((x1_p - principal_x), y1_p, model_F, model_W, model_a, model_b)
                            w2 = CVFunc.calc_vertical((x2_p - principal_x), y2_p, model_F, model_W, model_a, model_b)
                            # 水平距离200、垂直距离100以内
                            if (h1 < 200 or h2 < 200) and (abs(w1) < 200 or abs(w2) < 200):
                                if h1 != h2:
                                    angle_tan = np.arctan((w1 - w2) / (h2 - h1)) * 57.29577
                                else:
                                    angle_tan = 90.0
                                if abs(angle_tan) < 45:
                                    ver_sum += angle_tan
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
                # if num_hor > 0:
                #     temp_avg = hor_sum / num_hor
                #     for angle_hor in angles_hor:
                #         if abs(temp_avg - angle_hor) < 10.0:
                #             hor_avg += angle_hor
                #             hor_weight += 1
                #     if hor_weight > 0:
                #         hor_avg = hor_avg / hor_weight

                # 根据垂直线角度和水平线角度，输出偏航角yaw_avg，统一为偏左为负，偏右为正
                # 如果计算了偏航角，则反馈给IMU进程和计算进程
                if ver_avg != 0.0:
                    yaw_avg = ver_avg
                    q_yi.put(yaw_avg)
                    q_yi.get() if q_yi.qsize() > 1 else time.sleep(0.005)
                    q_ym.put(round(yaw_avg, 2))
                    q_ym.get() if q_ym.qsize() > 1 else time.sleep(0.005)
                # elif hor_avg != 0.0:
                #     yaw_avg = hor_avg - 90.0
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
                                    elif ((400 < h1 or h2 < 800) and abs(w1 - w2) < 200) or (
                                            h1 or h2 <= 400 and abs(w1 - w2) < 75):
                                        if angle_tan <= -45.0:
                                            angle_tan = 180.0 + angle_tan
                                        angle_tan = angle_tan - 90.0

                                        if abs(angle_tan - yaw_avg) < 10.0:
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
                                if 0.0 < line_l[0] - rect_l[1] <= 25.0:
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
                                    for i in range(1, 6, 1):
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
                    # 合并左侧相交的矩形或直线
                    if len(rectangles_left) > 1:
                        rectangles_left.sort(reverse=False)
                        num_rect_l = len(rectangles_left)
                        i = 0
                        while i < num_rect_l - 2:
                            j = i + 1
                            while j < num_rect_l - 1 and j <= i + 2:
                                if rectangles_left[i][0] - 2 <= rectangles_left[j][0] <= rectangles_left[i][1] + 2:
                                    rectangles_left[i][0] = min(rectangles_left[i][0], rectangles_left[j][0])
                                    rectangles_left[i][1] = max(rectangles_left[i][1], rectangles_left[j][1])
                                    rectangles_left[i][2] = min(rectangles_left[i][2], rectangles_left[j][2])
                                    rectangles_left[i][3] = max(rectangles_left[i][3], rectangles_left[j][3])
                                    del rectangles_left[j]
                                    num_rect_l = num_rect_l - 1
                                else:
                                    j += 1
                            i += 1

                    # 从右向左，根据右侧线段建立右侧矩形
                    if num_dis_r > 0:
                        lines_right.sort(reverse=True)
                        rectangles_right[0] = [lines_right[0][0], lines_right[0][0], lines_right[0][1],
                                               lines_right[0][2]]
                        for line_r in lines_right:
                            has_change = False
                            for rect_r in rectangles_right:
                                if 0.0 < rect_r[0] - line_r[0] <= 25.0:
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
                                    for i in range(1, 6, 1):
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
                    # 合并右侧相交的矩形或直线
                    if len(rectangles_right) > 1:
                        num_rect_r = len(rectangles_right)
                        i = 0
                        while i < num_rect_r - 2:
                            j = i + 1
                            while j < num_rect_r - 1 and j <= i + 2:
                                if rectangles_right[i][0] - 2 <= rectangles_right[j][1] <= rectangles_right[i][1] + 2:
                                    rectangles_right[i][0] = min(rectangles_right[i][0], rectangles_right[j][0])
                                    rectangles_right[i][1] = max(rectangles_right[i][1], rectangles_right[j][1])
                                    rectangles_right[i][2] = min(rectangles_right[i][2], rectangles_right[j][2])
                                    rectangles_right[i][3] = max(rectangles_right[i][3], rectangles_right[j][3])
                                    del rectangles_right[j]
                                    num_rect_r = num_rect_r - 1
                                else:
                                    j += 1
                            i += 1

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
                    # 合并相交的矩形或直线
                    if len(rectangles_front) > 1:
                        num_rect_f = len(rectangles_front)
                        i = 0
                        while i < num_rect_f - 2:
                            j = i + 1
                            while j < num_rect_f - 1 and j <= i + 2:
                                if (rectangles_front[i][2] - 5 <= rectangles_front[j][2] <= rectangles_front[i][3] + 5) \
                                        or (rectangles_front[i][2] - 5 <= rectangles_front[j][3] <= rectangles_front[i][
                                    3] + 5):
                                    rectangles_front[i][0] = min(rectangles_front[i][0], rectangles_front[j][0])
                                    rectangles_front[i][1] = max(rectangles_front[i][1], rectangles_front[j][1])
                                    rectangles_front[i][2] = min(rectangles_front[i][2], rectangles_front[j][2])
                                    rectangles_front[i][3] = max(rectangles_front[i][3], rectangles_front[j][3])
                                    del rectangles_front[j]
                                    num_rect_f = num_rect_f - 1
                                else:
                                    j += 1
                            i += 1
                    end_time = time.time()
                    time_mess += 'Ret:' + str(round((end_time - start_time) * 1000, 4)) + ';'
        except Exception as e:
            print(e)
            print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
            print(f"error line:{e.__traceback__.tb_lineno}")

        # 画相机中心十字
        start_time = time.time()
        # cv2.line(rgb_show, (principal_x, 0), (principal_x, img_height), (255, 255, 0), 1)
        # cv2.line(rgb_show, (0, principal_y), (img_width, principal_y), (255, 255, 0), 1)
        # cv2.circle(rgb_show, (principal_x, principal_y), 5, (255, 255, 0), 3)
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
        q_temp.put(gra_threshold)
        if q_temp.qsize() > 1:
            q_temp.get()
        # cv2.imshow('Test', rgb_show)
        # cv2.waitKey(1)

        # cv2.imwrite(file_address + 'C' + str(cap_id) + '-' + str_Time + '.jpg', rgb_frame)
        # cv2.imwrite(file_address + 'D' + str(cap_id) + '-' + str_Time + '.jpg', rgb_rot)

        # 保存txt，传输数据
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


# 融合前后单目、超声和IMU，快速更新四向和偏航角
def multi_calc(q_img, q_f, q_l, q_r, q_y, q_i, q_d, q_c, file_address, q_temp):
    cv2.waitKey(1000)
    print('calc start')

    # 根据相机编号分配模型参数
    global para_lines
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

    start_time = time.time()
    end_time = time.time()
    time_mess = round((end_time - start_time) * 1000, 0)

    # show_width = 640
    # show_height = 360
    # show_width = 480
    # show_height = 270
    show_width = 960
    show_height = 540
    half_width = int(show_width / 2)
    half_height = int(show_height / 2)

    # rgb_cam = np.zeros((show_height, show_width, 3), np.uint8)

    while True:
        if not q_img.empty():
            print('Get Img')
            break
        else:
            print('No Image')
            cv2.waitKey(200)

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
            start_time_calc = time.time()

            # 临时缓存，提取二值图
            gra_temp = q_temp.get()
            gra_threshold = cv2.resize(gra_temp, (show_width, show_height))
            rgb_show_1 = cv2.cvtColor(gra_threshold, cv2.COLOR_GRAY2BGR)

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
                rgb_tmp, num_rect_f, num_line_f, dis_f = CVFunc.draw_rectangles(False, rect_f, rgb_tmp, 15.0, cam_yaw, model_F, model_W, model_a, model_b, principal_x)
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
                rgb_tmp, num_rect_l, num_line_l, dis_l = CVFunc.draw_rectangles(True, rect_l, rgb_tmp, 15.0, cam_yaw, model_F, model_W, model_a, model_b, principal_x)
                if dis_l[0][0] != 0.0:
                    dis_l.sort(reverse=False)
                    if cam_yaw > 0.0:
                        side_left = round((dis_l[0][0] - vehicle_front * math.sin(
                            math.radians(abs(cam_yaw))) - vehicle_left * math.sin(math.radians(90.0 - abs(cam_yaw)))), 1)
                    else:
                        side_left = round((dis_l[0][0] + vehicle_front * math.sin(
                            math.radians(abs(cam_yaw))) - vehicle_left * math.sin(math.radians(90.0 - abs(cam_yaw)))), 1)
                else:
                    side_left = -999.9
            else:
                side_left = -999.9

            # 前向右侧垂直框
            if not q_r.empty():
                is_r = True
                rect_r = q_r.get()
                rgb_tmp, num_rect_r, num_line_r, dis_r = CVFunc.draw_rectangles(True, rect_r, rgb_tmp, 15.0, cam_yaw, model_F, model_W, model_a, model_b, principal_x)
                if dis_r[0][0] != 0.0:
                    dis_r.sort(reverse=True)
                    if cam_yaw >= 0:
                        side_right = round((dis_r[0][1] + vehicle_front * math.sin(
                            math.radians(abs(cam_yaw))) - vehicle_right * math.sin(math.radians(90.0 - abs(cam_yaw)))), 1)
                    else:
                        side_right = round((dis_r[0][1] - vehicle_front * math.sin(
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
            if is_l and dis_l[0][0] != 0.0:
                dis_l.sort(reverse=False)
                for i in range(0, len(dis_l), 1):
                    if i == 0:
                        temp_x = int(half_width + (half_width * dis_l[i][0] / 500))
                        cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (0, 0, 255), 1)
                        temp_x = int(half_width + (half_width * dis_l[i][1] / 500))
                        cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (0, 0, 255), 1)
                    else:
                        temp_x = int(half_width + (half_width * dis_l[i][0] / 500))
                        cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (255, 0, 0), 1)
                        temp_x = int(half_width + (half_width * dis_l[i][1] / 500))
                        cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (255, 0, 0), 1)
            if is_r and dis_r[0][0] != 0.0:
                dis_r.sort(reverse=True)
                for i in range(0, len(dis_r), 1):
                    if i == 0:
                        temp_x = int(half_width + (half_width * dis_r[i][0] / 500))
                        cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (0, 0, 255), 1)
                        temp_x = int(half_width + (half_width * dis_r[i][1] / 500))
                        cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (0, 0, 255), 1)
                    else:
                        temp_x = int(half_width + (half_width * dis_r[i][0] / 500))
                        cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (255, 0, 0), 1)
                        temp_x = int(half_width + (half_width * dis_r[i][1] / 500))
                        cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (255, 0, 0), 1)

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
            str_0 = str(time_mess)
            if is_y:
                str_0 += '  Y:' + str(cam_yaw)
            else:
                str_0 += '  Y:N/A'
            if is_f and dis_f[0][0] != 0.0:
                dis_f.sort(reverse=True)
                str_0 += '  F:' + str(int(dis_f[0][0]))
            else:
                str_0 += '  F:N/A'
            if is_l and dis_l[0][0] != 0.0:
                str_1 = 'L:' + str(int(dis_l[0][0])) + ',' + str(int(dis_l[0][1])) + '=' + str(int(dis_l[0][1] - dis_l[0][0]))
            else:
                str_1 = 'L:N/A'
            if is_r and dis_r[0][0] != 0.0:
                str_2 = 'R:' + str(int(dis_r[0][1])) + ',' + str(int(dis_r[0][0])) + '=' + str(int(dis_r[0][1] - dis_r[0][0]))
            else:
                str_2 = 'R:N/A'

            str_3 = 'Yaw:' + str(imu_yaw) + '  Roll:' + str(imu_roll) + '  Pitch:' + str(imu_pitch)

            # str_3 = str(ac_ctl[1])
            # if ac_ctl[0] == 0:
            #     str_3 += '  Stop'
            # elif ac_ctl[0] == 1:
            #     str_3 += '  Front'
            # elif ac_ctl[0] == 2:
            #     str_3 += '  Back'
            # elif ac_ctl[0] == 3:
            #     str_3 += '  Left'
            # elif ac_ctl[0] == 4:
            #     str_3 += '  Right'

            cv2.putText(rgb_show_data, str_0, (0, int(show_height / 5) - 5), cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 255, 255),
                        1)
            cv2.putText(rgb_show_data, str_1, (0, int(show_height * 2 / 5) - 5), cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 255, 255),
                        1)
            cv2.putText(rgb_show_data, str_2, (0, int(show_height * 3 / 5) - 5), cv2.FONT_HERSHEY_COMPLEX, 0.6,
                        (255, 255, 255), 1)
            cv2.putText(rgb_show_data, str_3, (0, int(show_height * 4 / 5) - 5), cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 255, 255), 1)

            # 4图拼接
            rgb_show_0 = cv2.resize(rgb_tmp, (show_width, show_height))
            rgb_mix = np.zeros(((show_height * 2), (show_width * 2), 3), np.uint8)
            rgb_mix[0:show_height, 0:show_width] = rgb_show_0
            rgb_mix[0:show_height, show_width:(show_width * 2)] = rgb_show_line
            rgb_mix[show_height:(show_height * 2), 0:show_width] = rgb_show_1
            rgb_mix[show_height:(show_height * 2), show_width:(show_width * 2)] = rgb_show_data
            cv2.imshow('Show', rgb_mix)

            end_time_calc = time.time()
            time_mess_calc = round((end_time_calc - start_time_calc) * 1000, 0)
            print(time_mess_calc)

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
    global glo_is_init

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
        hex_correctDis = CVFunc.set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_34)
        se.write(hex_correctDis)
        is_read, ret_list = read_message(hex_correctDis)
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
            cmd_2_corSpeed = CVFunc.trans_speed(str(correct_speed))
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
            cam_yaw = dis_list[3]

        if cam_yaw != -999.9:
            rot_yaw = cam_yaw - target_yaw
        else:
            rot_yaw = imu_yaw + diff_cam_imu - target_yaw
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
    glo_is_init = False
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
    cam_yaw = -999.9

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

    # correct_speed = 25
    # cmd_2_corSpeed = CVFunc.trans_speed(str(correct_speed))
    # if move_dis < 0:
    #     hex_correctDis = CVFunc.set_order(cmd_0_head + cmd_1_moveBack + cmd_2_corSpeed + cmd_34)
    # else:
    #     hex_correctDis = CVFunc.set_order(cmd_0_head + cmd_1_moveFront + cmd_2_corSpeed + cmd_34)
    # move_times = int(abs(move_dis) * 2 / 10)    # 30度角，移动距离需要是偏移量的2倍
    # # 根据距离，发送动作命令
    # for i in range(0, move_times, 1):
    #     se.write(hex_correctDis)
    #     is_read, ret_list = read_message(hex_correctDis)
    #     if is_read:
    #         q_c.put(ret_list)
    #         q_c.get() if q_c.qsize() > 1 else time.sleep(0.001)
    #         q_ci.put(ret_list)
    #         q_ci.get() if q_ci.qsize() > 1 else time.sleep(0.001)
    #     # str_send = binascii.b2a_hex(hex_correctDis).decode('utf-8')
    #     # str_Time_cy = datetime.datetime.now().strftime('%H:%M:%S.%f')
    #     # file_rec = open(file_address + 'Control.txt', 'a')
    #     # file_rec.write(str_Time_cy + ';CorrectYaw;s;' + str_send + ';\n')
    #     # file_rec.close()
    #     # 等待串口发送
    #     cv2.waitKey(40)
    #     # 读取主板反馈
    #     hex_rec = se.readline()
    #     if hex_rec:
    #         pass
    #         # 收到反馈
    #         # str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
    #         # file_rec = open(file_address + 'Control.txt', 'a')
    #         # file_rec.write(str_Time_cy + ';CorrectYaw;r;' + str_rec + ';\n')
    #         # file_rec.close()
    #     else:
    #         # 累计无反馈次数，继续执行
    #         loop_nofeedback += 1
    #         print('无反馈', str(loop_nofeedback))
    #         if loop_nofeedback >= threshold_nofeedback:
    #             get_correct_err = 98
    #             print('无反馈，结束运行')
    #     # 获取视觉数据
    #     if not q_d.empty():
    #         dis_list = q_d.get()
    #         side_l = dis_list[1]
    #     # 获取偏航角
    #     if not q_i.empty():
    #         imu_list = q_i.get()
    #         imu_yaw = -imu_list[2]
    #     # 如果调整过程中偏航，则保持-30度角
    #     if abs(imu_yaw + 30.0) > 5.0:
    #         # 左旋30度
    #         get_correct_err = correct_yaw(imu_yaw, -30.0, cmd_34, q_i, q_d, q_c, q_ci, file_address)
    #         # 无反馈退出
    #         if get_correct_err == 98:
    #             return get_correct_err

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
            side_temp = dis_list[1]
            if side_temp != -999.9:
                side_l = side_temp
            side_temp = dis_list[3]
            if side_temp != -999.9:
                cam_yaw = side_temp
        # 获取偏航角
        if not q_i.empty():
            imu_list = q_i.get()
            imu_yaw = -imu_list[2]

    move_dis = round(side_l - target_dis, 2)
    # while abs(move_dis) > 10.0:
    while target_count <= 5:
        print(move_dis)

        if target_count == 0:
            correct_speed = 5
            cmd_2_corSpeed = CVFunc.trans_speed(str(correct_speed))
            if move_dis < 0:
                hex_correctDis = CVFunc.set_order(cmd_0_head + cmd_1_moveBack + cmd_2_corSpeed + cmd_34)
            else:
                hex_correctDis = CVFunc.set_order(cmd_0_head + cmd_1_moveFront + cmd_2_corSpeed + cmd_34)
        else:
            hex_correctDis = CVFunc.set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_34)
        # 根据距离，发送动作命令
        se.write(hex_correctDis)
        is_read, ret_list = read_message(hex_correctDis)
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
        # 获取偏航角
        if not q_i.empty():
            imu_list = q_i.get()
            imu_yaw = -imu_list[2]
        # 获取视觉数据
        if not q_d.empty():
            dis_list = q_d.get()
            side_l = dis_list[1]
            cam_yaw = dis_list[3]
        # 如果调整过程中偏航，则保持-30度角
        if cam_yaw != -999.9:
            now_yaw = cam_yaw
        else:
            now_yaw = imu_yaw
        if abs(now_yaw + 30.0) > 5.0:
            # 左旋30度
            get_correct_err = correct_yaw(now_yaw, -30.0, cmd_34, q_i, q_d, q_c, q_ci, file_address)
            # 无反馈退出
            if get_correct_err == 98:
                return get_correct_err
        # 获取视觉数据，更新剩余距离
                # 获取视觉数据
        if not q_d.empty():
            dis_list = q_d.get()
            side_l = dis_list[1]
            cam_yaw = dis_list[3]
        if side_l != -999.9:
            move_dis = side_l - target_dis
        else:
            if move_dis < 0:
                move_dis = move_dis + 2
            else:
                move_dis = move_dis - 2

        if move_dis <= 10:      # 如果相差距离不大于10，则认为已完成，停止
            target_count += 1
        else:
            target_count = 0

    # 获取偏航角
    if not q_i.empty():
        imu_list = q_i.get()
        imu_yaw = -imu_list[2]
    # 获取视觉数据
    if not q_d.empty():
        dis_list = q_d.get()
        side_l = dis_list[1]
        cam_yaw = dis_list[3]
    # 如果调整过程中偏航，则保持-30度角
    if cam_yaw != -999.9:
        now_yaw = cam_yaw
    else:
        now_yaw = imu_yaw

    get_correct_err = correct_yaw(now_yaw, 0.0, cmd_34, q_i, q_d, q_c, q_ci, file_address)
    # 无反馈退出
    if get_correct_err == 98:
        return get_correct_err


# 前后清洗
def go_wash(is_front, cmd_2, cmd_34, q_i, q_c, q_ci, q_d, lock_ser, file_address, loop_times, wash_left, wash_front):
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

    side_f = -999.9
    side_l = -999.9
    side_r = -999.9
    cam_yaw = -999.9

    # 获取视觉测距
    if not q_d.empty():
        dis_list = q_d.get()
        side_f = dis_list[0]
        side_l = dis_list[1]
        side_r = dis_list[2]
        cam_yaw = dis_list[3]
    # 获取偏航角
    if not q_i.empty():
        imu_list = q_i.get()
        imu_roll = imu_list[0]
        imu_pitch = imu_list[1]
        imu_yaw = -imu_list[2]

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
                    q_ci.put(ret_list)
                    q_ci.get() if q_ci.qsize() > 1 else time.sleep(0.001)
                # 主板控制命令存入txt
                # str_send = binascii.b2a_hex(hex_wash).decode('utf-8')
                # str_Time_wash = datetime.datetime.now().strftime('%H:%M:%S.%f')
                # file_rec = open(file_address + 'Control.txt', 'a')
                # file_rec.write(str_Time_wash + ';wash;s;' + str_send + ';\n')
                # file_rec.close()

                # 等待串口发送
                cv2.waitKey(40)

                # 获取视觉测距
                if not q_d.empty():
                    dis_list = q_d.get()
                    side_f = dis_list[0]
                    side_l = dis_list[1]
                    side_r = dis_list[2]
                    cam_yaw = dis_list[3]
                # 获取偏航角
                if not q_i.empty():
                    imu_list = q_i.get()
                    imu_roll = imu_list[0]
                    imu_pitch = imu_list[1]
                    imu_yaw = -imu_list[2]

                # 读取串口反馈
                hex_rec = se.readline()

                # 获取视觉测距
                if not q_d.empty():
                    dis_list = q_d.get()
                    side_f = dis_list[0]
                    side_l = dis_list[1]
                    side_r = dis_list[2]
                    cam_yaw = dis_list[3]
                # 获取偏航角
                if not q_i.empty():
                    imu_list = q_i.get()
                    imu_roll = imu_list[0]
                    imu_pitch = imu_list[1]
                    imu_yaw = -imu_list[2]
                if cam_yaw != -999.9:
                    now_yaw = cam_yaw
                else:
                    now_yaw = imu_yaw

                if hex_rec:
                    # 收到反馈，跳出反馈循环，累加次数
                    loop_num += 1
                    no_feedback = False
                    # 主板传感数据存入txt
                    # str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
                    # file_rec = open(file_address + 'Control.txt', 'a')
                    # file_rec.write(str_Time_wash + ';wash;r;' + str_rec + ';\n')
                    # file_rec.close()

                    if is_front and side_f != -999.9 and side_f < wash_front:
                        print('到达前端边界')
                        get_correct = 1
                        return get_correct

                    if side_l != -999.9 and abs(side_l - wash_left) > 50.0:
                        print('与原航线偏移' + str(round(wash_left - side_l, 1)) + '毫米')
                        get_correct = correct_deviation(side_l, wash_left, cmd_34, q_i, q_d, q_c, q_ci, file_address)
                        if get_correct == 98:
                            return get_correct
                        print('完成偏移校正，继续清洗')
                    elif abs(now_yaw) > 10.0:     # 清洗偏航，执行偏航校正
                        print('与原方向偏航' + str(now_yaw) + '度')
                        get_correct = correct_yaw(now_yaw, 0.0, cmd_34, q_i, q_d, q_c, q_ci, file_address)  # 偏航校正
                        if get_correct == 98:
                            return get_correct
                        print('完成偏航校正，继续清洗')
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

    print('Auto Start')

    # 设置移动速度
    cmd_2_moveSpeed = CVFunc.trans_speed('30')
    # 设置旋转速度
    cmd_2_rotatePalstance = CVFunc.trans_speed('10')
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


    # 设置清洗参数
    loop_time = 2  # 前进+后退算作1次
    wash_loops = 70     # 设置前后清洗的移动距离限定，-1代表无限定。每次200ms，5次相当于1秒
    cmd_2_washSpeed = CVFunc.trans_speed('20')      # 清洗移动速度
    wash_left_dis = 100.0
    wash_front_dis = 400.0

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

        # 获取视觉数据
        if not q_d.empty():
            dis_list = q_d.get()
            side_f = dis_list[0]
            side_l = dis_list[1]
            side_r = dis_list[2]
            cam_yaw = dis_list[3]
        # 获取偏航角
        if not q_i.empty():
            imu_list = q_i.get()
            imu_roll = imu_list[0]
            imu_pitch = imu_list[1]
            imu_yaw = -imu_list[2]
        if cam_yaw != -999.9 or 0.0:
            now_yaw = cam_yaw
        else:
            now_yaw = imu_yaw

        # 如果角度有偏航，进行校正
        if abs(now_yaw) > 1.0:
            print('校正偏航')
            get_state = correct_yaw(now_yaw, 0.0, board_cmd, q_i, q_d, q_c, q_ci, file_address)
            # 无反馈退出
            if get_state == 98:
                print('无反馈，结束运行')
                no_feedBack = True
            print('校正完成')
        else:
            print('无偏航')
        if no_feedBack:
            continue
        # # 获取视觉数据
        # if not q_d.empty():
        #     dis_list = q_d.get()
        #     side_f = dis_list[0]
        #     side_l = dis_list[1]
        #     side_r = dis_list[2]
        #     cam_yaw = dis_list[3]
        #
        # # 如果距离轨迹有偏移，进行校正
        # if side_l != -999.9:
        #     if abs(side_l - wash_left_dis) > 10:
        #         print('校正偏移')
        #         get_state = correct_deviation(side_l, wash_left_dis, board_cmd, q_i, q_d, q_c, q_ci, file_address)
        #         # 无反馈退出
        #         if get_state == 98:
        #             print('无反馈，结束运行')
        #             no_feedBack = True
        #         print('校正完成')
        #     else:
        #         print('无偏移')
        # else:
        #     print('未找到左边界')
        # if no_feedBack:
        #     continue
        # # 进入自动控制
        # print('进入自动控制')
        # get_state = 0
        # # 建立通信，开始执行
        # for loop_num in range(0, loop_time, 1):
        #     # 清洗前进
        #     print('清洗前行，次数', str(wash_loops))
        #     get_state = go_wash(True, cmd_2_washSpeed, board_cmd, q_i, q_c, q_ci, q_d, lock_ser, file_address, wash_loops, wash_left_dis, wash_front_dis)
        #     if get_state == 98:
        #         no_feedBack = True
        #         break
        #     elif get_state > 90:
        #         print('故障')
        #
        #     # 到达位置，停止并调整
        #     glo_is_init = True
        #     print('到达前端')
        #     for i in range(0, 3, 1):
        #         hex_correctDis = CVFunc.set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + board_cmd)
        #         se.write(hex_correctDis)
        #         is_read, ret_list = read_message(hex_correctDis)
        #         if is_read:
        #             q_c.put(ret_list)
        #             q_c.get() if q_c.qsize() > 1 else time.sleep(0.001)
        #             q_ci.put(ret_list)
        #             q_ci.get() if q_ci.qsize() > 1 else time.sleep(0.001)
        #         # str_send = binascii.b2a_hex(hex_correctDis).decode('utf-8')
        #         # str_Time_cy = datetime.datetime.now().strftime('%H:%M:%S.%f')
        #         # file_rec = open(file_address + 'Control.txt', 'a')
        #         # file_rec.write(str_Time_cy + ';CorrectYaw;s;' + str_send + ';\n')
        #         # file_rec.close()
        #         # 等待串口发送
        #         cv2.waitKey(40)
        #         # 读取主板反馈
        #         hex_rec = se.readline()
        #         if hex_rec:
        #             pass
        #             # 收到反馈
        #             # str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
        #             # file_rec = open(file_address + 'Control.txt', 'a')
        #             # file_rec.write(str_Time_cy + ';CorrectYaw;r;' + str_rec + ';\n')
        #             # file_rec.close()
        #         # 获取视觉数据
        #         if not q_d.empty():
        #             dis_list = q_d.get()
        #             side_f = dis_list[0]
        #             side_l = dis_list[1]
        #             side_r = dis_list[2]
        #             cam_yaw = dis_list[3]
        #         # 获取偏航角
        #         if not q_i.empty():
        #             imu_list = q_i.get()
        #             imu_roll = imu_list[0]
        #             imu_pitch = imu_list[1]
        #             imu_yaw = -imu_list[2]
        #         if cam_yaw != -999.9:
        #             now_yaw = cam_yaw
        #         else:
        #             now_yaw = imu_yaw
        #         # 如果偏航或者视觉偏航角与IMU偏航角相差较大，进行旋转后置零
        #         if (cam_yaw != -999.9 and abs(cam_yaw - imu_yaw) > 1.0) or abs(now_yaw) > 1.0:
        #             get_state = correct_yaw(cam_yaw, 0.0, board_cmd, q_i, q_d, q_c, q_ci, file_address)
        #             # 无反馈退出
        #             if get_state == 98:
        #                 print('无反馈，结束运行')
        #                 no_feedBack = True
        #     glo_is_init = False
        #
        #     # 清洗后退
        #     print('清洗后退，次数', str(wash_loops))
        #     get_state = go_wash(False, cmd_2_washSpeed, board_cmd, q_i, q_c, q_ci, q_d, lock_ser, file_address, wash_loops, wash_left_dis, wash_front_dis)
        #     if get_state == 98:
        #         no_feedBack = True
        #         break
        #     elif get_state > 90:
        #         print('故障')
        #
        #     # 到达位置，停止并调整
        #     glo_is_init = True
        #     print('到达后端')
        #     for i in range(0, 3, 1):
        #         hex_correctDis = CVFunc.set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + board_cmd)
        #         se.write(hex_correctDis)
        #         is_read, ret_list = read_message(hex_correctDis)
        #         if is_read:
        #             q_c.put(ret_list)
        #             q_c.get() if q_c.qsize() > 1 else time.sleep(0.001)
        #             q_ci.put(ret_list)
        #             q_ci.get() if q_ci.qsize() > 1 else time.sleep(0.001)
        #         # str_send = binascii.b2a_hex(hex_correctDis).decode('utf-8')
        #         # str_Time_cy = datetime.datetime.now().strftime('%H:%M:%S.%f')
        #         # file_rec = open(file_address + 'Control.txt', 'a')
        #         # file_rec.write(str_Time_cy + ';CorrectYaw;s;' + str_send + ';\n')
        #         # file_rec.close()
        #         # 等待串口发送
        #         cv2.waitKey(40)
        #         # 读取主板反馈
        #         hex_rec = se.readline()
        #         if hex_rec:
        #             pass
        #             # 收到反馈
        #             # str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
        #             # file_rec = open(file_address + 'Control.txt', 'a')
        #             # file_rec.write(str_Time_cy + ';CorrectYaw;r;' + str_rec + ';\n')
        #             # file_rec.close()
        #         # 获取视觉数据
        #         if not q_d.empty():
        #             dis_list = q_d.get()
        #             side_f = dis_list[0]
        #             side_l = dis_list[1]
        #             side_r = dis_list[2]
        #             cam_yaw = dis_list[3]
        #         # 获取偏航角
        #         if not q_i.empty():
        #             imu_list = q_i.get()
        #             imu_roll = imu_list[0]
        #             imu_pitch = imu_list[1]
        #             imu_yaw = -imu_list[2]
        #         if cam_yaw != -999.9:
        #             now_yaw = cam_yaw
        #         else:
        #             now_yaw = imu_yaw
        #         # 如果偏航或者视觉偏航角与IMU偏航角相差较大，进行旋转后置零
        #         if (cam_yaw != -999.9 and abs(cam_yaw - imu_yaw) > 1.0) or abs(now_yaw) > 1.0:
        #             get_state = correct_yaw(cam_yaw, 0.0, board_cmd, q_i, q_d, q_c, q_ci, file_address)
        #             # 无反馈退出
        #             if get_state == 98:
        #                 print('无反馈，结束运行')
        #                 no_feedBack = True
        #     glo_is_init = False
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
    # IMU，偏航、俯仰和翻滚，分发前相机测距、计算和自动控制
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
