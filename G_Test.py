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


# 绘制矩形（2022-11-21）
def draw_rectangles(is_ver, rectangles_list, rgb_draw, rect_threshold, yaw_avg, f, w, a, b, p_x):
    num_rect = 0
    num_line = 0
    dis_line = [[0.0, 0.0]]
    for rect_value in rectangles_list:
        if is_ver:
            if rect_value[0] != rect_value[1]:
                num_rect += 1
                draw_left = rect_value[0]
                draw_right = rect_value[1]
                draw_button = rect_value[2]
                draw_top = rect_value[3]
                xd_lt, yd_lt = CVFunc.points_dis2xy(yaw_avg, draw_left, draw_top, f, w, a, b, p_x)
                xd_lb, yd_lb = CVFunc.points_dis2xy(yaw_avg, draw_left, draw_button, f, w, a, b, p_x)
                xd_rt, yd_rt = CVFunc.points_dis2xy(yaw_avg, draw_right, draw_top, f, w, a, b, p_x)
                xd_rb, yd_rb = CVFunc.points_dis2xy(yaw_avg, draw_right, draw_button, f, w, a, b, p_x)
                if abs(draw_right - draw_left) > rect_threshold:
                    cv2.line(rgb_draw, (xd_lt, yd_lt), (xd_lb, yd_lb), (0, 0, 255), 2)
                    cv2.line(rgb_draw, (xd_lt, yd_lt), (xd_rt, yd_rt), (0, 0, 255), 2)
                    cv2.line(rgb_draw, (xd_rb, yd_rb), (xd_lb, yd_lb), (0, 0, 255), 2)
                    cv2.line(rgb_draw, (xd_rb, yd_rb), (xd_rt, yd_rt), (0, 0, 255), 2)
                    dis_line.append([draw_left, draw_right])
                else:
                    cv2.line(rgb_draw, (xd_lt, yd_lt), (xd_lb, yd_lb), (255, 0, 0), 2)
                    cv2.line(rgb_draw, (xd_lt, yd_lt), (xd_rt, yd_rt), (255, 0, 0), 2)
                    cv2.line(rgb_draw, (xd_rb, yd_rb), (xd_lb, yd_lb), (255, 0, 0), 2)
                    cv2.line(rgb_draw, (xd_rb, yd_rb), (xd_rt, yd_rt), (255, 0, 0), 2)
            else:
                num_line += 1
                xd1_b, yd1_b = CVFunc.points_dis2xy(yaw_avg, rect_value[0], rect_value[2], f, w, a, b, p_x)
                xd2_b, yd2_b = CVFunc.points_dis2xy(yaw_avg, rect_value[0], rect_value[3], f, w, a, b, p_x)
                cv2.line(rgb_draw, (xd1_b, yd1_b), (xd2_b, yd2_b), (0, 255, 0), 2)
        else:
            if rect_value[2] != rect_value[3]:
                num_rect += 1
                draw_left = rect_value[0]
                draw_right = rect_value[1]
                draw_button = rect_value[2]
                draw_top = rect_value[3]
                xd_lt, yd_lt = CVFunc.points_dis2xy(yaw_avg, draw_left, draw_top, f, w, a, b, p_x)
                xd_lb, yd_lb = CVFunc.points_dis2xy(yaw_avg, draw_left, draw_button, f, w, a, b, p_x)
                xd_rt, yd_rt = CVFunc.points_dis2xy(yaw_avg, draw_right, draw_top, f, w, a, b, p_x)
                xd_rb, yd_rb = CVFunc.points_dis2xy(yaw_avg, draw_right, draw_button, f, w, a, b, p_x)
                if abs(draw_top - draw_button) > rect_threshold:
                    cv2.line(rgb_draw, (xd_lt, yd_lt), (xd_lb, yd_lb), (0, 0, 255), 2)
                    cv2.line(rgb_draw, (xd_lt, yd_lt), (xd_rt, yd_rt), (0, 0, 255), 2)
                    cv2.line(rgb_draw, (xd_rb, yd_rb), (xd_lb, yd_lb), (0, 0, 255), 2)
                    cv2.line(rgb_draw, (xd_rb, yd_rb), (xd_rt, yd_rt), (0, 0, 255), 2)
                    dis_line.append([draw_top, draw_button])
                else:
                    cv2.line(rgb_draw, (xd_lt, yd_lt), (xd_lb, yd_lb), (255, 0, 0), 2)
                    cv2.line(rgb_draw, (xd_lt, yd_lt), (xd_rt, yd_rt), (255, 0, 0), 2)
                    cv2.line(rgb_draw, (xd_rb, yd_rb), (xd_lb, yd_lb), (255, 0, 0), 2)
                    cv2.line(rgb_draw, (xd_rb, yd_rb), (xd_rt, yd_rt), (255, 0, 0), 2)
            else:
                num_line += 1
                xd1_b, yd1_b = CVFunc.points_dis2xy(yaw_avg, rect_value[0], rect_value[2], f, w, a, b, p_x)
                xd2_b, yd2_b = CVFunc.points_dis2xy(yaw_avg, rect_value[1], rect_value[2], f, w, a, b, p_x)
                cv2.line(rgb_draw, (xd1_b, yd1_b), (xd2_b, yd2_b), (0, 255, 0), 2)

    if len(dis_line) > 1:
        del dis_line[0]
    return rgb_draw, dis_line


# 读取模型参数
file_model = open('LingGuang/Model-360.txt', 'r', encoding='utf-8')
para_lines = file_model.readlines()
file_model.close()

# 初始化状态标志位
glo_is_init = True

# 相机编号、IMU串口信息、主板串口通信
# camera_id = 0
# imu_com = '/dev/ttyTHS1'
# imu_baud = 9600
# imu_timeout = 0.05
# laser_com = '/dev/ttyUSB1'
# laser_baud = 115200
# laser_timeout = 0.005
# se = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

# 机身尺寸、光伏板尺寸
vehicle_left = 118
vehicle_right = 127
vehicle_front = 117
vehicle_width = vehicle_left + vehicle_right
panel_width = 680

# 激光测距阈值
laser_threshold = 17

# 清洗速度及校正阈值
int_washSpeed = 50
wash_thre_yaw = 3.0
wash_thre_deviation = 40.0

# 侧边全局缓存
# manager = multiprocessing.Manager()
# rec_side_left = manager.list()
# rec_side_right = manager.list()
rec_side_left = [0.0]
rec_side_right = [0.0]

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

start_time_total = time.time()
str_Time = datetime.datetime.now().strftime('%H%M%S-%f')
ret_mess = ''  # 数据信息
time_mess = ''  # 时间信息
# 获取图像及参数
start_time = time.time()
# if not q_c.empty():
#     rgb_frame = q_c.get()
#     loop_num += 1
# else:
#     continue
# rgb_frame = cv2.imread('./TestData/20221206-1655/C0-165523-932253.jpg')

str_fileHome = './TestData/20221208-1603/'
title_li = os.listdir(str_fileHome)
for title in title_li:
    rgb_frame = cv2.imread(str_fileHome + title)
    img_height = int(rgb_frame.shape[0])
    img_width = int(rgb_frame.shape[1])
    mid_height = int(img_height / 2)
    mid_width = int(img_width / 2)
    rgb_hough = rgb_frame.copy()
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
    yaw_avg = 0.0

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
                            # cv2.line(gra_hough, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                            # cv2.putText(gra_hough, str(round(angle_tan, 2)), (x_mid, y_mid),
                            #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255, 1)

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
            # # 如果计算了偏航角，则反馈给IMU进程和计算进程
            # if ver_avg != 0.0 and num_yaw_l != 0 and num_yaw_r != 0:
            #     q_yi.put(yaw_avg)
            #     q_yi.get() if q_yi.qsize() > 1 else time.sleep(0.005)
            #     q_ym.put(round(yaw_avg, 2))
            #     q_ym.get() if q_ym.qsize() > 1 else time.sleep(0.005)
            # else:
            #     yaw_avg = 0.0
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
                                        cv2.line(rgb_thres_hough, (x1_p, y1_p), (x2_p, y2_p), (255, 0, 0), 1)
                                        cv2.line(rgb_hough, (x1_p, y1_p), (x2_p, y2_p), (255, 0, 0), 1)
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
                                        cv2.line(rgb_thres_hough, (x1_p, y1_p), (x2_p, y2_p), (0, 255, 0), 1)
                                        cv2.line(rgb_hough, (x1_p, y1_p), (x2_p, y2_p), (0, 255, 0), 1)
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
        # print(yaw_avg)
        if len(rectangles_left) > 1:
            rgb_show, dis_l = draw_rectangles(True, rectangles_left, rgb_show, 15.0, yaw_avg, model_F, model_W, model_a, model_b,
                                                    principal_x)
        if len(rectangles_right) > 1:
            rgb_show, dis_r = draw_rectangles(True, rectangles_right, rgb_show, 15.0, yaw_avg, model_F, model_W, model_a, model_b,
                                                    principal_x)

        rgb_hough_0 = cv2.resize(rgb_frame, (img_width * 2, img_height * 2))
        rgb_hough_1 = cv2.resize(rgb_show, (img_width * 2, img_height * 2))
        rgb_hough_2 = cv2.resize(rgb_thres_hough, (img_width * 2, img_height * 2))
        cv2.imshow('Frame', rgb_hough_0)
        cv2.imshow('show', rgb_hough_1)
        cv2.imshow('hough', rgb_hough_2)
        cv2.waitKey(100)
    # 保存txt，传输数据
    start_time = time.time()
    # if len(rectangles_left) > 1:
    #     q_l.put(rectangles_left)
    #     if q_l.qsize() > 1:
    #         q_l.get()
    # if len(rectangles_right) > 1:
    #     q_r.put(rectangles_right)
    #     if q_r.qsize() > 1:
    #         q_r.get()
    # if len(rectangles_front) > 1:
    #     q_f.put(rectangles_front)
    #     if q_f.qsize() > 1:
    #         q_f.get()
    # q_img.put(rgb_hough)
    # if q_img.qsize() > 1:
    #     q_img.get()
    # q_temp.put(rgb_thres_hough)
    # if q_temp.qsize() > 1:
    #     q_temp.get()
    # cv2.imwrite(file_address + 'C' + str(cap_id) + '-' + str_Time + '.jpg', rgb_frame)
    # cv2.imwrite(file_address + 'D' + str(cap_id) + '-' + str_Time + '.jpg', rgb_rot)
    # file_rec = open(file_address + 'Dis.txt', 'a')
    end_time = time.time()
    time_mess += 'Shw:' + str(round((end_time - start_time) * 1000, 4)) + ';'
    end_time_total = time.time()
    time_mess += 'All:' + str(round((end_time_total - start_time_total) * 1000, 4)) + ';\n'
    # if len(time_mess) > 0:
    #     file_rec.write('Tpy:Timer;' + time_mess)
    # file_rec.close()

