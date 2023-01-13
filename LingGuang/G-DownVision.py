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


def get_hough(gra_img):
    # 获取水平和垂直线
    start_time = time.time()

    gra_edge_rot = gra_img.copy()
    # gra_edge_rot[0:principal_y, :] = 0
    lines = cv2.HoughLinesP(gra_edge_rot, rho=1.0, theta=np.pi / 180, threshold=150, minLineLength=150, maxLineGap=10)
    end_time = time.time()
    time_mess = 'Hou:' + str(round((end_time - start_time) * 1000, 4)) + ';'


    # 旋转校正、拉平、融合
    start_time = time.time()
    gra_lines = np.zeros((img_height, img_width), np.uint8)  # 创建个全0的黑背景
    gra_hough = np.zeros((img_height, img_width), np.uint8)  # 创建个全0的黑背景
    yaw_avg = 0.0
    yaw_weight = 0.0
    other_avg = 0.0
    other_weight = 0.0

    num_ver = 0
    lines_ver = [[0, 0, 0, 0]]
    num_hor = 0
    lines_hor = [[0, 0, 0, 0]]

    data_ver = np.zeros((0, 3), dtype=float)  # 垂直线数据，0是最接近中轴的w（x）值，1是最接近相机的h（y）值，2是最远离相机的h（y）值

    num_left = 0
    num_right = 0
    dis_l = [0]
    dis_r = [0]
    try:
        if str(type(lines)) != "<class 'NoneType'>":
            if len(lines) > 0:
                # 计算偏航角
                for line in lines:
                    for x1_p, y1_p, x2_p, y2_p in line:
                        if CVFunc.getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 100.0:
                            cv2.line(gra_hough, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                            cv2.line(rgb_rot, (x1_p, y1_p), (x2_p, y2_p), (0, 0, 255), 1)
                            # 根据偏航角进行旋转
                            h1 = CVFunc.calc_horizontal(y1_p, model_F, model_W, model_a, model_b)
                            h2 = CVFunc.calc_horizontal(y2_p, model_F, model_W, model_a, model_b)
                            w1 = CVFunc.calc_vertical((x1_p - principal_x), y1_p, model_F, model_W, model_a, model_b)
                            w2 = CVFunc.calc_vertical((x2_p - principal_x), y2_p, model_F, model_W, model_a, model_b)

                            # 水平距离1米以内
                            # if h1 < 2000 and h2 < 2000 and w1 < 2000 and w2 < 2000:
                            if h1 < 200 or h2 < 200:
                                if h1 != h2:
                                    angle_tan = np.arctan((w1 - w2) / (h2 - h1)) * 57.29577
                                else:
                                    angle_tan = 90.0
                                x_mid = int((x1_p + x2_p) / 2)
                                y_mid = int((y1_p + y2_p) / 2)

                                cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                                cv2.putText(gra_lines, str(round(angle_tan, 2)), (x_mid, y_mid),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255, 1)
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
        end_time = time.time()
        time_mess += 'Cal:' + str(round((end_time - start_time) * 1000, 4)) + ';'
    except Exception as e:
        print(e)
        print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
        print(f"error line:{e.__traceback__.tb_lineno}")
    return gra_hough



start_time = time.time()
# 读取模型参数
file_model = open('../Feiyang/Model.txt', 'r', encoding='utf-8')
para_lines = file_model.readlines()
file_model.close()
model_F = float(para_lines[0].strip('\n'))
model_W = float(para_lines[1].strip('\n'))
model_a = float(para_lines[2].strip('\n'))
model_b = float(para_lines[3].strip('\n'))
principal_x = int(para_lines[4].strip('\n'))
principal_y = int(para_lines[5].strip('\n'))

# 连接摄像头
cap = cv2.VideoCapture(0)
cap.set(6, 1196444237)
cap.set(3, 1920)
cap.set(4, 1080)
cap.set(5, 30)
img_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
img_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
mid_height = int(img_height / 2)
mid_width = int(img_width / 2)

end_time = time.time()
time_mess = 'Int:' + str(round((end_time - start_time) * 1000, 4)) + ';'

print(time_mess)

while True:
    start_time_total = time.time()
    str_Time = datetime.datetime.now().strftime('%H%M%S-%f')
    ret_mess = ''  # 数据信息
    time_mess = ''  # 时间信息

    # 获取图像及参数
    start_time = time.time()
    ret, rgb_frame = cap.read()
    rgb_rot = rgb_frame.copy()
    rgb_show = rgb_frame.copy()
    # # 获取偏航角
    # if not q_i.empty():
    #     jy_list = q_i.get()
    #     imu_roll = jy_list[0]
    #     imu_pitch = jy_list[1]
    #     imu_yaw = jy_list[2]
    end_time = time.time()
    time_mess += 'Cap:' + str(round((end_time - start_time) * 1000, 4)) + ';'

    # 保留下半部分的红色区域（Canny提取边界，保留下半部分）
    start_time = time.time()
    rgb_half = rgb_frame.copy()
    # rgb_half[0:principal_y - 50, :] = (0, 0, 0)
    gra_gray = cv2.cvtColor(rgb_half, cv2.COLOR_BGR2GRAY)
    thre_0, gra_threshold_0 = cv2.threshold(gra_gray, 225, 255, cv2.THRESH_BINARY)
    thre_1, gra_threshold_1 = cv2.threshold(gra_gray, 175, 255, cv2.THRESH_BINARY)
    gra_canny = cv2.Canny(rgb_half, 200, 400)
    gra_canny_0 = cv2.Canny(gra_threshold_0, 200, 400)
    gra_canny_1 = cv2.Canny(gra_threshold_1, 200, 400)

    # hsv_half = cv2.cvtColor(rgb_half, cv2.COLOR_BGR2HSV)
    # low_range = np.array([100, 100, 100])
    # high_range = np.array([150, 150, 150])
    # gra_range_l = cv2.inRange(hsv_half, low_range, high_range)
    # low_range = np.array([150, 150, 150])
    # high_range = np.array([255, 255, 255])
    # gra_range_h = cv2.inRange(hsv_half, low_range, high_range)

    end_time = time.time()
    time_mess += 'Can:' + str(round((end_time - start_time) * 1000, 4)) + ';'

    gra_hough_org = get_hough(gra_canny)
    gra_hough_0 = get_hough(gra_canny_0)
    gra_hough_1 = get_hough(gra_canny_1)

    # show_height = int((img_height - principal_y) / 2)
    show_height = int(img_height / 2)
    show_width = int(img_width / 2)

    # show_0 = gra_gray[principal_y:img_height, :]
    # show_1 = gra_canny_0[principal_y:img_height, :]
    # show_2 = gra_canny_1[principal_y:img_height, :]
    # show_3 = gra_canny_2[principal_y:img_height, :]
    show_0 = gra_threshold_0
    show_1 = gra_hough_0
    show_2 = gra_threshold_1
    show_3 = gra_hough_1

    show_0 = cv2.resize(show_0, (show_width, show_height))
    show_1 = cv2.resize(show_1, (show_width, show_height))
    show_2 = cv2.resize(show_2, (show_width, show_height))
    show_3 = cv2.resize(show_3, (show_width, show_height))

    # 4图拼接
    rgb_mix = np.zeros(((show_height * 2), (show_width * 2)), np.uint8)
    rgb_mix[0:show_height, 0:show_width] = show_0
    rgb_mix[0:show_height, show_width:(show_width * 2)] = show_1
    rgb_mix[show_height:(show_height * 2), 0:show_width] = show_2
    rgb_mix[show_height:(show_height * 2), show_width:(show_width * 2)] = show_3
    cv2.imshow('Show', rgb_mix)

    # cv2.imshow('RGB', gra_threshold)

    cv2.waitKey(1)