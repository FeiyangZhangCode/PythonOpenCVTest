import os
import time
import datetime
import shutil
import cv2
import CVFunc
import numpy as np
import math
import CVFunc
import xlwt
import matplotlib.pyplot as plt

file_model = open('Model.txt', 'r', encoding='utf-8')
para_lines = file_model.readlines()
file_model.close()
model_F = float(para_lines[0].strip('\n'))
model_W = float(para_lines[1].strip('\n'))
model_a = float(para_lines[2].strip('\n'))
model_b = float(para_lines[3].strip('\n'))
principal_x = int(para_lines[4].strip('\n'))
principal_y = int(para_lines[5].strip('\n'))



# 计算两点间距离
def getDist_P2P(x1_d, y1_d, x2_d, y2_d):
    distance = math.pow((x1_d - x2_d), 2) + math.pow((y1_d - y2_d), 2)
    distance = math.sqrt(distance)
    return distance


# 提取红色线
def get_red(img_rgb):
    # 提取区域，下半部分
    # img_new = np.zeros((mid_height, img_width, 3), np.uint8)
    # for j in range(0, img_width, 1):
    #     for i in range(0, mid_height, 1):
    #         img_new[i, j] = img_rgb[i + mid_height, j]
    # img_red = img_new.copy()
    img_new = np.zeros((img_height, img_width, 3), np.uint8)
    # 提取红色部分
    b_threshold = 100
    g_threshold = 100
    r_threshold = 130
    b, g, r = cv2.split(img_rgb)
    for j_ut in range(0, img_width, 1):
        for i_ut in range(0, img_height, 1):
            b_value = b[i_ut, j_ut]
            g_value = g[i_ut, j_ut]
            r_value = r[i_ut, j_ut]
            if b_value < b_threshold and g_value < g_threshold and r_value > r_threshold:
                img_new[i_ut, j_ut] = img_rgb[i_ut, j_ut]
            else:
                img_new[i_ut, j_ut] = (0, 0, 0)
    # 二值化，去噪点
    gra_red_temp = cv2.cvtColor(img_new, cv2.COLOR_BGR2GRAY)
    ret, gra_red_temp = cv2.threshold(gra_red_temp, 10, 255, cv2.THRESH_BINARY)
    gra_red_temp = CVFunc.func_noise_1(gra_red_temp, 400)
    return gra_red_temp


# 识别水平线和垂线，分别返回水平、左、右线（2022-08-19）
def get_HoughLinesFLR(gra_edge_lines, rgb_lines, real_angle):
    global model_F, model_W, model_a, model_b, principal_x, principal_y
    # 提取各类型线段
    err_mess = ''
    gra_width_HLP = gra_edge_lines.shape[1]
    mid_width_HLP = int(gra_width_HLP / 2)
    gra_lines = np.zeros((gra_edge_lines.shape[0], gra_edge_lines.shape[1]), np.uint8)  # 创建个全0的黑背景
    y_area = np.zeros([gra_edge_lines.shape[0], 2], np.uint64)  # 0是权重，1是x1和x2中最接近中轴的那一端
    # ver_lines_org = []
    left_lines = []
    right_lines = []
    num_hor = 0
    try:
        lines = cv2.HoughLinesP(gra_edge_lines, rho=1.0, theta=np.pi / 180, threshold=50, minLineLength=50,
                                maxLineGap=20)
        num_line = 0
        num_ver = 0
        num_hor = 0
        angle_ver = [0.0]
        angle_hor = [0.0]
        sum_angle_ver = 0.0
        sum_angle_hor = 0.0

        for line in lines:
            for x1_p, y1_p, x2_p, y2_p in line:
                # cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                if getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 50.0:
                    num_line += 1
                    # cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                    h1 = CVFunc.calc_horizontal(y1_p, model_F, model_W, model_a, model_b)
                    h2 = CVFunc.calc_horizontal(y2_p, model_F, model_W, model_a, model_b)
                    w1 = CVFunc.calc_vertical((x1_p - principal_x), y1_p, model_F, model_W, model_a, model_b)
                    w2 = CVFunc.calc_vertical((x2_p - principal_x), y2_p, model_F, model_W, model_a, model_b)
                    cv2.line(rgb_lines, (x1_p, y1_p), (x2_p, y2_p), (255, 0, 0), 1)
                    cv2.putText(rgb_lines, str(w1) + ',' + str(h1), (x1_p, y1_p), cv2.FONT_HERSHEY_SIMPLEX,
                                0.4, (255, 0, 0), 1)
                    cv2.putText(rgb_lines, str(w2) + ',' + str(h2), (x2_p, y2_p), cv2.FONT_HERSHEY_SIMPLEX,
                                0.4, (255, 0, 0), 1)

                    # if ((x1_p > principal_x) and (x2_p > principal_x)) or (
                    #         (x1_p < principal_x) and (x2_p < principal_x)):
                    #     w12 = abs(w1 - w2)
                    # else:
                    #     w12 = abs(w1 + w2)
                    # h12 = abs(h1 - h2)
                    #
                    # if w12 == 0:
                    #     tan12 = 90.0
                    # else:
                    #     tan12 = np.arctan(h12 / w12) * 57.29577
                    #     cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                    #     cv2.putText(gra_lines, str(round(tan12, 2)), (int((x1_p + x2_p) / 2), int((y1_p + y2_p) / 2)),
                    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255, 1)
                    #     # print(str(round(result, 2)))
                    # if abs(tan12) <= real_angle + 10:
                    #     # cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                    #     # cv2.putText(gra_lines, str(round(tan12, 2)), (int((x1_p + x2_p) / 2), int((y1_p + y2_p) / 2)),
                    #     #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255, 1)
                    #     cv2.line(rgb_lines, (x1_p, y1_p), (x2_p, y2_p), (255, 0, 0), 1)
                    #     cv2.putText(rgb_lines, str(round(tan12 - real_angle, 2)),
                    #                 (int((x1_p + x2_p) / 2), int((y1_p + y2_p) / 2)),
                    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)
                    #     if num_hor == 0:
                    #         angle_hor[0] = round(tan12 - real_angle, 2)
                    #     else:
                    #         angle_hor.append(round(tan12 - real_angle, 2))
                    #     num_hor += 1
                    #     sum_angle_hor += round(tan12 - real_angle, 2)
                    # elif 90.0 >= abs(tan12) > real_angle + 10:
                    #     # cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                    #     # cv2.putText(gra_lines, str(round(abs(90.0 - tan12), 2)), (int((x1_p + x2_p) / 2), int((y1_p + y2_p) / 2)),
                    #     #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255, 1)
                    #     cv2.line(rgb_lines, (x1_p, y1_p), (x2_p, y2_p), (0, 255, 0), 1)
                    #     cv2.putText(rgb_lines, str(round(abs(90.0 - tan12) - real_angle, 2)),
                    #                 (int((x1_p + x2_p) / 2), int((y1_p + y2_p) / 2)),
                    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                    #     if num_ver == 0:
                    #         angle_ver[0] = round(abs(90.0 - tan12) - real_angle, 2)
                    #     else:
                    #         angle_ver.append(round(abs(90.0 - tan12) - real_angle, 2))
                    #     num_ver += 1
                    #     sum_angle_ver += round(abs(90.0 - tan12) - real_angle, 2)
    except Exception as e:
        err_mess += 'Hough lines\n' + str(e) + '\n'

    # cv2.imshow('Hough', rgb_lines)
    return rgb_lines, err_mess


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


# 开始主程序
# 读取模型参数
ret_mess = ''  # 数据信息
err_mess = ''  # 报错信息
time_mess = ''  # 时间信息
ret_value = [0] * 4  # 0是水平线，1是左垂线，2是右垂线，3是超声波

angle_set = 0.0
cap_id = 0
ultra_value = 0.0
width_threshold = 1180
rgb_frame = cv2.imread('./TestData/C0-152403-779391.jpg')
# angle_set = -10.81  # 车头向左偏，需要顺时针旋转，角度为负。车头向右偏，需要逆时针旋转，角度为正。
img_height = int(rgb_frame.shape[0])
img_width = int(rgb_frame.shape[1])
mid_height = int(img_height / 2)
mid_width = int(img_width / 2)

start_time_total = time.time()
str_Time = datetime.datetime.now().strftime('%H:%M:%S.%f')

x = 0
xls = xlwt.Workbook()
sheet = xls.add_sheet('sheet1', cell_overwrite_ok=True)

for minCan in range(40, 220, 20):
    for maxCan in range(100, 450, 50):
        # print('Canny', str(minCan), str(maxCan))
        find_width = False
        rec_width = 0.0
        rec_left = 0.0
        rec_right = 0.0

        rgb_rot = rgb_frame.copy()

        # Canny提取边界，保留下半部分
        start_time = time.time()
        rgb_half = rgb_frame.copy()
        rgb_half[0:principal_y - 50, :] = (0, 0, 0)
        gra_edge = cv2.Canny(rgb_half, minCan, maxCan)
        end_time = time.time()
        # time_mess += 'Canny:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'
        time_Can = round((end_time - start_time) * 1000, 4)

        start_time = time.time()
        gra_edge_rot = gra_edge.copy()
        gra_edge_rot[0:principal_y, :] = 0
        end_time = time.time()
        time_mess += 'Half:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

        cv2.imshow('test', gra_edge_rot)

        # 获取水平和垂直线
        start_time = time.time()
        lines = cv2.HoughLinesP(gra_edge_rot, rho=1.0, theta=np.pi / 180, threshold=100, minLineLength=100, maxLineGap=5)
        end_time = time.time()
        time_mess += 'Hou:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'
        time_Hough = round((end_time - start_time) * 1000, 4)

        # 旋转校正、拉平、融合
        start_time = time.time()
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
                                elif x1_imu == x2_imu:  # 垂直线
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
                        num_L += 1
                        data_L = np.append(data_L, [[int(abs(values_ver[0])), x1_b, y1_b, x2_b, y2_b]], axis=0)
                        if abs(dis_temp_l) > abs(values_ver[0]):
                            dis_temp_l = values_ver[0]
                        if num_R > 0:
                            for data_R_value in data_R:
                                width_value = int(abs(values_ver[0])) + int(data_R_value[0])
                                if abs(width_value - width_threshold) <= 50:
                                    # cv2.putText(rgb_rot, str(int(abs(values_ver[0]))), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    #             (0, 0, 255), 1)
                                    # temp_x = int((data_R_value[1] + data_R_value[3]) / 2)
                                    # temp_y = int((data_R_value[2] + data_R_value[4]) / 2)
                                    # cv2.putText(rgb_rot, str(int(data_R_value[0])), (temp_x, temp_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    #             (0, 0, 255), 1)
                                    # print(width_value, int(abs(values_ver[0])), data_R_value[0])
                                    # width_mess += 'W' + str(width_value) + ',L' + str(int(abs(values_ver[0]))) + ',R' + str(data_R_value[0])
                                    find_width = True
                                    rec_width = width_value
                                    rec_left = str(abs(values_ver[0]))
                                    rec_right = str(data_R_value[0])
                    else:
                        num_R += 1
                        data_R = np.append(data_R, [[int(abs(values_ver[0])), x1_b, y1_b, x2_b, y2_b]], axis=0)
                        if dis_temp_r > values_ver[0]:
                            dis_temp_r = values_ver[0]
                        if num_L > 0:
                            for data_L_value in data_L:
                                width_value = int(abs(values_ver[0])) + int(data_L_value[0])
                                if abs(width_value - width_threshold) <= 50:
                                    # cv2.putText(rgb_rot, str(int(abs(values_ver[0]))), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    #             (0, 0, 255), 1)
                                    # temp_x = int((data_L_value[1] + data_L_value[3]) / 2)
                                    # temp_y = int((data_L_value[2] + data_L_value[4]) / 2)
                                    # cv2.putText(rgb_rot, str(int(data_L_value[0])), (temp_x, temp_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    #             (0, 0, 255), 1)
                                    # print(width_value, int(abs(values_ver[0])), data_L_value[0])
                                    # width_mess += 'W' + str(width_value) + ',L' + str(data_L_value[0]) + ',R' + str(int(abs(values_ver[0])))
                                    find_width = True
                                    rec_width = width_value
                                    rec_left = str(data_L_value[0])
                                    rec_right = str(values_ver[0])

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

        # 显示及保存图片
        if find_width:
            if num_hor > 0:
                sheet.write(x, 0, minCan)
                sheet.write(x, 1, maxCan)
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
                # cv2.imshow(str(minCan) + ',' + str(maxCan), rgb_rot)
                # print('Canny', str(minCan), str(maxCan))
                # print(num_line, num_hor, num_L, num_R)
                # print(width_mess)
                # print('\n')

        end_time_total = time.time()
        time_total = str(round((end_time_total - start_time_total) * 1000, 4)) + 'ms\n'
        time_mess += 'All:' + str(round((end_time_total - start_time_total) * 1000, 4)) + ';'

print(x)
xls.save('./TestData/C0-152403-779391.xls')
cv2.waitKey(0)

cv2.destroyAllWindows()
