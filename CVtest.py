import cv2
import CVFunc
import numpy as np
import matplotlib.pyplot as plt
import math
import time
import datetime


def getDist_P2P(x1_d, y1_d, x2_d, y2_d):
    distance = math.pow((x1_d - x2_d), 2) + math.pow((y1_d - y2_d), 2)
    distance = math.sqrt(distance)
    return distance


# 边缘检测算法（2022-06-30）
def find_edge(img_org):
    gra_can_mix = np.zeros((img_org.shape[0], img_org.shape[1]), np.uint8)  # 创建个全0的黑背景
    for i in range(20, 80, 20):
        for j in range(100, 250, 50):
            gra_can = cv2.Canny(img_org, i, j)
            gra_can_mix = cv2.add(gra_can_mix, gra_can)
    return gra_can_mix


def nearest_vertical(gra_edge_rot):
    # 保留下半部
    start_time = time.time()

    ver_err_mess = ''
    time_mess = ''
    ver_img_height = int(gra_edge_rot.shape[0])
    ver_img_width = int(gra_edge_rot.shape[1])
    ver_mid_height = int(ver_img_height / 2)
    ver_mid_width = int(ver_img_width / 2)
    gra_test = gra_edge_rot

    # 保留下半部分,查找垂直线
    # for j in range(0, img_width, 1):
    #     for i in range(0, img_height, 1):
    #         if i >= mid_height - 10:
    #             break
    #         else:
    #             gra_test[i, j] = 0
    # cv2.imshow('half', gra_test)

    # 初始化

    # 提取并去除横线和竖线
    # gra_temp = gra_test
    # horizontal_structure = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 1))
    # gra_temp = cv2.erode(gra_temp, horizontal_structure)
    # gra_temp = cv2.dilate(gra_temp, horizontal_structure)
    # gra_test = cv2.subtract(gra_test, gra_temp)
    # gra_temp = gra_test
    # vertical_structure = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 10))
    # gra_temp = cv2.erode(gra_temp, vertical_structure)
    # gra_temp = cv2.dilate(gra_temp, vertical_structure)
    # gra_test = cv2.subtract(gra_test, gra_temp)

    # 去除噪点

    # 根据连通区域去噪点
    # gra_test = CVFunc.func_noise_40(gra_test, 20)
    # gra_test = CVFunc.func_noise_30(gra_test, 100)
    ver_ret, gra_test = cv2.threshold(gra_test, 10, 255, cv2.THRESH_BINARY)
    end_time = time.time()
    time_mess += 'Init:' + str((end_time - start_time) * 1000) + 'ms\n'
    start_time = time.time()

    # 霍夫变换找直线
    angle_left_near_ver = 0.0
    axis_left_near_ver = [int] * 8
    b_left_near = -1.0 * ver_mid_width
    angle_right_near_ver = 0.0
    axis_right_near_ver = [int] * 8
    b_right_near = 2.0 * ver_mid_width

    gra_temp = np.zeros((img_height, img_width), np.uint8)

    try:
        lines = cv2.HoughLinesP(gra_test, rho=1.0, theta=np.pi / 180, threshold=100, minLineLength=100, maxLineGap=5)
        for line in lines:
            for x1, y1, x2, y2 in line:
                x1_f = float(x1)
                x2_f = float(x2)
                y1_f = float(y1)
                y2_f = float(y2)
                if ((x1 - x2 != 0) and (y1 - y2 != 0)) and ((y1 > ver_mid_height) or (y2 > ver_mid_height)):
                    a_line = (y1_f - y2_f) / (x1_f - x2_f)
                    b_line = y1_f - (a_line * x1_f)
                    result = np.arctan(-a_line) * 57.29577
                    if 90.0 > abs(result) > 10.0:
                        # cv2.line(rgb_test, (x1, y1), (x2, y2), (0, 0, 255), 1)
                        if result > 0 and x1 < ver_mid_width and x2 < ver_mid_width:
                            cv2.line(gra_temp, (x1, y1), (x2, y2), 255, 1)
                            if result > angle_left_near_ver:
                                angle_left_near_ver = result
                                axis_left_near_ver[0] = x1
                                axis_left_near_ver[1] = y1
                                axis_left_near_ver[2] = x2
                                axis_left_near_ver[3] = y2
                            if b_line > b_left_near:
                                axis_left_near_ver[4] = x1
                                axis_left_near_ver[5] = y1
                                axis_left_near_ver[6] = x2
                                axis_left_near_ver[7] = y2
                                b_left_near = b_line
                                # cv2.line(rgb_test, (x1, y1), (x2, y2), (0, 0, 255), 1)
                        elif result < 0 and x1 > ver_mid_width and x2 > ver_mid_width:
                            cv2.line(gra_temp, (x1, y1), (x2, y2), 255, 1)
                            if result < angle_right_near_ver:
                                angle_right_near_ver = result
                                axis_right_near_ver[0] = x1
                                axis_right_near_ver[1] = y1
                                axis_right_near_ver[2] = x2
                                axis_right_near_ver[3] = y2
                            if b_line < b_right_near:
                                axis_right_near_ver[4] = x1
                                axis_right_near_ver[5] = y1
                                axis_right_near_ver[6] = x2
                                axis_right_near_ver[7] = y2
                                b_right_near = b_line
                                # cv2.line(rgb_test, (x1, y1), (x2, y2), (0, 0, 255), 1)

    except Exception as e:
        ver_err_mess = 'nearest_vertical_30: Can`t find lines\n' + str(e)
        for i in range(0, 8, 1):
            axis_left_near_ver[i] = 0
            axis_right_near_ver[i] = 0

    # 输出
    end_time = time.time()
    time_mess += 'Lines:' + str((end_time - start_time) * 1000) + 'ms\n'

    # cv2.imshow('0', gra_temp)

    # cv2.imshow('vertest', rgb_ver)

    return axis_left_near_ver, axis_right_near_ver, angle_left_near_ver, angle_right_near_ver, ver_err_mess, time_mess


# 计算水平线实际距离
def calc_horizontal(int_height, f, w, a, b):
    distance_hor = f * w / (a * int_height + b)
    return distance_hor


# 计算垂直线距离
def calc_vertical(int_width, int_height, f, w, a, b):
    distance_ver = int_width * w / (a * int_height - b)
    return distance_ver


def get_red(img_rgb):
    # 提取区域，下半部分
    img_new = np.zeros((mid_height, img_width, 3), np.uint8)
    for j in range(0, img_width, 1):
        for i in range(0, mid_height, 1):
            img_new[i, j] = img_rgb[i + mid_height, j]
    img_red = img_new.copy()
    # 提取红色部分
    b_threshold = 100
    g_threshold = 100
    r_threshold = 130
    b, g, r = cv2.split(img_new)
    for j_ut in range(0, img_width, 1):
        for i_ut in range(0, mid_height, 1):
            b_value = b[i_ut, j_ut]
            g_value = g[i_ut, j_ut]
            r_value = r[i_ut, j_ut]
            if b_value < b_threshold and g_value < g_threshold and r_value > r_threshold:
                img_red[i_ut, j_ut] = img_new[i_ut, j_ut]
            else:
                img_red[i_ut, j_ut] = (0, 0, 0)
    # 二值化，去噪点
    gra_red = cv2.cvtColor(img_red, cv2.COLOR_BGR2GRAY)
    ret, gra_red = cv2.threshold(gra_red, 10, 255, cv2.THRESH_BINARY)
    gra_red = CVFunc.func_noise_1(gra_red, 400)
    return gra_red


def get_HoughLinesP(gra_edge, pic_id):
    # 提取各类型线段
    # print(pic_id)
    gra_width_HLP = gra_edge.shape[1]
    mid_width_HLP = int(gra_width_HLP / 2)
    gra_lines = np.zeros((gra_edge.shape[0], gra_edge.shape[1]), np.uint8)  # 创建个全0的黑背景
    y_area = np.zeros([gra_edge.shape[0], 2], np.uint64)  # 0是权重，1是x1和x2中最接近中轴的那一端
    ver_lines = []
    ver_lines_org = []
    num_hor = 0
    num_ver = 0
    lines = cv2.HoughLinesP(gra_edge, rho=1.0, theta=np.pi / 180, threshold=20, minLineLength=20, maxLineGap=20)
    for line in lines:
        for x1, y1, x2, y2 in line:
            if getDist_P2P(x1, y1, x2, y2) > 50.0:
                # cv2.line(gra_lines, (x1, y1), (x2, y2), 255, 1)
                if abs(y1 - y2) < 2 and x2 > int(gra_width_HLP / 4) and x1 < int(gra_width_HLP * 3 / 4):
                    cv2.line(gra_lines, (x1, y1), (x2, y2), 255, 1)
                    y_avg = int((y1 + y2) / 2)
                    y_area[y_avg, 0] += abs(x1 - x2)
                    if abs(x1 - (gra_width_HLP / 2)) > abs(x2 - (gra_width_HLP / 2)):
                        y_area[y_avg, 1] = x2
                    else:
                        y_area[y_avg, 1] = x1
                    num_hor += 1
                    # print(y1, y2)
                    continue
                elif abs(y1 - y2) > 5:
                    # cv2.line(gra_lines, (x1, y1), (x2, y2), 255, 1)
                    # num_lines += 1
                    w1_f = float(x1)
                    w2_f = float(x2)
                    h1_f = float(y1)
                    h2_f = float(y2)
                    a_f = (w1_f - w2_f) / (h1_f - h2_f)
                    b_f = w1_f - (a_f * h1_f)
                    weight_f = getDist_P2P(x1, y1, x2, y2)
                    # if 900.0 > b_f > 600.0:
                    # cv2.line(gra_lines, (x1, y1), (x2, y2), 255, 1)
                    # print(a_f, b_f)
                    ver_lines.append([a_f, b_f, weight_f])
                    ver_lines_org.append(line)
                    continue
    # 垂直线数组
    ver_lines.sort(key=lambda x: x[0], reverse=True)
    ver_formulas_new = []
    a_sum = 0.0
    b_sum = 0.0
    weight_sum = 0.0
    for ver_line in ver_lines:
        if a_sum == 0.0:
            a_sum = ver_line[0] * ver_line[2]
            b_sum = ver_line[1] * ver_line[2]
            weight_sum = ver_line[2]
        else:
            if abs((a_sum / weight_sum) - ver_line[0]) < 0.1:
                a_sum += ver_line[0] * ver_line[2]
                b_sum += ver_line[1] * ver_line[2]
                weight_sum += ver_line[2]
            else:
                temp_a_avg = round((a_sum / weight_sum), 4)
                temp_b_avg = round((b_sum / weight_sum), 4)
                ver_formulas_new.append([temp_a_avg, temp_b_avg])
                a_sum = ver_line[0] * ver_line[2]
                b_sum = ver_line[1] * ver_line[2]
                weight_sum = ver_line[2]
    if a_sum != 0.0:
        temp_a_avg = round((a_sum / weight_sum), 4)
        temp_b_avg = round((b_sum / weight_sum), 4)
        ver_formulas_new.append([temp_a_avg, temp_b_avg])

    # gra_lines_new = np.zeros((gra_edge.shape[0], gra_edge.shape[1]), np.uint8)  # 创建个全0的黑背景
    for ver_formula in ver_formulas_new:
        h1 = 0
        w1 = int(ver_formula[1])
        h2 = 540
        w2 = int(ver_formula[0] * h2 + ver_formula[1])
        # cv2.line(gra_lines_new, (w1, h1), (w2, h2), 255, 1)
    # print(ver_formulas_new[1], ver_formulas_new[2])

    # 水平线数组
    hor_height = []
    temp_height = 0
    temp_width = 0
    temp_point = [0, 0]
    for i in range(0, gra_edge.shape[0], 1):
        if y_area[i, 0] > 0:
            # print(i, y_area[i, 0], y_area[i, 1])
            if temp_height == 0:
                temp_height = i
                temp_width = y_area[i, 1]
                temp_point = [temp_height, temp_width]
            else:
                if i - temp_height < 5:
                    temp_height = i
                    if abs(temp_width - mid_width_HLP) >= abs(y_area[i, 1] - mid_width_HLP):
                        temp_width = y_area[i, 1]
                        temp_point = [i, temp_width]
                else:
                    hor_height.append(temp_point)
                    # print(temp_point)
                    temp_height = i
                    temp_width = y_area[i, 1]
                    temp_point = [temp_height, temp_width]
    last_id = int(len(hor_height) - 1)
    if (temp_height - hor_height[last_id][0]) > 4:
        hor_height.append(temp_point)

    return hor_height, ver_lines_org, ver_formulas_new[1], ver_formulas_new[2]


# 开始主程序
ret_mess = ''
err_mess_all = ''

# 提取图像
rgb_frame_50 = cv2.imread('./TestData/50.jpg')
rgb_frame_40 = cv2.imread('./TestData/40.jpg')
rgb_frame_30 = cv2.imread('./TestData/30.jpg')
rgb_frame_20 = cv2.imread('./TestData/20.jpg')
# 获取图像位置参数
img_test = rgb_frame_50.copy()
img_height = int(img_test.shape[0])
img_width = int(img_test.shape[1])
mid_height = int(img_height / 2)
mid_width = int(img_width / 2)

# gra_red_50 = get_red(rgb_frame_50)
# gra_red_40 = get_red(rgb_frame_40)
# gra_red_30 = get_red(rgb_frame_30)
# gra_red_20 = get_red(rgb_frame_20)
gra_red = get_red(img_test)

# Canny提取边界
# gra_edge_50 = find_edge(gra_red_50)
# gra_edge_40 = find_edge(gra_red_40)
# gra_edge_30 = find_edge(gra_red_30)
# gra_edge_50 = cv2.Canny(gra_red_50, 70, 140)
# gra_edge_40 = cv2.Canny(gra_red_40, 70, 140)
# gra_edge_30 = cv2.Canny(gra_red_30, 70, 140)
# gra_edge_20 = cv2.Canny(gra_red_20, 70, 140)
gra_edge = cv2.Canny(gra_red, 70, 140)

# gra_edge_50[0:100, :] = 0
# gra_edge_40[0:120, :] = 0
# gra_edge_30[0:140, :] = 0
# gra_edge_20[0:160, :] = 0
gra_edge[0:100, :] = 0

# hor_lines_points_50, ver_lines_50, right_formular_50, left_formular_50 = get_HoughLinesP(gra_edge_50, '50')
# hor_lines_points_40, ver_lines_40, right_formular_40, left_formular_40 = get_HoughLinesP(gra_edge_40, '40')
# hor_lines_points_30, ver_lines_30, right_formular_30, left_formular_30 = get_HoughLinesP(gra_edge_30, '30')
# hor_lines_points_20, ver_lines_20, right_formular_20, left_formular_20 = get_HoughLinesP(gra_edge_20, '20')
hor_lines_points, ver_lines, right_formular, left_formular = get_HoughLinesP(gra_edge, '20')

model_F = 1122
model_W = 101
model_a = right_formular[0] - left_formular[0]
model_b = right_formular[1] - left_formular[1]

# model_a_50 = right_formular_50[0] - left_formular_50[0]
# model_b_50 = right_formular_50[1] - left_formular_50[1]
#
# model_a_40 = right_formular_40[0] - left_formular_40[0]
# model_b_40 = right_formular_40[1] - left_formular_40[1]
#
# model_a_30 = right_formular_30[0] - left_formular_30[0]
# model_b_30 = right_formular_30[1] - left_formular_30[1]
#
# model_a_20 = right_formular_20[0] - left_formular_20[0]
# model_b_20 = right_formular_20[1] - left_formular_20[1]

# print('50', model_a_50, model_b_50)
# print('40', model_a_40, model_b_40)
# print('30', model_a_30, model_b_30)
# print('20', model_a_20, model_b_20)

cv2.line(img_test, (mid_width, 0), (mid_width, img_height), (0, 0, 255), 1)
cv2.line(img_test, (0, mid_height), (img_width, mid_height), (0, 0, 255), 1)

for hor_point in hor_lines_points:
    dis_temp = calc_horizontal(hor_point[0], model_F, model_W, model_a, model_b)
    dis_temp = round(dis_temp, 2)
    cv2.line(img_test, (0, hor_point[0] + mid_height), (img_width, hor_point[0] + mid_height), (255, 0, 0), 1)
    cv2.putText(img_test, str(dis_temp) + 'mm', (mid_width, hor_point[0] + mid_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)
    print(hor_point[0], dis_temp)

for ver_line in ver_lines:
    for x1, y1, x2, y2 in ver_line:
        dis_ver_1 = calc_vertical(abs(x1 - mid_width), y1, model_F, model_W, model_a, model_b)
        dis_ver_2 = calc_vertical(abs(x2 - mid_width), y2, model_F, model_W, model_a, model_b)
        dis_ver = (dis_ver_1 + dis_ver_2) / 2
        # print(x1, y1, dis_ver_1)
        # print(x2, y2, dis_ver_2)
        x_ver = int((x1 + x2) / 2)
        y_ver = int((y1 + y2) / 2)
        cv2.line(img_test, (x1, y1 + mid_height), (x2, y2 + mid_height), (0, 255, 0), 1)
        cv2.putText(img_test, str(round(dis_ver, 0)), (x_ver, y_ver + mid_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        # cv2.putText(img_test, str(round(dis_ver_2, 0)), (x2, y2 + mid_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
        #             (0, 255, 0), 1)

# cv2.imwrite('gra-50.jpg', gra_edge_50)
# cv2.imwrite('gra-40.jpg', gra_edge_40)
# cv2.imwrite('gra-30.jpg', gra_edge_30)
# cv2.imwrite('gra-20.jpg', gra_edge_20)
# cv2.imshow('50', gra_edge_50)
# cv2.imshow('40', gra_edge_40)
# cv2.imshow('30', gra_edge_30)
# cv2.imshow('20', gra_edge_20)
cv2.imshow('test', img_test)
cv2.imwrite('dis-50.jpg', img_test)


cv2.waitKey(0)
cv2.destroyAllWindows()
