import cv2
import numpy as np
import time
import math


# 计算水平线实际距离（2022-08-01）
def calc_horizontal(int_height, f, w, a, b):
    distance_hor = round((f * w / (a * int_height + b)), 0)
    return distance_hor


# 计算垂直线距离（2022-08-01
def calc_vertical(int_width, int_height, f, w, a, b):
    distance_ver = round((int_width * w / (a * int_height + b)), 0)
    return distance_ver



# 边缘检测算法（2022-06-30）
def find_edge(img_org):
    gra_can_mix = np.zeros((img_org.shape[0], img_org.shape[1]), np.uint8)  # 创建个全0的黑背景
    for i in range(20, 120, 20):
        for j in range(100, 450, 50):
            gra_can = cv2.Canny(img_org, i, j)
            # int_pixNum = cv2.countNonZero(gra_can)
            # print(i, j, int_pixNum)
            # cv2.addWeighted(gra_can_mix, 1.0, gra_can, 0.05, 0.0, gra_can_mix)
            gra_can_mix = cv2.add(gra_can_mix, gra_can)

    # ret, gra_can_mix = cv2.threshold(gra_can_mix, 0, 255, cv2.THRESH_BINARY)
    return gra_can_mix


# 计算延长线
def func_extension_line(x1, y1, x2, y2, side, img_height, img_width):
    a = (y1 - y2) / (x1 - x2)
    b = y1 - (a * x1)
    line_axis = [int] * 4
    # 全图线
    if side == 'L' or side == 'l':
        if b < img_height:
            line_axis[0] = 0
            line_axis[1] = int(b)
        else:
            line_axis[0] = int((img_height - 1 - b) / a)
            line_axis[1] = img_height - 1
        if (- b / a) < img_width:
            line_axis[2] = int(- b / a)
            line_axis[3] = 0
        else:
            line_axis[2] = img_width - 1
            line_axis[3] = int(a * (img_width - 1) + b)
    elif side == 'R' or side == 'r':
        if a * img_width + b < img_height:
            line_axis[0] = img_width - 1
            line_axis[1] = int(a * (img_width - 1) + b)
        else:
            line_axis[0] = int(((img_height - 1) - b) / a)
            line_axis[1] = img_height - 1
        if b > 0:
            line_axis[2] = 0
            line_axis[3] = int(b)
        else:
            line_axis[2] = int(- b / a)
            line_axis[3] = 0

    return line_axis


# 去噪点方法1
def func_noise_1(img_org, area_thread):
    contours, hierarch = cv2.findContours(img_org, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    area = []
    for i in range(len(contours)):
        area.append(cv2.contourArea(contours[i]))
        if area[i] < area_thread:
            cv2.drawContours(img_org, [contours[i]], 0, 0, -1)
            continue
    return img_org


# 去噪点方法2
def func_noise_2(img_org, area_thread):
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(img_org, connectivity=8)
    gra_temp = np.zeros((img_org.shape[0], img_org.shape[1]), np.uint8)  # 创建个全0的黑背景
    for i in range(1, num_labels):
        mask = labels == i  # 这一步是通过labels确定区域位置，让labels信息赋给mask数组，再用mask数组做img数组的索引
        if stats[i][4] > area_thread:  # 300是面积 可以随便调
            gra_temp[mask] = 255  # 面积大于300的区域涂白留下，小于300的涂0抹去
        else:
            gra_temp[mask] = 0
    return gra_temp


# 图像根据中间50%区域的水平边界，获取翻滚角的角度（2022-07-04）
def angle_rotate_roll(gra_edge):
    img_height = int(gra_edge.shape[0])
    img_width = int(gra_edge.shape[1])

    near_bottom = img_height
    near_top = img_height
    temp_count = 0

    angle_sum = 0
    angle_count = 0
    angle_avg = 0.0

    err_mess = ''

    gra_temp = np.zeros((img_height, img_width), np.uint8)
    y_area = np.zeros([img_height], np.uint64)
    try:
        lines = lines_horizontal(gra_edge, 20)
        for line in lines:
            for x1, y1, x2, y2 in line:
                x1_f = float(x1)
                x2_f = float(x2)
                y1_f = float(y1)
                y2_f = float(y2)
                if ((int(0.75 * img_width) >= x1 >= int(0.25 * img_width)) or (
                        int(0.75 * img_width) >= x2 >= int(0.25 * img_width))):
                    if y2_f - y1_f == 0:
                        y_area[y1] = y_area[y1] + 2
                    elif x2_f - x1_f != 0:
                        k = -(y2_f - y1_f) / (x2_f - x1_f)
                        result = np.arctan(k) * 57.29577
                        if abs(result) <= 10.0:
                            y_area[y1] += 1
                            y_area[y2] += 1

    except Exception as e:
        err_mess = 'angle_rotate_roll: Can`t find lines\n' + str(e)
        angle_avg = 0.0
    else:
        for i in range(img_height - 1, 0, -1):
            if y_area[i] > 0:
                if near_bottom == img_height:
                    near_bottom = i
                else:
                    temp_count = 0
            else:
                if near_bottom != img_height:
                    if temp_count > 10:
                        near_top = i
                        break
                    else:
                        temp_count += 1

        for line in lines:
            for x1, y1, x2, y2 in line:
                x1_f = float(x1)
                x2_f = float(x2)
                y1_f = float(y1)
                y2_f = float(y2)
                if ((int(0.75 * img_width) >= x1 >= int(0.25 * img_width)) or (
                        int(0.75 * img_width) >= x2 >= int(0.25 * img_width))):
                    if (near_bottom >= y1 >= near_top) or (near_bottom >= y2 >= near_top):
                        cv2.line(gra_temp, (x1, y1), (x2, y2), 255, 1)
                        k = -(y2_f - y1_f) / (x2_f - x1_f)
                        result = np.arctan(k) * 57.29577
                        angle_sum += result
                        angle_count += 1

        if angle_count > 0:
            angle_avg = angle_sum / angle_count

        if abs(angle_avg) > 45.0:
            err_mess = 'angle_rotate_roll: Angle is too large, ' + str(angle_avg)

    return angle_avg, err_mess


# 搜索最近水平线，返回Y轴数值（2022-07-04）
def nearest_horizontal_1(gra_edge_rot):
    err_mess = ''
    img_width = int(gra_edge_rot.shape[1])
    height_nearest = 0.0
    try:
        lines = lines_horizontal(gra_edge_rot, 30)
        for line in lines:
            for x1, y1, x2, y2 in line:
                if ((int(0.8 * img_width) >= x1 >= int(0.2 * img_width)) and (
                        int(0.8 * img_width) >= x2 >= int(0.2 * img_width))):
                    if abs(y1 - y2) < 10:
                        y_avg = (y1 + y2) / 2
                        if y_avg > height_nearest:
                            height_nearest = y_avg
    except Exception as e:
        err_mess = 'nearest_horizontal_1: Can`t find lines\n' + str(e)
        height_nearest = -1.0
    return height_nearest, err_mess


# 搜索水平线（2022-06-30）
def lines_horizontal(gra_edge, horizontal_thread):
    img_height = int(gra_edge.shape[0])
    img_width = int(gra_edge.shape[1])
    mid_height = int(img_height / 2)
    mid_width = int(img_width / 2)

    gra_test = gra_edge

    kernel_horizontal = cv2.getStructuringElement(cv2.MORPH_RECT, (horizontal_thread, 1))
    gra_test = cv2.erode(gra_test, kernel_horizontal)
    gra_test = cv2.dilate(gra_test, kernel_horizontal)

    min_line_length = 70
    max_line_gap = 20
    angle_theta = 180
    int_threshold = 100
    lines = cv2.HoughLinesP(gra_test, rho=1.0, theta=np.pi / angle_theta, threshold=int_threshold,
                            minLineLength=min_line_length, maxLineGap=max_line_gap)
    return lines


# 搜索下半张图最近的左右两边垂直线，返回左右的两端坐标及角度（2022-07-07）
def nearest_vertical_1(gra_edge_rot):
    err_mess = ''
    img_height = int(gra_edge_rot.shape[0])
    img_width = int(gra_edge_rot.shape[1])
    mid_height = int(img_height / 2)
    mid_width = int(img_width / 2)

    # 保留下半部分,查找垂直线
    gra_test = gra_edge_rot
    for j in range(0, img_width, 1):
        for i in range(0, img_height, 1):
            if i >= mid_height - 10:
                break
            else:
                gra_test[i, j] = 0

    # 提取并去除横线和竖线
    gra_temp = gra_test
    horizontal_structure = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 1))
    gra_temp = cv2.erode(gra_temp, horizontal_structure)
    gra_temp = cv2.dilate(gra_temp, horizontal_structure)
    gra_test = cv2.subtract(gra_test, gra_temp)
    gra_temp = gra_test
    vertical_structure = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 5))
    gra_temp = cv2.erode(gra_temp, vertical_structure)
    gra_temp = cv2.dilate(gra_temp, vertical_structure)
    gra_test = cv2.subtract(gra_test, gra_temp)
    # 根据连通区域去噪点
    # gra_test = func_noise_1(gra_test, 20)
    # gra_test = func_noise_2(gra_test, 100)
    ret, gra_test = cv2.threshold(gra_test, 10, 255, cv2.THRESH_BINARY)
    # 霍夫变换找直线
    angle_left_near = 0.0
    axis_left_near = [int] * 4
    angle_right_near = 0.0
    axis_right_near = [int] * 4
    min_line_length = 20
    max_line_gap = 50
    angle_theta = 180
    int_threshold = 200
    lines = cv2.HoughLinesP(gra_test, rho=1.0, theta=np.pi / angle_theta, threshold=int_threshold,
                            minLineLength=min_line_length, maxLineGap=max_line_gap)
    try:
        for line in lines:
            for x1, y1, x2, y2 in line:
                x1_f = float(x1)
                x2_f = float(x2)
                y1_f = float(y1)
                y2_f = float(y2)
                if (x1 - x2 != 0) and (y1 - y2 != 0):
                    k = -(y2_f - y1_f) / (x2_f - x1_f)
                    result = np.arctan(k) * 57.29577
                    if abs(result) > 10.0:
                        if result > 0 and x1 < mid_width and x2 < mid_width:
                            if result > angle_left_near:
                                angle_left_near = result
                                axis_left_near[0] = x1
                                axis_left_near[1] = y1
                                axis_left_near[2] = x2
                                axis_left_near[3] = y2
                        elif result < 0 and x1 > mid_width and x2 > mid_width:
                            if result < angle_right_near:
                                angle_right_near = result
                                axis_right_near[0] = x1
                                axis_right_near[1] = y1
                                axis_right_near[2] = x2
                                axis_right_near[3] = y2
    except Exception as e:
        err_mess = 'nearest_vertical_1: Can`t find lines\n' + str(e)
        for i in range(0, 4, 1):
            axis_left_near[i] = 0
            axis_right_near[i] = 0

    return axis_left_near, axis_right_near, angle_left_near, angle_right_near, err_mess


def nearest_vertical_2(gra_edge_rot):
    # 保留下半部
    start_time = time.time()

    err_mess = ''
    time_mess = ''
    img_height = int(gra_edge_rot.shape[0])
    img_width = int(gra_edge_rot.shape[1])
    mid_height = int(img_height / 2)
    mid_width = int(img_width / 2)
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
    # gra_test = CVFunc.func_noise_1(gra_test, 20)
    # gra_test = CVFunc.func_noise_2(gra_test, 100)
    ret, gra_test = cv2.threshold(gra_test, 10, 255, cv2.THRESH_BINARY)
    end_time = time.time()
    time_mess += 'Init:' + str((end_time - start_time) * 1000) + 'ms\n'
    start_time = time.time()

    # 霍夫变换找直线
    angle_left_near_ver = 0.0
    axis_left_near_ver = [int] * 8
    b_left_near = -1.0 * mid_width
    angle_right_near_ver = 0.0
    axis_right_near_ver = [int] * 8
    b_right_near = 2.0 * mid_width

    # gra_temp = np.zeros((img_height, img_width), np.uint8)

    try:
        lines = cv2.HoughLinesP(gra_test, rho=1.0, theta=np.pi / 180, threshold=100, minLineLength=100, maxLineGap=5)
        for line in lines:
            for x1, y1, x2, y2 in line:
                x1_f = float(x1)
                x2_f = float(x2)
                y1_f = float(y1)
                y2_f = float(y2)
                if ((x1 - x2 != 0) and (y1 - y2 != 0)) and ((y1 > mid_height) or (y2 > mid_height)):
                    a_line = (y1_f - y2_f) / (x1_f - x2_f)
                    b_line = y1_f - (a_line * x1_f)
                    result = np.arctan(-a_line) * 57.29577
                    if 90.0 > abs(result) > 10.0:
                        # cv2.line(rgb_test, (x1, y1), (x2, y2), (0, 0, 255), 1)
                        if result > 0 and x1 < mid_width and x2 < mid_width:
                            # cv2.line(gra_temp, (x1, y1), (x2, y2), 255, 1)
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
                        elif result < 0 and x1 > mid_width and x2 > mid_width:
                            # cv2.line(gra_temp, (x1, y1), (x2, y2), 255, 1)
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
        err_mess = 'nearest_vertical_2: Can`t find lines\n' + str(e)
        for i in range(0, 8, 1):
            axis_left_near_ver[i] = 0
            axis_right_near_ver[i] = 0

    # 输出
    end_time = time.time()
    time_mess += 'Lines:' + str((end_time - start_time) * 1000) + 'ms\n'

    # cv2.imshow('0', gra_temp)

    # cv2.imshow('vertest', rgb_ver)

    return axis_left_near_ver, axis_right_near_ver, angle_left_near_ver, angle_right_near_ver, err_mess, time_mess


# （Backup）图像根据中间50%区域的水平边界，获取旋转角度（2022-06-30）
def angle_rotation(rgb_org):
    img_height = int(rgb_org.shape[0])
    img_width = int(rgb_org.shape[1])
    mid_height = int(img_height / 2)
    mid_width = int(img_width / 2)
    rgb_test = rgb_org
    gra_edge = find_edge(rgb_test)

    gra_temp = np.zeros((img_height, img_width), np.uint8)
    y_area = np.zeros([img_height], np.uint64)

    lines = lines_horizontal(gra_edge, 20)
    for line in lines:
        for x1, y1, x2, y2 in line:
            x1_f = float(x1)
            x2_f = float(x2)
            y1_f = float(y1)
            y2_f = float(y2)
            if ((int(0.75 * img_width) >= x1 >= int(0.25 * img_width)) or (
                    int(0.75 * img_width) >= x2 >= int(0.25 * img_width))):
                if x2_f - x1_f == 0:
                    result = 90.0
                elif y2_f - y1_f == 0:
                    result = 0.0
                    # cv2.line(gra_temp, (x1, y1), (x2, y2), 255, 1)
                    y_area[y1] = y_area[y1] + 2
                else:
                    k = -(y2_f - y1_f) / (x2_f - x1_f)
                    result = np.arctan(k) * 57.29577
                    if abs(result) <= 10.0:
                        # cv2.line(gra_temp, (x1, y1), (x2, y2), 255, 1)
                        y_area[y1] += 1
                        y_area[y2] += 1

    near_bottom = img_height
    near_top = img_height
    temp_count = 0
    for i in range(img_height - 1, 0, -1):
        if y_area[i] > 0:
            if near_bottom == img_height:
                near_bottom = i
            else:
                temp_count = 0
        else:
            if near_bottom != img_height:
                if temp_count > 10:
                    near_top = i
                    break
                else:
                    temp_count += 1

    angle_sum = 0
    angle_count = 0
    for line in lines:
        for x1, y1, x2, y2 in line:
            x1_f = float(x1)
            x2_f = float(x2)
            y1_f = float(y1)
            y2_f = float(y2)
            if ((int(0.75 * img_width) >= x1 >= int(0.25 * img_width)) or (
                    int(0.75 * img_width) >= x2 >= int(0.25 * img_width))):
                if (near_bottom >= y1 >= near_top) or (near_bottom >= y2 >= near_top):
                    cv2.line(gra_temp, (x1, y1), (x2, y2), 255, 1)
                    k = -(y2_f - y1_f) / (x2_f - x1_f)
                    result = np.arctan(k) * 57.29577
                    angle_sum += result
                    angle_count += 1
    angle_avg = 0.0
    if angle_count > 0:
        angle_avg = angle_sum / angle_count
    return angle_avg


# （Backup）搜索最近水平线，返回Y轴数值（2022-07-01）
def nearest_horizontal(gra_edge_rot):
    img_height = int(gra_edge_rot.shape[0])
    img_width = int(gra_edge_rot.shape[1])
    height_nearest = 0.0
    lines = lines_horizontal(gra_edge_rot, 30)
    for line in lines:
        for x1, y1, x2, y2 in line:
            if ((int(0.8 * img_width) >= x1 >= int(0.2 * img_width)) and (
                    int(0.8 * img_width) >= x2 >= int(0.2 * img_width))):
                if abs(y1 - y2) < 10:
                    # cv2.line(rgb_test, (x1, y1), (x2, y2), (0, 0, 255), 1)
                    y_avg = (y1 + y2) / 2
                    if y_avg > height_nearest:
                        height_nearest = y_avg
    return height_nearest


# （Backup）搜索左右的最近垂直线和最远边界
def func_vertical(gra_edge):
    img_height = int(gra_edge.shape[0])
    img_width = int(gra_edge.shape[1])
    mid_height = int(img_height / 2)
    mid_width = int(img_width / 2)

    # 保留下半部分,查找垂直线
    gra_test = gra_edge
    for j in range(0, img_width, 1):
        for i in range(0, img_height, 1):
            if i >= mid_height - 10:
                break
            else:
                gra_test[i, j] = 0
    # 提取并去除横线和竖线
    gra_temp = gra_test
    horizontal_structure = cv2.getStructuringElement(cv2.MORPH_RECT, (30, 1))
    gra_temp = cv2.erode(gra_temp, horizontal_structure)
    gra_temp = cv2.dilate(gra_temp, horizontal_structure)
    gra_test = cv2.subtract(gra_test, gra_temp)
    gra_temp = gra_test
    vertical_structure = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 30))
    gra_temp = cv2.erode(gra_temp, vertical_structure)
    gra_temp = cv2.dilate(gra_temp, vertical_structure)
    gra_test = cv2.subtract(gra_test, gra_temp)
    # 根据连通区域去噪点
    gra_test = func_noise_2(gra_test, 10)
    # 霍夫变换找直线
    angle_left_near = 0.0
    axis_left_near = [int] * 4
    angle_left_edge = 90.0
    axis_left_edge = [int] * 4
    angle_right_near = 0.0
    axis_right_near = [int] * 4
    angle_right_edge = -90.0
    axis_right_edge = [int] * 4
    min_line_length = 50
    max_line_gap = 5
    angle_theta = 180
    int_threshold = 10
    lines = cv2.HoughLinesP(gra_test, rho=1.0, theta=np.pi / angle_theta, threshold=int_threshold,
                            minLineLength=min_line_length, maxLineGap=max_line_gap)
    for line in lines:
        for x1, y1, x2, y2 in line:
            x1_f = float(x1)
            x2_f = float(x2)
            y1_f = float(y1)
            y2_f = float(y2)
            if x2_f - x1_f == 0:
                result = 90.0
            elif y2_f - y1_f == 0:
                result = 0.0
            else:
                k = -(y2_f - y1_f) / (x2_f - x1_f)
                result = np.arctan(k) * 57.29577
                if abs(result) < 85.0:
                    if result > 0 and x1 < mid_width and x2 < mid_width:
                        if result > angle_left_near:
                            angle_left_near = result
                            axis_left_near[0] = x1
                            axis_left_near[1] = y1
                            axis_left_near[2] = x2
                            axis_left_near[3] = y2
                        if result < angle_left_edge:
                            angle_left_edge = result
                            axis_left_edge[0] = x1
                            axis_left_edge[1] = y1
                            axis_left_edge[2] = x2
                            axis_left_edge[3] = y2
                    elif result < 0 and x1 > mid_width and x2 > mid_width:
                        if result < angle_right_near:
                            angle_right_near = result
                            axis_right_near[0] = x1
                            axis_right_near[1] = y1
                            axis_right_near[2] = x2
                            axis_right_near[3] = y2
                        if result > angle_right_edge:
                            angle_right_edge = result
                            axis_right_edge[0] = x1
                            axis_right_edge[1] = y1
                            axis_right_edge[2] = x2
                            axis_right_edge[3] = y2
    line_left_near = func_extension_line(axis_left_near[0], axis_left_near[1], axis_left_near[2],
                                         axis_left_near[3], 'l', img_height, img_width)
    line_left_edge = func_extension_line(axis_left_edge[0], axis_left_edge[1], axis_left_edge[2],
                                         axis_left_edge[3], 'l', img_height, img_width)
    line_right_near = func_extension_line(axis_right_near[0], axis_right_near[1], axis_right_near[2],
                                          axis_right_near[3], 'r', img_height, img_width)
    line_right_edge = func_extension_line(axis_right_edge[0], axis_right_edge[1], axis_right_edge[2],
                                          axis_right_edge[3], 'r', img_height, img_width)
    vertical_line = [axis_left_near, axis_left_edge, axis_right_near, axis_right_edge, line_left_near,
                     line_left_edge, line_right_near, line_right_edge]

    vertical_angle = [angle_left_near, angle_left_edge, angle_right_near, angle_right_edge]

    return vertical_angle, vertical_line


# （Backup）搜索最近的左右两边垂直线，返回左右的两端坐标及角度（2022-07-01）
def nearest_vertical(gra_edge_rot):
    img_height = int(gra_edge_rot.shape[0])
    img_width = int(gra_edge_rot.shape[1])
    mid_height = int(img_height / 2)
    mid_width = int(img_width / 2)
    gra_test = gra_edge_rot
    # 保留下半部分,查找垂直线
    # for j in range(0, img_width, 1):
    #     for i in range(0, img_height, 1):
    #         if i >= mid_height - 10:
    #             break
    #         else:
    #             gra_test[i, j] = 0
    # 提取并去除横线和竖线
    gra_temp = gra_test
    horizontal_structure = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 1))
    gra_temp = cv2.erode(gra_temp, horizontal_structure)
    gra_temp = cv2.dilate(gra_temp, horizontal_structure)
    gra_test = cv2.subtract(gra_test, gra_temp)
    gra_temp = gra_test
    vertical_structure = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 5))
    gra_temp = cv2.erode(gra_temp, vertical_structure)
    gra_temp = cv2.dilate(gra_temp, vertical_structure)
    gra_test = cv2.subtract(gra_test, gra_temp)
    # 根据连通区域去噪点
    gra_test = func_noise_1(gra_test, 20)
    gra_test = func_noise_2(gra_test, 100)
    ret, gra_test = cv2.threshold(gra_test, 10, 255, cv2.THRESH_BINARY)
    # 霍夫变换找直线
    angle_left_near = 0.0
    axis_left_near = [int] * 4
    angle_right_near = 0.0
    axis_right_near = [int] * 4
    min_line_length = 20
    max_line_gap = 50
    angle_theta = 180
    int_threshold = 200
    lines = cv2.HoughLinesP(gra_test, rho=1.0, theta=np.pi / angle_theta, threshold=int_threshold,
                            minLineLength=min_line_length, maxLineGap=max_line_gap)
    for line in lines:
        for x1, y1, x2, y2 in line:
            x1_f = float(x1)
            x2_f = float(x2)
            y1_f = float(y1)
            y2_f = float(y2)
            if (x1 - x2 != 0) and (y1 - y2 != 0):
                k = -(y2_f - y1_f) / (x2_f - x1_f)
                result = np.arctan(k) * 57.29577
                if abs(result) > 10.0:
                    if result > 0 and x1 < mid_width and x2 < mid_width:
                        # cv2.line(rgb_test, (x1, y1), (x2, y2), (0, 255, 0), 1)
                        if result > angle_left_near:
                            angle_left_near = result
                            axis_left_near[0] = x1
                            axis_left_near[1] = y1
                            axis_left_near[2] = x2
                            axis_left_near[3] = y2
                    elif result < 0 and x1 > mid_width and x2 > mid_width:
                        # cv2.line(rgb_test, (x1, y1), (x2, y2), (255, 0, 0), 1)
                        if result < angle_right_near:
                            angle_right_near = result
                            axis_right_near[0] = x1
                            axis_right_near[1] = y1
                            axis_right_near[2] = x2
                            axis_right_near[3] = y2

    return axis_left_near, axis_right_near, angle_left_near, angle_right_near


# （Backup）搜索左右最远边界的直线方程
def func_vertical_equation(gra_edge):
    img_height = int(gra_edge.shape[0])
    img_width = int(gra_edge.shape[1])
    mid_height = int(img_height / 2)
    mid_width = int(img_width / 2)

    # 保留下半部分,查找垂直线
    gra_test = gra_edge
    for j in range(0, img_width, 1):
        for i in range(0, img_height, 1):
            if i >= mid_height - 10:
                break
            else:
                gra_test[i, j] = 0
    # 提取并去除横线和竖线
    gra_temp = gra_test
    horizontal_structure = cv2.getStructuringElement(cv2.MORPH_RECT, (30, 1))
    gra_temp = cv2.erode(gra_temp, horizontal_structure)
    gra_temp = cv2.dilate(gra_temp, horizontal_structure)
    gra_test = cv2.subtract(gra_test, gra_temp)
    gra_temp = gra_test
    vertical_structure = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 30))
    gra_temp = cv2.erode(gra_temp, vertical_structure)
    gra_temp = cv2.dilate(gra_temp, vertical_structure)
    gra_test = cv2.subtract(gra_test, gra_temp)
    # 根据连通区域去噪点
    gra_test = func_noise_2(gra_test, 10)
    # 霍夫变换找直线
    angle_left_edge = 90.0
    axis_left_edge = [int] * 4
    angle_right_edge = -90.0
    axis_right_edge = [int] * 4
    min_line_length = 50
    max_line_gap = 5
    angle_theta = 180
    int_threshold = 10
    lines = cv2.HoughLinesP(gra_test, rho=1.0, theta=np.pi / angle_theta, threshold=int_threshold,
                            minLineLength=min_line_length, maxLineGap=max_line_gap)
    for line in lines:
        for x1, y1, x2, y2 in line:
            x1_f = float(x1)
            x2_f = float(x2)
            y1_f = float(y1)
            y2_f = float(y2)
            if x2_f - x1_f == 0:
                result = 90.0
            elif y2_f - y1_f == 0:
                result = 0.0
            else:
                k = -(y2_f - y1_f) / (x2_f - x1_f)
                result = np.arctan(k) * 57.29577
                if abs(result) < 85.0:
                    if result > 0 and x1 < mid_width and x2 < mid_width:
                        if result < angle_left_edge:
                            angle_left_edge = result
                            axis_left_edge[0] = x1
                            axis_left_edge[1] = y1
                            axis_left_edge[2] = x2
                            axis_left_edge[3] = y2
                    elif result < 0 and x1 > mid_width and x2 > mid_width:
                        if result > angle_right_edge:
                            angle_right_edge = result
                            axis_right_edge[0] = x1
                            axis_right_edge[1] = y1
                            axis_right_edge[2] = x2
                            axis_right_edge[3] = y2

    right_a = (axis_right_edge[1] - axis_right_edge[3]) / (axis_right_edge[0] - axis_right_edge[2])
    right_b = axis_right_edge[1] - (right_a * axis_right_edge[0])
    left_a = (axis_left_edge[1] - axis_left_edge[3]) / (axis_left_edge[0] - axis_left_edge[2])
    left_b = axis_left_edge[1] - (left_a * axis_left_edge[0])

    return left_a, left_b, right_a, right_b
