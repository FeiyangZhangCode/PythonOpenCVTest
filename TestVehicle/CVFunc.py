import cv2
import numpy as np
import time
import math
import crcmod


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
                xd_lt, yd_lt = points_dis2xy(yaw_avg, draw_left, draw_top, f, w, a, b, p_x)
                xd_lb, yd_lb = points_dis2xy(yaw_avg, draw_left, draw_button, f, w, a, b, p_x)
                xd_rt, yd_rt = points_dis2xy(yaw_avg, draw_right, draw_top, f, w, a, b, p_x)
                xd_rb, yd_rb = points_dis2xy(yaw_avg, draw_right, draw_button, f, w, a, b, p_x)
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
                xd1_b, yd1_b = points_dis2xy(yaw_avg, rect_value[0], rect_value[2], f, w, a, b, p_x)
                xd2_b, yd2_b = points_dis2xy(yaw_avg, rect_value[0], rect_value[3], f, w, a, b, p_x)
                cv2.line(rgb_draw, (xd1_b, yd1_b), (xd2_b, yd2_b), (0, 255, 0), 2)
        else:
            if rect_value[2] != rect_value[3]:
                num_rect += 1
                draw_left = rect_value[0]
                draw_right = rect_value[1]
                draw_button = rect_value[2]
                draw_top = rect_value[3]
                xd_lt, yd_lt = points_dis2xy(yaw_avg, draw_left, draw_top, f, w, a, b, p_x)
                xd_lb, yd_lb = points_dis2xy(yaw_avg, draw_left, draw_button, f, w, a, b, p_x)
                xd_rt, yd_rt = points_dis2xy(yaw_avg, draw_right, draw_top, f, w, a, b, p_x)
                xd_rb, yd_rb = points_dis2xy(yaw_avg, draw_right, draw_button, f, w, a, b, p_x)
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
                xd1_b, yd1_b = points_dis2xy(yaw_avg, rect_value[0], rect_value[2], f, w, a, b, p_x)
                xd2_b, yd2_b = points_dis2xy(yaw_avg, rect_value[1], rect_value[2], f, w, a, b, p_x)
                cv2.line(rgb_draw, (xd1_b, yd1_b), (xd2_b, yd2_b), (0, 255, 0), 2)

    if len(dis_line) > 1:
        del dis_line[0]
    return rgb_draw, dis_line


# 速度改16进制(2022-10-25）
def trans_speed(str_speed):
    int_speed = int(str_speed)
    cmd_speed = hex(int_speed)[2:]
    if int_speed > 100:
        cmd_speed = '64'
    elif int_speed < 16:
        cmd_speed = '0' + cmd_speed
    return cmd_speed


# 将命令更改为串口输出的16进制，并加上crc8校验(2022-10-25)
def set_order(str_order):
    hex_order = bytes.fromhex(str_order)
    crc8 = crcmod.predefined.Crc('crc-8')
    crc8.update(hex_order)
    if len(hex(crc8.crcValue)[2:]) == 1:
        hex_crc8 = bytes.fromhex('0' + hex(crc8.crcValue)[2:])
    else:
        hex_crc8 = bytes.fromhex(hex(crc8.crcValue)[2:])
    hex_order = hex_order + hex_crc8
    return hex_order


# 根据偏航角、水平距离、垂直距离，反推坐标点在图像上的像素位置（2022-11-18）
def points_dis2xy(angle, org_x, org_y, f, w, a, b, p_x):
    cos_angle = math.cos(math.radians(-angle))
    sin_angle = math.sin(math.radians(-angle))
    flo_w = org_x * cos_angle + org_y * sin_angle
    flo_h = org_y * cos_angle - org_x * sin_angle
    y_back = (f * w / flo_h - b) / a
    temp_x = abs(flo_w) * (a * y_back + b) / w
    if flo_w < 0:
        x_back = p_x - temp_x
    else:
        x_back = p_x + temp_x

    return int(x_back), int(y_back)


# 根据水平线距离反推图像像素高度(2022-10-25)
def calc_h2y(flo_h, f, w, a, b):
    y_back = (f * w / flo_h - b) / a
    return int(y_back)


# 根据垂直线距离反推图像像素宽度(2022-10-25)
def calc_w2x(flo_w, y, f, w, a, b, p_x):
    temp_x = abs(flo_w) * (a * y + b) / w
    if flo_w < 0:
        x_back = p_x - temp_x
    else:
        x_back = p_x + temp_x
    return int(x_back)


# 根据偏航角角度进行坐标旋转(2022-10-25)
def points_rotate(angle, org_x, org_y):
    cos_angle = math.cos(math.radians(angle))
    sin_angle = math.sin(math.radians(angle))
    new_x = org_x * cos_angle + org_y * sin_angle
    new_y = org_y * cos_angle - org_x * sin_angle
    return new_x, new_y


# 计算水平线实际距离（2022-08-01）
def calc_horizontal(int_height, f, w, a, b):
    distance_hor = round((f * w / (a * int_height + b)), 0)
    return distance_hor


# 计算垂直线距离（2022-08-01）
def calc_vertical(int_width, int_height, f, w, a, b):
    distance_ver = round((int_width * w / (a * int_height + b)), 0)
    return distance_ver


# 计算两点间距离(2022-10-25)
def getDist_P2P(x1_d, y1_d, x2_d, y2_d):
    distance = math.pow((x1_d - x2_d), 2) + math.pow((y1_d - y2_d), 2)
    distance = math.sqrt(distance)
    return distance


# 识别水平和垂直线（2022-08-08）
def get_HoughLinesP(gra_edge):
    # 提取各类型线段
    err_mess = ''
    gra_width_HLP = gra_edge.shape[1]
    mid_width_HLP = int(gra_width_HLP / 2)
    gra_lines = np.zeros((gra_edge.shape[0], gra_edge.shape[1]), np.uint8)  # 创建个全0的黑背景
    y_area = np.zeros([gra_edge.shape[0], 2], np.uint64)  # 0是权重，1是x1和x2中最接近中轴的那一端
    ver_lines_org = []
    num_hor = 0
    try:
        lines = cv2.HoughLinesP(gra_edge, rho=1.0, theta=np.pi / 180, threshold=50, minLineLength=50, maxLineGap=5)
        for line in lines:
            for x1_p, y1_p, x2_p, y2_p in line:
                # cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                if getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 50.0:
                    # cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                    if abs(y1_p - y2_p) < 2 and x2_p > int(gra_width_HLP / 4) and x1_p < int(gra_width_HLP * 3 / 4):
                        cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                        y_avg = int((y1_p + y2_p) / 2)
                        y_area[y_avg, 0] += abs(x1_p - x2_p)
                        if abs(x1_p - (gra_width_HLP / 2)) > abs(x2_p - (gra_width_HLP / 2)):
                            y_area[y_avg, 1] = x2_p
                        else:
                            y_area[y_avg, 1] = x1_p
                        num_hor += 1
                        # print(y1, y2)
                    elif abs(y1_p - y2_p) > 5:
                        x1_f = float(x1_p)
                        x2_f = float(x2_p)
                        y1_f = float(y1_p)
                        y2_f = float(y2_p)
                        if x2_f - x1_f != 0:
                            k = -(y2_f - y1_f) / (x2_f - x1_f)
                            result = np.arctan(k) * 57.29577
                        else:
                            result = 90
                        # if abs(result) > 2 and ((x2_p < (mid_width_HLP - 300)) or (x2_p > (mid_width_HLP + 300))):
                        if (85 >= abs(result) > 2) or (
                                (90 > abs(result) > 85) and abs(x1_p - mid_width_HLP) < 100):  # 提取斜线和中间200像素的垂直线
                            # cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                            # num_lines += 1
                            ver_lines_org.append(line)

    except Exception as e:
        err_mess += 'Hough lines\n' + str(e) + '\n'
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
    if len(hor_height) > 1:
        last_id = int(len(hor_height) - 1)
        if (temp_height - hor_height[last_id][0]) > 4:
            hor_height.append(temp_point)

    return hor_height, ver_lines_org, err_mess


# 识别水平线和垂线，分别返回水平、左、右线（2022-08-19）
def get_HoughLinesFLR(gra_edge, p_x, p_y):
    # 提取各类型线段
    err_mess = ''
    gra_width_HLP = gra_edge.shape[1]
    mid_width_HLP = int(gra_width_HLP / 2)
    gra_lines = np.zeros((gra_edge.shape[0], gra_edge.shape[1]), np.uint8)  # 创建个全0的黑背景
    y_area = np.zeros([gra_edge.shape[0], 2], np.uint64)  # 0是权重，1是x1和x2中最接近中轴的那一端
    # ver_lines_org = []
    left_lines = []
    right_lines = []
    num_hor = 0
    try:
        lines = cv2.HoughLinesP(gra_edge, rho=1.0, theta=np.pi / 180, threshold=50, minLineLength=50, maxLineGap=5)
        for line in lines:
            for x1_p, y1_p, x2_p, y2_p in line:
                # cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                if getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 50.0:
                    # cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                    if abs(y1_p - y2_p) < 2 and x2_p > int(gra_width_HLP / 4) and x1_p < int(gra_width_HLP * 3 / 4):
                        cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                        y_avg = int((y1_p + y2_p) / 2)
                        y_area[y_avg, 0] += abs(x1_p - x2_p)
                        if abs(x1_p - (gra_width_HLP / 2)) > abs(x2_p - (gra_width_HLP / 2)):
                            y_area[y_avg, 1] = x2_p
                        else:
                            y_area[y_avg, 1] = x1_p
                        num_hor += 1
                        # print(y1, y2)
                        # continue
                    elif abs(y1_p - y2_p) > 5:
                        x1_f = float(x1_p)
                        x2_f = float(x2_p)
                        y1_f = float(y1_p)
                        y2_f = float(y2_p)
                        if x2_f - x1_f != 0:
                            k = -(y2_f - y1_f) / (x2_f - x1_f)
                            result = np.arctan(k) * 57.29577
                        else:
                            result = 90
                        # if abs(result) > 2 and ((x2_p < (mid_width_HLP - 300)) or (x2_p > (mid_width_HLP + 300))):
                        if 88 >= abs(result) > 2:
                            # cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                            # num_lines += 1
                            # ver_lines_org.append(line)
                            if (result > 0) and (max(x1_p, x2_p) < p_x) and (min(y1_p, y2_p) > p_y):  # 提取左下角垂线
                                left_lines.append(line)
                            elif (result < 0) and (min(x1_p, x2_p) > p_x) and (min(y1_p, y2_p) > p_y):  # 提取右下角垂线
                                right_lines.append(line)
    except Exception as e:
        err_mess += 'Hough lines\n' + str(e) + '\n'
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
    if len(hor_height) > 1:
        last_id = int(len(hor_height) - 1)
        if (temp_height - hor_height[last_id][0]) > 4:
            hor_height.append(temp_point)

    return hor_height, left_lines, right_lines, err_mess


# 边缘检测算法（2022-06-30）
def find_edge(img_org):
    gra_can_mix = np.zeros((img_org.shape[0], img_org.shape[1]), np.uint8)  # 创建个全0的黑背景
    for i in range(20, 120, 20):
        for j in range(100, 450, 50):
            gra_can = cv2.Canny(img_org, i, j)
            gra_can_mix = cv2.add(gra_can_mix, gra_can)

    # ret, gra_can_mix = cv2.threshold(gra_can_mix, 0, 255, cv2.THRESH_BINARY)
    return gra_can_mix


# 边缘检测算法-轻量（2022-08-03）
def find_edge_light(img_org):
    gra_can_mix = np.zeros((img_org.shape[0], img_org.shape[1]), np.uint8)  # 创建个全0的黑背景
    for i in range(20, 100, 50):
        for j in range(100, 250, 100):
            gra_can = cv2.Canny(img_org, i, j)
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


# 计算下半部分的延长线
def extension_line_half(x1, y1, x2, y2, img_height, principal_y):
    a = (y1 - y2) / (x1 - x2)
    b = y1 - (a * x1)
    line_axis = [int] * 4
    # 半图线
    line_axis[0] = int((principal_y - b) / a)
    line_axis[1] = principal_y
    line_axis[2] = int((img_height - b) / a)
    line_axis[3] = img_height

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
