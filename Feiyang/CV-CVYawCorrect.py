import time
import cv2
import numpy as np
import math
import CVFunc
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

rgb_frame = cv2.imread('../TestData/Angle/10d81.jpg')
angle_set = -10.81  # 车头向左偏，需要顺时针旋转，角度为负。车头向右偏，需要逆时针旋转，角度为正。
img_height = int(rgb_frame.shape[0])
img_width = int(rgb_frame.shape[1])
mid_height = int(img_height / 2)
mid_width = int(img_width / 2)

# 提取红色线
gra_red = get_red(rgb_frame)

# Canny提取边界，保留下半部分
start_time = time.time()
# gra_edge = CVFunc.find_edge_light(rgb_frame)
minCan = 40
maxCan = 100
gra_edge = cv2.Canny(gra_red, minCan, maxCan)
end_time = time.time()
time_mess += 'Canny:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

start_time = time.time()
rgb_rot = rgb_frame.copy()
gra_edge_rot = gra_edge.copy()
gra_edge_rot[0:principal_y + 170, :] = 0
end_time = time.time()
time_mess += 'Half:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

rgb_type_imu = rgb_frame.copy()

# 获取水平和垂直线
try:
    start_time = time.time()
    lines = cv2.HoughLinesP(gra_edge_rot, rho=1.0, theta=np.pi / 180, threshold=20, minLineLength=20, maxLineGap=5)
    num_line = 0
    num_ver = 0
    num_hor = 0
    data_ver = np.zeros((0, 3), dtype=float)  # 垂直线数据，0是最接近中轴的w（x）值，1是最接近相机的h（y）值，2是最远离相机的h（y）值
    data_hor = np.zeros((0, 3), dtype=float)  # 水平线数据，0是最接近相机的h（y）值，1是最左侧（最小）的w（x）值，2是最右侧（最大）的w（x）值

    end_time = time.time()
    time_mess += 'Hough:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

    # 识别线提取偏航角
    start_time = time.time()
    hor_angle_avg = 0.0
    hor_angle_weight = 0
    ver_angle_avg = 0.0
    ver_angle_weight = 0

    for line in lines:
        for x1_p, y1_p, x2_p, y2_p in line:
            line_lengh = getDist_P2P(x1_p, y1_p, x2_p, y2_p)
            if line_lengh > 50.0:
                cv2.line(rgb_rot, (x1_p, y1_p), (x2_p, y2_p), (255, 0, 0), 1)
                # x_plt = np.array([x1_p, x2_p])
                # y_plt = np.array([y1_p, y2_p])
                # plt.plot(x_plt, y_plt, 'r')
                num_line += 1
                # cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                x_mid = int((x1_p + x2_p) / 2)
                y_mid = int((y1_p + y2_p) / 2)
                h1 = CVFunc.calc_horizontal(y1_p, model_F, model_W, model_a, model_b)
                h2 = CVFunc.calc_horizontal(y2_p, model_F, model_W, model_a, model_b)
                w1 = CVFunc.calc_vertical((x1_p - principal_x), y1_p, model_F, model_W, model_a, model_b)
                w2 = CVFunc.calc_vertical((x2_p - principal_x), y2_p, model_F, model_W, model_a, model_b)
                # if x1_p == x2_p:
                #     tan_hough = 90
                # else:
                #     tan_hough = np.arctan((y1_p - y2_p) / (x1_p - x2_p)) * 57.29577
                if w1 == w2:
                    tan_line = 90
                else:
                    tan_line = np.arctan((h1 - h2) / (w1 - w2)) * 57.29577
                if abs(tan_line) > 60:
                    tan_line = tan_line - 90.0
                    ver_angle_avg = (ver_angle_avg * ver_angle_weight + tan_line * line_lengh) / (ver_angle_weight + line_lengh)
                    ver_angle_weight = ver_angle_weight + line_lengh
                else:
                    hor_angle_avg = (hor_angle_avg * hor_angle_weight + tan_line * line_lengh) / (hor_angle_weight + line_lengh)
                    hor_angle_weight = hor_angle_weight + line_lengh
                # str_func = ''
                # if h1 == h2:
                #     str_func = 'h=' + str(round(h1, 2))
                # elif w1 == w2:
                #     str_func = 'w=' + str(round(w1, 2))
                # else:
                #     temp_a = (h1 - h2) / (w1 - w2)
                #     temp_b = h1 - temp_a * w1
                #     str_func = 'h=' + str(round(temp_a, 2)) + 'w+' + str(round(temp_b, 2))

                cv2.line(rgb_rot, (x1_p, y1_p), (x2_p, y2_p), (255, 0, 0), 1)
                # cv2.putText(rgb_rot, str_func, (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX,
                #             0.6, (255, 0, 0), 1)
                cv2.putText(rgb_rot, str(round(tan_line, 2)), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 0, 0), 1)

                x_plt = np.array([w1, w2])
                y_plt = np.array([h1, h2])
                plt.plot(x_plt, y_plt, 'r')
                # cv2.putText(rgb_rot, str(w1) + ',' + str(h1), (x1_p, y1_p), cv2.FONT_HERSHEY_SIMPLEX,
                #             0.4, (255, 0, 0), 1)
                # cv2.putText(rgb_rot, str(w2) + ',' + str(h2), (x2_p, y2_p), cv2.FONT_HERSHEY_SIMPLEX,
                #             0.4, (255, 0, 0), 1)

    # all_angle_avg = (hor_angle_avg * hor_angle_weight + ver_angle_avg * ver_angle_weight) / (hor_angle_weight + ver_angle_weight)
    # print('all', str(all_angle_avg), str(angle_set - all_angle_avg))
    # print('ver', str(ver_angle_avg), str(angle_set - ver_angle_avg))
    # print('hor', str(hor_angle_avg), str(angle_set - hor_angle_avg))
    end_time = time.time()
    time_mess += 'Angle:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

    # 旋转校正、拉平、融合
    start_time = time.time()
    for line in lines:
        for x1_p, y1_p, x2_p, y2_p in line:
            # cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
            if getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 50.0:
                x_mid = int((x1_p + x2_p) / 2)
                y_mid = int((y1_p + y2_p) / 2)
                h1 = CVFunc.calc_horizontal(y1_p, model_F, model_W, model_a, model_b)
                h2 = CVFunc.calc_horizontal(y2_p, model_F, model_W, model_a, model_b)
                w1 = CVFunc.calc_vertical((x1_p - principal_x), y1_p, model_F, model_W, model_a, model_b)
                w2 = CVFunc.calc_vertical((x2_p - principal_x), y2_p, model_F, model_W, model_a, model_b)

                # 第一种方式，根据IMU角度进行旋转
                x1_imu, y1_imu = points_rotate(angle_set, w1, h1)
                x2_imu, y2_imu = points_rotate(angle_set, w2, h2)

                # 如果调整后不为水平线或者垂直线，进行拉平拉直
                if x1_imu != x2_imu and y1_imu != y2_imu:
                    temp_tan = np.arctan((y1_imu - y2_imu) / (x1_imu - x2_imu)) * 57.29577
                    if abs(temp_tan) <= 30:  # 判断是水平线,按照最接近相机来拉平
                        if abs(x1_imu) > abs(x2_imu):
                            y1_imu = y2_imu
                        else:
                            y2_imu = y1_imu
                    elif abs(temp_tan) >= 60:  # 判断是垂直线,按照最接近中轴来拉直
                        if abs(x1_imu) > abs(x2_imu):
                            x1_imu = x2_imu
                        else:
                            x2_imu = x1_imu

                # 如果是同一条线，进行融合
                temp_show = ''
                if y1_imu == y2_imu:  # 水平线
                    num_hor += 1
                    temp_show = 'Y' + str(round(y1_imu, 0))
                    # 如果距离中轴的差值不超过20，则认为是同一条，按照最接近中轴的值进行融合。否则新增为新一条。
                    temp_left = min(x1_imu, x2_imu)
                    temp_right = max(x1_imu, x2_imu)
                    if len(data_hor) == 0:
                        data_hor = np.append(data_hor, [[y1_imu, temp_left, temp_right]], axis=0)
                    else:
                        new_hor = True
                        for values_hor in data_hor:
                            if abs(values_hor[0] - y1_imu) < 20:
                                if abs((values_hor[2] + values_hor[1]) / 2) > abs((temp_right + temp_left) / 2):
                                    values_hor[0] = y1_imu
                                values_hor[1] = min(values_hor[1], temp_left)
                                values_hor[2] = max(values_hor[2], temp_right)
                                new_hor = False
                                break
                        if new_hor:
                            data_hor = np.append(data_hor, [[y1_imu, temp_left, temp_right]], axis=0)
                elif x1_imu == x2_imu:
                    num_ver += 1
                    temp_show = 'X' + str(round(x1_imu, 0))
                    # 如果距离中轴的差值不超过20，则认为是同一条，按最接近中轴进行融合。否则新增为新一条。
                    temp_button = min(y1_imu, y2_imu)
                    temp_top = max(y1_imu, y2_imu)
                    if len(data_ver) == 0:
                        data_ver = np.append(data_ver, [[x1_imu, temp_button, temp_top]], axis=0)
                    else:
                        new_ver = True
                        for values_ver in data_ver:
                            if abs(values_ver[0] - x1_imu) < 20:
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
                # cv2.line(rgb_type_imu, (x1_p, y1_p), (x2_p, y2_p), (255, 0, 0), 1)
                # cv2.putText(rgb_type_imu, temp_show, (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
                # x_plt = np.array([x1_imu, x2_imu])
                # y_plt = np.array([y1_imu, y2_imu])
                # plt.plot(x_plt, y_plt, 'g')

                # sheet.write(num_line, 5, x1_imu)
                # sheet.write(num_line, 6, y1_imu)
                # sheet.write(num_line, 7, x2_imu)
                # sheet.write(num_line, 8, y2_imu)

    # print(num_line)
    # print(num_hor, num_ver)
    # print(len(data_hor), len(data_ver))

    # 反推图像上的直线位置
    for values_hor in data_hor:
        w1_b, h1_b = points_rotate(-angle_set, values_hor[1], values_hor[0])
        w2_b, h2_b = points_rotate(-angle_set, values_hor[2], values_hor[0])
        y1_b = calc_h2y(h1_b, model_F, model_W, model_a, model_b)
        y2_b = calc_h2y(h2_b, model_F, model_W, model_a, model_b)
        x1_b = calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
        x2_b = calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
        cv2.line(rgb_type_imu, (x1_b, y1_b), (x2_b, y2_b), (255, 0, 0), 1)
        x_mid = int((x1_b + x2_b) / 2)
        y_mid = int((y1_b + y2_b) / 2)
        cv2.putText(rgb_type_imu, str(round(values_hor[0], 0)), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)

        # x_plt = np.array([values_hor[1], values_hor[2]])
        # y_plt = np.array([values_hor[0], values_hor[0]])
        x_plt = np.array([w1_b, w2_b])
        y_plt = np.array([h1_b, h2_b])
        plt.plot(x_plt, y_plt, 'b')
    for values_ver in data_ver:
        w1_b, h1_b = points_rotate(-angle_set, values_ver[0], values_ver[1])
        w2_b, h2_b = points_rotate(-angle_set, values_ver[0], values_ver[2])
        y1_b = calc_h2y(h1_b, model_F, model_W, model_a, model_b)
        y2_b = calc_h2y(h2_b, model_F, model_W, model_a, model_b)
        x1_b = calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
        x2_b = calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
        cv2.line(rgb_type_imu, (x1_b, y1_b), (x2_b, y2_b), (255, 0, 0), 1)
        x_mid = int((x1_b + x2_b) / 2)
        y_mid = int((y1_b + y2_b) / 2)
        cv2.putText(rgb_type_imu, str(round(values_ver[0], 0)), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 0, 0), 1)

        # x_plt = np.array([values_ver[0], values_ver[0]])
        # y_plt = np.array([values_ver[1], values_ver[2]])
        x_plt = np.array([w1_b, w2_b])
        y_plt = np.array([h1_b, h2_b])
        plt.plot(x_plt, y_plt, 'b')
    end_time = time.time()
    time_mess += 'Rotate:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

    # 第二种参数比较
    num_ver_1 = 0
    num_hor_1 = 0
    data_ver_1 = np.zeros((0, 3), dtype=float)  # 垂直线数据，0是最接近中轴的w（x）值，1是最接近相机的h（y）值，2是最远离相机的h（y）值
    data_hor_1 = np.zeros((0, 3), dtype=float)  # 水平线数据，0是最接近相机的h（y）值，1是最左侧（最小）的w（x）值，2是最右侧（最大）的w（x）值
    rgb_type_imu_1 = rgb_frame.copy()
    start_time = time.time()
    for line in lines:
        for x1_p, y1_p, x2_p, y2_p in line:
            # cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
            if getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 50.0:
                x_mid = int((x1_p + x2_p) / 2)
                y_mid = int((y1_p + y2_p) / 2)
                h1 = CVFunc.calc_horizontal(y1_p, model_F, model_W, model_a, model_b)
                h2 = CVFunc.calc_horizontal(y2_p, model_F, model_W, model_a, model_b)
                w1 = CVFunc.calc_vertical((x1_p - principal_x), y1_p, model_F, model_W, model_a, model_b)
                w2 = CVFunc.calc_vertical((x2_p - principal_x), y2_p, model_F, model_W, model_a, model_b)

                # 第二种方式，根据IMU角度进行旋转
                x1_imu, y1_imu = points_rotate(ver_angle_avg, w1, h1)
                x2_imu, y2_imu = points_rotate(ver_angle_avg, w2, h2)

                # 如果调整后不为水平线或者垂直线，进行拉平拉直
                if x1_imu != x2_imu and y1_imu != y2_imu:
                    temp_tan = np.arctan((y1_imu - y2_imu) / (x1_imu - x2_imu)) * 57.29577
                    if abs(temp_tan) <= 30:  # 判断是水平线,按照最接近相机来拉平
                        if abs(y1_imu) > abs(y2_imu):
                            y1_imu = y2_imu
                        else:
                            y2_imu = y1_imu
                    elif abs(temp_tan) >= 60:  # 判断是垂直线,按照最接近中轴来拉直
                        if abs(y1_imu) > abs(y2_imu):
                            x1_imu = x2_imu
                        else:
                            x2_imu = x1_imu

                # 如果是同一条线，进行融合
                temp_show = ''
                if y1_imu == y2_imu:  # 水平线
                    num_hor_1 += 1
                    temp_show = 'Y' + str(round(y1_imu, 0))
                    # 如果距离中轴的差值不超过20，则认为是同一条，按照最接近中轴的值进行融合。否则新增为新一条。
                    temp_left = min(x1_imu, x2_imu)
                    temp_right = max(x1_imu, x2_imu)
                    if len(data_hor_1) == 0:
                        data_hor_1 = np.append(data_hor_1, [[y1_imu, temp_left, temp_right]], axis=0)
                    else:
                        new_hor = True
                        for values_hor in data_hor_1:
                            if abs(values_hor[0] - y1_imu) < 20:
                                if abs((values_hor[2] + values_hor[1]) / 2) > abs((temp_right + temp_left) / 2):
                                    values_hor[0] = y1_imu
                                values_hor[1] = min(values_hor[1], temp_left)
                                values_hor[2] = max(values_hor[2], temp_right)
                                new_hor = False
                                break
                        if new_hor:
                            data_hor_1 = np.append(data_hor_1, [[y1_imu, temp_left, temp_right]], axis=0)
                elif x1_imu == x2_imu:
                    num_ver_1 += 1
                    temp_show = 'X' + str(round(x1_imu, 0))
                    # 如果距离中轴的差值不超过20，则认为是同一条，按最接近中轴进行融合。否则新增为新一条。
                    temp_button = min(y1_imu, y2_imu)
                    temp_top = max(y1_imu, y2_imu)
                    if len(data_ver_1) == 0:
                        data_ver_1 = np.append(data_ver_1, [[x1_imu, temp_button, temp_top]], axis=0)
                    else:
                        new_ver = True
                        for values_ver in data_ver_1:
                            if abs(values_ver[0] - x1_imu) < 20:
                                if abs(values_ver[0]) > abs(x1_imu):
                                    values_ver[0] = x1_imu
                                temp_v = min(values_ver[1], temp_button)
                                values_ver[1] = temp_v
                                temp_v = max(values_ver[2], temp_top)
                                values_ver[2] = temp_v
                                new_ver = False
                                break
                        if new_ver:
                            data_ver_1 = np.append(data_ver_1, [[x1_imu, temp_button, temp_top]], axis=0)
                else:
                    temp_show = str(round(x1_imu, 0)) + ',' + str(round(y1_imu, 0)) + '\n' + str(
                        round(x2_imu, 0)) + ',' + str(round(y2_imu, 0))
                # cv2.line(rgb_type_imu_1, (x1_p, y1_p), (x2_p, y2_p), (255, 0, 0), 1)
                # cv2.putText(rgb_type_imu_1, temp_show, (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
                # x_plt = np.array([x1_imu, x2_imu])
                # y_plt = np.array([y1_imu, y2_imu])
                # plt.plot(x_plt, y_plt, 'g')

                # sheet.write(num_line, 5, x1_imu)
                # sheet.write(num_line, 6, y1_imu)
                # sheet.write(num_line, 7, x2_imu)
                # sheet.write(num_line, 8, y2_imu)

    # print(num_line)
    # print(num_hor_1, num_ver_1)
    # print(len(data_hor_1), len(data_ver_1))

    # 反推图像上的直线位置
    for values_hor in data_hor_1:
        w1_b, h1_b = points_rotate(-ver_angle_avg, values_hor[1], values_hor[0])
        w2_b, h2_b = points_rotate(-ver_angle_avg, values_hor[2], values_hor[0])
        y1_b = calc_h2y(h1_b, model_F, model_W, model_a, model_b)
        y2_b = calc_h2y(h2_b, model_F, model_W, model_a, model_b)
        x1_b = calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
        x2_b = calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
        cv2.line(rgb_type_imu_1, (x1_b, y1_b), (x2_b, y2_b), (255, 0, 0), 1)
        x_mid = int((x1_b + x2_b) / 2)
        y_mid = int((y1_b + y2_b) / 2)
        cv2.putText(rgb_type_imu_1, str(round(values_hor[0], 0)), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)

        # x_plt = np.array([values_hor[1], values_hor[2]])
        # y_plt = np.array([values_hor[0], values_hor[0]])
        x_plt = np.array([w1_b, w2_b])
        y_plt = np.array([h1_b, h2_b])
        plt.plot(x_plt, y_plt, 'g')
    for values_ver in data_ver_1:
        w1_b, h1_b = points_rotate(-ver_angle_avg, values_ver[0], values_ver[1])
        w2_b, h2_b = points_rotate(-ver_angle_avg, values_ver[0], values_ver[2])
        y1_b = calc_h2y(h1_b, model_F, model_W, model_a, model_b)
        y2_b = calc_h2y(h2_b, model_F, model_W, model_a, model_b)
        x1_b = calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
        x2_b = calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
        cv2.line(rgb_type_imu_1, (x1_b, y1_b), (x2_b, y2_b), (255, 0, 0), 1)
        x_mid = int((x1_b + x2_b) / 2)
        y_mid = int((y1_b + y2_b) / 2)
        cv2.putText(rgb_type_imu_1, str(round(values_ver[0], 0)), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 0, 0), 1)

        # x_plt = np.array([values_ver[0], values_ver[0]])
        # y_plt = np.array([values_ver[1], values_ver[2]])
        x_plt = np.array([w1_b, w2_b])
        y_plt = np.array([h1_b, h2_b])
        plt.plot(x_plt, y_plt, 'g')
    end_time = time.time()
    time_mess += 'Rotate-1:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

except Exception as e:
    print(e)

start_time = time.time()
# 画相机中心十字
cv2.line(rgb_rot, (principal_x, 0), (principal_x, img_height), (255, 255, 0), 1)
cv2.line(rgb_rot, (0, principal_y), (img_width, principal_y), (255, 255, 0), 1)
cv2.circle(rgb_rot, (principal_x, principal_y), 5, (255, 255, 0), 3)
cv2.line(rgb_type_imu, (principal_x, 0), (principal_x, img_height), (255, 255, 0), 1)
cv2.line(rgb_type_imu, (0, principal_y), (img_width, principal_y), (255, 255, 0), 1)
cv2.circle(rgb_type_imu, (principal_x, principal_y), 5, (255, 255, 0), 3)
cv2.line(rgb_type_imu_1, (principal_x, 0), (principal_x, img_height), (255, 255, 0), 1)
cv2.line(rgb_type_imu_1, (0, principal_y), (img_width, principal_y), (255, 255, 0), 1)
cv2.circle(rgb_type_imu_1, (principal_x, principal_y), 5, (255, 255, 0), 3)

rgb_half = np.zeros((mid_height, img_width, 3), np.uint8)  # 创建个全0的黑背景
rgb_half_imu = np.zeros((mid_height, img_width, 3), np.uint8)  # 创建个全0的黑背景
rgb_half_imu_1 = np.zeros((mid_height, img_width, 3), np.uint8)  # 创建个全0的黑背景
for i in range(0, mid_height, 1):
    for j in range(0, img_width, 1):
        rgb_half[i, j] = rgb_rot[(i + mid_height), j]
        rgb_half_imu[i, j] = rgb_type_imu[(i + mid_height), j]
        rgb_half_imu_1[i, j] = rgb_type_imu_1[(i + mid_height), j]
end_time = time.time()
time_mess += 'Show:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

# 显示及保存图片
# print(time_mess)
# cv2.imshow('Cap', rgb_rot)
# cv2.imshow('Cap', rgb_half)
cv2.imshow('IMU', rgb_half_imu)
cv2.imshow('IMU-1', rgb_half_imu_1)
plt.show()
# cv2.imwrite(file_address + 'C' + str(cap_id) + '-' + str_Time + '.jpg', rgb_frame)
# cv2.imwrite(file_address + 'D' + str(cap_id) + '-' + str_Time + '.jpg', frame_distance)
cv2.waitKey(0)

cv2.destroyAllWindows()
