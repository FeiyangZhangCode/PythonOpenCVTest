import cv2
import CVFunc
import numpy as np
import math


def getDist_P2P(x1_d, y1_d, x2_d, y2_d):
    distance = math.pow((x1_d - x2_d), 2) + math.pow((y1_d - y2_d), 2)
    distance = math.sqrt(distance)
    return distance


# 计算水平线实际距离
def calc_horizontal(int_height, f, w, a, b):
    distance_hor = f * w / (a * int_height + b)
    return distance_hor


# 计算垂直线距离
def calc_vertical(int_width, int_height, f, w, a, b):
    distance_ver = int_width * w / (a * int_height + b)
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
    gra_red_temp = cv2.cvtColor(img_red, cv2.COLOR_BGR2GRAY)
    ret, gra_red_temp = cv2.threshold(gra_red_temp, 10, 255, cv2.THRESH_BINARY)
    gra_red_temp = CVFunc.func_noise_1(gra_red_temp, 400)
    return gra_red_temp


def get_parameter(gra_edge):
    # 提取各类型线段
    gra_width_HLP = gra_edge.shape[1]
    mid_width_HLP = int(gra_width_HLP / 2)
    gra_lines = np.zeros((gra_edge.shape[0], gra_edge.shape[1]), np.uint8)  # 创建个全0的黑背景
    y_area = np.zeros([gra_edge.shape[0], 2], np.uint64)  # 0是权重，1是x1和x2中最接近中轴的那一端
    ver_lines_angle = []
    ver_lines_org = []
    num_hor = 0
    lines = cv2.HoughLinesP(gra_edge, rho=1.0, theta=np.pi / 180, threshold=20, minLineLength=20, maxLineGap=20)
    for line in lines:
        for x1_p, y1_p, x2_p, y2_p in line:
            if getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 50.0:
                # cv2.line(gra_lines, (x1, y1), (x2, y2), 255, 1)
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
                    continue
                elif abs(y1_p - y2_p) > 5:
                    cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                    # num_lines += 1
                    w1_f = float(x1_p)
                    w2_f = float(x2_p)
                    h1_f = float(y1_p)
                    h2_f = float(y2_p)
                    a_f = (w1_f - w2_f) / (h1_f - h2_f)
                    b_f = w1_f - (a_f * h1_f)
                    weight_f = getDist_P2P(x1_p, y1_p, x2_p, y2_p)

                    ver_lines_angle.append([a_f, b_f, weight_f])
                    ver_lines_org.append(line)
                    continue
    # 垂直线数组
    ver_lines_angle.sort(key=lambda x: x[0], reverse=True)
    ver_formulas_new = []
    a_sum = 0.0
    b_sum = 0.0
    weight_sum = 0.0
    for ver_line_angle in ver_lines_angle:
        if a_sum == 0.0:
            a_sum = ver_line_angle[0] * ver_line_angle[2]
            b_sum = ver_line_angle[1] * ver_line_angle[2]
            weight_sum = ver_line_angle[2]
        else:
            if abs((a_sum / weight_sum) - ver_line_angle[0]) < 0.1:
                a_sum += ver_line_angle[0] * ver_line_angle[2]
                b_sum += ver_line_angle[1] * ver_line_angle[2]
                weight_sum += ver_line_angle[2]
            else:
                temp_a_avg = round((a_sum / weight_sum), 4)
                temp_b_avg = round((b_sum / weight_sum), 4)
                ver_formulas_new.append([temp_a_avg, temp_b_avg])
                a_sum = ver_line_angle[0] * ver_line_angle[2]
                b_sum = ver_line_angle[1] * ver_line_angle[2]
                weight_sum = ver_line_angle[2]
    if a_sum != 0.0:
        temp_a_avg = round((a_sum / weight_sum), 4)
        temp_b_avg = round((b_sum / weight_sum), 4)
        ver_formulas_new.append([temp_a_avg, temp_b_avg])

    formular_l = [0.0, 0.0]
    formular_r = [0.0, 0.0]
    for i in range(0, len(ver_formulas_new) - 1, 1):
        a_r = ver_formulas_new[i][0]
        a_l = ver_formulas_new[i + 1][0]
        b_r = ver_formulas_new[i][1]
        b_l = ver_formulas_new[i + 1][1]
        if a_r > 0.0 and a_l < 0.0:
            formular_l = [a_l, b_l]
            formular_r = [a_r, b_r]
            break

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

    return hor_height, ver_lines_org, formular_r, formular_l


# 开始主程序
# 提取图像
file_name = 'h59t0'
rgb_frame = cv2.imread('./TestData/' + file_name + '.jpg')
# 获取图像位置参数
img_test = rgb_frame.copy()
img_height = int(img_test.shape[0])
img_width = int(img_test.shape[1])
mid_height = int(img_height / 2)
mid_width = int(img_width / 2)

# 提取红色线
gra_red = get_red(img_test)

# Canny提取边界
gra_edge = cv2.Canny(gra_red, 70, 140)

# 手动去除校准用红色区域
gra_edge[0:100, :] = 0
# cv2.imshow('gra', gra_edge)
# cv2.imwrite('./TestData/' + file_name + '-gra.jpg', gra_edge)
# 计算参数，返回水平线高度，垂直线HoughLinesP结果，从右向左的第3和第2条线的a和b值
hor_lines_points, ver_lines, right_formular, left_formular = get_parameter(gra_edge)

model_F = 1120
model_W = 101
model_a = right_formular[0] - left_formular[0]
model_b = right_formular[1] - left_formular[1]
principal_x = 1006
principal_y = 605

print(model_a, model_b)

# 画图像中心十字
cv2.line(img_test, (mid_width, 0), (mid_width, img_height), (255, 0, 255), 1)
cv2.line(img_test, (0, mid_height), (img_width, mid_height), (255, 0, 255), 1)
# 画相机中心十字
cv2.line(img_test, (principal_x, 0), (principal_x, img_height), (255, 255, 0), 1)
cv2.line(img_test, (0, principal_y), (img_width, principal_y), (255, 255, 0), 1)
# 画标定垂线位置
a_id_l = left_formular[0]
b_id_l = left_formular[1]
a_id_r = right_formular[0]
b_id_r = right_formular[1]
cv2.line(img_test, (int(b_id_l), mid_height), (int(a_id_l * 540 + b_id_l), 540 + mid_height), (0, 255, 255), 1)
cv2.line(img_test, (int(b_id_r), mid_height), (int(a_id_r * 540 + b_id_r), 540 + mid_height), (0, 255, 255), 1)

# 计算水平线到相机的距离，并画出水平线
for hor_point in hor_lines_points:
    dis_temp = calc_horizontal(hor_point[0], model_F, model_W, model_a, model_b)
    dis_temp = round(dis_temp, 2)
    cv2.line(img_test, (0, hor_point[0] + mid_height), (img_width, hor_point[0] + mid_height), (255, 0, 0), 1)
    cv2.putText(img_test, str(dis_temp) + 'mm', (mid_width, hor_point[0] + mid_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (255, 0, 0), 1)
    # print(hor_point[0], dis_temp)

# 计算垂直线到图像中轴的距离，并画出垂直线
for ver_line in ver_lines:
    for x1, y1, x2, y2 in ver_line:
        dis_ver_1 = calc_vertical(abs(x1 - principal_x), y1, model_F, model_W, model_a, model_b)
        dis_ver_2 = calc_vertical(abs(x2 - principal_x), y2, model_F, model_W, model_a, model_b)
        # dis_ver = (dis_ver_1 + dis_ver_2) / 2
        # print(x1, y1, dis_ver_1)
        # print(x2, y2, dis_ver_2)
        # x_ver = int((x1 + x2) / 2)
        # y_ver = int((y1 + y2) / 2)
        cv2.line(img_test, (x1, y1 + mid_height), (x2, y2 + mid_height), (0, 255, 0), 1)
        cv2.putText(img_test, str(round(dis_ver_1, 0)), (x1, y1 + mid_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0), 1)
        cv2.putText(img_test, str(round(dis_ver_2, 0)), (x2, y2 + mid_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0), 1)

cv2.imshow('test', img_test)
cv2.imwrite('./TestData/' + file_name + '-dis.jpg', img_test)

cv2.waitKey(0)
cv2.destroyAllWindows()
