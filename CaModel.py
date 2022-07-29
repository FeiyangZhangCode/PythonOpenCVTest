import cv2
import CVFunc
import numpy as np
import math
import datetime


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


def get_parameter(img_frame):
    a_calc = 0.0
    b_calc = 0.0
    # 提取红色线
    gra_red = get_red(img_frame)

    # Canny提取边界
    gra_edge = cv2.Canny(gra_red, 70, 140)

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
                    # cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
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
                    # cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                    cv2.line(img_frame, (x1_p, y1_p + mid_height), (x2_p, y2_p + mid_height), (0, 255, 0), 1)
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

    # 画出识别到的水平线
    for hor_height_point in hor_height:
        cv2.line(img_frame, (0, hor_height_point[0] + mid_height), (img_width, hor_height_point[0] + mid_height),
                 (255, 0, 0), 1)

    # 计算a和b，画出用于标定的垂直线
    a_calc = 0.0
    b_calc = 0.0
    for i in range(0, len(ver_formulas_new) - 1, 1):
        a_r = ver_formulas_new[i][0]
        a_l = ver_formulas_new[i + 1][0]
        b_r = ver_formulas_new[i][1]
        b_l = ver_formulas_new[i + 1][1]
        if a_r > 0.0 and a_l < 0.0:
            a_calc = round((a_r - a_l), 4)
            b_calc = round((b_r - b_l), 4)
            cv2.line(img_frame, (int(b_l), mid_height), (int(a_l * 540 + b_l), 540 + mid_height),
                     (0, 255, 255), 1)
            cv2.line(img_frame, (int(b_r), mid_height), (int(a_r * 540 + b_r), 540 + mid_height),
                     (0, 255, 255), 1)
            break

    return a_calc, b_calc, img_frame


# 开始主程序
# 连接摄像头
cap = cv2.VideoCapture(0)
cap.set(6, 1196444237)
cap.set(3, 1920)
cap.set(4, 1080)
cap.set(5, 30)
# 获取图像位置参数
img_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
img_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
mid_height = int(img_height / 2)
mid_width = int(img_width / 2)

# 打开模型参数存储文件
file_rec = open('Model.txt', 'w', encoding='utf-8')

# 录入焦距和间距
model_f = int(input('焦距F'))
model_w = int(input('间距W'))
principal_x = int(input('相机x'))
principal_y = int(input('相机y'))

while True:
    ret, frame = cap.read()

    if cv2.waitKey(1) & 0xFF == ord('c'):
        str_time = datetime.datetime.now().strftime('%H%M%S')
        str_address = './TestData/'
        # cv2.imwrite(str_address + str_time + '.jpg', frame)
        model_a, model_b, frame_lines = get_parameter(frame)
        cv2.line(frame_lines, (0, mid_height), (img_width, mid_height), (0, 255, 0), 1)
        cv2.line(frame_lines, (mid_width, 0), (mid_width, img_height), (0, 255, 0), 1)
        cv2.circle(frame_lines, (1092, 571), 5, (255, 0, 0), 3)
        cv2.imshow('Lines', frame_lines)
        print(model_f, model_w, model_a, model_b)
        file_rec.write(str(model_f) + '\n' + str(model_w) + '\n' + str(model_a) + '\n' + str(model_b) + '\n' + str(principal_x) + '\n' + str(principal_y) + '\n')
    elif cv2.waitKey(1) & 0xFF == ord('q'):
        break

    cv2.line(frame, (0, mid_height), (img_width, mid_height), (0, 255, 0), 1)
    cv2.line(frame, (mid_width, 0), (mid_width, img_height), (0, 255, 0), 1)
    # cv2.circle(frame, (principal_x, principal_y), 5, (255, 0, 0), 3)
    cv2.imshow('Cap', frame)

file_rec.close()
cap.release()
cv2.destroyAllWindows()
