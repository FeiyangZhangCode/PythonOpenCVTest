import cv2
import CVFunc
import numpy as np
import math
import datetime
import time


def getDist_P2P(x1_d, y1_d, x2_d, y2_d):
    distance = math.pow((x1_d - x2_d), 2) + math.pow((y1_d - y2_d), 2)
    distance = math.sqrt(distance)
    return distance


# 提取红色线,截取下半部分
def get_red(img_rgb):
    # 提取区域，下半部分
    # start_time = time.time()
    # img_new = np.zeros((mid_height, img_width, 3), np.uint8)
    # for j in range(0, img_width, 1):
    #     for i in range(0, mid_height, 1):
    #         img_new[i, j] = img_rgb[i + mid_height, j]
    # img_red = img_new.copy()
    # end_time = time.time()
    # print('CutHalf:' + str((end_time - start_time) * 1000) + 'ms\n')
    # 提取红色部分
    start_time = time.time()
    img_red = np.zeros((mid_height, img_width, 3), np.uint8)
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
                img_red[i_ut, j_ut] = img_rgb[i_ut, j_ut]
            else:
                img_red[i_ut, j_ut] = (0, 0, 0)
    end_time = time.time()
    print('Select:' + str((end_time - start_time) * 1000) + 'ms\n')
    # 二值化，去噪点
    start_time = time.time()
    gra_red_temp = cv2.cvtColor(img_red, cv2.COLOR_BGR2GRAY)
    ret, gra_red_temp = cv2.threshold(gra_red_temp, 10, 255, cv2.THRESH_BINARY)
    gra_red_temp = CVFunc.func_noise_1(gra_red_temp, 400)
    end_time = time.time()
    print('Noise:' + str((end_time - start_time) * 1000) + 'ms\n')
    return gra_red_temp


# 计算水平线实际距离
def calc_horizontal(int_height, f, w, a, b):
    distance_hor = f * w / (a * int_height + b)
    return distance_hor


# 计算垂直线距离
def calc_vertical(int_width, int_height, f, w, a, b):
    distance_ver = int_width * w / (a * int_height + b)
    return distance_ver


# 开始主程序
# 记录初始化时间
time_mess = ''
start_time = time.time()

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

# 读取模型参数
file_rec = open('Model.txt', 'r', encoding='utf-8')
para_lines = file_rec.readlines()
model_F = float(para_lines[0].strip('\n'))
model_W = float(para_lines[1].strip('\n'))
model_a = float(para_lines[2].strip('\n'))
model_b = float(para_lines[3].strip('\n'))
principal_x = int(para_lines[4].strip('\n'))
principal_y = int(para_lines[5].strip('\n'))

end_time = time.time()
print('Init:' + str((end_time - start_time) * 1000) + 'ms\n')

# while True:
#     ret, frame = cap.read()
file_name = 'h0t0'
frame = cv2.imread('./TestData/' + file_name + '.jpg')
# 提取红色线
# start_time = time.time()
gra_red = get_red(frame)
# end_time = time.time()
# print('GetRed:' + str((end_time - start_time) * 1000) + 'ms\n')

# Canny提取边界
start_time = time.time()
gra_edge = cv2.Canny(gra_red, 70, 140)
end_time = time.time()
print('Canny:' + str((end_time - start_time) * 1000) + 'ms\n')
# 手动去除校准用红色区域
# start_time = time.time()
# gra_edge[0:principal_y+100, :] = 0
# end_time = time.time()
# print('DeletePart:' + str((end_time - start_time) * 1000) + 'ms\n')

# 识别水平、垂直边界线
start_time = time.time()
gra_lines = np.zeros((gra_edge.shape[0], gra_edge.shape[1]), np.uint8)  # 创建个全0的黑背景
y_area = np.zeros([gra_edge.shape[0], 2], np.uint64)  # 0是权重，1是x1和x2中最接近中轴的那一端
ver_lines_angle = []
ver_lines_org = []
lines = cv2.HoughLinesP(gra_edge, rho=1.0, theta=np.pi / 180, threshold=20, minLineLength=20, maxLineGap=20)
try:
    for line in lines:
        for x1_p, y1_p, x2_p, y2_p in line:
            if getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 50.0:
                # cv2.line(gra_lines, (x1, y1), (x2, y2), 255, 1)
                if abs(y1_p - y2_p) < 2 and x2_p > int(img_width / 4) and x1_p < int(img_width * 3 / 4):
                    cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                    y_avg = int((y1_p + y2_p) / 2)
                    y_area[y_avg, 0] += abs(x1_p - x2_p)
                    if abs(x1_p - (img_width / 2)) > abs(x2_p - (img_width / 2)):
                        y_area[y_avg, 1] = x2_p
                    else:
                        y_area[y_avg, 1] = x1_p
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
                    if abs(temp_width - mid_width) >= abs(y_area[i, 1] - mid_width):
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
        # 计算水平线到相机的距离，并画出水平线
    for hor_point in hor_height:
        dis_temp = calc_horizontal(hor_point[0], model_F, model_W, model_a, model_b)
        dis_temp = round(dis_temp, 2)
        cv2.line(frame, (0, hor_point[0] + mid_height), (img_width, hor_point[0] + mid_height), (255, 0, 0), 1)
        cv2.putText(frame, str(dis_temp) + 'mm', (mid_width, hor_point[0] + mid_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 0, 0), 1)
        # print(hor_point[0], dis_temp)
        # 计算垂直线到图像中轴的距离，并画出垂直线
    for ver_line in ver_lines_org:
        for x1, y1, x2, y2 in ver_line:
            dis_ver_1 = calc_vertical(abs(x1 - principal_x), y1, model_F, model_W, model_a, model_b)
            dis_ver_2 = calc_vertical(abs(x2 - principal_x), y2, model_F, model_W, model_a, model_b)
            # dis_ver = (dis_ver_1 + dis_ver_2) / 2
            # print(x1, y1, dis_ver_1)
            # print(x2, y2, dis_ver_2)
            # x_ver = int((x1 + x2) / 2)
            # y_ver = int((y1 + y2) / 2)
            cv2.line(frame, (x1, y1 + mid_height), (x2, y2 + mid_height), (0, 255, 0), 1)
            cv2.putText(frame, str(round(dis_ver_1, 0)), (x1, y1 + mid_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0), 1)
            cv2.putText(frame, str(round(dis_ver_2, 0)), (x2, y2 + mid_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0), 1)
except Exception as e:
    print(e)
end_time = time.time()
print('Hough:' + str((end_time - start_time) * 1000) + 'ms\n')
# 画图像中心十字
cv2.line(frame, (0, mid_height), (img_width, mid_height), (0, 255, 0), 1)
cv2.line(frame, (mid_width, 0), (mid_width, img_height), (0, 255, 0), 1)
# 画相机中心
cv2.circle(frame, (principal_x, principal_y), 5, (255, 0, 0), 3)
cv2.imshow('Cap', frame)
# if cv2.waitKey(1) & 0xFF == ord('q'):
#     break
cv2.waitKey(0)
file_rec.close()
cap.release()
cv2.destroyAllWindows()
