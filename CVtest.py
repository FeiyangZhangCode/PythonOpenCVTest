import os
import time
import datetime
import shutil
import cv2
import CVFunc
import numpy as np
import math
import CVFunc


# 计算两点间距离
def getDist_P2P(x1_d, y1_d, x2_d, y2_d):
    distance = math.pow((x1_d - x2_d), 2) + math.pow((y1_d - y2_d), 2)
    distance = math.sqrt(distance)
    return distance


# 开始主程序
# 读取模型参数
file_model = open('Model.txt', 'r', encoding='utf-8')
para_lines = file_model.readlines()
file_model.close()

model_F = float(para_lines[0].strip('\n'))
model_W = float(para_lines[1].strip('\n'))
model_a = float(para_lines[2].strip('\n'))
model_b = float(para_lines[3].strip('\n'))
principal_x = int(para_lines[4].strip('\n'))
principal_y = int(para_lines[5].strip('\n'))

rgb_org = cv2.imread('./TestData/Angle/00d00.jpg')
gra_edge = cv2.imread('./TestData/Processing/g-00d00.jpg', 0)
real_angle = 0.0
ret, gra_edge = cv2.threshold(gra_edge, 10, 255, cv2.THRESH_BINARY)
# cv2.imshow('Edge', gra_edge)
# 获取图像位置参数
img_height = int(gra_edge.shape[0])
img_width = int(gra_edge.shape[1])
mid_height = int(img_height / 2)
mid_width = int(img_width / 2)
# 提取各类型线段
gra_width_HLP = gra_edge.shape[1]
mid_width_HLP = int(gra_width_HLP / 2)
gra_lines = np.zeros((gra_edge.shape[0], gra_edge.shape[1]), np.uint8)  # 创建个全0的黑背景
gra_hor = np.zeros((gra_edge.shape[0], gra_edge.shape[1]), np.uint8)  # 创建个全0的黑背景
gra_ver = np.zeros((gra_edge.shape[0], gra_edge.shape[1]), np.uint8)  # 创建个全0的黑背景
y_area = np.zeros([gra_edge.shape[0], 2], np.uint64)  # 0是权重，1是x1和x2中最接近中轴的那一端
ver_lines_org = []
left_lines = []
right_lines = []
num_hor = 0
lines = cv2.HoughLinesP(gra_edge, rho=1.0, theta=np.pi / 180, threshold=50, minLineLength=50, maxLineGap=20)
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
            w1 = CVFunc.calc_vertical(abs(x1_p - principal_x), y1_p, model_F, model_W, model_a, model_b)
            w2 = CVFunc.calc_vertical(abs(x2_p - principal_x), y2_p, model_F, model_W, model_a, model_b)
            if ((x1_p > principal_x) and (x2_p > principal_x)) or ((x1_p < principal_x) and (x2_p < principal_x)):
                w12 = abs(w1 - w2)
            else:
                w12 = abs(w1 + w2)
            h12 = abs(h1 - h2)

            if w12 == 0:
                tan12 = 90.0
            else:
                tan12 = np.arctan(h12 / w12) * 57.29577
                cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                cv2.putText(gra_lines, str(round(tan12, 2)), (int((x1_p + x2_p) / 2), int((y1_p + y2_p) / 2)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255, 1)
                # print(str(round(result, 2)))
            if abs(tan12) <= real_angle + 10:
                # cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                # cv2.putText(gra_lines, str(round(tan12, 2)), (int((x1_p + x2_p) / 2), int((y1_p + y2_p) / 2)),
                #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255, 1)
                cv2.line(rgb_org, (x1_p, y1_p), (x2_p, y2_p), (255, 0, 0), 1)
                cv2.putText(rgb_org, str(round(tan12 - real_angle, 2)), (int((x1_p + x2_p) / 2), int((y1_p + y2_p) / 2)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)
                if num_hor == 0:
                    angle_hor[0] = round(tan12 - real_angle, 2)
                else:
                    angle_hor.append(round(tan12 - real_angle, 2))
                num_hor += 1
                sum_angle_hor += round(tan12 - real_angle, 2)
            elif 90.0 >= abs(tan12) > real_angle + 10:
                # cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                # cv2.putText(gra_lines, str(round(abs(90.0 - tan12), 2)), (int((x1_p + x2_p) / 2), int((y1_p + y2_p) / 2)),
                #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255, 1)
                cv2.line(rgb_org, (x1_p, y1_p), (x2_p, y2_p), (0, 255, 0), 1)
                cv2.putText(rgb_org, str(round(abs(90.0 - tan12) - real_angle, 2)), (int((x1_p + x2_p) / 2), int((y1_p + y2_p) / 2)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                if num_ver == 0:
                    angle_ver[0] = round(abs(90.0 - tan12) - real_angle, 2)
                else:
                    angle_ver.append(round(abs(90.0 - tan12) - real_angle, 2))
                num_ver += 1
                sum_angle_ver += round(abs(90.0 - tan12) - real_angle, 2)
                # print(str(round(result, 2)))

            # if abs(y1_p - y2_p) < 2 and x2_p > int(gra_width_HLP / 4) and x1_p < int(gra_width_HLP * 3 / 4):
            #     cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
            #     y_avg = int((y1_p + y2_p) / 2)
            #     y_area[y_avg, 0] += abs(x1_p - x2_p)
            #     if abs(x1_p - (gra_width_HLP / 2)) > abs(x2_p - (gra_width_HLP / 2)):
            #         y_area[y_avg, 1] = x2_p
            #     else:
            #         y_area[y_avg, 1] = x1_p
            #     num_hor += 1
            #     # print(y1, y2)
            #     continue
            # elif abs(y1_p - y2_p) > 5:
            #     if x2_f - x1_f != 0:
            #         k = -(y2_f - y1_f) / (x2_f - x1_f)
            #         result = np.arctan(k) * 57.29577
            #     else:
            #         result = 90
            #     # if abs(result) > 2 and ((x2_p < (mid_width_HLP - 300)) or (x2_p > (mid_width_HLP + 300))):
            #     if 88 >= abs(result) > 2:
            #         cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
            #         # num_lines += 1
            #         ver_lines_org.append(line)
            #         if (result > 0) and (max(x1_p, x2_p) < principal_x) and (min(y1_p, y2_p) > principal_y):  # 提取左下角垂线
            #             left_lines.append(line)
            #         elif (result < 0) and (min(x1_p, x2_p) > principal_x) and (
            #                 min(y1_p, y2_p) > principal_y):  # 提取右下角垂线
            #             right_lines.append(line)

# 画相机中点和十字
cv2.circle(rgb_org, (principal_x, principal_y), 5, (255, 255, 0), 3)
cv2.line(rgb_org, (principal_x, 0), (principal_x, img_height), (255, 255, 0), 1)
cv2.line(rgb_org, (0, principal_y), (img_width, principal_y), (255, 255, 0), 1)

print(round((sum_angle_hor / num_hor), 2))
print(np.std(angle_hor))
print(round((sum_angle_ver / num_ver), 2))
print(np.std(angle_ver))
rgb_show = np.zeros((mid_height, img_width, 3), np.uint8)  # 创建个全0的黑背景
gra_show = np.zeros((mid_height, img_width), np.uint8)  # 创建个全0的黑背景
gra_show_hor = np.zeros((mid_height, img_width), np.uint8)  # 创建个全0的黑背景
gra_show_ver = np.zeros((mid_height, img_width), np.uint8)  # 创建个全0的黑背景
for i in range(0, mid_height, 1):
    for j in range(0, img_width, 1):
        rgb_show[i, j] = rgb_org[(i + mid_height), j]
        gra_show[i, j] = gra_lines[(i + mid_height), j]
        gra_show_hor[i, j] = gra_hor[(i + mid_height), j]
        gra_show_ver[i, j] = gra_ver[(i + mid_height), j]

# cv2.imshow('Hough', gra_show)
cv2.imshow('Test', rgb_show)
cv2.imwrite('./TestData/Processing/' + str(real_angle) + '.jpg', rgb_show)
# cv2.imshow('Hor', gra_show_hor)
# cv2.imshow('Ver', gra_show_ver)
# # 水平线数组
# hor_height = []
# temp_height = 0
# temp_width = 0
# temp_point = [0, 0]
# for i in range(0, gra_edge.shape[0], 1):
#     if y_area[i, 0] > 0:
#         # print(i, y_area[i, 0], y_area[i, 1])
#         if temp_height == 0:
#             temp_height = i
#             temp_width = y_area[i, 1]
#             temp_point = [temp_height, temp_width]
#         else:
#             if i - temp_height < 5:
#                 temp_height = i
#                 if abs(temp_width - mid_width_HLP) >= abs(y_area[i, 1] - mid_width_HLP):
#                     temp_width = y_area[i, 1]
#                     temp_point = [i, temp_width]
#             else:
#                 hor_height.append(temp_point)
#                 # print(temp_point)
#                 temp_height = i
#                 temp_width = y_area[i, 1]
#                 temp_point = [temp_height, temp_width]
# if len(hor_height) > 1:
#     last_id = int(len(hor_height) - 1)
#     if (temp_height - hor_height[last_id][0]) > 4:
#         hor_height.append(temp_point)

# frame0_resize = cv2.resize(frame0, (1280, 720))
# cv2.imwrite('./TestData/Cali-120-720.jpg', frame0_resize)
# cv2.imshow('Dis', frame0_distance)
# cv2.imshow('Gra', frame0_edge)
# print(ret0_value)
# print(time0_mess)
cv2.waitKey(0)

cv2.destroyAllWindows()
