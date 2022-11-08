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


# 计算两点间距离
def getDist_P2P(x1_d, y1_d, x2_d, y2_d):
    distance = math.pow((x1_d - x2_d), 2) + math.pow((y1_d - y2_d), 2)
    distance = math.sqrt(distance)
    return distance


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

start_time_total = time.time()
str_Time = datetime.datetime.now().strftime('%H%M%S-%f')
ret_mess = ''  # 数据信息
time_mess = ''  # 时间信息

# 获取图像及参数
start_time = time.time()
rgb_frame = cv2.imread('./TestData/WIN_20221107_17_38_09_Pro.jpg')
img_height = int(rgb_frame.shape[0])
img_width = int(rgb_frame.shape[1])
mid_height = int(img_height / 2)
mid_width = int(img_width / 2)
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
rgb_half[0:principal_y - 50, :] = (0, 0, 0)
gra_gray = cv2.cvtColor(rgb_half, cv2.COLOR_BGR2GRAY)

gra_canny = cv2.Canny(rgb_half, 200, 400)
#
hsv_half = cv2.cvtColor(rgb_half, cv2.COLOR_BGR2HSV)
low_range = np.array([0, 0, 0])
high_range = np.array([120, 120, 120])
gra_range = cv2.inRange(hsv_half, low_range, high_range)

end_time = time.time()
time_mess += 'Can:' + str(round((end_time - start_time) * 1000, 4)) + ';'

# 获取水平和垂直线
start_time = time.time()
gra_edge_rot = gra_canny.copy()
gra_edge_rot[0:principal_y, :] = 0
lines = cv2.HoughLinesP(gra_edge_rot, rho=1.0, theta=np.pi / 180, threshold=150, minLineLength=150, maxLineGap=50)
end_time = time.time()
time_mess += 'Hou:' + str(round((end_time - start_time) * 1000, 4)) + ';'

# 旋转校正、拉平、融合
start_time = time.time()
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
                    if getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 100.0:

                        # 根据偏航角进行旋转
                        h1 = CVFunc.calc_horizontal(y1_p, model_F, model_W, model_a, model_b)
                        h2 = CVFunc.calc_horizontal(y2_p, model_F, model_W, model_a, model_b)
                        w1 = CVFunc.calc_vertical((x1_p - principal_x), y1_p, model_F, model_W, model_a, model_b)
                        w2 = CVFunc.calc_vertical((x2_p - principal_x), y2_p, model_F, model_W, model_a, model_b)

                        # 水平距离1米以内
                        # if h1 < 2000 and h2 < 2000 and w1 < 2000 and w2 < 2000:
                        if h1 < 200 or h2 < 200:
                            cv2.line(gra_hough, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                            if h1 != h2:
                                angle_tan = np.arctan((w1 - w2) / (h2 - h1)) * 57.29577
                            else:
                                angle_tan = 90.0
                            x_mid = int((x1_p + x2_p) / 2)
                            y_mid = int((y1_p + y2_p) / 2)
                            cv2.line(rgb_rot, (x1_p, y1_p), (x2_p, y2_p), (255, 0, 0), 1)
                            cv2.putText(gra_hough, str(round(angle_tan, 2)), (x_mid, y_mid),
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
#             # 根据偏航角调整融合垂直线
#             if num_ver != 0:
#                 for w1, h1, w2, h2 in lines_ver:
#                     x1_imu, y1_imu = CVFunc.points_rotate(yaw_avg, w1, h1)
#                     x2_imu, y2_imu = CVFunc.points_rotate(yaw_avg, w2, h2)
#                     # 垂直线,按照最接近中轴来拉直
#                     if x1_imu != x2_imu and y1_imu != y2_imu:
#                         if abs(x1_imu) > abs(x2_imu):
#                             x1_imu = x2_imu
#                         else:
#                             x2_imu = x1_imu
#                     # 如果距离中轴的差值不超过50，则认为是同一条，按最接近中轴进行融合。否则新增为新一条。
#                     temp_button = min(y1_imu, y2_imu)
#                     temp_top = max(y1_imu, y2_imu)
#                     if len(data_ver) == 0:
#                         data_ver = np.append(data_ver, [[x1_imu, temp_button, temp_top]], axis=0)
#                     else:
#                         new_ver = True
#                         for values_ver in data_ver:
#                             if abs(values_ver[0] - x1_imu) < 50:
#                                 if abs(values_ver[0]) > abs(x1_imu):
#                                     values_ver[0] = x1_imu
#                                 temp_v = min(values_ver[1], temp_button)
#                                 values_ver[1] = temp_v
#                                 temp_v = max(values_ver[2], temp_top)
#                                 values_ver[2] = temp_v
#                                 new_ver = False
#                                 break
#                         if new_ver:
#                             data_ver = np.append(data_ver, [[x1_imu, temp_button, temp_top]], axis=0)
#                 for values_ver in data_ver:
#                     w1_b, h1_b = CVFunc.points_rotate(-yaw_avg, values_ver[0], values_ver[1])
#                     w2_b, h2_b = CVFunc.points_rotate(-yaw_avg, values_ver[0], values_ver[2])
#                     y1_b = CVFunc.calc_h2y(h1_b, model_F, model_W, model_a, model_b)
#                     y2_b = CVFunc.calc_h2y(h2_b, model_F, model_W, model_a, model_b)
#                     x1_b = CVFunc.calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
#                     x2_b = CVFunc.calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
#                     x_mid = int((x1_b + x2_b) / 2)
#                     y_mid = int((y1_b + y2_b) / 2)
#                     cv2.line(rgb_rot, (x1_b, y1_b), (x2_b, y2_b), (255, 0, 0), 1)
#                     cv2.putText(rgb_rot, str(round(values_ver[0], 0)), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
#                                 (255, 0, 0), 1)
#                     # cv2.line(rgb_show, (x1_b, y1_b), (x2_b, y2_b), (0, 0, 0), 6)
#                     if values_ver[0] < 0:
#                         num_left += 1
#                         if dis_l[0] == 0:
#                             dis_l[0] = int(abs(values_ver[0]))
#                         else:
#                             dis_l.append(int(abs(values_ver[0])))
#                     else:
#                         num_right += 1
#                         if dis_r[0] == 0:
#                             dis_r[0] = int(values_ver[0])
#                         else:
#                             dis_r.append(int(values_ver[0]))
#             if num_left != 0:
#                 dis_l.sort()
#             if num_right != 0:
#                 dis_r.sort()
#     end_time = time.time()
#     time_mess += 'Cal:' + str(round((end_time - start_time) * 1000, 4)) + ';'
except Exception as e:
    print(e)
    print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
    print(f"error line:{e.__traceback__.tb_lineno}")
#
# cv2.imshow('rgb', rgb_rot)
# # print(num_ver, len(lines_ver))
# # print(lines_ver)
# # print(num_hor, len(lines_hor))
# # print(lines_hor)
# print(num_left, len(dis_l))
# print(dis_l)
# print(num_right, len(dis_r))
# print(dis_r)
#
# # 画相机中心十字
# start_time = time.time()
# cv2.line(rgb_rot, (principal_x, 0), (principal_x, img_height), (255, 255, 0), 1)
# cv2.line(rgb_rot, (0, principal_y), (img_width, principal_y), (255, 255, 0), 1)
# cv2.circle(rgb_rot, (principal_x, principal_y), 5, (255, 255, 0), 3)
# # 显示及保存图片
# show_height = 540
# show_width = 960
# rgb_show = cv2.resize(rgb_rot, (show_width, show_height))
# gra_hough_show = cv2.resize(gra_hough, (show_width, show_height))
# rgb_hough_show = cv2.cvtColor(gra_hough_show, cv2.COLOR_GRAY2BGR)
# gra_canny_show = cv2.resize(gra_canny, (show_width, show_height))
# rgb_canny_show = cv2.cvtColor(gra_canny_show, cv2.COLOR_GRAY2BGR)
# rgb_text_show = np.zeros((show_height, show_width, 3), np.uint8)
# # str_0 = 'Roll:' + str(imu_roll) + '  Pitch:' + str(imu_pitch)
# # str_1 = 'iYaw:' + str(imu_yaw) + '  cYaw:' + str(round(yaw_avg, 2)) + '  cOth:' + str(round(other_avg, 2))
# # cv2.putText(rgb_text_show, str_0, (0, int(show_height / 4)), cv2.FONT_HERSHEY_COMPLEX, 0.8, (255, 255, 255), 1)
# # cv2.putText(rgb_text_show, str_1, (0, int(show_height / 2)), cv2.FONT_HERSHEY_COMPLEX, 0.8, (255, 255, 255), 1)
# # 4图拼接
# rgb_mix = np.zeros(((show_height * 2), (show_width * 2), 3), np.uint8)
# rgb_mix[0:show_height, 0:show_width] = rgb_canny_show
# rgb_mix[0:show_height, show_width:(show_width * 2)] = rgb_hough_show
# rgb_mix[show_height:(show_height * 2), 0:show_width] = rgb_show
# rgb_mix[show_height:(show_height * 2), show_width:(show_width * 2)] = rgb_text_show
# # cv2.imshow('Cap' + str(cap_id), rgb_mix)
# # cv2.imwrite(file_address + 'C' + str(cap_id) + '-' + str_Time + '.jpg', rgb_frame)
# # cv2.imwrite(file_address + 'D' + str(cap_id) + '-' + str_Time + '.jpg', rgb_rot)
#
# # 保存txt，传输数据
# # q_s.put(rgb_rot)
# # if q_s.qsize() > 1:
# #     q_s.get()
# # file_rec = open(file_address + str(cap_id) + '.txt', 'a')
# # end_time = time.time()
# # time_mess += 'Shw:' + str(round((end_time - start_time) * 1000, 4)) + ';'
# # end_time_total = time.time()
# # time_mess += 'All:' + str(round((end_time_total - start_time_total) * 1000, 4)) + ';\n'
# #
# # if len(time_mess) > 0:
# #     file_rec.write('Tpy:Timer;' + time_mess)
# # file_rec.close()

# cv2.imshow('Gra', gra_gray)
cv2.imshow('Can', gra_canny)
# cv2.imshow('Ran', gra_range)
cv2.imshow('Hou', gra_hough)
cv2.waitKey(0)