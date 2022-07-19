import cv2
import CVFunc
import numpy as np
import matplotlib.pyplot as plt
import math


def getDist_P2P(x1_1, y1_1, x2_1, y2_1):
    distance = math.pow((x1_1 - x2_1), 2) + math.pow((y1_1 - y2_1), 2)
    distance = math.sqrt(distance)
    return distance


# 提取图像
img_org_30 = cv2.imread('./TestData/30.jpg')
img_org_40 = cv2.imread('./TestData/40.jpg')
img_org_50 = cv2.imread('./TestData/50.jpg')
img_test = img_org_30.copy()
img_height = int(img_org_30.shape[0])
img_width = int(img_org_30.shape[1])
mid_height = int(img_height / 2)
mid_width = int(img_width / 2)

gra_can_30 = CVFunc.find_edge(img_org_30)
gra_can_40 = CVFunc.find_edge(img_org_40)
gra_can_50 = CVFunc.find_edge(img_org_50)

img_new = np.zeros(((img_height - 700), (mid_width + 10), 3), np.uint8)

for j in range(0, img_width, 1):
    for i in range(0, img_height, 1):
        if i > 700 and j < (mid_width + 10):
            if gra_can_30[i, j] > 200:
                img_new[i - 700, j, 0] = 255
            if gra_can_40[i, j] > 200:
                img_new[i - 700, j, 1] = 255
            if gra_can_50[i, j] > 200:
                img_new[i - 700, j, 2] = 255
        else:
            gra_can_30[i, j] = 0
            gra_can_40[i, j] = 0
            gra_can_50[i, j] = 0

# cv2.imshow('30', gra_can_30)
# cv2.imshow('40', gra_can_40)
# cv2.imshow('50', gra_can_50)
cv2.imshow('mix', img_new)

# b, g, r = cv2.split(img_org)
# for j in range(0, img_width, 1):
#     for i in range(0, img_height, 1):
#         b_value = b[i, j]
#         g_value = g[i, j]
#         r_value = r[i, j]
#         if b_value < 110 and g_value < 110 and r_value > 130 and abs(float(b_value) - float(g_value)) < 30.0 and i > (
#                 mid_height + 137) and j < (mid_width + 10):
#             continue
#         else:
#             img_test[i, j] = (0, 0, 0)
# cv2.imshow('org', img_test)
# gra_test = cv2.cvtColor(img_test, cv2.COLOR_BGR2GRAY)
# gra_test = CVFunc.func_noise_1(gra_test, 100)
# gra_test = cv2.Canny(gra_test, 50, 300)
#
# gra_lines = np.zeros((img_org.shape[0], img_org.shape[1]), np.uint8)  # 创建个全0的黑背景
# y_area = np.zeros([img_height], np.uint64)
# num_lines = 0
# lines = cv2.HoughLinesP(gra_test, rho=1.0, theta=np.pi / 180, threshold=20, minLineLength=5, maxLineGap=10)
# for line in lines:
#     for x1, y1, x2, y2 in line:
#         # cv2.line(gra_lines, (x1, y1), (x2, y2), 255, 1)
#         if abs(y1 - y2) < 2 and abs(x1 - x2) > 20:
#             # cv2.line(gra_lines, (x1, y1), (x2, y2), 255, 1)
#             # y_area[y1] += 1
#             # y_area[y2] += 1
#             # num_lines += 1
#             # print(y1, y2)
#             continue
#         elif abs(y1 - y2) > 5 and x1 != x2 and getDist_P2P(x1, y1, x2, y2) > 10.0:
#             # continue
#             num_lines += 1
#             x1_f = float(x1)
#             x2_f = float(x2)
#             y1_f = float(y1)
#             y2_f = float(y2)
#             a_f = (y1_f - y2_f) / (x1_f - x2_f)
#             b_f = y1_f - (a_f * x1_f)
#
#             cv2.line(gra_lines, (x1, y1), (x2, y2), 255, 1)
#             print(a_f, b_f)
#         elif x1 == x2:
#             # cv2.line(gra_lines, (x1, y1), (x2, y2), 255, 1)
#             # num_lines += 1
#             # print(x1)
#             continue
# # for i in range(0, 1079, 1):
# #     if y_area[i] > 0:
# #         print(i, y_area[i])
#
# # print(num_lines)
# # gra_org = gra_test.copy()
# #
# cv2.imshow('test', gra_lines)


cv2.waitKey(0)
cv2.destroyAllWindows()
