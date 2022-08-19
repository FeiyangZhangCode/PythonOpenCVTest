import os
import time
import datetime
import shutil
import cv2
import CVFunc
import numpy as np
import math

# 读取模型参数
file_model = open('Model.txt', 'r', encoding='utf-8')
para_lines = file_model.readlines()
file_model.close()


# 获取当前时间
def get_current_time():
    """[summary] 获取当前时间

    [description] 用time.localtime()+time.strftime()实现
    :returns: [description] 返回str类型
    """
    ct = time.time()
    local_time = time.localtime(ct)
    data_head = time.strftime("%Y-%m-%d %H:%M:%S", local_time)
    data_secs = (ct - int(ct)) * 1000
    time_stamp = "%s.%04d" % (data_head, data_secs)
    return time_stamp


# 计算两点间距离
def getDist_P2P(x1_d, y1_d, x2_d, y2_d):
    distance = math.pow((x1_d - x2_d), 2) + math.pow((y1_d - y2_d), 2)
    distance = math.sqrt(distance)
    return distance

def get_HoughLinesP(gra_edge, p_x, p_y):
    # 提取各类型线段
    gra_width_HLP = gra_edge.shape[1]
    mid_width_HLP = int(gra_width_HLP / 2)
    gra_lines = np.zeros((gra_edge.shape[0], gra_edge.shape[1]), np.uint8)  # 创建个全0的黑背景
    y_area = np.zeros([gra_edge.shape[0], 2], np.uint64)  # 0是权重，1是x1和x2中最接近中轴的那一端
    ver_lines_org = []
    left_lines = []
    right_lines = []
    num_hor = 0
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
                    continue
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
                        ver_lines_org.append(line)
                        if (result > 0) and (max(x1_p, x2_p) < p_x) and (min(y1_p, y2_p) > p_y):      # 提取左下角垂线
                            left_lines.append(line)
                        elif (result < 0) and (min(x1_p, x2_p) > p_x) and (min(y1_p, y2_p) > p_y):      # 提取右下角垂线
                            right_lines.append(line)


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

    return hor_height, ver_lines_org, left_lines, right_lines


# 图像测距
def dis_calc(rgb_frame, c_id):
    global para_lines
    if int(c_id) == 0:
        model_F = float(para_lines[0].strip('\n'))
        model_W = float(para_lines[1].strip('\n'))
        model_a = float(para_lines[2].strip('\n'))
        model_b = float(para_lines[3].strip('\n'))
        principal_x = int(para_lines[4].strip('\n'))
        principal_y = int(para_lines[5].strip('\n'))
    else:
        model_F = float(para_lines[6].strip('\n'))
        model_W = float(para_lines[7].strip('\n'))
        model_a = float(para_lines[8].strip('\n'))
        model_b = float(para_lines[9].strip('\n'))
        principal_x = int(para_lines[10].strip('\n'))
        principal_y = int(para_lines[11].strip('\n'))

    ret_mess = ''  # 数据信息
    err_mess_all = ''  # 报错信息
    time_mess = ''  # 时间信息
    ret_value = [0.0] * 3  # 0是水平线，1是左垂线，2是右垂线

    # 获取图像位置参数
    img_height = int(rgb_frame.shape[0])
    img_width = int(rgb_frame.shape[1])
    mid_height = int(img_height / 2)
    mid_width = int(img_width / 2)

    # Canny提取边界
    start_time = time.time()
    gra_edge = CVFunc.find_edge_light(rgb_frame)
    # gra_edge_1 = CVFunc.find_edge(rgb_frame)
    # gra_edge = cv2.Canny(rgb_frame, 70, 140)
    end_time = time.time()
    time_mess += 'Can:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'
    # 截取下半部
    start_time = time.time()
    rgb_rot = rgb_frame.copy()
    gra_edge_rot = gra_edge.copy()
    gra_edge_rot[0:principal_y, :] = 0
    end_time = time.time()
    time_mess += 'Half:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

    # 获取水平和垂直线
    start_time = time.time()
    hor_lines_points, ver_lines, left_lines, right_lines = get_HoughLinesP(gra_edge_rot, principal_x, principal_y)
    end_time = time.time()
    time_mess += 'Hou:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

    # 画相机中心十字
    cv2.line(rgb_rot, (principal_x, 0), (principal_x, img_height), (255, 255, 0), 1)
    cv2.line(rgb_rot, (0, principal_y), (img_width, principal_y), (255, 255, 0), 1)
    cv2.circle(rgb_rot, (principal_x, principal_y), 5, (255, 255, 0), 3)

    # 计算水平线到相机的距离，输出最近距离，并画出水平线
    start_time = time.time()
    dis_temp = 9999.99
    hight_edge = - model_b / model_a
    axis_temp_h = 0
    for hor_point in hor_lines_points:
        if float(hor_point[0]) > hight_edge:
            dis_hor = CVFunc.calc_horizontal(hor_point[0], model_F, model_W, model_a, model_b)
            dis_hor = round(dis_hor, 2)
            if dis_hor < dis_temp:
                dis_temp = dis_hor
                axis_temp_h = hor_point[0]
            cv2.line(rgb_rot, (0, hor_point[0]), (img_width, hor_point[0]), (255, 0, 0), 1)
            cv2.putText(rgb_rot, str(dis_hor), (mid_width, hor_point[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 0, 0), 1)
        # else:
        #     cv2.line(rgb_rot, (0, hor_point[0]), (img_width, hor_point[0]), (255, 0, 0), 1)
        #     cv2.putText(rgb_rot, 'out', (mid_width, hor_point[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
        #                 (255, 0, 0), 1)
    # cv2.line(rgb_rot, (0, axis_temp_h), (img_width, axis_temp_h), (255, 0, 0), 1)
    # cv2.putText(rgb_rot, str(dis_temp), (mid_width, axis_temp_h), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
    #             (255, 0, 0), 1)

    ret_value[0] = int(dis_temp)
    ret_mess += 'F,' + str(dis_temp) + '\n'
    end_time = time.time()
    time_mess += 'Hor:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

    # 计算垂直线到图像中轴的距离，并画出垂直线
    start_time = time.time()
    dis_temp_l = 9999.0
    dis_temp_r = 9999.0
    axis_temp_l = [0] * 4
    axis_temp_r = [0] * 4

    ver_left = [[0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0]]      # 平均a，平均b，平均dis，累加权重，最小dis，最小x1，最小y1，最小x2，最小y2
    ver_right = [[0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0]]

    for l_line in left_lines:
        for x1, y1, x2, y2 in l_line:
            # cv2.line(rgb_rot, (x1, y1), (x2, y2), (0, 255, 0), 1)
            # dis_ver_1 = CVFunc.calc_vertical(abs(x1 - principal_x), y1, model_F, model_W, model_a, model_b)
            # dis_ver_2 = CVFunc.calc_vertical(abs(x2 - principal_x), y2, model_F, model_W, model_a, model_b)
            # cv2.putText(rgb_rot, str(round(dis_ver_1, 0)), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
            #             (0, 255, 0), 1)
            # cv2.putText(rgb_rot, str(round(dis_ver_2, 0)), (x2, y2), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
            #             (0, 255, 0), 1)

            if float(y1) > hight_edge and float(y2) > hight_edge and min(y1, y2) > (principal_y + 50):     # 剔除过于接近中线的数据
                dis_ver_1 = CVFunc.calc_vertical(abs(x1 - principal_x), y1, model_F, model_W, model_a, model_b)
                dis_ver_2 = CVFunc.calc_vertical(abs(x2 - principal_x), y2, model_F, model_W, model_a, model_b)
                x1_f = float(x1)
                x2_f = float(x2)
                y1_f = float(y1)
                y2_f = float(y2)
                func_a = (y1_f - y2_f) / (x1_f - x2_f)
                func_b = y1_f - func_a * x1_f
                weight_ver = getDist_P2P(x1, y1, x2, y2)
                avg_dis = (dis_ver_1 + dis_ver_2) / 2
                temp_x = int((principal_y - func_b) / func_a)
                if temp_x < (principal_x + 50):
                    if ver_left[0][0] == 0.0:
                        ver_left[0][0] = func_a
                        ver_left[0][1] = func_b
                        ver_left[0][2] = avg_dis
                        ver_left[0][3] = weight_ver
                        ver_left[0][4] = min(dis_ver_1, dis_ver_2)
                        ver_left[0][5] = x1
                        ver_left[0][6] = y1
                        ver_left[0][7] = x2
                        ver_left[0][8] = y2
                    else:
                        bool_newline = True
                        for i in range(0, len(ver_left), 1):
                            temp_a = ver_left[i][0]
                            temp_b = ver_left[i][1]
                            temp_dis = ver_left[i][2]
                            temp_weight = ver_left[i][3]
                            # if abs(func_a - temp_a) <= 0.05:
                            # if abs(temp_dis - avg_dis) <= 10.0:
                            if abs(func_a - temp_a) <= 0.05 or abs(temp_dis - avg_dis) <= 10.0:     # 如果新线段与已有线段的斜率相近，或者距离相近，则累加进已有线段
                                sum_weight = temp_weight + weight_ver
                                ver_left[i][0] = (temp_a * temp_weight + func_a * weight_ver) / sum_weight
                                ver_left[i][1] = (temp_b * temp_weight + func_b * weight_ver) / sum_weight
                                ver_left[i][2] = (temp_dis * temp_weight + avg_dis * weight_ver) / sum_weight
                                ver_left[i][3] = sum_weight
                                if ver_left[i][4] > min(dis_ver_1, dis_ver_2):
                                    ver_left[i][4] = min(dis_ver_1, dis_ver_2)
                                    ver_left[i][5] = x1
                                    ver_left[i][6] = y1
                                    ver_left[i][7] = x2
                                    ver_left[i][8] = y2
                                bool_newline = False
                                break
                        if bool_newline:
                            temp_list = [func_a, func_b, avg_dis, weight_ver, min(dis_ver_1, dis_ver_2),
                                         x1, y1, x2, y2]
                            ver_left.append(temp_list)

    if abs(ver_left[0][0]) > 0.0:
        for ver_list in ver_left:
            temp_a = ver_list[0]
            temp_b = ver_list[1]
            temp_dis = ver_list[2]
            temp_weight = ver_list[3]
            y_m = principal_y
            y_b = img_height
            x_m = int((y_m - temp_b) / temp_a)
            x_b = int((y_b - temp_b) / temp_a)
            cv2.line(rgb_rot, (x_m, y_m), (x_b, y_b), (0, 255, 0), 1)
            cv2.line(rgb_rot, (ver_list[5], ver_list[6]), (ver_list[7], ver_list[8]), (0, 0, 255), 1)
            cv2.putText(rgb_rot, str(ver_list[4]) + '-' + str(round(temp_dis, 0)),
                        (int((ver_list[5] + ver_list[7]) / 2), int((ver_list[6] + ver_list[8]) / 2)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
            if dis_temp_l > temp_dis:
                dis_temp_l = temp_dis
    else:
        dis_temp_l = 9999.0

    for r_line in right_lines:
        for x1, y1, x2, y2 in r_line:
            if float(y1) > hight_edge and float(y2) > hight_edge:
                dis_ver_1 = CVFunc.calc_vertical(abs(x1 - principal_x), y1, model_F, model_W, model_a, model_b)
                dis_ver_2 = CVFunc.calc_vertical(abs(x2 - principal_x), y2, model_F, model_W, model_a, model_b)
                x1_f = float(x1)
                x2_f = float(x2)
                y1_f = float(y1)
                y2_f = float(y2)
                func_a = (y1_f - y2_f) / (x1_f - x2_f)
                func_b = y1_f - func_a * x1_f
                weight_ver = getDist_P2P(x1, y1, x2, y2)
                avg_dis = (dis_ver_1 + dis_ver_2) / 2
                temp_x = int((principal_y - func_b) / func_a)
                if temp_x > (principal_x - 50):
                    if ver_right[0][0] == 0.0:
                        ver_right[0][0] = func_a
                        ver_right[0][1] = func_b
                        ver_right[0][2] = avg_dis
                        ver_right[0][3] = weight_ver
                        ver_right[0][4] = min(dis_ver_1, dis_ver_2)
                        ver_right[0][5] = x1
                        ver_right[0][6] = y1
                        ver_right[0][7] = x2
                        ver_right[0][8] = y2
                    else:
                        bool_newline = True
                        for i in range(0, len(ver_right), 1):
                            temp_a = ver_right[i][0]
                            temp_b = ver_right[i][1]
                            temp_dis = ver_right[i][2]
                            temp_weight = ver_right[i][3]
                            # if abs(func_a - temp_a) <= 0.05:
                            # if abs(temp_dis - avg_dis) <= 10.0:
                            if abs(func_a - temp_a) <= 0.05 or abs(temp_dis - avg_dis) <= 10.0:     # 如果新线段与已有线段的斜率相近，或者距离相近，则累加进已有线段
                                sum_weight = temp_weight + weight_ver
                                ver_right[i][0] = (temp_a * temp_weight + func_a * weight_ver) / sum_weight
                                ver_right[i][1] = (temp_b * temp_weight + func_b * weight_ver) / sum_weight
                                ver_right[i][2] = (temp_dis * temp_weight + avg_dis * weight_ver) / sum_weight
                                ver_right[i][3] = sum_weight
                                if ver_right[i][4] > min(dis_ver_1, dis_ver_2):
                                    ver_right[i][4] = min(dis_ver_1, dis_ver_2)
                                    ver_right[i][5] = x1
                                    ver_right[i][6] = y1
                                    ver_right[i][7] = x2
                                    ver_right[i][8] = y2
                                bool_newline = False
                                break
                        if bool_newline:
                            temp_list = [func_a, func_b, avg_dis, weight_ver, min(dis_ver_1, dis_ver_2),
                                         x1, y1, x2, y2]
                            ver_right.append(temp_list)
    if abs(ver_right[0][0]) > 0.0:
        for ver_list in ver_right:
            temp_a = ver_list[0]
            temp_b = ver_list[1]
            temp_dis = ver_list[2]
            temp_weight = ver_list[3]
            y_m = principal_y
            y_b = img_height
            x_m = int((y_m - temp_b) / temp_a)
            x_b = int((y_b - temp_b) / temp_a)
            cv2.line(rgb_rot, (x_m, y_m), (x_b, y_b), (0, 255, 0), 1)
            cv2.line(rgb_rot, (ver_list[5], ver_list[6]), (ver_list[7], ver_list[8]), (0, 0, 255), 1)
            cv2.putText(rgb_rot, str(ver_list[4]) + '-' + str(round(temp_dis, 0)),
                        (int((ver_list[5] + ver_list[7]) / 2), int((ver_list[6] + ver_list[8]) / 2)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
            if dis_temp_r > temp_dis:
                dis_temp_r = temp_dis
    else:
        dis_temp_r = 9999.0

    ret_value[1] = int(dis_temp_l)
    ret_value[2] = int(dis_temp_r)
    ret_mess += 'L & R,' + str(dis_temp_l) + ',' + str(dis_temp_r) + '\n'
    end_time = time.time()
    time_mess += 'Ver:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'
    # print('Ver:' + str(round((end_time - start_time) * 1000, 4)) + 'ms')
    # cv2.putText(rgb_rot, str(ret_value[1]) + '+' + str(ret_value[2]), (mid_width, img_height - 100),
    #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

    return rgb_rot, ret_mess, err_mess_all, time_mess, ret_value, gra_edge


# 开始主程序
# frame0 = cv2.imread('../TestData-1/org/C1-111500-686370.jpg')
# frame0_distance, ret0_mess, err0_mess, time0_mess, ret0_value, frame0_edge = dis_calc(frame0, 1)
# cv2.imshow('Dis', frame0_distance)
# # cv2.imshow('Gra', frame0_edge)
# # print(ret0_value)
# # print(time0_mess)
# cv2.waitKey(0)


# 选择摄像头
cap_id = int(input('相机编号(0/1)'))
while (cap_id != 0) and (cap_id != 1):
    cap_id = int(input('相机编号(0/1)'))

# 新建文件夹,读取时间作为文件名
str_fileAddress = '../TestData-1/'
str_fileHome = str_fileAddress + 'org-90/'
str_Time = datetime.datetime.now().strftime('%Y%m%d-%H%M')
file_rec = open(str_fileAddress + str_Time + '.txt', 'w', encoding='utf-8')
str_fileAddress = str_fileAddress + str_Time
if not os.path.exists(str_fileAddress):
    os.makedirs(str_fileAddress)
str_fileAddress += '/'

# 按照最近修改时间升序排序
title_li = os.listdir(str_fileHome)
title_li = sorted(title_li, key=lambda x: os.path.getmtime(os.path.join(str_fileHome, x)), reverse=False)
loop_num = 0
for title in title_li:
    # print(title)
    loop_num = loop_num + 1
    str_Time = datetime.datetime.now().strftime('%H%M%S')
    file_rec.write(str_Time + ',' + str(loop_num) + ',' + title + '\n')
    print(str_Time + ',' + str(loop_num) + ',' + title)
    frame0 = cv2.imread(str_fileHome + title)
    frame0_distance, ret0_mess, err0_mess, time0_mess, ret0_value, frame0_edge = dis_calc(frame0, cap_id)
    # 屏幕输出
    print('C' + str(cap_id) + '  ' + str_Time + '  ' + str(loop_num))
    front_value = 'F' + str(ret0_value[0])
    print(front_value)
    left_value = 'L' + str(ret0_value[1])
    print(left_value)
    right_value = 'R' + str(ret0_value[2])
    print(right_value)

    # if len(ret0_mess) > 0:
    #     # file_rec.write('Get Data:\n' + ret0_mess)
    #     print('Data:\n' + ret0_mess)
    # if len(err0_mess) > 0:
    #     # file_rec.write('Error Message:\n' + err0_mess)
    #     print('Error:\n' + err0_mess)
    # if len(time0_mess) > 0:
    #     print('Timer:\n' + time0_mess)
    cv2.imshow('Dis', frame0_distance)
    cv2.imwrite(str_fileAddress + title, frame0_distance)
    # cv2.imwrite(str_fileAddress + 'g-' + title, frame0_edge)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
