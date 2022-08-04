import os
import time
import datetime
import shutil
import cv2
import CVFunc
import numpy as np
import math

# 读取模型参数
# para_camera = [0.0] * 12
file_model = open('Model.txt', 'r', encoding='utf-8')
para_lines = file_model.readlines()
# if len(para_lines) < 12:
#     for i in range(0, len(para_lines), 1):
#         para_camera[i] = float(para_lines[i].strip('\n'))
# else:
#     for i in range(0, 12, 1):
#         para_camera[i] = float(para_lines[i].strip('\n'))
file_model.close()


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


def getDist_P2P(x1_d, y1_d, x2_d, y2_d):
    distance = math.pow((x1_d - x2_d), 2) + math.pow((y1_d - y2_d), 2)
    distance = math.sqrt(distance)
    return distance


# 图像测距
def get_distance(rgb_frame, c_id):
    global para_lines
    # 获取模型参数
    start_time = time.time()
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
    file_model.close()
    # model_F = 1120.0
    # model_W = 1180.0
    # model_a = 3.7662
    # model_b = -2232.36
    # principal_x = 1006
    # principal_y = 605
    ret_mess = ''
    err_mess_all = ''
    ret_value = [0.0] * 3  # 0是水平线，1是左垂线，2是右垂线

    # 获取图像位置参数
    img_height = int(rgb_frame.shape[0])
    img_width = int(rgb_frame.shape[1])
    mid_height = int(img_height / 2)
    mid_width = int(img_width / 2)

    end_time = time.time()
    # print('Init:' + str((end_time - start_time) * 1000) + 'ms\n')

    # Canny提取边界
    start_time = time.time()
    gra_edge = CVFunc.find_edge_light(rgb_frame)
    end_time = time.time()
    # print('Can:' + str((end_time - start_time) * 1000) + 'ms\n')

    # 计算相机翻滚角
    start_time = time.time()
    angle_roll, err_mess = CVFunc.angle_rotate_roll(gra_edge)
    if len(err_mess) > 0:
        # print(err_mess)
        err_mess_all += err_mess + '\n'
    # 根据翻滚角摆正照片
    rgb_rot = cv2.warpAffine(rgb_frame, cv2.getRotationMatrix2D((mid_width, mid_height), -angle_roll, 1),
                             (img_width, img_height))
    gra_edge_rot = cv2.warpAffine(gra_edge, cv2.getRotationMatrix2D((mid_width, mid_height), -angle_roll, 1),
                                  (img_width, img_height))
    ret, gra_edge_rot = cv2.threshold(gra_edge_rot, 0, 255, cv2.THRESH_BINARY)
    gra_edge_rot[0:principal_y, 0:img_width] = 0
    # cv2.imshow('gra', gra_edge_rot)
    end_time = time.time()
    # print('Rot:' + str((end_time - start_time) * 1000) + 'ms\n')

    # 获取水平和垂直线
    start_time = time.time()
    hor_lines_points, ver_lines, right_formular, left_formular = CVFunc.get_HoughLinesP(gra_edge_rot)
    end_time = time.time()
    # print('Hou:' + str((end_time - start_time) * 1000) + 'ms\n')

    # 画图像中心十字
    start_time = time.time()
    cv2.line(rgb_rot, (mid_width, 0), (mid_width, img_height), (255, 0, 255), 1)
    cv2.line(rgb_rot, (0, mid_height), (img_width, mid_height), (255, 0, 255), 1)
    # 画相机中心十字
    cv2.line(rgb_rot, (principal_x, 0), (principal_x, img_height), (255, 255, 0), 1)
    cv2.line(rgb_rot, (0, principal_y), (img_width, principal_y), (255, 255, 0), 1)
    # 画标定垂线位置
    a_id_l = left_formular[0]
    b_id_l = left_formular[1]
    a_id_r = right_formular[0]
    b_id_r = right_formular[1]
    cv2.line(rgb_rot, (int(a_id_l * mid_height + b_id_l), mid_height), (int(a_id_l * img_height + b_id_l), img_height),
             (0, 255, 255), 1)
    cv2.line(rgb_rot, (int(a_id_r * mid_height + b_id_r), mid_height), (int(a_id_r * img_height + b_id_r), img_height),
             (0, 255, 255), 1)
    end_time = time.time()
    # print('Print:' + str((end_time - start_time) * 1000) + 'ms\n')

    # 计算水平线到相机的距离，输出最近距离，并画出水平线
    start_time = time.time()
    dis_temp = 9999.0
    for hor_point in hor_lines_points:
        dis_hor = CVFunc.calc_horizontal(hor_point[0], model_F, model_W, model_a, model_b)
        dis_hor = round(dis_hor, 2)
        if dis_hor < dis_temp:
            dis_temp = dis_hor
        cv2.line(rgb_rot, (0, hor_point[0]), (img_width, hor_point[0]), (255, 0, 0), 1)
        cv2.putText(rgb_rot, str(dis_hor) + 'mm', (mid_width, hor_point[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 0, 0), 1)
        # print(hor_point[0], dis_temp)
    ret_value[0] = dis_temp
    ret_mess += 'F,' + str(dis_temp) + '\n'
    end_time = time.time()
    # print('Hor:' + str((end_time - start_time) * 1000) + 'ms\n')

    # 计算垂直线到图像中轴的距离，并画出垂直线
    start_time = time.time()
    dis_temp_l = 9999.0
    dis_temp_r = 9999.0
    for ver_line in ver_lines:
        for x1, y1, x2, y2 in ver_line:
            dis_ver_1 = CVFunc.calc_vertical(abs(x1 - principal_x), y1, model_F, model_W, model_a, model_b)
            dis_ver_2 = CVFunc.calc_vertical(abs(x2 - principal_x), y2, model_F, model_W, model_a, model_b)
            cv2.line(rgb_rot, (x1, y1), (x2, y2), (0, 255, 0), 1)
            cv2.putText(rgb_rot, str(round(dis_ver_1, 0)), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0), 1)
            cv2.putText(rgb_rot, str(round(dis_ver_2, 0)), (x2, y2), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0), 1)
            if x2 - principal_x < 0:
                if min(dis_ver_1, dis_ver_2) < dis_temp_l:
                    dis_temp_l = min(dis_ver_1, dis_ver_2)
            else:
                if min(dis_ver_1, dis_ver_2) < dis_temp_r:
                    dis_temp_r = min(dis_ver_1, dis_ver_2)
    ret_value[1] = dis_temp_l
    ret_value[2] = dis_temp_r
    ret_mess += 'L & R,' + str(dis_temp_l) + ',' + str(dis_temp_r) + '\n'
    end_time = time.time()
    # print('Ver:' + str((end_time - start_time) * 1000) + 'ms\n')

    # # 查找最近水平线
    # height_nearest, err_mess = CVFunc.nearest_horizontal_1(gra_edge_rot)
    # if len(err_mess) > 0:
    #     err_mess_all += err_mess + '\n'
    #     # print(err_mess)
    #     # file_rec.write(str_CID + err_mess + '\n')
    # else:
    #     cv2.line(rgb_rot, (1, int(height_nearest)), (img_width - 1, int(height_nearest)), (0, 0, 255), 1)
    #     dis_hor = CVFunc.calc_horizontal(int(height_nearest), model_F, model_W, model_a, model_b)
    #     cv2.putText(rgb_rot, str(dis_hor) + 'mm', (mid_width, int(height_nearest)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
    #                 (0, 0, 255), 1)
    #     # print('前向距离%d像素\n' % height_nearest)
    #     # file_rec.write(str_CID + 'F,' + str(height_nearest) + '\n')
    #     ret_mess += 'F,' + str(dis_hor) + '\n'
    #
    # # 查找最近左右垂直线
    # axis_left_near, axis_right_near, angle_left_near, angle_right_near, err_mess, ver_time_mess = CVFunc.nearest_vertical_2(
    #     gra_edge_rot)
    # # axis_left_near, axis_right_near, angle_left_near, angle_right_near, err_mess = CVFunc.nearest_vertical_1(
    # #     gra_edge_rot)
    # if len(err_mess) > 0:
    #     err_mess_all += err_mess + '\n'
    #     # print(err_mess)
    #     # file_rec.write(str_CID + err_mess + '\n')
    # else:
    #     if abs(angle_left_near) > 0.0:
    #         # 计算距离并画线
    #         dis_l_1 = CVFunc.calc_vertical(abs(axis_left_near[0] - principal_x), axis_left_near[1], model_F, model_W, model_a, model_b)
    #         dis_l_2 = CVFunc.calc_vertical(abs(axis_left_near[2] - principal_x), axis_left_near[3], model_F, model_W, model_a, model_b)
    #         cv2.line(rgb_rot, (axis_left_near[0], axis_left_near[1]), (axis_left_near[2], axis_left_near[3]), (0, 255, 0), 1)
    #         cv2.putText(rgb_rot, str(round(dis_l_1, 0)), (axis_left_near[0], axis_left_near[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
    #                     (0, 255, 0), 1)
    #         cv2.putText(rgb_rot, str(round(dis_l_2, 0)), (axis_left_near[2], axis_left_near[3]), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
    #                     (0, 255, 0), 1)
    #         ret_mess += 'L1,' + str(round(dis_l_1, 0)) + ',' + str(round(dis_l_1, 0)) + '\n'
    #     else:
    #         # print('左侧未找到垂直线')
    #         # file_rec.write(str_CID + 'Not found Left ')
    #         err_mess_all += 'Not found Left 1\n'
    #
    #     if abs(angle_right_near) > 0.0:
    #         # 计算距离并画线
    #         dis_r_1 = CVFunc.calc_vertical(abs(axis_right_near[0] - principal_x), axis_right_near[1], model_F, model_W, model_a, model_b)
    #         dis_r_2 = CVFunc.calc_vertical(abs(axis_right_near[2] - principal_x), axis_right_near[3], model_F, model_W, model_a, model_b)
    #         cv2.line(rgb_rot, (axis_right_near[0], axis_right_near[1]), (axis_right_near[2], axis_right_near[3]), (255, 0, 0), 1)
    #         cv2.putText(rgb_rot, str(round(dis_r_1, 0)), (axis_right_near[0], axis_right_near[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
    #                     (255, 0, 0), 1)
    #         cv2.putText(rgb_rot, str(round(dis_r_2, 0)), (axis_right_near[2], axis_right_near[3]), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
    #                     (255, 0, 0), 1)
    #         ret_mess += 'R1,' + str(round(dis_r_1, 0)) + ',' + str(round(dis_r_2, 0)) + '\n'
    #     else:
    #         err_mess_all += 'Not found right 1\n'
    #         # print('右侧未找到垂直线')
    #         # file_rec.write(str_CID + 'Not found right\n')

    return rgb_rot, ret_mess, err_mess_all


# 开始主程序
# 选择摄像头
cap_id = int(input('相机编号(0/1)'))
while (cap_id != 0) and (cap_id != 1):
    cap_id = int(input('相机编号(0/1)'))

# 新建文件夹,读取时间作为文件名
str_fileAddress = '../TestData-1/'
str_fileHome = str_fileAddress + 'org/'
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
    frame0_distance, ret0_mess, err0_mess = get_distance(frame0, cap_id)
    if len(ret0_mess) > 0:
        # file_rec.write('Get Data:\n' + ret0_mess)
        print('Get Data:\n' + ret0_mess)

    if len(err0_mess) > 0:
        # file_rec.write('Error Message:\n' + err0_mess)
        print('Error:\n' + err0_mess)
    cv2.imshow('Dis', frame0_distance)
    cv2.imwrite(str_fileAddress + title, frame0_distance)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
