import os
import time
import datetime
import shutil
import cv2
import CVFunc
import numpy as np


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


def nearest_vertical_2(rgb_rot, gra_edge_rot):
    # 保留下半部
    start_time = time.time()

    err_mess = ''
    time_mess = ''
    img_height = int(gra_edge_rot.shape[0])
    img_width = int(gra_edge_rot.shape[1])
    mid_height = int(img_height / 2)
    mid_width = int(img_width / 2)
    gra_test = gra_edge_rot
    rgb_ver = rgb_rot

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
    b_left_near = 0.0
    angle_right_near_ver = 0.0
    axis_right_near_ver = [int] * 8
    b_right_near = 0.0

    gra_temp = np.zeros((img_height, img_width), np.uint8)

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
                            cv2.line(gra_temp, (x1, y1), (x2, y2), 255, 1)
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
                            cv2.line(gra_temp, (x1, y1), (x2, y2), 255, 1)
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


# 主程序
if __name__ == '__main__':

    # 获取时间段，初始化
    str_start_all = get_current_time()
    start_time = time.time()

    # 正常识别
    # rgb_frame = cv2.imread('TestData-1/org/P1-15-51-17-658303.jpg')
    # 识别到上半部线段
    rgb_frame = cv2.imread('TestData-1/org/P1-15-50-15-743574.jpg')

    # 获取图像位置参数
    img_height = int(rgb_frame.shape[0])
    img_width = int(rgb_frame.shape[1])
    mid_height = int(img_height / 2)
    mid_width = int(img_width / 2)

    # 获取时间段，提取边界
    end_time = time.time()
    init_time = str((end_time - start_time) * 1000) + 'ms'
    start_time = time.time()

    # Canny提取边界
    gra_edge = CVFunc.find_edge(rgb_frame)

    # 获取时间段，校正翻滚角
    end_time = time.time()
    edge_time = str((end_time - start_time) * 1000) + 'ms'
    start_time = time.time()

    # 计算相机翻滚角
    # angle_roll, err_mess = angle_rotate_roll_1(rgb_frame)
    angle_roll, err_mess = CVFunc.angle_rotate_roll(gra_edge)
    print(angle_roll)
    if len(err_mess) > 0:
        print(err_mess)
        # err_mess_all += err_mess + '\n'

    # 根据翻滚角摆正照片
    rgb_rot = cv2.warpAffine(rgb_frame, cv2.getRotationMatrix2D((mid_width, mid_height), -angle_roll, 1),
                             (img_width, img_height))
    gra_edge_rot = cv2.warpAffine(gra_edge, cv2.getRotationMatrix2D((mid_width, mid_height), -angle_roll, 1),
                                  (img_width, img_height))
    ret, gra_edge_rot = cv2.threshold(gra_edge_rot, 50, 255, cv2.THRESH_BINARY)
    gra_edge_rot[0:mid_height, 0:img_width] = 0
    # cv2.imshow('edgerot', gra_edge_rot)
    # 获取时间段，搜索水平线
    end_time = time.time()
    roll_time = str((end_time - start_time) * 1000) + 'ms'
    start_time = time.time()

    # 查找最近水平线
    height_nearest, err_mess = CVFunc.nearest_horizontal_1(gra_edge_rot)
    if len(err_mess) > 0:
        # err_mess_all += err_mess + '\n'
        print(err_mess)
        # file_rec.write(str_CID + err_mess + '\n')
    else:
        cv2.line(rgb_rot, (1, int(height_nearest)), (img_width - 1, int(height_nearest)), (0, 0, 255), 1)
        print('前向距离%d像素\n' % height_nearest)
        # file_rec.write(str_CID + 'F,' + str(height_nearest) + '\n')
        # ret_mess += 'F,' + str(height_nearest) + '\n'

    # 获取时间段，搜索垂直线
    end_time = time.time()
    hor_time = str((end_time - start_time) * 1000) + 'ms'
    start_time = time.time()

    # 查找最近左右垂直线
    axis_left_near, axis_right_near, angle_left_near, angle_right_near, err_mess, ver_time_mess = nearest_vertical_2(
        rgb_rot, gra_edge_rot)

    if len(err_mess) > 0:
        # err_mess_all += err_mess + '\n'
        print(err_mess)
        # file_rec.write(str_CID + err_mess + '\n')
    else:
        if abs(angle_left_near) > 0.0:
            # 画延长线
            line_left_near = CVFunc.func_extension_line(axis_left_near[0], axis_left_near[1], axis_left_near[2],
                                                        axis_left_near[3], 'l', img_height, img_width)
            cv2.line(rgb_rot, (line_left_near[0], line_left_near[1]), (line_left_near[2], line_left_near[3]),
                     (0, 255, 0), 1)
            line_left_near_1 = CVFunc.func_extension_line(axis_left_near[4], axis_left_near[5], axis_left_near[6],
                                                        axis_left_near[7], 'l', img_height, img_width)
            cv2.line(rgb_rot, (line_left_near_1[0], line_left_near_1[1]), (line_left_near_1[2], line_left_near_1[3]),
                     (255, 0, 0), 1)
            # 计算直线方程
            a_left = (axis_left_near[1] - axis_left_near[3]) / (axis_left_near[0] - axis_left_near[2])
            b_left = axis_left_near[1] - (a_left * axis_left_near[0])
            print('左侧角度%0.4f°，公式y=%0.2fx+%0.2f' % (angle_left_near, a_left, b_left))
            # file_rec.write(str_CID + 'L,%0.4f,y=%0.2fx+%0.2f\n' % (angle_left_near, a_left, b_left))
            # ret_mess += 'L,%0.4f,y=%0.2fx+%0.2f\n' % (angle_left_near, a_left, b_left)
        else:
            print('左侧未找到垂直线')
            # file_rec.write(str_CID + 'Not found Left ')
            # err_mess_all += 'Not found Left\n'

        if abs(angle_right_near) > 0.0:
            # 画延长线
            line_right_near = CVFunc.func_extension_line(axis_right_near[0], axis_right_near[1], axis_right_near[2],
                                                         axis_right_near[3], 'r', img_height, img_width)
            cv2.line(rgb_rot, (line_right_near[0], line_right_near[1]), (line_right_near[2], line_right_near[3]),
                     (0, 255, 0), 1)
            line_right_near_1 = CVFunc.func_extension_line(axis_right_near[4], axis_right_near[5], axis_right_near[6],
                                                         axis_right_near[7], 'r', img_height, img_width)
            cv2.line(rgb_rot, (line_right_near_1[0], line_right_near_1[1]), (line_right_near_1[2], line_right_near_1[3]),
                     (255, 0, 0), 1)
            # 计算直线方程
            a_right = (axis_right_near[1] - axis_right_near[3]) / (axis_right_near[0] - axis_right_near[2])
            b_right = axis_right_near[1] - (a_right * axis_right_near[0])
            print('右侧角度%0.4f°，公式y=%0.2fx+%0.2f' % (angle_right_near, a_right, b_right))
            # file_rec.write(str_CID + 'R,%0.4f,y=%0.2fx+%0.2f\n' % (angle_right_near, a_right, b_right))
            # ret_mess += 'R,%0.4f,y=%0.2fx+%0.2f\n' % (angle_right_near, a_right, b_right)
        else:
            # err_mess_all += 'Not found right\n'
            print('右侧未找到垂直线')
            # file_rec.write(str_CID + 'Not found right\n')

    # 完成计时
    end_time = time.time()
    ver_time = str((end_time - start_time) * 1000) + 'ms'
    str_end_all = get_current_time()
    print('From ' + str_start_all)
    print('To ' + str_end_all)
    print('Init:' + init_time)
    print('Edge:' + edge_time)
    print('Roll:' + roll_time)
    print('Hor:' + hor_time)
    print('Ver:' + ver_time)
    print('in Ver:\n' + ver_time_mess)

    cv2.imshow('Fin', rgb_rot)
    cv2.waitKey(0)

    cv2.destroyAllWindows()
