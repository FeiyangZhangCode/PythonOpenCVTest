import os
import time
import datetime
import shutil
import cv2
import CVFunc


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


# 图像测距
def get_distance(rgb_frame):
    ret_mess = ''
    err_mess_all = ''

    # 获取图像位置参数
    img_height = int(rgb_frame.shape[0])
    img_width = int(rgb_frame.shape[1])
    mid_height = int(img_height / 2)
    mid_width = int(img_width / 2)

    # Canny提取边界
    gra_edge = CVFunc.find_edge(rgb_frame)

    # 计算相机翻滚角
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
    gra_edge_rot[0:mid_height, 0:img_width] = 0
    # 查找最近水平线
    height_nearest, err_mess = CVFunc.nearest_horizontal_1(gra_edge_rot)
    if len(err_mess) > 0:
        err_mess_all += err_mess + '\n'
        # print(err_mess)
        # file_rec.write(str_CID + err_mess + '\n')
    else:
        cv2.line(rgb_rot, (1, int(height_nearest)), (img_width - 1, int(height_nearest)), (0, 0, 255), 1)
        # print('前向距离%d像素\n' % height_nearest)
        # file_rec.write(str_CID + 'F,' + str(height_nearest) + '\n')
        ret_mess += 'F,' + str(height_nearest) + '\n'

    # 查找最近左右垂直线
    axis_left_near, axis_right_near, angle_left_near, angle_right_near, err_mess, ver_time_mess = CVFunc.nearest_vertical_2(
        gra_edge_rot)
    # axis_left_near, axis_right_near, angle_left_near, angle_right_near, err_mess = CVFunc.nearest_vertical_1(
    #     gra_edge_rot)
    if len(err_mess) > 0:
        err_mess_all += err_mess + '\n'
        # print(err_mess)
        # file_rec.write(str_CID + err_mess + '\n')
    else:
        if abs(angle_left_near) > 0.0:
            # 画延长线
            line_left_near = CVFunc.func_extension_line(axis_left_near[0], axis_left_near[1], axis_left_near[2],
                                                        axis_left_near[3], 'l', img_height, img_width)
            cv2.line(rgb_rot, (line_left_near[0], line_left_near[1]), (line_left_near[2], line_left_near[3]),
                     (0, 255, 0), 1)

            # 计算直线方程
            a_left = (axis_left_near[1] - axis_left_near[3]) / (axis_left_near[0] - axis_left_near[2])
            b_left = axis_left_near[1] - (a_left * axis_left_near[0])
            # print('左侧角度%0.4f°，公式y=%0.2fx+%0.2f' % (angle_left_near, a_left, b_left))
            # file_rec.write(str_CID + 'L,%0.4f,y=%0.2fx+%0.2f\n' % (angle_left_near, a_left, b_left))
            ret_mess += 'L 1 ,%0.4f,y=%0.2fx+%0.2f\n' % (angle_left_near, a_left, b_left)
        else:
            # print('左侧未找到垂直线')
            # file_rec.write(str_CID + 'Not found Left ')
            err_mess_all += 'Not found Left 1\n'

        if (axis_left_near[4] - axis_left_near[6]) != 0:
            # 画延长线
            line_left_near_1 = CVFunc.func_extension_line(axis_left_near[4], axis_left_near[5], axis_left_near[6],
                                                          axis_left_near[7], 'l', img_height, img_width)
            cv2.line(rgb_rot, (line_left_near_1[0], line_left_near_1[1]), (line_left_near_1[2], line_left_near_1[3]),
                     (255, 0, 0), 1)
            # 计算直线方程
            a_left_1 = (axis_left_near[5] - axis_left_near[7]) / (axis_left_near[4] - axis_left_near[6])
            b_left_1 = axis_left_near[5] - (a_left_1 * axis_left_near[4])
            # print('左侧角度%0.4f°，公式y=%0.2fx+%0.2f' % (angle_left_near, a_left, b_left))
            # file_rec.write(str_CID + 'L,%0.4f,y=%0.2fx+%0.2f\n' % (angle_left_near, a_left, b_left))
            ret_mess += 'L 2 ,y=%0.2fx+%0.2f\n' % (a_left_1, b_left_1)
        else:
            # print('左侧未找到垂直线')
            # file_rec.write(str_CID + 'Not found Left ')
            err_mess_all += 'Not found Left 2\n'

        if abs(angle_right_near) > 0.0:
            # 画延长线
            line_right_near = CVFunc.func_extension_line(axis_right_near[0], axis_right_near[1], axis_right_near[2],
                                                         axis_right_near[3], 'r', img_height, img_width)
            cv2.line(rgb_rot, (line_right_near[0], line_right_near[1]), (line_right_near[2], line_right_near[3]),
                     (0, 255, 0), 1)
            # 计算直线方程
            a_right = (axis_right_near[1] - axis_right_near[3]) / (axis_right_near[0] - axis_right_near[2])
            b_right = axis_right_near[1] - (a_right * axis_right_near[0])
            # print('右侧角度%0.4f°，公式y=%0.2fx+%0.2f' % (angle_right_near, a_right, b_right))
            # file_rec.write(str_CID + 'R,%0.4f,y=%0.2fx+%0.2f\n' % (angle_right_near, a_right, b_right))
            ret_mess += 'R 1 ,%0.4f,y=%0.2fx+%0.2f\n' % (angle_right_near, a_right, b_right)
        else:
            err_mess_all += 'Not found right 1\n'
            # print('右侧未找到垂直线')
            # file_rec.write(str_CID + 'Not found right\n')

        if (axis_right_near[4] - axis_right_near[6]) != 0:
            # 画延长线
            line_right_near_1 = CVFunc.func_extension_line(axis_right_near[4], axis_right_near[5], axis_right_near[6],
                                                           axis_right_near[7], 'r', img_height, img_width)
            cv2.line(rgb_rot, (line_right_near_1[0], line_right_near_1[1]),
                     (line_right_near_1[2], line_right_near_1[3]), (255, 0, 0), 1)
            # 计算直线方程
            a_right_2 = (axis_right_near[1] - axis_right_near[3]) / (axis_right_near[0] - axis_right_near[2])
            b_right_2 = axis_right_near[1] - (a_right_2 * axis_right_near[0])
            # print('右侧角度%0.4f°，公式y=%0.2fx+%0.2f' % (angle_right_near, a_right, b_right))
            # file_rec.write(str_CID + 'R,%0.4f,y=%0.2fx+%0.2f\n' % (angle_right_near, a_right, b_right))
            ret_mess += 'R 2 ,y=%0.2fx+%0.2f\n' % (a_right_2, b_right_2)
        else:
            err_mess_all += 'Not found right 2\n'
            # print('右侧未找到垂直线')
            # file_rec.write(str_CID + 'Not found right\n')

    return rgb_rot, ret_mess, err_mess_all


# 开始主程序
# 新建文件夹,读取时间作为文件名
str_fileAddress = './TestData-1/'
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
    loop_num = loop_num + 1
    str_Time = datetime.datetime.now().strftime('%H%M%S')
    file_rec.write(str_Time + ',' + str(loop_num) + ',' + title + '\n')
    print(str_Time + ',' + str(loop_num) + ',' + title)
    frame0 = cv2.imread(str_fileHome + title)
    frame0_distance, ret0_mess, err0_mess = get_distance(frame0)
    if len(ret0_mess) > 0:
        file_rec.write('Get Data:\n' + ret0_mess)
        print('Get Data:\n' + ret0_mess)

    if len(err0_mess) > 0:
        file_rec.write('Error Message:\n' + err0_mess)
        print('Error:\n' + err0_mess)
    # print(title)
    cv2.imshow('Dis', frame0_distance)
    cv2.imwrite(str_fileAddress + title, frame0_distance)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
