import cv2
import datetime
import CVFunc
# import INA219
import os


# 确定摄像头数量和编号，提取从0-6
def get_camera_id():
    list_ID = [int] * 7
    int_num = 0
    for k in range(0, 7, 1):
        cap_test = cv2.VideoCapture(k, cv2.CAP_DSHOW)
        if cap_test.isOpened():
            list_ID[int_num] = k
            int_num += 1
            cap_test.release()
    return list_ID, int_num


# 返回摄像头格式
def get_camera_data(cap, num):
    str_Info = 'C' + str(num) + ':' + str(cap.get(3)) + '*' + str(cap.get(4)) + '; FPS' + str(cap.get(5)) + '\n'
    str_fourcc = str("".join([chr((int(cap.get(cv2.CAP_PROP_FOURCC)) >> 8 * k) & 0xFF) for k in range(4)])) + '\n'
    str_Info = str_Info + str_fourcc
    return str_Info


# 图像测距
def get_distance(rgb_frame, str_CID):
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
    angle_roll, err_mess = CVFunc.angle_rotate_roll(rgb_frame)
    if len(err_mess) > 0:
        # print(err_mess)
        err_mess_all += err_mess + '\n'

    # 根据翻滚角摆正照片
    rgb_rot = cv2.warpAffine(rgb_frame, cv2.getRotationMatrix2D((mid_width, mid_height), -angle_roll, 1),
                             (img_width, img_height))
    gra_edge_rot = cv2.warpAffine(gra_edge, cv2.getRotationMatrix2D((mid_width, mid_height), -angle_roll, 1),
                                  (img_width, img_height))

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
    axis_left_near, axis_right_near, angle_left_near, angle_right_near, err_mess = CVFunc.nearest_vertical_1(
        gra_edge_rot)
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
            ret_mess += 'L,%0.4f,y=%0.2fx+%0.2f\n' % (angle_left_near, a_left, b_left)
        else:
            # print('左侧未找到垂直线')
            # file_rec.write(str_CID + 'Not found Left ')
            err_mess_all += 'Not found Left\n'

        if abs(angle_right_near) > 0.0:
            # 画延长线
            line_right_near = CVFunc.func_extension_line(axis_right_near[0], axis_right_near[1], axis_right_near[2],
                                                         axis_right_near[3], 'r', img_height, img_width)
            cv2.line(rgb_rot, (line_right_near[0], line_right_near[1]), (line_right_near[2], line_right_near[3]),
                     (255, 0, 0), 1)
            # 计算直线方程
            a_right = (axis_right_near[1] - axis_right_near[3]) / (axis_right_near[0] - axis_right_near[2])
            b_right = axis_right_near[1] - (a_right * axis_right_near[0])
            # print('右侧角度%0.4f°，公式y=%0.2fx+%0.2f' % (angle_right_near, a_right, b_right))
            # file_rec.write(str_CID + 'R,%0.4f,y=%0.2fx+%0.2f\n' % (angle_right_near, a_right, b_right))
            ret_mess += 'R,%0.4f,y=%0.2fx+%0.2f\n' % (angle_right_near, a_right, b_right)
        else:
            err_mess_all += 'Not found right\n'
            # print('右侧未找到垂直线')
            # file_rec.write(str_CID + 'Not found right\n')

    return rgb_rot, ret_mess, err_mess_all


# 开始主程序
# 新建文件夹,读取时间作为文件名
str_fileAddress = './TestData/'
str_fileHome = str_fileAddress
str_Time = datetime.datetime.now().strftime('%Y%m%d-%H%M')
file_rec = open(str_fileHome + str_Time + '.txt', 'w', encoding='utf-8')
str_fileAddress = str_fileAddress + str_Time
if not os.path.exists(str_fileAddress):
    os.makedirs(str_fileAddress)
str_fileAddress = str_fileAddress + '/'

# 第一个摄像头0
cap0 = cv2.VideoCapture(0)
cap0.set(6, 1196444237)
cap0.set(3, 1920)
cap0.set(4, 1080)
cap0.set(5, 30)
fps0 = cap0.get(cv2.CAP_PROP_FPS)
size0 = (int(cap0.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap0.get(cv2.CAP_PROP_FRAME_HEIGHT)))
file_rec.write(get_camera_data(cap0, 0))
print(get_camera_data(cap0, 0))

# 设置循环计数器，开始循环采集数据
loop_num = 0
while True:
    loop_num = loop_num + 1
    # str_Time = datetime.datetime.now().strftime('%H%M%S-%f')
    str_Time = datetime.datetime.now().strftime('%H%M%S')

    ret0, frame0 = cap0.read()
    # 存储原图
    # cv2.imwrite(str_fileAddress + str_Time + "-C0" + '.jpg', frame0)
    # file_rec.write(str_Time + ',' + str(loop_num) + ',C0' + '\n')
    # 运行边界识别子程序
    frame0_distance, ret0_mess, err0_mess = get_distance(frame0, 'C0:')
    if len(ret0_mess) > 0:
        file_rec.write('Get Data:\n' + ret0_mess)
        print('Get Data:\n' + ret0_mess)

    if len(err0_mess) > 0:
        file_rec.write('Error Message:\n' + err0_mess)
        # print('Error:\n' + err0_mess)
    # 存储边界图
    # cv2.imwrite(str_fileAddress + str_Time + "-C0D" + '.jpg', frame0_distance)

    cv2.namedWindow('Camera-0', cv2.WINDOW_NORMAL)
    cv2.imshow('Camera-0', frame0_distance)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

file_rec.close()
cap0.release()
cv2.destroyAllWindows()
