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
    global model_F, model_W, model_a, model_b, principal_x, principal_y

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
    cv2.imshow('gra', gra_edge_rot)
    # 查找最近水平线
    height_nearest, err_mess = CVFunc.nearest_horizontal_1(gra_edge_rot)
    if len(err_mess) > 0:
        err_mess_all += err_mess + '\n'
        # print(err_mess)
        # file_rec.write(str_CID + err_mess + '\n')
    else:
        cv2.line(rgb_rot, (1, int(height_nearest)), (img_width - 1, int(height_nearest)), (0, 0, 255), 1)
        dis_hor = CVFunc.calc_horizontal(int(height_nearest), model_F, model_W, model_a, model_b)
        cv2.putText(rgb_rot, str(dis_hor) + 'mm', (mid_width, int(height_nearest)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 0, 255), 1)
        # print('前向距离%d像素\n' % height_nearest)
        # file_rec.write(str_CID + 'F,' + str(height_nearest) + '\n')
        ret_mess += 'F,' + str(dis_hor) + '\n'

    # 查找最近左右垂直线
    axis_left_near, axis_right_near, angle_left_near, angle_right_near, err_mess, ver_time_mess = CVFunc.nearest_vertical_2(
        gra_edge_rot)
    if len(err_mess) > 0:
        err_mess_all += err_mess + '\n'
        # print(err_mess)
        # file_rec.write(str_CID + err_mess + '\n')
    else:
        if abs(angle_left_near) > 0.0:
            # 计算距离并画线
            dis_l_1 = CVFunc.calc_vertical(abs(axis_left_near[0] - principal_x), axis_left_near[1], model_F, model_W,
                                           model_a, model_b)
            dis_l_2 = CVFunc.calc_vertical(abs(axis_left_near[2] - principal_x), axis_left_near[3], model_F, model_W,
                                           model_a, model_b)
            cv2.line(rgb_rot, (axis_left_near[0], axis_left_near[1]), (axis_left_near[2], axis_left_near[3]),
                     (0, 255, 0), 1)
            cv2.putText(rgb_rot, str(round(dis_l_1, 0)), (axis_left_near[0], axis_left_near[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0), 1)
            cv2.putText(rgb_rot, str(round(dis_l_2, 0)), (axis_left_near[2], axis_left_near[3]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0), 1)
            ret_mess += 'L1,' + str(round(dis_l_1, 0)) + ',' + str(round(dis_l_1, 0)) + '\n'
        else:
            # print('左侧未找到垂直线')
            # file_rec.write(str_CID + 'Not found Left ')
            err_mess_all += 'Not found Left 1\n'

        if abs(angle_right_near) > 0.0:
            # 计算距离并画线
            dis_r_1 = CVFunc.calc_vertical(abs(axis_right_near[0] - principal_x), axis_right_near[1], model_F, model_W,
                                           model_a, model_b)
            dis_r_2 = CVFunc.calc_vertical(abs(axis_right_near[2] - principal_x), axis_right_near[3], model_F, model_W,
                                           model_a, model_b)
            cv2.line(rgb_rot, (axis_right_near[0], axis_right_near[1]), (axis_right_near[2], axis_right_near[3]),
                     (255, 0, 0), 1)
            cv2.putText(rgb_rot, str(round(dis_r_1, 0)), (axis_right_near[0], axis_right_near[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 0, 0), 1)
            cv2.putText(rgb_rot, str(round(dis_r_2, 0)), (axis_right_near[2], axis_right_near[3]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 0, 0), 1)
            ret_mess += 'R1,' + str(round(dis_r_1, 0)) + ',' + str(round(dis_r_2, 0)) + '\n'
        else:
            err_mess_all += 'Not found right 1\n'
            # print('右侧未找到垂直线')
            # file_rec.write(str_CID + 'Not found right\n')

    return rgb_rot, ret_mess, err_mess_all


# 开始主程序
# 选择摄像头
cap_id = int(input('相机编号(0/1)'))
while (cap_id != 0) and (cap_id != 1):
    cap_id = int(input('相机编号(0/1)'))

# 读取模型参数
file_model = open('Model.txt', 'r', encoding='utf-8')
para_lines = file_model.readlines()
if cap_id == 0:
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
cap0 = cv2.VideoCapture(cap_id)
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
    cv2.imwrite(str_fileAddress + 'C' + str_Time + '.jpg', frame0)
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
    cv2.imwrite(str_fileAddress + 'D' + str_Time + '.jpg', frame0_distance)

    cv2.namedWindow('Camera-0', cv2.WINDOW_NORMAL)
    cv2.imshow('Camera-0', frame0_distance)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

file_rec.close()
cap0.release()
cv2.destroyAllWindows()
