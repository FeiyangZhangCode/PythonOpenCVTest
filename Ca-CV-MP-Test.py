# import queue
import cv2
import time
import datetime
import multiprocessing as mp
import os
import CVFunc

# import INA219
import sys
import signal
# import serial

# se = serial.Serial('/dev/ttyTHS1', 9600, timeout=1)

# 读取模型参数
file_model = open('Model.txt', 'r', encoding='utf-8')
para_lines = file_model.readlines()
file_model.close()


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

    ret_mess = ''       # 数据信息
    err_mess_all = ''   # 报错信息
    time_mess = ''      # 时间信息
    ret_value = [0.0] * 3  # 0是水平线，1是左垂线，2是右垂线

    # 获取图像位置参数
    img_height = int(rgb_frame.shape[0])
    img_width = int(rgb_frame.shape[1])
    mid_height = int(img_height / 2)
    mid_width = int(img_width / 2)

    # Canny提取边界
    start_time = time.time()
    gra_edge = CVFunc.find_edge_light(rgb_frame)
    end_time = time.time()
    time_mess += 'Can:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

    # 计算相机翻滚角，根据翻滚角摆正照片
    start_time = time.time()
    angle_roll, err_mess = CVFunc.angle_rotate_roll(rgb_frame)
    if len(err_mess) > 0:
        err_mess_all += err_mess + '\n'
    rgb_rot = cv2.warpAffine(rgb_frame, cv2.getRotationMatrix2D((mid_width, mid_height), -angle_roll, 1),
                             (img_width, img_height))
    gra_edge_rot = cv2.warpAffine(gra_edge, cv2.getRotationMatrix2D((mid_width, mid_height), -angle_roll, 1),
                                  (img_width, img_height))
    gra_edge_rot[0:principal_y, :] = 0
    end_time = time.time()
    time_mess += 'Rot:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

    # 获取水平和垂直线
    start_time = time.time()
    hor_lines_points, ver_lines, right_formular, left_formular = CVFunc.get_HoughLinesP(gra_edge_rot)
    end_time = time.time()
    time_mess += 'Hou:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

    # 画图像中心十字
    cv2.line(rgb_rot, (mid_width, 0), (mid_width, img_height), (255, 0, 255), 1)
    cv2.line(rgb_rot, (0, mid_height), (img_width, mid_height), (255, 0, 255), 1)
    # 画相机中心十字
    cv2.line(rgb_rot, (principal_x, 0), (principal_x, img_height), (255, 255, 0), 1)
    cv2.line(rgb_rot, (0, principal_y), (img_width, principal_y), (255, 255, 0), 1)
    # 画标定垂线位置
    # a_id_l = left_formular[0]
    # b_id_l = left_formular[1]
    # a_id_r = right_formular[0]
    # b_id_r = right_formular[1]
    # cv2.line(rgb_rot, (int(a_id_l * mid_height + b_id_l), mid_height), (int(a_id_l * img_height + b_id_l), img_height),
    #          (0, 255, 255), 1)
    # cv2.line(rgb_rot, (int(a_id_r * mid_height + b_id_r), mid_height), (int(a_id_r * img_height + b_id_r), img_height),
    #          (0, 255, 255), 1)

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
    time_mess += 'Hor:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

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
                if 0 < min(dis_ver_1, dis_ver_2) < dis_temp_l:
                    dis_temp_l = min(dis_ver_1, dis_ver_2)
            else:
                if 0 < min(dis_ver_1, dis_ver_2) < dis_temp_r:
                    dis_temp_r = min(dis_ver_1, dis_ver_2)
    ret_value[1] = dis_temp_l
    ret_value[2] = dis_temp_r
    ret_mess += 'L & R,' + str(dis_temp_l) + ',' + str(dis_temp_r) + '\n'
    end_time = time.time()
    time_mess += 'Ver:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

    # # 查找最近水平线
    # height_nearest, err_mess = CVFunc.nearest_horizontal_1(gra_edge_rot)
    # if len(err_mess) > 0:
    #     err_mess_all += err_mess + '\n'
    #     ret_value[0] = -1.0
    # else:
    #     cv2.line(rgb_rot, (1, int(height_nearest)), (img_width - 1, int(height_nearest)), (0, 0, 255), 1)
    #     dis_hor = CVFunc.calc_horizontal(int(height_nearest), para_camera[c_id_para + 0], para_camera[c_id_para + 1],
    #                                      para_camera[c_id_para + 2], para_camera[c_id_para + 3])
    #     ret_mess += 'F,' + str(dis_hor) + '\n'
    #     ret_value[0] = dis_hor
    #
    # # 查找最近左右垂直线
    # axis_left_near, axis_right_near, angle_left_near, angle_right_near, err_mess = CVFunc.nearest_vertical_1(
    #     gra_edge_rot)
    # if len(err_mess) > 0:
    #     err_mess_all += err_mess + '\n'
    #     ret_value[1] = -1.0
    #     ret_value[2] = -1.0
    # else:
    #     if abs(angle_left_near) > 0.0:
    #         dis_l_1 = CVFunc.calc_vertical(abs(axis_left_near[0] - para_camera[c_id_para + 4]), axis_left_near[1],
    #                                        para_camera[c_id_para + 0], para_camera[c_id_para + 1],
    #                                        para_camera[c_id_para + 2], para_camera[c_id_para + 3])
    #         dis_l_2 = CVFunc.calc_vertical(abs(axis_left_near[2] - para_camera[c_id_para + 4]), axis_left_near[3],
    #                                        para_camera[c_id_para + 0], para_camera[c_id_para + 1],
    #                                        para_camera[c_id_para + 2], para_camera[c_id_para + 3])
    #         dis_l = round(((dis_l_1 + dis_l_2) / 2), 0)
    #         ret_mess += 'L,' + str(dis_l) + '\n'
    #         ret_value[1] = dis_l
    #         # 画延长线
    #         line_left_near = CVFunc.func_extension_line(axis_left_near[0], axis_left_near[1], axis_left_near[2],
    #                                                     axis_left_near[3], 'l', img_height, img_width)
    #         cv2.line(rgb_rot, (line_left_near[0], line_left_near[1]), (line_left_near[2], line_left_near[3]),
    #                  (0, 255, 0), 1)
    #         cv2.putText(rgb_rot, str(round(dis_l_1, 0)), (axis_left_near[0], axis_left_near[1]),
    #                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
    #         cv2.putText(rgb_rot, str(round(dis_l_2, 0)), (axis_left_near[2], axis_left_near[3]),
    #                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
    #     else:
    #         err_mess_all += 'Not found Left\n'
    #         ret_value[1] = -1.0
    #
    #     if abs(angle_right_near) > 0.0:
    #         dis_r_1 = CVFunc.calc_vertical(abs(axis_right_near[0] - para_camera[c_id_para + 4]), axis_right_near[1],
    #                                        para_camera[c_id_para + 0], para_camera[c_id_para + 1],
    #                                        para_camera[c_id_para + 2], para_camera[c_id_para + 3])
    #         dis_r_2 = CVFunc.calc_vertical(abs(axis_right_near[2] - para_camera[c_id_para + 4]), axis_right_near[1],
    #                                        para_camera[c_id_para + 0], para_camera[c_id_para + 1],
    #                                        para_camera[c_id_para + 2], para_camera[c_id_para + 3])
    #         dis_r = round(((dis_r_1 + dis_r_2) / 2), 0)
    #         ret_mess += 'R,' + str(dis_r) + '\n'
    #         ret_value[2] = dis_r
    #         # 画延长线
    #         line_right_near = CVFunc.func_extension_line(axis_right_near[0], axis_right_near[1], axis_right_near[2],
    #                                                      axis_right_near[3], 'r', img_height, img_width)
    #         cv2.line(rgb_rot, (line_right_near[0], line_right_near[1]), (line_right_near[2], line_right_near[3]),
    #                  (255, 0, 0), 1)
    #         cv2.putText(rgb_rot, str(round(dis_r_1, 0)), (axis_right_near[0], axis_right_near[1]),
    #                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)
    #         cv2.putText(rgb_rot, str(round(dis_r_2, 0)), (axis_right_near[2], axis_right_near[3]),
    #                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)
    #     else:
    #         err_mess_all += 'Not found right\n'
    #         ret_value[2] = -1.0

    return rgb_rot, ret_mess, err_mess_all, time_mess, ret_value


# 确定摄像头数量和编号，提取从0-6
def get_camera_id():
    list_ID = [int] * 7
    int_num = 0
    for k in range(0, 7, 1):
        cap_test = cv2.VideoCapture(k)
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


# 抓取图片，确认视频流的读入
def image_put(q, c_id, file_address):
    cap = cv2.VideoCapture(c_id)
    cap.set(6, 1196444237)
    cap.set(3, 1920)
    cap.set(4, 1080)
    cap.set(5, 30)
    # 获取视频帧率
    # fps = cap.get(cv2.CAP_PROP_FPS)
    # size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    print(get_camera_data(cap, c_id))
    if cap.isOpened():
        print('Get1', c_id)
    else:
        cap = cv2.VideoCapture(c_id)
        cap.set(6, 1196444237)
        cap.set(3, 1920)
        cap.set(4, 1080)
        cap.set(5, 30)
        # fps = cap.get(cv2.CAP_PROP_FPS)
        print(get_camera_data(cap, c_id))
        print('Get2', c_id)

    while cap.isOpened():
        # print('cap.read()[0]:', cap.read()[0])
        ret, frame = cap.read()
        # print('ret:', ret)
        # frame = cv2.resize(frame, (1920, 1080))
        # 抓取图片不成功再重新抓取
        if not ret:
            cap = cv2.VideoCapture(c_id)
            print('Get3', c_id)
            ret, frame = cap.read()
            # frame = cv2.resize(frame, (1920, 1080))
        q.put(frame)
        # print('q.qsize():', q.qsize())
        q.get() if q.qsize() > 1 else time.sleep(0.01)


# 获得视频流帧数图片，保存读入的视频
def video_get(q, window_name):
    # loop_num = 0

    cv2.namedWindow(str(window_name), cv2.WINDOW_NORMAL)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # 视频保存路径
    path = "./TestData/"
    out = cv2.VideoWriter(path + str(window_name) + ".avi", fourcc, 20.0, (1920, 1080), True)

    while True:
        frame = q.get()
        out.write(frame)
        cv2.imshow(str(window_name), frame)

        # loop_num += 1
        # now_time = datetime.datetime.now()
        # str_time_doc = now_time.strftime('%H%M%S-%f')
        # if loop_num % 10 == 0:
        #     cv2.imwrite(path + str(window_name) + '-' + str_time_doc + '.jpg', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            quit_all()
            break


# 获得视频流帧数图片，保存图片
def picture_get(q, cap_id, file_address):
    loop_num = 0
    while True:
        rgb_frame = q.get()
        loop_num += 1
        now_time = datetime.datetime.now()
        str_time_doc = now_time.strftime('%H%M%S-%f')
        if loop_num % 10 == 0:
            cv2.imwrite(file_address + str(cap_id) + '-' + str_time_doc + '.jpg', rgb_frame)


# 获得视频流帧数图片，测距
def distance_get(q, lock_ser, cap_id, file_address):
    loop_num = 0
    file_rec = open(file_address + str(cap_id) + '.txt', 'a')
    while True:
        rgb_frame = q.get()
        loop_num += 1
        str_Time = datetime.datetime.now().strftime('%H%M%S-%f')
        # 测距
        frame_distance, ret_mess, err_mess, time_mess, ret_value = dis_calc(rgb_frame, cap_id)
        # 显示及保存图片
        cv2.imshow('Cap', frame_distance)
        cv2.imwrite(file_address + 'C' + str(cap_id) + '-' + str_Time + '.jpg', rgb_frame)
        cv2.imwrite(file_address + 'D' + str(cap_id) + '-' + str_Time + '.jpg', frame_distance)
        # 屏幕输出
        print('C' + str(cap_id) + '  ' + str_Time + '  ' + str(loop_num))
        front_value = 'F' + str(ret_value[0])
        print(front_value)
        left_value = 'L' + str(ret_value[1])
        print(left_value)
        right_value = 'R' + str(ret_value[2])
        print(right_value)
        # 串口输出（自锁开始）
        lock_ser.acquire()
        # se.write(front_value.encode())
        # se.write(left_value.encode())
        # se.write(right_value.encode())

        # 保存txt（自锁结束）
        file_rec.write(str_Time + '  ' + str(loop_num) + '\n')
        if len(ret_mess) > 0:
            file_rec.write('Data:\n' + ret_mess)
            # print('Data ' + str(cap_id) + ':\n' + ret_mess)
        if len(err_mess) > 0:
            file_rec.write('Error:\n' + err_mess)
            # print('Error ' + str(cap_id) + ':\n' + err_mess)
        if len(time_mess) > 0:
            file_rec.write('Timer:\n' + time_mess)
        lock_ser.release()


# 开两个进程，分别读取UPS和Uart
# def ups_uart_get(q, cap_id, file_address):
#     loop_num = 0
#     i_last = 0.0
#     if cap_id == 0:
#         file_rec = open(file_address + 'UPS.txt', 'a')
#         str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
#         ina_mess, i_value = INA219.get_ina219_data()
#         i_last = i_value
#         print(str_time, str(loop_num), ina_mess)
#         file_rec.write(str_time + '   ' + str(loop_num) + '\n' + ina_mess)
#     elif cap_id == 1:
#         quit_all()
#
#     while True:
#         time.sleep(1)
#         loop_num += 1
#         if loop_num % 20 == 0 and cap_id == 0:
#             str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
#             ina_mess, i_value = INA219.get_ina219_data()
#             print(str_time, ina_mess)
#             file_rec.write(str_time + '   ' + '\n' + ina_mess)
#             if (i_last - i_value) > 0.3:
#                 os.system('shutdown now')
#                 # print('shutdown', i_last)
#                 # i_last = i_value
#             else:
#                 i_last = i_value
#

def quit_all():
    print('Exit ALL')
    os.kill(os.getpid(), signal.SIGTERM)
    # try:
    #     sys.exit()
    # except Exception as e:
    #     print(e)


# 解决进程问题
def run_multi_camera():
    # 摄像头编号
    cap_list, cap_num = get_camera_id()
    if cap_num == 1:
        camera_id_l = [0, ]
    elif cap_num == 2:
        camera_id_l = [0, 1, ]
    else:
        quit()

    # 新建文件夹,读取时间作为文件名
    str_fileAddress = './TestData/'
    str_Time = datetime.datetime.now().strftime('%Y%m%d-%H%M')
    # file_rec = open(str_fileHome + str_Time + '.txt', 'w', encoding='utf-8')
    str_fileAddress += str_Time
    if not os.path.exists(str_fileAddress):
        os.makedirs(str_fileAddress)
    str_fileAddress += '/'

    mp.set_start_method(method='spawn')  # init
    queues = [mp.Queue(maxsize=2) for _ in camera_id_l]
    lock = mp.Lock()

    processes = []
    for queue, camera_id in zip(queues, camera_id_l):
        processes.append(mp.Process(target=image_put, args=(queue, camera_id, str_fileAddress)))
        # processes.append(mp.Process(target=video_get, args=(queue, camera_id)))
        # processes.append(mp.Process(target=picture_get, args=(queue, camera_id, str_fileAddress)))
        processes.append(mp.Process(target=distance_get, args=(queue, lock, camera_id, str_fileAddress)))
        # processes.append(mp.Process(target=ups_uart_get, args=(queue, camera_id, str_fileAddress)))

    for process in processes:
        process.daemon = True
        process.start()
    for process in processes:
        process.join()


if __name__ == '__main__':
    run_multi_camera()  # 调用主函数
