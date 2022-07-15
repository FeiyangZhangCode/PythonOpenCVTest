# import queue
import cv2
import time
import datetime
import multiprocessing as mp
import os
import CVFunc
import INA219
import sys
import signal
import serial

se = serial.Serial('/dev/ttyTHS1', 9600, timeout=1)


# 图像测距
def get_distance(rgb_frame):
    ret_mess = ''
    err_mess_all = ''
    ret_value = [0.0] * 3  # 0是水平线，1是左垂线，2是右垂线

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
        ret_value[0] = -1.0
    else:
        cv2.line(rgb_rot, (1, int(height_nearest)), (img_width - 1, int(height_nearest)), (0, 0, 255), 1)
        ret_mess += 'F,' + str(height_nearest) + '\n'
        ret_value[0] = height_nearest

    # 查找最近左右垂直线
    axis_left_near, axis_right_near, angle_left_near, angle_right_near, err_mess = CVFunc.nearest_vertical_1(
        gra_edge_rot)
    if len(err_mess) > 0:
        err_mess_all += err_mess + '\n'
        ret_value[1] = -1.0
        ret_value[2] = -1.0
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
            ret_mess += 'L,%0.4f,y=%0.2fx+%0.2f\n' % (angle_left_near, a_left, b_left)
            ret_value[1] = b_left
        else:
            err_mess_all += 'Not found Left\n'
            ret_value[1] = -1.0

        if abs(angle_right_near) > 0.0:
            # 画延长线
            line_right_near = CVFunc.func_extension_line(axis_right_near[0], axis_right_near[1], axis_right_near[2],
                                                         axis_right_near[3], 'r', img_height, img_width)
            cv2.line(rgb_rot, (line_right_near[0], line_right_near[1]), (line_right_near[2], line_right_near[3]),
                     (255, 0, 0), 1)
            # 计算直线方程
            a_right = (axis_right_near[1] - axis_right_near[3]) / (axis_right_near[0] - axis_right_near[2])
            b_right = axis_right_near[1] - (a_right * axis_right_near[0])
            ret_mess += 'R,%0.4f,y=%0.2fx+%0.2f\n' % (angle_right_near, a_right, b_right)
            ret_value[2] = b_right
        else:
            err_mess_all += 'Not found right\n'
            ret_value[2] = -1.0

    return rgb_rot, ret_mess, err_mess_all, ret_value


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
def distance_get(q, cap_id, file_address):
    loop_num = 0
    file_rec = open(file_address + str(cap_id) + '.txt', 'a')
    while True:
        rgb_frame = q.get()
        loop_num += 1
        str_Time = datetime.datetime.now().strftime('%H%M%S-%f')
        # 测距
        frame_distance, ret_mess, err_mess, ret_value = get_distance(rgb_frame)
        # 保存图片
        cv2.imwrite(file_address + str(cap_id) + '-' + str_Time + '.jpg', rgb_frame)
        # 串口及屏幕输出
        print('C' + str(cap_id) + '  ' + str_Time + '  ' + str(loop_num) + '\n')
        front_value = 'F' + str(ret_value[0]) + '\r\n'
        se.write(front_value.encode())
        print(front_value)
        left_value = 'L' + str(ret_value[1]) + '\r\n'
        se.write(left_value.encode())
        print(left_value)
        right_value = 'R' + str(ret_value[2]) + '\r\n'
        se.write(right_value.encode())
        print(right_value)
        # 保存txt
        file_rec.write(str_Time + '  ' + str(loop_num) + '\n')
        if len(ret_mess) > 0:
            file_rec.write('Data:\n' + ret_mess)
            # print('Data ' + str(cap_id) + ':\n' + ret_mess)
        if len(err_mess) > 0:
            file_rec.write('Error:\n' + err_mess)
            # print('Error ' + str(cap_id) + ':\n' + err_mess)


# 开两个进程，分别读取UPS和Uart
def ups_uart_get(q, cap_id, file_address):
    loop_num = 0
    i_last = 0.0
    if cap_id == 0:
        file_rec = open(file_address + 'UPS.txt', 'a')
        str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
        ina_mess, i_value = INA219.get_ina219_data()
        i_last = i_value
        print(str_time, str(loop_num), ina_mess)
        file_rec.write(str_time + '   ' + str(loop_num) + '\n' + ina_mess)
    elif cap_id == 1:
        quit_all()

    while True:
        time.sleep(1)
        loop_num += 1
        if loop_num % 20 == 0 and cap_id == 0:
            str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
            ina_mess, i_value = INA219.get_ina219_data()
            print(str_time, ina_mess)
            file_rec.write(str_time + '   ' + '\n' + ina_mess)
            if (i_last - i_value) > 0.3:
                os.system('shutdown now')
                # print('shutdown', i_last)
                # i_last = i_value
            else:
                i_last = i_value


def quit_all():
    print('Exit ALL')
    os.kill(os.getpid(), signal.SIGTERM)
    # try:
    #     sys.exit()
    # except Exception as e:
    #     print(e)


# 解决进程问题
def run_multi_camera():
    # 新建文件夹,读取时间作为文件名
    str_fileAddress = './TestData/'
    str_Time = datetime.datetime.now().strftime('%Y%m%d-%H%M')
    # file_rec = open(str_fileHome + str_Time + '.txt', 'w', encoding='utf-8')
    str_fileAddress += str_Time
    if not os.path.exists(str_fileAddress):
        os.makedirs(str_fileAddress)
    str_fileAddress += '/'

    # 摄像头编号
    cap_list, cap_num = get_camera_id()
    if cap_num == 1:
        camera_id_l = [0, ]
    elif cap_num == 2:
        camera_id_l = [0, 1, ]
    else:
        quit()

    mp.set_start_method(method='spawn')  # init
    queues = [mp.Queue(maxsize=3) for _ in camera_id_l]

    processes = []
    for queue, camera_id in zip(queues, camera_id_l):
        processes.append(mp.Process(target=image_put, args=(queue, camera_id, str_fileAddress)))
        # processes.append(mp.Process(target=video_get, args=(queue, camera_id)))
        # processes.append(mp.Process(target=picture_get, args=(queue, camera_id, str_fileAddress)))
        processes.append(mp.Process(target=distance_get, args=(queue, camera_id, str_fileAddress)))
        processes.append(mp.Process(target=ups_uart_get, args=(queue, camera_id, str_fileAddress)))

    for process in processes:
        process.daemon = True
        process.start()
    for process in processes:
        process.join()


if __name__ == '__main__':
    run_multi_camera()  # 调用主函数
