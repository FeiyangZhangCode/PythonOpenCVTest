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

se = serial.Serial('/dev/ttyTHS1', 9600, timeout=0.2)

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
    err_mess = ''       # 报错信息
    time_mess = ''      # 时间信息
    ret_value = [0.0] * 3  # 0是水平线，1是左垂线，2是右垂线

    # 获取图像位置参数
    img_height = int(rgb_frame.shape[0])
    img_width = int(rgb_frame.shape[1])
    mid_height = int(img_height / 2)
    mid_width = int(img_width / 2)

    # Canny提取边界，保留下半部分
    start_time = time.time()
    gra_edge = CVFunc.find_edge_light(rgb_frame)
    rgb_rot = rgb_frame.copy()
    gra_edge_rot = gra_edge.copy()
    gra_edge_rot[0:principal_y, :] = 0
    end_time = time.time()
    time_mess += 'Can:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

    # 获取水平和垂直线
    start_time = time.time()
    hor_lines_points, ver_lines, err_mess_hough = CVFunc.get_HoughLinesP(gra_edge_rot)
    err_mess += err_mess_hough
    end_time = time.time()
    time_mess += 'Hou:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

    # 画相机中心十字
    cv2.line(rgb_rot, (principal_x, 0), (principal_x, img_height), (255, 255, 0), 1)
    cv2.line(rgb_rot, (0, principal_y), (img_width, principal_y), (255, 255, 0), 1)
    cv2.circle(rgb_rot, (principal_x, principal_y), 5, (255, 255, 0), 3)

    # 计算水平线到相机的距离，输出最近距离，并画出水平线
    start_time = time.time()
    dis_temp = 9999.99
    height_edge = - model_b / model_a
    axis_temp_h = 0
    try:
        for hor_point in hor_lines_points:
            if float(hor_point[0]) > height_edge:
                dis_hor = CVFunc.calc_horizontal(hor_point[0], model_F, model_W, model_a, model_b)
                dis_hor = round(dis_hor, 2)
                if dis_hor < dis_temp:
                    dis_temp = dis_hor
                    axis_temp_h = hor_point[0]
        cv2.line(rgb_rot, (0, axis_temp_h), (img_width, axis_temp_h), (255, 0, 0), 1)
        cv2.putText(rgb_rot, str(dis_temp), (mid_width, axis_temp_h), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 0, 0), 1)
    except Exception as e:
        err_mess += 'calc horizontal line\n' + str(e) + '\n'

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
    try:
        for ver_line in ver_lines:
            for x1, y1, x2, y2 in ver_line:
                # cv2.line(rgb_rot, (x1, y1), (x2, y2), (0, 255, 0), 1)
                if float(y1) > height_edge and float(y2) > height_edge:
                    dis_ver_1 = CVFunc.calc_vertical(abs(x1 - principal_x), y1, model_F, model_W, model_a, model_b)
                    dis_ver_2 = CVFunc.calc_vertical(abs(x2 - principal_x), y2, model_F, model_W, model_a, model_b)

                    if x2 - principal_x < 0:
                        if 0 < min(dis_ver_1, dis_ver_2) < dis_temp_l:
                            dis_temp_l = min(dis_ver_1, dis_ver_2)
                            axis_temp_l[0] = x1
                            axis_temp_l[1] = y1
                            axis_temp_l[2] = x2
                            axis_temp_l[3] = y2
                    else:
                        if 0 < min(dis_ver_1, dis_ver_2) < dis_temp_r:
                            dis_temp_r = min(dis_ver_1, dis_ver_2)
                            axis_temp_r[0] = x1
                            axis_temp_r[1] = y1
                            axis_temp_r[2] = x2
                            axis_temp_r[3] = y2

        # 画出左端最近垂直线
        x1 = axis_temp_l[0]
        y1 = axis_temp_l[1]
        x2 = axis_temp_l[2]
        y2 = axis_temp_l[3]
        axis_ex = CVFunc.extension_line_half(x1, y1, x2, y2, img_height, principal_y)
        cv2.line(rgb_rot, (axis_ex[0], axis_ex[1]), (axis_ex[2], axis_ex[3]), (0, 255, 0), 1)
        dis_ver_1 = CVFunc.calc_vertical(abs(x1 - principal_x), y1, model_F, model_W, model_a, model_b)
        dis_ver_2 = CVFunc.calc_vertical(abs(x2 - principal_x), y2, model_F, model_W, model_a, model_b)
        cv2.putText(rgb_rot, str(round(dis_ver_1, 0)), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0), 1)
        cv2.putText(rgb_rot, str(round(dis_ver_2, 0)), (x2, y2), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0), 1)

        # 画出右端最近垂直线
        x1 = axis_temp_r[0]
        y1 = axis_temp_r[1]
        x2 = axis_temp_r[2]
        y2 = axis_temp_r[3]
        axis_ex = CVFunc.extension_line_half(x1, y1, x2, y2, img_height, principal_y)
        cv2.line(rgb_rot, (axis_ex[0], axis_ex[1]), (axis_ex[2], axis_ex[3]), (0, 255, 0), 1)
        dis_ver_1 = CVFunc.calc_vertical(abs(x1 - principal_x), y1, model_F, model_W, model_a, model_b)
        dis_ver_2 = CVFunc.calc_vertical(abs(x2 - principal_x), y2, model_F, model_W, model_a, model_b)
        cv2.putText(rgb_rot, str(round(dis_ver_1, 0)), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0), 1)
        cv2.putText(rgb_rot, str(round(dis_ver_2, 0)), (x2, y2), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0), 1)
    except Exception as e:
        err_mess += 'calc vertical line\n' + str(e) + '\n'

    ret_value[1] = int(dis_temp_l)
    ret_value[2] = int(dis_temp_r)
    ret_mess += 'L & R,' + str(dis_temp_l) + ',' + str(dis_temp_r) + '\n'
    end_time = time.time()
    time_mess += 'Ver:' + str(round((end_time - start_time) * 1000, 4)) + 'ms\n'

    return rgb_rot, ret_mess, err_mess, time_mess, ret_value


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
    cap.set(11, 80)
    cap.set(12, 80)
    # 获取视频帧率
    print(get_camera_data(cap, c_id))
    if cap.isOpened():
        print('Get1', c_id)
    else:
        cap = cv2.VideoCapture(c_id)
        cap.set(6, 1196444237)
        cap.set(3, 1920)
        cap.set(4, 1080)
        cap.set(5, 30)
        print(get_camera_data(cap, c_id))
        print('Get2', c_id)

    while cap.isOpened():
        ret, frame = cap.read()
        # 抓取图片不成功再重新抓取
        if not ret:
            cap = cv2.VideoCapture(c_id)
            print('Get3', c_id)
            ret, frame = cap.read()
        q.put(frame)
        # print('q.qsize():', q.qsize())
        q.get() if q.qsize() > 1 else time.sleep(0.01)


# 获得视频流帧数图片，调用测距
def distance_get(q, q_s, lock_ser, cap_id, file_address):
    loop_num = 0

    while True:
        rgb_frame = q.get()
        loop_num += 1
        str_Time = datetime.datetime.now().strftime('%H%M%S-%f')
        # 测距
        frame_distance, ret_mess, err_mess, time_mess, ret_value = dis_calc(rgb_frame, cap_id)
        # 显示及保存图片
        # cv2.imshow('Cap', frame_distance)
        cv2.imwrite(file_address + 'C' + str(cap_id) + '-' + str_Time + '.jpg', rgb_frame)
        cv2.imwrite(file_address + 'D' + str(cap_id) + '-' + str_Time + '.jpg', frame_distance)
        # 保存txt
        file_rec = open(file_address + str(cap_id) + '.txt', 'a')
        file_rec.write(str_Time + '  ' + str(loop_num) + '\n')
        if len(ret_mess) > 0:
            file_rec.write('Data:\n' + ret_mess)
        if len(err_mess) > 0:
            file_rec.write('Error:\n' + err_mess)
        if len(time_mess) > 0:
            file_rec.write('Timer:\n' + time_mess)
        file_rec.close()
        q_s.put(ret_value)
        if q_s.qsize() > 1:
            q_s.get()


# 读取UPS和Uart
def uart_ups_get(q0, q1, lock_ser, file_address):
    global se
    loop_num = 0
    i_last = 0.0
    file_rec = open(file_address + 'UPS.txt', 'a')
    str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
    ina_mess, i_value = INA219.get_ina219_data()
    i_last = i_value
    file_rec.write(str_time + '   ' + str(loop_num) + '\n' + ina_mess)
    file_rec.close()
    front_value = 0
    rear_value = 0
    left_value = 0
    right_value = 0
    while True:
        loop_num += 1
        start_time = time.time()
        # 读取两个摄像头测距进程的数据
        get_0 = False
        get_1 = False
        se_send = ''
        if not q0.empty() and not q1.empty():
            get_0 = True
            get_1 = True
            t0_list = q0.get()
            t1_list = q1.get()
            front_value = t0_list[0]
            rear_value = t1_list[0]
            left_value = min(t0_list[1], t1_list[2])
            right_value = min(t0_list[2], t1_list[1])
            se_send = '01 ' + 'F' + str(front_value) + 'R' + str(rear_value) + 'L' + str(left_value) + 'R' + str(right_value)
        elif not q0.empty() and q1.empty():
            get_0 = True
            get_1 = False
            temp_list = q0.get()
            front_value = temp_list[0]
            left_value = temp_list[1]
            right_value = temp_list[2]
            se_send = '0- ' + 'F' + str(front_value) + 'R' + str(rear_value) + 'L' + str(left_value) + 'R' + str(right_value)
        elif q0.empty() and not q1.empty():
            get_0 = False
            get_1 = True
            temp_list = q1.get()
            rear_value = temp_list[0]
            left_value = temp_list[2]
            right_value = temp_list[1]
            se_send = '-1 ' + 'F' + str(front_value) + 'R' + str(rear_value) + 'L' + str(left_value) + 'R' + str(right_value)
        else:
            get_0 = False
            get_1 = False
            se_send = ''
        # 如果有C0或C1的数据，锁定后串口传输
        if get_0 or get_1:
            lock_ser.acquire()
            try:
                se.write(se_send.encode())
            except Exception as e:
                print(e)
            finally:
                str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
                print(str_time)
                print(se_send)
            lock_ser.release()

        # 锁定后读取串口数据
        lock_ser.acquire()
        try:
            line = se.readline()
            if line:
                file_serial = open(file_address + 'Serial.txt', 'a')
                str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
                file_serial.write(str_time + str(line) + '\n')
                print(str_time)
                print(line)
                file_serial.close()
        except Exception as e:
            print(e)
        lock_ser.release()
        # 读取UPS
        if loop_num % 20 == 0:
            file_rec = open(file_address + 'UPS.txt', 'a')
            str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
            ina_mess, i_value = INA219.get_ina219_data()
            file_rec.write(str_time + '   ' + '\n' + ina_mess)
            file_rec.close()
            print('I=' + str(round(i_value, 4)) + 'A')
            if (i_last - i_value) > 1.0:
                # os.system('shutdown now')
                print('shutdown', i_last)
                i_last = i_value
            else:
                i_last = i_value
        end_time = time.time()
        # print('timer', str(loop_num), str(round((end_time - start_time) * 1000, 4)))




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

    mp.set_start_method(method='spawn')  # init
    processes = []
    lock = mp.Lock()
    queue_c0 = mp.Queue(maxsize=2)
    queue_c1 = mp.Queue(maxsize=2)
    queue_s0 = mp.Queue(maxsize=2)
    queue_s1 = mp.Queue(maxsize=2)
    if cap_num == 1:
        processes.append(mp.Process(target=image_put, args=(queue_c0, 0, str_fileAddress)))
        processes.append(mp.Process(target=distance_get, args=(queue_c0, queue_s0, lock, 0, str_fileAddress)))
    elif cap_num > 1:
        processes.append(mp.Process(target=image_put, args=(queue_c0, 0, str_fileAddress)))
        processes.append(mp.Process(target=distance_get, args=(queue_c0, queue_s0, lock, 0, str_fileAddress)))
        processes.append(mp.Process(target=image_put, args=(queue_c1, 1, str_fileAddress)))
        processes.append(mp.Process(target=distance_get, args=(queue_c1, queue_s1, lock, 1, str_fileAddress)))
    else:
        quit()

    processes.append(mp.Process(target=uart_ups_get, args=(queue_s0, queue_s1, lock, str_fileAddress)))

    for process in processes:
        process.daemon = True
        process.start()
    for process in processes:
        process.join()


if __name__ == '__main__':
    run_multi_camera()  # 调用主函数
