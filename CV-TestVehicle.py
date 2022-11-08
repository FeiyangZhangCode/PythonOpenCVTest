# import queue
import cv2
import time
import datetime
import multiprocessing as mp
import numpy as np
import os
import CVFunc
# import INA219
import sys
import signal
import serial
import binascii
# import smbus
# import MPU6050
import JY61
import math
import xlwt


# 读取模型参数
file_model = open('Model.txt', 'r', encoding='utf-8')
para_lines = file_model.readlines()
file_model.close()

# 初始化Canny参数
minCan = 150
maxCan = 400

# 前后单目、IMU串口、超声波串口编号
camera_id = 0
imu_com = '/dev/ttyTHS1'


# 返回摄像头格式
def get_camera_data(cap, num):
    str_Info = 'C' + str(num) + ':' + str(cap.get(3)) + '*' + str(cap.get(4)) + '; FPS' + str(cap.get(5)) + '\n'
    str_fourcc = str("".join([chr((int(cap.get(cv2.CAP_PROP_FOURCC)) >> 8 * k) & 0xFF) for k in range(4)])) + '\n'
    str_Info = str_Info + str_fourcc
    return str_Info


# 计算两点间距离
def getDist_P2P(x1_d, y1_d, x2_d, y2_d):
    distance = math.pow((x1_d - x2_d), 2) + math.pow((y1_d - y2_d), 2)
    distance = math.sqrt(distance)
    return distance


# 根据偏航角角度进行坐标旋转
def points_rotate(angle, org_x, org_y):
    cos_angle = math.cos(math.radians(angle))
    sin_angle = math.sin(math.radians(angle))
    new_x = org_x * cos_angle + org_y * sin_angle
    new_y = org_y * cos_angle - org_x * sin_angle
    return new_x, new_y


# 根据水平线距离反推图像像素高度
def calc_h2y(flo_h, f, w, a, b):
    y_back = (f * w / flo_h - b) / a
    return int(y_back)


# 根据垂直线距离反推图像像素宽度
def calc_w2x(flo_w, y, f, w, a, b, p_x):
    temp_x = abs(flo_w) * (a * y + b) / w
    if flo_w < 0:
        x_back = p_x - temp_x
    else:
        x_back = p_x + temp_x
    return int(x_back)


# 抓取图片，确认视频流的读入
def image_put(q, c_id):
    cap = cv2.VideoCapture(c_id)
    cap.set(6, 1196444237)
    cap.set(3, 1920)
    cap.set(4, 1080)
    cap.set(5, 30)
    if cap.isOpened():
        print('Get1', c_id)
    else:
        cap = cv2.VideoCapture(c_id)
        cap.set(6, 1196444237)
        cap.set(3, 1920)
        cap.set(4, 1080)
        cap.set(5, 30)
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


# 调用相机获取图片进行测距
def distance_get(q_c, q_i, q_y, lock_ser, cap_id, file_address):
    # 根据相机编号分配模型参数
    global para_lines
    global minCan, maxCan

    model_F = float(para_lines[0].strip('\n'))
    model_W = float(para_lines[1].strip('\n'))
    model_a = float(para_lines[2].strip('\n'))
    model_b = float(para_lines[3].strip('\n'))
    principal_x = int(para_lines[4].strip('\n'))
    principal_y = int(para_lines[5].strip('\n'))

    # 循环处理图像
    loop_num = 0
    imu_roll = 0.0
    imu_pitch = 0.0
    imu_yaw = 0.0
    while True:
        start_time_total = time.time()
        str_Time = datetime.datetime.now().strftime('%H%M%S-%f')
        ret_mess = ''  # 数据信息
        time_mess = ''  # 时间信息

        # 获取图像及参数
        start_time = time.time()
        if not q_c.empty():
            rgb_frame = q_c.get()
            loop_num += 1
        else:
            continue
        img_height = int(rgb_frame.shape[0])
        img_width = int(rgb_frame.shape[1])
        mid_height = int(img_height / 2)
        mid_width = int(img_width / 2)
        rgb_rot = rgb_frame.copy()
        rgb_show = rgb_frame.copy()
        # 获取偏航角
        if not q_i.empty():
            jy_list = q_i.get()
            imu_roll = jy_list[0]
            imu_pitch = jy_list[1]
            imu_yaw = jy_list[2]
        end_time = time.time()
        time_mess += 'Cap:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 保留下半部分的红色区域（Canny提取边界，保留下半部分）
        start_time = time.time()
        rgb_half = rgb_frame.copy()
        rgb_half[0:principal_y - 50, :] = (0, 0, 0)
        hsv_half = cv2.cvtColor(rgb_half, cv2.COLOR_BGR2HSV)
        low_range = np.array([0, 123, 100])
        high_range = np.array([5, 255, 255])
        gra_edge = cv2.inRange(hsv_half, low_range, high_range)
        gra_canny = gra_edge.copy()
        # gra_edge = cv2.Canny(rgb_half, minCan, maxCan)
        end_time = time.time()
        time_mess += 'Can:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 获取水平和垂直线
        start_time = time.time()
        gra_edge_rot = gra_edge.copy()
        gra_edge_rot[0:principal_y, :] = 0
        lines = cv2.HoughLinesP(gra_edge_rot, rho=1.0, theta=np.pi / 180, threshold=100, minLineLength=100, maxLineGap=5)
        end_time = time.time()
        time_mess += 'Hou:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 旋转校正、拉平、融合
        start_time = time.time()
        gra_hough = np.zeros((img_height, img_width), np.uint8)  # 创建个全0的黑背景
        # 偏航角
        yaw_avg = 0.0
        yaw_weight = 0.0
        other_avg = 0.0
        other_weight = 0.0
        # 垂直、水平线坐标中继
        num_ver = 0
        lines_ver = [[0, 0, 0, 0]]
        num_hor = 0
        lines_hor = [[0, 0, 0, 0]]
        data_ver = np.zeros((0, 3), dtype=float)  # 垂直线数据，0是最接近中轴的w（x）值，1是最接近相机的h（y）值，2是最远离相机的h（y）值
        # 左边、右边垂直线数据
        num_left = 0
        num_right = 0
        dis_l = [0]
        dis_r = [0]
        # 车身边界数据
        side_left = 0.0
        try:
            if str(type(lines)) != "<class 'NoneType'>":
                if len(lines) > 0:
                    # 计算偏航角
                    for line in lines:
                        for x1_p, y1_p, x2_p, y2_p in line:
                            if getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 100.0:
                                cv2.line(gra_hough, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                                # 根据偏航角进行旋转
                                h1 = CVFunc.calc_horizontal(y1_p, model_F, model_W, model_a, model_b)
                                h2 = CVFunc.calc_horizontal(y2_p, model_F, model_W, model_a, model_b)
                                w1 = CVFunc.calc_vertical((x1_p - principal_x), y1_p, model_F, model_W, model_a,
                                                          model_b)
                                w2 = CVFunc.calc_vertical((x2_p - principal_x), y2_p, model_F, model_W, model_a,
                                                          model_b)

                                # 在2米范围内
                                if h1 < 2000 and h2 < 2000 and w1 < 2000 and w2 < 2000:
                                    if h1 != h2:
                                        angle_tan = np.arctan((w1 - w2) / (h2 - h1)) * 57.29577
                                    else:
                                        angle_tan = 90.0
                                    # x_mid = int((x1_p + x2_p) / 2)
                                    # y_mid = int((y1_p + y2_p) / 2)
                                    # cv2.line(rgb_rot, (x1_p, y1_p), (x2_p, y2_p), (255, 0, 0), 1)
                                    # cv2.putText(rgb_rot, str(round(angle_tan, 0)), (x_mid, y_mid),
                                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)
                                    # cv2.putText(rgb_rot, str(h1) + ',' + str(w1), (x1_p, y1_p),
                                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)
                                    # cv2.putText(rgb_rot, str(h2) + ',' + str(w2), (x2_p, y2_p),
                                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)
                                    if abs(angle_tan) < 45:
                                        temp_weight = math.sqrt(np.square(h1 - h2) + np.square(w1 - w2))
                                        temp_sum = yaw_avg * yaw_weight + angle_tan * temp_weight
                                        yaw_weight = yaw_weight + temp_weight
                                        yaw_avg = temp_sum / yaw_weight
                                        if num_ver == 0:
                                            num_ver = 1
                                            lines_ver[0] = [w1, h1, w2, h2]
                                        else:
                                            num_ver += 1
                                            lines_ver.append([w1, h1, w2, h2])
                                    else:
                                        temp_weight = math.sqrt(np.square(h1 - h2) + np.square(w1 - w2))
                                        temp_sum = other_avg * other_weight + angle_tan * temp_weight
                                        other_weight = other_weight + temp_weight
                                        other_avg = temp_sum / other_weight
                                        if num_hor == 0:
                                            num_hor = 1
                                            lines_hor[0] = [w1, h1, w2, h2]
                                        else:
                                            num_hor += 1
                                            lines_hor.append([w1, h1, w2, h2])
                    # 如果计算了偏航角，则反馈给IMU进程
                    if yaw_weight != 0:
                        q_y.put(yaw_avg)
                        q_y.get() if q_y.qsize() > 1 else time.sleep(0.005)
                    if num_ver != 0:
                        # 根据偏航角调整融合垂直线
                        for w1, h1, w2, h2 in lines_ver:
                            x1_imu, y1_imu = CVFunc.points_rotate(yaw_avg, w1, h1)
                            x2_imu, y2_imu = CVFunc.points_rotate(yaw_avg, w2, h2)
                            # 垂直线,按照最接近中轴来拉直
                            if x1_imu != x2_imu and y1_imu != y2_imu:
                                if abs(x1_imu) > abs(x2_imu):
                                    x1_imu = x2_imu
                                else:
                                    x2_imu = x1_imu
                            # 如果距离中轴的差值不超过50，则认为是同一条，按最接近中轴进行融合。否则新增为新一条。
                            temp_button = min(y1_imu, y2_imu)
                            temp_top = max(y1_imu, y2_imu)
                            if len(data_ver) == 0:
                                data_ver = np.append(data_ver, [[x1_imu, temp_button, temp_top]], axis=0)
                            else:
                                new_ver = True
                                for values_ver in data_ver:
                                    if abs(values_ver[0] - x1_imu) < 50:
                                        if abs(values_ver[0]) > abs(x1_imu):
                                            values_ver[0] = x1_imu
                                        temp_v = min(values_ver[1], temp_button)
                                        values_ver[1] = temp_v
                                        temp_v = max(values_ver[2], temp_top)
                                        values_ver[2] = temp_v
                                        new_ver = False
                                        break
                                if new_ver:
                                    data_ver = np.append(data_ver, [[x1_imu, temp_button, temp_top]], axis=0)
                        # 反推图像位置
                        for values_ver in data_ver:
                            w1_b, h1_b = CVFunc.points_rotate(-yaw_avg, values_ver[0], values_ver[1])
                            w2_b, h2_b = CVFunc.points_rotate(-yaw_avg, values_ver[0], values_ver[2])
                            y1_b = CVFunc.calc_h2y(h1_b, model_F, model_W, model_a, model_b)
                            y2_b = CVFunc.calc_h2y(h2_b, model_F, model_W, model_a, model_b)
                            x1_b = CVFunc.calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
                            x2_b = CVFunc.calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
                            x_mid = int((x1_b + x2_b) / 2)
                            y_mid = int((y1_b + y2_b) / 2)
                            cv2.line(rgb_rot, (x1_b, y1_b), (x2_b, y2_b), (255, 0, 0), 1)
                            cv2.putText(rgb_rot, str(round(values_ver[0], 0)), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.8,
                                        (255, 0, 0), 1)
                            # cv2.line(rgb_show, (x1_b, y1_b), (x2_b, y2_b), (0, 0, 0), 6)
                            if values_ver[0] < 0:
                                num_left += 1
                                if dis_l[0] == 0:
                                    dis_l[0] = int(abs(values_ver[0]))
                                else:
                                    dis_l.append(int(abs(values_ver[0])))
                            else:
                                num_right += 1
                                if dis_r[0] == 0:
                                    dis_r[0] = int(values_ver[0])
                                else:
                                    dis_r.append(int(values_ver[0]))
                    if num_left != 0:
                        dis_l.sort()
                    if num_right != 0:
                        dis_r.sort()
            if num_left != 0:
                temp_yaw = 0.0
                if yaw_weight != 0:
                    temp_yaw = yaw_avg
                else:
                    temp_yaw = -imu_yaw
                if temp_yaw >= 0:
                    side_left = dis_l[0] - 148 * math.sin(math.radians(abs(temp_yaw))) - 115 * math.sin(math.radians(90.0 - abs(temp_yaw)))
                else:
                    side_left = dis_l[0] + 148 * math.sin(math.radians(abs(temp_yaw))) - 115 * math.sin(math.radians(90.0 - abs(temp_yaw)))
            end_time = time.time()
            time_mess += 'Cal:' + str(round((end_time - start_time) * 1000, 4)) + ';'
        except Exception as e:
            print(e)
            print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
            print(f"error line:{e.__traceback__.tb_lineno}")

        # 画相机中心十字
        start_time = time.time()
        cv2.line(rgb_rot, (principal_x, 0), (principal_x, img_height), (255, 255, 0), 1)
        cv2.line(rgb_rot, (0, principal_y), (img_width, principal_y), (255, 255, 0), 1)
        cv2.circle(rgb_rot, (principal_x, principal_y), 5, (255, 255, 0), 3)
        # 显示及保存图片
        show_height = 540
        show_width = 960
        rgb_show = cv2.resize(rgb_rot, (show_width, show_height))
        gra_hough_show = cv2.resize(gra_hough, (show_width, show_height))
        rgb_hough_show = cv2.cvtColor(gra_hough_show, cv2.COLOR_GRAY2BGR)
        gra_canny_show = cv2.resize(gra_canny, (show_width, show_height))
        rgb_canny_show = cv2.cvtColor(gra_canny_show, cv2.COLOR_GRAY2BGR)
        rgb_text_show = np.zeros((show_height, show_width, 3), np.uint8)
        str_0 = 'Roll:' + str(imu_roll) + '  Pitch:' + str(imu_pitch)
        str_1 = 'iYaw:' + str(imu_yaw) + '  cYaw:' + str(round(yaw_avg, 2)) + '  cOth:' + str(round(other_avg, 2))
        str_2 = 'Left:' + str(dis_l[0]) + '  L-side:' + str(side_left)
        cv2.putText(rgb_text_show, str_0, (0, int(show_height / 4)), cv2.FONT_HERSHEY_COMPLEX, 0.8, (255, 255, 255), 1)
        cv2.putText(rgb_text_show, str_1, (0, int(show_height / 2)), cv2.FONT_HERSHEY_COMPLEX, 0.8, (255, 255, 255), 1)
        cv2.putText(rgb_text_show, str_2, (0, int(show_height * 3 / 4)), cv2.FONT_HERSHEY_COMPLEX, 0.8, (255, 255, 255), 1)
        # 4图拼接
        rgb_mix = np.zeros(((show_height * 2), (show_width * 2), 3), np.uint8)
        rgb_mix[0:show_height, 0:show_width] = rgb_canny_show
        rgb_mix[0:show_height, show_width:(show_width * 2)] = rgb_hough_show
        rgb_mix[show_height:(show_height * 2), 0:show_width] = rgb_show
        rgb_mix[show_height:(show_height * 2), show_width:(show_width * 2)] = rgb_text_show
        cv2.imshow('Cap' + str(cap_id), rgb_mix)
        # cv2.imwrite(file_address + 'C' + str(cap_id) + '-' + str_Time + '.jpg', rgb_frame)
        # cv2.imwrite(file_address + 'D' + str(cap_id) + '-' + str_Time + '.jpg', rgb_rot)

        # 保存txt，传输数据
        # q_s.put(rgb_rot)
        # if q_s.qsize() > 1:
        #     q_s.get()
        file_rec = open(file_address + str(cap_id) + '.txt', 'a')
        end_time = time.time()
        time_mess += 'Shw:' + str(round((end_time - start_time) * 1000, 4)) + ';'
        end_time_total = time.time()
        time_mess += 'All:' + str(round((end_time_total - start_time_total) * 1000, 4)) + ';\n'

        if len(time_mess) > 0:
            file_rec.write('Tpy:Timer;' + time_mess)
        file_rec.close()
        cv2.waitKey(1)


# 读取IMU数据
def imu_get(q_id, q_im, q_y, lock_ser, file_address):
    cv2.waitKey(500)
    se_i = serial.Serial(imu_com, 9600, timeout=0.05)
    # 释放串口积攒的数据
    se_i.flushInput()
    se_i.flushOutput()
    print('IMU start')
    while True:
        try:
            # 串口j采集JY61的IMU数据
            # lock_ser.acquire()
            imu_rec = se_i.read(33)
            # lock_ser.release()
            if imu_rec:
                str_imu = binascii.b2a_hex(imu_rec).decode()
                file_rec = open(file_address + 'JY61.txt', 'a')
                str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
                if len(str_imu) == 66 and str_imu[0:4] == '5551':
                    jy_list = JY61.DueData(imu_rec)
                    if str(type(jy_list)) != "<class 'NoneType'>":
                        sav_mess = ("normal;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;\n" % jy_list)
                        send_list = [round(jy_list[6], 2), round(jy_list[7], 2), round(jy_list[8], 2)]
                        # print(str(round(jy_list[6], 2)), str(round(jy_list[7], 2)), str(round(jy_list[8], 2)))
                        q_id.put(send_list)
                        q_id.get() if q_id.qsize() > 1 else time.sleep(0.005)
                        q_im.put(send_list)
                        q_im.get() if q_im.qsize() > 1 else time.sleep(0.005)
                    else:
                        sav_mess = ('NoneType;' + str_imu + ';\n')
                        se_i.flushOutput()
                else:
                    sav_mess = ('error;' + str_imu + ';\n')
                    se_i.flushOutput()
                file_rec.write(str_time + ';' + sav_mess)
                file_rec.close()

            if not q_y.empty():
                yaw_c = q_y.get()
                if abs(yaw_c) <= 0.1:
                    str_zero = 'ff aa 52'
                    hex_zero = bytes.fromhex(str_zero)
                    se_i.write(hex_zero)
                    cv2.waitKey(50)

        except Exception as e:
            print(e)
            print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
            print(f"error line:{e.__traceback__.tb_lineno}")



def quit_all():
    print('Exit ALL')
    os.kill(os.getpid(), signal.SIGTERM)


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

    mp.set_start_method(method='spawn')  # init
    processes = []
    lock = mp.Lock()
    queue_c = mp.Queue(maxsize=2)

    queue_dis_yaw = mp.Queue(maxsize=2)

    queue_imu_dis = mp.Queue(maxsize=2)
    queue_imu_main = mp.Queue(maxsize=2)

    processes.append(mp.Process(target=image_put, args=(queue_c, camera_id)))
    processes.append(
        mp.Process(target=distance_get, args=(queue_c, queue_imu_dis, queue_dis_yaw, lock, camera_id, str_fileAddress)))
    processes.append(
        mp.Process(target=imu_get, args=(queue_imu_dis, queue_imu_main, queue_dis_yaw, lock, str_fileAddress)))

    for process in processes:
        process.daemon = True
        process.start()
    for process in processes:
        process.join()


if __name__ == '__main__':
    run_multi_camera()  # 调用主函数
