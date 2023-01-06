import numpy as np
import signal
import math
import binascii
import serial
import time
import cv2
import datetime
import multiprocessing as mp
import os
import JY61
import G_Vision
import G_Action
import G_RGBD


# 相机编号、IMU串口信息、主板串口通信
camera_id = 0
imu_com = '/dev/ttyTHS1'
imu_baud = 9600
imu_timeout = 0.05
laser_com = '/dev/ttyUSB1'
laser_baud = 115200
laser_timeout = 0.005
machine_com = '/dev/ttyUSB0'
machine_baud = 115200
machine_timeout = 0.15
# se = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

# 机身尺寸、光伏板尺寸
vehicle_left = 118
vehicle_right = 127
vehicle_front = 117
vehicle_width = vehicle_left + vehicle_right
panel_width = 630

# 激光测距阈值
laser_threshold = 17


# 读取IMU数据
def imu_get(q_iv, q_im, q_y, q_c, lock_ser, file_address):
    # cv2.waitKey(500)
    time.sleep(0.5)
    se_i = serial.Serial(imu_com, imu_baud, timeout=imu_timeout)
    # 释放串口积攒的数据
    se_i.flushInput()
    se_i.flushOutput()
    print('IMU start')
    while True:
        try:
            # 控制信号标志位
            is_ctl = False
            # 串口j采集JY61的IMU数据
            # lock_ser.acquire()
            imu_rec = se_i.read(33)
            # lock_ser.release()
            if imu_rec:
                str_imu = binascii.b2a_hex(imu_rec).decode()
                # file_rec = open(file_address + 'JY61.txt', 'a')
                # str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
                if len(str_imu) == 66 and str_imu[0:4] == '5551':
                    jy_list = JY61.DueData(imu_rec)
                    if str(type(jy_list)) != "<class 'NoneType'>":
                        sav_mess = ("normal;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;\n" % jy_list)
                        send_list = [round(jy_list[6], 2), round(jy_list[7], 2), round(jy_list[8], 2)]
                        q_iv.put(send_list)
                        q_iv.get() if q_iv.qsize() > 1 else time.sleep(0.005)
                        q_im.put(send_list)
                        q_im.get() if q_im.qsize() > 1 else time.sleep(0.005)
                    else:
                        sav_mess = ('NoneType;' + str_imu + ';\n')
                        print(sav_mess)
                        se_i.flushOutput()
                else:
                    sav_mess = ('IMU error;' + str_imu + ';\n')
                    print(sav_mess)
                    se_i.flushOutput()
                # file_rec.write(str_time + ';' + sav_mess)
                # file_rec.close()

            if not q_y.empty() and not q_c.empty():
                yaw_c = q_y.get()
                ac_ctl = q_c.get()
                if abs(yaw_c) <= 0.5 and yaw_c != 0.00 and ac_ctl[0] == 0:
                    str_zero = 'ff aa 52'
                    hex_zero = bytes.fromhex(str_zero)
                    se_i.write(hex_zero)
                    # cv2.waitKey(50)
                    time.sleep(0.05)

        except Exception as e:
            print(e)
            print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
            print(f"error line:{e.__traceback__.tb_lineno}")


# 读取激光测距数据
def laser_get(q_lm, file_address):
    se_l = serial.Serial(laser_com, laser_baud, timeout=laser_timeout)
    # 释放串口积攒的数据
    se_l.flushInput()
    se_l.flushOutput()
    while True:
        try:
            # 串口采集TFminiPlus的测距数据
            laser_rec = se_l.read(18)
            if laser_rec:
                str_laser = binascii.b2a_hex(laser_rec).decode()
                # file_rec = open('./TestData/JY61.txt', 'a')
                str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
                if str_laser[0:4] == '5959':
                    hex_f_l = laser_rec[2]
                    hex_f_h = laser_rec[3]
                    hex_s_l = laser_rec[4]
                    hex_s_h = laser_rec[5]
                    laser_F = hex_f_h << 8 | hex_f_l
                    laser_S = hex_s_h << 8 | hex_s_l
                    # print(str(laser_F), str(laser_S))
                    q_lm.put(laser_F)
                    q_lm.get() if q_lm.qsize() > 1 else time.sleep(0.005)
                else:
                    print('laser error:' + str_laser)
                    # se_l.flushOutput()
                # file_rec.write(str_time + ';' + sav_mess)
                # file_rec.close()

        except Exception as e:
            print(e)
            print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
            print(f"error line:{e.__traceback__.tb_lineno}")


# 融合前后单目、超声和IMU，快速更新四向和偏航角
def multi_calc(q_v2s, q_cs, q_m2s, file_address):
    # cv2.waitKey(1000)
    # time.sleep(5)

    # 根据相机编号分配模型参数

    imu_yaw = 0.0
    imu_roll = 0.0
    imu_pitch = 0.0
    ac_ctl = [0]
    dis_laser = laser_threshold
    cam_yaw = -999.9
    side_left = -999.9
    side_right = -999.9

    start_time = time.time()
    end_time = time.time()
    time_mess = round((end_time - start_time) * 1000, 0)

    show_width = 512
    show_height = 600
    # show_width = 480
    # show_height = 270
    # show_width = 960
    # show_height = 540
    half_width = int(show_width / 2)
    half_height = int(show_height / 2)

    while True:
        if not q_v2s.empty():
            print('Get Img')
            break
        else:
            print('No Image')
            # cv2.waitKey(1000)
            time.sleep(5)
    print('calc start')
    while True:
        # 机器定时循环
        if not q_m2s.empty():
            end_time = time.time()
            time_mess = round((end_time - start_time) * 1000, 0)
            start_time_calc = time.time()

            # 接收机器数据
            sensor_list = q_m2s.get()
            if sensor_list[0] < 90:
                dis_laser = sensor_list[4]
                imu_roll = sensor_list[5]
                imu_pitch = sensor_list[6]
                imu_yaw = sensor_list[7]
            else:
                dis_laser = sensor_list[1]
                imu_roll = sensor_list[2]
                imu_pitch = sensor_list[3]
                imu_yaw = sensor_list[4]

            # 发往主板的控制命令
            if not q_cs.empty():
                ac_ctl = q_cs.get()

            # 接收视觉数据
            if not q_v2s.empty():
                cam_yaw, side_left, side_right = q_v2s.get()

            # 右上的距离展示区
            # 左右边界
            rgb_show_line = np.zeros((show_height, show_width, 3), np.uint8)
            if side_left != -999.9:
                temp_x = int(half_width - (half_width * (side_left + vehicle_left) / 500))
                cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (0, 0, 255), 1)
            if side_right != -999.9:
                temp_x = int(half_width + (half_width * (side_right + vehicle_right) / 500))
                cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (0, 0, 255), 1)

            # 视觉的偏航角显示
            if cam_yaw != -999.9:
                yaw_sin = math.sin(math.radians(abs(cam_yaw)))
                yaw_cos = math.cos(math.radians(abs(cam_yaw)))
                if -90.0 <= cam_yaw < 0.0:
                    temp_x = int(half_width - yaw_sin * (half_height * 0.5))
                    temp_y = int(show_height - yaw_cos * (half_height * 0.5))
                    cv2.line(rgb_show_line, (half_width, show_height), (temp_x, temp_y), (160, 0, 240), 3)
                elif 0.0 < cam_yaw <= 90.:
                    temp_x = int(half_width + yaw_sin * (half_height * 0.5))
                    temp_y = int(show_height - yaw_cos * (half_height * 0.5))
                    cv2.line(rgb_show_line, (half_width, show_height), (temp_x, temp_y), (160, 0, 240), 3)

            # IMU的偏航角显示
            yaw_sin = math.sin(math.radians(abs(imu_yaw)))
            yaw_cos = math.cos(math.radians(abs(imu_yaw)))
            if -90 <= imu_yaw < 0:
                temp_x = int(half_width - yaw_sin * (half_height * 0.5))
                temp_y = int(half_height - yaw_cos * (half_height * 0.5))
            elif 0 <= imu_yaw <= 90:
                temp_x = int(half_width + yaw_sin * (half_height * 0.5))
                temp_y = int(half_height - yaw_cos * (half_height * 0.5))
            elif -180 <= imu_yaw < -90:
                temp_x = int(half_width - yaw_sin * (half_height * 0.5))
                temp_y = int(half_height - yaw_cos * (half_height * 0.5))
            elif 90 < imu_yaw <= 180:
                temp_x = int(half_width + yaw_sin * (half_height * 0.5))
                temp_y = int(half_height - yaw_cos * (half_height * 0.5))
            else:
                temp_x = half_width
                temp_y = half_height - (half_height * 0.5)
            cv2.line(rgb_show_line, (half_width, half_height), (temp_x, temp_y), (160, 0, 240), 3)

            # 车身宽度
            temp_x = int(half_width - (half_width * vehicle_left / 500))
            cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (255, 255, 255), 1)
            temp_x = int(half_width + (half_width * vehicle_right / 500))
            cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (255, 255, 255), 1)

            # 右下的数据展示区
            rgb_show_data = np.zeros((show_height, show_width, 3), np.uint8)
            str_0 = 'Timer:' + str(time_mess)
            if cam_yaw != -999.9:
                str_0 += '  Y:' + str(round(cam_yaw, 2))
            else:
                str_0 += '  Y:N/A'
            if dis_laser != -999.9:
                str_0 += '  Laser:' + str(dis_laser)
            else:
                str_0 += '  Laser:N/A'
            if side_left != -999.9:
                str_1 = 'L:' + str(int(side_left))
            else:
                str_1 = 'L:N/A'

            if side_right != -999.9:
                str_2 = 'R:' + str(int(side_right))
            else:
                str_2 = 'R:N/A'

            if len(ac_ctl) > 2:
                if ac_ctl[0] == ac_ctl[2] == 1:
                    if ac_ctl[1] == ac_ctl[3]:
                        str_3 = 'Front'
                    elif ac_ctl[1] > ac_ctl[3]:
                        str_3 = 'Right'
                    elif ac_ctl[1] < ac_ctl[3]:
                        str_3 = 'Left'
                    else:
                        str_3 = 'error'
                elif ac_ctl[0] == ac_ctl[2] == 2:
                    str_3 = 'Back'
                elif ac_ctl[0] == 1 and ac_ctl[2] == 2:
                    str_3 = 'rotateRight'
                elif ac_ctl[0] == 2 and ac_ctl[2] == 1:
                    str_3 = 'rotateLeft'
                elif ac_ctl[0] == ac_ctl[2] == 0:
                    str_3 = 'Stop'
                else:
                    str_3 = 'error'
                if ac_ctl[0] == 0:
                    str_3 += ' '
                elif ac_ctl[0] == 1:
                    str_3 += '+'
                elif ac_ctl[0] == 2:
                    str_3 += '-'
                str_3 += str(ac_ctl[1])
                str_3 += '  '
                if ac_ctl[2] == 0:
                    str_3 += ' '
                elif ac_ctl[2] == 1:
                    str_3 += '+'
                elif ac_ctl[2] == 2:
                    str_3 += '-'
                str_3 += str(ac_ctl[3])
            else:
                str_3 = 'Control:N/A'

            str_3  += '  Yaw:' + str(imu_yaw) + '  Roll:' + str(imu_roll) + '  Pitch:' + str(imu_pitch)

            cv2.putText(rgb_show_data, str_0, (0, int(show_height / 5) - 5), cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 255, 255),
                        1)
            cv2.putText(rgb_show_data, str_1, (0, int(show_height * 2 / 5) - 5), cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 255, 255),
                        1)
            cv2.putText(rgb_show_data, str_2, (0, int(show_height * 3 / 5) - 5), cv2.FONT_HERSHEY_COMPLEX, 0.6,
                        (255, 255, 255), 1)
            cv2.putText(rgb_show_data, str_3, (0, int(show_height * 4 / 5) - 5), cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 255, 255), 1)

            # 4图拼接
            rgb_mix = np.zeros((show_height, (show_width * 2), 3), np.uint8)
            rgb_mix[0:show_height, 0:show_width] = rgb_show_line
            rgb_mix[0:show_height, show_width:(show_width * 2)] = rgb_show_data
            cv2.imshow('ShowData', rgb_mix)
            cv2.waitKey(1)
            # cv2.imwrite(file_address + 'C' + str_Time + '.jpg', rgb_show_0)
            # cv2.imwrite(file_address + 'M' + str_Time + '.jpg', rgb_mix)

            end_time_calc = time.time()
            time_mess_calc = round((end_time_calc - start_time_calc) * 1000, 0)

            start_time = time.time()


def quit_all():
    print('Exit ALL')
    os.kill(os.getpid(), signal.SIGTERM)


# 根据发送的数据，拆分控制信号和主板信号
def read_message(hex_rec):
    str_rec = binascii.b2a_hex(hex_rec).decode()
    is_normal = True
    if len(str_rec) >= 16 and str_rec[0:4] == 'aa01':
        int_move_left = int(str_rec[4:6])
        int_speed_left = int(str_rec[6:8], 16)
        int_move_right = int(str_rec[8:10])
        int_speed_right = int(str_rec[10:12], 16)
        int_board = int(str_rec[12:14])
        int_water = int(str_rec[14:16])
        is_normal = True
        ret_list = [int_move_left, int_speed_left, int_move_right, int_speed_right, int_board, int_water]
    elif len(str_rec) >= 20 and str_rec[0:4] == 'aa02':
        int_fall_FL = int(str_rec[4:6])
        int_fall_FR = int(str_rec[6:8])
        int_fall_BL = int(str_rec[8:10])
        int_fall_BR = int(str_rec[10:12])
        hex_f_h = hex_rec[6]
        hex_f_l = hex_rec[7]
        hex_b_h = hex_rec[8]
        hex_b_l = hex_rec[9]
        laser_F = hex_f_h << 8 | hex_f_l
        laser_B = hex_b_h << 8 | hex_b_l
        is_normal = True
        ret_list = [int_fall_FL, int_fall_FR, int_fall_BL, int_fall_BR]
    else:
        is_normal = False
        ret_list = [0]
    return is_normal, ret_list


# 主板通信交互
def machine_communication(q_a2m, q_m2a, q_m2s, q_i, q_l):
    print('Communication Start')
    se_m = serial.Serial(machine_com, machine_baud, timeout=machine_timeout)
    loop_nofeedback = 0  # 累加无反馈次数
    threshold_nofeedback = 10  # 累计无反馈上限值
    imu_roll = 0.0
    imu_pitch = 0.0
    imu_yaw = 0.0
    dis_laser = 999.9

    while True:
        ret_list = [99]
        try:
            if not q_a2m.empty():
                hex_send = q_a2m.get()
            else:
                # cv2.waitKey(30)
                time.sleep(0.03)
                if not q_a2m.empty():
                    hex_send = q_a2m.get()
                else:
                    # print('Receive Action Error')
                    hex_send = bytes.fromhex('aa 01 00 00 00 00 00 00 b0')
            is_normal, ret_list = read_message(hex_send)
            if is_normal:
                str_send_msg = ''
                if ret_list[0] == ret_list[2] == 1:
                    str_send_msg += 'Front'
                    if ret_list[1] == ret_list[3]:
                        str_send_msg += 'Straight'
                    elif ret_list[1] > ret_list[3]:
                        str_send_msg += 'Right'
                    elif ret_list[1] < ret_list[3]:
                        str_send_msg += 'Left'
                elif ret_list[0] == ret_list[2] == 2:
                    str_send_msg += 'Back'
                elif ret_list[0] == 1 and ret_list[2] == 2:
                    str_send_msg += 'RotateRight'
                elif ret_list[0] == 2 and ret_list[2] == 1:
                    str_send_msg += 'RotateLeft'
                # if len(str_send_msg) > 1:
                    # print(str_send_msg)
            else:
                print('Action Message Error')
            no_feedback = True
            while no_feedback:
                se_m.write(hex_send)
                # cv2.waitKey(50)
                time.sleep(0.05)
                if not q_i.empty():
                    imu_list = q_i.get()
                    imu_roll = imu_list[0]
                    imu_pitch = imu_list[1]
                    imu_yaw = -imu_list[2]
                if not q_l.empty():
                    dis_laser = q_l.get()
                hex_receive = se_m.readline()
                if hex_receive:
                    # 收到反馈，跳出反馈循环
                    no_feedback = False
                    str_rec = binascii.b2a_hex(hex_receive).decode('utf-8')
                    if len(str_rec) >= 12:
                        if str_rec[0:4] == 'aa02':
                            is_normal, ret_list = read_message(hex_receive)
                        else:
                            ret_list = [99]
                            print('头部异常', str_rec)
                    else:
                        ret_list = [99]
                        print('长度异常', str_rec)
                else:
                    # 重新发送命令，并累计无反馈次数。到达阈值后报错退出
                    loop_nofeedback += 1
                    if loop_nofeedback >= threshold_nofeedback:
                        print('无反馈')
                        ret_list = [98]
                        no_feedback = False
        except Exception as e:
            print('******   Machine Communication Error   ******')
            print(e)
            print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
            print(f"error line:{e.__traceback__.tb_lineno}")
            print('******   Error Message End   ******')
        finally:
            ret_list.append(dis_laser)
            ret_list.append(imu_roll)
            ret_list.append(imu_pitch)
            ret_list.append(imu_yaw)
            q_m2a.put(ret_list)
            q_m2a.get() if q_m2a.qsize() > 1 else time.sleep(0.001)
            q_m2s.put(ret_list)
            q_m2s.get() if q_m2s.qsize() > 1 else time.sleep(0.001)


if __name__ == '__main__':
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
    # 单目相机测距，视觉数据给自动控制和展示，视觉偏航角给imu
    queue_capture = mp.Queue(maxsize=2)
    queue_vision2action = mp.Queue(maxsize=2)
    queue_yaw_imu = mp.Queue(maxsize=2)
    queue_vision2show = mp.Queue(maxsize=2)

    # 自动控制交互，控制hex给机器，控制命令给展示和imu，接收键盘输入
    queue_action2machine = mp.Queue(maxsize=2)
    queue_control_show = mp.Queue(maxsize=2)
    queue_control_imu = mp.Queue(maxsize=2)
    queue_keyboard = mp.Queue(maxsize=2)

    # 机器通信交互，反馈数据给自动控制
    queue_machine2action = mp.Queue(maxsize=2)
    queue_machine2show = mp.Queue(maxsize=2)

    # 激光测距数据，发送至自动控制和计算
    queue_laser_machine = mp.Queue(maxsize=2)
    # IMU，偏航、俯仰和翻滚，分发前相机测距、计算和自动控制
    queue_imu_vision = mp.Queue(maxsize=2)
    queue_imu_machine = mp.Queue(maxsize=2)

    # # 单目视觉测距，获取+测距
    # processes.append(mp.Process(target=G_Vision.image_put, args=(queue_capture, camera_id)))
    # processes.append(
    #     mp.Process(target=G_Vision.distance_get, args=(
    #         queue_capture, camera_id, queue_vision2action, queue_yaw_imu, lock, str_fileAddress, queue_vision2show)))

    # 深度相机测距
    processes.append(mp.Process(target=G_RGBD.distance_get, args=(queue_vision2action, queue_yaw_imu, queue_vision2show)))

    # 自动运行导航
    processes.append(
        mp.Process(target=G_Action.autocontrol_run, args=(
            queue_vision2action, queue_machine2action, queue_action2machine, queue_control_show, queue_control_imu, lock, str_fileAddress, queue_keyboard)))


    # 对接机器主板
    processes.append(mp.Process(target=machine_communication, args=(queue_action2machine, queue_machine2action, queue_machine2show, queue_imu_machine, queue_laser_machine)))


    # 视觉感知融合
    processes.append(mp.Process(target=multi_calc, args=(
            queue_vision2show, queue_control_show, queue_machine2show, str_fileAddress)))
    # 激光测距
    processes.append(mp.Process(target=laser_get, args=( queue_laser_machine, str_fileAddress)))
    # IMU
    processes.append(
        mp.Process(target=imu_get, args=(
            queue_imu_vision, queue_imu_machine, queue_yaw_imu, queue_control_imu, lock, str_fileAddress)))

    for process in processes:
        process.daemon = True
        process.start()
    for process in processes:
        process.join()
