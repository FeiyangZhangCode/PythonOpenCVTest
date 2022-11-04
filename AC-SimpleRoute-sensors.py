import binascii
import serial
import time
import cv2
import crcmod
import keyboard
import datetime
import multiprocessing as mp
import os
import JY61

se = serial.Serial('/dev/ttyTHS1', 115200, timeout=0.1)
imu_com = '/dev/ttyUSB0'

laser_TH_F = 200  # 前向超声波阈值
laser_TH_B = 150  # 后向超声波阈值
laser_TH_LR = 80

cmd_0_head = 'aa 01'
cmd_1_stop = '00'
cmd_1_moveFront = '01'
cmd_1_moveBack = '02'
cmd_1_rotateLeft = '03'
cmd_1_rotateRight = '04'
cmd_2_speed0 = '00'
cmd_2_max = '64'
cmd_3_stop = '00'
cmd_3_front = '01'
cmd_3_back = '02'
cmd_4_stop = '00'
cmd_4_start = '01'


# 将命令更改为串口输出的16进制，并加上crc8校验
def set_order(str_order):
    hex_order = bytes.fromhex(str_order)
    crc8 = crcmod.predefined.Crc('crc-8')
    crc8.update(hex_order)
    hex_crc8 = bytes.fromhex(hex(crc8.crcValue)[2:])
    hex_order = hex_order + hex_crc8
    return hex_order


# 速度改16进制
def trans_speed(str_speed):
    int_speed = int(str_speed)
    cmd_speed = hex(int_speed)[2:]
    if int_speed > 100:
        cmd_speed = '64'
    elif int_speed < 16:
        cmd_speed = '0' + cmd_speed
    return cmd_speed


# 读取各个传感器信息，返回判断后的状态
def read_feedback(hex_rec, i_yaw):
    rec_mess = binascii.b2a_hex(hex_rec).decode()
    sensor_state = 0
    # 拆分数据包的4个防跌落
    if len(rec_mess) >= 20:
        if rec_mess[0:4] == 'aa02':
            int_fall_FL = int(rec_mess[4:6])
            int_fall_FR = int(rec_mess[6:8])
            int_fall_BL = int(rec_mess[8:10])
            int_fall_BR = int(rec_mess[10:12])
            hex_f_h = hex_rec[6]
            hex_f_l = hex_rec[7]
            hex_b_h = hex_rec[8]
            hex_b_l = hex_rec[9]
            laser_F = hex_f_h << 8 | hex_f_l
            laser_B = hex_b_h << 8 | hex_b_l
        else:
            int_fall_FL = int_fall_FR = int_fall_BL = int_fall_BR = 0
            laser_F = laser_B = 0
    else:
        int_fall_FL = int_fall_FR = int_fall_BL = int_fall_BR = 0
        laser_F = laser_B = 0
    # 多传感器判断
    if (int_fall_FL + int_fall_FR + int_fall_BL + int_fall_BR) > 0:  # 优先判断防跌落
        if (int_fall_FL + int_fall_FR + int_fall_BL + int_fall_BR) > 2:
            sensor_state = 99
        elif int_fall_FL + int_fall_FR == 2:
            sensor_state = 1
        elif int_fall_BL + int_fall_BR == 2:
            sensor_state = 2
        elif int_fall_FL + int_fall_BL == 2:
            sensor_state = 3
        elif int_fall_FR + int_fall_BR == 2:
            sensor_state = 4
        elif int_fall_FL == 1:
            if i_yaw <= -2.0:
                sensor_state = 3
            else:
                sensor_state = 1
        elif int_fall_FR == 1:
            if i_yaw >= 2.0:
                sensor_state = 4
            else:
                sensor_state = 1
        elif int_fall_BL == 1:
            if i_yaw >= 2.0:
                sensor_state = 3
            else:
                sensor_state = 2
        elif int_fall_BR == 1:
            if i_yaw <= -2.0:
                sensor_state = 4
            else:
                sensor_state = 2
    elif laser_F < laser_TH_F or laser_B < laser_TH_B:  # 无跌落风险，判断前后测距
        if laser_F < laser_TH_F and laser_B < laser_TH_B:
            sensor_state = 99
        elif abs(i_yaw) <= 5:
            if laser_F < laser_TH_F:
                sensor_state = 1
            elif laser_B < laser_TH_B:
                sensor_state = 2
        elif i_yaw < -5.0:
            if laser_F < laser_TH_LR:
                sensor_state = 3
            elif laser_B < laser_TH_LR:
                sensor_state = 4
        elif i_yaw > 5.0:
            if laser_F < laser_TH_LR:
                sensor_state = 4
            elif laser_B < laser_TH_LR:
                sensor_state = 3
    elif abs(i_yaw) > 5:  # 没到边沿，判断偏航
        sensor_state = 5
    return sensor_state


# 读取主板的反馈信息，返回各个传感器数据
def read_sensors(hex_rec):
    rec_mess = binascii.b2a_hex(hex_rec).decode()
    fall_FL = str(int(rec_mess[4:6]))
    fall_FR = str(int(rec_mess[6:8]))
    fall_BL = str(int(rec_mess[8:10]))
    fall_BR = str(int(rec_mess[10:12]))
    hex_f_h = hex_rec[6]
    hex_f_l = hex_rec[7]
    hex_b_h = hex_rec[8]
    hex_b_l = hex_rec[9]
    laser_F = hex_f_h << 8 | hex_f_l
    laser_B = hex_b_h << 8 | hex_b_l
    show_mess = 'Fall:' + fall_FL + ';' + fall_FR + ';' + fall_BL + ';' + fall_BR + '. Laser:' + str(laser_F) + '-' + str(laser_B)
    return show_mess


# 串口读取JY-61的IMU数据
def imu_get(q_i, lock_ser, file_address):
    se_i = serial.Serial(imu_com, 9600, timeout=0.05)
    while True:
        try:
            # lock_ser.acquire()
            imu_rec = se_i.read(33)
            # lock_ser.release()
            if imu_rec:
                str_imu = binascii.b2a_hex(imu_rec).decode()
                file_rec = open(file_address + 'JY61.txt', 'a')
                str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
                if len(str_imu) == 66 and str_imu[0:4] == '5551':
                    jy_list = JY61.DueData(imu_rec)
                    sav_mess = (
                                "normal;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;\n" % jy_list)
                    send_list = [round(jy_list[6], 2), round(jy_list[7], 2), round(jy_list[8], 2)]
                    q_i.put(send_list)
                    q_i.get() if q_i.qsize() > 1 else time.sleep(0.005)
                else:
                    sav_mess = ('error;' + str_imu + ';\n')
                    se_i.flushOutput()
                file_rec.write(str_time + ';' + sav_mess)
                file_rec.close()
        except Exception as e:
            print(e)
            print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
            print(f"error line:{e.__traceback__.tb_lineno}")


# 根据动作组进行执行，list的0是动作的16进制命令，1是执行多少次
def func_action(list_action, q_i, lock_ser, file_address):
    sensor_state = 0  # 传感器状态，99是传感器异常，98是无反馈，97是运行错误

    imu_yaw = 0.0
    imu_pitch = 0.0
    imu_roll = 0.0

    for id_action in range(0, len(list_action), 1):
        loop_nofeedback = 0  # 累加无反馈次数
        threshold_nofeedback = 10  # 累计无反馈上限值
        hex_action = list_action[id_action][0]
        time_action = list_action[id_action][1]
        for num_action in range(0, time_action, 1):
            no_feedback = True
            while no_feedback:
                try:
                    se.write(hex_action)
                    str_send = binascii.b2a_hex(hex_action).decode('utf-8')
                    str_Time = datetime.datetime.now().strftime('%H:%M:%S.%f')
                    file_rec = open(file_address + 'Control.txt', 'a')
                    file_rec.write(str_Time + ';auto;s;' + str_send + ';\n')
                    file_rec.close()

                    cv2.waitKey(60)

                    # 获取偏航角
                    if not q_i.empty():
                        imu_list = q_i.get()
                        imu_yaw = imu_list[0]
                        imu_pitch = imu_list[1]
                        imu_roll = imu_list[2]

                    hex_rec = se.readline()
                    if hex_rec:
                        # 收到反馈，跳出反馈循环
                        no_feedback = False
                        str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
                        if len(str_rec) >= 12:
                            if str_rec[0:4] == 'aa02':
                                sensor_mess = read_sensors(hex_rec)
                                print(sensor_mess, str(imu_yaw))
                            else:
                                print('头部异常', str_rec)
                        else:
                            print('长度异常', str_rec)
                        file_rec = open(file_address + 'Control.txt', 'a')
                        file_rec.write(str_Time + ';auto;r;' + str_rec + ';\n')
                        file_rec.close()
                        # if sensor_state > 0:
                        #     return sensor_state, id_action
                    else:
                        # 重新发送命令，并累计无反馈次数
                        loop_nofeedback += 1
                        print('无反馈', str(loop_nofeedback))
                        if loop_nofeedback >= threshold_nofeedback:
                            sensor_state = 98
                            return sensor_state, id_action
                except Exception as e_fa:
                    print(e_fa)
                    print(f'error file:{e_fa.__traceback__.tb_frame.f_globals["__file__"]}')
                    print(f"error line:{e_fa.__traceback__.tb_lineno}")
                    sensor_state = 97
                    return sensor_state, id_action

    return sensor_state, 0


# 多步执行动作
def multi_action(hex_action, times_action, q_i, lock_ser, file_address):
    sensor_state = 0  # 传感器状态，99是传感器异常，98是无反馈，97是运行报错
    loop_nofeedback = 0  # 累加无反馈次数
    threshold_nofeedback = 10  # 累计无反馈上限值
    imu_yaw = 0.0
    imu_pitch = 0.0
    imu_roll = 0.0

    for id_action in range(0, times_action, 1):
        no_feedback = True
        while no_feedback:
            try:
                se.write(hex_action)
                str_send = binascii.b2a_hex(hex_action).decode('utf-8')
                str_Time = datetime.datetime.now().strftime('%H:%M:%S.%f')
                file_rec = open(file_address + 'Control.txt', 'a')
                file_rec.write(str_Time + ',multi,s,' + str_send + '\r\n')
                file_rec.close()
                # 获取偏航角
                if not q_i.empty():
                    imu_list = q_i.get()
                    imu_yaw = imu_list[0]
                    imu_pitch = imu_list[1]
                    imu_roll = imu_list[2]

                cv2.waitKey(40)
                hex_rec = se.readline()
                if hex_rec:
                    # 收到反馈，跳出反馈循环
                    no_feedback = False
                    str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
                    file_rec = open(file_address + 'Control.txt', 'a')
                    file_rec.write(str_Time + ',multi,r,' + str_rec + '\r\n')
                    file_rec.close()
                    sensor_state = read_feedback(hex_rec, imu_yaw)
                    if sensor_state != 0 and sensor_state != 5:
                        break
                else:
                    # 重新发送命令，并累计无反馈次数
                    loop_nofeedback += 1
                    print('无反馈', str(loop_nofeedback))
                    if loop_nofeedback >= threshold_nofeedback:
                        sensor_state = 98
                        no_feedback = False
                        break
            except Exception as e_sa:
                print(e_sa)
                print(f'error file:{e_sa.__traceback__.tb_frame.f_globals["__file__"]}')
                print(f"error line:{e_sa.__traceback__.tb_lineno}")
                sensor_state = 97
                no_feedback = False
                break
        if sensor_state != 0 and sensor_state != 5:
            break
    if sensor_state == 5:
        sensor_state = 0
    return sensor_state


# 校正偏航
def correct_yaw(now_yaw, target_yaw, cmd_34, q_i, lock_ser, file_address):
    get_correct_err = 0
    correct_speed = 100
    imu_yaw = now_yaw
    rot_yaw = now_yaw - target_yaw

    loop_nofeedback = 0  # 累加无反馈次数
    threshold_nofeedback = 10  # 累计无反馈上限值

    while abs(rot_yaw) > 1.0:
        last_yaw = imu_yaw - target_yaw
        # 设置旋转动作
        cmd_2_corSpeed = trans_speed(str(correct_speed))
        if rot_yaw < 0:
            hex_correctYaw = set_order(cmd_0_head + cmd_1_rotateRight + cmd_2_corSpeed + cmd_34)
        else:
            hex_correctYaw = set_order(cmd_0_head + cmd_1_rotateLeft + cmd_2_corSpeed + cmd_34)

        # 发送动作命令
        se.write(hex_correctYaw)
        str_send = binascii.b2a_hex(hex_correctYaw).decode('utf-8')
        str_Time_cy = datetime.datetime.now().strftime('%H:%M:%S.%f')
        file_rec = open(file_address + 'Control.txt', 'a')
        file_rec.write(str_Time_cy + ';CorrectYaw;s;' + str_send + ';\n')
        file_rec.close()
        # 等待串口发送
        cv2.waitKey(40)

        # 获取偏航角
        if not q_i.empty():
            imu_list = q_i.get()
            imu_yaw = imu_list[0]
            imu_pitch = imu_list[1]
            imu_roll = imu_list[2]

        # 读取主板反馈
        hex_rec = se.readline()
        if hex_rec:
            # 收到反馈
            str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
            file_rec = open(file_address + 'Control.txt', 'a')
            file_rec.write(str_Time_cy + ';CorrectYaw;r;' + str_rec + ';\n')
            file_rec.close()
            # sensor_state = read_feedback(hex_rec, imu_yaw)
        else:
            # 累计无反馈次数，继续执行
            loop_nofeedback += 1
            print('无反馈', str(loop_nofeedback))
            if loop_nofeedback >= threshold_nofeedback:
                get_correct_err = 98
                print('无反馈，结束运行')
                break

        # 获取偏航角
        if not q_i.empty():
            imu_list = q_i.get()
            imu_yaw = imu_list[0]
            imu_pitch = imu_list[1]
            imu_roll = imu_list[2]

        rot_yaw = imu_yaw - target_yaw
        if abs(rot_yaw) <= 1.0:     # 如果与目标角度相差不大于1，则认为已完成，
            get_correct_err = 0
            break
        else:
            if (abs(last_yaw) > abs(rot_yaw)) and (
                    (last_yaw < 0 and rot_yaw < 0) or (last_yaw > 0 and rot_yaw > 0)):  # 相差变小并且同向
                pass
            elif (last_yaw < 0 and rot_yaw > 0) or (last_yaw > 0 and rot_yaw < 0):  # 偏航角反向了，降低旋转速度
                if correct_speed > 25:
                    correct_speed = correct_speed - 25
    return get_correct_err


# 前后清洗
def go_wash(is_front, cmd_2, cmd_34, q_i, lock_ser, file_address, loop_times):
    sensor_state = 0  # 传感器状态，99是传感器异常，98是无反馈，97是运行报错
    loop_nofeedback = 0  # 累加无反馈次数
    threshold_nofeedback = 10  # 累计无反馈上限值
    imu_yaw = 0.0
    imu_pitch = 0.0
    imu_roll = 0.0
    loop_num = 0

    if is_front:
        hex_wash = set_order(cmd_0_head + cmd_1_moveFront + cmd_2 + cmd_34)
    else:
        hex_wash = set_order(cmd_0_head + cmd_1_moveBack + cmd_2 + cmd_34)

    while sensor_state == 0 and loop_num <= loop_times:
        no_feedback = True
        while no_feedback:
            try:
                se.write(hex_wash)
                str_send = binascii.b2a_hex(hex_wash).decode('utf-8')
                str_Time_wash = datetime.datetime.now().strftime('%H:%M:%S.%f')
                file_rec = open(file_address + 'Control.txt', 'a')
                file_rec.write(str_Time_wash + ';wash;s;' + str_send + ';\n')
                file_rec.close()

                # 等待串口发送
                cv2.waitKey(40)

                # 获取偏航角
                if not q_i.empty():
                    imu_list = q_i.get()
                    imu_yaw = imu_list[0]
                    imu_pitch = imu_list[1]
                    imu_roll = imu_list[2]

                # 读取串口反馈
                hex_rec = se.readline()

                # 获取偏航角
                if not q_i.empty():
                    imu_list = q_i.get()
                    imu_yaw = imu_list[0]
                    imu_pitch = imu_list[1]
                    imu_roll = imu_list[2]

                if hex_rec:
                    # 收到反馈，跳出反馈循环，累加次数
                    loop_num += 1
                    no_feedback = False
                    str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
                    file_rec = open(file_address + 'Control.txt', 'a')
                    file_rec.write(str_Time_wash + ';wash;r;' + str_rec + ';\n')
                    file_rec.close()
                    sensor_state = read_feedback(hex_rec, imu_yaw)
                    # 到左右边，移动远离边界
                    if 2 < sensor_state < 5:
                        # 停止移动，旋转角度
                        if is_front:
                            hex_correct = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_34)
                            get_correct = multi_action(hex_correct, 2, q_i, lock_ser, file_address)
                            if get_correct == 98:
                                return get_correct

                            if sensor_state == 3:
                                get_correct = correct_yaw(imu_yaw, 10.0, cmd_34, q_i, lock_ser, file_address)
                                if get_correct == 98:
                                    return get_correct
                            elif sensor_state == 4:
                                get_correct = correct_yaw(imu_yaw, -10.0, cmd_34, q_i, lock_ser, file_address)
                                if get_correct == 98:
                                    return get_correct
                        else:
                            hex_correct = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_34)
                            get_correct = multi_action(hex_correct, 2, q_i, lock_ser, file_address)
                            if get_correct == 98:
                                return get_correct

                            if sensor_state == 3:
                                get_correct = correct_yaw(imu_yaw, -10.0, cmd_34, q_i, lock_ser, file_address)
                                if get_correct == 98:
                                    return get_correct
                            elif sensor_state == 4:
                                get_correct = correct_yaw(imu_yaw, 10.0, cmd_34, q_i, lock_ser, file_address)
                                if get_correct == 98:
                                    return get_correct
                        # 旋转后直行，远离边界
                        get_correct = multi_action(hex_wash, 10, q_i, lock_ser, file_address)
                        if get_correct == 98:
                            return get_correct
                    # 执行远离边界或清洗偏航，偏航校正
                    if 2 < sensor_state < 6:
                        get_correct = correct_yaw(imu_yaw, 0.0, cmd_34, q_i, lock_ser, file_address)  # 偏航校正
                        if get_correct == 98:
                            return get_correct
                        sensor_state = 0
                else:
                    # 重新发送命令，并累计无反馈次数
                    loop_nofeedback += 1
                    print('无反馈', str(loop_nofeedback))
                    if loop_nofeedback >= threshold_nofeedback:
                        sensor_state = 98
                        no_feedback = False
            except Exception as e_sa:
                print(e_sa)
                print(f'error file:{e_sa.__traceback__.tb_frame.f_globals["__file__"]}')
                print(f"error line:{e_sa.__traceback__.tb_lineno}")
                sensor_state = 97
                no_feedback = False

    return sensor_state


# 左右平移
def change_column(is_top, to_left, cmd_34, q_i, lock_ser, file_address):
    re_state = 0
    hex_moveFront = set_order(cmd_0_head + cmd_1_moveFront + cmd_2_max + cmd_34)
    hex_moveBack = set_order(cmd_0_head + cmd_1_moveBack + cmd_2_max + cmd_34)
    if is_top:  # 如果在上边
        # 后退1秒，离开上边
        move_state = multi_action(hex_moveBack, 5, q_i, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state
        # 左旋或右旋30度
        if to_left:
            move_state = correct_yaw(0.0, 30.0, cmd_34, q_i, lock_ser, file_address)
        else:
            move_state = correct_yaw(0.0, -30.0, cmd_34, q_i, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state

        # 后退2秒，斜向行驶形成横向距离
        move_state = multi_action(hex_moveBack, 10, q_i, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state

        # 旋转回垂直
        if to_left:
            move_state = correct_yaw(30.0, 0.0, cmd_34, q_i, lock_ser, file_address)
        else:
            move_state = correct_yaw(-30.0, 0.0, cmd_34, q_i, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state

        # 前进2秒，回到上边
        move_state = multi_action(hex_moveFront, 10, q_i, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state

    else:  # 如果在下边
        # 前进1秒，离开下边
        move_state = multi_action(hex_moveFront, 5, q_i, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state

        # 左旋或右旋30度
        if to_left:
            move_state = correct_yaw(0.0, -30.0, cmd_34, q_i, lock_ser, file_address)
        else:
            move_state = correct_yaw(0.0, 30.0, cmd_34, q_i, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state

        # 前进2秒，斜向行驶形成横向距离
        move_state = multi_action(hex_moveFront, 10, q_i, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state

        # 旋转回垂直
        if to_left:
            move_state = correct_yaw(-30.0, 0.0, cmd_34, q_i, lock_ser, file_address)
        else:
            move_state = correct_yaw(30.0, 0.0, cmd_34, q_i, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state

        # 后退2秒，回到下边
        move_state = multi_action(hex_moveBack, 10, q_i, lock_ser, file_address)
        if move_state == 98:
            return move_state
        elif 2 < move_state < 5:
            re_state = move_state

    return re_state


# 执行自动控制
def autocontrol_run(q_i, lock_ser, file_address):
    # 设置上下次数
    loop_time = 2  # 上行+下行算作1次
    # 设置平移方向及次数
    move_side = 0  # 0不平移，1左移，2右移
    move_times = 0  # 设置平移次数
    move_num = 0  # 累计平移次数

    # 设置清洗速度
    cmd_2_washSpeed = trans_speed('50')
    # 设置移动速度
    cmd_2_moveSpeed = trans_speed('100')
    # 设置旋转速度
    cmd_2_rotatePalstance = trans_speed('50')

    # 设置上下清洗的移动距离限定，-1代表无限定。每次200ms，5次相当于1秒
    wash_loops = 20

    # 启动水系统
    hex_sprayStart = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_stop + cmd_4_start)
    # 全停止
    hex_allStop = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_stop + cmd_4_stop)
    # 刮板向前
    hex_boardFront = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_front + cmd_4_stop)
    # 刮板向前，启动水系统
    hex_sprayFront = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_front + cmd_4_start)
    # 清洗前进
    hex_washFront = set_order(cmd_0_head + cmd_1_moveFront + cmd_2_washSpeed + cmd_3_front + cmd_4_start)
    # 清洗前进，移动停止
    hex_stopFront = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_front + cmd_4_start)
    # 刮板向前，关闭水系统
    hex_sprayStop_boardFront = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_front + cmd_4_stop)
    # 刮板向后
    hex_boardBack = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_back + cmd_4_stop)
    # 刮板向下，启动水系统
    hex_sprayBack = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_back + cmd_4_start)
    # 清洗下行
    hex_washBack = set_order(cmd_0_head + cmd_1_moveBack + cmd_2_washSpeed + cmd_3_back + cmd_4_start)
    # 清洗下行，移动停止
    hex_stopBack = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_back + cmd_4_start)
    # 刮板向下，关闭水系统
    hex_sprayStop_boardBack = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_back + cmd_4_stop)
    # 移动向后
    hex_moveBack = set_order(cmd_0_head + cmd_1_moveBack + cmd_2_moveSpeed + cmd_3_stop + cmd_4_stop)
    # 右旋
    hex_rotateRight = set_order(cmd_0_head + cmd_1_rotateRight + cmd_2_rotatePalstance + cmd_3_stop + cmd_4_stop)
    # 左旋
    hex_rotateLeft = set_order(cmd_0_head + cmd_1_rotateLeft + cmd_2_rotatePalstance + cmd_3_stop + cmd_4_stop)
    # 移动向前
    hex_moveFront = set_order(cmd_0_head + cmd_1_moveFront + cmd_2_moveSpeed + cmd_3_stop + cmd_4_stop)

    # 动作组-清洗前进
    list_washFront = [[hex_boardFront, 15],  # 刮板向前，15次约3秒
                      [hex_sprayFront, 1],  # 启动水系统，1次
                      [hex_washFront, 20]]  # 保持清洗向前，测试阶段用20次
    # 动作组-清洗向后
    list_washBack = [[hex_boardBack, 15],  # 刮板后，15次约3秒
                     [hex_sprayBack, 1],  # 启动水系统，1次
                     [hex_washBack, 20]]  # 保持清洗向后，暂用20次

    # 动作组-准备清洗向前
    list_prepareFront = [[hex_boardFront, 15],  # 刮板向前，15次约3秒
                         [hex_sprayFront, 1]]  # 启动水系统，1次

    # 动作组-准备清洗向后
    list_prepareBack = [[hex_boardBack, 15],  # 刮板后，15次约3秒
                        [hex_sprayBack, 1]]  # 启动水系统，1次

    # 动作组-清洗向前到边
    list_edgeFront = [[hex_stopFront, 2],  # 移动停止，2次
                      [hex_sprayStop_boardFront, 1],  # 关闭水系统，1次
                      [hex_boardBack, 7],  # 刮板由前，改为向后，7次约1.4秒，预计到中间位置
                      [hex_allStop, 1]]  # 全停止，进行下一指令
    # 动作组-清洗向后到边
    list_edgeBack = [[hex_stopBack, 2],  # 移动停止，2次
                     [hex_sprayStop_boardBack, 1],  # 关闭水系统，1次
                     [hex_boardFront, 7],  # 刮板由前，改为向后，7次与1.4秒，预计到中间位置
                     [hex_allStop, 1]]  # 全停止，进行下一指令

    # 动作组-上边向左
    list_Top2Left = [[hex_moveBack, 5],  # 移动下行，5次
                     [hex_rotateRight, 10],  # 右旋，10次
                     [hex_moveBack, 10],  # 移动下行，10次，斜向移动挪向左
                     [hex_rotateLeft, 10],  # 左旋，10次，转回垂直状态
                     [hex_moveFront, 10]]  # 移动上行，10次，移回接近边沿
    # 动作组-上边向右
    list_Top2Right = [[hex_moveBack, 5],  # 移动下行，5次
                      [hex_rotateLeft, 10],  # 左旋，10次
                      [hex_moveBack, 10],  # 移动下行，10次
                      [hex_rotateRight, 10],  # 右旋，10次
                      [hex_moveFront, 10]]  # 移动上行，10次
    # 动作组-下边向左
    list_Button2Left = [[hex_moveFront, 5],
                        [hex_rotateLeft, 10],
                        [hex_moveFront, 10],
                        [hex_rotateRight, 10],
                        [hex_moveBack, 10]]
    # 动作组-下边向右
    list_Button2Right = [[hex_moveFront, 5],
                         [hex_rotateRight, 10],
                         [hex_moveFront, 10],
                         [hex_rotateLeft, 10],
                         [hex_moveBack, 10]]

    # 动作组-动作测试
    list_ActionTest = [[hex_moveFront, 10],
                       [hex_allStop, 10],
                       [hex_moveBack, 10],
                       [hex_allStop, 10],
                       [hex_rotateLeft, 10],
                       [hex_allStop, 10],
                       [hex_rotateRight, 20],
                       [hex_allStop, 10],
                       [hex_rotateLeft, 10],
                       [hex_allStop, 10],
                       [hex_boardFront, 15],
                       [hex_allStop, 10],
                       [hex_boardBack, 7],
                       [hex_allStop, 10],
                       [hex_boardBack, 15],
                       [hex_allStop, 10],
                       [hex_boardFront, 7],
                       [hex_allStop, 10],
                       [hex_sprayStart, 10],
                       [hex_allStop, 10]]

    imu_yaw = 0.0
    imu_pitch = 0.0
    imu_roll = 0.0
    board_cmd = cmd_3_stop + cmd_4_stop

    # 无反馈重新循环标志位
    no_feedBack = False
    while True:
        # 测试通信
        in_init = True
        while in_init:
            str_init = 'aa 01 00 00 00 00 a8'
            hex_init_send = bytes.fromhex(str_init)
            str_Time_ac = datetime.datetime.now().strftime('%H:%M:%S.%f')
            try:
                # 获取偏航角
                if not q_i.empty():
                    imu_list = q_i.get()
                    imu_yaw = imu_list[0]
                    imu_pitch = imu_list[1]
                    imu_roll = imu_list[2]

                se.write(hex_init_send)
                file_rec = open(file_address + 'Control.txt', 'a')
                file_rec.write(str_Time_ac + ';init;s;' + str_init + ';\n')
                file_rec.close()
                # 发送后，延时读取
                cv2.waitKey(30)

                # 获取偏航角
                if not q_i.empty():
                    imu_list = q_i.get()
                    imu_yaw = imu_list[0]
                    imu_pitch = imu_list[1]
                    imu_roll = imu_list[2]

                hex_init_rec = se.readline()
                if hex_init_rec:
                    str_init_rec = binascii.b2a_hex(hex_init_rec).decode('utf-8')
                    if len(str_init_rec) >= 12:
                        if str_init_rec[0:4] == 'aa02':
                            sensor_init_mess = read_sensors(hex_init_rec)
                            print(sensor_init_mess, str(imu_yaw))
                            in_init = False
                            no_feedBack = False
                        else:
                            print('串口数据异常')
                    else:
                        print('串口数据异常')
                    file_rec = open(file_address + 'Control.txt', 'a')
                    file_rec.write(str_Time_ac + ';init;r;' + str_init_rec + ';\n')
                    file_rec.close()
            except Exception as e:
                print(e)
                print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
                print(f"error line:{e.__traceback__.tb_lineno}")

        # 初始偏航校正
        print('初始偏航校正')
        # 获取偏航角
        if not q_i.empty():
            imu_list = q_i.get()
            imu_yaw = imu_list[0]
            imu_pitch = imu_list[1]
            imu_roll = imu_list[2]

        if abs(imu_yaw) > 3.0:
            board_cmd = cmd_3_stop + cmd_4_stop
            get_state = correct_yaw(imu_yaw, 0.0, board_cmd, q_i, lock_ser, file_address)
            if get_state == 98:
                no_feedBack = True
        if no_feedBack:
            continue

        # 进入自动控制
        print('进入自动控制')
        get_state = 0
        # 建立通信，开始执行
        for loop_num in range(0, loop_time, 1):
            # 准备清洗前进
            print('刮板向前，启动水系统')
            get_state = func_action(list_prepareFront, q_i, lock_ser, file_address)
            board_cmd = cmd_3_front + cmd_4_start
            if get_state == 98:
                no_feedBack = True
                break

            # 清洗前进
            print('清洗前行，次数', str(wash_loops))
            get_state = go_wash(True, cmd_2_washSpeed, board_cmd, q_i, lock_ser, file_address, wash_loops)
            if get_state == 98:
                no_feedBack = True
                break
            elif get_state == 2:
                print('后向误判')
            elif get_state > 90:
                print('故障')

            # 上行到上边
            print('到达顶部')
            get_state = func_action(list_edgeFront, q_i, lock_ser, file_address)
            board_cmd = cmd_3_stop + cmd_4_stop
            if get_state == 98:
                no_feedBack = True
                break

            # 到上边平移
            if move_side == 0:  # 直行，不平移
                pass
            elif move_side == 1:  # 上边左移
                print('向左平移，已移动', str(move_num))
                get_err_T2L = change_column(True, True, board_cmd, q_i, lock_ser, file_address)
                move_num += 1
                if move_num >= move_times:
                    move_num = 0
                    move_side = 2
                if get_err_T2L == 98:
                    no_feedBack = True
                    break
                elif get_err_T2L == 3:
                    move_num = 0
                    move_side = 2
            elif move_side == 2:  # 上边右移
                print('向右平移，已移动', str(move_num))
                get_err_T2R = change_column(True, False, board_cmd, q_i, lock_ser, file_address)
                move_num += 1
                if move_num >= move_times:
                    move_num = 0
                    move_side = 1
                if get_err_T2R == 98:
                    no_feedBack = True
                    break
                elif get_err_T2R == 4:
                    move_num = 0
                    move_side = 1

            # 准备清洗后退
            print('刮板向后，启动水系统')
            get_state = func_action(list_prepareBack, q_i, lock_ser, file_address)
            board_cmd = cmd_3_back + cmd_4_start
            if get_state == 98:
                no_feedBack = True
                break

            # 清洗后退
            print('清洗后退，次数', str(wash_loops))
            get_state = go_wash(False, cmd_2_washSpeed, board_cmd, q_i, lock_ser, file_address, wash_loops)
            if get_state == 98:
                no_feedBack = True
                break
            elif get_state == 1:
                print('前向误判')
            elif get_state > 90:
                print('故障')

            # 下行到下边
            print('到达底部')
            get_state = func_action(list_edgeBack, q_i, lock_ser, file_address)
            board_cmd = cmd_3_stop + cmd_4_stop
            if get_state == 98:
                no_feedBack = True
                break

            # 到下边，如果不是最后一轮，则平移
            if loop_num < (loop_time - 1):
                if move_side == 0:  # 直行，不平移
                    pass
                elif move_side == 1:  # 左移
                    print('向左平移，已移动', str(move_num))
                    get_err_B2L = change_column(False, True, board_cmd, q_i, lock_ser, file_address)
                    move_num += 1
                    if move_num >= move_times:
                        move_num = 0
                        move_side = 2
                    if get_err_B2L == 98:
                        no_feedBack = True
                        break
                    elif get_err_B2L == 3:
                        move_num = 0
                        move_side = 2
                elif move_side == 2:  # 右移
                    print('向右平移，已移动', str(move_num))
                    get_err_B2R = change_column(False, False, board_cmd, q_i, lock_ser, file_address)
                    move_num += 1
                    if move_num >= move_times:
                        move_num = 0
                        move_side = 1
                    if get_err_B2R == 98:
                        no_feedBack = True
                        break
                    elif get_err_B2R == 4:
                        move_num = 0
                        move_side = 1
            print('循环剩余', str(loop_time - loop_num))
        if no_feedBack:
            continue


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
    queue_imu = mp.Queue(maxsize=2)

    processes.append(mp.Process(target=imu_get, args=(queue_imu, lock, str_fileAddress)))
    processes.append(mp.Process(target=autocontrol_run, args=(queue_imu, lock, str_fileAddress)))

    for process in processes:
        process.daemon = True
        process.start()
    for process in processes:
        process.join()
