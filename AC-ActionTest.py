import binascii
import serial
import time
import cv2
import crcmod
import keyboard
import datetime

se = serial.Serial('/dev/ttyTHS1', 115200, timeout=0.09)
# se = serial.Serial('COM6', 115200, timeout=0.09)

ultra_TH_F = 200  # 前向超声波阈值
ultra_TH_B = 40  # 后向超声波阈值

cmd_0_head = 'aa 01'
cmd_1_stop = '00'
cmd_1_moveFront = '01'
cmd_1_moveBack = '02'
cmd_1_rotateLeft = '03'
cmd_1_rotateRight = '04'
cmd_2_speed0 = '00'
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


# 读取主板的反馈信息，返回判断后的状态
def read_feedback(rec_mess):
    fall_FL = int(rec_mess[4:6])
    fall_FR = int(rec_mess[6:8])
    fall_BL = int(rec_mess[8:10])
    fall_BR = int(rec_mess[10:12])
    if (fall_FL + fall_FR + fall_BL + fall_BR) == 0:
        ret_state = 0
    elif (fall_FL + fall_FR) > 0 and (fall_BL + fall_BR) == 0:  # 判断到上边
        ret_state = 1
    elif (fall_FL + fall_FR) == 0 and (fall_BL + fall_BR) > 0:  # 判断到下边
        ret_state = 2
    else:  # 未知情况，
        ret_state = 99

    return ret_state


# 读取主板的反馈信息，返回各个传感器数据
def read_sensors(rec_mess):
    fall_FL = str(int(rec_mess[4:6]))
    fall_FR = str(int(rec_mess[6:8]))
    fall_BL = str(int(rec_mess[8:10]))
    fall_BR = str(int(rec_mess[10:12]))

    return fall_FL, fall_FR, fall_BL, fall_BR


def single_action(hex_action, single_time):
    sensor_state = 0  # 传感器状态，99是传感器异常，98是无反馈
    for single_num in range(0, single_time, 1):
        loop_nofeedback = 0  # 累加无反馈次数
        threshold_nofeedback = 10  # 累计无反馈上限值
        se.write(hex_action)
        data_1 = binascii.b2a_hex(hex_action)
        print(data_1)
        no_feedback = True
        while no_feedback:
            cv2.waitKey(40)
            hex_rec = se.readline()
            if hex_rec:
                # 收到反馈，跳出反馈循环
                no_feedback = False
                str_rec = binascii.b2a_hex(hex_rec)
                sig_fall_FL, sig_fall_FR, sig_fall_BL, sig_fall_BR = read_sensors(str_rec)
                print('防跌落', sig_fall_FL, sig_fall_FR, sig_fall_BL, sig_fall_BR)
            else:
                # 重新发送命令，并累计无反馈次数
                se.write(hex_action)
                loop_nofeedback += 1
                print('无反馈', loop_nofeedback)
                if loop_nofeedback >= threshold_nofeedback:
                    sensor_state = 98
                    return sensor_state
    return sensor_state


# 根据动作组进行执行，list的0是动作的16进制命令，1是执行多少次
def func_action(list_action):
    sensor_state = 0  # 传感器状态，99是传感器异常，98是无反馈
    for id_action in range(0, len(list_action), 1):
        loop_nofeedback = 0  # 累加无反馈次数
        threshold_nofeedback = 10  # 累计无反馈上限值
        hex_action = list_action[id_action][0]
        time_action = list_action[id_action][1]
        for num_action in range(0, time_action, 1):
            se.write(hex_action)
            data_1 = binascii.b2a_hex(hex_action)
            str_Time = datetime.datetime.now().strftime('%H:%M:%S.%f')
            # print(str_Time)
            # print(data_1)
            no_feedback = True
            while no_feedback:
                cv2.waitKey(60)
                hex_rec = se.readline()
                if hex_rec:
                    # 收到反馈，跳出反馈循环
                    no_feedback = False
                    str_rec = binascii.b2a_hex(hex_rec)
                    sensor_state = read_feedback(str_rec)
                    print(str_rec)
                    # if sensor_state > 0:
                    #     return sensor_state, id_action
                else:
                    # 重新发送命令，并累计无反馈次数
                    se.write(hex_action)
                    loop_nofeedback += 1
                    if loop_nofeedback >= threshold_nofeedback:
                        sensor_state = 98
                        return sensor_state, id_action
    return sensor_state, 0


if __name__ == '__main__':
    # 设置上下次数
    loop_time = 2  # 上行+下行算作1次
    # 设置平移方向及次数
    move_side = 0  # 0不平移，1左移，2右移
    move_times = 0  # 设置平移次数
    move_num = 0  # 累计平移次数

    # 设置清洗速度
    cmd_2_washSpeed = trans_speed('10')
    # 设置移动速度
    cmd_2_moveSpeed = trans_speed('20')
    # 设置旋转速度
    cmd_2_rotatePalstance = trans_speed('10')

    # 刮板向上
    hex_boardFront = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_front + cmd_4_stop)
    # 刮板向上，启动水系统
    hex_sprayFront = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_front + cmd_4_start)
    # 清洗上行
    hex_washFront = set_order(cmd_0_head + cmd_1_moveFront + cmd_2_washSpeed + cmd_3_front + cmd_4_start)
    # 动作组-清洗上行
    list_washFront = [[hex_boardFront, 10],  # 刮板向上，10次约2秒
                      [hex_sprayFront, 1],  # 启动水系统，1次
                      [hex_washFront, 20]]  # 保持移动向上，测试阶段用20次

    # 清洗上行，移动停止
    hex_stopFront = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_front + cmd_4_start)
    # 刮板向上，关闭水系统
    hex_sprayStop_boardFront = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_front + cmd_4_stop)
    # 刮板抬起
    hex_boardStop = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_stop + cmd_4_stop)
    # 动作组-清洗上行到边
    list_edgeFront = [[hex_stopFront, 2],  # 移动停止，2次
                      [hex_sprayStop_boardFront, 1],  # 关闭水系统，1次
                      [hex_boardStop, 10]]  # 刮板停止，2秒

    # 刮板向下
    hex_boardBack = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_back + cmd_4_stop)
    # 刮板向下，启动水系统
    hex_sprayBack = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_back + cmd_4_start)
    # 清洗下行
    hex_washBack = set_order(cmd_0_head + cmd_1_moveBack + cmd_2_washSpeed + cmd_3_back + cmd_4_start)
    # 动作组-清洗下行
    list_washBack = [[hex_boardBack, 10],  # 刮板向下，10次约2秒
                     [hex_sprayBack, 1],  # 启动水系统，1次
                     [hex_washBack, 20]]  # 保持移动向下，暂用20次

    # 清洗下行，移动停止
    hex_stopBack = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_back + cmd_4_start)
    # 刮板向下，关闭水系统
    hex_sprayStop_boardBack = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_back + cmd_4_stop)
    # 动作组-清洗下行到边
    list_edgeBack = [[hex_stopBack, 2],  # 移动停止，2次
                     [hex_sprayStop_boardBack, 1],  # 关闭水系统，1次
                     [hex_boardStop, 10]]  # 刮板停止，2秒

    # 移动下行
    hex_moveBack = set_order(cmd_0_head + cmd_1_moveBack + cmd_2_moveSpeed + cmd_3_stop + cmd_4_stop)
    # 右旋
    hex_rotateRight = set_order(cmd_0_head + cmd_1_rotateRight + cmd_2_rotatePalstance + cmd_3_stop + cmd_4_stop)
    # 左旋
    hex_rotateLeft = set_order(cmd_0_head + cmd_1_rotateLeft + cmd_2_rotatePalstance + cmd_3_stop + cmd_4_stop)
    # 移动上行
    hex_moveFront = set_order(cmd_0_head + cmd_1_moveFront + cmd_2_moveSpeed + cmd_3_stop + cmd_4_stop)

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

    # 启动水系统
    hex_sprayStart = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_stop + cmd_4_start)
    # 全停止
    hex_allStop = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_stop + cmd_4_stop)

    # 动作组-动作测试
    list_ActionTest = [[hex_moveFront, 10],
                       [hex_allStop, 20],
                       [hex_moveBack, 10],
                       [hex_allStop, 20],
                       [hex_rotateLeft, 10],
                       [hex_allStop, 20],
                       [hex_rotateRight, 20],
                       [hex_allStop, 20],
                       [hex_rotateLeft, 10],
                       [hex_allStop, 20],
                       [hex_boardFront, 20],
                       [hex_boardBack, 20],
                       [hex_allStop, 20],
                       [hex_sprayStart, 10],
                       [hex_allStop, 20]]

    # 无反馈重新循环标志位
    no_feedBack = False
    while True:
        # 测试通信
        in_init = True
        while in_init:
            str_init = 'aa 01 00 00 00 00 a8'
            hex_init_send = bytes.fromhex(str_init)
            se.write(hex_init_send)
            str_Time = datetime.datetime.now().strftime('%H:%M:%S.%f')
            print(str_Time)
            print(str_init)
            cv2.waitKey(40)
            hex_init_rec = se.readline()
            if hex_init_rec:
                str_init_rec = binascii.b2a_hex(hex_init_rec)
                if len(str_init_rec) >= 12:
                    if str_init_rec[0:4].decode('utf-8') == 'aa02':
                        str_fall_FL, str_fall_FR, str_fall_BL, str_fall_BR = read_sensors(str_init_rec)
                        print('防跌落', str_fall_FL, str_fall_FR, str_fall_BL, str_fall_BR)
                        in_init = False
                        no_feedBack = False
                        print('通信正常')
                    else:
                        print('数据包头部异常')
                else:
                    print('数据包长度异常')
        print('进入动作测试')

        # 单个动作测试
        is_oneAction = True
        while is_oneAction:
            str_Time = datetime.datetime.now().strftime('%H:%M:%S.%f')
            print(str_Time)
            get_one_err = 0
            get_stop_err = 0
            # 输入单步命令
            if keyboard.is_pressed('w') or keyboard.is_pressed('8'):  # 向前移动，刮板和水系统停止
                get_one_err = single_action(hex_moveFront, 1)
            elif keyboard.is_pressed('s') or keyboard.is_pressed('2'):  # 向后移动，刮板和水系统停止
                get_one_err = single_action(hex_moveBack, 1)
            elif keyboard.is_pressed('a') or keyboard.is_pressed('4'):  # 向左旋转，刮板和水系统停止
                get_one_err = single_action(hex_rotateLeft, 1)
            elif keyboard.is_pressed('d') or keyboard.is_pressed('6'):  # 向右旋转，刮板和水系统停止
                get_one_err = single_action(hex_rotateRight, 1)
            elif keyboard.is_pressed('x') or keyboard.is_pressed('1'):  # 刮板向前，停止移动，水系统停止
                get_one_err = single_action(hex_boardFront, 1)
            elif keyboard.is_pressed('c') or keyboard.is_pressed('3'):  # 刮板向后，停止移动，水系统停止
                get_one_err = single_action(hex_boardBack, 1)
            elif keyboard.is_pressed('z') or keyboard.is_pressed('5'):  # 启动水系统，停止移动，抬起刮板
                get_one_err = single_action(hex_sprayStart, 1)
            elif keyboard.is_pressed('q'):  # 结束运行，全部停止
                print('结束运行')
                get_stop_err = single_action(hex_allStop, 1)
                is_oneAction = False
            else:  # 无输入，停止移动，刮板和水系统停止
                get_stop_err = single_action(hex_allStop, 1)
            # 无反馈退出
            if get_one_err == 98 or get_stop_err == 98:
                print('无反馈，结束运行')
                is_oneAction = False
                no_feedBack = True
        if no_feedBack:
            continue

        # 开始自动运行测试
        print('进入自动动作测试')
        get_err_at, stop_id = func_action(list_ActionTest)
        if get_err_at == 98:
            print('无反馈，结束运行')
            is_oneAction = False
            no_feedBack = True
        if no_feedBack:
            continue

        print('测试结束')




        # # 单步执行，测试各个动作
        # temp_c1 = cmd_1_stop
        # temp_c2 = cmd_2_speed0
        # temp_c3 = cmd_3_stop
        # temp_c4 = cmd_4_stop
        # is_SingleLoop = True
        # while is_SingleLoop:
        #     # 输入单步命令
        #     kb_action = input('输入动作')
        #     kb_time = int(input('输入次数'))
        #     # kb_time = 5
        #     if kb_action == 'w' or '8':  # 向前移动，刮板和水系统不变
        #         kb_speed = input('输入速度')
        #         cmd_2_inputSpeed = trans_speed(kb_speed)
        #         temp_c1 = cmd_1_moveFront
        #         temp_c2 = cmd_2_inputSpeed
        #     elif kb_action == 's' or '2':  # 向后移动，刮板和水系统不变
        #         kb_speed = input('输入速度')
        #         cmd_2_inputSpeed = trans_speed(kb_speed)
        #         temp_c1 = cmd_1_moveBack
        #         temp_c2 = cmd_2_inputSpeed
        #     elif kb_action == 'a' or '4':  # 向左旋转，刮板和水系统不变
        #         kb_speed = input('输入速度')
        #         cmd_2_inputSpeed = trans_speed(kb_speed)
        #         temp_c1 = cmd_1_rotateLeft
        #         temp_c2 = cmd_2_inputSpeed
        #     elif kb_action == 'd' or '6':  # 向右旋转，刮板和水系统不变
        #         kb_speed = input('输入速度')
        #         cmd_2_inputSpeed = trans_speed(kb_speed)
        #         temp_c1 = cmd_1_rotateLeft
        #         temp_c2 = cmd_2_inputSpeed
        #     elif kb_action == 'z' or '5':  # 抬起刮板，停止移动，水系统不变
        #         temp_c1 = cmd_1_stop
        #         temp_c2 = cmd_2_speed0
        #         temp_c3 = cmd_3_stop
        #     elif kb_action == 'x' or '1':  # 刮板向前，停止移动，水系统不变
        #         temp_c1 = cmd_1_stop
        #         temp_c2 = cmd_2_speed0
        #         temp_c3 = cmd_3_front
        #     elif kb_action == 'c' or '3':  # 刮板向后，停止移动，水系统不变
        #         temp_c1 = cmd_1_stop
        #         temp_c2 = cmd_2_speed0
        #         temp_c3 = cmd_3_back
        #     elif kb_action == 'e' or '7':  # 关闭水系统，停止移动，刮板不变
        #         temp_c1 = cmd_1_stop
        #         temp_c2 = cmd_2_speed0
        #         temp_c4 = cmd_4_stop
        #     elif kb_action == 'r' or '9':  # 启动水系统，停止移动，刮板不变
        #         temp_c1 = cmd_1_stop
        #         temp_c2 = cmd_2_speed0
        #         temp_c4 = cmd_4_start
        #     elif kb_action == 'q':  # 结束运行，全部停止
        #         print('结束运行')
        #         is_SingleLoop = False
        #         temp_c1 = cmd_1_stop
        #         temp_c2 = cmd_2_speed0
        #         temp_c3 = cmd_3_stop
        #         temp_c4 = cmd_4_stop
        #     else:  # 输入错误，停止移动，刮板和水系统不变
        #         print('输入错误')
        #         temp_c1 = cmd_1_stop
        #         temp_c2 = cmd_2_speed0
        #     # 发送单步命令
        #     hex_temp_order = set_order(cmd_0_head + temp_c1 + temp_c2 + temp_c3 + temp_c4)
        #     get_single_err = single_action(hex_temp_order, kb_time)
        #     # 执行单步后移动停止
        #     hex_temp_stop = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + temp_c3 + temp_c4)
        #     get_stop_err = single_action(hex_temp_stop, 1)
        #     # 无反馈退出
        #     if get_single_err == 98 or get_stop_err == 98:
        #         print('无反馈，结束运行')
        #         is_SingleLoop = False
