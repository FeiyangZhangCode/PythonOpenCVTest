import binascii
import serial
import time
import cv2
import crcmod
import keyboard
import datetime

se = serial.Serial('/dev/ttyTHS1', 115200, timeout=0.09)
# se = serial.Serial('COM6', 115200, timeout=0.09)

str_fileAddress = './TestData/' + datetime.datetime.now().strftime('%H%M%S') + '-Record.txt'

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


def single_action(hex_action):
    sensor_state = 0  # 传感器状态，99是传感器异常，98是无反馈，97是运行报错
    loop_nofeedback = 0  # 累加无反馈次数
    threshold_nofeedback = 10  # 累计无反馈上限值
    no_feedback = True
    while no_feedback:
        try:
            se.write(hex_action)
            str_send = binascii.b2a_hex(hex_action).decode('utf-8')
            str_Time = datetime.datetime.now().strftime('%H:%M:%S.%f')
            file_rec = open(str_fileAddress, 'a')
            file_rec.write(str_Time + ',single,s,' + str_send + '\r\n')
            file_rec.close()
            # print(str_Time, 's', str_send)
            cv2.waitKey(40)
            hex_rec = se.readline()
            if hex_rec:
                # 收到反馈，跳出反馈循环
                no_feedback = False
                str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
                if len(str_rec) >= 12:
                    if str_rec[0:4] == 'aa02':
                        sig_fall_FL, sig_fall_FR, sig_fall_BL, sig_fall_BR = read_sensors(str_rec)
                        # print('防跌落', sig_fall_FL, sig_fall_FR, sig_fall_BL, sig_fall_BR)
                    else:
                        print('头部异常', str_rec)
                else:
                    print('长度异常', str_rec)
                file_rec = open(str_fileAddress, 'a')
                file_rec.write(str_Time + ',single,r,' + str_rec + '\r\n')
                file_rec.close()
            else:
                # 重新发送命令，并累计无反馈次数
                loop_nofeedback += 1
                print('无反馈', str(loop_nofeedback))
                if loop_nofeedback >= threshold_nofeedback:
                    sensor_state = 98
                    no_feedback = False
        except Exception as e_sa:
            print(e_sa)
            sensor_state = 97
            no_feedback = False
    return sensor_state


def multi_action(hex_action, times_action):
    sensor_state = 0  # 传感器状态，99是传感器异常，98是无反馈，97是运行报错
    loop_nofeedback = 0  # 累加无反馈次数
    threshold_nofeedback = 10  # 累计无反馈上限值
    for id_action in range(0, times_action, 1):
        no_feedback = True
        while no_feedback:
            try:
                se.write(hex_action)
                str_send = binascii.b2a_hex(hex_action).decode('utf-8')
                str_Time = datetime.datetime.now().strftime('%H:%M:%S.%f')
                file_rec = open(str_fileAddress, 'a')
                file_rec.write(str_Time + ',multi,s,' + str_send + '\r\n')
                file_rec.close()
                # print(str_Time, 's', str_send)
                cv2.waitKey(40)
                hex_rec = se.readline()
                if hex_rec:
                    # 收到反馈，跳出反馈循环
                    no_feedback = False
                    str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
                    if len(str_rec) >= 12:
                        if str_rec[0:4] == 'aa02':
                            sig_fall_FL, sig_fall_FR, sig_fall_BL, sig_fall_BR = read_sensors(str_rec)
                            # print('防跌落', sig_fall_FL, sig_fall_FR, sig_fall_BL, sig_fall_BR)
                        else:
                            print('头部异常', str_rec)
                    else:
                        print('长度异常', str_rec)
                    file_rec = open(str_fileAddress, 'a')
                    file_rec.write(str_Time + ',multi,r,' + str_rec + '\r\n')
                    file_rec.close()
                else:
                    # 重新发送命令，并累计无反馈次数
                    loop_nofeedback += 1
                    print('无反馈', str(loop_nofeedback))
                    if loop_nofeedback >= threshold_nofeedback:
                        sensor_state = 98
                        no_feedback = False
            except Exception as e_sa:
                print(e_sa)
                sensor_state = 97
                no_feedback = False
    return sensor_state


# 根据动作组进行执行，list的0是动作的16进制命令，1是执行多少次
def func_action(list_action):
    sensor_state = 0  # 传感器状态，99是传感器异常，98是无反馈，97是运行错误
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
                    file_rec = open(str_fileAddress, 'a')
                    file_rec.write(str_Time + ',auto,s,' + str_send + '\r\n')
                    file_rec.close()
                    # print(str_Time, 's', str_send)
                    cv2.waitKey(60)
                    hex_rec = se.readline()
                    if hex_rec:
                        # 收到反馈，跳出反馈循环
                        no_feedback = False
                        str_rec = binascii.b2a_hex(hex_rec).decode('utf-8')
                        if len(str_rec) >= 12:
                            if str_rec[0:4] == 'aa02':
                                sig_fall_FL, sig_fall_FR, sig_fall_BL, sig_fall_BR = read_sensors(str_rec)
                                # print('防跌落', sig_fall_FL, sig_fall_FR, sig_fall_BL, sig_fall_BR)
                            else:
                                print('头部异常', str_rec)
                        else:
                            print('长度异常', str_rec)
                        file_rec = open(str_fileAddress, 'a')
                        file_rec.write(str_Time + ',auto,r,' + str_rec + '\r\n')
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
                    sensor_state = 97
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
    cmd_2_washSpeed = trans_speed('50')
    # 设置移动速度
    cmd_2_moveSpeed = trans_speed('100')
    # 设置旋转速度
    cmd_2_rotatePalstance = trans_speed('50')

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
    # 动作组-清洗前进
    list_washFront = [[hex_boardFront, 15],  # 刮板向前，15次约3秒
                      [hex_sprayFront, 1],  # 启动水系统，1次
                      [hex_washFront, 20]]  # 保持清洗向前，测试阶段用20次

    # 清洗前进，移动停止
    hex_stopFront = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_front + cmd_4_start)
    # 刮板向前，关闭水系统
    hex_sprayStop_boardFront = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_front + cmd_4_stop)
    # 刮板向后
    hex_boardBack = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_back + cmd_4_stop)
    # 动作组-清洗向前到边
    list_edgeFront = [[hex_stopFront, 2],  # 移动停止，2次
                      [hex_sprayStop_boardFront, 1],  # 关闭水系统，1次
                      [hex_boardBack, 7],  # 刮板由前，改为向后，7次约1.4秒，预计到中间位置
                      [hex_allStop, 1]]  # 全停止，进行下一指令

    # 刮板向下，启动水系统
    hex_sprayBack = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_back + cmd_4_start)
    # 清洗下行
    hex_washBack = set_order(cmd_0_head + cmd_1_moveBack + cmd_2_washSpeed + cmd_3_back + cmd_4_start)
    # 动作组-清洗向后
    list_washBack = [[hex_boardBack, 15],  # 刮板后，15次约3秒
                     [hex_sprayBack, 1],  # 启动水系统，1次
                     [hex_washBack, 20]]  # 保持清洗向后，暂用20次

    # 清洗下行，移动停止
    hex_stopBack = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_back + cmd_4_start)
    # 刮板向下，关闭水系统
    hex_sprayStop_boardBack = set_order(cmd_0_head + cmd_1_stop + cmd_2_speed0 + cmd_3_back + cmd_4_stop)
    # 动作组-清洗下行到边
    list_edgeBack = [[hex_stopBack, 2],  # 移动停止，2次
                     [hex_sprayStop_boardBack, 1],  # 关闭水系统，1次
                     [hex_boardFront, 7],  # 刮板由前，改为向后，7次与1.4秒，预计到中间位置
                     [hex_allStop, 1]]  # 全停止，进行下一指令

    # 移动向后
    hex_moveBack = set_order(cmd_0_head + cmd_1_moveBack + cmd_2_moveSpeed + cmd_3_stop + cmd_4_stop)
    # 右旋
    hex_rotateRight = set_order(cmd_0_head + cmd_1_rotateRight + cmd_2_rotatePalstance + cmd_3_stop + cmd_4_stop)
    # 左旋
    hex_rotateLeft = set_order(cmd_0_head + cmd_1_rotateLeft + cmd_2_rotatePalstance + cmd_3_stop + cmd_4_stop)
    # 移动向前
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

    # 无反馈重新循环标志位
    no_feedBack = False
    while True:
        # 测试通信
        in_init = True
        while in_init:
            str_init = 'aa 01 00 00 00 00 a8'
            hex_init_send = bytes.fromhex(str_init)
            str_Time = datetime.datetime.now().strftime('%H:%M:%S.%f')
            try:
                se.write(hex_init_send)
                print(str_Time, 'i', str_init)
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
                            print('头部异常')
                    else:
                        print('长度异常')
            except Exception as e:
                print(e)
        print('进入动作测试')

        # 单个动作测试
        is_oneAction = True
        while is_oneAction:
            get_one_err = 0
            get_stop_err = 0
            # 输入单步命令
            if keyboard.is_pressed('w') or keyboard.is_pressed('8'):  # 向前移动，刮板和水系统停止
                print('前进')
                get_one_err = single_action(hex_moveFront)
            elif keyboard.is_pressed('s') or keyboard.is_pressed('2'):  # 向后移动，刮板和水系统停止
                print('后退')
                get_one_err = single_action(hex_moveBack)
            elif keyboard.is_pressed('a') or keyboard.is_pressed('4'):  # 向左旋转，刮板和水系统停止
                print('左旋')
                get_one_err = single_action(hex_rotateLeft)
            elif keyboard.is_pressed('d') or keyboard.is_pressed('6'):  # 向右旋转，刮板和水系统停止
                print('右旋')
                get_one_err = single_action(hex_rotateRight)
            elif keyboard.is_pressed('x') or keyboard.is_pressed('1'):  # 刮板向前，停止移动，水系统停止
                print('刮板向前')
                get_one_err = single_action(hex_boardFront)
            elif keyboard.is_pressed('c') or keyboard.is_pressed('3'):  # 刮板向后，停止移动，水系统停止
                print('刮板向后')
                get_one_err = single_action(hex_boardBack)
            elif keyboard.is_pressed('z') or keyboard.is_pressed('5'):  # 启动水系统，停止移动，抬起刮板
                print('水系统启动')
                get_one_err = single_action(hex_sprayStart)
            elif keyboard.is_pressed('q'):  # 结束运行，全部停止
                print('结束运行')
                get_stop_err = single_action(hex_allStop)
                is_oneAction = False
            else:  # 无输入，停止移动，刮板和水系统停止
                get_stop_err = single_action(hex_allStop)
            # 无反馈退出
            if get_one_err == 98 or get_stop_err == 98:
                print('无反馈，结束运行')
                is_oneAction = False
                no_feedBack = True
        if no_feedBack:
            continue

        # 开始动作测试
        print('进入动作测试')
        get_err_at, stop_id = func_action(list_ActionTest)
        if get_err_at == 98:
            print('无反馈，结束运行')
            is_oneAction = False
            no_feedBack = True
            continue

        # 进入运行状态测量
        print('进入运行状态测量')
        is_oneAction = True
        while is_oneAction:
            get_one_err = 0
            get_two_err = 0
            get_stop_err = 0
            # 输入单步命令
            if keyboard.is_pressed('w'):  # 向前移动，刮板和水系统停止
                print('前进1秒')
                get_one_err = multi_action(hex_moveFront, 5)
            elif keyboard.is_pressed('s'):  # 向后移动，刮板和水系统停止
                print('后退1秒')
                get_one_err = multi_action(hex_moveBack, 5)
            elif keyboard.is_pressed('a'):  # 向左旋转，刮板和水系统停止
                print('左旋1秒')
                get_one_err = multi_action(hex_rotateLeft, 5)
            elif keyboard.is_pressed('d'):  # 向右旋转，刮板和水系统停止
                print('右旋1秒')
                get_one_err = multi_action(hex_rotateRight, 5)
            elif keyboard.is_pressed('x'):  # 刮板向前，停止移动，水系统停止
                print('刮板向前3秒')
                get_one_err = multi_action(hex_boardFront, 15)
            elif keyboard.is_pressed('c'):  # 刮板向后，停止移动，水系统停止
                print('刮板向后3秒')
                get_one_err = multi_action(hex_boardBack, 15)
            elif keyboard.is_pressed('z'):  # 启动水系统，停止移动，抬起刮板
                print('水系统运行1秒')
                get_one_err = multi_action(hex_sprayStart, 5)
            elif keyboard.is_pressed('i'):  # 向前移动，刮板和水系统停止
                print('前进2秒')
                get_one_err = multi_action(hex_moveFront, 10)
            elif keyboard.is_pressed('k'):  # 向后移动，刮板和水系统停止
                print('后退2秒')
                get_one_err = multi_action(hex_moveBack, 10)
            elif keyboard.is_pressed('j'):  # 向左旋转，刮板和水系统停止
                print('左旋2秒')
                get_one_err = multi_action(hex_rotateLeft, 10)
            elif keyboard.is_pressed('l'):  # 向右旋转，刮板和水系统停止
                print('右旋2秒')
                get_one_err = multi_action(hex_rotateRight, 10)
            elif keyboard.is_pressed('m'):  # 刮板向前然后返回中位，停止移动，水系统停止
                print('刮板向前，返回中位')
                get_one_err = multi_action(hex_boardFront, 15)
                get_two_err = multi_action(hex_boardBack, 7)
            elif keyboard.is_pressed('n'):  # 刮板向后，停止移动，水系统停止
                print('刮板向后，返回中位')
                get_one_err = multi_action(hex_boardBack, 15)
                get_two_err = multi_action(hex_boardFront, 7)
            elif keyboard.is_pressed('q'):  # 结束运行，全部停止
                print('结束运行')
                get_stop_err = single_action(hex_allStop)
                is_oneAction = False
            else:  # 无输入，停止移动，刮板和水系统停止
                get_stop_err = single_action(hex_allStop)
            # 无反馈退出
            if get_one_err == 98 or get_stop_err == 98 or get_two_err == 98:
                print('无反馈，结束运行')
                is_oneAction = False
                no_feedBack = True
        if no_feedBack:
            continue

        # 开始自动运行测试
        print('进入运行测试')
        # 清洗前进
        print('向前清洗状态，前进4秒')
        get_err_WF = func_action(list_washFront)
        if get_err_WF == 98:
            print('无反馈，结束运行')
            is_oneAction = False
            no_feedBack = True
            continue

        # 前进到边
        print('到达上边，准备平移')
        get_err_EF = func_action(list_edgeFront)
        if get_err_EF == 98:
            print('无反馈，结束运行')
            is_oneAction = False
            no_feedBack = True
            continue

        # 上边向左
        print('上边平移向左')
        get_err_T2L = func_action(list_Top2Left)
        if get_err_T2L == 98:
            print('无反馈，结束运行')
            is_oneAction = False
            no_feedBack = True
            continue

        # 上边向右
        print('上边平移向右')
        get_err_T2R = func_action(list_Top2Right)
        if get_err_T2R == 98:
            print('无反馈，结束运行')
            is_oneAction = False
            no_feedBack = True
            continue

        # 清洗向后
        print('向后清洗状态，后退4秒')
        get_err_WB = func_action(list_washBack)
        if get_err_WB == 98:
            print('无反馈，结束运行')
            is_oneAction = False
            no_feedBack = True
            continue

        # 向后到边
        print('到达下边，准备平移')
        get_err_EB = func_action(list_edgeBack)
        if get_err_EB == 98:
            print('无反馈，结束运行')
            is_oneAction = False
            no_feedBack = True
            continue

        # 下边向左
        print('下边平移向左')
        get_err_B2L = func_action(list_Button2Left)
        if get_err_B2L == 98:
            print('无反馈，结束运行')
            is_oneAction = False
            no_feedBack = True
            continue

        # 下边向右
        print('下边平移向右')
        get_err_B2R = func_action(list_Button2Right)
        if get_err_B2R == 98:
            print('无反馈，结束运行')
            is_oneAction = False
            no_feedBack = True
            continue

        print('测试结束')
