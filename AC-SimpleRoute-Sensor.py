import binascii
import serial
import time
import cv2
import crcmod


se = serial.Serial('/dev/ttyTHS1', 9600, timeout=0.1)
ultra_TH_F = 200  # 前向超声波阈值
ultra_TH_B = 40  # 后向超声波阈值

cmd_0_head = 'aa 01'
cmd_1_stop = '00'
cmd_1_moveFront = '01'
cmd_1_moveBack = '02'
cmd_1_rotateLeft = '03'
cmd_1_rotateRight = '04'
cmd_2_speed0 = '00'
cmd_2_speed10 = '0a'
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


# 读取主板的反馈信息
def read_feedback(rec_mess):
    fall_FL = int(rec_mess[4:6])
    fall_FR = int(rec_mess[6:8])
    fall_BL = int(rec_mess[8:10])
    fall_BR = int(rec_mess[10:12])
    ultra_F = int(rec_mess[12:14], 16)
    ultra_B = int(rec_mess[14:16], 16)
    if ((fall_FL + fall_FR + fall_BL + fall_BR) == 0) and (ultra_F < ultra_TH_F) and (ultra_B < ultra_TH_B):
        ret_state = 0
    elif ((fall_FL + fall_FR) > 0 and (fall_BL + fall_BR) == 0) or (
            ultra_F < ultra_TH_F and ultra_B > ultra_TH_B):  # 判断到上边
        ret_state = 1
    elif ((fall_FL + fall_FR) == 0 and (fall_BL + fall_BR) > 0) or (
            ultra_F > ultra_TH_F and ultra_B < ultra_TH_B):  # 判断到下边
        ret_state = 2
    else:  # 未知情况，
        ret_state = 99

    return ret_state


# 根据动作组进行执行，list的0是动作的16进制命令，1是执行多少次
def func_action(list_action):
    sensor_state = 0  # 传感器状态，99是传感器异常，98是无反馈
    loop_nofeedback = 0  # 累加无反馈次数
    threshold_nofeedback = 10  # 累计无反馈上限值
    for id_action in range(0, len(list_action), 1):
        hex_action = list_action[id_action][0]
        time_action = list_action[id_action][1]
        if time_action < 90:  # 有运行次数
            for num_action in range(0, time_action, 1):
                # se.write(hex_action)
                data_1 = binascii.b2a_hex(hex_action)
                print(data_1)
                no_feedback = True
                while no_feedback:
                    cv2.waitKey(100)
                    hex_rec = se.readline()
                    if hex_rec:
                        # 收到反馈，跳出反馈循环
                        no_feedback = False
                        str_rec = binascii.b2a_hex(hex_rec)
                        sensor_state = read_feedback(str_rec)
                        print(str_rec)
                        if sensor_state > 0:
                            return sensor_state, id_action
                    else:
                        # 重新发送命令，并累计无反馈次数
                        se.write(hex_action)
                        loop_nofeedback += 1
                        if loop_nofeedback >= threshold_nofeedback:
                            sensor_state = 98
                            return sensor_state, id_action
        else:  # 保持运行
            no_edge = True
            while no_edge:
                # se.write(hex_action)
                data_1 = binascii.b2a_hex(hex_action)
                print(data_1)
                no_feedback = True
                while no_feedback:
                    cv2.waitKey(100)
                    hex_rec = se.readline()
                    if hex_rec:
                        no_feedback = False
                        str_rec = binascii.b2a_hex(hex_rec)
                        sensor_state = read_feedback(str_rec)
                        print(str_rec)
                        if sensor_state == 1 or sensor_state == 2:
                            no_edge = False
                            break
                        elif sensor_state > 2:
                            return sensor_state, id_action
                    else:
                        # 重新发送命令，累计无反馈次数
                        se.write(hex_action)
                        loop_nofeedback += 1
                        if loop_nofeedback >= threshold_nofeedback:
                            sensor_state = 98
                            return sensor_state, id_action
    return sensor_state, 0


if __name__ == '__main__':
    # 设置清洗速度
    int_washSpeed = 10
    cmd_2_washSpeed = hex(int_washSpeed)[2:]
    if int_washSpeed > 100:
        cmd_2_washSpeed = '64'
    elif int_washSpeed < 16:
        cmd_2_washSpeed = '0' + cmd_2_washSpeed

    # 设置移动速度
    int_moveSpeed = 20
    cmd_2_moveSpeed = hex(int_moveSpeed)[2:]
    if int_moveSpeed > 100:
        cmd_2_moveSpeed = '64'
    elif int_moveSpeed < 16:
        cmd_2_moveSpeed = '0' + cmd_2_moveSpeed

    # 设置旋转速度
    int_rotatePalstance = 10
    cmd_2_rotatePalstance = hex(int_rotatePalstance)[2:]
    if int_rotatePalstance > 100:
        cmd_2_rotatePalstance = '64'
    elif int_rotatePalstance < 16:
        cmd_2_rotatePalstance = '0' + cmd_2_rotatePalstance

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
    list_Top2Left = [[hex_moveBack, 5],         # 移动下行，5次
                     [hex_rotateRight, 10],     # 右旋，10次
                     [hex_moveBack, 10],        # 移动下行，10次，斜向移动挪向左
                     [hex_rotateLeft, 10],      # 左旋，10次，转回垂直状态
                     [hex_moveFront, 10]]       # 移动上行，10次，移回接近边沿

    # 动作组-上边向右
    list_Top2Right = [[hex_moveBack, 5],        # 移动下行，5次
                      [hex_rotateLeft, 10],     # 左旋，10次
                      [hex_moveBack, 10],       # 移动下行，10次
                      [hex_rotateRight, 10],    # 右旋，10次
                      [hex_moveFront, 10]]      # 移动上行，10次

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

    # 上下次数设置
    loop_time = 2       # 上行+下行算作1次
    # 平移方向及次数设置
    move_side = 0       # 0不平移，1左移，2右移
    move_times = 0      # 设置平移次数
    move_num = 0        # 累计平移次数

    # 测试通信
    in_init = True
    while in_init:
        str_init = 'aa 01 00 00 00 00 a8'
        hex_init_send = bytes.fromhex(str_init)
        se.write(hex_init_send)
        # print(str_init)
        cv2.waitKey(100)
        hex_init_rec = se.readline()
        if hex_init_rec:
            str_init_rec = binascii.b2a_hex(hex_init_rec)
            print(str_init_rec)
            get_state = read_feedback(str_init_rec)
            if get_state == 0:
                in_init = False
                break
            else:
                print('Error, State ID' + str(get_state))

    # 建立通信，开始执行
    for loop_num in range(0, loop_time, 1):
        # 清洗上行
        get_err_WF = func_action(list_washFront)
        # 上行到边
        get_err_EF = func_action(list_edgeFront)
        # 到上边平移
        if move_side == 0:  # 直行，不平移
            pass
        elif move_side == 1:  # 左移
            get_err_T2L = func_action(list_Top2Left)
            move_num += 1
            if move_num >= move_times:
                move_num = 0
                move_side = 2
        elif move_side == 2:  # 右移
            get_err_T2R = func_action(list_Top2Right)
            move_num += 1
            if move_num >= move_times:
                move_num = 0
                move_side = 1
        # 清洗下行
        get_err_WB = func_action(list_washBack)
        # 下行到边
        get_err_EB = func_action(list_edgeBack)
        # 到下边，如果不是最后一轮，则平移
        if loop_num < (loop_time - 1):
            if move_side == 0:  # 直行，不平移
                pass
            elif move_side == 1:  # 左移
                get_err_B2L = func_action(list_Button2Left)
                move_num += 1
                if move_num >= move_times:
                    move_num = 0
                    move_side = 2
            elif move_side == 2:  # 右移
                get_err_B2R = func_action(list_Button2Right)
                move_num += 1
                if move_num >= move_times:
                    move_num = 0
                    move_side = 1
