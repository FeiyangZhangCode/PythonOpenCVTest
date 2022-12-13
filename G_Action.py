import multiprocessing
import numpy as np
import CVFunc
import signal
import math
import xlwt
import binascii
import serial
import time
import cv2
import crcmod
import keyboard
import datetime
import multiprocessing as mp
import os

# 机身尺寸、光伏板尺寸
vehicle_left = 118
vehicle_right = 127
vehicle_front = 117
vehicle_width = vehicle_left + vehicle_right
panel_width = 630

# 激光测距阈值
laser_threshold = 17

# 清洗速度及校正阈值
int_washSpeed = 50
wash_thre_yaw = 0.5
correct_thre_yaw = 30.0
wash_thre_deviation = 30.0
correct_thre_deviation = 120.0

# 侧边全局缓存
rec_side_left = [0.0]
rec_side_right = [0.0]

cmd_0_head = 'aa 01'
cmd_13_stop = '00'
cmd_13_front = '01'
cmd_13_back = '02'
cmd_24_stop = '00'
cmd_24_normal = CVFunc.trans_speed('50')
cmd_24_slow = CVFunc.trans_speed('30')
cmd_24_fast = CVFunc.trans_speed('70')
cmd_5_stop = '00'
cmd_5_front = '01'
cmd_5_back = '02'
cmd_6_stop = '00'
cmd_6_start = '01'

# 移动向前
hex_Front = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_normal + cmd_13_front + cmd_24_normal + cmd_5_stop + cmd_6_stop)
# 移动向后
hex_Back = CVFunc.set_order(cmd_0_head + cmd_13_back + cmd_24_normal + cmd_13_back + cmd_24_normal + cmd_5_stop + cmd_6_stop)
# 右旋
hex_rotateRight = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_slow + cmd_13_back + cmd_24_slow + cmd_5_stop + cmd_6_stop)
# 左旋
hex_rotateLeft = CVFunc.set_order(cmd_0_head + cmd_13_back + cmd_24_slow + cmd_13_front + cmd_24_slow + cmd_5_stop + cmd_6_stop)
# 右前转
hex_FrontRight = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_fast + cmd_13_front + cmd_24_slow + cmd_5_stop + cmd_6_stop)
# 左前转
hex_FrontLeft = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_slow + cmd_13_front + cmd_24_fast + cmd_5_stop + cmd_6_stop)
# 全停止
hex_allStop = CVFunc.set_order(cmd_0_head + cmd_13_stop + cmd_24_stop + cmd_13_stop + cmd_24_stop + cmd_5_stop + cmd_6_stop)




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
        ret_list = [int_fall_FL, int_fall_FR, int_fall_BL, int_fall_BR, laser_F, laser_B]
    else:
        is_normal = False
        ret_list = [0]
    return is_normal, ret_list


def control_communication(hex_order, q_v2a, q_m2a, q_a2m, q_c, q_ci):
    # 发送命令
    q_a2m.put(hex_order)
    q_a2m.get() if q_a2m.qsize() > 1 else time.sleep(0.001)
    is_read, ret_list = read_message(hex_order)
    if is_read:
        q_c.put(ret_list)
        q_c.get() if q_c.qsize() > 1 else time.sleep(0.001)
        q_ci.put(ret_list)
        q_ci.get() if q_ci.qsize() > 1 else time.sleep(0.001)
    # 等待反馈
    while q_m2a.empty():
        cv2.waitKey(10)
    # 读取数据
    if not q_v2a.empty():
        cam_yaw, dis_left, dis_right = q_v2a.get()
    else:
        cam_yaw = dis_left = dis_right = -999.9

    sensor_list = q_m2a.get()
    if sensor_list[0] < 90:
        get_state = 0
        dis_laser = sensor_list[4]
        imu_roll = sensor_list[5]
        imu_pitch = sensor_list[6]
        imu_yaw = sensor_list[7]
    else:
        get_state = sensor_list[0]
        dis_laser = sensor_list[1]
        imu_roll = sensor_list[2]
        imu_pitch = sensor_list[3]
        imu_yaw = sensor_list[4]
    return get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw

# 校正偏航角
def correct_yaw(now_yaw, target_yaw, cmd_56, q_v2a, q_m2a, q_a2m, q_c, q_ci, file_address):
    global rec_side_left, rec_side_right

    get_state = 0
    diff_cam_imu = 0.0
    rot_yaw = now_yaw - target_yaw
    target_count = 0

    # 等待视觉测距稳定
    for i in range(0, 3, 1):
        get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
        if get_state > 90:
            get_error, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return get_state
        if dis_left != -999.9:
            rec_side_left.append(dis_left)
        if dis_right != -999.9:
            rec_side_right.append(dis_right)
        if cam_yaw != -999.9:
            diff_cam_imu = cam_yaw - imu_yaw

    while target_count <= 5:
        # 键盘输入急停
        if keyboard.is_pressed('b'):
            get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return 96

        # 设置旋转动作
        if target_count == 0:
            # 根据偏航角设定初始旋转速度
            if abs(rot_yaw) > 20:
                correct_speed = 10
            elif abs(rot_yaw) > 10:
                correct_speed = 5
            else:
                correct_speed = 2
            cmd_24_corSpeed = CVFunc.trans_speed(str(correct_speed))
            if rot_yaw < 0:     # yaw负数，偏向左，向右旋转
                hex_correctYaw = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_corSpeed + cmd_13_back + cmd_24_corSpeed + cmd_56)
            else:       # yaw正数，偏向右，向左旋转
                hex_correctYaw = CVFunc.set_order(cmd_0_head + cmd_13_back + cmd_24_corSpeed + cmd_13_front + cmd_24_corSpeed + cmd_56)
        else:
            hex_correctYaw = hex_allStop

        # 发送动作命令
        get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(hex_correctYaw, q_v2a, q_m2a, q_a2m, q_c, q_ci)
        if get_state > 90:
            get_error, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return get_state
        if dis_left != -999.9:
            rec_side_left.append(dis_left)
        if dis_right != -999.9:
            rec_side_right.append(dis_right)
        if cam_yaw != -999.9:
            rot_yaw = cam_yaw - target_yaw
        else:
            rot_yaw = imu_yaw + diff_cam_imu - target_yaw

        if abs(rot_yaw) <= 1.0:  # 如果与目标角度相差不大于1，则认为已完成，
            target_count += 1
        else:
            target_count = 0

        # 键盘输入急停
        if keyboard.is_pressed('b'):
            get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return 96

    return get_state


# 校正偏移距离
def correct_deviation(dis_dev, target_l, target_r, cmd_56, q_v2a, q_m2a, q_a2m, q_c, q_ci, file_address):
    global rec_side_left, rec_side_right
    is_right = False
    imu_yaw = 0.0
    side_l = -999.9
    side_r = -999.9
    cam_yaw = -999.9
    now_yaw = 0.0
    move_dis = 0.0

    # 1.更新调整距离
    num_timer = 0
    try_again = True
    while num_timer < 3 and try_again:
        num_timer += 1
        # 停止
        get_state, cam_yaw, side_l, side_r, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
        if get_state > 90:
            return get_state
        if cam_yaw != -999.9:
            now_yaw = cam_yaw

        # 判断平移距离
        if target_l <= target_r:
            if side_l != -999.9:
                move_dis = side_l - target_l
                dis_dev = move_dis
                try_again = False
            elif side_r != -999.9:
                move_dis = target_r - side_r
            else:
                move_dis = dis_dev
        else:
            if side_r != -999.9:
                move_dis = target_r - side_r
                dis_dev = move_dis
                try_again = False
            elif side_l != -999.9:
                move_dis = side_l - target_l
            else:
                move_dis = dis_dev

    # 2.根据偏航距离，判断左旋还是右旋
    if move_dis < 0:
        is_right = True
        rot_angle = 30.0
    else:
        is_right = False
        rot_angle = -30.0
    get_correct_err = correct_yaw(now_yaw, rot_angle, cmd_56, q_v2a, q_m2a, q_a2m, q_c, q_ci, file_address)
    if get_correct_err > 90:
        return get_correct_err

    # 3.直线移动
    num_error_dis = 0
    target_count = 0
    dis_dev = abs(dis_dev)
    move_dis = abs(move_dis)
    while target_count < 5 and num_error_dis < 10:
        print('测量距离' + str(int(move_dis)) + '  估算距离' + str(int(dis_dev)))
        # 键盘输入急停
        if keyboard.is_pressed('b'):
            get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return 96

        # 根据剩余距离设定速度和前进后退
        if target_count == 0:
            if -50.0 < move_dis < -10.0:
                cmd_24_corSpeed = CVFunc.trans_speed('10')
                hex_correctDis = CVFunc.set_order(cmd_0_head + cmd_13_back + cmd_24_corSpeed + cmd_13_back + cmd_24_corSpeed + cmd_56)
                int_move = 3
                num_error_dis = 0
            elif move_dis >= 10.0:
                if move_dis > 50.0:
                    cmd_24_corSpeed = CVFunc.trans_speed('20')
                    int_move = 2
                else:
                    cmd_24_corSpeed = CVFunc.trans_speed('10')
                    int_move = 1
                hex_correctDis = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_corSpeed + cmd_13_front + cmd_24_corSpeed + cmd_56)
                num_error_dis = 0
            else:
                hex_correctDis = hex_allStop
                int_move = 0
                num_error_dis += 1
        else:
            hex_correctDis = hex_allStop
            int_move = 0
        # 发送动作命令
        get_state, cam_yaw, side_l, side_r, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(hex_correctDis, q_v2a, q_m2a, q_a2m, q_c, q_ci)
        if get_state > 90:
            get_error, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return get_state

        # 估算的剩余行驶距离
        if int_move == 1:
            dis_dev = dis_dev - 2
        elif int_move == 2:
            dis_dev = dis_dev - 4
        elif int_move == 3:
            dis_dev = dis_dev + 2
        # 实际测得的剩余行驶距离，相差过大则直接使用估算距离
        if is_right:
            if side_r != -999.9:
                temp_dis = side_r - target_r
                if abs(temp_dis - dis_dev) < 50.0:
                    move_dis = temp_dis
                    dis_dev = move_dis
                    num_error_dis = 0
                else:
                    num_error_dis += 1
                    move_dis = dis_dev
            else:
                if int_move == 1:
                    move_dis = move_dis - 2
                elif int_move == 2:
                    move_dis = move_dis - 4
                elif int_move == 3:
                    move_dis = move_dis + 2
        else:
            if side_l != -999.9:
                temp_dis = side_l - target_l
                if abs(temp_dis - dis_dev) < 50.0:
                    move_dis = temp_dis
                    dis_dev = move_dis
                    num_error_dis = 0
                else:
                    num_error_dis += 1
                    move_dis = dis_dev
            else:
                if int_move == 1:
                    move_dis = move_dis - 2
                elif int_move == 2:
                    move_dis = move_dis - 4
                elif int_move == 3:
                    move_dis = move_dis + 2

        # 如果相差距离不大于10，开始累加5次后进入下一步；否则继续调整
        if abs(move_dis) <= 10 or abs(dis_dev) <= 10.0:
            target_count += 1
        else:
            target_count = 0

        # 键盘输入急停
        if keyboard.is_pressed('b'):
            get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return 96
    if target_count < 5 and num_error_dis == 10:
        get_correct_err = 95

    # 3.旋转回直
    if cam_yaw != -999.9:
        now_yaw = cam_yaw
    else:
        now_yaw = imu_yaw
    get_straight_err = correct_yaw(now_yaw, 0.0, cmd_56, q_v2a, q_m2a, q_a2m, q_c, q_ci, file_address)
    if get_straight_err > 90:
        return get_straight_err

    # 左右边全局记录清零
    rec_side_left = [target_l]
    rec_side_right = [target_r]
    return get_correct_err


# 前后清洗
def go_wash(wash_speed, cmd_56, q_v2a, q_m2a, q_a2m, q_c, q_ci, lock_ser, file_address, loop_times, wash_left, wash_right):
    global rec_side_left, rec_side_right
    # 传感器状态，0是正常，1是到边。99是传感器异常，98是无反馈，97是运行报错，96是手动急停
    get_state = 0
    imu_yaw = 0.0
    imu_pitch = 0.0
    imu_roll = 0.0
    loop_num = 0

    side_l = -999.9
    side_r = -999.9
    cam_yaw = -999.9

    now_yaw = 0.0
    dis_deviation = 0.0
    dis_deviation_left = 0.0
    dis_deviation_right = 0.0

    dis_laser = 999.9
    # 视觉偏航与IMU偏航相符标识位。如果相符，无视觉时靠IMU；如果不符，无视觉时置零。
    cy_equal_iy = False
    # 偏航调整
    moveDis_dev = 0
    stage_dev = 0
    toLeft_dev = False

    while get_state == 0 and (loop_num <= loop_times or loop_times == -1):
        loop_num += 1
        # 键盘输入急停
        if keyboard.is_pressed('b'):
            get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return 96

        # 1.特殊状态调整
        if keyboard.is_pressed('r') or dis_laser <= laser_threshold:      # 手动或测距到边
            get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            get_state = 1
            break
        elif abs(now_yaw) >= correct_thre_yaw:      # 偏航角过大，直接偏航校正
            print('偏航' + str(now_yaw))
            get_state = correct_yaw(now_yaw, 0.0, cmd_56, q_v2a, q_m2a, q_a2m, q_c, q_ci, file_address)
            if get_state > 90:
                get_error, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
                return get_state
            cam_yaw = 0.0
            now_yaw = 0.0
            loop_num = 0
            continue
        elif abs(dis_deviation) >= correct_thre_deviation:      # 偏移量过大，直接偏移校正
            print('偏移' + str(dis_deviation) + '  左侧' + str(side_l) + '  右侧' + str(side_r))
            get_state = correct_deviation(dis_deviation, wash_left, wash_right, cmd_56, q_v2a, q_m2a, q_a2m,
                                          q_c, q_ci, file_address)
            if get_state > 90:
                get_error, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
                return get_state
            cam_yaw = 0.0
            now_yaw = 0.0
            loop_num = 0
            continue
        elif wash_thre_deviation <= abs(dis_deviation) < correct_thre_deviation and stage_dev == 0:      # 偏移量较小，左右转后直行
            print('偏移调整', str(dis_deviation), str(dis_deviation_left), str(dis_deviation_right), rec_side_left, rec_side_right)
            moveDis_dev = int(round((abs(dis_deviation) - 10) / 4, 0)) + 4
            stage_dev = 0
            if dis_deviation < 0:
                toLeft_dev = False
            else:
                toLeft_dev = True
        elif abs(dis_deviation) < wash_thre_deviation < abs(moveDis_dev) and stage_dev > 0:     # 如果偏移调整过程中发现进入区域内，则跳出调整
            print('取消调整')
            if stage_dev == 1:
                stage_dev = moveDis_dev - 1
            elif stage_dev < moveDis_dev - 2:
                stage_dev = moveDis_dev - 2

        # 2.判断移动状态
        if loop_num * 5 < wash_speed:      # 启动阶段，每步5%加速
            cmd_24_speedUp = CVFunc.trans_speed(str(5 * loop_num))
            hex_washFront = CVFunc.set_order(
                cmd_0_head + cmd_13_front + cmd_24_speedUp + cmd_13_front + cmd_24_speedUp + cmd_56)
        elif moveDis_dev > 0:
            stage_dev += 1
            if 0 < stage_dev <= 2:
                if toLeft_dev:
                    hex_washFront = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_slow + cmd_13_front + cmd_24_fast + cmd_56)
                else:
                    hex_washFront = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_fast + cmd_13_front + cmd_24_slow + cmd_56)
            elif 2 < stage_dev <= (moveDis_dev - 2):
                print('剩余', str(moveDis_dev - stage_dev - 2))
                cmd_24_washSpeed = CVFunc.trans_speed(str(wash_speed))
                hex_washFront = CVFunc.set_order(
                    cmd_0_head + cmd_13_front + cmd_24_washSpeed + cmd_13_front + cmd_24_washSpeed + cmd_56)
            elif (moveDis_dev - 2) < stage_dev <= moveDis_dev:
                if toLeft_dev:
                    hex_washFront = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_fast + cmd_13_front + cmd_24_slow + cmd_56)
                else:
                    hex_washFront = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_slow + cmd_13_front + cmd_24_fast + cmd_56)
            else:
                print('完成调整')
                cmd_24_washSpeed = CVFunc.trans_speed(str(wash_speed))
                hex_washFront = CVFunc.set_order(
                    cmd_0_head + cmd_13_front + cmd_24_washSpeed + cmd_13_front + cmd_24_washSpeed + cmd_56)
                moveDis_dev = 0
                stage_dev = 0
        elif wash_thre_yaw < abs(now_yaw) < correct_thre_yaw:       # 偏航角在可控范围内，差速调整
            speed_left = wash_speed - int(round(now_yaw, 0))
            if speed_left < 0:
                speed_left = 0
            elif speed_left > 100:
                speed_left = 100
            cmd_2_corYaw = CVFunc.trans_speed(str(speed_left))
            speed_right = wash_speed + int(round(now_yaw, 0))
            if speed_right < 0:
                speed_right = 0
            elif speed_right > 100:
                speed_right = 100
            cmd_4_corYaw = CVFunc.trans_speed(str(speed_right))
            hex_washFront = CVFunc.set_order(
                cmd_0_head + cmd_13_front + cmd_2_corYaw + cmd_13_front + cmd_4_corYaw + cmd_56)
            print('差速调整', str(now_yaw), str(cam_yaw), str(imu_yaw), str(speed_left), str(speed_right) )
        else:
            cmd_24_washSpeed = CVFunc.trans_speed(str(wash_speed))
            hex_washFront = CVFunc.set_order(
                cmd_0_head + cmd_13_front + cmd_24_washSpeed + cmd_13_front + cmd_24_washSpeed + cmd_56)

        # 3.执行清洗移动
        get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
            hex_washFront, q_v2a, q_m2a, q_a2m, q_c, q_ci)
        if get_state > 90:
            get_error, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return get_state

        # 4.获取感知数据
        # 提取偏航角
        if cam_yaw != -999.9:
            now_yaw = cam_yaw
            if abs(cam_yaw - imu_yaw) < 2.0:
                cy_equal_iy = True
            else:
                cy_equal_iy = False
        else:
            if cy_equal_iy:
                now_yaw = imu_yaw
            else:
                now_yaw = 0.0
        # 有任意一边识别到距离，则开始提取偏移距离
        if panel_width > dis_left > 0.0:
            side_l = dis_left
            rec_side_left.append(dis_left)
        if panel_width > dis_right > 0.0:
            side_r = dis_right
            rec_side_right.append(dis_right)
        if side_l != -999.9 or side_r != -999.9:
            num_rec_right = len(rec_side_right)
            if num_rec_right > 2:
                last_right = (rec_side_right[num_rec_right - 2] + rec_side_right[num_rec_right - 3]) / 2
            else:
                last_right = wash_right

            num_rec_left = len(rec_side_left)
            if num_rec_left > 2:
                last_left = (rec_side_left[num_rec_left - 2] + rec_side_left[num_rec_left - 3]) / 2
            else:
                last_left = wash_left

            # if num_rec_left > 1 or num_rec_right > 1:
                # print(wash_left, rec_side_left, wash_right, rec_side_right)

            if (max(0.0, wash_right - 200) <= side_r <= wash_right + 200) and (
                    max(0.0, last_right - 10) <= side_r <= last_right + 10):
                dis_deviation_left = wash_right - side_r
            else:
                dis_deviation_left = -999.9

            if (max(0.0, wash_left - 200) <= side_l <= wash_left + 200) and (
                    max(0.0, last_left - 10) <= side_l <= last_left + 10):
                dis_deviation_right = side_l - wash_left
            else:
                dis_deviation_right = -999.9

            if dis_deviation_left != -999.9 and dis_deviation_right != -999.9:
                if abs(dis_deviation_left) < abs(dis_deviation_right):
                    dis_deviation = dis_deviation_left
                else:
                    dis_deviation = dis_deviation_right
            elif dis_deviation_left == -999.9 and dis_deviation_right != -999.9:
                dis_deviation = dis_deviation_right
            elif dis_deviation_left != -999.9 and dis_deviation_right == -999.9:
                dis_deviation = dis_deviation_left
            else:
                dis_deviation = 0.0
            # print(int(dis_deviation), int(dis_deviation_left), int(dis_deviation_right))
        else:
            dis_deviation = 0.0
            dis_deviation_left = 0.0
            dis_deviation_right = 0.0

        # 键盘输入急停
        if keyboard.is_pressed('b'):
            get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return 96

    return get_state


# 到边调头
def turn_around(cmd_56, q_v2a, q_m2a, q_a2m, q_c, q_ci, lock_ser, file_address):
    get_state = 0
    cmd_24_rotateSpeed = CVFunc.trans_speed('40')
    hex_turnAround = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_rotateSpeed + cmd_13_back + cmd_24_rotateSpeed + cmd_56)
    now_yaw = 0.0
    loop_times = 0

    # 向右不断旋转，直至转到180±20度
    while loop_times < 22:
        loop_times += 1
        # 键盘输入急停
        if keyboard.is_pressed('b'):
            get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return 96

        get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
            hex_turnAround, q_v2a, q_m2a, q_a2m, q_c, q_ci)
        if get_state > 90:
            get_error, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return get_state

    for i in range(0, 3, 1):
        # 键盘输入急停
        if keyboard.is_pressed('b'):
            get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return 96
        get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
            hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
        if get_state > 90:
            get_error, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return get_state
        if cam_yaw != -999.9:
            now_yaw = cam_yaw
    get_state = correct_yaw(now_yaw, 0.0, cmd_56, q_v2a, q_m2a, q_a2m, q_c, q_ci, file_address)

    return get_state


# 到边后“U”型转向
def u_turn(is_far, cmd_56, q_v2a, q_m2a, q_a2m, q_c, q_ci, lock_ser, file_address):
    get_state = 0
    moveSpeed_uTurn = 40
    cmd_24_rotateSpeed = CVFunc.trans_speed('40')
    cmd_24_moveSpeed = CVFunc.trans_speed(str(moveSpeed_uTurn))
    hex_turnRight = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_rotateSpeed + cmd_13_back + cmd_24_rotateSpeed + cmd_56)
    hex_turnLeft = CVFunc.set_order(cmd_0_head + cmd_13_back + cmd_24_rotateSpeed + cmd_13_front + cmd_24_rotateSpeed + cmd_56)
    hex_moveFront = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_moveSpeed + cmd_13_front + cmd_24_moveSpeed + cmd_56)

    imu_yaw = 0.0
    cam_yaw = -999.9
    loop_times = 0
    dis_laser = 999.9
    now_yaw = 0.0

    # 1.根据弓型方向，向左或向右不断旋转，直至转到90±20度。如果已到边界，直接退出并返回1
    while loop_times < 11:
        loop_times += 1
        # 键盘输入急停
        if keyboard.is_pressed('b'):
            get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return 96

        if is_far:
            hex_order = hex_turnLeft
        else:
            hex_order = hex_turnRight
        get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
            hex_order, q_v2a, q_m2a, q_a2m, q_c, q_ci)
        if get_state > 90:
            get_error, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return get_state
        if dis_laser <= laser_threshold:
            get_state = 1
            return get_state

    # 2.校准90度
    for i in range(0, 3, 1):
        get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
            hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
        if get_state > 90:
            get_error, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return get_state
        if cam_yaw != -999.9:
            now_yaw = cam_yaw
    get_state = correct_yaw(now_yaw, 0.0, cmd_56, q_v2a, q_m2a, q_a2m, q_c, q_ci, file_address)
    if get_state > 90:
        get_error, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
            hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
        return get_state

    # 3.根据速度和光伏板宽度，向前定次数移动。如果已到边界，直接退出并返回1
    for i in range(0, 41, 1):
        # 键盘输入急停
        if keyboard.is_pressed('b'):
            get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return 96

        if wash_thre_yaw < abs(cam_yaw):
            speed_left = moveSpeed_uTurn - int(round(cam_yaw, 0))
            if speed_left < 0:
                speed_left = 0
            elif speed_left > 100:
                speed_left = 100
            cmd_2_corYaw = CVFunc.trans_speed(str(speed_left))
            speed_right = moveSpeed_uTurn + int(round(cam_yaw, 0))
            if speed_right < 0:
                speed_right = 0
            elif speed_right > 100:
                speed_right = 100
            cmd_4_corYaw = CVFunc.trans_speed(str(speed_right))
            hex_order = CVFunc.set_order(
                cmd_0_head + cmd_13_front + cmd_2_corYaw + cmd_13_front + cmd_4_corYaw + cmd_56)
        else:
            hex_order = hex_moveFront
        get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
            hex_order, q_v2a, q_m2a, q_a2m, q_c, q_ci)
        if get_state > 90:
            get_error, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return get_state
        if dis_laser <= laser_threshold:
            get_state = 1
            return get_state

    # 4.向左或向右不断旋转，直至转到90±20度
    loop_times = 0
    while loop_times < 11:
        # 键盘输入急停
        if keyboard.is_pressed('b'):
            get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return 96

        loop_times += 1
        if is_far:
            hex_order = hex_turnLeft
        else:
            hex_order = hex_turnRight
        get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
            hex_order, q_v2a, q_m2a, q_a2m, q_c, q_ci)
        if get_state > 90:
            get_error, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return get_state
        if dis_laser <= laser_threshold:
            get_state = 1
            return get_state

    # 5.校准90度
    for i in range(0, 3, 1):
        get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
            hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
        if get_state > 90:
            get_error, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return get_state
        if cam_yaw != -999.9:
            now_yaw = cam_yaw
    get_state = correct_yaw(now_yaw, 0.0, cmd_56, q_v2a, q_m2a, q_a2m, q_c, q_ci, file_address)
    if get_state > 90:
        get_error, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
            hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
        return get_state

    return get_state


# 到达终点后返回
def end_back(cmd_56, q_v2a, q_m2a, q_a2m, q_c, q_ci, lock_ser, file_address):
    get_state = 0
    moveSpeed_uTurn = 40
    cmd_24_rotateSpeed = CVFunc.trans_speed('40')
    cmd_24_moveSpeed = CVFunc.trans_speed(str(moveSpeed_uTurn))
    hex_turnRight = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_rotateSpeed + cmd_13_back + cmd_24_rotateSpeed + cmd_56)
    hex_turnLeft = CVFunc.set_order(cmd_0_head + cmd_13_back + cmd_24_rotateSpeed + cmd_13_front + cmd_24_rotateSpeed + cmd_56)
    hex_moveFront = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_moveSpeed + cmd_13_front + cmd_24_moveSpeed + cmd_56)
    imu_yaw = 0.0
    cam_yaw = -999.9
    loop_times = 0
    dis_laser = 999.9
    now_yaw = 0.0

    # 1.向右不断旋转，直至转到180±20度
    while loop_times < 22:
        # 键盘输入急停
        if keyboard.is_pressed('b'):
            get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return 96

        loop_times += 1
        get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
            hex_turnRight, q_v2a, q_m2a, q_a2m, q_c, q_ci)
        if get_state > 90:
            get_error, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return get_state

    # 2.校准180度
    for i in range(0, 3, 1):
        # 键盘输入急停
        if keyboard.is_pressed('b'):
            get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return 96

        get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
            hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
        if get_state > 90:
            get_error, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return get_state
        if cam_yaw != -999.9:
            now_yaw = cam_yaw
    get_state = correct_yaw(now_yaw, 0.0, cmd_56, q_v2a, q_m2a, q_a2m, q_c, q_ci, file_address)
    cam_yaw = 0.0
    if get_state > 90:
        get_error, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
            hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
        return get_state

    # 3.向前移动直至到边界
    while dis_laser > laser_threshold:
        # 键盘输入急停
        if keyboard.is_pressed('b'):
            get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return 96

        if wash_thre_yaw < abs(cam_yaw):
            speed_left = moveSpeed_uTurn - int(round(cam_yaw, 0))
            if speed_left < 0:
                speed_left = 0
            elif speed_left > 100:
                speed_left = 100
            cmd_2_corYaw = CVFunc.trans_speed(str(speed_left))
            speed_right = moveSpeed_uTurn + int(round(cam_yaw, 0))
            if speed_right < 0:
                speed_right = 0
            elif speed_right > 100:
                speed_right = 100
            cmd_4_corYaw = CVFunc.trans_speed(str(speed_right))
            hex_order = CVFunc.set_order(
                cmd_0_head + cmd_13_front + cmd_2_corYaw + cmd_13_front + cmd_4_corYaw + cmd_56)
        else:
            hex_order = hex_moveFront
        get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
            hex_order, q_v2a, q_m2a, q_a2m, q_c, q_ci)
        if get_state > 90:
            get_error, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            return get_state
        elif keyboard.is_pressed('e') or dis_laser < laser_threshold:
            get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            get_state = 1
            break

    return get_state

# 执行自动控制
def autocontrol_run(q_v2a, q_m2a, q_a2m, q_c, q_ci, lock_ser, file_address):
    global panel_width, rec_side_left, rec_side_right

    print('Auto Start')

    # 设置清洗参数
    loop_time = -1  # 单向算作1次
    wash_loops = -1     # 设置前后清洗的移动距离限定，-1代表无限定。每次200ms，5次相当于1秒
    wash_right_dis = 200.0    # 右侧边界距离
    wash_left_dis = panel_width - vehicle_width - wash_right_dis       # 左侧边界距离
    need_back = False

    imu_yaw = 0.0
    imu_pitch = 0.0
    imu_roll = 0.0
    cmd_56 = cmd_5_stop + cmd_6_stop

    cam_yaw = -999.9
    side_f = -999.9
    side_l = -999.9
    side_r = -999.9

    # 无反馈重新循环标志位
    no_feedBack = False
    while True:
# 1.键盘输入运行
        no_feedBack = False
        is_oneAction = True
        while is_oneAction:
            # 发送数据
            if keyboard.is_pressed('w') or keyboard.is_pressed('8'):  # 向前移动，刮板和水系统停止
                print('前进')
                hex_order = hex_Front
            elif keyboard.is_pressed('s') or keyboard.is_pressed('2'):  # 向后移动，刮板和水系统停止
                print('后退')
                hex_order = hex_Back
            elif keyboard.is_pressed('a') or keyboard.is_pressed('4'):  # 向左旋转，刮板和水系统停止
                print('左旋')
                hex_order = hex_rotateLeft
            elif keyboard.is_pressed('d') or keyboard.is_pressed('6'):  # 向右旋转，刮板和水系统停止
                print('右旋')
                hex_order = hex_rotateRight
            elif keyboard.is_pressed('q') or keyboard.is_pressed('7'):  # 前行并左转，刮板和水系统停止
                print('左前转')
                hex_order = hex_FrontLeft
            elif keyboard.is_pressed('e') or keyboard.is_pressed('9'):  # 前行并左转，刮板和水系统停止
                print('右前转')
                hex_order = hex_FrontRight
            elif keyboard.is_pressed('r'):  # 结束运行，全部停止
                print('结束运行')
                hex_order = hex_allStop
                is_oneAction = False
            else:  # 无输入，停止移动，刮板和水系统停止
                # print('停止')
                hex_order = hex_allStop
            get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                hex_order, q_v2a, q_m2a, q_a2m, q_c, q_ci)
            if get_state > 90:
                get_error, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                    hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
                if get_state == 98:
                    no_feedBack = True
                return get_state
        if no_feedBack:
            continue

# 2.进入自动控制
        print('进入自动控制')
        get_state = 0
        # 建立通信，开始执行
        loop_num = -1
        while (not need_back) and (loop_num < loop_time or loop_time == -1):
            loop_num += 1
            print('左边距' + str(wash_left_dis) + ' 右边距' + str(wash_right_dis))
            # 边距记录清零
            rec_side_right = [wash_right_dis]
            rec_side_left = [wash_left_dis]
            now_yaw = 0.0

            # 3.1.如果角度有偏航，进行校正
            for i in range(0, 3, 1):
                get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                    hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
                if get_state == 98:
                    print('无反馈，结束运行')
                    no_feedBack = True
                    break
                elif get_state == 96:
                    print('手动急停')
                    no_feedBack = True
                    break
                elif get_state > 90:
                    print('故障')
                    no_feedBack = True
                    break
                if dis_left != -999.9:
                    rec_side_left.append(dis_left)
                if dis_right != -999.9:
                    rec_side_right.append(dis_right)
                if cam_yaw != -999.9:
                    now_yaw = cam_yaw

            # 偏航大于1.0就调整拉直
            if abs(now_yaw) > wash_thre_yaw:
                print('偏航角度' + str(now_yaw))
                get_state = correct_yaw(now_yaw, 0.0, cmd_56, q_v2a, q_m2a, q_a2m, q_c, q_ci, file_address)
                # 无反馈退出
                if get_state == 98:
                    print('无反馈，结束运行')
                    no_feedBack = True
                elif get_state == 96:
                    print('手动急停')
                    no_feedBack = True
                elif get_state > 90:
                    print('故障')
                    no_feedBack = True
                    break
                print('校正完成')
            else:
                print('偏航在范围内')
            if no_feedBack:
                continue

            # 3.2. 如果距离轨迹有偏移，进行校正
            # 静止获取左右任意一边距离。如果同时获取到两边，则更新光伏板宽度。
            for i in range(0, 3, 1):
                get_state, cam_yaw, dis_left, dis_right, dis_laser, imu_roll, imu_pitch, imu_yaw = control_communication(
                    hex_allStop, q_v2a, q_m2a, q_a2m, q_c, q_ci)
                if get_state == 98:
                    print('无反馈，结束运行')
                    no_feedBack = True
                    break
                elif get_state == 96:
                    print('手动急停')
                    no_feedBack = True
                    break
                elif get_state > 90:
                    print('故障')
                    no_feedBack = True
                    break
                if dis_left != -999.9:
                    rec_side_left.append(dis_left)
                if dis_right != -999.9:
                    rec_side_right.append(dis_right)
                if cam_yaw != -999.9:
                    now_yaw = cam_yaw
            # 累计左侧距离取平均
            if len(rec_side_left) > 1:
                temp_sum = 0.0
                for i in range(1, len(rec_side_left), 1):
                    temp_sum += rec_side_left[i]
                side_l = temp_sum / (len(rec_side_left) - 1)
            else:
                side_l = -999.9
            # 累计右侧距离取平均
            if len(rec_side_right) > 1:
                temp_sum = 0.0
                for i in range(1, len(rec_side_right), 1):
                    temp_sum += rec_side_right[i]
                side_r = temp_sum / (len(rec_side_right) - 1)
            else:
                side_r = -999.9
            print(rec_side_left, rec_side_right)
            # 根据识别情况进行处理
            if side_l != -999.9 or side_r != -999.9:
                # 如果同时识别到两边，则更新光伏板宽度
                if side_l != -999.9 and side_r != -999.9 and abs(panel_width - (side_l + side_r + vehicle_width)) < 50:
                    panel_width = side_l + side_r + vehicle_width
                    if wash_left_dis <= wash_right_dis:
                        wash_right_dis = panel_width - vehicle_width - wash_left_dis
                    else:
                        wash_left_dis = panel_width - vehicle_width - wash_right_dis
                    print('光伏板全长' + str(round(panel_width, 0)))

                # 优先判断距离近的边，如果没有再判断距离远的边
                dis_deviation = 0.0
                if wash_left_dis <= wash_right_dis:
                    if side_l != -999.9 and abs(side_l - wash_left_dis) > 20:
                        dis_deviation = side_l - wash_left_dis
                    elif side_r != -999.9 and abs(side_r - wash_right_dis) > 20:
                        dis_deviation = wash_right_dis - side_r
                else:
                    if side_r != -999.9 and abs(side_r - wash_right_dis) > 20:
                        dis_deviation = wash_right_dis - side_r
                    elif side_l != -999.9 and abs(side_l - wash_left_dis) > 20:
                        dis_deviation = side_l - wash_left_dis
                if abs(dis_deviation) > 50.0:
                    print('偏移距离' + str(dis_deviation))
                    get_state = correct_deviation(dis_deviation, wash_left_dis, wash_right_dis, cmd_56, q_v2a, q_m2a, q_a2m,
                                                  q_c, q_ci, file_address)
                    # 无反馈退出
                    if get_state == 98:
                        print('无反馈，结束运行')
                        no_feedBack = True
                    elif get_state == 96:
                        print('手动急停')
                        no_feedBack = True
                    elif get_state > 90:
                        print('故障')
                        no_feedBack = True
                        break
                    print('校正完成')
                else:
                    print('偏移在范围内')
            else:
                print('未找到边界')
            if no_feedBack:
                continue

            # 3.3.清洗前进
            print('清洗前行，次数', str(wash_loops))
            get_state = go_wash(int_washSpeed, cmd_56, q_v2a, q_m2a, q_a2m, q_c, q_ci, lock_ser, file_address, wash_loops, wash_left_dis, wash_right_dis)
            if get_state == 98:
                print('无反馈，结束运行')
                no_feedBack = True
                break
            elif get_state == 96:
                print('手动急停')
                no_feedBack = True
                break
            elif get_state > 90:
                print('故障')
                no_feedBack = True
                break
            elif get_state == 1:
                print('到边')

            # # 3.4.到达位置，调头
            # get_state = turn_around(cmd_56, q_v2a, q_m2a, q_a2m, q_c, q_ci, lock_ser, file_address)
            # if get_state == 98:
            #     print('无反馈，结束运行')
            #     no_feedBack = True
            #     break
            # elif get_state == 96:
            #     print('手动急停')
            #     no_feedBack = True
            #     break
            # elif get_state > 90:
            #     print('故障')
            #     no_feedBack = True
            #     break
            # print('完成调头')


            # 3.4 根据循环次数判断左移还是右移
            if loop_num % 2 == 0:
                get_state = u_turn(True, cmd_56, q_v2a, q_m2a, q_a2m, q_c, q_ci, lock_ser, file_address)
            else:
                get_state = u_turn(False, cmd_56, q_v2a, q_m2a, q_a2m, q_c, q_ci, lock_ser, file_address)
            if get_state == 98:
                print('无反馈，结束运行')
                no_feedBack = True
                break
            elif get_state == 96:
                print('手动急停')
                no_feedBack = True
                break
            elif get_state > 90:
                print('故障')
                no_feedBack = True
                break
            elif get_state == 1:
                need_back = True

            # 3.5 左右设定距离对调
            temp_lr = wash_right_dis
            wash_right_dis = wash_left_dis
            wash_left_dis = temp_lr
            # print('完成调头')

            # 3.6 如果到尽头，调头返回
            if need_back:
                print('完成清洗，返回')
                get_state = end_back(cmd_56, q_v2a, q_m2a, q_a2m, q_c, q_ci, lock_ser, file_address)
                if get_state == 98:
                    print('无反馈，结束运行')
                    no_feedBack = True
                    break
                elif get_state == 96:
                    print('手动急停')
                    no_feedBack = True
                    break
                elif get_state > 90:
                    print('故障')
                    no_feedBack = True
                    break

            if loop_time != -1:
                print('循环剩余' + str(loop_time - loop_num))
        print('自动循环结束')
        need_back = False
        if no_feedBack:
            continue
