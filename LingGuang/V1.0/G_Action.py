import binascii
import time

# 读取参数
file_model = open('./Parameters.txt', 'r', encoding='utf-8')
para_lines = file_model.readlines()
file_model.close()

ATTR_STATE_RSSI = (1 << 5 | 0)  # 信号强度
ATTR_STATE_UPDOWN_LOAD = (1 << 5 | 1)  # 提升负荷
ATTR_STATE_WATER = (1 << 5 | 2)  # 液位
ATTR_STATE_BATTERY = (1 << 5 | 3)  # 电量
ATTR_STATE_DIR_U = (1 << 5 | 4)  # 转向上
ATTR_STATE_DIR_D = (1 << 5 | 5)  # 转向下
ATTR_STATE_DISTANCE_U = (1 << 5 | 6)  # 测距上
ATTR_STATE_DISTANCE_D = (1 << 5 | 7)  # 测试下
ATTR_STATE_DISTANCE_L = (1 << 5 | 8)  # 测距左
ATTR_STATE_DISTANCE_R = (1 << 5 | 9)  # 测距右
ATTR_STATE_NTC = (1 << 5 | 10)  # NTC温度
ATTR_STATE_TOUCH = (1 << 5 | 11)  # 防跌落状态 STATE_ON: 至少有一个防跌落触发
ATTR_STATE_TIME_ONCE = (1 << 5 | 12)  # 本次工作时长
ATTR_STATE_TIME_SUM = (1 << 5 | 13)  # 累计工作时长
ATTR_STATE_VISION_STATE = (1 << 5 | 14)  # 视觉工作状态

ATTR_CONTROL_MODE = (2 << 5 | 0)  # 运行模式
ATTR_CONTROL_BOOST_STATE = (2 << 5 | 1)  # 推进使能
ATTR_CONTROL_BOOST_VALUE = (2 << 5 | 2)  # 推进力度
ATTR_CONTROL_ADHESION = (2 << 5 | 3)  # 吸附
ATTR_CONTROL_UPDOWN = (2 << 5 | 4)  # 上下行
ATTR_CONTROL_DIR = (2 << 5 | 5)  # 转向
ATTR_CONTROL_CLEAN = (2 << 5 | 6)  # 清洗系统
ATTR_CONTROL_WATER = (2 << 5 | 7)  # 水循环系统
ATTR_CONTROL_AUTO_TYPE = (2 << 5 | 8)  # 自动类型
ATTR_CONTROL_MOVE_PATH = (2 << 5 | 9)  # 贴墙距离
ATTR_CONTROL_WATER_RECYCLE = (2 << 5 | 10)  # 水回收
ATTR_CONTROL_WATER_HEAT = (2 << 5 | 11)  # 水加热
ATTR_CONTROL_CATERPILLA_LEFT = (2 << 5 | 12)  # 左履带
ATTR_CONTROL_CATERPILLA_RIGHT = (2 << 5 | 13)  # 右履带


# 侧边全局缓存
rec_side_left = [0.0]
rec_side_right = [0.0]

ctl_order = [
            ATTR_CONTROL_CATERPILLA_LEFT, 0, 0,  # 左履带
            ATTR_CONTROL_CATERPILLA_RIGHT, 0, 0,  # 右履带
            ATTR_CONTROL_ADHESION, 0, 0,  # 吸附
            ATTR_CONTROL_CLEAN, 0, 0,  # 清洗系统
            ATTR_CONTROL_WATER, 0, 0,  # 水循环系统
            ATTR_STATE_VISION_STATE, 0, 0   # 视觉状态反馈
        ]

ctl_allStop = [
            ATTR_CONTROL_CATERPILLA_LEFT, 0, 0,  # 左履带
            ATTR_CONTROL_CATERPILLA_RIGHT, 0, 0,  # 右履带
            ATTR_CONTROL_ADHESION, 0, 0,  # 吸附
            ATTR_CONTROL_CLEAN, 0, 0,  # 清洗系统
            ATTR_CONTROL_WATER, 0, 0,  # 水循环系统
            ATTR_STATE_VISION_STATE, 0, 0   # 视觉状态反馈
        ]

ctl_stop = [
            ATTR_CONTROL_CATERPILLA_LEFT, 0, 0,  # 左履带
            ATTR_CONTROL_CATERPILLA_RIGHT, 0, 0,  # 右履带
        ]

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


def find_id(get_list, target_id):
    get_id = -1
    get_list_size = len(get_list)
    for i in range(0, get_list_size, 1):
        if get_list[i][0] == target_id:
            get_id = i
            break

    return get_id


def control_communication(ctl_com, qin, qout, q_ci):
    get_state = 0

    left_id = ctl_com.index(ATTR_CONTROL_CATERPILLA_LEFT)
    # left_id = find_id(ctl_com, ATTR_CONTROL_CATERPILLA_LEFT)
    right_id = ctl_com.index(ATTR_CONTROL_CATERPILLA_RIGHT)
    # right_id = find_id(ctl_com, ATTR_CONTROL_CATERPILLA_RIGHT)

    # 发送命令
    qout.put(ctl_com)
    qout.get() if qout.qsize() > 1 else time.sleep(0.001)
    if ctl_order[left_id + 2] == 0 and ctl_order[right_id + 2] == 0:
        q_ci.put(True)
        q_ci.get() if q_ci.qsize() > 1 else time.sleep(0.001)
    else:
        q_ci.put(False)
        q_ci.get() if q_ci.qsize() > 1 else time.sleep(0.001)

    # print(ctl_com)

    # 等待反馈
    if not qin.empty():
        sensor_list = qin.get()
        # print('Get Ret', sensor_list, len(sensor_list))
        if len(sensor_list) == 1:
            print(sensor_list)
            get_state = 98
            return get_state
        elif len(sensor_list) > 1:
            # 读取前向测距
            laser_front_id = find_id(sensor_list, ATTR_STATE_DISTANCE_U)
            laser_front_dis = sensor_list[laser_front_id][1]

            # 读取后向测距
            laser_back_id = find_id(sensor_list, ATTR_STATE_DISTANCE_D)
            laser_back_dis = sensor_list[laser_back_id][1]

            # 读取防跌落传感器
            falling_id = find_id(sensor_list, ATTR_STATE_TOUCH)
            if sensor_list[falling_id][1] != 0xff:
                # int转4位二进制
                temp_state = sensor_list[falling_id][2]
                falling_state = format(temp_state, "b")
                if len(falling_state) < 4:
                    falling_state = '0' * (4 - len(falling_state)) + falling_state

                # 判断4个防跌落状态
                if falling_state[0:1] == '0':
                    fall_BackRight = False
                elif falling_state[0:1] == '1':
                    fall_BackRight = True
                else:
                    fall_BackRight = False
                if falling_state[1:2] == '0':
                    fall_BackLeft = False
                elif falling_state[1:2] == '1':
                    fall_BackLeft = True
                else:
                    fall_BackLeft = False
                if falling_state[2:3] == '0':
                    fall_FrontRight = False
                elif falling_state[2:3] == '1':
                    fall_FrontRight = True
                else:
                    fall_FrontRight = False
                if falling_state[3:4] == '0':
                    fall_FrontLeft = False
                elif falling_state[3:4] == '1':
                    fall_FrontLeft = True
                else:
                    fall_FrontLeft = False

                print(fall_FrontLeft, fall_BackLeft, fall_FrontRight, fall_BackRight)

                if (not fall_BackLeft and not fall_BackRight) and (fall_FrontLeft and fall_FrontRight) or laser_front_dis < 50:
                    get_state = 1
                elif (not fall_FrontLeft and not fall_FrontRight) and (fall_BackLeft and fall_BackRight) or laser_back_dis < 50:
                    get_state = 2
                elif (not fall_FrontRight and not fall_BackRight) and (fall_FrontLeft and fall_BackLeft):
                    get_state = 3
                elif (not fall_FrontLeft and not fall_BackLeft) and (fall_FrontRight and fall_BackRight):
                    get_state = 4
                elif not fall_FrontLeft and not fall_BackLeft and not fall_FrontRight and not fall_BackRight:
                    get_state = 0
                else:
                    get_state = 5
            else:
                falling_state = sensor_list[falling_id][2]

            # if falling_state == 0xfd and laser_front_dis == 0xfffd and laser_back_dis == 0xfffd:
            #     get_state = 98

            print(laser_front_dis, laser_back_dis, falling_state, get_state)
    else:
        time.sleep(0.05)
        if not qin.empty():
            sensor_list = qin.get()
            if len(sensor_list) == 1:
                print(sensor_list)
                get_state = 98
                return get_state
            elif len(sensor_list) > 1:
                # 读取前向测距
                laser_front_id = find_id(sensor_list, ATTR_STATE_DISTANCE_U)
                laser_front_dis = sensor_list[laser_front_id][1]

                # 读取后向测距
                laser_back_id = find_id(sensor_list, ATTR_STATE_DISTANCE_D)
                laser_back_dis = sensor_list[laser_back_id][1]

                # 读取防跌落传感器
                falling_id = find_id(sensor_list, ATTR_STATE_TOUCH)
                if sensor_list[falling_id][1] != 0xff:
                    # int转4位二进制
                    temp_state = sensor_list[falling_id][2]
                    falling_state = format(temp_state, "b")
                    if len(falling_state) < 4:
                        falling_state = '0' * (4 - len(falling_state)) + falling_state

                    # 判断4个防跌落状态
                    if falling_state[0:1] == '0':
                        fall_BackRight = False
                    elif falling_state[0:1] == '1':
                        fall_BackRight = True
                    else:
                        fall_BackRight = False
                    if falling_state[1:2] == '0':
                        fall_BackLeft = False
                    elif falling_state[1:2] == '1':
                        fall_BackLeft = True
                    else:
                        fall_BackLeft = False
                    if falling_state[2:3] == '0':
                        fall_FrontRight = False
                    elif falling_state[2:3] == '1':
                        fall_FrontRight = True
                    else:
                        fall_FrontRight = False
                    if falling_state[3:4] == '0':
                        fall_FrontLeft = False
                    elif falling_state[3:4] == '1':
                        fall_FrontLeft = True
                    else:
                        fall_FrontLeft = False

                    print(fall_FrontLeft, fall_BackLeft, fall_FrontRight, fall_BackRight)

                    if (not fall_BackLeft and not fall_BackRight) and (
                            fall_FrontLeft and fall_FrontRight) or laser_front_dis < 50:
                        get_state = 1
                    elif (not fall_FrontLeft and not fall_FrontRight) and (
                            fall_BackLeft and fall_BackRight) or laser_back_dis < 50:
                        get_state = 2
                    elif (not fall_FrontRight and not fall_BackRight) and (fall_FrontLeft and fall_BackLeft):
                        get_state = 3
                    elif (not fall_FrontLeft and not fall_BackLeft) and (fall_FrontRight and fall_BackRight):
                        get_state = 4
                    elif not fall_FrontLeft and not fall_BackLeft and not fall_FrontRight and not fall_BackRight:
                        get_state = 0
                    else:
                        get_state = 5
                else:
                    falling_state = sensor_list[falling_id][2]

                # if falling_state == 0xfd and laser_front_dis == 0xfffd and laser_back_dis == 0xfffd:
                #     get_state = 98

                print(laser_front_dis, laser_back_dis, falling_state, get_state)

    return get_state

# 校正偏航角
def correct_yaw(now_yaw, target_yaw, q_v2a, qin, q_i2a, qout, q_ci):
    global rec_side_left, rec_side_right, ctl_order

    print('Yaw1' + str(now_yaw))

    get_state = 0
    diff_cam_imu = 0.0
    rot_yaw = now_yaw - target_yaw
    target_count = 0

    cam_yaw = dis_left = dis_right = side_l = side_r = imu_roll = imu_pitch = imu_yaw = -999.9
    num_front = 0
    dis_front = [-999.9]

    if rot_yaw < 0:
        last_to_left = False
        now_to_left = False
    else:
        last_to_left = True
        now_to_left = True

    # 等待视觉测距稳定
    for i in range(0, 3, 1):
        get_state = control_communication(ctl_stop, qin, qout, q_ci)
        if get_state > 90:
            get_error = control_communication(ctl_allStop, qin, qout, q_ci)
            return get_state

        time.sleep(0.2)

        if not q_v2a.empty():
            dis_list = q_v2a.get()
            cam_yaw = dis_list[0]
            dis_left = dis_list[1]
            dis_right = dis_list[2]
            # num_front = dis_list[3]
            print(dis_list)

        if not q_i2a.empty():
            imu_roll, imu_pitch, imu_yaw = q_i2a.get()

        if dis_left != -999.9:
            rec_side_left.append(dis_left)
        if dis_right != -999.9:
            rec_side_right.append(dis_right)
        if cam_yaw != -999.9:
            diff_cam_imu = cam_yaw - imu_yaw

    print('Yaw2' + str(cam_yaw))

    while target_count <= 2:
        # 设置旋转动作
        if target_count == 0:
            # 根据偏航角设定初始旋转速度
            if abs(rot_yaw) > 20:
                correct_speed = 40
            elif abs(rot_yaw) > 10:
                correct_speed = 25
            else:
                correct_speed = 15

            if rot_yaw < 0:     # yaw负数，偏向左，向右旋转
                ctl_correctYaw = [
                    ATTR_CONTROL_CATERPILLA_LEFT, 2, correct_speed,  # 左履带
                    ATTR_CONTROL_CATERPILLA_RIGHT, 3, correct_speed,  # 右履带
                ]
                move_side = 1
            else:       # yaw正数，偏向右，向左旋转
                ctl_correctYaw = [
                    ATTR_CONTROL_CATERPILLA_LEFT, 3, correct_speed,  # 左履带
                    ATTR_CONTROL_CATERPILLA_RIGHT, 2, correct_speed,  # 右履带
                ]
                move_side = -1
        else:
            correct_speed = 0
            ctl_correctYaw = [
                ATTR_CONTROL_CATERPILLA_LEFT, 0, 0,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 0, 0,  # 右履带
            ]
            move_side = 0

        # 发送动作命令
        get_state = control_communication(ctl_correctYaw, qin, qout, q_ci)
        if get_state > 90:
            get_error = control_communication(ctl_allStop, qin, qout, q_ci)
            return get_state

        time.sleep(0.15)

        if not q_v2a.empty():
            dis_list = q_v2a.get()
            cam_yaw = dis_list[0]
            dis_left = dis_list[1]
            dis_right = dis_list[2]
            # num_front = dis_list[3]
            # print(dis_list)
        else:
            time.sleep(0.05)
            dis_list = q_v2a.get()
            cam_yaw = dis_list[0]
            dis_left = dis_list[1]
            dis_right = dis_list[2]
            # num_front = dis_list[3]
            # print(dis_list)
        if not q_i2a.empty():
            imu_roll, imu_pitch, imu_yaw = q_i2a.get()

        # 发送动作命令
        get_state = control_communication(ctl_stop, qin, qout, q_ci)
        if get_state > 90:
            get_error = control_communication(ctl_allStop, qin, qout, q_ci)
            return get_state

        time.sleep(0.15)

        if not q_v2a.empty():
            dis_list = q_v2a.get()
            cam_yaw = dis_list[0]
            dis_left = dis_list[1]
            dis_right = dis_list[2]
            # num_front = dis_list[3]
            # print(dis_list)
        else:
            time.sleep(0.05)
            dis_list = q_v2a.get()
            cam_yaw = dis_list[0]
            dis_left = dis_list[1]
            dis_right = dis_list[2]
            # num_front = dis_list[3]
            # print(dis_list)
        if not q_i2a.empty():
            imu_roll, imu_pitch, imu_yaw = q_i2a.get()

        if not q_i2a.empty():
            imu_roll, imu_pitch, imu_yaw = q_i2a.get()
        if dis_left != -999.9:
            rec_side_left.append(dis_left)
        if dis_right != -999.9:
            rec_side_right.append(dis_right)
        if cam_yaw != -999.9:
            rot_yaw = cam_yaw - target_yaw
            print(str(rot_yaw), str(cam_yaw))
        else:
            pass
            # rot_yaw = rot_yaw + move_side * correct_speed * angular_speed
            # print(str(rot_yaw), '估测', str(move_side * correct_speed * angular_speed))

        if abs(rot_yaw) <= 2.0:  # 如果与目标角度相差不大于1，则认为已完成，
            target_count += 1
        else:
            target_count = 0


    return get_state


# # 校正偏移距离
# def correct_deviation(dis_dev, target_l, target_r, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address):
#     global rec_side_left, rec_side_right
#     is_right = False
#     imu_yaw = 0.0
#     side_l = -999.9
#     side_r = -999.9
#     cam_yaw = -999.9
#     now_yaw = 0.0
#     move_dis = 0.0
#
#     # 1.更新调整距离
#     num_timer = 0
#     try_again = True
#     while num_timer < 3 and try_again:
#         num_timer += 1
#         # 停止
#         get_state, cam_yaw, side_l, side_r, imu_roll, imu_pitch, imu_yaw = control_communication(hex_allStop, q_v2a, qin, q_i2a, qout, q_ci)
#         if get_state > 0:
#             return get_state
#         if cam_yaw != -999.9:
#             now_yaw = cam_yaw
#
#         # 判断平移距离
#         if target_l <= target_r:
#             if side_l != -999.9:
#                 move_dis = side_l - target_l
#                 dis_dev = move_dis
#                 try_again = False
#             elif side_r != -999.9:
#                 move_dis = target_r - side_r
#             else:
#                 move_dis = dis_dev
#         else:
#             if side_r != -999.9:
#                 move_dis = target_r - side_r
#                 dis_dev = move_dis
#                 try_again = False
#             elif side_l != -999.9:
#                 move_dis = side_l - target_l
#             else:
#                 move_dis = dis_dev
#
#     # 2.根据偏航距离，判断左旋还是右旋
#     if move_dis < 0:
#         is_right = True
#         rot_angle = 30.0
#     else:
#         is_right = False
#         rot_angle = -30.0
#     get_correct_err = correct_yaw(now_yaw, rot_angle, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
#     if get_correct_err > 0:
#         return get_correct_err
#
#     # 3.直线移动
#     num_error_dis = 0
#     target_count = 0
#     dis_dev = abs(dis_dev)
#     move_dis = abs(move_dis)
#     while target_count < 5 and num_error_dis < 10:
#         print('测量距离' + str(int(move_dis)) + '  估算距离' + str(int(dis_dev)))
#
#         # 根据剩余距离设定速度和前进后退
#         if target_count == 0:
#             if -50.0 < move_dis < -10.0:
#                 cmd_24_corSpeed = CVFunc.trans_speed('10')
#                 hex_correctDis = CVFunc.set_order(cmd_0_head + cmd_13_back + cmd_24_corSpeed + cmd_13_back + cmd_24_corSpeed + cmd_56)
#                 int_move = 3
#                 num_error_dis = 0
#             elif move_dis >= 10.0:
#                 if move_dis > 50.0:
#                     cmd_24_corSpeed = CVFunc.trans_speed('20')
#                     int_move = 2
#                 else:
#                     cmd_24_corSpeed = CVFunc.trans_speed('10')
#                     int_move = 1
#                 hex_correctDis = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_corSpeed + cmd_13_front + cmd_24_corSpeed + cmd_56)
#                 num_error_dis = 0
#             else:
#                 hex_correctDis = hex_allStop
#                 int_move = 0
#                 num_error_dis += 1
#         else:
#             hex_correctDis = hex_allStop
#             int_move = 0
#         # 发送动作命令
#         get_state, cam_yaw, side_l, side_r, imu_roll, imu_pitch, imu_yaw = control_communication(hex_correctDis, q_v2a, qin, q_i2a, qout, q_ci)
#         if get_state > 0:
#             get_error, cam_yaw, dis_left, dis_right, imu_roll, imu_pitch, imu_yaw = control_communication(
#                 hex_allStop, q_v2a, qin, q_i2a, qout, q_ci)
#             return get_state
#
#         # 估算的剩余行驶距离
#         if int_move == 1:
#             dis_dev = dis_dev - 2
#         elif int_move == 2:
#             dis_dev = dis_dev - 4
#         elif int_move == 3:
#             dis_dev = dis_dev + 2
#         # 实际测得的剩余行驶距离，相差过大则直接使用估算距离
#         if is_right:
#             if side_r != -999.9:
#                 temp_dis = side_r - target_r
#                 if abs(temp_dis - dis_dev) < 50.0:
#                     move_dis = temp_dis
#                     dis_dev = move_dis
#                     num_error_dis = 0
#                 else:
#                     num_error_dis += 1
#                     move_dis = dis_dev
#             else:
#                 if int_move == 1:
#                     move_dis = move_dis - 2
#                 elif int_move == 2:
#                     move_dis = move_dis - 4
#                 elif int_move == 3:
#                     move_dis = move_dis + 2
#         else:
#             if side_l != -999.9:
#                 temp_dis = side_l - target_l
#                 if abs(temp_dis - dis_dev) < 50.0:
#                     move_dis = temp_dis
#                     dis_dev = move_dis
#                     num_error_dis = 0
#                 else:
#                     num_error_dis += 1
#                     move_dis = dis_dev
#             else:
#                 if int_move == 1:
#                     move_dis = move_dis - 2
#                 elif int_move == 2:
#                     move_dis = move_dis - 4
#                 elif int_move == 3:
#                     move_dis = move_dis + 2
#
#         # 如果相差距离不大于10，开始累加5次后进入下一步；否则继续调整
#         if abs(move_dis) <= 10 or abs(dis_dev) <= 10.0:
#             target_count += 1
#         else:
#             target_count = 0
#
#     if target_count < 5 and num_error_dis == 10:
#         get_correct_err = 95
#
#     # 3.旋转回直
#     if cam_yaw != -999.9:
#         now_yaw = cam_yaw
#     else:
#         now_yaw = imu_yaw
#     get_straight_err = correct_yaw(now_yaw, 0.0, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
#     if get_straight_err > 0:
#         return get_straight_err
#
#     # 左右边全局记录清零
#     rec_side_left = [target_l]
#     rec_side_right = [target_r]
#     return get_correct_err
#
#
# 前后清洗
# def go_wash(wash_speed, q_v2a, qin, q_i2a, qout, q_ci, loop_times, wash_left, wash_right):
#     global rec_side_left, rec_side_right
#     # 传感器状态，0是正常，1是到边。99是传感器异常，98是无反馈，97是运行报错，96是手动急停
#     get_state = 0
#     imu_yaw = 0.0
#     imu_pitch = 0.0
#     imu_roll = 0.0
#     loop_num = 0
#
#     side_l = -999.9
#     side_r = -999.9
#     cam_yaw = -999.9
#
#     now_yaw = 0.0
#     dis_deviation = 0.0
#     dis_deviation_left = 0.0
#     dis_deviation_right = 0.0
#
#     # 视觉偏航与IMU偏航相符标识位。如果相符，无视觉时靠IMU；如果不符，无视觉时置零。
#     cy_equal_iy = False
#     # 偏航调整
#     moveDis_dev = 0
#     stage_dev = 0
#     toLeft_dev = False
#
#     # 到边前减速标识符
#     need_slowdown = False
#     slow_loop_num = 0
#
#     ctl_go = [
#         ATTR_CONTROL_CATERPILLA_LEFT, 0, 0,  # 左履带
#         ATTR_CONTROL_CATERPILLA_RIGHT, 0, 0,  # 右履带
#     ]
#
#     while get_state == 0 and (loop_num <= loop_times or loop_times == -1):
#         loop_num += 1
#         # 1.特殊状态调整
#         if get_state == 1:      # 手动或测距到边
#             get_error = control_communication(ctl_allStop, qin, qout, q_ci)
#             get_state = 1
#             break
#         elif abs(now_yaw) >= correct_thre_yaw:      # 偏航角过大，直接偏航校正
#             print('偏航' + str(now_yaw))
#             get_state = correct_yaw(cam_yaw, 0.0, q_v2a, qin, queue_imu2action, qout, q_ci)
#             if get_state > 0:
#                 get_error = control_communication(ctl_allStop, qin, qout, q_ci)
#                 return get_state
#             cam_yaw = 0.0
#             now_yaw = 0.0
#             loop_num = 0
#             continue
#         # elif abs(dis_deviation) >= correct_thre_deviation:      # 偏移量过大，直接偏移校正
#         #     print('偏移' + str(dis_deviation) + '  左侧' + str(side_l) + '  右侧' + str(side_r))
#         #     get_state = correct_deviation(dis_deviation, wash_left, wash_right, cmd_56, q_v2a, qin, qout,
#         #                                   q_ci, file_address)
#         #     if get_state > 0:
#         #         get_error, cam_yaw, dis_left, dis_right, imu_roll, imu_pitch, imu_yaw = control_communication(hex_allStop, q_v2a, qin, q_i2a, qout, q_ci)
#         #         return get_state
#         #     cam_yaw = 0.0
#         #     now_yaw = 0.0
#         #     loop_num = 0
#         #     continue
#         # elif wash_thre_deviation <= abs(dis_deviation) < correct_thre_deviation and stage_dev == 0:      # 偏移量较小，左右转后直行
#         elif wash_thre_deviation <= abs(dis_deviation) and stage_dev == 0:  # 偏移量较小，左右转后直行
#             print('偏移调整', str(dis_deviation), str(dis_deviation_left), str(dis_deviation_right), rec_side_left, rec_side_right)
#             moveDis_dev = int(round((abs(dis_deviation) - 10) / 4, 0)) + 4
#             stage_dev = 0
#             if dis_deviation < 0:
#                 toLeft_dev = False
#             else:
#                 toLeft_dev = True
#         elif abs(dis_deviation) < wash_thre_deviation < abs(moveDis_dev) and stage_dev > 0:     # 如果偏移调整过程中发现进入区域内，则跳出调整
#             print('取消调整')
#             if stage_dev == 1:
#                 stage_dev = moveDis_dev - 1
#             elif stage_dev < moveDis_dev - 2:
#                 stage_dev = moveDis_dev - 2
#         # 画面丢失光伏板，速度减半（测试阶段用手动逻辑）
#         # if side_l == -999.0 and side_r == -999.9 and cam_yaw == -999.9:
#         #     print('减速')
#         #     need_slowdown = True
#         #     slow_loop_num = loop_num
#         # else:
#         #     need_slowdown = False
#         #     slow_loop_num = 0
#
#         # 2.判断移动状态
#         # if loop_num * 5 < wash_speed:      # 启动阶段，每步5%加速
#         #     cmd_24_speedUp = CVFunc.trans_speed(str(5 * loop_num))
#         #     hex_washFront = CVFunc.set_order(
#         #         cmd_0_head + cmd_13_front + cmd_24_speedUp + cmd_13_front + cmd_24_speedUp + cmd_56)
#         # elif need_slowdown:
#         #     if wash_speed - (loop_num - slow_loop_num) * 5 > 10:
#         #         cmd_24_slowDown = CVFunc.trans_speed(str(wash_speed - (loop_num - slow_loop_num) * 5))
#         #     else:
#         #         cmd_24_slowDown = CVFunc.trans_speed('10')
#         #     hex_washFront = CVFunc.set_order(
#         #         cmd_0_head + cmd_13_front + cmd_24_slowDown + cmd_13_front + cmd_24_slowDown + cmd_56)
#         if moveDis_dev > 0:
#             stage_dev += 1
#             if 0 < stage_dev <= 2:
#                 if toLeft_dev:
#                     ctl_go = [
#                         ATTR_CONTROL_CATERPILLA_LEFT, 2, wash_speed - 10,  # 左履带
#                         ATTR_CONTROL_CATERPILLA_RIGHT, 2, wash_speed + 10,  # 右履带
#                     ]
#                 else:
#                     ctl_go = [
#                         ATTR_CONTROL_CATERPILLA_LEFT, 2, wash_speed + 10,  # 左履带
#                         ATTR_CONTROL_CATERPILLA_RIGHT, 2, wash_speed - 10,  # 右履带
#                     ]
#             elif 2 < stage_dev <= (moveDis_dev - 2):
#                 print('剩余', str(moveDis_dev - stage_dev - 2))
#                 ctl_go = [
#                     ATTR_CONTROL_CATERPILLA_LEFT, 2, wash_speed,  # 左履带
#                     ATTR_CONTROL_CATERPILLA_RIGHT, 2, wash_speed,  # 右履带
#                 ]
#             elif (moveDis_dev - 2) < stage_dev <= moveDis_dev:
#                 if toLeft_dev:
#                     ctl_go = [
#                         ATTR_CONTROL_CATERPILLA_LEFT, 2, wash_speed + 10,  # 左履带
#                         ATTR_CONTROL_CATERPILLA_RIGHT, 2, wash_speed - 10,  # 右履带
#                     ]
#                 else:
#                     ctl_go = [
#                         ATTR_CONTROL_CATERPILLA_LEFT, 2, wash_speed - 10,  # 左履带
#                         ATTR_CONTROL_CATERPILLA_RIGHT, 2, wash_speed + 10,  # 右履带
#                     ]
#             else:
#                 print('完成调整')
#                 ctl_go = [
#                     ATTR_CONTROL_CATERPILLA_LEFT, 2, wash_speed,  # 左履带
#                     ATTR_CONTROL_CATERPILLA_RIGHT, 2, wash_speed,  # 右履带
#                 ]
#                 moveDis_dev = 0
#                 stage_dev = 0
#         # elif wash_thre_yaw < abs(now_yaw) < correct_thre_yaw:       # 偏航角在可控范围内，差速调整
#         elif wash_thre_yaw < abs(now_yaw):  # 偏航角在可控范围内，差速调整
#             speed_left = move_speed - int(round(now_yaw, 0))
#             if speed_left < 0:
#                 speed_left = 0
#             elif speed_left > 100:
#                 speed_left = 100
#             # cmd_2_corYaw = CVFunc.trans_speed(str(speed_left))
#             speed_right = move_speed + int(round(now_yaw, 0))
#             if speed_right < 0:
#                 speed_right = 0
#             elif speed_right > 100:
#                 speed_right = 100
#             ctl_go = [
#                 ATTR_CONTROL_CATERPILLA_LEFT, 2, speed_left,  # 左履带
#                 ATTR_CONTROL_CATERPILLA_RIGHT, 2, speed_right,  # 右履带
#             ]
#             print('差速调整', str(now_yaw), str(cam_yaw), str(imu_yaw), str(speed_left), str(speed_right) )
#         else:
#             ctl_go = [
#                 ATTR_CONTROL_CATERPILLA_LEFT, 2, move_speed,  # 左履带
#                 ATTR_CONTROL_CATERPILLA_RIGHT, 2, move_speed,  # 右履带
#             ]
#
#         # 3.执行清洗移动
#         get_state = control_communication(ctl_go, qin, qout, q_ci)
#         if get_state == 1:
#             print('Board')
#             get_error = control_communication(ctl_allStop, qin, qout, q_ci)
#             get_state = 1
#             return get_state
#         else:
#             print('Error')
#             get_error = control_communication(ctl_allStop, qin, qout, q_ci)
#             return get_state
#
#         time.sleep(0.2)
#
#         # 4.获取感知数据
#         if not q_v2a.empty():
#             dis_list = q_v2a.get()
#             cam_yaw = dis_list[0]
#             side_l = dis_list[1]
#             side_r = dis_list[2]
#             # num_front = dis_list[3]
#             print(dis_list)
#
#         if not q_i2a.empty():
#             imu_roll, imu_pitch, imu_yaw = q_i2a.get()
#         # 提取偏航角
#         if cam_yaw != -999.9:
#             now_yaw = cam_yaw
#             if abs(cam_yaw - imu_yaw) < 2.0:
#                 cy_equal_iy = True
#             else:
#                 cy_equal_iy = False
#         else:
#             if cy_equal_iy:
#                 now_yaw = imu_yaw
#             else:
#                 now_yaw = 0.0
#         # 有任意一边识别到距离，则开始提取偏移距离
#         if side_l != -999.9:
#             rec_side_left.append(side_l)
#         if side_r != -999.9:
#             rec_side_right.append(side_r)
#         if side_l != -999.9 or side_r != -999.9:
#             num_rec_right = len(rec_side_right)
#             if num_rec_right > 2:
#                 last_right = (rec_side_right[num_rec_right - 2] + rec_side_right[num_rec_right - 3]) / 2
#             else:
#                 last_right = wash_right
#
#             num_rec_left = len(rec_side_left)
#             if num_rec_left > 2:
#                 last_left = (rec_side_left[num_rec_left - 2] + rec_side_left[num_rec_left - 3]) / 2
#             else:
#                 last_left = wash_left
#
#             if (max(0.0, wash_right - 200) <= side_r <= wash_right + 200) and (
#                     max(0.0, last_right - 10) <= side_r <= last_right + 10):
#                 dis_deviation_left = wash_right - side_r
#             else:
#                 dis_deviation_left = -999.9
#
#             if (max(0.0, wash_left - 200) <= side_l <= wash_left + 200) and (
#                     max(0.0, last_left - 10) <= side_l <= last_left + 10):
#                 dis_deviation_right = side_l - wash_left
#             else:
#                 dis_deviation_right = -999.9
#
#             if dis_deviation_left != -999.9 and dis_deviation_right != -999.9:
#                 if abs(dis_deviation_left) < abs(dis_deviation_right):
#                     dis_deviation = dis_deviation_left
#                 else:
#                     dis_deviation = dis_deviation_right
#             elif dis_deviation_left == -999.9 and dis_deviation_right != -999.9:
#                 dis_deviation = dis_deviation_right
#             elif dis_deviation_left != -999.9 and dis_deviation_right == -999.9:
#                 dis_deviation = dis_deviation_left
#             else:
#                 dis_deviation = 0.0
#         else:
#             dis_deviation = 0.0
#             dis_deviation_left = 0.0
#             dis_deviation_right = 0.0
#
#     return get_state

#
# 到边调头
def turn_around(to_left, q_v2a, qin, q_i2a, qout, q_ci):
    global para_lines
    # 机身尺寸、光伏板尺寸
    vehicle_left = int(para_lines[0].strip('\n'))
    vehicle_right = int(para_lines[1].strip('\n'))
    vehicle_front = int(para_lines[2].strip('\n'))
    vehicle_front_falling = int(para_lines[3].strip('\n'))
    vehicle_width = vehicle_left + vehicle_right
    # 校正阈值(度或毫米)
    wash_thre_yaw = float(para_lines[4].strip('\n'))
    correct_thre_yaw = float(para_lines[5].strip('\n'))
    wash_thre_deviation = float(para_lines[6].strip('\n'))
    correct_thre_deviation = float(para_lines[7].strip('\n'))
    # 机器速率
    straight_speed = float(para_lines[8].strip('\n'))
    angular_speed = float(para_lines[9].strip('\n'))
    # 运行速度
    int_washSpeed = int(para_lines[10].strip('\n'))
    int_moveSpeed = int(para_lines[11].strip('\n'))
    int_turnSpeed = int(para_lines[12].strip('\n'))
    # Times
    times_Lturn = int(para_lines[13].strip('\n'))
    times_GoBack = int(para_lines[14].strip('\n'))
    times_ChangeLine = int(para_lines[15].strip('\n'))
    times_Uturn = int(para_lines[16].strip('\n'))

    get_state = 0

    imu_roll = imu_pitch = imu_yaw = 0.0
    cam_yaw = -999.9
    loop_times = 0
    now_yaw = -999.9

    # 根据角速度计算转动次数
    rotate_times = int(round(90 / (int_turnSpeed * angular_speed), 0))

    # 1.根据转向型方向，向左或向右不断旋转，根据角速度预计转到90度。如果已到边界，直接退出并返回1
    for i in range(0, times_Uturn, 1):
        if to_left:
            ctl_Lturn = [
                ATTR_CONTROL_CATERPILLA_LEFT, 3, int_turnSpeed,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 2, int_turnSpeed,  # 右履带
            ]
        else:
            ctl_Lturn = [
                ATTR_CONTROL_CATERPILLA_LEFT, 2, int_turnSpeed,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 3, int_turnSpeed,  # 右履带
            ]
        get_state = control_communication(ctl_Lturn, qin, qout, q_ci)
        if get_state > 90:
            get_error = control_communication(ctl_allStop, qin, qout, q_ci)
            return get_state
        # if cam_yaw != -999.9:
        #     now_yaw = cam_yaw

        time.sleep(0.2)
        if not q_v2a.empty():
            dis_list = q_v2a.get()
            cam_yaw = dis_list[0]
            side_l = dis_list[1]
            side_r = dis_list[2]
            # num_front = dis_list[3]
            # print(dis_list)
        if not q_i2a.empty():
            imu_roll, imu_pitch, imu_yaw = q_i2a.get()
        if cam_yaw != -999.9:
            now_yaw = cam_yaw

    # if now_yaw != -999.9 and abs(now_yaw) > 5.0:
    #     for i in range(0, 3, 1):
    #         get_state = control_communication(ctl_stop, qin, qout, q_ci)
    #
    #         time.sleep(0.2)
    #
    #         if get_state > 90:
    #             get_error = control_communication(ctl_allStop, qin, qout, q_ci)
    #             return get_state
    #         if not q_v2a.empty():
    #             dis_list = q_v2a.get()
    #             cam_yaw = dis_list[0]
    #             side_l = dis_list[1]
    #             side_r = dis_list[2]
    #             # num_front = dis_list[3]
    #             # print(dis_list)
    #
    #         if not q_i2a.empty():
    #             imu_roll, imu_pitch, imu_yaw = q_i2a.get()
    #         if cam_yaw != -999.9:
    #             now_yaw = cam_yaw
    #
    #     loop_num = -1
    #     while (to_left and now_yaw > 5.0) or (not to_left and now_yaw < -5.0):
    #         if to_left:
    #             ctl_Lturn = [
    #                 ATTR_CONTROL_CATERPILLA_LEFT, 3, int(int_turnSpeed / 4),  # 左履带
    #                 ATTR_CONTROL_CATERPILLA_RIGHT, 2, int(int_turnSpeed / 4),  # 右履带
    #             ]
    #         else:
    #             ctl_Lturn = [
    #                 ATTR_CONTROL_CATERPILLA_LEFT, 2, int(int_turnSpeed / 4),  # 左履带
    #                 ATTR_CONTROL_CATERPILLA_RIGHT, 3, int(int_turnSpeed / 4),  # 右履带
    #             ]
    #         get_state = control_communication(ctl_Lturn, qin, qout, q_ci)
    #         if get_state > 90:
    #             get_error = control_communication(ctl_allStop, qin, qout, q_ci)
    #             return get_state
    #         # if cam_yaw != -999.9:
    #         #     now_yaw = cam_yaw
    #
    #         time.sleep(0.2)
    #         if not q_v2a.empty():
    #             dis_list = q_v2a.get()
    #             cam_yaw = dis_list[0]
    #             side_l = dis_list[1]
    #             side_r = dis_list[2]
    #             # num_front = dis_list[3]
    #             # print(dis_list)
    #         if not q_i2a.empty():
    #             imu_roll, imu_pitch, imu_yaw = q_i2a.get()
    #         if cam_yaw != -999.9:
    #             now_yaw = cam_yaw
    #
    #         if loop_num > times_Lturn or abs(now_yaw) <= 5.0 or (to_left and now_yaw < 0.0) or (not to_left and now_yaw > 0.0):
    #             break
#
# # 到达终点后返回
# def end_back(cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address):
#     get_state = 0
#     moveSpeed_uTurn = 40
#     cmd_24_rotateSpeed = CVFunc.trans_speed('40')
#     cmd_24_moveSpeed = CVFunc.trans_speed(str(moveSpeed_uTurn))
#     hex_turnRight = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_rotateSpeed + cmd_13_back + cmd_24_rotateSpeed + cmd_56)
#     hex_turnLeft = CVFunc.set_order(cmd_0_head + cmd_13_back + cmd_24_rotateSpeed + cmd_13_front + cmd_24_rotateSpeed + cmd_56)
#     hex_moveFront = CVFunc.set_order(cmd_0_head + cmd_13_front + cmd_24_moveSpeed + cmd_13_front + cmd_24_moveSpeed + cmd_56)
#     imu_yaw = 0.0
#     cam_yaw = -999.9
#     loop_times = 0
#     dis_laser = 999.9
#     now_yaw = 0.0
#
#     # 1.向右不断旋转，直至转到180±20度
#     while loop_times < 22:
#         # 键盘输入急停
#         if keyboard.is_pressed('b'):
#             get_state, cam_yaw, dis_left, dis_right, imu_roll, imu_pitch, imu_yaw = control_communication(
#                 hex_allStop, q_v2a, qin, q_i2a, qout, q_ci)
#             return 96
#
#         loop_times += 1
#         get_state, cam_yaw, dis_left, dis_right, imu_roll, imu_pitch, imu_yaw = control_communication(
#             hex_turnRight, q_v2a, qin, q_i2a, qout, q_ci)
#         if get_state > 0:
#             get_error, cam_yaw, dis_left, dis_right, imu_roll, imu_pitch, imu_yaw = control_communication(
#                 hex_allStop, q_v2a, qin, q_i2a, qout, q_ci)
#             return get_state
#
#     # 2.校准180度
#     for i in range(0, 3, 1):
#         # 键盘输入急停
#         if keyboard.is_pressed('b'):
#             get_state, cam_yaw, dis_left, dis_right, imu_roll, imu_pitch, imu_yaw = control_communication(
#                 hex_allStop, q_v2a, qin, q_i2a, qout, q_ci)
#             return 96
#
#         get_state, cam_yaw, dis_left, dis_right, imu_roll, imu_pitch, imu_yaw = control_communication(
#             hex_allStop, q_v2a, qin, q_i2a, qout, q_ci)
#         if get_state > 0:
#             get_error, cam_yaw, dis_left, dis_right, imu_roll, imu_pitch, imu_yaw = control_communication(
#                 hex_allStop, q_v2a, qin, q_i2a, qout, q_ci)
#             return get_state
#         if cam_yaw != -999.9:
#             now_yaw = cam_yaw
#     get_state = correct_yaw(now_yaw, 0.0, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
#     cam_yaw = 0.0
#     if get_state > 0:
#         get_error, cam_yaw, dis_left, dis_right, imu_roll, imu_pitch, imu_yaw = control_communication(
#             hex_allStop, q_v2a, qin, q_i2a, qout, q_ci)
#         return get_state
#
#     # 3.向前移动直至到边界
#     while dis_laser > laser_threshold:
#         # 键盘输入急停
#         if keyboard.is_pressed('b'):
#             get_state, cam_yaw, dis_left, dis_right, imu_roll, imu_pitch, imu_yaw = control_communication(
#                 hex_allStop, q_v2a, qin, q_i2a, qout, q_ci)
#             return 96
#
#         if wash_thre_yaw < abs(cam_yaw):
#             speed_left = moveSpeed_uTurn - int(round(cam_yaw, 0))
#             if speed_left < 0:
#                 speed_left = 0
#             elif speed_left > 100:
#                 speed_left = 100
#             cmd_2_corYaw = CVFunc.trans_speed(str(speed_left))
#             speed_right = moveSpeed_uTurn + int(round(cam_yaw, 0))
#             if speed_right < 0:
#                 speed_right = 0
#             elif speed_right > 100:
#                 speed_right = 100
#             cmd_4_corYaw = CVFunc.trans_speed(str(speed_right))
#             ctl_order = CVFunc.set_order(
#                 cmd_0_head + cmd_13_front + cmd_2_corYaw + cmd_13_front + cmd_4_corYaw + cmd_56)
#         else:
#             ctl_order = hex_moveFront
#         get_state, cam_yaw, dis_left, dis_right, imu_roll, imu_pitch, imu_yaw = control_communication(
#             ctl_order, q_v2a, qin, q_i2a, qout, q_ci)
#         if get_state > 0:
#             get_error, cam_yaw, dis_left, dis_right, imu_roll, imu_pitch, imu_yaw = control_communication(
#                 hex_allStop, q_v2a, qin, q_i2a, qout, q_ci)
#             return get_state
#
#         if keyboard.is_pressed('e') or dis_laser < laser_threshold:
#             get_state, cam_yaw, dis_left, dis_right, imu_roll, imu_pitch, imu_yaw = control_communication(
#                 hex_allStop, q_v2a, qin, q_i2a, qout, q_ci)
#             get_state = 1
#             break
#
#     return get_state
#
#
# 90度“L”型转向动作
def l_turn(to_left, q_v2a, qin, q_i2a, qout, q_ci):
    global para_lines
    # 机身尺寸、光伏板尺寸
    vehicle_left = int(para_lines[0].strip('\n'))
    vehicle_right = int(para_lines[1].strip('\n'))
    vehicle_front = int(para_lines[2].strip('\n'))
    vehicle_front_falling = int(para_lines[3].strip('\n'))
    vehicle_width = vehicle_left + vehicle_right
    # 校正阈值(度或毫米)
    wash_thre_yaw = float(para_lines[4].strip('\n'))
    correct_thre_yaw = float(para_lines[5].strip('\n'))
    wash_thre_deviation = float(para_lines[6].strip('\n'))
    correct_thre_deviation = float(para_lines[7].strip('\n'))
    # 机器速率
    straight_speed = float(para_lines[8].strip('\n'))
    angular_speed = float(para_lines[9].strip('\n'))
    # 运行速度
    int_washSpeed = int(para_lines[10].strip('\n'))
    int_moveSpeed = int(para_lines[11].strip('\n'))
    int_turnSpeed = int(para_lines[12].strip('\n'))
    # Times
    times_Lturn = int(para_lines[13].strip('\n'))
    times_GoBack = int(para_lines[14].strip('\n'))
    times_ChangeLine = int(para_lines[15].strip('\n'))
    times_Uturn = int(para_lines[16].strip('\n'))

    get_state = 0

    imu_roll = imu_pitch = imu_yaw = 0.0
    cam_yaw = -999.9
    loop_times = 0
    now_yaw = -999.9

    # 根据角速度计算转动次数
    rotate_times = int(round(90 / (int_turnSpeed * angular_speed), 0))

    # 1.根据转向型方向，向左或向右不断旋转，根据角速度预计转到90度。如果已到边界，直接退出并返回1
    for i in range(0, times_Lturn, 1):
        if to_left:
            ctl_Lturn = [
                ATTR_CONTROL_CATERPILLA_LEFT, 3, int_turnSpeed,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 2, int_turnSpeed,  # 右履带
            ]
        else:
            ctl_Lturn = [
                ATTR_CONTROL_CATERPILLA_LEFT, 2, int_turnSpeed,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 3, int_turnSpeed,  # 右履带
            ]
        get_state = control_communication(ctl_Lturn, qin, qout, q_ci)
        if get_state > 90:
            get_error = control_communication(ctl_allStop, qin, qout, q_ci)
            return get_state
        # if cam_yaw != -999.9:
        #     now_yaw = cam_yaw

        time.sleep(0.2)
        if not q_v2a.empty():
            dis_list = q_v2a.get()
            cam_yaw = dis_list[0]
            side_l = dis_list[1]
            side_r = dis_list[2]
            # num_front = dis_list[3]
            # print(dis_list)
        if not q_i2a.empty():
            imu_roll, imu_pitch, imu_yaw = q_i2a.get()
        if cam_yaw != -999.9:
            now_yaw = cam_yaw

    # if now_yaw != -999.9 and abs(now_yaw) > 5.0:
    #     for i in range(0, 5, 1):
    #         get_state = control_communication(ctl_stop, qin, qout, q_ci)
    #
    #         time.sleep(0.2)
    #
    #         if get_state > 90:
    #             get_error = control_communication(ctl_allStop, qin, qout, q_ci)
    #             return get_state
    #         if not q_v2a.empty():
    #             dis_list = q_v2a.get()
    #             cam_yaw = dis_list[0]
    #             side_l = dis_list[1]
    #             side_r = dis_list[2]
    #             # num_front = dis_list[3]
    #             # print(dis_list)
    #
    #         if not q_i2a.empty():
    #             imu_roll, imu_pitch, imu_yaw = q_i2a.get()
    #         if cam_yaw != -999.9:
    #             now_yaw = cam_yaw
    #
    #     loop_num = -1
    #     if abs(now_yaw) > 5.0:
    #         while (to_left and now_yaw > 5.0) or (not to_left and now_yaw < -5.0):
    #             if to_left:
    #                 ctl_Lturn = [
    #                     ATTR_CONTROL_CATERPILLA_LEFT, 3, int(int_turnSpeed / 4),  # 左履带
    #                     ATTR_CONTROL_CATERPILLA_RIGHT, 2, int(int_turnSpeed / 4),  # 右履带
    #                 ]
    #             else:
    #                 ctl_Lturn = [
    #                     ATTR_CONTROL_CATERPILLA_LEFT, 2, int(int_turnSpeed / 4),  # 左履带
    #                     ATTR_CONTROL_CATERPILLA_RIGHT, 3, int(int_turnSpeed / 4),  # 右履带
    #                 ]
    #             get_state = control_communication(ctl_Lturn, qin, qout, q_ci)
    #             if get_state > 90:
    #                 get_error = control_communication(ctl_allStop, qin, qout, q_ci)
    #                 return get_state
    #             # if cam_yaw != -999.9:
    #             #     now_yaw = cam_yaw
    #
    #             time.sleep(0.2)
    #             if not q_v2a.empty():
    #                 dis_list = q_v2a.get()
    #                 cam_yaw = dis_list[0]
    #                 side_l = dis_list[1]
    #                 side_r = dis_list[2]
    #                 # num_front = dis_list[3]
    #                 # print(dis_list)
    #             if not q_i2a.empty():
    #                 imu_roll, imu_pitch, imu_yaw = q_i2a.get()
    #             if cam_yaw != -999.9:
    #                 now_yaw = cam_yaw
    #
    #             if loop_num > times_Lturn or abs(now_yaw) <= 10.0 or (to_left and now_yaw < 0.0) or (not to_left and now_yaw > 0.0):
    #                 get_stop = control_communication(ctl_stop, qin, qout, q_ci)
    #                 break


    # # 2.校准90度
    # for i in range(0, 3, 1):
    #     get_state = control_communication(ctl_allStop, qin, qout, q_ci)
    #
    #     time.sleep(0.2)
    #
    #     if get_state > 90:
    #         get_error = control_communication(ctl_allStop, qin, qout, q_ci)
    #         return get_state
    #     if not q_v2a.empty():
    #         dis_list = q_v2a.get()
    #         cam_yaw = dis_list[0]
    #         side_l = dis_list[1]
    #         side_r = dis_list[2]
    #         # num_front = dis_list[3]
    #         print(dis_list)
    #
    #     if not q_i2a.empty():
    #         imu_roll, imu_pitch, imu_yaw = q_i2a.get()
    #     if cam_yaw != -999.9:
    #         now_yaw = cam_yaw
    # if now_yaw != -999.9:
    #     get_state = correct_yaw(now_yaw, 0.0, q_v2a, qin, q_i2a, qout, q_ci)
    #     if get_state > 90:
    #         get_error = control_communication(ctl_allStop, qin, qout, q_ci)
    #         return get_state
    # else:
    #     print('Cam Yaw not found')

    get_stop = control_communication(ctl_stop, qin, qout, q_ci)

    return get_state


# 直行指令（0：直行、1：加速、2：减速、3：后退、4：测试直行）
def go_straight(move_type, move_speed, move_times, q_v2a, qin, q_i2a, qout, q_ci):
    global rec_side_left, rec_side_right, ctl_order, para_lines
    # 机身尺寸、光伏板尺寸
    vehicle_left = int(para_lines[0].strip('\n'))
    vehicle_right = int(para_lines[1].strip('\n'))
    vehicle_front = int(para_lines[2].strip('\n'))
    vehicle_front_falling = int(para_lines[3].strip('\n'))
    vehicle_width = vehicle_left + vehicle_right
    # 校正阈值(度或毫米)
    wash_thre_yaw = float(para_lines[4].strip('\n'))
    correct_thre_yaw = float(para_lines[5].strip('\n'))
    wash_thre_deviation = float(para_lines[6].strip('\n'))
    correct_thre_deviation = float(para_lines[7].strip('\n'))
    # 机器速率
    straight_speed = float(para_lines[8].strip('\n'))
    angular_speed = float(para_lines[9].strip('\n'))
    # 运行速度
    int_washSpeed = int(para_lines[10].strip('\n'))
    int_moveSpeed = int(para_lines[11].strip('\n'))
    int_turnSpeed = int(para_lines[12].strip('\n'))
    # Times
    times_Lturn = int(para_lines[13].strip('\n'))
    times_GoBack = int(para_lines[14].strip('\n'))
    times_ChangeLine = int(para_lines[15].strip('\n'))
    times_Uturn = int(para_lines[16].strip('\n'))

    get_state = 0
    imu_yaw = 0.0
    imu_pitch = 0.0
    imu_roll = 0.0
    side_l = -999.9
    side_r = -999.9
    cam_yaw = -999.9
    now_yaw = 0.0
    dis_left = -999.9
    dis_right = -999.9

    slow_num = 0
    now_speed = move_speed

    ctl_go = [
        ATTR_CONTROL_CATERPILLA_LEFT, 0, 0,  # 左履带
        ATTR_CONTROL_CATERPILLA_RIGHT, 0, 0,  # 右履带
    ]

    loop_num = 0
    # if move_type > 0:
    #     loop_times = move_times + 3
    # else:
    #     loop_times = move_times
    loop_times = move_times
    print('Times' + str(loop_times))
    if not q_v2a.empty():
        dis_list = q_v2a.get()
        cam_yaw = dis_list[0]
        dis_left = dis_list[1]
        dis_right = dis_list[2]
        # num_front = dis_list[3]
        print(dis_list)

    if not q_i2a.empty():
        imu_roll, imu_pitch, imu_yaw = q_i2a.get()

    # 视觉偏航与IMU偏航相符标识位。如果相符，无视觉时靠IMU；如果不符，无视觉时置零。
    cy_equal_iy = False
    while loop_num <= loop_times or loop_times == -1:
        loop_num += 1
        print(loop_num)
        # 更新偏航角
        if not q_v2a.empty():
            dis_list = q_v2a.get()
            cam_yaw = dis_list[0]
            dis_left = dis_list[1]
            dis_right = dis_list[2]
            # num_front = dis_list[3]
            print(dis_list)
        else:
            time.sleep(0.05)
            if not q_v2a.empty():
                dis_list = q_v2a.get()
                cam_yaw = dis_list[0]
                dis_left = dis_list[1]
                dis_right = dis_list[2]
                # num_front = dis_list[3]
                print(dis_list)
            else:
                cam_yaw = dis_left = dis_right = -999.9
        if not q_i2a.empty():
            imu_roll, imu_pitch, imu_yaw = q_i2a.get()
        if cam_yaw != -999.9:
            now_yaw = cam_yaw
        else:
            now_yaw = 0.0

        if move_type == 1:    # 加速
            if loop_num < move_times:
                # cmd_24_moveSpeed = CVFunc.trans_speed(str(int(round(move_speed * loop_num / move_times, 0))))
                ctl_go = [
                    ATTR_CONTROL_CATERPILLA_LEFT, 2, int(round(move_speed * loop_num / move_times, 0)),  # 左履带
                    ATTR_CONTROL_CATERPILLA_RIGHT, 2, int(round(move_speed * loop_num / move_times, 0)),  # 右履带
                ]
            else:
                # cmd_24_moveSpeed = CVFunc.trans_speed(str(move_speed))
                ctl_go = [
                    ATTR_CONTROL_CATERPILLA_LEFT, 2, move_speed,  # 左履带
                    ATTR_CONTROL_CATERPILLA_RIGHT, 2, move_speed,  # 右履带
                ]
        elif move_type == 2:    # 减速
            if loop_num <= 3:
                ctl_go = [
                    ATTR_CONTROL_CATERPILLA_LEFT, 2, move_speed,  # 左履带
                    ATTR_CONTROL_CATERPILLA_RIGHT, 2, move_speed,  # 右履带
                ]
                # cmd_24_moveSpeed = CVFunc.trans_speed(str(move_speed))
                # hex_goStraight = CVFunc.set_order(
                #     cmd_0_head + cmd_13_front + cmd_24_moveSpeed + cmd_13_front + cmd_24_moveSpeed + cmd_56)
            elif loop_num - 3 < move_times:
                ctl_go = [
                    ATTR_CONTROL_CATERPILLA_LEFT, 2, int(round(move_speed * (move_times - loop_num + 3) / move_times, 0)),  # 左履带
                    ATTR_CONTROL_CATERPILLA_RIGHT, 2, int(round(move_speed * (move_times - loop_num + 3) / move_times, 0)),  # 右履带
                ]
                # cmd_24_moveSpeed = CVFunc.trans_speed(str(int(round(move_speed * (move_times - loop_num + 3) / move_times, 0))))
                # hex_goStraight = CVFunc.set_order(
                #     cmd_0_head + cmd_13_front + cmd_24_moveSpeed + cmd_13_front + cmd_24_moveSpeed + cmd_56)
            else:
                ctl_go = [
                    ATTR_CONTROL_CATERPILLA_LEFT, 0, 0,  # 左履带
                    ATTR_CONTROL_CATERPILLA_RIGHT, 0, 0,  # 右履带
                ]
                # hex_goStraight = hex_allStop
        elif move_type == 3:    # 后退
            ctl_go = [
                ATTR_CONTROL_CATERPILLA_LEFT, 3, move_speed,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 3, move_speed,  # 右履带
            ]
        elif move_type == 0:       # 直行并带偏航差速调整
            if cam_yaw == -999.9 and dis_left == -999.9 and dis_right == -999.9:
                slow_num += 1
                if slow_num < 3:
                    ctl_go = [
                        ATTR_CONTROL_CATERPILLA_LEFT, 2, move_speed,  # 左履带
                        ATTR_CONTROL_CATERPILLA_RIGHT, 2, move_speed,  # 右履带
                    ]
                else:
                    ctl_go = [
                        ATTR_CONTROL_CATERPILLA_LEFT, 2, int(move_speed / 2),  # 左履带
                        ATTR_CONTROL_CATERPILLA_RIGHT, 2, int(move_speed / 2),  # 右履带
                    ]

            elif wash_thre_yaw < abs(now_yaw):
                slow_num = 0
                speed_left = move_speed - int(round(now_yaw * 2, 0))
                if speed_left < 0:
                    speed_left = 0
                elif speed_left > 100:
                    speed_left = 100
                # cmd_2_corYaw = CVFunc.trans_speed(str(speed_left))
                speed_right = move_speed + int(round(now_yaw * 2, 0))
                if speed_right < 0:
                    speed_right = 0
                elif speed_right > 100:
                    speed_right = 100
                ctl_go = [
                    ATTR_CONTROL_CATERPILLA_LEFT, 2, speed_left,  # 左履带
                    ATTR_CONTROL_CATERPILLA_RIGHT, 2, speed_right,  # 右履带
                ]
            else:
                slow_num = 0
                ctl_go = [
                    ATTR_CONTROL_CATERPILLA_LEFT, 2, move_speed,  # 左履带
                    ATTR_CONTROL_CATERPILLA_RIGHT, 2, move_speed,  # 右履带
                ]
        elif move_type == 4:       # 测试直行并带偏航差速调整
            if dis_left != -999.9 and dis_right != -999.9:
                now_devision = dis_left - dis_right
            else:
                now_devision = 0.0
            if cam_yaw == -999.9 and dis_left == -999.9 and dis_right == -999.9:
                slow_num += 1
                if loop_num < 10:
                    now_speed = move_speed / 2
                else:
                    if slow_num < 6:
                        now_speed = move_speed
                    else:
                        now_speed = move_speed / 2
            else:
                slow_num = 0
                now_speed = move_speed

            if wash_thre_deviation < abs(now_devision):
                speed_left = now_speed
                speed_right = now_speed
                if now_devision < 0:
                    if now_yaw != -999.9:
                        if now_yaw < 10.0:
                            speed_left = now_speed + 20
                            speed_right = now_speed - 20
                        else:
                            speed_left = now_speed
                            speed_right = now_speed
                    else:
                        speed_left = now_speed
                        speed_right = now_speed
                else:
                    if now_yaw != -999.9:
                        if now_yaw > -10.0:
                            speed_left = now_speed - 20
                            speed_right = now_speed + 20
                        else:
                            speed_left = now_speed
                            speed_right = now_speed
                    else:
                        speed_left = now_speed
                        speed_right = now_speed

                if speed_left < 0:
                    speed_left = 0
                elif speed_left > 100:
                    speed_left = 100
                if speed_right < 0:
                    speed_right = 0
                elif speed_right > 100:
                    speed_right = 100
                ctl_go = [
                    ATTR_CONTROL_CATERPILLA_LEFT, 2, int(speed_left),  # 左履带
                    ATTR_CONTROL_CATERPILLA_RIGHT, 2, int(speed_right),  # 右履带
                ]
            elif wash_thre_yaw < abs(now_yaw):
                speed_left = now_speed - int(round(now_yaw * 2, 0))
                speed_right = now_speed + int(round(now_yaw * 2, 0))
                if speed_left < 0:
                    speed_left = 0
                elif speed_left > 100:
                    speed_left = 100

                if speed_right < 0:
                    speed_right = 0
                elif speed_right > 100:
                    speed_right = 100
                ctl_go = [
                    ATTR_CONTROL_CATERPILLA_LEFT, 2, int(speed_left),  # 左履带
                    ATTR_CONTROL_CATERPILLA_RIGHT, 2, int(speed_right),  # 右履带
                ]
            else:
                ctl_go = [
                    ATTR_CONTROL_CATERPILLA_LEFT, 2, int(now_speed),  # 左履带
                    ATTR_CONTROL_CATERPILLA_RIGHT, 2, int(now_speed),  # 右履带
                ]
        elif move_type == 5:       # Stop

            ctl_go = [
                ATTR_CONTROL_CATERPILLA_LEFT, 0, 0,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 0, 0,  # 右履带
            ]

        get_state = control_communication(ctl_go, qin, qout, q_ci)
        if get_state > 90:
            get_error = control_communication(ctl_allStop, qin, qout, q_ci)
            return get_state

        # 前向到边，除去退后状态
        if get_state == 1 and (move_type == 0 or move_type == 4):
            print('Board')
            get_error = control_communication(ctl_stop, qin, qout, q_ci)
            get_state = 1
            break
        elif get_state == 2 and move_type == 3:
            print('Back Board')
            get_error = control_communication(ctl_stop, qin, qout, q_ci)
            get_state = 2
            break

        if loop_num > loop_times and loop_times != -1:
            get_error = control_communication(ctl_stop, qin, qout, q_ci)
            get_state = 0
            break

        time.sleep(0.2)

    get_stop = control_communication(ctl_stop, qin, qout, q_ci)
    if get_state > 90:
        get_error = control_communication(ctl_allStop, qin, qout, q_ci)
        return get_state

    return get_state


def clean_water(clean_type, water_type, qin, qout, q_ci):
    if clean_type == 0:
        clean_value = 0
    else:
        clean_value = 100

    if water_type == 0:
        water_value = 0
    else:
        water_value = 100

    ctl_clean_water = [
        ATTR_CONTROL_CLEAN, clean_type, clean_value,
        ATTR_CONTROL_WATER, water_type, water_value,
    ]

    # 发送命令
    qout.put(ctl_clean_water)
    qout.get() if qout.qsize() > 1 else time.sleep(0.001)

    time.sleep(0.1)

    # 等待反馈
    if not qin.empty():
        sensor_list = qin.get()
        # print('Get Ret', sensor_list, len(sensor_list))
        if len(sensor_list) == 1:
            print(sensor_list)
            get_state = 98
            return get_state
        elif len(sensor_list) > 1:
            # 读取前向测距
            laser_front_id = find_id(sensor_list, ATTR_STATE_DISTANCE_U)
            laser_front_dis = sensor_list[laser_front_id][1]

            # 读取后向测距
            laser_back_id = find_id(sensor_list, ATTR_STATE_DISTANCE_D)
            laser_back_dis = sensor_list[laser_back_id][1]

            # 读取防跌落传感器
            falling_id = find_id(sensor_list, ATTR_STATE_TOUCH)
            if sensor_list[falling_id][1] != 0xff:
                # int转4位二进制
                temp_state = sensor_list[falling_id][2]
                falling_state = format(temp_state, "b")
                if len(falling_state) < 4:
                    falling_state = '0' * (4 - len(falling_state)) + falling_state

                # 判断4个防跌落状态
                if falling_state[0:1] == '0':
                    fall_BackRight = False
                elif falling_state[0:1] == '1':
                    fall_BackRight = True
                else:
                    fall_BackRight = False
                if falling_state[1:2] == '0':
                    fall_BackLeft = False
                elif falling_state[1:2] == '1':
                    fall_BackLeft = True
                else:
                    fall_BackLeft = False
                if falling_state[2:3] == '0':
                    fall_FrontRight = False
                elif falling_state[2:3] == '1':
                    fall_FrontRight = True
                else:
                    fall_FrontRight = False
                if falling_state[3:4] == '0':
                    fall_FrontLeft = False
                elif falling_state[3:4] == '1':
                    fall_FrontLeft = True
                else:
                    fall_FrontLeft = False

                print(fall_FrontLeft, fall_BackLeft, fall_FrontRight, fall_BackRight)

                if (not fall_BackLeft and not fall_BackRight) and (fall_FrontLeft and fall_FrontRight) or laser_front_dis < 50:
                    get_state = 1
                elif (not fall_FrontLeft and not fall_FrontRight) and (fall_BackLeft and fall_BackRight) or laser_back_dis < 50:
                    get_state = 2
                elif (not fall_FrontRight and not fall_BackRight) and (fall_FrontLeft and fall_BackLeft):
                    get_state = 3
                elif (not fall_FrontLeft and not fall_BackLeft) and (fall_FrontRight and fall_BackRight):
                    get_state = 4
                elif not fall_FrontLeft and not fall_BackLeft and not fall_FrontRight and not fall_BackRight:
                    get_state = 0
                else:
                    get_state = 5
            else:
                falling_state = sensor_list[falling_id][2]

            # if falling_state == 0xfd and laser_front_dis == 0xfffd and laser_back_dis == 0xfffd:
            #     get_state = 98

            print(laser_front_dis, laser_back_dis, falling_state, get_state)
    else:
        time.sleep(0.05)
        if not qin.empty():
            sensor_list = qin.get()
            if len(sensor_list) == 1:
                print(sensor_list)
                get_state = 98
                return get_state
            elif len(sensor_list) > 1:
                # 读取前向测距
                laser_front_id = find_id(sensor_list, ATTR_STATE_DISTANCE_U)
                laser_front_dis = sensor_list[laser_front_id][1]

                # 读取后向测距
                laser_back_id = find_id(sensor_list, ATTR_STATE_DISTANCE_D)
                laser_back_dis = sensor_list[laser_back_id][1]

                # 读取防跌落传感器
                falling_id = find_id(sensor_list, ATTR_STATE_TOUCH)
                if sensor_list[falling_id][1] != 0xff:
                    # int转4位二进制
                    temp_state = sensor_list[falling_id][2]
                    falling_state = format(temp_state, "b")
                    if len(falling_state) < 4:
                        falling_state = '0' * (4 - len(falling_state)) + falling_state

                    # 判断4个防跌落状态
                    if falling_state[0:1] == '0':
                        fall_BackRight = False
                    elif falling_state[0:1] == '1':
                        fall_BackRight = True
                    else:
                        fall_BackRight = False
                    if falling_state[1:2] == '0':
                        fall_BackLeft = False
                    elif falling_state[1:2] == '1':
                        fall_BackLeft = True
                    else:
                        fall_BackLeft = False
                    if falling_state[2:3] == '0':
                        fall_FrontRight = False
                    elif falling_state[2:3] == '1':
                        fall_FrontRight = True
                    else:
                        fall_FrontRight = False
                    if falling_state[3:4] == '0':
                        fall_FrontLeft = False
                    elif falling_state[3:4] == '1':
                        fall_FrontLeft = True
                    else:
                        fall_FrontLeft = False

                    print(fall_FrontLeft, fall_BackLeft, fall_FrontRight, fall_BackRight)

                    if (not fall_BackLeft and not fall_BackRight) and (
                            fall_FrontLeft and fall_FrontRight) or laser_front_dis < 50:
                        get_state = 1
                    elif (not fall_FrontLeft and not fall_FrontRight) and (
                            fall_BackLeft and fall_BackRight) or laser_back_dis < 50:
                        get_state = 2
                    elif (not fall_FrontRight and not fall_BackRight) and (fall_FrontLeft and fall_BackLeft):
                        get_state = 3
                    elif (not fall_FrontLeft and not fall_BackLeft) and (fall_FrontRight and fall_BackRight):
                        get_state = 4
                    elif not fall_FrontLeft and not fall_BackLeft and not fall_FrontRight and not fall_BackRight:
                        get_state = 0
                    else:
                        get_state = 5
                else:
                    falling_state = sensor_list[falling_id][2]

                # if falling_state == 0xfd and laser_front_dis == 0xfffd and laser_back_dis == 0xfffd:
                #     get_state = 98

                print(laser_front_dis, laser_back_dis, falling_state, get_state)

    return get_state

def ReRead():
    global para_lines
    # 读取参数
    file_model_reread = open('./Parameters.txt', 'r', encoding='utf-8')
    para_lines = file_model_reread.readlines()
    file_model_reread.close()
    # 机身尺寸
    vehicle_left = int(para_lines[0].strip('\n'))
    vehicle_right = int(para_lines[1].strip('\n'))
    vehicle_front = int(para_lines[2].strip('\n'))
    vehicle_front_falling = int(para_lines[3].strip('\n'))
    vehicle_width = vehicle_left + vehicle_right
    # 校正阈值(度或毫米)
    wash_thre_yaw = float(para_lines[4].strip('\n'))
    correct_thre_yaw = float(para_lines[5].strip('\n'))
    wash_thre_deviation = float(para_lines[6].strip('\n'))
    correct_thre_deviation = float(para_lines[7].strip('\n'))
    # 机器速率
    straight_speed = float(para_lines[8].strip('\n'))
    angular_speed = float(para_lines[9].strip('\n'))
    # 运行速度
    int_washSpeed = int(para_lines[10].strip('\n'))
    int_moveSpeed = int(para_lines[11].strip('\n'))
    int_turnSpeed = int(para_lines[12].strip('\n'))
    # Times
    times_Lturn = int(para_lines[13].strip('\n'))
    times_GoBack = int(para_lines[14].strip('\n'))
    times_ChangeLine = int(para_lines[15].strip('\n'))
    times_Uturn = int(para_lines[16].strip('\n'))


# 执行自动控制
def autocontrol_run(q_v2a, q_i2a, qin, qout, q_ci):
    global rec_side_left, rec_side_right, ctl_order, ctl_allStop, para_lines
    # 机身尺寸
    vehicle_left = int(para_lines[0].strip('\n'))
    vehicle_right = int(para_lines[1].strip('\n'))
    vehicle_front = int(para_lines[2].strip('\n'))
    vehicle_front_falling = int(para_lines[3].strip('\n'))
    vehicle_width = vehicle_left + vehicle_right
    # 校正阈值(度或毫米)
    wash_thre_yaw = float(para_lines[4].strip('\n'))
    correct_thre_yaw = float(para_lines[5].strip('\n'))
    wash_thre_deviation = float(para_lines[6].strip('\n'))
    correct_thre_deviation = float(para_lines[7].strip('\n'))
    # 机器速率
    straight_speed = float(para_lines[8].strip('\n'))
    angular_speed = float(para_lines[9].strip('\n'))
    # 运行速度
    int_washSpeed = int(para_lines[10].strip('\n'))
    int_moveSpeed = int(para_lines[11].strip('\n'))
    int_turnSpeed = int(para_lines[12].strip('\n'))
    # Times
    times_Lturn = int(para_lines[13].strip('\n'))
    times_GoBack = int(para_lines[14].strip('\n'))
    times_ChangeLine = int(para_lines[15].strip('\n'))
    times_Uturn = int(para_lines[16].strip('\n'))


    # 设置清洗参数
    loop_time = -1  # 单向算作1次
    wash_loops = -1  # 设置前后清洗的移动距离限定，-1代表无限定。每次200ms，5次相当于1秒
    wash_left_dis = 560.0  # 左侧边界距离
    wash_right_dis = 560.0  # 右侧边界距离

    need_back = False

    cam_yaw = dis_left = dis_right = side_l = side_r = imu_roll = imu_pitch = imu_yaw = -999.9
    num_front = 0
    dis_front = [-999.9]

    num_order = 0
    times_input = 0

    str_vision = ''
    str_imu = ''

    left_id = ctl_order.index(ATTR_CONTROL_CATERPILLA_LEFT)
    right_id = ctl_order.index(ATTR_CONTROL_CATERPILLA_RIGHT)
    adhesion_id = ctl_order.index(ATTR_CONTROL_ADHESION)
    clean_id = ctl_order.index(ATTR_CONTROL_CLEAN)
    water_id = ctl_order.index(ATTR_CONTROL_WATER)
    state_id = ctl_order.index(ATTR_STATE_VISION_STATE)

    # 无反馈重新循环标志位
    no_feedBack = False
    while True:
        no_feedBack = False
        is_oneAction = True

        wait_imu = True
        wait_vision = True
        wait_com = True

        ReRead()

        while wait_imu:
            time.sleep(0.5)
            if not q_i2a.empty():
                print('Init IMU')
                wait_imu = False
            else:
                print('Wait IMU')

        while wait_vision:
            time.sleep(1)
            if not q_v2a.empty():
                print('Init Vision')
                wait_vision = False
            else:
                print('Wait Vision')

        while wait_com:
            time.sleep(0.5)
            if not qin.empty():
                com_msg = qin.get()
                if len(com_msg) == 1:
                    com_str = com_msg[0][0]
                    if com_str == 'offline':
                        print('offline')
                    else:
                        print('One word error')
                else:
                    print('Init Communication')
                    wait_com = False

            else:
                print('No message')


        while is_oneAction:
            wait_begin = True

            while wait_begin:
                time.sleep(0.5)
                if not qin.empty():
                    com_msg = qin.get()
                    # print(com_msg)
                    if len(com_msg) == 1:
                        com_str = com_msg[0][0]
                        if com_str == 'offline':
                            is_oneAction = False
                            break
                        else:
                            print('One word error')
                    else:
                        work_id = find_id(com_msg, ATTR_CONTROL_UPDOWN)
                        if work_id != -1:
                            work_state = com_msg[work_id][1]
                            if work_state == 0:
                                print('Stop and Wait')
                            elif work_state == 2:
                                print('Begin Autorun')
                                wait_begin = False
                            else:
                                print('State error')
                else:
                    print('No message')

            is_left = True
            is_loop = True
            print('Open clean')
            ret_state = clean_water(2, 0, qin, qout, q_ci)
            if ret_state == 98:
                print('Out of communication')
                is_oneAction = False
                break

            print('Stop')
            time.sleep(0.1)
            ret_state = go_straight(5, 0, 3, q_v2a, qin, q_i2a, qout, q_ci)
            if ret_state == 98:
                print('Out of communication')
                is_oneAction = False
                break

            print('开始直行到边')
            ret_state = go_straight(0, int_moveSpeed, -1, q_v2a, qin, q_i2a, qout, q_ci)
            if ret_state == 98:
                print('Out of communication')
                is_oneAction = False
                break

            print('Stop')
            time.sleep(0.1)
            ret_state = go_straight(5, 0, 3, q_v2a, qin, q_i2a, qout, q_ci)
            if ret_state == 98:
                print('Out of communication')
                is_oneAction = False
                break

            print('开始后退')
            time.sleep(0.1)
            ret_state = go_straight(3, int_moveSpeed, times_GoBack, q_v2a, qin, q_i2a, qout, q_ci)
            if ret_state == 98:
                print('Out of communication')
                is_oneAction = False
                break

            print('Stop')
            time.sleep(0.1)
            ret_state = go_straight(5, 0, 3, q_v2a, qin, q_i2a, qout, q_ci)
            if ret_state == 98:
                print('Out of communication')
                is_oneAction = False
                break

            print('开始L型Left')
            time.sleep(0.1)
            ret_state = l_turn(is_left, q_v2a, qin, q_i2a, qout, q_ci)
            if ret_state == 98:
                print('Out of communication')
                is_oneAction = False
                break

            print('Stop')
            time.sleep(0.1)
            ret_state = go_straight(5, 0, 3, q_v2a, qin, q_i2a, qout, q_ci)
            if ret_state == 98:
                print('Out of communication')
                is_oneAction = False
                break

            print('开始Change Line直行')
            time.sleep(0.1)
            ret_state = go_straight(0, int_moveSpeed, times_ChangeLine, q_v2a, qin, q_i2a, qout, q_ci)
            if ret_state == 98:
                print('Out of communication')
                is_oneAction = False
                break

            print('Stop')
            time.sleep(0.1)
            ret_state = go_straight(5, 0, 3, q_v2a, qin, q_i2a, qout, q_ci)
            if ret_state == 98:
                print('Out of communication')
                is_oneAction = False
                break

            if ret_state == 0:
                is_left = True
            elif ret_state == 1:
                is_left = False

                print('开始后退')
                time.sleep(0.1)
                ret_state = go_straight(3, int_moveSpeed, times_GoBack, q_v2a, qin, q_i2a, qout, q_ci)
                if ret_state == 98:
                    print('Out of communication')
                    is_oneAction = False
                    break

                print('Stop')
                time.sleep(0.1)
                ret_state = go_straight(5, 0, 3, q_v2a, qin, q_i2a, qout, q_ci)
                if ret_state == 98:
                    print('Out of communication')
                    is_oneAction = False
                    break

                print('开始Turn Around')
                ret_state = turn_around(True, q_v2a, qin, q_i2a, qout, q_ci)
                if ret_state == 98:
                    print('Out of communication')
                    is_oneAction = False
                    break

                print('Stop')
                time.sleep(0.1)
                ret_state = go_straight(5, 0, 3, q_v2a, qin, q_i2a, qout, q_ci)
                if ret_state == 98:
                    print('Out of communication')
                    is_oneAction = False
                    break

                print('开始直行')
                time.sleep(0.1)
                ret_state = go_straight(0, int_moveSpeed, times_ChangeLine, q_v2a, qin, q_i2a, qout, q_ci)
                if ret_state == 98:
                    print('Out of communication')
                    is_oneAction = False
                    break

                print('Stop')
                time.sleep(0.1)
                ret_state = go_straight(5, 0, 3, q_v2a, qin, q_i2a, qout, q_ci)
                if ret_state == 98:
                    print('Out of communication')
                    is_oneAction = False
                    break

            print('开始L型Left')
            time.sleep(0.1)
            ret_state = l_turn(is_left, q_v2a, qin, q_i2a, qout, q_ci)
            if ret_state == 98:
                print('Out of communication')
                is_oneAction = False
                break

            print('Stop')
            time.sleep(0.1)
            ret_state = go_straight(5, 0, 3, q_v2a, qin, q_i2a, qout, q_ci)
            if ret_state == 98:
                print('Out of communication')
                is_oneAction = False
                break

            while is_loop:
                if is_left:
                    is_left = False
                else:
                    is_left = True

                print('开始直行到边')
                time.sleep(0.1)
                ret_state = go_straight(0, int_moveSpeed, -1, q_v2a, qin, q_i2a, qout, q_ci)
                if ret_state == 98:
                    print('Out of communication')
                    is_oneAction = False
                    break

                print('Stop')
                time.sleep(0.1)
                ret_state = go_straight(5, 0, 3, q_v2a, qin, q_i2a, qout, q_ci)
                if ret_state == 98:
                    print('Out of communication')
                    is_oneAction = False
                    break

                print('开始后退')
                time.sleep(0.1)
                ret_state = go_straight(3, int_moveSpeed, times_GoBack, q_v2a, qin, q_i2a, qout, q_ci)
                if ret_state == 98:
                    print('Out of communication')
                    is_oneAction = False
                    break

                print('Stop')
                time.sleep(0.1)
                ret_state = go_straight(5, 0, 3, q_v2a, qin, q_i2a, qout, q_ci)
                if ret_state == 98:
                    print('Out of communication')
                    is_oneAction = False
                    break

                print('开始L型')
                time.sleep(0.1)
                ret_state = l_turn(is_left, q_v2a, qin, q_i2a, qout, q_ci)
                if ret_state == 98:
                    print('Out of communication')
                    is_oneAction = False
                    break

                print('Stop')
                time.sleep(0.1)
                ret_state = go_straight(5, 0, 3, q_v2a, qin, q_i2a, qout, q_ci)
                if ret_state == 98:
                    print('Out of communication')
                    is_oneAction = False
                    break

                print('开始直行')
                ret_state = go_straight(0, int_moveSpeed, times_ChangeLine, q_v2a, qin, q_i2a, qout, q_ci)
                if ret_state == 98:
                    print('Out of communication')
                    is_oneAction = False
                    break

                print('Stop')
                time.sleep(0.1)
                ret_state = go_straight(5, 0, 3, q_v2a, qin, q_i2a, qout, q_ci)
                if ret_state == 98:
                    print('Out of communication')
                    is_oneAction = False
                    break

                if ret_state == 1:
                    print('开始后退')
                    time.sleep(0.1)
                    ret_state = go_straight(3, int_moveSpeed, times_GoBack, q_v2a, qin, q_i2a, qout, q_ci)
                    if ret_state == 98:
                        print('Out of communication')
                        is_oneAction = False
                        break

                    print('Stop')
                    time.sleep(0.1)
                    ret_state = go_straight(5, 0, 3, q_v2a, qin, q_i2a, qout, q_ci)
                    if ret_state == 98:
                        print('Out of communication')
                        is_oneAction = False
                        break

                    print('开始L型')
                    time.sleep(0.1)
                    ret_state = l_turn(is_left, q_v2a, qin, q_i2a, qout, q_ci)
                    if ret_state == 98:
                        print('Out of communication')
                        is_oneAction = False
                        break

                    print('Stop')
                    time.sleep(0.1)
                    ret_state = go_straight(5, 0, 3, q_v2a, qin, q_i2a, qout, q_ci)
                    if ret_state == 98:
                        print('Out of communication')
                        is_oneAction = False
                        break
                    is_loop = False
                    break

                print('开始L型')
                time.sleep(0.1)
                ret_state = l_turn(is_left, q_v2a, qin, q_i2a, qout, q_ci)
                if ret_state == 98:
                    print('Out of communication')
                    is_oneAction = False
                    break

                print('Stop')
                time.sleep(0.1)
                ret_state = go_straight(5, 0, 3, q_v2a, qin, q_i2a, qout, q_ci)
                if ret_state == 98:
                    print('Out of communication')
                    is_oneAction = False
                    break

            print('Close clean')
            ret_state = clean_water(0, 0, qin, qout, q_ci)
            if ret_state == 98:
                print('Out of communication')
                is_oneAction = False
                break

            print('Finish')
            if is_oneAction:
                wait_begin = True
                while wait_begin:
                    if not qin.empty():
                        com_msg = qin.get()
                        # print(com_msg)
                        if len(com_msg) == 1:
                            com_str = com_msg[0][0]
                            if com_str == 'offline':
                                is_oneAction = False
                                wait_begin = False
                            else:
                                print('One word error')
                        else:
                            work_id = find_id(com_msg, ATTR_CONTROL_UPDOWN)
                            if work_id != -1:
                                work_state = com_msg[work_id][1]
                                if work_state == 0:
                                    print('Next loop')
                                    wait_begin = False
                                elif work_state == 2:
                                    print('Wait Stop')
                                else:
                                    print('State error')
                    else:
                        print('No message')
                    time.sleep(0.5)

        #
        # # 3.进入自动控制
        # print('进入自动清洗')
        # loop_num = -1
        # while (not need_back) and (loop_num < loop_time or loop_time == -1):
        #     loop_num += 1
        #
        #     # 边距记录清零
        #     rec_side_right = [wash_right_dis]
        #     rec_side_left = [wash_left_dis]
        #     now_yaw = 0.0
        #
        #     # 3.1. 如果偏移或偏航大于校正阈值，进行校正
        #     # 静止获取左右任意一边距离及偏航角。
        #     for i in range(0, 3, 1):
        #         get_state = control_communication(ctl_stop, qin, qout, q_ci)
        #
        #         time.sleep(0.2)
        #
        #         if not q_v2a.empty():
        #             dis_list = q_v2a.get()
        #             cam_yaw = dis_list[0]
        #             dis_left = dis_list[1]
        #             dis_right = dis_list[2]
        #             # num_front = dis_list[3]
        #             # print(dis_list)
        #
        #         if not q_i2a.empty():
        #             imu_roll, imu_pitch, imu_yaw = q_i2a.get()
        #
        #         if dis_left != -999.9:
        #             rec_side_left.append(dis_left)
        #         if dis_right != -999.9:
        #             rec_side_right.append(dis_right)
        #         if cam_yaw != -999.9:
        #             now_yaw = cam_yaw
        #     # 累计左侧距离取平均
        #     if len(rec_side_left) > 1:
        #         temp_sum = 0.0
        #         for i in range(1, len(rec_side_left), 1):
        #             temp_sum += rec_side_left[i]
        #         side_l = temp_sum / (len(rec_side_left) - 1)
        #     else:
        #         side_l = -999.9
        #     # 累计右侧距离取平均
        #     if len(rec_side_right) > 1:
        #         temp_sum = 0.0
        #         for i in range(1, len(rec_side_right), 1):
        #             temp_sum += rec_side_right[i]
        #         side_r = temp_sum / (len(rec_side_right) - 1)
        #     else:
        #         side_r = -999.9
        #
        #     # 根据识别情况确认是否需要偏移校正或偏航校正
        #     if side_l != -999.9 or side_r != -999.9:
        #         if abs(now_yaw) > correct_thre_yaw:
        #             print('偏航角度' + str(now_yaw))
        #             get_state = correct_yaw(now_yaw, 0.0, q_v2a, qin, q_i2a, qout, q_ci)
        #             # 无反馈退出
        #             if get_state == 98:
        #                 print('无反馈，结束运行')
        #                 no_feedBack = True
        #             elif get_state == 96:
        #                 print('手动急停')
        #                 no_feedBack = True
        #             elif get_state > 0:
        #                 print('故障')
        #                 no_feedBack = True
        #                 break
        #             print('偏航校正完成')
        #         else:
        #             print('偏移/偏航在范围内')
        #     else:
        #         print('未找到边界')
        #
        #     print('开始直行到边')
        #     get_state = go_straight(0, int_moveSpeed, -1, q_v2a, qin, q_i2a, qout, q_ci)
        #     print('开始后退')
        #     get_state = go_straight(3, int_moveSpeed, times_GoBack, q_v2a, qin, q_i2a, qout, q_ci)
        #     print('开始L型Left')
        #     get_state = l_turn(True, q_v2a, qin, queue_imu2action, qout, q_ci)
        #     print('开始直行')
        #     get_state = go_straight(0, int_moveSpeed, times_ChangeLine, q_v2a, qin, q_i2a, qout, q_ci)
        #     print('开始L型Left')
        #     get_state = l_turn(True, q_v2a, qin, q_i2a, qout, q_ci)
        #     print('开始直行到边')
        #     get_state = go_straight(0, int_moveSpeed, -1, q_v2a, qin, q_i2a, qout, q_ci)
        #     print('开始后退')
        #     get_state = go_straight(3, int_moveSpeed, times_GoBack, q_v2a, qin, q_i2a, qout, q_ci)
        #     print('开始L型Left')
        #     get_state = l_turn(True, q_v2a, qin, q_i2a, qout, q_ci)
        #     print('开始直行')
        #     get_state = go_straight(0, int_moveSpeed, times_ChangeLine, q_v2a, qin, q_i2a, qout, q_ci)
        #     print('开始L型Left')
        #     get_state = l_turn(True, q_v2a, qin, q_i2a, qout, q_ci)



        # # 2.初始化
        # print('进入初始化')
        # is_leftEdge = True
        # dis_back = vehicle_width / 2 - vehicle_front_falling
        # times_moveBack = int(round(dis_back / (int_moveSpeed * straight_speed), 0))
        # now_yaw = -999.9
        # now_pitch = -999.9
        # now_roll = -999.9
        # width_vertical = -999.9
        # width_horizon = -999.9
        # is_singleLine = True
        #
        # # 2.0 静止获取姿态，判断是在斜面还是平面
        # for i in range(0, 3, 1):
        #     get_state, cam_yaw, dis_left, dis_right, imu_roll, imu_pitch, imu_yaw = control_communication(
        #         hex_allStop, q_v2a, qin, q_i2a, qout, q_ci)
        #     if get_state == 98:
        #         print('无反馈，结束运行')
        #         no_feedBack = True
        #         break
        #     elif get_state == 96:
        #         print('手动急停')
        #         no_feedBack = True
        #         break
        #     elif get_state > 0:
        #         print('故障')
        #         no_feedBack = True
        #         break
        #     if cam_yaw != -999.9:
        #         now_yaw = cam_yaw
        #     if imu_pitch != 0.0:
        #         now_pitch = imu_pitch
        #     if imu_roll != 0.0:
        #         now_roll = imu_roll
        #
        # if abs(now_pitch) + abs(now_roll) > 4.0:    # 如果是运行在斜面
        #     print('斜面')
        #     # 2.1 根据俯仰角，选择动作转至水平向右
        #     if abs(now_pitch) > abs(now_roll):      # 向上/向下
        #         if now_pitch > 0:       # 如果向上，向右转
        #             get_state = l_turn(False, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #         else:                   # 如果向下，向左转
        #             get_state = l_turn(True, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #     else:       # 向左/向右
        #         if now_roll > 0:        # 如果向左，调头
        #             get_state = turn_around(cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #         else:                   # 如果向右，则拉直
        #             get_state = correct_yaw(now_yaw, 0.0, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #
        #     now_yaw = -999.9
        #     now_left = -999.9
        #     now_right = -999.9
        #     for i in range(0, 3, 1):
        #         get_state, cam_yaw, dis_left, dis_right, imu_roll, imu_pitch, imu_yaw = control_communication(
        #             hex_allStop, q_v2a, qin, q_i2a, qout, q_ci)
        #
        #         if cam_yaw != -999.9:
        #             if now_yaw == -999.9:
        #                 now_yaw = cam_yaw
        #             else:
        #                 now_yaw = (now_yaw + cam_yaw) / 2
        #         if dis_left != -999.9:
        #             if now_left == -999.9:
        #                 now_left = dis_left
        #             else:
        #                 now_left = (now_left + dis_left) / 2
        #         if dis_right != -999.9:
        #             if now_right == -999.9:
        #                 now_right = dis_right
        #             else:
        #                 now_right = (now_right + dis_right) / 2
        #
        #     # 2.2 如果能识别到左右距离，则判断右侧未到边，读取水平行驶的宽度。否则判断右侧为边，调头后读取水平行驶的宽度
        #     if now_left != -999.9 and now_right != -999.9:
        #         width_horizon = now_left + now_right
        #         is_leftEdge = True
        #     else:
        #         is_leftEdge = False
        #         get_state = turn_around(cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #
        #         now_yaw = -999.9
        #         now_left = -999.9
        #         now_right = -999.9
        #         for i in range(0, 3, 1):
        #             get_state, cam_yaw, dis_left, dis_right, imu_roll, imu_pitch, imu_yaw = control_communication(
        #                 hex_allStop, q_v2a, qin, q_i2a, qout, q_ci)
        #
        #             if cam_yaw != -999.9:
        #                 if now_yaw == -999.9:
        #                     now_yaw = cam_yaw
        #                 else:
        #                     now_yaw = (now_yaw + cam_yaw) / 2
        #             if dis_left != -999.9:
        #                 if now_left == -999.9:
        #                     now_left = dis_left
        #                 else:
        #                     now_left = (now_left + dis_left) / 2
        #             if dis_right != -999.9:
        #                 if now_right == -999.9:
        #                     now_right = dis_right
        #                 else:
        #                     now_right = (now_right + dis_right) / 2
        #         if now_left != -999.9 and now_right != -999.9:
        #             width_horizon = now_left + now_right
        #         else:
        #             width_horizon = -999.9
        #
        #     # 2.3 旋转向上，读取垂直方向的宽度
        #     if is_leftEdge:
        #         get_state = l_turn(True, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #     else:
        #         get_state = l_turn(False, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #
        #     now_yaw = -999.9
        #     now_left = -999.9
        #     now_right = -999.9
        #     for i in range(0, 3, 1):
        #         get_state, cam_yaw, dis_left, dis_right, imu_roll, imu_pitch, imu_yaw = control_communication(
        #             hex_allStop, q_v2a, qin, q_i2a, qout, q_ci)
        #
        #         if cam_yaw != -999.9:
        #             if now_yaw == -999.9:
        #                 now_yaw = cam_yaw
        #             else:
        #                 now_yaw = (now_yaw + cam_yaw) / 2
        #         if dis_left != -999.9:
        #             if now_left == -999.9:
        #                 now_left = dis_left
        #             else:
        #                 now_left = (now_left + dis_left) / 2
        #         if dis_right != -999.9:
        #             if now_right == -999.9:
        #                 now_right = dis_right
        #             else:
        #                 now_right = (now_right + dis_right) / 2
        #     if now_left != -999.9 and now_right != -999.9:
        #         width_vertical = now_left + now_right
        #     else:
        #         width_vertical = -999.9
        #
        #     # 2.4 行驶到左右侧边界
        #     # 旋转朝向边界
        #     if is_leftEdge:
        #         get_state = l_turn(True, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #     else:
        #         get_state = l_turn(False, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #
        #     # 带校正直行直至到边
        #     get_state = go_straight(0, int_moveSpeed, -1, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #
        #     # 计算所需退后距离并退后
        #     times_moveBack = int(round((vehicle_width / 2) / (int_moveSpeed * straight_speed), 0))
        #     get_state = go_straight(3, int_moveSpeed, times_moveBack, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #
        #     # 2.5 行驶到顶角
        #     # 旋转朝向上
        #     if is_leftEdge:
        #         get_state = l_turn(False, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #     else:
        #         get_state = l_turn(True, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #
        #     # 带校正直行直至到边
        #     get_state = go_straight(0, int_moveSpeed, -1, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #
        #     # 计算所需退后距离并退后
        #     if width_horizon < vehicle_width:
        #         dis_back = width_horizon / 2 - vehicle_front_falling
        #     else:
        #         dis_back = width_horizon / 3 - vehicle_front_falling
        #     times_moveBack = int(round(dis_back / (int_moveSpeed * straight_speed), 0))
        #     get_state = go_straight(3, int_moveSpeed, times_moveBack, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #
        #     # 旋转朝向起始方向
        #     if is_leftEdge:
        #         get_state = l_turn(False, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #     else:
        #         get_state = l_turn(True, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #
        #     # 2.6 计算光伏板参数
        #     if width_horizon < vehicle_width:
        #         is_singleLine = True
        #         wash_left_dis = width_horizon / 2
        #         wash_right_dis = width_horizon / 2
        #     else:
        #         is_singleLine = False
        #         if is_leftEdge:
        #             wash_left_dis = width_horizon / 3
        #             wash_right_dis = width_horizon * 2 / 3
        #         else:
        #             wash_left_dis = width_horizon * 2 / 3
        #             wash_right_dis = width_horizon / 3
        #
        #     dis_back = vehicle_width / 2 - vehicle_front_falling
        #     times_moveBack = int(round(dis_back / (int_moveSpeed * straight_speed), 0))
        #
        # else:       # 如果是运行在平面
        #     print('平面')
        #     ret_4_sides = [0.0, 0.0, 0.0, 0.0]
        #     side_id = 0
        #
        #     # 2.1 识别四向数据
        #     if now_yaw != -999.9:
        #         get_state = correct_yaw(now_yaw, 0.0, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #
        #         now_yaw = -999.9
        #         now_left = -999.9
        #         now_right = -999.9
        #         for i in range(0, 3, 1):
        #             get_state, cam_yaw, dis_left, dis_right, imu_roll, imu_pitch, imu_yaw = control_communication(
        #                 hex_allStop, q_v2a, qin, q_i2a, qout, q_ci)
        #
        #             if cam_yaw != -999.9:
        #                 if now_yaw == -999.9:
        #                     now_yaw = cam_yaw
        #                 else:
        #                     now_yaw = (now_yaw + cam_yaw) / 2
        #             if dis_left != -999.9:
        #                 if now_left == -999.9:
        #                     now_left = dis_left
        #                 else:
        #                     now_left = (now_left + dis_left) / 2
        #             if dis_right != -999.9:
        #                 if now_right == -999.9:
        #                     now_right = dis_right
        #                 else:
        #                     now_right = (now_right + dis_right) / 2
        #
        #         if now_left != -999.9 and now_right != -999.9:
        #             ret_4_sides[0] = now_left + now_right
        #         else:
        #             ret_4_sides[0] = -999.9
        #
        #     else:
        #         ret_4_sides[0] = -999.9
        #
        #     for i in range(1, 4, 1):
        #         get_state = l_turn(True, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #
        #         now_yaw = -999.9
        #         now_left = -999.9
        #         now_right = -999.9
        #         for j in range(0, 3, 1):
        #             get_state, cam_yaw, dis_left, dis_right, imu_roll, imu_pitch, imu_yaw = control_communication(
        #                 hex_allStop, q_v2a, qin, q_i2a, qout, q_ci)
        #
        #             if cam_yaw != -999.9:
        #                 if now_yaw == -999.9:
        #                     now_yaw = cam_yaw
        #                 else:
        #                     now_yaw = (now_yaw + cam_yaw) / 2
        #             if dis_left != -999.9:
        #                 if now_left == -999.9:
        #                     now_left = dis_left
        #                 else:
        #                     now_left = (now_left + dis_left) / 2
        #             if dis_right != -999.9:
        #                 if now_right == -999.9:
        #                     now_right = dis_right
        #                 else:
        #                     now_right = (now_right + dis_right) / 2
        #
        #         if now_left != -999.9 and now_right != -999.9:
        #             ret_4_sides[i] = now_left + now_right
        #         else:
        #             ret_4_sides[i] = -999.9
        #
        #     # 2.2 指向最短边方向
        #     if ret_4_sides[0] != -999.9 and ret_4_sides[0] < vehicle_width:
        #         side_id = 0
        #         get_state = l_turn(True, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #     elif ret_4_sides[1] != -999.9 and ret_4_sides[1] < vehicle_width:
        #         side_id = 1
        #         get_state = turn_around(cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #     elif ret_4_sides[2] != -999.9 and ret_4_sides[2] < vehicle_width:
        #         side_id = 2
        #         get_state = l_turn(False, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #     elif ret_4_sides[3] != -999.9 and ret_4_sides[3] < vehicle_width:
        #         side_id = 3
        #
        #     # 2.3 整理平面参数
        #     width_horizon = ret_4_sides[side_id]
        #     wash_left_dis = width_horizon / 2
        #     wash_right_dis = width_horizon / 2
        #     is_singleLine = True
        #     if side_id == 0:
        #         if ret_4_sides[3] != -999.9 and ret_4_sides[1] == -999.9:
        #             is_leftEdge = False
        #         elif ret_4_sides[3] == -999.9 and ret_4_sides[1] != -999.9:
        #             is_leftEdge = True
        #         else:
        #             is_leftEdge = True
        #     elif side_id == 3:
        #         if ret_4_sides[2] != -999.9 and ret_4_sides[0] == -999.9:
        #             is_leftEdge = False
        #         elif ret_4_sides[2] == -999.9 and ret_4_sides[0] != -999.9:
        #             is_leftEdge = True
        #         else:
        #             is_leftEdge = True
        #     elif side_id == 1 or side_id == 2:
        #         if ret_4_sides[side_id - 1] != -999.9 and ret_4_sides[side_id + 1] == -999.9:
        #             is_leftEdge = False
        #         elif ret_4_sides[side_id - 1] == -999.9 and ret_4_sides[side_id + 1] != -999.9:
        #             is_leftEdge = True
        #         else:
        #             is_leftEdge = True
        #
        # # 3.进入自动控制
        # print('进入自动清洗')
        # loop_num = -1
        # while (not need_back) and (loop_num < loop_time or loop_time == -1):
        #     loop_num += 1
        #     print('左边距' + str(wash_left_dis) + ' 右边距' + str(wash_right_dis))
        #     # 边距记录清零
        #     rec_side_right = [wash_right_dis]
        #     rec_side_left = [wash_left_dis]
        #     now_yaw = 0.0
        #
        #     # 3.1. 如果偏移或偏航大于校正阈值，进行校正
        #     # 静止获取左右任意一边距离及偏航角。
        #     for i in range(0, 3, 1):
        #         get_state, cam_yaw, dis_left, dis_right, imu_roll, imu_pitch, imu_yaw = control_communication(
        #             hex_allStop, q_v2a, qin, q_i2a, qout, q_ci)
        #         if get_state == 98:
        #             print('无反馈，结束运行')
        #             no_feedBack = True
        #             break
        #         elif get_state == 96:
        #             print('手动急停')
        #             no_feedBack = True
        #             break
        #         elif get_state > 0:
        #             print('故障')
        #             no_feedBack = True
        #             break
        #         if dis_left != -999.9:
        #             rec_side_left.append(dis_left)
        #         if dis_right != -999.9:
        #             rec_side_right.append(dis_right)
        #         if cam_yaw != -999.9:
        #             now_yaw = cam_yaw
        #     # 累计左侧距离取平均
        #     if len(rec_side_left) > 1:
        #         temp_sum = 0.0
        #         for i in range(1, len(rec_side_left), 1):
        #             temp_sum += rec_side_left[i]
        #         side_l = temp_sum / (len(rec_side_left) - 1)
        #     else:
        #         side_l = -999.9
        #     # 累计右侧距离取平均
        #     if len(rec_side_right) > 1:
        #         temp_sum = 0.0
        #         for i in range(1, len(rec_side_right), 1):
        #             temp_sum += rec_side_right[i]
        #         side_r = temp_sum / (len(rec_side_right) - 1)
        #     else:
        #         side_r = -999.9
        #
        #     # 根据识别情况确认是否需要偏移校正或偏航校正
        #     if side_l != -999.9 or side_r != -999.9:
        #         # 优先判断距离近的边，如果没有再判断距离远的边
        #         dis_deviation = 0.0
        #         if wash_left_dis <= wash_right_dis:
        #             if side_l != -999.9 and abs(side_l - wash_left_dis) > 20:
        #                 dis_deviation = side_l - wash_left_dis
        #             elif side_r != -999.9 and abs(side_r - wash_right_dis) > 20:
        #                 dis_deviation = wash_right_dis - side_r
        #         else:
        #             if side_r != -999.9 and abs(side_r - wash_right_dis) > 20:
        #                 dis_deviation = wash_right_dis - side_r
        #             elif side_l != -999.9 and abs(side_l - wash_left_dis) > 20:
        #                 dis_deviation = side_l - wash_left_dis
        #         if abs(dis_deviation) > correct_thre_deviation:
        #             print('偏移距离' + str(dis_deviation))
        #             get_state = correct_deviation(dis_deviation, wash_left_dis, wash_right_dis, cmd_56, q_v2a, qin, qout,
        #                                           q_ci, file_address)
        #             # 无反馈退出
        #             if get_state == 98:
        #                 print('无反馈，结束运行')
        #                 no_feedBack = True
        #             elif get_state == 96:
        #                 print('手动急停')
        #                 no_feedBack = True
        #             elif get_state > 90:
        #                 print('故障')
        #                 no_feedBack = True
        #                 break
        #             elif get_state == 1:
        #                 print('前部到边')
        #             elif get_state == 2:
        #                 print('后部到边')
        #             elif get_state == 3:
        #                 print('防跌落故障')
        #             print('偏移校正完成')
        #         elif abs(now_yaw) > correct_thre_yaw:
        #             print('偏航角度' + str(now_yaw))
        #             get_state = correct_yaw(now_yaw, 0.0, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #             # 无反馈退出
        #             if get_state == 98:
        #                 print('无反馈，结束运行')
        #                 no_feedBack = True
        #             elif get_state == 96:
        #                 print('手动急停')
        #                 no_feedBack = True
        #             elif get_state > 0:
        #                 print('故障')
        #                 no_feedBack = True
        #                 break
        #             print('偏航校正完成')
        #         else:
        #             print('偏移/偏航在范围内')
        #     else:
        #         print('未找到边界')
        #     if no_feedBack:
        #         continue
        #
        #     # 3.2.清洗前进
        #     print('清洗前行，次数', str(wash_loops))
        #     get_state = go_wash(int_washSpeed, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address, wash_loops, wash_left_dis, wash_right_dis)
        #     if get_state == 98:
        #         print('无反馈，结束运行')
        #         no_feedBack = True
        #         break
        #     elif get_state == 96:
        #         print('手动急停')
        #         no_feedBack = True
        #         break
        #     elif get_state > 90:
        #         print('故障')
        #         no_feedBack = True
        #         break
        #     elif get_state == 2:
        #         print('后部到边')
        #         break
        #     elif get_state == 3:
        #         print('防跌落故障')
        #         break
        #     elif get_state == 1:
        #         print('前部到边')
        #         get_state = go_straight(3, int_moveSpeed, times_moveBack, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #         if get_state == 98:
        #             print('无反馈，结束运行')
        #             no_feedBack = True
        #             break
        #         elif get_state == 96:
        #             print('手动急停')
        #             no_feedBack = True
        #             break
        #         elif get_state > 90:
        #             print('故障')
        #             no_feedBack = True
        #             break
        #
        #
        #     # 3.3 到边平移
        #     # 旋转向下
        #     if loop_num % 2 == 0:
        #         if is_leftEdge:
        #             get_state = l_turn(False, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #         else:
        #             get_state = l_turn(True, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #     else:
        #         if is_leftEdge:
        #             get_state = l_turn(True, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #         else:
        #             get_state = l_turn(False, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #
        #     # 平移对应距离
        #     if is_singleLine:
        #         times_translate = int(round((width_horizon / (int_moveSpeed * straight_speed)), 0))
        #         get_state = go_straight(0, int_moveSpeed, times_translate, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #     else:
        #         if loop_num % 2 == 0:
        #             times_translate = int(round(((width_horizon / 3) / (int_moveSpeed * straight_speed)), 0))
        #         else:
        #             times_translate = int(round(((width_horizon * 2 / 3) / (int_moveSpeed * straight_speed)), 0))
        #         get_state = go_straight(0, int_moveSpeed, times_translate, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #
        #
        #     # 如果未到边界，继续旋转至下一行方向。如果已到边界，则停止
        #     if get_state == 0:
        #         if loop_num % 2 == 0:
        #             if is_leftEdge:
        #                 get_state = l_turn(False, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #             else:
        #                 get_state = l_turn(True, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #         else:
        #             if is_leftEdge:
        #                 get_state = l_turn(True, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #             else:
        #                 get_state = l_turn(False, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #     elif get_state == 1:
        #         get_state = go_straight(3, int_moveSpeed, times_moveBack, cmd_56, q_v2a, qin, q_i2a, qout, q_ci, file_address)
        #
        # print('自动循环结束')
        # need_back = False
        # if no_feedBack:
        #     continue
