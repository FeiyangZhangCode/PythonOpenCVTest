import crcmod
import serial
import time
import binascii

import CVFunc

ATTR_STATE_RSSI = hex(1<<5 | 0)[2:]   # 信号强度
ATTR_STATE_UPDOWN_LOAD = hex(1<<5 | 1)[2:]   # 提升负荷
ATTR_STATE_WATER = hex(1<<5 | 2)[2:]   # 液位
ATTR_STATE_BATTERY = hex(1<<5 | 3)[2:]   # 电量
ATTR_STATE_DIR_U = hex(1<<5 | 4)[2:]   # 转向上
ATTR_STATE_DIR_D = hex(1<<5 | 5)[2:]   # 转向下
ATTR_STATE_DISTANCE_U = hex(1<<5 | 6)[2:]   # 测距上
ATTR_STATE_DISTANCE_D = hex(1<<5 | 7)[2:]   # 测试下
ATTR_STATE_DISTANCE_L = hex(1<<5 | 8)[2:]   # 测距左
ATTR_STATE_DISTANCE_R = hex(1<<5 | 9)[2:]   # 测距右
ATTR_STATE_NTC = hex(1<<5 | 10)[2:]   # NTC温度
ATTR_STATE_TOUCH = hex(1<<5 | 11)[2:]   # 防跌落状态 STATE_ON: 至少有一个防跌落触发
ATTR_STATE_TIME_ONCE = hex(1<<5 | 12)[2:]   # 本次工作时长
ATTR_STATE_TIME_SUM = hex(1<<5 | 13)[2:]   # 累计工作时长

ATTR_CONTROL_MODE = hex(2<<5 | 0)[2:]   # 运行模式
ATTR_CONTROL_BOOST_STATE = hex(2<<5 | 1)[2:]   # 推进使能
ATTR_CONTROL_BOOST_VALUE = hex(2<<5 | 2)[2:]   # 推进力度
ATTR_CONTROL_ADHESION = hex(2<<5 | 3)[2:]   # 吸附
ATTR_CONTROL_UPDOWN = hex(2<<5 | 4)[2:]   # 上下行
ATTR_CONTROL_DIR = hex(2<<5 | 5)[2:]   # 转向
ATTR_CONTROL_CLEAN = hex(2<<5 | 6)[2:]   # 清洗系统
ATTR_CONTROL_WATER = hex(2<<5 | 7)[2:]   # 水循环系统
ATTR_CONTROL_AUTO_TYPE = hex(2<<5 | 8)[2:]   # 自动类型
ATTR_CONTROL_MOVE_PATH = hex(2<<5 | 9)[2:]   # 贴墙距离
ATTR_CONTROL_WATER_RECYCLE = hex(2<<5 | 10)[2:]   # 水回收
ATTR_CONTROL_WATER_HEAT = hex(2<<5 | 11)[2:]   # 水加热

ATTR_CONFIG_ADDR = hex(3<<5 | 0)[2:]   # 设备地址
ATTR_CONFIG_UD_RESTART = hex(3<<5 | 1)[2:]   # 提升重启
ATTR_CONFIG_OBSTACLE = hex(3<<5 | 2)[2:]   # 避障距离

ATTR_SPEED_UP = hex(4<<5 | 0)[2:]   # 速度 - 上行
ATTR_SPEED_DOWN = hex(4<<5 | 1)[2:]   # 速度 - 下行
ATTR_SPEED_ADHESION = hex(4<<5 | 2)[2:]   # 速度 - 吸附
ATTR_SPEED_STEAM = hex(4<<5 | 3)[2:]   # 速度 - 水汽
ATTR_SPEED_CLEAN = hex(4<<5 | 4)[2:]   # 速度 - 清洗
ATTR_SPEED_U_SPRAY1 = hex(4<<5 | 5)[2:]   # 速度 - 上行喷水1
ATTR_SPEED_U_SPRAY2 = hex(4<<5 | 6)[2:]   # 速度 - 上行喷水2
ATTR_SPEED_D_SPRAY1 = hex(4<<5 | 7)[2:]   # 速度 - 下行喷水1
ATTR_SPEED_D_SPRAY2 = hex(4<<5 | 8)[2:]   # 速度 - 下行喷水2



# 根据发送的数据，拆分控制信号和主板信号
def read_message(hex_rec):
    str_rec = binascii.b2a_hex(hex_rec).decode()
    is_normal = True

    length_rec = len(str_rec)

    if str_rec[0:2] == 'aa':
        ret_list = [[0, '']]
        if str_rec[2:4] == 'ff' or str_rec[2:4] == '0f':
            msg_rec = str_rec[8:length_rec - 2]
            str_msg = bytes.fromhex(msg_rec).decode()
            value_num = 0
            while len(str_msg) > 0:
                value_num += 1
                end_value = str_msg.find('#')
                take_msg = str_msg[0:end_value]
                end_id = take_msg.find(':')
                take_id = take_msg[0:end_id]
                take_value = take_msg[end_id + 1:]
                str_msg = str_msg[end_value + 1:]
                ret_list.append([int(take_id), take_value])
            ret_list[0][0] = value_num
            ret_list[0][1] = 'ff'
            print(ret_list)

        elif str_rec[2:4] == 'f0':
            pass
        elif str_rec[2:4] == 'f1':
            if str_rec[4:6] == '04':
                str_msg = str_rec[8:length_rec-2]
                for i in range(0, int(len(str_msg) / 3), 1):
                    ret_list.append([str_msg[i*3:i*3+1], str_msg[i*3+1:i*3+3]])
                ret_list[0][0] = int(len(str_msg) / 3)
                ret_list[0][1] = '04'
        elif str_rec[2:4] == 'f2':
            pass
        else:
            is_normal = False
            ret_list = [[0, 'fe']]
    else:
        is_normal = False
        ret_list = [[0, 'fe']]

    return is_normal, ret_list


if __name__ == '__main__':
    ser = serial.Serial('COM6', 115200, timeout=0.05)

    print('等待握手协议')

    no_feedback = True
    # 编制握手信息
    hex_message = bytes.hex(b'0:#1:#2:#3:#4:#5:#6:#')
    msg_size = hex(int(len(hex_message)))[2:]
    if len(hex_message) < 16:
        msg_size = '0' + msg_size
    head_HandShake = 'aa0fff' + msg_size
    hex_HandShake = bytes.fromhex(head_HandShake + hex_message)
    crc8 = crcmod.predefined.Crc('crc-8')
    crc8.update(hex_HandShake)
    if len(hex(crc8.crcValue)[2:]) == 1:
        hex_crc8 = bytes.fromhex('0' + hex(crc8.crcValue)[2:])
    else:
        hex_crc8 = bytes.fromhex(hex(crc8.crcValue)[2:])
    hex_HandShake = hex_HandShake + hex_crc8

    while no_feedback:
        ser.write(hex_HandShake)
        time.sleep(0.15)
        hex_receive = ser.readline()
        if hex_receive:
            str_receive = binascii.b2a_hex(hex_receive).decode('utf-8')
            if len(str_receive) > 10:
                is_normal, ret_list = read_message(hex_receive)
                if is_normal:
                    print(ret_list)
                    no_feedback = False
                else:
                    print('握手包读取错误')
            else:
                print('握手包过短')

    print('完成握手，获取配置')

    str_config_head = 'aa 01 04'

    int_config_len = 5 + (3 * 3)
    str_config_len = hex(int_config_len)[2:]
    if int_config_len < 16:
        str_config_size = '0' + str_config_len

    str_config_msg = ATTR_CONFIG_ADDR + '0000'
    str_config_msg += ATTR_CONFIG_UD_RESTART + '0000'
    str_config_msg += ATTR_CONFIG_OBSTACLE + '0000'

    hex_config = CVFunc.set_order(str_config_head + str_config_len + str_config_msg)

    no_feedback = True
    while no_feedback:
        ser.write(hex_config)
        time.sleep(0.15)
        hex_receive = ser.readline()
        if hex_receive:
            str_receive = binascii.b2a_hex(hex_receive).decode('utf-8')
            if len(str_receive) > 10:
                no_feedback = False
                is_normal, ret_list = read_message(hex_receive)
                if is_normal:
                    print(ret_list)
                    no_feedback = False
                else:
                    print('配置包读取错误')
            else:
                print('配置包过短')

    print('已获取配置，开始运行')
    # 状态查询语句
    str_state_head = 'aa 01 01'
    int_state_len = 5 + (4 * 3)
    str_state_len = hex(int_state_len)[2:]
    if int_state_len < 16:
        str_state_len = '0' + str_state_len
    str_state_msg = ATTR_STATE_BATTERY + '0000'
    str_state_msg += ATTR_STATE_DISTANCE_U + '0000'
    str_state_msg += ATTR_STATE_TOUCH + '0000'
    str_state_msg += ATTR_STATE_TIME_ONCE + '0000'
    hex_state = CVFunc.set_order(str_state_head + str_state_len + str_state_msg)
    # 控制设置语句
    str_control_head = 'aa 02 01'


    str_control_msg =

