import serial
import time
import threading
import queue
from pynput import keyboard
from pynput.keyboard import Listener
import binascii
from Feiyang import JY61
import G_Action
import G_RGBD


# 读取参数
file_model = open('./Parameters.txt', 'r', encoding='utf-8')
para_lines = file_model.readlines()
file_model.close()
# # 机身尺寸、光伏板尺寸
# vehicle_left = int(para_lines[0].strip('\n'))
# vehicle_right = int(para_lines[1].strip('\n'))
# vehicle_front = int(para_lines[2].strip('\n'))
# vehicle_front_falling = int(para_lines[3].strip('\n'))
# vehicle_width = vehicle_left + vehicle_right
# # 校正阈值(度或毫米)
# wash_thre_yaw = float(para_lines[4].strip('\n'))
# correct_thre_yaw = float(para_lines[5].strip('\n'))
# wash_thre_deviation = float(para_lines[6].strip('\n'))
# correct_thre_deviation = float(para_lines[7].strip('\n'))
# # 机器速率
# straight_speed = float(para_lines[8].strip('\n'))
# angular_speed = float(para_lines[9].strip('\n'))
# # 运行速度
# int_washSpeed = int(para_lines[10].strip('\n'))
# int_moveSpeed = int(para_lines[11].strip('\n'))
# int_turnSpeed = int(para_lines[12].strip('\n'))
# # Times
# times_Lturn = int(para_lines[13].strip('\n'))
# times_GoBack = int(para_lines[14].strip('\n'))
# times_ChangeLine = int(para_lines[15].strip('\n'))


imu_com = '/dev/ttyTHS1'
# imu_com = 'COM19'
imu_baud = 9600
imu_timeout = 0.05

# com_id = 'COM8'
# com_id = '/dev/ttyTHS1'
com_id = '/dev/ttyUSB0'
com_bit_rate = 115200
com_time_out = 0.2

recv_delay = 0.01
send_delay = 0.2
fail_cnt_thr = 5

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

ATTR_CONFIG_ADDR = (3 << 5 | 0)  # 设备地址
ATTR_CONFIG_UD_RESTART = (3 << 5 | 1)  # 提升重启
ATTR_CONFIG_OBSTACLE = (3 << 5 | 2)  # 避障距离

ATTR_SPEED_UP = (4 << 5 | 0)  # 速度 - 上行
ATTR_SPEED_DOWN = (4 << 5 | 1)  # 速度 - 下行
ATTR_SPEED_ADHESION = (4 << 5 | 2)  # 速度 - 吸附
ATTR_SPEED_STEAM = (4 << 5 | 3)  # 速度 - 水汽
ATTR_SPEED_CLEAN = (4 << 5 | 4)  # 速度 - 清洗
ATTR_SPEED_U_SPRAY1 = (4 << 5 | 5)  # 速度 - 上行喷水1
ATTR_SPEED_U_SPRAY2 = (4 << 5 | 6)  # 速度 - 上行喷水2
ATTR_SPEED_D_SPRAY1 = (4 << 5 | 7)  # 速度 - 下行喷水1
ATTR_SPEED_D_SPRAY2 = (4 << 5 | 8)  # 速度 - 下行喷水2

# 分开两个字节的消息
V_BASIC = [
    ATTR_STATE_RSSI,  # 信号强度
    ATTR_STATE_WATER,  # 液位
    ATTR_STATE_BATTERY,  # 电量
    ATTR_STATE_DIR_U,  # 转向上
    ATTR_STATE_DIR_D,  # 转向下
    ATTR_STATE_NTC,  # NTC温度
    ATTR_STATE_TOUCH,  # 防跌落状态 STATE_ON: 至少有一个防跌落触发

    ATTR_SPEED_UP,  # 速度 - 上行
    ATTR_SPEED_DOWN,  # 速度 - 下行
    ATTR_SPEED_ADHESION,  # 速度 - 吸附
    ATTR_SPEED_STEAM,  # 速度 - 水汽
    ATTR_SPEED_CLEAN,  # 速度 - 清洗
    ATTR_SPEED_U_SPRAY1,  # 速度 - 上行喷水1
    ATTR_SPEED_U_SPRAY2,  # 速度 - 上行喷水2
    ATTR_SPEED_D_SPRAY1,  # 速度 - 下行喷水1
    ATTR_SPEED_D_SPRAY2,  # 速度 - 下行喷水2


    ATTR_CONTROL_UPDOWN,
    ATTR_CONTROL_ADHESION,  # 吸附
    ATTR_CONTROL_CLEAN,  # 清洗系统
    ATTR_CONTROL_WATER,  # 水循环系统
    ATTR_CONTROL_CATERPILLA_LEFT,  # 左履带
    ATTR_CONTROL_CATERPILLA_RIGHT,  # 右履带
]

# 2字节组成u16的消息
V_VALUE = [
    ATTR_STATE_UPDOWN_LOAD,  # 提升负荷
    ATTR_STATE_DISTANCE_U,  # 测距上
    ATTR_STATE_DISTANCE_D,  # 测试下
    ATTR_STATE_DISTANCE_L,  # 测距左
    ATTR_STATE_DISTANCE_R,  # 测距右
    ATTR_STATE_TIME_ONCE,  # 本次工作时长
    ATTR_STATE_TIME_SUM,  # 累计工作时长
]



# 读取IMU数据
def imu_get(q_i2v, q_i2a, q_v2i, q_c):
    init_pitch = init_roll = -999.9
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
            imu_rec = se_i.read(33)
            if imu_rec:
                str_imu = binascii.b2a_hex(imu_rec).decode()
                if len(str_imu) == 66 and str_imu[0:4] == '5551':
                    jy_list = JY61.DueData(imu_rec)
                    if str(type(jy_list)) != "<class 'NoneType'>":
                        sav_mess = ("normal;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;%8.2f;\n" % jy_list)
                        send_list = [round(jy_list[6], 2), round(jy_list[7], 2), round(jy_list[8], 2)]

                        if init_pitch == -999.9 and init_roll == -999.9:
                            print('IMU start')
                            init_roll = send_list[0]
                            init_pitch = send_list[1]
                            print('Init Roll:' + str(init_roll) + '  Init Pitch:' + str(init_pitch))
                        else:
                            send_list[0] = send_list[0] - init_roll
                            send_list[1] = send_list[1] - init_pitch
                            send_list[2] = -send_list[2]
                            q_i2v.put(send_list)
                            q_i2v.get() if q_i2v.qsize() > 1 else time.sleep(0.005)
                            q_i2a.put(send_list)
                            q_i2a.get() if q_i2a.qsize() > 1 else time.sleep(0.005)
                    else:
                        sav_mess = ('NoneType;' + str_imu + ';\n')
                        print(sav_mess)
                        se_i.flushOutput()
                else:
                    sav_mess = ('IMU error;' + str_imu + ';\n')
                    print(sav_mess)
                    se_i.flushOutput()

            if not q_v2i.empty() and not q_c.empty():
                yaw_c = q_v2i.get()
                is_stop = q_c.get()
                if abs(yaw_c) <= 0.5 and yaw_c != 0.00 and is_stop:
                    str_zero = 'ff aa 52'
                    hex_zero = bytes.fromhex(str_zero)
                    se_i.write(hex_zero)
                    # cv2.waitKey(50)
                    time.sleep(0.05)
        except Exception as e:
            print(e)
            print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
            print(f"error line:{e.__traceback__.tb_lineno}")


# 数据CRC8校验
def xdata_CRC8(data):
    crc8 = data[0]
    for i in range(1, len(data), 1):
        crc8 = crc8 ^ data[i]
    return crc8


# 串口发送命令，自动计算长度，并添加crc。 然后接收并返回响应
def com_send_and_recv_resp(ser, msg, resp_len):
    # 计算并填充长度
    length = len(msg) + 1
    msg[3] = length

    # 计算并添加crc8
    crc8 = xdata_CRC8(msg)
    msg.append(crc8)

    # 发送命令
    ret = ser.write(msg)
    # 判断长度
    if ret != len(msg):
        print("com_send len error")
        return None, -1

    # 读取信息
    if resp_len == 0:
        hex_receive = ser.read(length)
    elif resp_len > 0:
        hex_receive = ser.read(resp_len)
    else:
        hex_receive = ser.readline()
    # 判断长度
    if len(hex_receive) <= 5:
        print("com recv too short：%d" % (len(hex_receive)))
        return None, -2
    elif hex_receive[3] != len(hex_receive):
        print("com recv len error")
        return None, -3

    # 头判断
    if hex_receive[0] != 0xaa:
        print("com recv head[0] error:%d" % (hex_receive[0]))
        return None, -4

    # 比较CRC
    crc_cal = xdata_CRC8(hex_receive[:-1])
    crc_rec = hex_receive[-1]
    if crc_cal != crc_rec:
        print("com recv crc error")
        return None, -5

    return hex_receive, 0


# 握手函数，成功接收到握手信息才返回
def HandShake(ser):
    # 握手的头及消息
    head = [0xaa, 0x0f, 0xff, 0]
    msg = "0:1.0#1:2.0#2:#3:#4:#5:#6:#"

    # 转为askii码
    msg_values = [ord(character) for character in msg]

    # 合并消息
    shake = head + msg_values

    # 发送收握手信息, 返回响应
    hex_receive, status = com_send_and_recv_resp(ser, shake, -1)

    # 判断状态
    if status != 0:
        print("hangshake error=%d" % (status))
        return status

    # 头判断
    if hex_receive[1] != 0xff:
        print("hangshake recv head[1] error")
        return -4

    # 截取其中的字符串
    str_receive = str(hex_receive[4:-2])
    print("handshake ok:", str_receive)
    # spl = str_receive.split('#')
    # print(spl)
    return 0


# 解析接收到的消息
def para_msg(msg):
    ret = []
    l = len(msg)
    for i in range(0, l, 3):
        cmd = msg[i]
        bh = msg[i + 1]
        bl = msg[i + 2]
        if cmd in V_BASIC:
            ret.append([cmd, bh, bl])
        elif cmd in V_VALUE:
            ret.append([cmd, (bh << 8 | bl)])
        else:
            print("unknow cmd:%d" % (cmd))

    return ret


# 解析控制响应消息的错误码
def para_set_resp(msg):
    ret = []
    l = len(msg)
    for i in range(0, l, 3):
        cmd = msg[i]
        bh = msg[i + 1]
        bl = msg[i + 2]

        if bh != 0 or bl != 0:
            error = [cmd, hex(bh << 8 | bl)]
            ret.append(error)
            print("ctl error:", error)
    return ret


# 获取状态
def get_status(ser):
    # 握手的头及消息
    head = [0xaa, 0x01, 0x1, 0]
    msg = [ATTR_STATE_WATER, 0, 0, ATTR_STATE_BATTERY, 0, 0, ATTR_STATE_DISTANCE_U, 0, 0, ATTR_STATE_DISTANCE_D, 0, 0,
           ATTR_STATE_TOUCH, 0, 0,ATTR_CONTROL_UPDOWN,0,0]

    # 合并消息
    msg = head + msg

    # 发送信息, 返回响应
    hex_receive, status = com_send_and_recv_resp(ser, msg, 0)

    # 判断状态
    if status != 0:
        print("get_status error=%d" % (status))
        return None, status

    # 判断长度
    if hex_receive[3] != len(hex_receive):
        print("get_status recv len error")
        return None, -3

    # 头判断
    if hex_receive[1] != 0xf1 or hex_receive[2] != 0x1:
        print("get_status recv len error")
        return None, -4

    # 截取消息
    hex_msg = list(hex_receive[4:-1])

    # 解释消息
    msgs = para_msg(hex_msg)

    return msgs, 0


# 获取速度
def get_speed(ser):
    head = [0xaa, 0x01, 0x2, 0]
    msg = [
        ATTR_SPEED_UP,0,0,  # 速度 - 上行
        ATTR_SPEED_DOWN,0,0,  # 速度 - 下行
        ATTR_SPEED_ADHESION,0,0,  # 速度 - 吸附
        ATTR_SPEED_STEAM,0,0,  # 速度 - 水汽
        ATTR_SPEED_CLEAN,0,0,  # 速度 - 清洗
        ATTR_SPEED_U_SPRAY1,0,0,  # 速度 - 上行喷水1
        ATTR_SPEED_U_SPRAY2,0,0,  # 速度 - 上行喷水2
        ATTR_SPEED_D_SPRAY1,0,0,  # 速度 - 下行喷水1
        ATTR_SPEED_D_SPRAY2,0,0,  # 速度 - 下行喷水2
    ]

    # 合并消息
    msg = head + msg

    # 发送信息, 返回响应
    hex_receive, status = com_send_and_recv_resp(ser, msg, 0)

    # 判断状态
    if status != 0:
        print("get_speed error=%d" % (status))
        return None, status

    # 判断长度
    if hex_receive[3] != len(hex_receive):
        print("get_speed recv len error")
        return None, -3

    # 头判断
    if hex_receive[1] != 0xf1 :
        print("get_speed recv len error")
        return None, -4

    # 截取消息
    hex_msg = list(hex_receive[4:-1])

    # 解释消息
    msgs = para_msg(hex_msg)

    return msgs, 0


# 发送控制命令
set_ctl_cnt = 0

#发送控制命令
def set_ctl(ser, msg):
    # 握手的头及消息
    head = [0xaa, 0x02, 0x4, 0]

    # 计算长度并填充
    length = len(head) + len(msg) + 1
    head[3] = length

    # 合并消息
    send_msg = head + msg

    # 发送并接收响应
    hex_receive, status = com_send_and_recv_resp(ser, send_msg, 0)

    # 判断状态
    if status != 0:
        print("set_ctl error=%d" % (status))
        return None, status

    # 头判断
    if hex_receive[1] != 0xf2:
        print("set_ctl recv error")
        return None, -4

    #global set_ctl_cnt
    #set_ctl_cnt += 1
    #print(datetime.datetime.now(), "set ctl %d" % (set_ctl_cnt), msg)
    # print('[{}]'.format(', '.join(hex(x) for x in list(msg))))
    print('[{}]\n'.format(', '.join(hex(x) for x in list(hex_receive))))

    # 截取消息
    hex_msg = list(hex_receive[4:-1])

    # hex_msg[1]=0xff
    # hex_msg[2]=0xfc

    # 解释消息,判断是否有错误码
    msgs = para_set_resp(hex_msg)
    return [hex_msg, msgs], 0


# 通讯失败的时候，往队列发送获取失败信息
def send_fail_msg(qout):
    # msg = [
    #     ATTR_STATE_WATER, 0xff, 0xfd,
    #     ATTR_STATE_BATTERY, 0xff, 0xfd,
    #     ATTR_STATE_DISTANCE_U, 0xfffd,
    #     ATTR_STATE_DISTANCE_D, 0xfffd,
    #     ATTR_STATE_TOUCH, 0xff, 0xfd,
    # ]
    msg = [["offline"]]
    try:
        qout.put(msg, timeout=1)
    except Exception as e:
        print(e)



def communication_thread(ser, qin, qout):
    need_handshake = True
    fail_cnt = 0
    while True:
        # 握手
        if need_handshake:
            ret = HandShake(ser)
            if ret == 0:
                need_handshake = False
                fail_cnt = 0
                #get_speed(ser)
            else:
                send_fail_msg(qin)
                time.sleep(recv_delay)
                continue

        # 获取状态
        msgs, status = get_status(ser)

        # 判断状态
        if status < 0 or len(msgs) == 0:
            fail_cnt += 1
            if fail_cnt > fail_cnt_thr:
                send_fail_msg(qin)
                need_handshake = True
            continue

        # 将接收到的信息放入队列
        try:
            qin.put(msgs, timeout=1)
        except Exception as e:
            print(e)

        # 查看是否需要发送控制命令
        while qout.qsize() >= 1:
            ctl_msg = qout.get()
            print(ctl_msg)
            set_ctl(ser, ctl_msg)
        time.sleep(recv_delay)
        continue


# 失败返回-1,成功返回0，会创建线程循环读取和发送控制信息
def communication_init(qin, qout):
    # 初始化串口
    try:
        ser = serial.Serial(com_id, com_bit_rate, timeout=com_time_out)
    except Exception as e:
        print(e)
        return -1

    # 控制线程
    com_thread = threading.Thread(name='communication_thread', target=communication_thread, args=(ser, qin, qout))
    com_thread.setDaemon(True)  # 把子进程设置为守护线程，必须在start()之前设置
    com_thread.start()

    return 0


#键盘监听
def keyboardListener(on_press):
    with Listener(on_press=on_press) as listener:
        listener.join()


# 创建队列
qout = queue.Queue(1)
qin = queue.Queue(1)
# 深度相机测距，偏航、左距离、右距离给自动控制，视觉偏航角给imu
queue_vision2action = queue.Queue(2)
queue_vision2imu = queue.Queue(2)
# 自动控制交互，控制hex给机器，控制命令给imu
queue_control_imu = queue.Queue(2)
# IMU，偏航、俯仰和翻滚，分发前相机测距和自动控制
queue_imu2vision = queue.Queue(2)
queue_imu2action = queue.Queue(2)

#按键处理
def on_press(key):
    global para_lines
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

    ctl = []

    try:
        #前进
        if key == keyboard.Key.up or key == keyboard.KeyCode.from_char('w'):
            ctl = [
                ATTR_CONTROL_CATERPILLA_LEFT, 2, 50,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 2, 50,  # 右履带
            ]
        #后退
        elif key == keyboard.Key.down or key == keyboard.KeyCode.from_char('s'):
            ctl = [
                ATTR_CONTROL_CATERPILLA_LEFT, 3, 50,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 3, 50,  # 右履带
            ]
        #左旋
        elif key == keyboard.Key.left or key == keyboard.KeyCode.from_char('a'):
            ctl = [
                ATTR_CONTROL_CATERPILLA_LEFT, 3, 50,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 2, 50,  # 右履带
            ]
        #右旋
        elif key == keyboard.Key.right or key == keyboard.KeyCode.from_char('d'):
            ctl = [
                ATTR_CONTROL_CATERPILLA_LEFT, 2, 50,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 3, 50,  # 右履带
            ]
        #停止
        elif key == keyboard.KeyCode.from_char('q') or key == keyboard.KeyCode.from_char('space'):
            ctl = [
                ATTR_CONTROL_CATERPILLA_LEFT, 0, 0,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 0, 0,  # 右履带
            ]
        # 测试ReRead
        elif key == keyboard.KeyCode.from_char('r'):
            # 读取参数
            file_model_reread = open('./Parameters.txt', 'r', encoding='utf-8')
            para_lines = file_model_reread.readlines()
            file_model_reread.close()
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

            G_Action.ReRead()

            print('Finish ReRead')

        #测试停止
        elif key == keyboard.KeyCode.from_char('e'):
            ctl_single = [
                ATTR_CONTROL_CATERPILLA_LEFT, 0, 0,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 0, 0,  # 右履带
            ]
            ret_state = G_Action.control_communication(ctl_single, qin, qout, queue_control_imu)
        #测试前进
        elif key == keyboard.KeyCode.from_char('i'):
            ctl_single = [
                ATTR_CONTROL_CATERPILLA_LEFT, 2, int_moveSpeed,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 2, int_moveSpeed,  # 右履带
            ]
            ret_state = G_Action.control_communication(ctl_single, qin, qout, queue_control_imu)

        #测试后退
        elif key == keyboard.KeyCode.from_char('k'):
            ctl_single = [
                ATTR_CONTROL_CATERPILLA_LEFT, 3, int_moveSpeed,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 3, int_moveSpeed,  # 右履带
            ]
            ret_state = G_Action.control_communication(ctl_single, qin, qout, queue_control_imu)
        #测试左旋
        elif key == keyboard.KeyCode.from_char('j'):
            ctl_single = [
                ATTR_CONTROL_CATERPILLA_LEFT, 3, int_turnSpeed,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 2, int_turnSpeed,  # 右履带
            ]
            ret_state = G_Action.control_communication(ctl_single, qin, qout, queue_control_imu)
        #测试右旋
        elif key == keyboard.KeyCode.from_char('l'):
            ctl_single = [
                ATTR_CONTROL_CATERPILLA_LEFT, 2, int_turnSpeed,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 3, int_turnSpeed,  # 右履带
            ]
            ret_state = G_Action.control_communication(ctl_single, qin, qout, queue_control_imu)
        #测试左转
        elif key == keyboard.KeyCode.from_char('u'):
            ctl_single = [
                ATTR_CONTROL_CATERPILLA_LEFT, 2, 30,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 2, 70,  # 右履带
            ]
            ret_state = G_Action.control_communication(ctl_single, qin, qout, queue_control_imu)
        #测试右转
        elif key == keyboard.KeyCode.from_char('o'):
            ctl_single = [
                ATTR_CONTROL_CATERPILLA_LEFT, 2, 70,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 2, 30,  # 右履带
            ]
            ret_state = G_Action.control_communication(ctl_single, qin, qout, queue_control_imu)
        # #测试直行
        # elif key == keyboard.KeyCode.from_char('n'):
        #     cam_yaw = -999.9
        #     if not queue_vision2action.empty():
        #         dis_list = queue_vision2action.get()
        #         cam_yaw = dis_list[0]
        #     else:
        #         time.sleep(0.1)
        #         if not queue_vision2action.empty():
        #             dis_list = queue_vision2action.get()
        #             cam_yaw = dis_list[0]
        #     if cam_yaw != -999.9:
        #         print('开始直行到边')
        #         ret_state = G_Action.go_straight(0, int_moveSpeed, -1, queue_vision2action, qin, queue_imu2action, qout, queue_control_imu)
        #     else:
        #         print('视觉识别失败')
        # #测试后退
        # elif key == keyboard.KeyCode.from_char('m'):
        #     print('开始后退')
        #     ret_state = G_Action.go_straight(3, int_moveSpeed, times_GoBack, queue_vision2action, qin, queue_imu2action, qout, queue_control_imu)
        #
        # #测试校正
        # elif key == keyboard.KeyCode.from_char('b'):
        #     cam_yaw = -999.9
        #     if not queue_vision2action.empty():
        #         dis_list = queue_vision2action.get()
        #         cam_yaw = dis_list[0]
        #     else:
        #         time.sleep(0.1)
        #         if not queue_vision2action.empty():
        #             dis_list = queue_vision2action.get()
        #             cam_yaw = dis_list[0]
        #     if cam_yaw != -999.9:
        #         print('开始校正', str(cam_yaw))
        #         ret_state = G_Action.correct_yaw(cam_yaw, 0.0, queue_vision2action, qin, queue_imu2action, qout, queue_control_imu)
        #     else:
        #         print('视觉识别失败')
        #测试直行2
        elif key == keyboard.KeyCode.from_char('b'):
            cam_yaw = -999.9
            if not queue_vision2action.empty():
                dis_list = queue_vision2action.get()
                cam_yaw = dis_list[0]
            else:
                time.sleep(0.1)
                if not queue_vision2action.empty():
                    dis_list = queue_vision2action.get()
                    cam_yaw = dis_list[0]
            if cam_yaw != -999.9:
                print('开始直行2')
                ret_state = G_Action.go_straight(4, int_moveSpeed, times_GoBack, queue_vision2action, qin, queue_imu2action, qout, queue_control_imu)
                ctl = [
                    ATTR_CONTROL_CATERPILLA_LEFT, 0, 0,  # 左履带
                    ATTR_CONTROL_CATERPILLA_RIGHT, 0, 0,  # 右履带
                ]
            else:
                print('视觉识别失败')
        # 测试TurnAround
        elif key == keyboard.KeyCode.from_char('n'):
            print('开始Turn Around')
            ret_state = G_Action.turn_around(True, queue_vision2action, qin, queue_imu2action, qout, queue_control_imu)
            ctl = [
                ATTR_CONTROL_CATERPILLA_LEFT, 0, 0,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 0, 0,  # 右履带
            ]
        #测试L型Left
        elif key == keyboard.KeyCode.from_char('m'):
            print('开始L型Left')
            ret_state = G_Action.l_turn(True, queue_vision2action, qin, queue_imu2action, qout, queue_control_imu)
            ctl = [
                ATTR_CONTROL_CATERPILLA_LEFT, 0, 0,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 0, 0,  # 右履带
            ]
        #测试Open Clean
        elif key == keyboard.KeyCode.from_char('h'):
            print('Open clean')
            G_Action.clean_water(2, 0, qin, qout, queue_control_imu)
            time.sleep(0.1)
            ctl = [
                ATTR_CONTROL_CATERPILLA_LEFT, 0, 0,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 0, 0,  # 右履带
            ]
        # 测试Close Clean
        elif key == keyboard.KeyCode.from_char('g'):
            print('Close clean')
            G_Action.clean_water(0, 0, qin, qout, queue_control_imu)
            time.sleep(0.1)
            ctl = [
                ATTR_CONTROL_CATERPILLA_LEFT, 0, 0,  # 左履带
                ATTR_CONTROL_CATERPILLA_RIGHT, 0, 0,  # 右履带
            ]
        # 测试Around
        elif key == keyboard.KeyCode.from_char('y'):
            cam_yaw = -999.9
            if not queue_vision2action.empty():
                dis_list = queue_vision2action.get()
                cam_yaw = dis_list[0]
            else:
                time.sleep(0.1)
                if not queue_vision2action.empty():
                    dis_list = queue_vision2action.get()
                    cam_yaw = dis_list[0]
            if cam_yaw != -999.9:
                print('AutoAround')
                is_left = True

                print('Stop')
                time.sleep(0.1)
                ret_state = G_Action.go_straight(5, 0, 3, queue_vision2action, qin, queue_imu2action, qout,
                                                 queue_control_imu)

                for i in range(0, 8, 1):
                    print('开始直行到边')
                    ret_state = G_Action.go_straight(0, int_moveSpeed, -1, queue_vision2action, qin, queue_imu2action, qout,
                                                     queue_control_imu)

                    print('Stop')
                    time.sleep(0.1)
                    ret_state = G_Action.go_straight(5, 0, 3, queue_vision2action, qin, queue_imu2action, qout,
                                                     queue_control_imu)

                    print('开始后退')
                    time.sleep(0.1)
                    ret_state = G_Action.go_straight(3, int_moveSpeed, times_GoBack, queue_vision2action, qin, queue_imu2action, qout,
                                                     queue_control_imu)
                    print('Stop')
                    time.sleep(0.1)
                    ret_state = G_Action.go_straight(5, 0, 3, queue_vision2action, qin, queue_imu2action, qout,
                                                     queue_control_imu)
                    print('开始L型Left')
                    time.sleep(0.1)
                    ret_state = G_Action.l_turn(is_left, queue_vision2action, qin, queue_imu2action, qout, queue_control_imu)

                    print('Stop')
                    time.sleep(0.1)
                    ret_state = G_Action.go_straight(5, 0, 3, queue_vision2action, qin, queue_imu2action, qout,
                                                     queue_control_imu)
                print('Finish Around')
                ctl = [
                    ATTR_CONTROL_CATERPILLA_LEFT, 0, 0,  # 左履带
                    ATTR_CONTROL_CATERPILLA_RIGHT, 0, 0,  # 右履带
                ]
            else:
                print('视觉识别失败')

        # 测试AutoRun
        elif key == keyboard.KeyCode.from_char('t'):
            cam_yaw = -999.9
            if not queue_vision2action.empty():
                dis_list = queue_vision2action.get()
                cam_yaw = dis_list[0]
            else:
                time.sleep(0.1)
                if not queue_vision2action.empty():
                    dis_list = queue_vision2action.get()
                    cam_yaw = dis_list[0]
            if cam_yaw != -999.9:
                print('AutoAround')
                is_left = True
                is_loop = True
                print('Stop')
                time.sleep(0.1)
                ret_state = G_Action.go_straight(5, 0, 3, queue_vision2action, qin, queue_imu2action, qout,
                                                 queue_control_imu)

                print('开始直行到边')
                ret_state = G_Action.go_straight(0, int_moveSpeed, -1, queue_vision2action, qin, queue_imu2action, qout,
                                                 queue_control_imu)

                print('Stop')
                time.sleep(0.1)
                ret_state = G_Action.go_straight(5, 0, 3, queue_vision2action, qin, queue_imu2action, qout,
                                                 queue_control_imu)

                print('开始后退')
                time.sleep(0.1)
                ret_state = G_Action.go_straight(3, int_moveSpeed, times_GoBack, queue_vision2action, qin,
                                                 queue_imu2action, qout,
                                                 queue_control_imu)
                print('Stop')
                time.sleep(0.1)
                ret_state = G_Action.go_straight(5, 0, 3, queue_vision2action, qin, queue_imu2action, qout,
                                                 queue_control_imu)
                print('开始L型Left')
                time.sleep(0.1)
                ret_state = G_Action.l_turn(is_left, queue_vision2action, qin, queue_imu2action, qout,
                                            queue_control_imu)

                print('Stop')
                time.sleep(0.1)
                ret_state = G_Action.go_straight(5, 0, 3, queue_vision2action, qin, queue_imu2action, qout,
                                                 queue_control_imu)

                print('开始Change Line直行')
                time.sleep(0.1)
                ret_state = G_Action.go_straight(0, int_moveSpeed, times_ChangeLine, queue_vision2action, qin, queue_imu2action, qout, queue_control_imu)

                if ret_state == 0:
                    is_left = True

                elif ret_state == 1:
                    is_left = False
                    print('Stop')
                    time.sleep(0.1)
                    ret_state = G_Action.go_straight(5, 0, 3, queue_vision2action, qin, queue_imu2action, qout,
                                                     queue_control_imu)
                    print('开始后退')
                    time.sleep(0.1)
                    ret_state = G_Action.go_straight(3, int_moveSpeed, times_GoBack, queue_vision2action, qin,
                                                     queue_imu2action, qout,
                                                     queue_control_imu)
                    print('开始L型Left')
                    time.sleep(0.1)
                    ret_state = G_Action.l_turn(True, queue_vision2action, qin, queue_imu2action, qout,
                                                queue_control_imu)
                    print('开始L型Left')
                    time.sleep(0.1)
                    ret_state = G_Action.l_turn(True, queue_vision2action, qin, queue_imu2action, qout,
                                                queue_control_imu)
                    print('开始直行')
                    time.sleep(0.1)
                    ret_state = G_Action.go_straight(0, int_moveSpeed, times_ChangeLine, queue_vision2action, qin,
                                                     queue_imu2action, qout, queue_control_imu)

                print('Stop')
                time.sleep(0.1)
                ret_state = G_Action.go_straight(5, 0, 3, queue_vision2action, qin, queue_imu2action, qout,
                                                 queue_control_imu)
                print('开始L型Left')
                time.sleep(0.1)
                ret_state = G_Action.l_turn(is_left, queue_vision2action, qin, queue_imu2action, qout, queue_control_imu)
                print('Stop')
                time.sleep(0.1)
                ret_state = G_Action.go_straight(5, 0, 3, queue_vision2action, qin, queue_imu2action, qout,
                                                 queue_control_imu)
                while is_loop:
                    if is_left:
                        is_left = False
                    else:
                        is_left = True

                    print('开始直行到边')
                    time.sleep(0.1)
                    ret_state = G_Action.go_straight(0, int_moveSpeed, -1, queue_vision2action, qin, queue_imu2action, qout,
                                                     queue_control_imu)
                    print('Stop')
                    time.sleep(0.1)
                    ret_state = G_Action.go_straight(5, 0, 3, queue_vision2action, qin, queue_imu2action, qout,
                                                     queue_control_imu)
                    print('开始后退')
                    time.sleep(0.1)
                    ret_state = G_Action.go_straight(3, int_moveSpeed, times_GoBack, queue_vision2action, qin, queue_imu2action, qout,
                                                     queue_control_imu)
                    print('Stop')
                    time.sleep(0.1)
                    ret_state = G_Action.go_straight(5, 0, 3, queue_vision2action, qin, queue_imu2action, qout,
                                                     queue_control_imu)
                    print('开始L型')
                    time.sleep(0.1)
                    ret_state = G_Action.l_turn(is_left, queue_vision2action, qin, queue_imu2action, qout, queue_control_imu)

                    print('Stop')
                    time.sleep(0.1)
                    ret_state = G_Action.go_straight(5, 0, 3, queue_vision2action, qin, queue_imu2action, qout,
                                                     queue_control_imu)
                    print('开始直行')
                    ret_state = G_Action.go_straight(0, int_moveSpeed, times_ChangeLine, queue_vision2action, qin, queue_imu2action, qout, queue_control_imu)

                    if ret_state == 1:
                        print('Stop')
                        ret_state = G_Action.go_straight(5, 0, 3, queue_vision2action, qin, queue_imu2action, qout,
                                                         queue_control_imu)
                        print('开始后退')
                        time.sleep(0.1)
                        ret_state = G_Action.go_straight(3, int_moveSpeed, times_GoBack, queue_vision2action, qin,
                                                         queue_imu2action, qout,
                                                         queue_control_imu)
                        print('Stop')
                        time.sleep(0.1)
                        ret_state = G_Action.go_straight(5, 0, 3, queue_vision2action, qin, queue_imu2action, qout,
                                                         queue_control_imu)
                        print('开始L型')
                        time.sleep(0.1)
                        ret_state = G_Action.l_turn(is_left, queue_vision2action, qin, queue_imu2action, qout,
                                                    queue_control_imu)
                        print('Stop')
                        time.sleep(0.1)
                        ret_state = G_Action.go_straight(5, 0, 3, queue_vision2action, qin, queue_imu2action, qout,
                                                         queue_control_imu)
                        is_loop = False
                        break

                    print('Stop')
                    time.sleep(0.1)
                    ret_state = G_Action.go_straight(5, 0, 3, queue_vision2action, qin, queue_imu2action, qout,
                                                     queue_control_imu)
                    print('开始L型')
                    time.sleep(0.1)
                    ret_state = G_Action.l_turn(is_left, queue_vision2action, qin, queue_imu2action, qout, queue_control_imu)
                    print('Stop')
                    time.sleep(0.1)
                    ret_state = G_Action.go_straight(5, 0, 3, queue_vision2action, qin, queue_imu2action, qout,
                                                     queue_control_imu)
                print('Finish')
                ctl = [
                    ATTR_CONTROL_CATERPILLA_LEFT, 0, 0,  # 左履带
                    ATTR_CONTROL_CATERPILLA_RIGHT, 0, 0,  # 右履带
                ]

            else:
                print('视觉识别失败')


    except AttributeError:
        print('special key {0} pressed'.format(key))
        return

    #没有命令
    if len(ctl)<=0:
        return
    else:
        print(ctl)

    #发送控制命令
    try:
        qout.put(ctl, timeout=1)
    except Exception as e:
        print(e)

    return


def autorun_init(queue_vision2action, queue_imu2action, qin, qout, queue_control_imu):
    # 控制线程
    com_thread = threading.Thread(name='autocontrol_run', target=G_Action.autocontrol_run, args=(queue_vision2action, queue_imu2action, qin, qout, queue_control_imu))
    com_thread.setDaemon(True)  # 把子进程设置为守护线程，必须在start()之前设置
    com_thread.start()

    return 0


def vision_init(queue_vision2action, queue_vision2imu):
    # 控制线程
    com_thread = threading.Thread(name='distance_get', target=G_RGBD.distance_get, args=(queue_vision2action, queue_vision2imu))
    com_thread.setDaemon(True)  # 把子进程设置为守护线程，必须在start()之前设置
    com_thread.start()

    return 0


def imu_init(queue_imu2vision, queue_imu2action, queue_vision2imu, queue_control_imu):
    # 控制线程
    com_thread = threading.Thread(name='imu_get', target=imu_get, args=(queue_imu2vision, queue_imu2action, queue_vision2imu, queue_control_imu))
    com_thread.setDaemon(True)  # 把子进程设置为守护线程，必须在start()之前设置
    com_thread.start()

    return 0


if __name__ == '__main__':
    #通讯线程初始化
    communication_init(qin, qout)

    print('Main Start')
    # 深度相机测距
    vision_init(queue_vision2action, queue_vision2imu)

    # IMU
    imu_init(queue_imu2vision, queue_imu2action, queue_vision2imu, queue_control_imu)

    # print('Keyboard')
    # #键盘按钮监控
    # keyboardListener(on_press)

    # 自动运行导航
    # autorun_init(queue_vision2action, queue_imu2action, qin, qout, queue_control_imu)
    G_Action.autocontrol_run(queue_vision2action, queue_imu2action, qin, qout, queue_control_imu)
