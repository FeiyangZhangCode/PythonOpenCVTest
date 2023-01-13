import serial
import serial.tools.list_ports
import re
import time
import binascii
import datetime
import cv2

COM_NAME = 'CH341'
BAUD_RATE = 115200


def auto_answer():
    ser = serial.Serial('COM6', 115200, timeout=0.1)
    print('Start Read')
    get_num = 0
    last_rec = ''
    start_time = time.time()
    end_time = time.time()
    while True:
        str_send = 'aa 02 00 00 00 00 00 64 00 64 b1'
        hex_send = bytes.fromhex(str_send)
        ser.write(hex_send)
        cv2.waitKey(80)
        line = ser.readline()
        if line:
            str_rec = binascii.b2a_hex(line).decode()

            if str_rec != last_rec:
                start_time = time.time()
                last_rec = str_rec
            end_time = time.time()
            time_mess = str(round((end_time - start_time) * 1000, 0)) + 'ms'

            if len(str_rec) > 12 and str_rec[0:4] == 'aa01':
                str_move = str_rec[4:6]
                str_speed = str_rec[6:8]
                str_board = str_rec[8:10]
                str_water = str_rec[10:12]
                if str_move == '00':
                    state_move = 'Move Stop'
                elif str_move == '01':
                    state_move = 'Move Front'
                elif str_move == '02':
                    state_move = 'Move Back'
                elif str_move == '03':
                    state_move = 'Rotate Left'
                elif str_move == '04':
                    state_move = 'Rotate Right'
                else:
                    state_move = 'Move Error'
                state_move += '  ' + str(int(str_speed, 16)) + '%'
                if str_board == '00':
                    state_board = 'Board Stop'
                elif str_board == '01':
                    state_board = 'Board Front'
                elif str_board == '02':
                    state_board = 'Board Back'
                else:
                    state_board = 'Board Error'
                if str_water == '00':
                    state_water = 'Water Stop'
                elif str_water == '01':
                    state_water = 'Water Start'
                else:
                    state_water = 'Water Error'

                print(state_move, state_board, state_water, time_mess)
            else:
                show_mess = 'Receive Error'
                print(show_mess, time_mess)


def get_ports():
    comList = list(serial.tools.list_ports.comports())
    i = 0
    while i < len(comList):
        tmp_com = list(comList[i])
        str_com = str(tmp_com)
        r = re.search(COM_NAME, str_com)
        if r != None:
            port_num = tmp_com[0]
        i = i + 1
    return port_num


def serial_read():
    se1 = serial.Serial('COM4', 9600, timeout=0.02)
    print('Read Ready')
    while True:
        # time.sleep(1)
        line = se1.readline()
        if line:
            # se1.write('Get\r\n'.encode())
            str_rec = binascii.b2a_hex(line).decode()
            print(str_rec)



if __name__ == '__main__':
    auto_answer()
    # serial_read()
    # ser = serial.Serial(get_ports(), BAUD_RATE, timeout=0.5)
    # ser = serial.Serial('COM8', 115200, timeout=0.2)
    # print('Start Read')
    # get_num = 0
    # while True:
    #     # time.sleep(0.15)
    #     str_send = 'aa 02 00 00 00 00 00 64 00 05 b1'
    #     hex_send = bytes.fromhex(str_send)
    #     ser.write(hex_send)
    #     line = ser.readline()
    #     if line:
    #         str_rec = binascii.b2a_hex(line)
    #         # if str_rec.decode('utf-8') != 'aa0100000000a8':
    #         print(str_rec)
    #         get_num += 1
    #         # str_Time = datetime.datetime.now().strftime('%H:%M:%S.%f')
    #         # print(str_Time)

    # success_bytes = ser.write(b"This is data for test")
    # print("发送数据长度:", success_bytes)  # 发送数据长度
    # recv_data = ser.readline()
    # print("接收到的数据是：", recv_data.decode())
