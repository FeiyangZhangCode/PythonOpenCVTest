import serial
import serial.tools.list_ports
import re
import time
import binascii
import datetime

COM_NAME = 'CH341'
BAUD_RATE = 115200


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
    serial_read()
    # ser = serial.Serial(get_ports(), BAUD_RATE, timeout=0.5)
    # ser = serial.Serial('COM5', 115200, timeout=0.1)
    # print('Start Read')
    # get_num = 0
    # while True:
    #     time.sleep(0.2)
    #     # line = ser.readline()
    #     # if line:
    #     str_send = 'aa 02 00 00 00 00 a8'
    #     hex_send = bytes.fromhex(str_send)
    #     ser.write(hex_send)
        # str_rec = binascii.b2a_hex(line)
        # # if str_rec.decode('utf-8') != 'aa0100000000a8':
        # print(str_rec)
        # get_num += 1
        # str_Time = datetime.datetime.now().strftime('%H:%M:%S.%f')
        # print(str_Time)

    # success_bytes = ser.write(b"This is data for test")
    # print("发送数据长度:", success_bytes)  # 发送数据长度
    # recv_data = ser.readline()
    # print("接收到的数据是：", recv_data.decode())
