import binascii
import serial
import time
import datetime


def ultra_rw():
    se = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.1)
    loop_num = 0
    while True:
        loop_num += 1
        se.write('1'.encode())
        time.sleep(0.1)
        hex_rec = se.readline()
        if hex_rec:
            str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
            print(str_time)
            data_1 = binascii.b2a_hex(hex_rec)
            print(data_1)
            s1 = str(data_1[2:6].decode())
            s2 = str(data_1[6:10].decode())
            n1 = int(s1, 16)
            n2 = int(s2, 16)
            # print(n1, n2)
        print(loop_num)


if __name__ == '__main__':
    ultra_rw()
