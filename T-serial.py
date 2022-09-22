import binascii
import serial
import time

def serial_rw():
    se = serial.Serial('/dev/ttyTHS1', 115200, timeout=0.5)

    loop_num = 0
    while True:
        loop_num += 1
        se.write('1'.encode())
        time.sleep(0.5)
        hex_rec = se.readline()
        if hex_rec:
            data_1 = binascii.b2a_hex(hex_rec)
            print(data_1)
        print(loop_num)


def serial_write():
    se = serial.Serial('/dev/ttyTHS1', 9600, timeout=1)
    time.sleep(1)
    se.write('TestNull'.encode())
    se.write(b'Test B')
    se.write('Test GB'.encode("GB2312"))
    print('Write Finish')


def serial_read():
    se1 = serial.Serial('/dev/ttyTHS1', 9600, timeout=1)
    print('Read Ready')
    while True:
        time.sleep(1)
        line = se1.readline()
        if line:
            se1.write('Get\r\n'.encode())
            print(line)


if __name__ == '__main__':
    print('Start')
    # serial_write()
    # serial_read()
    serial_rw()
    print('End')