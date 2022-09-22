import binascii
import serial
import time
import datetime

def serial_rw():
    se = serial.Serial('/dev/ttyTHS1', 115200, timeout=0.15)

    loop_num = 0
    while True:
        loop_num += 1
        # se.write('1'.encode())
        str_init = 'aa01040a000077'
        hex_init_send = bytes.fromhex(str_init)
        se.write(hex_init_send)

        time.sleep(0.05)
        hex_rec = se.readline()
        if hex_rec:
            data_1 = binascii.b2a_hex(hex_rec)
            print(data_1)
        print(loop_num)


def auto_answer():
    se = serial.Serial('/dev/ttyTHS1', 115200, timeout=0.15)
    file_rec = open('/TestData/SerAutoAnswer.txt', 'a')

    print('Start Read')
    get_num = 0
    while True:
        time.sleep(0.1)
        line = se.readline()
        if line:
            str_send = 'aa 02 00 00 00 00 a8'
            hex_send = bytes.fromhex(str_send)
            se.write(hex_send)
            str_rec = binascii.b2a_hex(line)
            # if str_rec.decode('utf-8') != 'aa0100000000a8':
            get_num += 1
            str_Time = datetime.datetime.now().strftime('%H:%M:%S.%f')
            # print(str_Time)
            file_rec.write(str_Time)
            print(str_rec)
            file_rec.write(str_rec.decode('utf-8'))



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
    # serial_rw()
    auto_answer()
    print('End')