import serial
import time


def serial_write():
    se = serial.Serial('/dev/ttyTHS2', 9600, timeout=1)
    time.sleep(1)
    str_b = 'Get Data\r\n'
    # print(len(str_b))
    se.write(str_b.encode())
    line = se.readline().strip()
    if line:
        # se.write('Get\r\n'.encode())
        str_c = str(line)
        print(str_c)
        # print(len(str_c))




def serial_read():
    se1 = serial.Serial('/dev/ttyTHS2', 9600, timeout=1)
    while True:
        time.sleep(1)
        line = se1.readline()
        if line:
            se1.write('Get'.encode())
            print(line)


if __name__ == '__main__':
    print('Start')
    serial_write()
    serial_read()
    print('End')
