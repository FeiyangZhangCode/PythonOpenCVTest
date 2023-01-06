import datetime
import serial
import binascii
import time

if __name__ == '__main__':
    se_l = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.005)
    # 释放串口积攒的数据
    se_l.flushInput()
    se_l.flushOutput()
    while True:
        try:
            # 串口采集TFminiPlus的测距数据
            laser_rec = se_l.read(18)
            if laser_rec:
                str_laser = binascii.b2a_hex(laser_rec).decode()
                # file_rec = open('./TestData/JY61.txt', 'a')
                str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
                if str_laser[0:4] == '5959':
                    hex_f_l = laser_rec[2]
                    hex_f_h = laser_rec[3]
                    hex_s_l = laser_rec[4]
                    hex_s_h = laser_rec[5]
                    laser_F = hex_f_h << 8 | hex_f_l
                    laser_S = hex_s_h << 8 | hex_s_l
                    print(str(laser_F), str(laser_S))
                else:
                    print(str_laser)
                    # se_l.flushOutput()
                # file_rec.write(str_time + ';' + sav_mess)
                # file_rec.close()

        except Exception as e:
            print(e)
            print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
            print(f"error line:{e.__traceback__.tb_lineno}")
