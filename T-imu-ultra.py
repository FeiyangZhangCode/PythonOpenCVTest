import binascii
import serial
import time
import cv2
import datetime
import multiprocessing as mp
import os
import INA219
import MPU6050


# 串口读取超声数据
def ultra_get(q_u, lock_ser, file_address):
    se_u = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.1)
    while True:
        try:
            se_u.write('1'.encode())
            time.sleep(0.1)
            ult_rec = se_u.readline()
            lock_ser.acquire()
            if ult_rec:
                file_serial = open(file_address + 'Ultra.txt', 'a')
                str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
                str_ult = binascii.b2a_hex(ult_rec).decode()
                file_serial.write(str_time + str(str_ult) + '\n')
                file_serial.close()
                if str_ult[0:2] == 'ff':
                    s1 = str_ult[2:6]
                    s2 = str_ult[6:10]
                    u0 = int(s1, 16)
                    u1 = int(s2, 16)
                    send_list = [str_time, u0, u1]
                    q_u.put(send_list)
                    q_u.get() if q_u.qsize() > 1 else time.sleep(0.005)
        except Exception as e:
            print(e)
        finally:
            lock_ser.release()


# I2C读取IMU数据
def imu_get(q_i, lock_ser, file_address):
    while True:
        cv2.waitKey(50)
        temp_out, rot_x, rot_y, sav_mess = MPU6050.get_imu_data()
        file_rec = open(file_address + 'MPU.txt', 'a')
        str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
        file_rec.write(str_time + ';' + sav_mess)
        file_rec.close()
        send_list = [str_time, round(temp_out, 2), round(rot_x, 2), round(rot_y, 2)]
        q_i.put(send_list)
        q_i.get() if q_i.qsize() > 1 else time.sleep(0.005)


def autocontrol_run(q_u, q_i, lock_ser, file_address):
    ultra_front = 0
    ultra_back = 0
    ultra_time = ''
    imu_yaw = 0.0
    imu_pitch = 0.0
    imu_temp = 0.0
    imu_time = ''

    while True:
        cv2.waitKey(50)
        imu_state = 'old'
        ultra_state = 'old'
        # 获取偏航角
        if not q_i.empty():
            imu_list = q_i.get()
            imu_time = imu_list[0]
            imu_temp = imu_list[1]
            imu_yaw = imu_list[2]
            imu_pitch = imu_list[3]
            imu_state = 'new'
        # 获取前后超声波
        if not q_u.empty():
            ultra_list = q_u.get()
            ultra_time = ultra_list[0]
            ultra_front = ultra_list[1]
            ultra_back = ultra_list[2]
            ultra_state = 'new'
        file_rec = open(file_address + 'acSensor.txt', 'a')
        str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
        save_mess = str_time + ';' + ultra_state + ';' + ultra_time + ';' + str(ultra_front) + ';' + str(ultra_back) + ';'
        save_mess += imu_state + ';' + imu_time + ';' + str(imu_temp) + ';' + str(imu_yaw) + ';' + str(imu_pitch) + ';\n'
        file_rec.write(save_mess)
        file_rec.close()
        print(ultra_front, ultra_back, imu_yaw, imu_pitch)


if __name__ == '__main__':
    # 新建文件夹,读取时间作为文件名
    str_fileAddress = './TestData/'
    str_Time = datetime.datetime.now().strftime('%Y%m%d-%H%M')
    # file_rec = open(str_fileHome + str_Time + '.txt', 'w', encoding='utf-8')
    str_fileAddress += str_Time
    if not os.path.exists(str_fileAddress):
        os.makedirs(str_fileAddress)
    str_fileAddress += '/'

    mp.set_start_method(method='spawn')  # init
    processes = []
    lock = mp.Lock()
    queue_ultra = mp.Queue(maxsize=2)
    queue_imu = mp.Queue(maxsize=2)

    processes.append(mp.Process(target=ultra_get, args=(queue_ultra, lock, str_fileAddress)))
    processes.append(mp.Process(target=imu_get, args=(queue_imu, lock, str_fileAddress)))
    processes.append(mp.Process(target=autocontrol_run, args=(queue_ultra, queue_imu, lock, str_fileAddress)))

    for process in processes:
        process.daemon = True
        process.start()
    for process in processes:
        process.join()
