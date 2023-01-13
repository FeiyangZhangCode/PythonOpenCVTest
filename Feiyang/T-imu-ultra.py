import binascii
import serial
import time
import cv2
import datetime
import multiprocessing as mp
import os
# import INA219
# import MPU6050
import JY61


# 串口读取JY61的IMU数据
def imu_JY61_get(q_ij, lock_ser, file_address):
    se_ij = serial.Serial('COM13', 9600, timeout=0.02)
    while True:
        datahex = se_ij.read(33)
        if datahex:
            jy_list = JY61.DueData(datahex)
            file_rec = open(file_address + 'JY61.txt', 'a')
            str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
            sav_mess = ("%10.2f;%10.2f;%10.2f;%10.2f;%10.2f;%10.2f;%10.2f;%10.2f;%10.2f;\n" % jy_list)
            file_rec.write(str_time + ';' + sav_mess)
            file_rec.close()
            send_list = [str_time, round(jy_list[6], 2), round(jy_list[7], 2), round(jy_list[8], 2)]
            q_ij.put(send_list)
            q_ij.get() if q_ij.qsize() > 1 else time.sleep(0.005)


# 串口读取超声数据
def ultra_get(q_u, lock_ser, file_address):
    se_u = serial.Serial('COM12', 9600, timeout=0.1)
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
def imu_GY521_get(q_i, lock_ser, file_address):
    while True:
        cv2.waitKey(50)
        temp_out, rot_x, rot_y, sav_mess = MPU6050.get_imu_data()
        file_rec = open(file_address + 'GY521.txt', 'a')
        str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
        file_rec.write(str_time + ';' + sav_mess)
        file_rec.close()
        send_list = [str_time, round(temp_out, 2), round(rot_x, 2), round(rot_y, 2)]
        q_i.put(send_list)
        q_i.get() if q_i.qsize() > 1 else time.sleep(0.005)


def autocontrol_run(q_u, q_ig, q_ij, lock_ser, file_address):
    ultra_front = 0
    ultra_back = 0
    ultra_time = ''
    imu1_yaw = 0.0
    imu1_pitch = 0.0
    imu1_temp = 0.0
    imu1_time = ''
    imu2_yaw = 0.0
    imu2_pitch = 0.0
    imu2_roll = 0.0
    imu2_time = ''

    while True:
        cv2.waitKey(50)
        imu1_state = 'old'
        imu2_state = 'old'
        ultra_state = 'old'
        # # 获取GY521偏航角
        # if not q_ig.empty():
        #     imu1_list = q_ig.get()
        #     imu1_time = imu1_list[0]
        #     imu1_temp = imu1_list[1]
        #     imu1_yaw = imu1_list[2]
        #     imu1_pitch = imu1_list[3]
        #     imu1_state = 'new'
        # 获取JY61偏航角
        if not q_ij.empty():
            imu2_list = q_ij.get()
            imu2_time = imu2_list[0]
            imu2_yaw = imu2_list[1]
            imu2_pitch = imu2_list[2]
            imu2_roll = imu2_list[3]
            imu2_state = 'new'
        # 获取前后超声波
        if not q_u.empty():
            ultra_list = q_u.get()
            ultra_time = ultra_list[0]
            ultra_front = ultra_list[1]
            ultra_back = ultra_list[2]
            ultra_state = 'new'
        file_rec = open(file_address + 'acSensor.txt', 'a')
        str_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
        save_mess = str_time + ';' + ultra_state + ';' + ultra_time + ';' + str(ultra_front) + ';' + str(
            ultra_back) + ';'
        # save_mess += imu1_state + ';' + imu1_time + ';' + str(imu1_temp) + ';' + str(imu1_yaw) + ';' + str(
        #     imu1_pitch) + ';'
        save_mess += imu2_state + ';' + imu2_time + ';' + str(imu2_yaw) + ';' + str(imu2_pitch) + ';' + str(
            imu2_roll) + ';\n'
        file_rec.write(save_mess)
        file_rec.close()
        # print(ultra_front, ultra_back, imu1_yaw, imu2_yaw)
        print(ultra_front, ultra_back, imu2_yaw)



if __name__ == '__main__':
    # 新建文件夹,读取时间作为文件名
    str_fileAddress = './TestData/'
    str_time_main = datetime.datetime.now().strftime('%Y%m%d-%H%M')
    # file_rec = open(str_fileHome + str_Time + '.txt', 'w', encoding='utf-8')
    str_fileAddress += str_time_main
    if not os.path.exists(str_fileAddress):
        os.makedirs(str_fileAddress)
    str_fileAddress += '/'

    mp.set_start_method(method='spawn')  # init
    processes = []
    lock = mp.Lock()
    queue_ultra = mp.Queue(maxsize=2)
    queue_gy521 = mp.Queue(maxsize=2)
    queue_jy61 = mp.Queue(maxsize=2)

    processes.append(mp.Process(target=ultra_get, args=(queue_ultra, lock, str_fileAddress)))
    # processes.append(mp.Process(target=imu_GY521_get, args=(queue_gy521, lock, str_fileAddress)))
    processes.append(mp.Process(target=imu_JY61_get, args=(queue_jy61, lock, str_fileAddress)))
    processes.append(
        mp.Process(target=autocontrol_run, args=(queue_ultra, queue_gy521, queue_jy61, lock, str_fileAddress)))

    for process in processes:
        process.daemon = True
        process.start()
    for process in processes:
        process.join()
