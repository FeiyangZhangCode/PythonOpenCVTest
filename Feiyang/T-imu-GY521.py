import math
import time
import smbus
import datetime
import openpyxl
import cv2

# 电源管理寄存器地址
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
bus = smbus.SMBus(0)  # or bus = smbus.SMBus(1) for Revision 2 boards
address = 0x68  # This is the address value read via the i2cdetect command
# Now wake the 6050 up as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1, 0)


def read_byte(adr):
    return bus.read_byte_data(address, adr)


def read_word(adr):
    high = bus.read_byte_data(address, adr)

    low = bus.read_byte_data(address, adr + 1)

    val = (high << 8) + low

    return val


def read_word_2c(adr):
    val = read_word(adr)

    if val >= 0x8000:

        return -((65535 - val) + 1)

    else:

        return val


def dist(a, b):
    return math.sqrt((a * a) + (b * b))

    # math.sqrt(x) 方法返回数字x的平方根


def get_y_rotation(x, y, z):
    radians = math.atan2(x, dist(y, z))
    # math.atan2(y, x) 返回给定的 X 及 Y 坐标值的反正切值
    return -math.degrees(radians)
    # math.degrees(x) 将弧度x转换为角度


def get_x_rotation(x, y, z):
    radians = math.atan2(y, dist(x, z))
    return math.degrees(radians)


def get_imu_data():
    # work_book = openpyxl.load_workbook('./TestData/MPU-y90.xlsx')
    # work_sheet = work_book['Sheet1']

    str_Time = datetime.datetime.now().strftime('%H:%M:%S.%f')
    # start_time = time.time()
    temp_out = read_word_2c(0x41)
    temp_out = temp_out / 340 + 36.53
    # print("温度传感器")
    # print("temperature: ", temp_out)

    gyro_xout = read_word_2c(0x43)
    gyro_yout = read_word_2c(0x45)
    gyro_zout = read_word_2c(0x47)
    gyro_xout_scaled = gyro_xout / 131
    gyro_yout_scaled = gyro_yout / 131
    gyro_zout_scaled = gyro_zout / 131
    # print("XYZ轴陀螺仪计数值,每秒的旋转度数")
    # print("gyro_xout : ", gyro_xout, " scaled: ", gyro_xout_scaled)  # 倍率250/s
    # print("gyro_yout : ", gyro_yout, " scaled: ", gyro_yout_scaled)
    # print("gyro_zout : ", gyro_zout, " scaled: ", gyro_zout_scaled)

    accel_xout = read_word_2c(0x3b)
    accel_yout = read_word_2c(0x3d)
    accel_zout = read_word_2c(0x3f)
    accel_xout_scaled = accel_xout / 16384.0  # 倍率2g
    accel_yout_scaled = accel_yout / 16384.0
    accel_zout_scaled = accel_zout / 16384.0
    # print("XYZ轴加速度计数值,每秒的旋转度数")
    # print("accel_xout: ", accel_xout, " scaled: ", accel_xout_scaled)
    # print("accel_yout: ", accel_yout, " scaled: ", accel_yout_scaled)
    # print("accel_zout: ", accel_zout, " scaled: ", accel_zout_scaled)

    # print("XY轴旋转度数")
    rot_x = get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
    rot_y = get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
    # print("x rotation: ", rot_x)
    # print("y rotation: ", rot_y)

    sav_mess = str_Time + ';'
    sav_mess += str(round(temp_out, 2)) + ';'
    sav_mess += str(round(gyro_xout_scaled, 4)) + ';'
    sav_mess += str(round(gyro_yout_scaled, 4)) + ';'
    sav_mess += str(round(gyro_zout_scaled, 4)) + ';'
    sav_mess += str(round(accel_xout_scaled, 4)) + ';'
    sav_mess += str(round(accel_yout_scaled, 4)) + ';'
    sav_mess += str(round(accel_zout_scaled, 4)) + ';'
    sav_mess += str(round(rot_x, 4)) + ';'
    sav_mess += str(round(rot_y, 4)) + ';\n'

    # end_time = time.time()
    # print(str(round((end_time - start_time) * 1000, 4)) + 'ms')
    return temp_out, rot_x, rot_y, sav_mess


if __name__ == '__main__':
    while True:
        cv2.waitKey(50)
        flo_temp, flo_rX, flo_rY, get_mess = get_imu_data()
        file_rec = open('../TestData/MPU-UpDownTest.txt', 'a')
        file_rec.write(get_mess)
        file_rec.close()
        print('T: ', str(round(flo_temp, 2)))
        print("x: ", flo_rX)
        print("y: ", flo_rY)
