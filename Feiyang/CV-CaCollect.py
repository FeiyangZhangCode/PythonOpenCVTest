import cv2
import datetime
import INA219
import os


# 确定摄像头数量和编号，提取从0-6
def get_camera_id():
    list_ID = [int] * 7
    int_num = 0
    for k in range(0, 7, 1):
        cap_test = cv2.VideoCapture(k)
        if cap_test.isOpened():
            list_ID[int_num] = k
            int_num += 1
            cap_test.release()
    return list_ID, int_num


# 返回摄像头格式
def get_camera_data(cap, num):
    str_Info = 'C' + str(num) + ':' + str(cap.get(3)) + '*' + str(cap.get(4)) + '; FPS' + str(cap.get(5)) + '\n'
    str_fourcc = str("".join([chr((int(cap.get(cv2.CAP_PROP_FOURCC)) >> 8 * k) & 0xFF) for k in range(4)])) + '\n'
    str_Info = str_Info + str_fourcc
    return str_Info


# 开始主程序
# 打开文件夹,读取时间作为文件名，开始本轮采集
str_fileAddress = './TestData/'
str_fileHome = str_fileAddress
str_Time = datetime.datetime.now().strftime('%Y%m%d-%H%M')
file_rec = open(str_fileHome + str_Time + '.txt', 'w', encoding='utf-8')
str_fileAddress = str_fileAddress + str_Time
if not os.path.exists(str_fileAddress):
    os.makedirs(str_fileAddress)
str_fileAddress = str_fileAddress + '/'

cap_num = 0
fourcc = cv2.VideoWriter_fourcc(*'XVID')
while cap_num == 0:
    # 确定摄像头数量和编号，提取从0-6
    cap_ID, cap_num = get_camera_id()
    if cap_num == 1:
        # 第一个摄像头0
        cap0 = cv2.VideoCapture(cap_ID[0])
        cap0.set(6, 1196444237)
        cap0.set(3, 1920)
        cap0.set(4, 1080)
        cap0.set(5, 30)
        fps0 = cap0.get(cv2.CAP_PROP_FPS)
        size0 = (int(cap0.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap0.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        out0 = cv2.VideoWriter(str_fileAddress + 'Video-0.avi', fourcc, fps0, size0, True)
        file_rec.write(get_camera_data(cap0, cap_ID[0]))
        print(get_camera_data(cap0, cap_ID[0]))
        # UPS情况
        ina_mess = INA219.get_ina219_data()
        file_rec.write(ina_mess)
    elif cap_num > 1:
        # 第一个摄像头0
        cap0 = cv2.VideoCapture(cap_ID[0])
        cap0.set(6, 1196444237)
        cap0.set(3, 1920)
        cap0.set(4, 1080)
        cap0.set(5, 30)
        fps0 = cap0.get(cv2.CAP_PROP_FPS)
        size0 = (int(cap0.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap0.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        out0 = cv2.VideoWriter(str_fileAddress + 'Video-0.avi', fourcc, fps0, size0, True)
        file_rec.write(get_camera_data(cap0, cap_ID[0]))
        print(get_camera_data(cap0, cap_ID[0]))
        # 第二个摄像头1
        cap1 = cv2.VideoCapture(cap_ID[1])
        cap1.set(6, 1196444237)
        cap1.set(3, 1920)
        cap1.set(4, 1080)
        cap1.set(5, 30)
        fps1 = cap1.get(cv2.CAP_PROP_FPS)
        size1 = (int(cap1.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap1.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        out1 = cv2.VideoWriter(str_fileAddress + 'Video-1.avi', fourcc, fps1, size1, True)
        file_rec.write(get_camera_data(cap1, cap_ID[1]))
        print(get_camera_data(cap1, cap_ID[1]))
        # UPS情况
        ina_mess = INA219.get_ina219_data()
        file_rec.write(ina_mess)
    else:
        ina_mess = INA219.get_ina219_data()
        print('未找到摄像头，30秒后重新尝试')
        for i in range(0, 30, 1):
            print(30 - i)
            cv2.waitKey(1000)

# 设置循环计数器，每循环多少帧进行一次操作
loop_num = 0

# 开始循环采集数据
while True:
    loop_num = loop_num + 1
    now_time = datetime.datetime.now()
    str_time_doc = now_time.strftime('%H%M%S')
    str_time_record = now_time.strftime('%H:%M:%S.%f')

    if cap_num == 1:
        ret0, frame0 = cap0.read()
        out0.write(frame0)
        cv2.namedWindow('Camera-0', cv2.WINDOW_NORMAL)
        cv2.imshow('Camera-0', frame0)
        # 存储原图
        if loop_num % 5 == 0:
            cv2.imwrite(str_fileAddress + "P0-" + str_time_doc + '-' + str(loop_num) + '.jpg', frame0)
            # file_rec.write(str_time_record + ',' + str(loop_num) + ',C0' + '\n')
            print(str_time_record + ',' + str(loop_num) + ',C0')

    elif cap_num > 1:
        ret0, frame0 = cap0.read()
        ret1, frame1 = cap1.read()
        out0.write(frame0)
        out1.write(frame1)
        cv2.namedWindow('Camera-0', cv2.WINDOW_NORMAL)
        cv2.imshow('Camera-0', frame0)
        cv2.namedWindow('Camera-1', cv2.WINDOW_NORMAL)
        cv2.imshow('Camera-1', frame1)
        # 存储原图
        if loop_num % 5 == 0:
            cv2.imwrite(str_fileAddress + "P0-" + str_time_doc + '-' + str(loop_num) + '.jpg', frame0)
            cv2.imwrite(str_fileAddress + "P1-" + str_time_doc + '-' + str(loop_num) + '.jpg', frame1)
            # file_rec.write(str_time_record + ',' + str(loop_num) + ',C0+C1' + '\n')
            print(str_time_record + ',' + str(loop_num) + ',C0+C1' + '\n')

    elif cap_num == 0:
        print('未找到摄像头')
        break

    if loop_num % 30 == 0:
        # UPS情况
        ina_mess = INA219.get_ina219_data()
        # file_rec.write(str_time_record + ',' + ina_mess)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

file_rec.close()
if cap_num == 1:
    cap0.release()
elif cap_num > 1:
    cap0.release()
    cap1.release()
cv2.destroyAllWindows()
