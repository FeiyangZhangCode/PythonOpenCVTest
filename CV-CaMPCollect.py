# import queue
import cv2
import time
import datetime
import multiprocessing as mp


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


# 抓取图片，确认视频流的读入
def image_put(q, c_id):
    cap = cv2.VideoCapture(c_id)
    cap.set(6, 1196444237)
    cap.set(3, 1920)
    cap.set(4, 1080)
    cap.set(5, 30)
    # 获取视频帧率
    fps = cap.get(cv2.CAP_PROP_FPS)
    size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    print(get_camera_data(cap, c_id))
    if cap.isOpened():
        print('Get1', c_id)
    else:
        cap = cv2.VideoCapture(c_id)
        cap.set(6, 1196444237)
        cap.set(3, 1920)
        cap.set(4, 1080)
        cap.set(5, 30)
        fps = cap.get(cv2.CAP_PROP_FPS)
        print(get_camera_data(cap, c_id))
        print('Get2', c_id)

    while cap.isOpened():
        # print('cap.read()[0]:', cap.read()[0])
        ret, frame = cap.read()
        # print('ret:', ret)
        frame = cv2.resize(frame, (1920, 1080))
        # 抓取图片不成功再重新抓取
        if not ret:
            cap = cv2.VideoCapture(c_id)
            print('Get3', c_id)
            ret, frame = cap.read()
            frame = cv2.resize(frame, (1920, 1080))
        q.put(frame)
        # print('q.qsize():', q.qsize())
        q.get() if q.qsize() > 1 else time.sleep(0.01)


# 获得视频流帧数图片，保存读入的视频
def video_get(q, window_name):
    loop_num = 0

    cv2.namedWindow(str(window_name), cv2.WINDOW_NORMAL)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # 视频保存路径
    path = "./TestData-1/"
    out = cv2.VideoWriter(path + str(window_name) + ".avi", fourcc, 20.0, (1920, 1080), True)

    while True:
        frame = q.get()
        out.write(frame)
        cv2.imshow(str(window_name), frame)

        loop_num += 1
        now_time = datetime.datetime.now()
        str_time_doc = now_time.strftime('%H%M%S-%f')
        if loop_num % 10 == 0:
            cv2.imwrite(path + str(window_name) + '-' + str_time_doc + '.jpg', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


# 获得视频流帧数图片，保存图片
def picture_get(q, cap_id):
    loop_num = 0
    file_address = "./TestData-1/"
    while True:
        rgb_frame = q.get()
        loop_num += 1
        now_time = datetime.datetime.now()
        str_time_doc = now_time.strftime('%H%M%S-%f')
        if loop_num % 10 == 0:
            cv2.imwrite(file_address + str(cap_id) + '-' + str_time_doc + '.jpg', rgb_frame)
        cv2.waitKey(1000)


# 解决进程问题
def run_multi_camera():
    # 摄像头编号
    camera_id_l = [0, 1, ]

    mp.set_start_method(method='spawn')  # init
    queues = [mp.Queue(maxsize=2) for _ in camera_id_l]

    processes = []
    for queue, camera_id in zip(queues, camera_id_l):
        processes.append(mp.Process(target=image_put, args=(queue, camera_id)))
        processes.append(mp.Process(target=video_get, args=(queue, camera_id)))
        # processes.append(mp.Process(target=picture_get, args=(queue, camera_id)))

    for process in processes:
        process.daemon = True
        process.start()
    for process in processes:
        process.join()


if __name__ == '__main__':
    run_multi_camera()  # 调用主函数
