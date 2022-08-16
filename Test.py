import multiprocessing as mp
import cv2
import time
import datetime


# 抓取图片，确认视频流的读入
def image_put(q, c_id, file_address):
    cap = cv2.VideoCapture(c_id)
    cap.set(6, 1196444237)
    cap.set(3, 1920)
    cap.set(4, 1080)
    cap.set(5, 30)
    # 获取视频帧率
    # fps = cap.get(cv2.CAP_PROP_FPS)
    # size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    if cap.isOpened():
        print('Get1', c_id)
    else:
        cap = cv2.VideoCapture(c_id)
        cap.set(6, 1196444237)
        cap.set(3, 1920)
        cap.set(4, 1080)
        cap.set(5, 30)
        # fps = cap.get(cv2.CAP_PROP_FPS)
        print('Get2', c_id)

    while cap.isOpened():
        # start_time = time.time()
        ret, frame = cap.read()
        # print('ret:', ret)
        # frame = cv2.resize(frame, (1920, 1080))
        # 抓取图片不成功再重新抓取
        if not ret:
            cap = cv2.VideoCapture(c_id)
            print('Get3', c_id)
            ret, frame = cap.read()
            # frame = cv2.resize(frame, (1920, 1080))
        q.put(frame)
        # print('q.qsize():', q.qsize())
        q.get() if q.qsize() > 1 else time.sleep(0.01)
        # end_time = time.time()
        # print('C' + str(c_id) + str(round((end_time - start_time) * 1000, 4)))


# 获得视频流帧数图片，保存图片
def picture_get(q, q_s, cap_id, file_address):
    while True:
        time.sleep(2)
        rgb_frame = q.get()
        now_time = datetime.datetime.now()
        str_time_doc = now_time.strftime('%H%M%S-%f')
        cv2.imwrite(file_address + str(cap_id) + '-' + str_time_doc + '.jpg', rgb_frame)
        # print(str(cap_id) + '-' + str_time_doc)
        q_s.put(str(cap_id) + '-' + str_time_doc)
        if q_s.qsize() > 1:
            q.get()


def data_collect(q0, q1, file_address):
    str_c0 = 'E0'
    str_c1 = 'E1'
    temp_0 = ''
    temp_1 = ''
    while True:
        if q0.qsize() > 1:
            temp_0 = q0.get()
        if q1.qsize() > 1:
            temp_1 = q1.get()
        print(str_c0, str_c1)
        time.sleep(1)

# 解决进程问题
def run_multi_camera():
    str_fileAddress = './TestData/'
    mp.set_start_method(method='spawn')  # init
    queue_c0 = mp.Queue(maxsize=2)
    queue_c1 = mp.Queue(maxsize=2)
    queue_s0 = mp.Queue(maxsize=2)
    queue_s1 = mp.Queue(maxsize=2)

    processes = [mp.Process(target=image_put, args=(queue_c0, 0, str_fileAddress)),
                 mp.Process(target=picture_get, args=(queue_c0, queue_s0, 0, str_fileAddress)),
                 mp.Process(target=image_put, args=(queue_c1, 1, str_fileAddress)),
                 mp.Process(target=picture_get, args=(queue_c1, queue_s1, 1, str_fileAddress)),
                 mp.Process(target=data_collect, args=(queue_s0, queue_s1, str_fileAddress))]

    for process in processes:
        process.daemon = True
        process.start()
    for process in processes:
        process.join()


if __name__ == '__main__':
    run_multi_camera()  # 调用主函数
