import numpy as np
import CVFunc
import signal
import math
import binascii
import time
import cv2
import datetime
import multiprocessing as mp
import os



# 抓取图片，确认视频流的读入
def image_put(q, c_id):
    cap = cv2.VideoCapture(c_id)
    cap.set(6, 1196444237)
    cap.set(3, 640)
    cap.set(4, 360)
    cap.set(5, 30)
    if cap.isOpened():
        print('Get1', c_id)
    else:
        cap = cv2.VideoCapture(c_id)
        cap.set(6, 1196444237)
        cap.set(3, 640)
        cap.set(4, 360)
        cap.set(5, 30)
        print('Get2', c_id)

    while cap.isOpened():
        ret, frame = cap.read()
        # 抓取图片不成功再重新抓取
        if not ret:
            cap = cv2.VideoCapture(c_id)
            print('Get3', c_id)
            ret, frame = cap.read()
        q.put(frame)
        # print('q.qsize():', q.qsize())
        q.get() if q.qsize() > 1 else time.sleep(0.01)



# 调用相机获取图片进行测距
def distance_get(q_cap, c_id, q_v2a, q_yi, lock_ser, file_address, q_v2s):
    # 机身尺寸、光伏板尺寸
    vehicle_left = 118
    vehicle_right = 127
    vehicle_front = 117
    vehicle_width = vehicle_left + vehicle_right

    # 读取模型参数
    file_model = open('../Model-360.txt', 'r', encoding='utf-8')
    para_lines = file_model.readlines()
    file_model.close()
    model_F = float(para_lines[0].strip('\n'))
    model_W = float(para_lines[1].strip('\n'))
    model_a = float(para_lines[2].strip('\n'))
    model_b = float(para_lines[3].strip('\n'))
    principal_x = int(para_lines[4].strip('\n'))
    principal_y = int(para_lines[5].strip('\n'))

    # 循环处理图像
    loop_num = 0
    imu_roll = 0.0
    imu_pitch = 0.0
    imu_yaw = 0.0
    while True:
        start_time_total = time.time()
        start_time = time.time()
        str_Time = datetime.datetime.now().strftime('%H%M%S-%f')
        ret_mess = ''  # 数据信息
        time_mess = ''  # 时间信息
        # 获取图像及参数
        start_time = time.time()
        if not q_cap.empty():
            rgb_frame = q_cap.get()
            loop_num += 1
        else:
            continue
        img_height = int(rgb_frame.shape[0])
        img_width = int(rgb_frame.shape[1])
        mid_height = int(img_height / 2)
        mid_width = int(img_width / 2)
        rgb_show = rgb_frame.copy()
        end_time = time.time()
        time_mess += 'Cap:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 保留下半部分
        start_time = time.time()
        rgb_half = rgb_frame.copy()
        rgb_half[0:principal_y - 2, :] = (0, 0, 0)
        end_time = time.time()
        time_mess += 'Hal:' + str(round((end_time - start_time) * 1000, 4)) + ';'
        # 灰度并二值化
        start_time = time.time()
        gra_gray = cv2.cvtColor(rgb_half, cv2.COLOR_BGR2GRAY)
        thre_gray, gra_threshold = cv2.threshold(gra_gray, 140, 255, cv2.THRESH_BINARY)
        end_time = time.time()
        time_mess += 'Gra:' + str(round((end_time - start_time) * 1000, 4)) + ';'
        # Canny
        start_time = time.time()
        rgb_thres = cv2.cvtColor(gra_threshold, cv2.COLOR_GRAY2BGR)
        rgb_thres_hough = rgb_thres.copy()
        gra_canny = cv2.Canny(rgb_half, 100, 400)
        end_time = time.time()
        time_mess += 'Can:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 获取水平和垂直线
        start_time = time.time()
        gra_edge_rot = gra_canny.copy()
        gra_edge_rot[0:principal_y, :] = 0
        lines = cv2.HoughLinesP(gra_edge_rot, rho=1.0, theta=np.pi / 180, threshold=50, minLineLength=50,
                                maxLineGap=5)
        end_time = time.time()
        time_mess += 'Hou:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 旋转校正、拉平、融合
        gra_hough = np.zeros((img_height, img_width), np.uint8)  # 创建个全0的黑背景
        # 垂直线计算偏航角参数
        ver_avg = 0.0
        ver_weight = 0.0
        ver_sum = 0.0
        angles_ver = [0.0]
        num_yaw_l = 0
        num_yaw_r = 0
        # 水平线计算偏航角参数
        hor_avg = 0.0
        hor_weight = 0.0
        hor_sum = 0.0
        angles_hor = [0.0]

        num_ver = 0
        num_hor = 0
        # 提取浅色矩形参数
        rectangles_front = [[0.0, 0.0, 0.0, 0.0]]  # 垂直矩形，0是左边，1是右边，2是下边，3是上边
        rectangles_left = [[0.0, 0.0, 0.0, 0.0]]  # 垂直矩形，0是左边，1是右边，2是下边，3是上边
        rectangles_right = [[0.0, 0.0, 0.0, 0.0]]  # 垂直矩形，0是左边，1是右边，2是下边，3是上边
        lines_left = [[0.0, 0.0, 0.0]]  # 0是最接近中轴的w（x）值，1是最接近相机的h（y）值，2是最远离相机的h（y）值
        lines_right = [[0.0, 0.0, 0.0]]  # 0是最接近中轴的w（x）值，1是最接近相机的h（y）值，2是最远离相机的h（y）值
        num_dis_l = 0
        num_dis_r = 0
        lines_front = [[0.0, 0.0, 0.0]]  # 0是最接近相机的h（y）值，1是最左侧（最小）的w（x）值，2是最右侧（最大）的w（x）值
        num_dis_f = 0

        # 输出数据
        yaw_avg = -999.9
        side_left = -999.9
        side_right = -999.9

        try:
            if lines is not None:
                start_time = time.time()
                # 初次计算偏航角
                for line in lines:
                    for x1_p, y1_p, x2_p, y2_p in line:
                        if CVFunc.getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 50.0:
                            # cv2.line(gra_hough, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                            # 根据偏航角进行旋转
                            h1 = CVFunc.calc_horizontal(y1_p, model_F, model_W, model_a, model_b)
                            h2 = CVFunc.calc_horizontal(y2_p, model_F, model_W, model_a, model_b)
                            w1 = CVFunc.calc_vertical((x1_p - principal_x), y1_p, model_F, model_W, model_a, model_b)
                            w2 = CVFunc.calc_vertical((x2_p - principal_x), y2_p, model_F, model_W, model_a, model_b)
                            # 水平距离200、垂直距离100以内
                            if (h1 < 200 or h2 < 200) and (abs(w1) < 200 or abs(w2) < 200):
                                # cv2.line(rgb_thres_hough, (x1_p, y1_p), (x2_p, y2_p), (0, 0, 255), 2)
                                if h1 != h2:
                                    angle_tan = np.arctan((w1 - w2) / (h2 - h1)) * 57.29577
                                else:
                                    angle_tan = 90.0

                                x_mid = int((x1_p + x2_p) / 2)
                                y_mid = int((y1_p + y2_p) / 2)
                                cv2.line(gra_hough, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                                cv2.putText(gra_hough, str(round(angle_tan, 2)), (x_mid, y_mid),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255, 1)

                                if abs(angle_tan) < 45:
                                    ver_sum += angle_tan
                                    if w1 < 0 and w2 < 0:
                                        num_yaw_l += 1
                                    elif w1 >= 0 and w2 >= 0:
                                        num_yaw_r += 1
                                    else:
                                        num_yaw_l += 1
                                    if num_ver == 0:
                                        num_ver = 1
                                        angles_ver[0] = angle_tan
                                    else:
                                        num_ver += 1
                                        angles_ver.append(angle_tan)
                                else:
                                    if angle_tan <= -45:
                                        angle_tan = 180 + angle_tan
                                    hor_sum += angle_tan
                                    if num_hor == 0:
                                        num_hor = 1
                                        angles_hor[0] = angle_tan
                                    else:
                                        num_hor += 1
                                        angles_hor.append(angle_tan)
                # 算数平均垂直线角度计算视觉偏航角
                if num_ver > 0:
                    temp_avg = ver_sum / num_ver
                    for angle_ver in angles_ver:
                        if abs(temp_avg - angle_ver) < 10.0:
                            ver_avg += angle_ver
                            ver_weight += 1
                    if ver_weight > 0:
                        ver_avg = ver_avg / ver_weight
                yaw_avg = ver_avg
                # 根据垂直线角度和水平线角度，输出偏航角yaw_avg，统一为偏左为负，偏右为正
                # 如果计算了偏航角，则反馈给IMU进程和计算进程
                if ver_avg != 0.0 and num_yaw_l != 0 and num_yaw_r != 0:
                    q_yi.put(yaw_avg)
                    q_yi.get() if q_yi.qsize() > 1 else time.sleep(0.001)
                else:
                    yaw_avg = -999.9
                end_time = time.time()
                time_mess += 'Yaw:' + str(round((end_time - start_time) * 1000, 4)) + ';'

                # 如果识别到了偏航角，则进行旋转校正
                if yaw_avg != -999.9:
                    # 用均值的±10范围剔除误差数据，更新垂直线和水平线的角度。垂直线是-45至45，水平线是45至135
                    start_time = time.time()
                    for line in lines:
                        for x1_p, y1_p, x2_p, y2_p in line:
                            if CVFunc.getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 100.0:
                                h1 = CVFunc.calc_horizontal(y1_p, model_F, model_W, model_a, model_b)
                                h2 = CVFunc.calc_horizontal(y2_p, model_F, model_W, model_a, model_b)
                                w1 = CVFunc.calc_vertical((x1_p - principal_x), y1_p, model_F, model_W, model_a,
                                                          model_b)
                                w2 = CVFunc.calc_vertical((x2_p - principal_x), y2_p, model_F, model_W, model_a,
                                                          model_b)
                                # 水平距离1000、垂直距离600以内，根据偏航角进行旋转
                                if (h1 < 1000 and h2 < 1000) and (abs(w1) < 700 and abs(w2) < 700):
                                    if h1 != h2:
                                        angle_tan = np.arctan((w1 - w2) / (h2 - h1)) * 57.29577
                                    else:
                                        angle_tan = 90.0
                                    x1_imu, y1_imu = CVFunc.points_rotate(yaw_avg, w1, h1)
                                    x2_imu, y2_imu = CVFunc.points_rotate(yaw_avg, w2, h2)
                                    # 在yaw的均值±10内，认定为垂直线或水平线，拉直
                                    if abs(angle_tan) < 45.0:
                                        if abs(angle_tan - ver_avg) < 10.0:
                                            temp_value = str(x1_imu) + '-' + str(x2_imu)
                                            x_mid = int((x1_p + x2_p) / 2)
                                            y_mid = int((y1_p + y2_p) / 2)
                                            cv2.line(rgb_thres_hough, (x1_p, y1_p), (x2_p, y2_p), (255, 0, 0), 2)
                                            # cv2.line(rgb_show, (x1_p, y1_p), (x2_p, y2_p), (255, 0, 0), 2)
                                            # cv2.putText(rgb_thres_hough, temp_value, (x_mid, y_mid),
                                            #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
                                            # print('Ver:' + temp_value)
                                            # 垂直线,按照最接近中轴来拉直
                                            if abs(x1_imu) > abs(x2_imu):
                                                x1_imu = x2_imu
                                            else:
                                                x2_imu = x1_imu
                                            temp_button = min(y1_imu, y2_imu)
                                            temp_top = max(y1_imu, y2_imu)
                                            # 分别存进左右线段数组
                                            if x1_imu < 0.0:
                                                if num_dis_l == 0:
                                                    lines_left[0] = [x1_imu, temp_button, temp_top]
                                                    num_dis_l = 1
                                                else:
                                                    lines_left.append([x1_imu, temp_button, temp_top])
                                                    num_dis_l += 1
                                            else:
                                                if num_dis_r == 0:
                                                    lines_right[0] = [x1_imu, temp_button, temp_top]
                                                    num_dis_r = 1
                                                else:
                                                    lines_right.append([x1_imu, temp_button, temp_top])
                                                    num_dis_r += 1
                                    # elif ((400 < h1 or h2 < 800) and abs(w1 - w2) < 200) or (
                                    #         h1 or h2 <= 400 and abs(w1 - w2) < 75):
                                    else:
                                        if angle_tan <= -45.0:
                                            angle_tan = 180.0 + angle_tan
                                        angle_tan = angle_tan - 90.0

                                        if abs(angle_tan - yaw_avg) < 10.0:
                                            temp_value = str(y1_imu) + '-' + str(y2_imu)
                                            x_mid = int((x1_p + x2_p) / 2)
                                            y_mid = int((y1_p + y2_p) / 2)
                                            cv2.line(rgb_thres_hough, (x1_p, y1_p), (x2_p, y2_p), (0, 255, 0), 2)
                                            # cv2.line(rgb_show, (x1_p, y1_p), (x2_p, y2_p), (0, 255, 0), 2)
                                            # cv2.putText(rgb_thres_hough, temp_value, (x_mid, y_mid),
                                            #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                                            # print('Hor:' + temp_value)
                                            # 水平线,按照最接近相机来拉平
                                            if abs(y1_imu) > abs(y2_imu):
                                                y1_imu = y2_imu
                                            else:
                                                y2_imu = y1_imu
                                            temp_left = min(x1_imu, x2_imu)
                                            temp_right = max(x1_imu, x2_imu)
                                            # 存进前向线段数组
                                            if num_dis_f == 0:
                                                lines_front[0] = [y1_imu, temp_left, temp_right]
                                                num_dis_f = 1
                                            else:
                                                lines_front.append([y1_imu, temp_left, temp_right])
                                                num_dis_f += 1
                    end_time = time.time()
                    time_mess += 'Lin:' + str(round((end_time - start_time) * 1000, 4)) + ';'
                    start_time = time.time()
                    # 从左向右，根据左侧线段建立左侧矩形
                    if num_dis_l > 1:
                        lines_left.sort(reverse=False)
                        rectangles_left[0] = [lines_left[0][0], lines_left[0][0], lines_left[0][1], lines_left[0][2]]
                        for line_l in lines_left:
                            has_change = False
                            for rect_l in rectangles_left:
                                if 0.0 < line_l[0] - rect_l[1] <= 100.0:
                                    temp_left = rect_l[0]
                                    temp_right = line_l[0]
                                    temp_button = rect_l[2]
                                    temp_top = rect_l[3]
                                    temp_width = abs(temp_right - temp_left)
                                    temp_length = temp_top - temp_button

                                    num_list_255 = 0
                                    num_list_0 = 0
                                    w1_s = temp_left
                                    w2_s = temp_right
                                    for i in range(3, 6, 1):
                                        h_s = temp_top - temp_length * i / 6
                                        x1_s, y1_s = CVFunc.points_dis2xy(yaw_avg, w1_s, h_s, model_F, model_W,
                                                                          model_a, model_b, principal_x)
                                        x2_s, y2_s = CVFunc.points_dis2xy(yaw_avg, w2_s, h_s, model_F, model_W,
                                                                          model_a, model_b, principal_x)
                                        if y1_s >= img_height:
                                            y1_s = img_height - 1
                                        if x1_s <= 0:
                                            x1_s = 1
                                        elif x1_s >= img_width:
                                            x1_s = img_width - 1
                                        if y2_s >= img_height:
                                            y2_s = img_height - 1
                                        if x2_s <= 0:
                                            x2_s = 1
                                        elif x2_s >= img_width:
                                            x2_s = img_width - 1
                                        num_255 = 0
                                        num_0 = 0
                                        for x12_s in range(min(x1_s, x2_s), max(x1_s, x2_s), 1):
                                            if gra_threshold[y1_s, x12_s] == 255:
                                                num_255 += 1
                                            else:
                                                num_0 += 1
                                        if num_255 + num_0 > 0:
                                            if num_255 / (num_0 + num_255) > 0.8:  # 如果采样点中白色多于80%
                                                num_list_255 += 1
                                            else:
                                                num_list_0 += 1
                                    if num_list_255 > num_list_0:
                                        has_change = True
                                        rect_l[1] = line_l[0]
                                        if line_l[1] < rect_l[2]:
                                            rect_l[2] = line_l[1]
                                        if line_l[2] > rect_l[3]:
                                            rect_l[3] = line_l[2]

                                elif rect_l[1] == line_l[0]:
                                    has_change = True
                                    if line_l[1] < rect_l[2]:
                                        rect_l[2] = line_l[1]
                                    if line_l[2] > rect_l[3]:
                                        rect_l[3] = line_l[2]
                            if not has_change:
                                rectangles_left.append([line_l[0], line_l[0], line_l[1], line_l[2]])
                        rgb_show, dis_l = CVFunc.draw_rectangles(True, rectangles_left, rgb_show, 15.0, yaw_avg,
                                                                 model_F, model_W, model_a, model_b, principal_x)
                        if dis_l[0][0] != 0.0:
                            dis_l.sort(reverse=False)
                            cam_left = abs(dis_l[0][1])
                            if yaw_avg >= 0.0:
                                side_left = round((cam_left - vehicle_front * math.sin(
                                    math.radians(abs(yaw_avg))) - vehicle_left * math.sin(
                                    math.radians(90.0 - abs(yaw_avg)))),
                                                  1)
                            else:
                                side_left = round((cam_left + vehicle_front * math.sin(
                                    math.radians(abs(yaw_avg))) - vehicle_left * math.sin(
                                    math.radians(90.0 - abs(yaw_avg)))),
                                                  1)
                        else:
                            side_left = -999.9
                    else:
                        side_left = -999.9

                    # 从右向左，根据右侧线段建立右侧矩形
                    if num_dis_r > 0:
                        lines_right.sort(reverse=True)
                        rectangles_right[0] = [lines_right[0][0], lines_right[0][0], lines_right[0][1],
                                               lines_right[0][2]]
                        for line_r in lines_right:
                            has_change = False
                            for rect_r in rectangles_right:
                                if 0.0 < rect_r[0] - line_r[0] <= 100.0:
                                    temp_left = line_r[0]
                                    temp_right = rect_r[1]
                                    temp_button = rect_r[2]
                                    temp_top = rect_r[3]
                                    temp_width = abs(temp_right - temp_left)
                                    temp_length = temp_top - temp_button

                                    num_list_255 = 0
                                    num_list_0 = 0
                                    w1_s = temp_left
                                    w2_s = temp_right
                                    for i in range(3, 6, 1):
                                        h_s = temp_top - temp_length * i / 6
                                        x1_s, y1_s = CVFunc.points_dis2xy(yaw_avg, w1_s, h_s, model_F, model_W,
                                                                          model_a, model_b, principal_x)
                                        x2_s, y2_s = CVFunc.points_dis2xy(yaw_avg, w2_s, h_s, model_F, model_W,
                                                                          model_a, model_b, principal_x)
                                        if y1_s >= img_height:
                                            y1_s = img_height - 1
                                        if x1_s <= 0:
                                            x1_s = 1
                                        elif x1_s >= img_width:
                                            x1_s = img_width - 1
                                        if y2_s >= img_height:
                                            y2_s = img_height - 1
                                        if x2_s <= 0:
                                            x2_s = 1
                                        elif x2_s >= img_width:
                                            x2_s = img_width - 1
                                        num_255 = 0
                                        num_0 = 0
                                        for x12_s in range(min(x1_s, x2_s), max(x1_s, x2_s), 1):
                                            if gra_threshold[y1_s, x12_s] == 255:
                                                num_255 += 1
                                            else:
                                                num_0 += 1
                                        if num_255 + num_0 > 0:
                                            if num_255 / (num_0 + num_255) > 0.8:  # 如果采样点中白色多于80%
                                                num_list_255 += 1
                                            else:
                                                num_list_0 += 1
                                    if num_list_255 > num_list_0:
                                        has_change = True
                                        rect_r[0] = line_r[0]
                                        rect_r[2] = min(line_r[1], rect_r[2])
                                        rect_r[3] = max(line_r[2], rect_r[3])
                                elif rect_r[0] == line_r[0]:
                                    has_change = True
                                    rect_r[2] = min(line_r[1], rect_r[2])
                                    rect_r[3] = max(line_r[2], rect_r[3])
                            if not has_change:
                                rectangles_right.append([line_r[0], line_r[0], line_r[1], line_r[2]])
                        rgb_show, dis_r = CVFunc.draw_rectangles(True, rectangles_right, rgb_show, 15.0, yaw_avg,
                                                                 model_F, model_W, model_a, model_b, principal_x)
                        if dis_r[0][0] != 0.0:
                            dis_r.sort(reverse=True)
                            cam_right = dis_r[0][0]
                            if yaw_avg >= 0:
                                side_right = round((cam_right + vehicle_front * math.sin(
                                    math.radians(abs(yaw_avg))) - vehicle_right * math.sin(
                                    math.radians(90.0 - abs(yaw_avg)))), 1)
                            else:
                                side_right = round((cam_right - vehicle_front * math.sin(
                                    math.radians(abs(yaw_avg))) - vehicle_right * math.sin(
                                    math.radians(90.0 - abs(yaw_avg)))), 1)
                        else:
                            side_right = -999.9
                    else:
                        side_right = -999.9

                    # 从近到远，根据最近线段建立前向矩形
                    if num_dis_f > 1:
                        lines_front.sort(reverse=False)
                        rectangles_front[0] = [lines_front[0][1], lines_front[0][2], lines_front[0][0],
                                               lines_front[0][0]]
                        for line_f in lines_front:
                            has_change = False
                            for rect_f in rectangles_front:
                                if 0.0 < line_f[0] - rect_f[3] <= 30.0:
                                    temp_left = rect_f[0]
                                    temp_right = rect_f[1]
                                    temp_button = rect_f[2]
                                    temp_top = line_f[0]
                                    temp_width = temp_right - temp_left
                                    temp_length = temp_top - temp_button

                                    num_list_255 = 0
                                    num_list_0 = 0
                                    h1_s = temp_top
                                    h2_s = temp_button
                                    for i in range(1, 4, 1):
                                        w_s = temp_left + temp_width * i / 4
                                        x1_s, y1_s = CVFunc.points_dis2xy(yaw_avg, w_s, h1_s, model_F, model_W,
                                                                          model_a, model_b, principal_x)
                                        x2_s, y2_s = CVFunc.points_dis2xy(yaw_avg, w_s, h2_s, model_F, model_W,
                                                                          model_a, model_b, principal_x)
                                        if y1_s >= img_height:
                                            y1_s = img_height - 1
                                        if x1_s <= 0:
                                            x1_s = 1
                                        elif x1_s >= img_width:
                                            x1_s = img_width - 1
                                        if y2_s >= img_height:
                                            y2_s = img_height - 1
                                        if x2_s <= 0:
                                            x2_s = 1
                                        elif x2_s >= img_width:
                                            x2_s = img_width - 1
                                        num_255 = 0
                                        num_0 = 0
                                        for y12_s in range(min(y1_s, y2_s), max(y1_s, y2_s), 1):
                                            if gra_threshold[y12_s, x1_s] == 255:
                                                num_255 += 1
                                            else:
                                                num_0 += 1
                                        if num_255 + num_0 > 0:
                                            if num_255 / (num_0 + num_255) > 0.8:  # 如果采样点中白色多于80%
                                                num_list_255 += 1
                                            else:
                                                num_list_0 += 1
                                    if num_list_255 > num_list_0:
                                        has_change = True
                                        rect_f[2] = line_f[0]
                                        rect_f[0] = min(rect_f[0], line_f[1])
                                        rect_f[1] = max(rect_f[1], line_f[2])
                                elif rect_f[2] <= line_f[0] <= rect_f[3]:
                                    has_change = True
                                    rect_f[0] = min(rect_f[0], line_f[1])
                                    rect_f[1] = max(rect_f[1], line_f[2])
                            if not has_change:
                                rectangles_front.append([line_f[1], line_f[2], line_f[0], line_f[0]])

                    end_time = time.time()
                    time_mess += 'Ret:' + str(round((end_time - start_time) * 1000, 4)) + ';'
                else:
                    yaw_avg = -999.9
                    side_left = -999.9
                    side_right = -999.9
        except Exception as e:
            print(e)
            print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
            print(f"error line:{e.__traceback__.tb_lineno}")
        finally:
            pass
            # print(yaw_avg)
            # cv2.imshow('canny', gra_canny)
            # cv2.imshow('thre', gra_threshold)
            # cv2.imshow('Hough', gra_hough)
            # cv2.imshow('ShowRect', rgb_show)
            # cv2.waitKey(1)
        # 保存txt，传输数据
        start_time = time.time()
        send_list = [yaw_avg, side_left, side_right]
        q_v2a.put(send_list)
        q_v2a.get() if q_v2a.qsize() > 1 else time.sleep(0.001)
        q_v2s.put(send_list)
        q_v2s.get() if q_v2s.qsize() > 1 else time.sleep(0.001)
        # cv2.imwrite(file_address + 'C' + str(cap_id) + '-' + str_Time + '.jpg', rgb_frame)
        # cv2.imwrite(file_address + 'D' + str(cap_id) + '-' + str_Time + '.jpg', rgb_rot)
        file_rec = open(file_address + 'Dis.txt', 'a')
        end_time = time.time()
        time_mess += 'Shw:' + str(round((end_time - start_time) * 1000, 4)) + ';'
        end_time_total = time.time()
        time_mess += 'All:' + str(round((end_time_total - start_time_total) * 1000, 4)) + ';\n'
        if len(time_mess) > 0:
            file_rec.write('Tpy:Timer;' + time_mess)
        file_rec.close()

