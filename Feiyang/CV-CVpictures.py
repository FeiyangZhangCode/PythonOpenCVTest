import os
import time
import datetime
import cv2
import CVFunc
import numpy as np
import math
import xlrd3

# 开始主程序
if __name__ == '__main__':
    # 读取模型参数
    file_model = open('Model.txt', 'r', encoding='utf-8')
    para_lines = file_model.readlines()
    file_model.close()

    minCan = 40
    maxCan = 100

    # 新建文件夹,读取时间作为文件名
    str_fileAddress = './TestData/'
    str_fileHome = str_fileAddress + 'C/'
    str_Time = datetime.datetime.now().strftime('%Y%m%d-%H%M')
    file_address = str_fileAddress + str_Time
    if not os.path.exists(file_address):
        os.makedirs(file_address)
    file_address += '/'

    # 提取两个摄像头记录的Excel表
    workbook_0 = xlrd3.open_workbook('../TestData/C/0.xls')
    table_0 = workbook_0.sheet_by_name('Data')
    nrows_0 = table_0.nrows
    workbook_1 = xlrd3.open_workbook('../TestData/C/1.xls')
    table_1 = workbook_1.sheet_by_name('Data')
    nrows_1 = table_1.nrows

    # 按照名称调用，文件夹内已按名称排序
    title_li = os.listdir(str_fileHome)
    # title_li = sorted(title_li, key=lambda x: os.path.getmtime(os.path.join(str_fileHome, x)), reverse=False)
    loop_num = 0
    cam_front = 0
    cam_back = 0
    cam_left_f = 0
    cam_left_b = 0
    cam_right_f = 0
    cam_right_b = 0
    cam_left = 0
    cam_right = 0
    dis_ff = [0]
    dis_lf = [0]
    dis_rf = [0]
    dis_fb = [0]
    dis_lb = [0]
    dis_rb = [0]

    show_width = 960
    show_height = 540
    half_width = int(show_width / 2)
    half_height = int(show_height / 2)
    rgb_c0 = np.zeros((show_height, show_width, 3), np.uint8)
    rgb_c1 = np.zeros((show_height, show_width, 3), np.uint8)

    for title in title_li:
        img_name = title[0:13]
        cap_id = title[15:16]

        # 根据相机编号分配模型参数
        if int(cap_id) == 0:
            model_F = float(para_lines[0].strip('\n'))
            model_W = float(para_lines[1].strip('\n'))
            model_a = float(para_lines[2].strip('\n'))
            model_b = float(para_lines[3].strip('\n'))
            principal_x = int(para_lines[4].strip('\n'))
            principal_y = int(para_lines[5].strip('\n'))
        else:
            model_F = float(para_lines[6].strip('\n'))
            model_W = float(para_lines[7].strip('\n'))
            model_a = float(para_lines[8].strip('\n'))
            model_b = float(para_lines[9].strip('\n'))
            principal_x = int(para_lines[10].strip('\n'))
            principal_y = int(para_lines[11].strip('\n'))

        start_time_total = time.time()
        str_Time = datetime.datetime.now().strftime('%H%M%S-%f')
        loop_num += 1
        ret_mess = ''  # 数据信息
        time_mess = ''  # 时间信息

        # 获取图像及参数
        start_time = time.time()
        rgb_frame = cv2.imread(str_fileHome + title)

        img_height = int(rgb_frame.shape[0])
        img_width = int(rgb_frame.shape[1])
        mid_height = int(img_height / 2)
        mid_width = int(img_width / 2)
        rgb_rot = rgb_frame.copy()
        rgb_show = rgb_frame.copy()
        # 获取偏航角
        angle_set = 0.0
        if cap_id == '0':
            for i in range(0, nrows_0, 1):
                time_name = table_0.cell_value(i, 0)
                if time_name == img_name:
                    angle_set = float(table_0.cell_value(i, 1))
                    break
        else:
            for i in range(0, nrows_1, 1):
                time_name = table_1.cell_value(i, 0)
                if time_name == img_name:
                    angle_set = float(table_1.cell_value(i, 1))
                    break
        end_time = time.time()
        time_mess += 'Cap:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # Canny提取边界，保留下半部分
        start_time = time.time()
        rgb_half = rgb_frame.copy()
        rgb_half[0:principal_y - 50, :] = (0, 0, 0)
        gra_edge = cv2.Canny(rgb_half, minCan, maxCan)
        end_time = time.time()
        time_mess += 'Can:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 获取水平和垂直线
        start_time = time.time()
        gra_edge_rot = gra_edge.copy()
        gra_edge_rot[0:principal_y, :] = 0
        lines = cv2.HoughLinesP(gra_edge_rot, rho=1.0, theta=np.pi / 180, threshold=100, minLineLength=100,
                                maxLineGap=5)
        end_time = time.time()
        time_mess += 'Hou:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        # 旋转校正、拉平、融合
        start_time = time.time()
        num_line = 0
        num_ver = 0
        num_hor = 0
        num_front = 0
        num_left = 0
        num_right = 0
        data_ver = np.zeros((0, 3), dtype=float)  # 垂直线数据，0是最接近中轴的w（x）值，1是最接近相机的h（y）值，2是最远离相机的h（y）值
        data_hor = np.zeros((0, 3), dtype=float)  # 水平线数据，0是最接近相机的h（y）值，1是最左侧（最小）的w（x）值，2是最右侧（最大）的w（x）值
        dis_f = [0]
        dis_l = [0]
        dis_r = [0]
        try:
            if str(type(lines)) != "<class 'NoneType'>":
                if len(lines) > 0:
                    for line in lines:
                        for x1_p, y1_p, x2_p, y2_p in line:
                            if CVFunc.getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 50.0:
                                # 根据偏航角进行旋转
                                h1 = CVFunc.calc_horizontal(y1_p, model_F, model_W, model_a, model_b)
                                h2 = CVFunc.calc_horizontal(y2_p, model_F, model_W, model_a, model_b)
                                w1 = CVFunc.calc_vertical((x1_p - principal_x), y1_p, model_F, model_W, model_a,
                                                          model_b)
                                w2 = CVFunc.calc_vertical((x2_p - principal_x), y2_p, model_F, model_W, model_a,
                                                          model_b)
                                # 在2米范围内
                                if h1 < 2000 and h2 < 2000 and w1 < 2000 and w2 < 2000:
                                    num_line += 1
                                    x1_imu, y1_imu = CVFunc.points_rotate(angle_set, w1, h1)
                                    x2_imu, y2_imu = CVFunc.points_rotate(angle_set, w2, h2)

                                    # 调整后处理不为水平线或者垂直线的线段，进行拉平拉直，小于±15认为是水平线，大于±75认为是垂直线
                                    if x1_imu != x2_imu and y1_imu != y2_imu:
                                        temp_tan = np.arctan((y1_imu - y2_imu) / (x1_imu - x2_imu)) * 57.29577
                                        if abs(temp_tan) <= 15:  # 判断是水平线,按照最接近相机来拉平
                                            if abs(x1_imu) > abs(x2_imu):
                                                y1_imu = y2_imu
                                            else:
                                                y2_imu = y1_imu
                                        elif abs(temp_tan) >= 75:  # 判断是垂直线,按照最接近中轴来拉直
                                            if abs(x1_imu) > abs(x2_imu):
                                                x1_imu = x2_imu
                                            else:
                                                x2_imu = x1_imu

                                    # 如果是同一条线，进行融合
                                    if y1_imu == y2_imu:  # 水平线
                                        num_hor += 1
                                        # 如果距离中轴的差值不超过50，则认为是同一条，按照最接近中轴的值进行融合。否则新增为新一条。
                                        temp_left = min(x1_imu, x2_imu)
                                        temp_right = max(x1_imu, x2_imu)
                                        if len(data_hor) == 0:  # 第1条水平线，直接保存
                                            data_hor = np.append(data_hor, [[y1_imu, temp_left, temp_right]], axis=0)
                                        else:
                                            new_hor = True
                                            for values_hor in data_hor:
                                                if abs(values_hor[0] - y1_imu) < 50:
                                                    if abs((values_hor[2] + values_hor[1]) / 2) > abs(
                                                            (temp_right + temp_left) / 2):
                                                        values_hor[0] = y1_imu
                                                    values_hor[1] = min(values_hor[1], temp_left)
                                                    values_hor[2] = max(values_hor[2], temp_right)
                                                    new_hor = False
                                                    break
                                            if new_hor:
                                                data_hor = np.append(data_hor, [[y1_imu, temp_left, temp_right]],
                                                                     axis=0)
                                    elif x1_imu == x2_imu:  # 垂直线
                                        num_ver += 1
                                        # 如果距离中轴的差值不超过50，则认为是同一条，按最接近中轴进行融合。否则新增为新一条。
                                        temp_button = min(y1_imu, y2_imu)
                                        temp_top = max(y1_imu, y2_imu)
                                        if len(data_ver) == 0:
                                            data_ver = np.append(data_ver, [[x1_imu, temp_button, temp_top]], axis=0)
                                        else:
                                            new_ver = True
                                            for values_ver in data_ver:
                                                if abs(values_ver[0] - x1_imu) < 50:
                                                    if abs(values_ver[0]) > abs(x1_imu):
                                                        values_ver[0] = x1_imu
                                                    temp_v = min(values_ver[1], temp_button)
                                                    values_ver[1] = temp_v
                                                    temp_v = max(values_ver[2], temp_top)
                                                    values_ver[2] = temp_v
                                                    new_ver = False
                                                    break
                                            if new_ver:
                                                data_ver = np.append(data_ver, [[x1_imu, temp_button, temp_top]],
                                                                     axis=0)
                                    else:
                                        pass
                    end_time = time.time()
                    time_mess += 'Rot:' + str(round((end_time - start_time) * 1000, 4)) + ';'

                    # 反推图像上的直线位置，找出最近的水平线和左右垂直线
                    start_time = time.time()
                    if len(data_hor) > 0:
                        for values_hor in data_hor:
                            w1_b, h1_b = CVFunc.points_rotate(-angle_set, values_hor[1], values_hor[0])
                            w2_b, h2_b = CVFunc.points_rotate(-angle_set, values_hor[2], values_hor[0])
                            y1_b = CVFunc.calc_h2y(h1_b, model_F, model_W, model_a, model_b)
                            y2_b = CVFunc.calc_h2y(h2_b, model_F, model_W, model_a, model_b)
                            x1_b = CVFunc.calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
                            x2_b = CVFunc.calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
                            x_mid = int((x1_b + x2_b) / 2)
                            y_mid = int((y1_b + y2_b) / 2)
                            cv2.line(rgb_rot, (x1_b, y1_b), (x2_b, y2_b), (255, 0, 0), 1)
                            cv2.putText(rgb_rot, str(round(values_hor[0], 0)), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.8, (255, 0, 0), 1)
                            cv2.line(rgb_show, (x1_b, y1_b), (x2_b, y2_b), (255, 0, 0), 1)
                            num_front += 1
                            if dis_f[0] == 0:
                                dis_f[0] = int(values_hor[0])
                            else:
                                dis_f.append(int(values_hor[0]))

                    if len(data_ver) > 0:
                        for values_ver in data_ver:
                            w1_b, h1_b = CVFunc.points_rotate(-angle_set, values_ver[0], values_ver[1])
                            w2_b, h2_b = CVFunc.points_rotate(-angle_set, values_ver[0], values_ver[2])
                            y1_b = CVFunc.calc_h2y(h1_b, model_F, model_W, model_a, model_b)
                            y2_b = CVFunc.calc_h2y(h2_b, model_F, model_W, model_a, model_b)
                            x1_b = CVFunc.calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
                            x2_b = CVFunc.calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
                            x_mid = int((x1_b + x2_b) / 2)
                            y_mid = int((y1_b + y2_b) / 2)
                            cv2.line(rgb_rot, (x1_b, y1_b), (x2_b, y2_b), (0, 255, 0), 1)
                            cv2.putText(rgb_rot, str(round(values_ver[0], 0)), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.8,
                                        (0, 255, 0), 1)
                            cv2.line(rgb_show, (x1_b, y1_b), (x2_b, y2_b), (0, 255, 0), 6)
                            if values_ver[0] < 0:
                                num_left += 1
                                if dis_l[0] == 0:
                                    dis_l[0] = int(abs(values_ver[0]))
                                else:
                                    dis_l.append(int(abs(values_ver[0])))
                            else:
                                num_right += 1
                                if dis_r[0] == 0:
                                    dis_r[0] = int(values_ver[0])
                                else:
                                    dis_r.append(int(values_ver[0]))
                    end_time = time.time()
                    time_mess += 'Dra:' + str(round((end_time - start_time) * 1000, 4)) + ';'

        except Exception as e:
            print(e)
            print(f'error file:{e.__traceback__.tb_frame.f_globals["__file__"]}')
            print(f"error line:{e.__traceback__.tb_lineno}")

        # 画相机中心十字
        start_time = time.time()
        cv2.line(rgb_rot, (principal_x, 0), (principal_x, img_height), (255, 255, 0), 1)
        cv2.line(rgb_rot, (0, principal_y), (img_width, principal_y), (255, 255, 0), 1)
        cv2.circle(rgb_rot, (principal_x, principal_y), 5, (255, 255, 0), 3)

        # 开始计算边界
        if cap_id == '0':
            dis_ff = dis_f
            dis_lf = dis_l
            dis_rf = dis_r
        else:
            dis_fb = dis_f
            dis_lb = dis_r
            dis_rb = dis_l

        # 上下单目的左侧融合
        if dis_lf[0] != 0 and dis_lb[0] != 0:
            is_fb_left = False
            for i in range(0, len(dis_lf), 1):
                for j in range(0, len(dis_lb), 1):
                    if abs(dis_lf[i] - dis_lb[j]) < 20:
                        cam_left = int((dis_lf[i] + dis_lb[j]) / 2)
                        cam_left_f = dis_lf[i]
                        cam_left_b = dis_lb[j]
                        is_fb_left = True
                        break
                if is_fb_left:
                    break
        elif dis_lf[0] != 0 and cam_left != 0:
            for i in range(0, len(dis_lf), 1):
                if abs(dis_lf[i] - cam_left) < 20:
                    cam_left = int((cam_left * 2 + dis_lf[i]) / 3)
                    cam_left_f = dis_lf[i]
                    break
        elif dis_lb[0] != 0 and cam_left != 0:
            for i in range(0, len(dis_lb), 1):
                if abs(dis_lb[i] - cam_left) < 20:
                    cam_left = int((cam_left * 2 + dis_lb[i]) / 3)
                    cam_left_b = dis_lb[i]
                    break

        # 上下单目的右侧融合
        if dis_rf[0] != 0 and dis_rb[0] != 0:
            is_fb_right = False
            for i in range(0, len(dis_rf), 1):
                for j in range(0, len(dis_rb), 1):
                    if abs(dis_rf[i] - dis_rb[j]) < 20:
                        cam_right = int((dis_rf[i] + dis_rb[j]) / 2)
                        cam_right_f = dis_rf[i]
                        cam_right_b = dis_rb[j]
                        is_fb_right = True
                        break
                if is_fb_right:
                    break
        elif dis_rf[0] != 0 and cam_right != 0:
            for i in range(0, len(dis_rf), 1):
                if abs(dis_rf[i] - cam_right) < 20:
                    cam_right = int((cam_right + dis_rf[i]) / 2)
                    cam_right_f = dis_rf[i]
                    break
        elif dis_rb[0] != 0 and cam_right != 0:
            for i in range(0, len(dis_rb), 1):
                if abs(dis_rb[i] - cam_right) < 20:
                    cam_right = int((cam_right + dis_rb[i]) / 2)
                    cam_right_b = dis_rb[i]
                    break

        # 左上和左下展示图像
        if cap_id == '0':
            # 整体左边线
            if cam_left != 0:
                w1_b, h1_b = CVFunc.points_rotate(-angle_set, -cam_left, 100.0)
                w2_b, h2_b = CVFunc.points_rotate(-angle_set, -cam_left, 2000.0)
                y1_b = CVFunc.calc_h2y(h1_b, model_F, model_W, model_a, model_b)
                y2_b = CVFunc.calc_h2y(h2_b, model_F, model_W, model_a, model_b)
                x1_b = CVFunc.calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
                x2_b = CVFunc.calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
                x_mid = int((x1_b + x2_b) / 2)
                y_mid = int((y1_b + y2_b) / 2)
                cv2.line(rgb_rot, (x1_b, y1_b), (x2_b, y2_b), (0, 0, 255), 1)
                cv2.putText(rgb_rot, str(cam_left), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 0, 255), 1)
                cv2.line(rgb_show, (x1_b, y1_b), (x2_b, y2_b), (0, 0, 255), 6)
            # 整体右边线
            if cam_right != 0:
                w1_b, h1_b = CVFunc.points_rotate(-angle_set, cam_right, 100.0)
                w2_b, h2_b = CVFunc.points_rotate(-angle_set, cam_right, 2000.0)
                y1_b = CVFunc.calc_h2y(h1_b, model_F, model_W, model_a, model_b)
                y2_b = CVFunc.calc_h2y(h2_b, model_F, model_W, model_a, model_b)
                x1_b = CVFunc.calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
                x2_b = CVFunc.calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
                x_mid = int((x1_b + x2_b) / 2)
                y_mid = int((y1_b + y2_b) / 2)
                cv2.line(rgb_rot, (x1_b, y1_b), (x2_b, y2_b), (0, 0, 255), 1)
                cv2.putText(rgb_rot, str(cam_right), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 0, 255), 1)
                cv2.line(rgb_show, (x1_b, y1_b), (x2_b, y2_b), (0, 0, 255), 6)
            # 前向左边线
            if cam_left_f != 0:
                w1_b, h1_b = CVFunc.points_rotate(-angle_set, -cam_left_f, 100.0)
                w2_b, h2_b = CVFunc.points_rotate(-angle_set, -cam_left_f, 2000.0)
                y1_b = CVFunc.calc_h2y(h1_b, model_F, model_W, model_a, model_b)
                y2_b = CVFunc.calc_h2y(h2_b, model_F, model_W, model_a, model_b)
                x1_b = CVFunc.calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
                x2_b = CVFunc.calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
                cv2.line(rgb_show, (x1_b, y1_b), (x2_b, y2_b), (0, 165, 255), 6)
            # 前向右边线
            if cam_right_f != 0:
                w1_b, h1_b = CVFunc.points_rotate(-angle_set, cam_right_f, 100.0)
                w2_b, h2_b = CVFunc.points_rotate(-angle_set, cam_right_f, 2000.0)
                y1_b = CVFunc.calc_h2y(h1_b, model_F, model_W, model_a, model_b)
                y2_b = CVFunc.calc_h2y(h2_b, model_F, model_W, model_a, model_b)
                x1_b = CVFunc.calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
                x2_b = CVFunc.calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
                cv2.line(rgb_show, (x1_b, y1_b), (x2_b, y2_b), (0, 165, 255), 6)
            # 左侧车宽
            w1_b, h1_b = CVFunc.points_rotate(-0.0, -300.0, 100.0)
            w2_b, h2_b = CVFunc.points_rotate(-0.0, -300.0, 2000.0)
            y1_b = CVFunc.calc_h2y(h1_b, model_F, model_W, model_a, model_b)
            y2_b = CVFunc.calc_h2y(h2_b, model_F, model_W, model_a, model_b)
            x1_b = CVFunc.calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
            x2_b = CVFunc.calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
            cv2.line(rgb_show, (x1_b, y1_b), (x2_b, y2_b), (0, 0, 0), 3)
            # 右侧车宽
            w1_b, h1_b = CVFunc.points_rotate(-0.0, 300.0, 100.0)
            w2_b, h2_b = CVFunc.points_rotate(-0.0, 300.0, 2000.0)
            y1_b = CVFunc.calc_h2y(h1_b, model_F, model_W, model_a, model_b)
            y2_b = CVFunc.calc_h2y(h2_b, model_F, model_W, model_a, model_b)
            x1_b = CVFunc.calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
            x2_b = CVFunc.calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
            cv2.line(rgb_show, (x1_b, y1_b), (x2_b, y2_b), (0, 0, 0), 3)

            rgb_c0 = cv2.resize(rgb_show, (960, 540))
        else:
            # 整体左边线
            if cam_right != 0:
                w1_b, h1_b = CVFunc.points_rotate(-angle_set, -cam_right, 100.0)
                w2_b, h2_b = CVFunc.points_rotate(-angle_set, -cam_right, 2000.0)
                y1_b = CVFunc.calc_h2y(h1_b, model_F, model_W, model_a, model_b)
                y2_b = CVFunc.calc_h2y(h2_b, model_F, model_W, model_a, model_b)
                x1_b = CVFunc.calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
                x2_b = CVFunc.calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
                x_mid = int((x1_b + x2_b) / 2)
                y_mid = int((y1_b + y2_b) / 2)
                cv2.line(rgb_rot, (x1_b, y1_b), (x2_b, y2_b), (0, 0, 255), 1)
                cv2.putText(rgb_rot, str(cam_right), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 0, 255), 1)
                cv2.line(rgb_show, (x1_b, y1_b), (x2_b, y2_b), (0, 0, 255), 6)
            # 整体右边线
            if cam_left != 0:
                w1_b, h1_b = CVFunc.points_rotate(-angle_set, cam_left, 100.0)
                w2_b, h2_b = CVFunc.points_rotate(-angle_set, cam_left, 2000.0)
                y1_b = CVFunc.calc_h2y(h1_b, model_F, model_W, model_a, model_b)
                y2_b = CVFunc.calc_h2y(h2_b, model_F, model_W, model_a, model_b)
                x1_b = CVFunc.calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
                x2_b = CVFunc.calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
                x_mid = int((x1_b + x2_b) / 2)
                y_mid = int((y1_b + y2_b) / 2)
                cv2.line(rgb_rot, (x1_b, y1_b), (x2_b, y2_b), (0, 0, 255), 1)
                cv2.putText(rgb_rot, str(cam_left), (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 0, 255), 1)
                cv2.line(rgb_show, (x1_b, y1_b), (x2_b, y2_b), (0, 0, 255), 6)
            # 后向左边线
            if cam_left_b != 0:
                w1_b, h1_b = CVFunc.points_rotate(-angle_set, -cam_left_b, 100.0)
                w2_b, h2_b = CVFunc.points_rotate(-angle_set, -cam_left_b, 2000.0)
                y1_b = CVFunc.calc_h2y(h1_b, model_F, model_W, model_a, model_b)
                y2_b = CVFunc.calc_h2y(h2_b, model_F, model_W, model_a, model_b)
                x1_b = CVFunc.calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
                x2_b = CVFunc.calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
                cv2.line(rgb_show, (x1_b, y1_b), (x2_b, y2_b), (0, 165, 255), 6)
            # 后向右边线
            if cam_right_b != 0:
                w1_b, h1_b = CVFunc.points_rotate(-angle_set, cam_right_b, 100.0)
                w2_b, h2_b = CVFunc.points_rotate(-angle_set, cam_right_b, 2000.0)
                y1_b = CVFunc.calc_h2y(h1_b, model_F, model_W, model_a, model_b)
                y2_b = CVFunc.calc_h2y(h2_b, model_F, model_W, model_a, model_b)
                x1_b = CVFunc.calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
                x2_b = CVFunc.calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
                cv2.line(rgb_show, (x1_b, y1_b), (x2_b, y2_b), (0, 165, 255), 6)
            # 左侧车宽
            w1_b, h1_b = CVFunc.points_rotate(-0.0, -300.0, 100.0)
            w2_b, h2_b = CVFunc.points_rotate(-0.0, -300.0, 2000.0)
            y1_b = CVFunc.calc_h2y(h1_b, model_F, model_W, model_a, model_b)
            y2_b = CVFunc.calc_h2y(h2_b, model_F, model_W, model_a, model_b)
            x1_b = CVFunc.calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
            x2_b = CVFunc.calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
            cv2.line(rgb_show, (x1_b, y1_b), (x2_b, y2_b), (0, 0, 0), 3)
            # 右侧车宽
            w1_b, h1_b = CVFunc.points_rotate(-0.0, 300.0, 100.0)
            w2_b, h2_b = CVFunc.points_rotate(-0.0, 300.0, 2000.0)
            y1_b = CVFunc.calc_h2y(h1_b, model_F, model_W, model_a, model_b)
            y2_b = CVFunc.calc_h2y(h2_b, model_F, model_W, model_a, model_b)
            x1_b = CVFunc.calc_w2x(w1_b, y1_b, model_F, model_W, model_a, model_b, principal_x)
            x2_b = CVFunc.calc_w2x(w2_b, y2_b, model_F, model_W, model_a, model_b, principal_x)
            cv2.line(rgb_show, (x1_b, y1_b), (x2_b, y2_b), (0, 0, 0), 3)

            rgb_c1 = cv2.resize(rgb_show, (960, 540))
        rgb_show_0 = rgb_c0
        rgb_show_1 = rgb_c1

        # 右上的距离展示区
        rgb_show_line = np.zeros((show_height, show_width, 3), np.uint8)
        if dis_ff[0] != 0:
            for i in range(0, len(dis_ff), 1):
                temp_y = int(half_height - (half_height * dis_ff[i] / 2000))
                cv2.line(rgb_show_line, (0, temp_y), (show_width, temp_y), (255, 0, 0), 1)
        if dis_fb[0] != 0:
            for i in range(0, len(dis_fb), 1):
                temp_y = int(half_height + (half_height * dis_fb[i] / 2000))
                cv2.line(rgb_show_line, (0, temp_y), (show_width, temp_y), (255, 0, 0), 1)
        if dis_lf[0] != 0:
            for i in range(0, len(dis_lf), 1):
                temp_x = int(half_width - (half_width * dis_lf[i] / 2000))
                cv2.line(rgb_show_line, (temp_x, 0), (temp_x, half_height), (255, 0, 0), 1)
        if dis_lb[0] != 0:
            for i in range(0, len(dis_lb), 1):
                temp_x = int(half_width - (half_width * dis_lb[i] / 2000))
                cv2.line(rgb_show_line, (temp_x, half_height), (temp_x, show_height), (255, 0, 0), 1)
        if dis_rf[0] != 0:
            for i in range(0, len(dis_rf), 1):
                temp_x = int(half_width + (half_width * dis_rf[i] / 2000))
                cv2.line(rgb_show_line, (temp_x, 0), (temp_x, half_height), (255, 0, 0), 1)
        if dis_rb[0] != 0:
            for i in range(0, len(dis_rb), 1):
                temp_x = int(half_width + (half_width * dis_rb[i] / 2000))
                cv2.line(rgb_show_line, (temp_x, half_height), (temp_x, show_height), (255, 0, 0), 1)
        if cam_left != 0:
            temp_x = int(half_width - (half_width * cam_left / 2000))
            cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (0, 0, 255), 1)
        if cam_right != 0:
            temp_x = int(half_width + (half_width * cam_right / 2000))
            cv2.line(rgb_show_line, (temp_x, 0), (temp_x, show_height), (0, 0, 255), 1)
        if cam_front != 0:
            temp_y = int(half_height - (half_height * cam_front / 2000))
            cv2.line(rgb_show_line, (0, temp_y), (show_width, temp_y), (0, 0, 255), 1)
        if cam_back != 0:
            temp_y = int(half_height + (half_height * cam_back / 2000))
            cv2.line(rgb_show_line, (0, temp_y), (show_width, temp_y), (0, 0, 255), 1)
        if cam_left_f != 0:
            temp_x = int(half_width - (half_width * cam_left_f / 2000))
            cv2.line(rgb_show_line, (temp_x, 0), (temp_x, half_height), (0, 165, 255), 1)
        if cam_left_b != 0:
            temp_x = int(half_width - (half_width * cam_left_b / 2000))
            cv2.line(rgb_show_line, (temp_x, half_height), (temp_x, show_height), (0, 165, 255), 1)
        if cam_right_f != 0:
            temp_x = int(half_width + (half_width * cam_right_f / 2000))
            cv2.line(rgb_show_line, (temp_x, 0), (temp_x, half_height), (0, 165, 255), 1)
        if cam_right_b != 0:
            temp_x = int(half_width + (half_width * cam_right_b / 2000))
            cv2.line(rgb_show_line, (temp_x, half_height), (temp_x, show_height), (0, 165, 255), 1)

        yaw_sin = math.sin(math.radians(abs(angle_set)))
        yaw_cos = math.cos(math.radians(abs(angle_set)))
        if -90 <= angle_set < 0:
            temp_x = int(half_width - yaw_sin * (half_height * 0.5))
            temp_y = int(half_height - yaw_cos * (half_height * 0.5))
        elif 0 <= angle_set <= 90:
            temp_x = int(half_width + yaw_sin * (half_height * 0.5))
            temp_y = int(half_height - yaw_cos * (half_height * 0.5))
        elif -180 <= angle_set < -90:
            temp_x = int(half_width - yaw_sin * (half_height * 0.5))
            temp_y = int(half_height - yaw_cos * (half_height * 0.5))
        elif 90 < angle_set <= 180:
            temp_x = int(half_width + yaw_sin * (half_height * 0.5))
            temp_y = int(half_height - yaw_cos * (half_height * 0.5))
        else:
            temp_x = half_width
            temp_y = half_height - (half_height * 0.5)
        cv2.line(rgb_show_line, (half_width, half_height), (temp_x, temp_y), (160, 0, 240), 3)

        cv2.line(rgb_show_line, (0, half_height), (show_width, half_height), (0, 255, 0), 1)
        cv2.line(rgb_show_line, (half_width, 0), (half_width, show_height), (0, 255, 0), 1)

        # 右下的数据展示区
        rgb_show_data = np.zeros((show_height, show_width, 3), np.uint8)
        str_0 = str_Time + '    ' + str(time_mess) + ' ms'
        str_1 = 'Yaw:' + str(angle_set) + '  Left:' + str(cam_left) + '  Right:' + str(cam_right)
        str_2 = 'L:' + str(cam_left_f) + '-' + str(cam_left_b) + '  R:' + str(cam_right_f) + '-' + str(cam_right_b)
        str_2 += '  F:' + str(cam_front) + '  B:' + str(cam_back)
        str_3 = ''
        if dis_ff[0] != 0:
            str_3 += 'nF:' + str(len(dis_ff))
        else:
            str_3 += 'nF:0'
        if dis_lf[0] != 0:
            str_3 += '  nLF:' + str(len(dis_lf))
        else:
            str_3 += '  nLF:0'
        if dis_rf[0] != 0:
            str_3 += '  nRF:' + str(len(dis_rf))
        else:
            str_3 += '  nRF:0'
        if dis_fb[0] != 0:
            str_3 += '  nB:' + str(len(dis_fb))
        else:
            str_3 += '  nB:0'
        if dis_lb[0] != 0:
            str_3 += '  nLB:' + str(len(dis_lb))
        else:
            str_3 += '  nLB:0'
        if dis_rb[0] != 0:
            str_3 += '  nRB:' + str(len(dis_rb))
        else:
            str_3 += '  nRB:0'
        cv2.putText(rgb_show_data, str_0, (0, int(show_height / 4)), cv2.FONT_HERSHEY_COMPLEX, 0.8, (255, 255, 255), 1)
        cv2.putText(rgb_show_data, str_1, (0, int(show_height / 2)), cv2.FONT_HERSHEY_COMPLEX, 0.8, (255, 255, 255), 1)
        cv2.putText(rgb_show_data, str_2, (0, int(show_height * 3 / 4)), cv2.FONT_HERSHEY_COMPLEX, 0.8, (255, 255, 255), 1)
        cv2.putText(rgb_show_data, str_3, (0, show_height), cv2.FONT_HERSHEY_COMPLEX, 0.8, (255, 255, 255), 1)

        # 4图拼接
        rgb_mix = np.zeros(((show_height * 2), (show_width * 2), 3), np.uint8)
        rgb_mix[0:show_height, 0:show_width] = rgb_show_0
        rgb_mix[0:show_height, show_width:(show_width * 2)] = rgb_show_line
        rgb_mix[show_height:(show_height * 2), 0:show_width] = rgb_show_1
        rgb_mix[show_height:(show_height * 2), show_width:(show_width * 2)] = rgb_show_data
        cv2.imshow('Show', rgb_mix)

        # 显示及保存图片
        # rgb_show = cv2.resize(rgb_show, (960, 540))
        # cv2.imshow('Cap' + str(cap_id), rgb_show)
        # cv2.imwrite(str_fileAddress + 'C' + str(cap_id) + '-' + str_Time + '.jpg', rgb_frame)
        cv2.imwrite(file_address + 'D' + str(cap_id) + '-' + str_Time + '.jpg', rgb_rot)

        # 保存txt，传输数据
        ret_mess += 'Tim:' + str_Time + ';Yaw:' + str(angle_set) + ';all:' + str(num_line)
        ret_mess += ';frt:' + str(num_front) + ';lft:' + str(num_left) + ';rgt:' + str(num_right)
        ret_mess += ';c_l:' + str(cam_left) + ';c_r:' + str(cam_right) + ';clf:' + str(cam_left_f)
        ret_mess += ';clb:' + str(cam_left_b) + ';crf:' + str(cam_right_f) + ';crb:' + str(cam_right_b) + ';'
        # if dis_f[0] != 0:
        #     dis_f.sort()
        #     for i in range(0, len(dis_f), 1):
        #         ret_mess += ';f_' + str(i) + ':' + str(dis_f[i])
        # if dis_l[0] != 0:
        #     dis_l.sort()
        #     for i in range(0, len(dis_l), 1):
        #         ret_mess += ';l_' + str(i) + ':' + str(dis_l[i])
        # if dis_r[0] != 0:
        #     dis_r.sort()
        #     for i in range(0, len(dis_r), 1):
        #         ret_mess += ';r_' + str(i) + ':' + str(dis_r[i])
        # ret_mess += ';\n'
        file_rec = open(file_address + str(cap_id) + '.txt', 'a')
        if len(ret_mess) > 0:
            file_rec.write('Tpy:Date;' + ret_mess)

        end_time = time.time()
        time_mess += 'Shw:' + str(round((end_time - start_time) * 1000, 4)) + ';'
        end_time_total = time.time()
        time_mess += 'All:' + str(round((end_time_total - start_time_total) * 1000, 4)) + ';\n'

        if len(time_mess) > 0:
            file_rec.write('Tpy:Timer;' + time_mess)
        file_rec.close()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
