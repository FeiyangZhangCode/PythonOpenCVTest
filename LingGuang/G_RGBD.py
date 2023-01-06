# -*- coding: UTF-8 -*-
# import argparse
import cv2
import time

import numpy
import numpy as np
import threading
import queue
from realsense import realsense_init, get_aligned_images, get_frames, frame_align
import pyrealsense2 as rs
from numba import jit

import math
import scipy.linalg as linalg
import platform
import collections

from gf_detect import gf_mod_init, gf_detect, scale_line_to_mask

show_depth = False  # 显示深度图
show_all_lines = False  # 显示所有检测到的直线
show_row_col_lines = False  # 显示用于计算偏航角的行列显
show_all_border_lines = False  # 显示所有的边线
show_object_box = False  # 显示检测框
show_object_mask = False  # 显示分割掩码


# 视频采集线程
def realsense_thread(queue_image, show_deth_image):
    frame_cnt = 0
    while True:
        start_time = time.time()
        if queue_image.qsize() >= 1:
            time.sleep(0.005)
            continue

        color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame, vtx = get_aligned_images()

        # 深度图转换及显示
        if show_deth_image:
            colorizer = rs.colorizer()
            depth_colormap = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
            cv2.imshow('depth', depth_colormap)
            cv2.waitKey(1)

        frame_cnt += 1
        if frame_cnt > 10:
            queue_image.put([img_color, img_depth, vtx])
        time.sleep(0.001)
        # print('realsense_thread cap_proc =%.2f\n' % ((time.time() - start_time) * 1000))


def realsense_threadf(queue_image, show_deth_image):
    frame_cnt = 0
    while True:
        start_time = time.time()
        frame = get_frames()

        if queue_image.qsize() >= 1:
            time.sleep(0.005)
            continue

        color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame, vtx = frame_align(frame)

        # 深度图转换及显示
        if show_deth_image:
            colorizer = rs.colorizer()
            depth_colormap = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
            cv2.imshow('depth', depth_colormap)
            cv2.waitKey(1)

        frame_cnt += 1
        if frame_cnt > 10:
            queue_image.put([img_color, img_depth, vtx])
        time.sleep(0.001)
        # print('realsense_thread cap_proc =%.2f\n' % ((time.time() - start_time) * 1000))


# 直线检测的线程
def line_detect_thread(q_in, q_out):
    frame_cnt = 0
    lsd = cv2.createLineSegmentDetector(0)
    start_time = time.time()
    while True:
        # 获取采集图像
        img = q_in.get()
        one_time = time.time()
        img_color, img_depth, vtx = img[0], img[1], img[2]
        frame_cnt += 1

        h, w, _ = vtx.shape
        # 直线检测
        if w == 1280:
            res_img = cv2.resize(img_color, (640, 480), interpolation=cv2.INTER_AREA)
            image_g = cv2.cvtColor(res_img, cv2.COLOR_BGR2GRAY)
            dlines = lsd.detect(image_g)
            lines = dlines[0][:]
            dlines = lines

            dlines[:, :, 0] = dlines[:, :, 0] * 2
            dlines[:, :, 1] = dlines[:, :, 1] * 1.5
            dlines[:, :, 2] = dlines[:, :, 2] * 2
            dlines[:, :, 3] = dlines[:, :, 3] * 1.5

        else:
            image_g = cv2.cvtColor(img_color, cv2.COLOR_BGR2GRAY)
            dlines = lsd.detect(image_g)
            dlines = dlines[0]

        # image_g = cv2.cvtColor(img_color, cv2.COLOR_BGR2GRAY)
        # dlines = lsd.detect(image_g)
        # dlines = dlines[0]

        if dlines is None:
            dlines = []

        # 帧率
        fps = frame_cnt / (time.time() - start_time)
        ms = (time.time() - one_time) * 1000
        print('line proc fsp= %.2f  one proc =%.2f\n' % (fps, ms))

        txt = format("fps=%.1f c=%.1f ms" % (fps, ms))
        cv2.putText(img_color, txt, (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # 结果放入队列
        q_out.put([img_color, img_depth, vtx, dlines])

        if frame_cnt > 100:
            frame_cnt = 0
            start_time = time.time()
        time.sleep(0.001)


# 从直线两侧获取靠近平面的点
@jit(nopython=True)
def get_plane_near_point(vtx, x, y, vx, vy, h_low, h_high, r):
    h, w, _ = vtx.shape

    mid = (h_low + h_high) / 2.0

    best_dis = 10000
    best_p = vtx[int(y)][int(x)]

    # 从2D坐标获取附近3D点
    for j in range(0, r, 1):
        tx = x + vx * j
        ty = y + vy * j

        tx = max(0, tx)
        ty = max(0, ty)
        tx = min(w - 1, tx)
        ty = min(h - 1, ty)

        tmp = vtx[int(ty)][int(tx)]
        if tmp[2] > 400 and tmp[1] >= h_low and tmp[1] <= h_high:
            dis = abs(tmp[1] - mid)
            if dis < best_dis:
                best_dis = dis
                best_p = tmp

        tx = x - vx * j
        ty = y - vy * j
        tx = max(0, tx)
        ty = max(0, ty)
        tx = min(w - 1, tx)
        ty = min(h - 1, ty)
        tmp = vtx[int(ty)][int(tx)]

        if tmp[2] > 400 and tmp[1] >= h_low and tmp[1] <= h_high:
            dis = abs(tmp[1] - mid)
            if dis < best_dis:
                best_dis = dis
                best_p = tmp
    return best_p


# 重新计算真实的3D坐标
@jit(nopython=True)
def recal_3d_line(x0, y0, x1, y1, xm, ym, xn, yn, PM, PN):
    # 中间两点的3D向量，M点作为起点
    vec3D = PN - PM

    # 中间2点2D向量，M点作为起点
    vecxn = xn - xm
    vecyn = yn - ym
    ln = (vecxn ** 2 + vecyn ** 2) ** 0.5

    # 端点0到M点向量，M点作为起点
    vecx0 = x0 - xm
    vecy0 = y0 - ym
    l0 = (vecx0 ** 2 + vecy0 ** 2) ** 0.5
    tmp0 = vecx0 * vecxn + vecy0 * vecyn
    if tmp0 > 0:
        P0 = PM + vec3D * l0 / ln
    else:
        P0 = PM - vec3D * l0 / ln

    # 端点0到M点向量，M点作为起点
    vecx1 = x1 - xm
    vecy1 = y1 - ym
    l1 = (vecx1 ** 2 + vecy1 ** 2) ** 0.5
    tmp1 = vecx1 * vecxn + vecy1 * vecyn
    if tmp1 > 0:
        P1 = PM + vec3D * l1 / ln
    else:
        P1 = PM - vec3D * l1 / ln

    return [P0, P1]


@jit(nopython=True)
def get_plane_line_points(vtx, x1, y1, x2, y2, YL, YH, Xthr, Zthr):
    # 直线向量及长度
    vecx = x2 - x1
    vecy = y2 - y1

    # 单位向量的分量，在垂线方向查找，所以xy交换
    lenth = (vecx ** 2 + vecy ** 2) ** 0.5
    vx = -vecy * 1.0 / lenth
    vy = vecx * 1.0 / lenth

    point_cnt = 4
    scale = 1.0 / point_cnt / 3

    ret = []
    cnt = 0
    xa = 0
    ya = 0
    xb = 0
    yb = 0

    r = 3
    for i in range(0, point_cnt, 1):
        x = int(x1 + scale * vecx * i)
        y = int(y1 + scale * vecy * i)

        # 从2D坐标获取3D点
        p = get_plane_near_point(vtx, x, y, vx, vy, YL, YH, r)
        if p[2] > 0 and p[2] < Zthr and p[1] > YL and p[1] < YH and abs(p[0]) < Xthr:
            cnt += 1
            xa = x
            ya = y
            P1 = p
            break

    if cnt == 0:
        return ret

    for i in range(0, point_cnt, 1):
        x = int(x2 - scale * vecx * i)
        y = int(y2 - scale * vecy * i)
        # 从2D坐标获取3D点
        p = get_plane_near_point(vtx, x, y, vx, vy, YL, YH, r)
        if p[2] > 0 and p[2] < Zthr and p[1] > YL and p[1] < YH and abs(p[0]) < Xthr:
            # ret.append(p)
            cnt += 1
            xb = x
            yb = y
            P2 = p
            break

    if cnt == 2:
        P1, P2 = recal_3d_line(x1, y1, x2, y2, xa, ya, xb, yb, P1, P2)
        ret.append(P1)
        ret.append(P2)
    return ret


# 计算直线对应的3D点
@jit(nopython=True)
def cal_line3D(vtx, dlines, YL, YH, Xthr, Zthr):
    ret = []
    for dline in dlines:
        x0, y0, x1, y1 = dline[0], dline[1], dline[2], dline[3]
        lp = get_plane_line_points(vtx, x0, y0, x1, y1, YL, YH, Xthr, Zthr)
        ret.append(lp)
    return ret


# 从两旁查找点的边界
@jit(nopython=True)
def cal_point_near_border(masks, padx, pady, x, y, vx, vy, r):
    h, w = masks.shape

    ret = [r, -1.0, -1.0]
    # 对比直线两边的mask值是否相同，并返回
    for j in range(1, r, 1):
        tx = x + vx * j
        ty = y + vy * j

        tx = max(padx, tx)
        ty = max(pady, ty)
        tx = min(w - padx - 1, tx)
        ty = min(h - pady - 1, ty)

        tmp1 = masks[int(ty)][int(tx)]

        tx = x - vx * j
        ty = y - vy * j
        tx = max(padx, tx)
        ty = max(pady, ty)
        tx = min(w - padx - 1, tx)
        ty = min(h - pady - 1, ty)
        tmp2 = masks[int(ty)][int(tx)]

        if tmp1 != tmp2:
            ret = [j, tmp1, tmp2]
            return ret
    return ret


@jit(nopython=True)
def cal_line_mask_dis(masks, padx, pady, x1, y1, x2, y2, dis_thr):
    # 直线向量及长度
    vecx = x2 - x1
    vecy = y2 - y1

    # 单位向量的分量，在垂线方向查找，所以xy交换
    lenth = (vecx ** 2 + vecy ** 2) ** 0.5
    vx = -vecy * 1.0 / lenth
    vy = vecx * 1.0 / lenth

    point_cnt = 8
    scale = 1.0 / point_cnt

    ret = []

    for i in range(0, point_cnt, 1):
        x = int(x1 + scale * vecx * i)
        y = int(y1 + scale * vecy * i)

        # 从2D坐标获取3D点
        b = cal_point_near_border(masks, padx, pady, x, y, vx, vy, dis_thr)
        ret.append(b)

    return ret


# 计算直线到mask的距离, 并排除光伏板内的直线
def cal_line_mask_border(lines_2d, l3d, masks, dis_thr, image_in, show_debug_img):
    ret_2d = []
    ret_3d = []
    start_time = time.time()
    # 直线到mask转换
    lines, padx, pady = scale_line_to_mask(lines_2d, image_in.shape, masks)

    # 对每一条计算边界及所属ID
    for i in range(len(lines)):
        lm = lines[i]
        x0, y0, x1, y1 = lm[0], lm[1], lm[2], lm[3]
        border = cal_line_mask_dis(masks, padx, pady, x0, y0, x1, y1, dis_thr)
        # print(border)
        border = numpy.array(border)
        dis = np.average(border[..., [0]], axis=0)

        # 边界
        if dis >= dis_thr * 0.75:
            continue

        # 统计ID
        id = border[..., [1, 2]]
        id = id.flatten().astype(int)
        count = collections.Counter(id)

        # 没有一个属于0的ID，意思是两个完全粘连在一起
        zero_cnt = count.get(0)
        if zero_cnt is None or zero_cnt <= 2:
            continue

        # 查找频率最高的两个ID值
        cnt = count.most_common(2)
        # print(cnt)

        # 边界必然会有2个ID，最多的ID为-1，表示大多数点找不到边框
        if len(cnt) <= 1 or cnt[0][0] == -1 or cnt[1][0] == -1:
            continue

        # 返回直线，包含到边界距离，2个主属ID
        line_src = lines_2d[i]
        x0, y0, x1, y1 = line_src[0], line_src[1], line_src[2], line_src[3]
        length = ((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0)) ** 0.5
        score = (dis_thr - dis[0]) * length
        # print(x0, y0, x1, y1,dis[0])
        if cnt[0][0] != 0:
            id = np.log2(cnt[0][0])
            dif = id-round(id)
            #主ID应该为2的整数次方
            if abs(dif)>0.01:
                continue
            ret_2d.append([x0, y0, x1, y1, score, cnt[0][0], cnt[1][0], length, dis[0]])
        else:
            id = np.log2(cnt[1][0])
            dif = id-round(id)
            # 主ID应该为2的整数次方
            if abs(dif)>0.01:
                continue
            ret_2d.append([x0, y0, x1, y1, score, cnt[1][0], cnt[0][0], length, dis[0]])
        ret_3d.append(l3d[i])

    print('cal_line_mask_border time= %.4f ms' % ((time.time() - start_time) * 1000))
    if show_debug_img:
        image_ml = image_in.copy()
        for l in ret_2d:
            cv2.line(image_ml, (l[0], l[1]), (l[2], l[3]), (0, 255, 0), 3, cv2.LINE_AA)
        cv2.imshow('image_ml', image_ml)
    return ret_2d, ret_3d


# 线段的长度过滤
def line_len_filter(width, height, dlines, thr):
    # 计算长度并过滤
    start = dlines[:, :, :2]
    end = dlines[:, :, 2:]
    dist = np.sqrt(np.sum((start - end) ** 2, axis=-1))
    select = np.where(dist > thr)
    ret = dlines[select]

    # 四舍五入
    ret = np.round(ret)

    # 宽度高度限制，并转换为整形
    ret = np.clip(ret, 0, width - 1)
    ret[:, 1] = np.clip(ret[:, 1], 0, height - 1)
    ret[:, 3] = np.clip(ret[:, 3], 0, height - 1)
    ret = ret.astype(int)

    return ret


# 直线过滤
def line_filter(image_in, dlines, vtx, install_height, len_thr, h_thr, Xthr, Zthr, show_debug):
    h, w, _ = vtx.shape
    l2d_ret = []
    l3d_ret = []

    start_time = time.time()

    # 按长度过滤
    dlines = line_len_filter(w, h, dlines, len_thr)

    YL = install_height - h_thr
    YH = install_height + h_thr

    l_3dps = cal_line3D(vtx, dlines, YL, YH, Xthr, Zthr)

    for i in range(len(l_3dps)):
        if len(l_3dps[i]) == 2:
            x0, y0, x1, y1 = dlines[i]
            P1, P2 = l_3dps[i]
            if P1[2] < P2[2]:
                l2d_ret.append([x0, y0, x1, y1])
                l3d_ret.append([P1, P2])
            else:
                l2d_ret.append([x1, y1, x0, y0])
                l3d_ret.append([P2, P1])

    #print('line_filter time= %.4f ms   line = %d' % ((time.time() - start_time) * 1000, len(l_3dps)))

    if show_debug:
        image_l = image_in.copy()
        for l in dlines:
            cv2.line(image_l, (l[0], l[1]), (l[2], l[3]), (255, 255, 0), 1, cv2.LINE_AA)

        for l in l2d_ret:
            cv2.line(image_l, (l[0], l[1]), (l[2], l[3]), (0, 255, 0), 1, cv2.LINE_AA)
        cv2.imshow('Lines', image_l)

    return [np.array(l2d_ret), np.array(l3d_ret)]


# 计算方向，主要是找出互相垂直的直线，并分组
def cal_derection(image_in, lines, show_debug):
    if show_debug:
        image = image_in.copy()

    if len(lines[0]) < 5:
        return None

    # 夹角阈值
    deg_thr = 85
    cos_deg_thr = np.cos(np.radians(deg_thr))

    start_time = time.time()

    # 分离2d和3d直线的向量
    lines_2d = lines[0]
    lines_3d = lines[1]
    lines3d_vec = [item[1] - item[0] for item in lines_3d]

    # 转为单位向量，简化计算, 下面的方式在nano板上会快
    # all_vec = [item / np.linalg.norm(item) for item in all_vec]
    vn = np.linalg.norm(lines3d_vec, axis=-1).reshape(-1, 1)
    vn = np.repeat(vn, 3, 1)
    lines3d_vec_n = lines3d_vec / vn

    # 计算与Y轴的夹角,删除夹角小于82度的
    cos_val = np.abs(np.dot(lines3d_vec_n, [0, 1, 0]))
    del_index = np.where(cos_val > np.cos(np.radians(82)))

    # 删除对应的线
    lines_2d = np.delete(lines_2d, del_index, axis=0)
    lines_3d = np.delete(lines_3d, del_index, axis=0)
    lines3d_vec_n = np.delete(lines3d_vec_n, del_index, axis=0)

    if len(lines_3d) < 5:
        return None

    # 计算两两之间的内积
    dot = np.dot(lines3d_vec_n, lines3d_vec_n.transpose())
    cos_val = np.abs(dot)
    # 符合条件的配对
    row, col = np.where(cos_val < cos_deg_thr)

    if len(row) < 16:
        return None

    cnt_list = np.bincount(row)
    sort_index = np.argsort(cnt_list)
    sort_index = sort_index[::-1]

    maxid = sort_index[0]

    #print('degree cal  time0= %.4f ms lcnt=%d' % ((time.time() - start_time) * 1000, len(lines3d_vec_n)))

    # 查找垂直于maxid，且出现次数最多的线
    cnt = len(sort_index)
    second_id = -1
    for i in range(1, cnt - 1, 1):
        id = sort_index[i]
        tmp = np.dot(lines3d_vec_n[maxid], lines3d_vec_n[id])
        if abs(tmp) < cos_deg_thr:
            second_id = id
            break

    # 计算与Z轴的夹角
    tmpF = np.dot(lines3d_vec_n[maxid], [0, 0, 1])
    tmpS = np.dot(lines3d_vec_n[second_id], [0, 0, 1])

    if abs(tmpF) < abs(tmpS):
        t = second_id
        second_id = maxid
        maxid = t

    # 与maxid平行的线
    cols1 = np.where(cos_val[maxid] > 0.995)
    cols2 = np.where(cos_val[second_id] < 0.087)
    cols = np.unique(np.append(cols1, cols2))

    # 与second_id 平行的线
    rows1 = np.where(cos_val[second_id] > 0.985)
    rows2 = np.where(cos_val[maxid] < 0.087)
    rows = np.unique(np.append(rows1, rows2))

    cline_2d = lines_2d[cols]
    cline_3d = lines_3d[cols]
    rline_2d = lines_2d[rows]
    rline_3d = lines_3d[rows]

    C3dN = lines3d_vec_n[cols]
    R3dN = lines3d_vec_n[rows]

    degr = np.dot(C3dN, [-1, 0, 0])
    degr = np.sort(degr)
    lc = len(degr)
    if lc > 5:
        cavg = np.average(degr[int(lc * 0.2):int(lc * 0.8)])
    cavg = np.average(degr)
    degree = np.arccos(cavg)
    YAW = 90 - np.degrees(degree)
    # print(degree)

    # degr = abs(np.dot(R3dN, [-1,0,0]))
    # cavg = np.average(degr)
    # degree = np.arccos(cavg)
    # degree = np.degrees(degree)
    # print(degree)
    #print('degree cal  time= %.4f ms YAW=%.2f lcnt=%d ' % ((time.time() - start_time) * 1000, YAW, len(lines3d_vec_n)))

    if show_debug:
        for idx in del_index[0]:
            l = lines[0][idx]
            cv2.line(image, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 1, cv2.LINE_AA)
        for l in cline_2d:
            cv2.line(image, (l[0], l[1]), (l[2], l[3]), (0, 255, 0), 1, cv2.LINE_AA)
        for l in rline_2d:
            # r = random.randint(0,255)
            # g = random.randint(0,255)
            # b = random.randint(0,255)
            cv2.line(image, (l[0], l[1]), (l[2], l[3]), (255, 0, 0), 1, cv2.LINE_AA)
            # cv2.line(image, (l[0], l[1]), (l[2], l[3]), (b,g,r), 2, cv2.LINE_AA)

        cv2.imshow('RCLine', image)

    return YAW, [cline_2d, cline_3d], [rline_2d, rline_3d]


# 通过轴及角度，计算旋转矩阵
def rotate_mat(axis, radian):
    rot_matrix = linalg.expm(np.cross(np.eye(3), axis / linalg.norm(axis) * radian))
    return rot_matrix


# 旋转3D直线
def rot_yaw_3D_line(lines, yaw):
    rot_matrix = rotate_mat([0, 1, 0], yaw * math.pi / 180)

    l2d = lines[0]
    l3d = lines[1]
    rl3d = []
    for ll in l3d:
        P1 = ll[0]
        P2 = ll[1]
        P1R = np.dot(rot_matrix, P1)
        P2R = np.dot(rot_matrix, P2)
        # print(P1,P1R)
        # print(P2, P2R)
        # print("\n")
        rl3d.append([P1, P2, P1R, P2R])
    return [l2d, np.array(rl3d)]


# 点到直线的距离
@jit(nopython=True)
def point_2_line_dis(p, a, b):
    # 计算用到的向量
    ab = b - a
    ap = p - a

    # 计算投影长度，并做正则化处理
    r = np.dot(ap, ab) / (np.linalg.norm(ab))
    dis = math.sqrt(math.fabs((np.linalg.norm(ap)) ** 2 - (r ** 2)))
    return dis


# 点到直线的距离
def point_2_line(point, line_point1, line_point2):
    # 计算向量
    vec1 = line_point1 - point
    vec2 = line_point2 - point
    distance = np.abs(np.cross(vec1, vec2)) / np.linalg.norm(line_point1 - line_point2)
    return distance


# 查找配对
def find_LR_pair(image, line_2d, line_3d, overlay):
    # 按左右分组
    lid = []
    rid = []

    # 提取近点X坐标,并排序
    cX = line_3d[:, 0:1, 0:1]
    cX = cX.flatten()

    # cX_sort = np.sort(cX)
    order = np.argsort(cX)
    l_2d_s = line_2d[order]
    l_3d_s = line_3d[order]

    # 左右分组
    for i in range(len(l_3d_s)):
        if l_3d_s[i][0][0] < 0:
            lid.append(i)
        else:
            rid.append(i)

    # 查找左边
    lcnt = len(lid)
    if lcnt <= 0:
        best_lid = -1
    else:
        # 反向排序
        lid.reverse()

        best_lid = lid[0]
        for i in range(lcnt - 1):
            id = lid[i + 1]

            # 当前分值与最优分值比
            ratio = l_2d_s[id][4] / l_2d_s[best_lid][4]

            # 当前分值较少，则继续
            if ratio <= 1:
                continue

            # 距离
            dis = abs(l_3d_s[best_lid][0][0] - l_3d_s[id][0][0])
            if dis > 35:
                continue

            # 属于同一个框，且分值较大
            if l_2d_s[best_lid][5] == l_2d_s[id][5]:
                best_lid = id
                continue

            # 分值两倍以上，距离20mm以内，可以不考虑ID
            if ratio > 2 and dis < 20:
                best_lid = id
                continue

    # 查找右边
    rcnt = len(rid)
    if rcnt <= 0:
        best_rid = -1
    else:
        # print("###")
        # print(l_2d_s[best_lid])
        best_rid = rid[0]
        for i in range(rcnt - 1):
            id = rid[i + 1]

            # 当前分值与最优分值比
            ratio = l_2d_s[id][4] / l_2d_s[best_rid][4]

            # 当前分值较少，则继续
            if ratio <= 1:
                continue

            # 距离
            dis = abs(l_3d_s[best_rid][0][0] - l_3d_s[id][0][0])
            if dis > 35:
                continue

            # 属于同一个框，且分值较大
            if l_2d_s[best_rid][5] == l_2d_s[id][5]:
                best_rid = id
                continue

            # 分值两倍以上，距离20mm以内，可以不考虑ID
            if ratio > 2 and dis < 20:
                best_rid = id
                continue

    disl, disr = -1, -1
    if best_lid != -1:
        pa1 = np.array(l_3d_s[best_lid][0])
        pa2 = np.array(l_3d_s[best_lid][1])
        pa1[1] = 0
        pa2[1] = 0
        disl = point_2_line_dis(np.array([0, 0, 0]), pa1, pa2)

    if best_rid != -1:
        pa1 = np.array(l_3d_s[best_rid][0])
        pa2 = np.array(l_3d_s[best_rid][1])
        pa1[1] = 0
        pa2[1] = 0
        disr = point_2_line_dis(np.array([0, 0, 0]), pa1, pa2)

    ret = [best_lid, disl, l_2d_s[best_lid][4], best_rid, disr, l_2d_s[best_rid][4]]

    if overlay:
        if best_lid != -1:
            l2d = l_2d_s[best_lid]
            cv2.line(image, (int(l2d[0]), int(l2d[1])), (int(l2d[2]), int(l2d[3])), (0, 0, 255), 3, cv2.LINE_AA)
            txt = format("disL=%.1f" % (disl))
            cv2.putText(image, txt, (0, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        if best_rid != -1:
            l2d = l_2d_s[best_rid]
            cv2.line(image, (int(l2d[0]), int(l2d[1])), (int(l2d[2]), int(l2d[3])), (255, 0, 0), 3, cv2.LINE_AA)
            txt = format("disR=%.1f" % (disr))
            cv2.putText(image, txt, (0, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        if disl != -1 and disr != -1:
            txt = format("W=%.1f" % (disl + disr))
            cv2.putText(image, txt, (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    return ret

@jit(nopython=True)
def find_border(border_flag):
    cnt = len(border_flag)
    ret = []
    for i in range(cnt-1):
        a = border_flag[i]
        b = border_flag[i+1]
        if a != b:
            id1 = 0 if a<=0 else np.log2(a)
            id2 = 0 if b<=0 else np.log2(b)

            dif1 = id1-round(id1)
            dif2 = id2-round(id2)
            # 主ID应该为2的整数次方
            if abs(dif1)>0.01 or abs(dif2)>0.01:
                continue
            ret.append([i,a,b])
    return ret


# 计算正前方的边界距离
def cal_front_border(image_in, msk, vtx,install_height, Ythr):
    # 掩码及深度图大小
    msk_shape = msk.shape
    vtx_shape = vtx.shape

    # 计算比例及填充大小
    gain = min(msk_shape[0] / vtx_shape[0], msk_shape[1] / vtx_shape[1])
    pad = int((msk_shape[1] - vtx_shape[1] * gain) / 2), int((msk_shape[0] - vtx_shape[0] * gain) / 2)

    #提取正前方的掩码
    xs = int(msk_shape[1] / 2)
    fmask = msk[pad[1]:msk_shape[0] - pad[1],xs:xs+1].flatten()
    #bf = fmask #<= 0
    #bf = np.log2(fmask)

    #提取深度图正前方的值
    #xs = int(vtx_shape[1] / 2)
    # fvtx = vtx[0:vtx_shape[0]:2,xs:xs+1].reshape(-1,3)
    # fvtx_y = fvtx[:,1:2].flatten()
    #
    # yabs = np.abs(fvtx_y-install_height)
    # yf = yabs>Ythr
    # border = yf & bf

    borders = find_border(fmask)

    borders = [int((i[0])/gain) for i in borders]
    #borders = [i[0]*2 for i in borders]

    xs = int(vtx_shape[1] / 2)
    ret = []
    for pos in borders:
        cv2.circle(image_in, (xs, pos), 5, (0,0,255), -1)
        P = get_plane_near_point(vtx, xs, pos, 0, 1, install_height-Ythr, install_height+Ythr, 2)
        txt = format("%.1fmm" % (P[2]))
        cv2.putText(image_in, txt, (xs, pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1)
        ret.append(P[2])

    #近到远排序
    ret.reverse()
    #print(ret)
    return ret


# 偏航角及左右直线检测线程
def yaw_and_line_detect_thread(queue_line, install_height, Lthr, Hthr, Xthr, Zthr, show_debug_img, q_v2a, q_v2i):
    frame_cnt = 0
    while True:
        # 获取对齐图像帧及直线检测结果
        data = queue_line.get()
        frame_cnt += 1
        print("frame_cnt=%d" % (frame_cnt))

        if cv2.waitKey(1) == ord(' '):  # or frame_cnt==1068:
            cv2.waitKey(0)

        # 获取图像，深度，直线检测结果
        img_color, img_depth, vtx, dlines = data[0], data[1], data[2], data[3]

        # 没有直线, 发送消息，循环继续
        if len(dlines) == 0:
            cv2.imshow('image', img_color)
            ret = None
            continue

        # 直线过滤
        lines_filter = line_filter(img_color, dlines, vtx, install_height, Lthr, Hthr, Xthr, Zthr, show_all_lines)

        # 偏航角计算，并分别找出水平及垂直
        ret = cal_derection(img_color, lines_filter, show_row_col_lines)

        # 没有直线, 发送消息，循环继续
        if ret is None:
            cv2.imshow('image', img_color)
            ret = None
            continue

        # 叠加偏航角
        yaw, cline, rline = ret
        txt = format("YAW=%.2f" % (yaw))
        cv2.putText(img_color, txt, (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # 根据偏航角，转换直线坐标，方便后面的计算
        cline_r = rot_yaw_3D_line(cline, yaw)
        rline_r = rot_yaw_3D_line(rline, yaw)

        # 提取旋转后的坐标
        l3d = cline_r[1]
        l3d = l3d[:, 2:4]

        # 光伏板检测框及掩码
        start_time = time.time()
        obj_box, masks_flag = gf_detect(img_color, 0.25, 0.45, show_object_box, show_object_mask)
        #print('gf_detecttime= %.4f ms' % ((time.time() - start_time) * 1000))

        # 没有检测框，不能确定左右边界
        if masks_flag is None:
            cv2.imshow('image', img_color)
            # 返回偏航角
            ret = [yaw, -1, 0, 0, -1, 0, 0]
            continue

        # 计算正前方的边界距离
        borders = cal_front_border(img_color,masks_flag, vtx,install_height, Hthr)


        # 计算直线到mask的距离
        l2d, l3d = cal_line_mask_border(cline_r[0], l3d, masks_flag, 12, img_color, show_all_border_lines)
        l2d = np.array(l2d)
        l3d = np.array(l3d)

        # 没有符合的左右直线
        if len(l2d) <= 0:
            cv2.imshow('image', img_color)
            # 返回偏航角
            ret = [yaw, -1, 0, 0, -1, 0, 0]
            continue

        # 查找配对
        ret = find_LR_pair(img_color, l2d, l3d, show_debug_img)
        ret.insert(int(0), yaw)

        send_list = [yaw]
        if ret[1] == -1:
            send_list.append(-999.9)
        else:
            send_list.append(ret[2])
        if ret[4] == -1:
            send_list.append(-999.9)
        else:
            send_list.append(ret[5])
        if len(borders) == 0:
            send_list.append(0)
        else:
            send_list.append(len(borders))
            send_list.append(borders)

        q_v2i.put(yaw)
        q_v2i.get() if q_v2i.qsize() > 1 else time.sleep(0.001)
        q_v2a.put(send_list)
        q_v2a.get() if q_v2a.qsize() > 1 else time.sleep(0.001)

        cv2.imshow('image', img_color)
        continue

        # print(ret)


# 初始化函数，创建采集线程、直线检测线程、 偏航角及距离计算 3个线程
def yaw_and_line_detect_init(install_height, Lthr, Hthr, Xthr, Zthr, show_debug_img, q_v2a, q_v2i):
    # 队列
    queue_image = queue.Queue(2)
    queue_line = queue.Queue(2)

    # 采集线程
    cap_thread = threading.Thread(name='real_cap_thread', target=realsense_thread, args=(queue_image, show_depth))
    cap_thread.setDaemon(True)  # 把子进程设置为守护线程，必须在start()之前设置
    cap_thread.start()

    # 直线检测线程
    line_thread = threading.Thread(name='line_alg_thread', target=line_detect_thread,
                                   args=(queue_image, queue_line))
    line_thread.setDaemon(True)  # 把子进程设置为守护线程，必须在start()之前设置
    line_thread.start()

    # 偏航角及左右直线检测线程
    line_thread = threading.Thread(name='yaw_and_line_detect_thread', target=yaw_and_line_detect_thread,
                                   args=(queue_line, install_height, Lthr, Hthr, Xthr, Zthr, show_debug_img, q_v2a, q_v2i))
    line_thread.setDaemon(True)  # 把子进程设置为守护线程，必须在start()之前设置
    line_thread.start()
    return


def distance_get(q_v2a, q_v2i):
    plat = platform.system()
    if plat != 'Windows':
        gf_mod_init("./model/gf.trt")
        realsense_init(640, 480, replay_file=None)
    else:
        gf_mod_init("./model/nei/best.torchscript")
        # 初始化realsense深度相机，传入分辨率参数， 支持实时数据及录像数据模式
        # realsense_init(1280, 720, replay_file=None)
        realsense_init(640, 480, replay_file=None)
        #realsense_init(1280, 720, replay_file="D:\\hgh\\data\\10.bag")
        # realsense_init(640, 480, replay_file="/mnt/usb/data/1.bag")
        # realsense_init(1280, 720, replay_file="E:\\hgh data\\6.bag")

    # 安装高度，毫米为单位
    install_height = 250
    Lthr = 40  # 直线长度的阈值，像素为单位

    Xthr = 1500  # 左右的阈值，毫米为单位
    Ythr = 40  # 平面高度的阈值，毫米为单位
    Zthr = 2300  # 前方的阈值，毫米为单位

    show_debug_img = True
    yaw_and_line_detect_init(install_height, Lthr, Ythr, Xthr, Zthr, show_debug_img, q_v2a, q_v2i)

    # 主循环
    while True:
        time.sleep(0.1)
        continue
