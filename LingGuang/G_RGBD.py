# -*- coding: UTF-8 -*-
# import argparse
import cv2
import time
import numpy as np
import threading
import queue

from realsense import realsense_init, get_aligned_images, get_frames, frame_align
import pyrealsense2 as rs
from numba import jit

#import random
#import sklearn.cluster as skc
import math
import scipy.linalg as linalg
import dbscan

# 视频采集线程
def realsense_threadm(queue_image, show_deth_image):
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


def realsense_thread(queue_image, show_deth_image):
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
        if w== 1280:
            res_img = cv2.resize(img_color, (640, 480),interpolation=cv2.INTER_AREA)
            image_g = cv2.cvtColor(res_img, cv2.COLOR_BGR2GRAY)
            dlines = lsd.detect(image_g)
            lines = dlines[0][:]
            dlines = lines

            dlines[:,:, 0] = dlines[:,:, 0]* 2
            dlines[:,:, 1] = dlines[:,:, 1] * 1.5
            dlines[:,:, 2] = dlines[:,:, 2] * 2
            dlines[:,:, 3] = dlines[:,:, 3] * 1.5

        else:
            image_g = cv2.cvtColor(img_color, cv2.COLOR_BGR2GRAY)
            dlines = lsd.detect(image_g)
            dlines = dlines[0]

        # image_g = cv2.cvtColor(img_color, cv2.COLOR_BGR2GRAY)
        # dlines = lsd.detect(image_g)
        # dlines = dlines[0]

        if dlines is None:
            dlines = []

        #帧率
        fps = frame_cnt / (time.time() - start_time)
        ms = (time.time() - one_time) * 1000
        # print('line proc fsp= %.2f  one proc =%.2f\n' % (fps, ms))

        txt = format("fps=%.1f c=%.1f ms" % (fps,ms))
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

    mid = (h_low+h_high)/2.0

    best_dis =10000
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
            dis = abs(tmp[1]-mid)
            if dis<best_dis:
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

#重新计算真实的3D坐标
@jit(nopython=True)
def recal_3d_line(x0,y0,x1,y1, xm,ym,xn,yn,PM,PN):
    #中间两点的3D向量，M点作为起点
    vec3D = PN-PM

    #中间2点2D向量，M点作为起点
    vecxn = xn-xm
    vecyn = yn-ym
    ln = (vecxn**2+vecyn**2)**0.5

    #端点0到M点向量，M点作为起点
    vecx0 = x0-xm
    vecy0 = y0 - ym
    l0 = (vecx0**2+vecy0**2)**0.5
    tmp0 = vecx0*vecxn+vecy0*vecyn
    if tmp0>0:
        P0 = PM+vec3D*l0/ln
    else:
        P0 = PM - vec3D * l0 / ln

    #端点0到M点向量，M点作为起点
    vecx1 = x1 - xm
    vecy1 = y1 - ym
    l1 = (vecx1**2+vecy1**2)**0.5
    tmp1 = vecx1*vecxn+vecy1*vecyn
    if tmp1>0:
        P1 = PM+vec3D*l1/ln
    else:
        P1 = PM - vec3D * l1 / ln

    return [P0,P1]

@jit(nopython=True)
def get_plane_line_points(vtx, x1, y1, x2, y2, YL, YH,Xthr,Zthr):
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

    r = 5
    for i in range(0, point_cnt, 1):
        x = int(x1 + scale * vecx * i)
        y = int(y1 + scale * vecy * i)

        # 从2D坐标获取3D点
        p = get_plane_near_point(vtx, x, y, vx, vy, YL, YH,r)
        if p[2] > 0 and p[2]<Zthr and p[1] > YL and p[1] < YH and abs(p[0])<Xthr:
            cnt+=1
            xa = x
            ya = y
            P1 = p
            break

    if cnt==0:
        return ret

    for i in range(0, point_cnt, 1):
        x = int(x2 - scale * vecx * i)
        y = int(y2 - scale * vecy * i)
        # 从2D坐标获取3D点
        p = get_plane_near_point(vtx, x, y, vx, vy, YL, YH,r)
        if p[2] > 0 and p[2]<Zthr and p[1] > YL and p[1] < YH and abs(p[0])<Xthr:
            #ret.append(p)
            cnt+=1
            xb = x
            yb = y
            P2 = p
            break

    if cnt==2:
        P1, P2 = recal_3d_line(x1, y1, x2, y2, xa, ya, xb, yb, P1, P2)
        ret.append(P1)
        ret.append(P2)
    return ret


#计算直线对应的3D点
@jit(nopython=True)
def cal_line3D(vtx, dlines, YL, YH,Xthr,Zthr):
    ret = []
    for dline in dlines:
        x0, y0, x1, y1 = dline[0], dline[1], dline[2], dline[3]
        lp = get_plane_line_points(vtx, x0, y0, x1, y1, YL, YH, Xthr, Zthr)
        ret.append(lp)
    return ret


#线段的长度过滤
def line_len_filter(width,height,dlines, thr):
    #计算长度并过滤
    start = dlines[:,:,:2]
    end = dlines[:,:, 2:]
    dist = np.sqrt(np.sum((start - end) ** 2, axis=-1))
    select = np.where(dist>thr)
    ret = dlines[select]

    #四舍五入
    ret = np.round(ret)

    #宽度高度限制，并转换为整形
    ret = np.clip(ret,0,width - 1)
    ret[:,1]= np.clip(ret[:,1],0,height-1)
    ret[:,3]= np.clip(ret[:,3],0,height-1)
    ret = ret.astype(int)

    return ret


#直线过滤
def line_filter(image_in, dlines, vtx, install_height, len_thr, h_thr, Xthr, Zthr, show_debug):
    h, w, _ = vtx.shape
    l2d_ret = []
    l3d_ret = []

    start_time = time.time()

    #按长度过滤
    dlines = line_len_filter(w,h,dlines, len_thr)

    YL = install_height - h_thr
    YH = install_height + h_thr

    l_3dps = cal_line3D(vtx, dlines, YL, YH, Xthr, Zthr)

    for i in range(len(l_3dps)):
        if len(l_3dps[i])==2:
            x0,y0,x1,y1 = dlines[i]
            P1,P2 = l_3dps[i]
            if P1[2] < P2[2]:
                l2d_ret.append([x0, y0, x1, y1])
                l3d_ret.append([P1, P2])
            else:
                l2d_ret.append([x1, y1, x0, y0])
                l3d_ret.append([P2, P1])

    # print('line_filter time= %.4f ms   line = %d' % ((time.time() - start_time) * 1000, len(l_3dps)))

    if show_debug:
        image_l = image_in.copy()
        for l in dlines:
            cv2.line(image_l, (l[0], l[1]), (l[2], l[3]), (255, 255, 0), 1, cv2.LINE_AA)

        for l in l2d_ret:
            cv2.line(image_l, (l[0], l[1]), (l[2], l[3]), (0, 255, 0), 1, cv2.LINE_AA)
        cv2.imshow('Lines', image_l)


    return [np.array(l2d_ret),np.array(l3d_ret)]

#计算方向，主要是找出互相垂直的直线，并分组
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

    #删除对应的线
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

    # print('degree cal  time0= %.4f ms lcnt=%d' % ((time.time() - start_time) * 1000,len(lines3d_vec_n)))

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

    #与maxid平行的线
    cols1 = np.where(cos_val[maxid] > 0.995)
    cols2 = np.where(cos_val[second_id] < 0.052)
    cols  = np.unique(np.append(cols1,cols2))

    #与second_id 平行的线
    rows1 = np.where(cos_val[second_id] > 0.985)
    rows2 = np.where(cos_val[maxid] < 0.087)
    rows = np.unique(np.append(rows1,rows2))

    cline_2d = lines_2d[cols]
    cline_3d = lines_3d[cols]
    rline_2d = lines_2d[rows]
    rline_3d = lines_3d[rows]

    C3dN = lines3d_vec_n[cols]
    R3dN = lines3d_vec_n[rows]

    degr = np.dot(C3dN, [-1, 0, 0])
    degr = np.sort(degr)
    lc = len(degr)
    if lc>5:
        cavg = np.average(degr[int(lc * 0.2):int(lc * 0.8)])
    cavg = np.average(degr)
    degree = np.arccos(cavg)
    YAW = 90 - np.degrees(degree)
    #print(degree)

    # degr = abs(np.dot(R3dN, [-1,0,0]))
    # cavg = np.average(degr)
    # degree = np.arccos(cavg)
    # degree = np.degrees(degree)
    # print(degree)
    # print('degree cal  time= %.4f ms YAW=%.2f lcnt=%d ' % ((time.time() - start_time) * 1000,YAW, len(lines3d_vec_n)))

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
            #cv2.line(image, (l[0], l[1]), (l[2], l[3]), (b,g,r), 2, cv2.LINE_AA)

        cv2.imshow('RCLine', image)

    return YAW, [cline_2d,cline_3d], [rline_2d,rline_3d]

# 通过轴及角度，计算旋转矩阵
def rotate_mat(axis, radian):
    rot_matrix = linalg.expm(np.cross(np.eye(3), axis / linalg.norm(axis) * radian))
    return rot_matrix

#旋转3D直线
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
        rl3d.append([P1,P2, P1R, P2R])
    return [l2d,np.array(rl3d)]


#点到直线的距离
@jit(nopython=True)
def point_2_line_dis(p, a, b):
    # 计算用到的向量
    ab = b - a
    ap = p - a

    # 计算投影长度，并做正则化处理
    r = np.dot(ap, ab) / (np.linalg.norm(ab))
    dis = math.sqrt(math.fabs((np.linalg.norm(ap)) ** 2 - (r ** 2)))
    return dis

#点到直线的距离
def point_2_line(point, line_point1, line_point2):
    # 计算向量
    vec1 = line_point1 - point
    vec2 = line_point2 - point
    distance = np.abs(np.cross(vec1, vec2)) / np.linalg.norm(line_point1 - line_point2)
    return distance

# 空间点到空间线段距离
def dis_point_to_seg_line(p, a, b):
    a, b, p = np.array(a), np.array(b), np.array(p)  # trans to np.array
    d = np.divide(b - a, np.linalg.norm(b - a))  # normalized tangent vector
    s = np.dot(a - p, d)  # signed parallel distance components
    t = np.dot(p - b, d)
    h = np.maximum.reduce([s, t, 0])  # clamped parallel distance
    c = np.cross(p - a, d)  # perpendicular distance component
    return np.hypot(h, np.linalg.norm(c))


# 空间点到空间线段距离
def point_to_line_seg3D_dis(p, a, b):
    # 计算用到的向量
    ab = b - a
    ap = p - a
    bp = p - b

    abl = np.linalg.norm(ab)
    apl = np.linalg.norm(ap)
    bpl = np.linalg.norm(bp)
    # 计算投影长度，并做正则化处理
    r = np.dot(ap, ab) / (apl * abl)
    # 分了三种情况
    if r > 0 and r < 1:
        dis = math.sqrt(abs(apl ** 2 - (r * apl) ** 2))
    elif r >= 1:
        dis = bpl
    else:
        dis = apl

    return dis

#计算二维直线b中的2点到直线a的距离
@jit(nopython=True)
def cal_line2d_dis(la,lb):
    ax1,ay1,ax2,ay2 = la
    bx1,by1,bx2,by2 = lb

    A = ay2 - ay1
    B = ax1-ax2
    C = ay1 * ax2 - ax1 * ay2

    sqr_AB = (A*A+B*B)**0.5

    d1 = (A*bx1+B*by1+C)/sqr_AB
    d2 = (A*bx2+B*by2+C)/sqr_AB

    return[d1,d2]


#排除交叉直线
@jit(nopython=True)
def filter_by_cross(xa,xb, cl_2d, rline_r,rl_2d):
    near_r = 0
    for j in range(len(rline_r)):
        r = rline_r[j]
        x1 = r[0][0]
        x2 = r[1][0]

        lth = abs(x1 - x2)

        dis_2d = cal_line2d_dis(cl_2d,rl_2d[j])

        #还需要细化, 3d直线有误差，还需要从2d的距离判断
        if lth > 300:
            # 判断是否相交
            d1 = xa - x1
            d2 = xa - x2
            d3 = xb - x1
            d4 = xb - x2

            abs_d1 = abs(d1)
            abs_d2 = abs(d2)
            abs_d3 = abs(d3)
            abs_d4 = abs(d4)

            lx = min(x1, x2)
            rx = max(x1, x2)

            #相交,而且交点到水平线的端点较远
            if d1 * d2 < 0 and abs_d1 > 70 and abs_d2 > 70 and abs_d3 > 70 and abs_d4 > 70 and abs(dis_2d[0])>15 and abs(dis_2d[1])>15:
                return [j,0]

            # if d1 * d2 < 0:
            #     if x < 0 and x - lx > 70:
            #         return [j, 0]
            #     elif x >= 0 and rx - x > 70:
            #         return [j, 0]

        maxx = max(x1, x2)
        minx = min(x1, x2)
        # maxl = max(abs_d1,abs_d2)

        # 左线 , 最左边的点较远， 且端点距离较近
        if xa < 0 and xa - minx > 100 and (abs(xa - maxx) < 20 or abs(dis_2d[0])<10 or abs(dis_2d[1])<10):
            near_r += 1

        # 右线，最右边的点较远，且端点距离较近
        if xa > 0 and maxx - xa > 100 and (abs(minx - xa) < 20 or abs(dis_2d[0])<10 or abs(dis_2d[1])<10):
            near_r += 1

        # #左线, 且两点在同一侧
        # if xa < 0 and xa - minx > 100:
        #     (dis_2d[0]*dis_2d[1])>=0:
        #      xa - minx > 100 and (abs(xa - maxx) < 20 or abs(dis_2d[0])<10 or abs(dis_2d[1])<10):
        #     near_r += 1

    return [-1, near_r]

#左右直线的过滤，主要通过排除交叉及相交情况
def LR_filter(image_in, rline_r, cline_r, show_debug):
    # 左右候选直线
    select_2d = []
    select_3d = []

    start_time = time.time()
    # 直线
    rl_2d = rline_r[0]
    rl_3d_4 = rline_r[1]
    rl_3d = rl_3d_4[:,2:4]

    cl_2d = cline_r[0]
    cl_3d_4 = cline_r[1]
    cl_3d = cl_3d_4[:, 2:4]

    c_line_type = []
    for i in range(len(cl_3d)):
        xa = cl_3d[i][0][0]
        xb = cl_3d[i][1][0]
        cross, near_r = filter_by_cross(xa,xb, cl_2d[i],rl_3d,rl_2d)
        if cross >= 0:
            c_line_type.append(1)
            continue
        if near_r >= 1:
            c_line_type.append(2)
            continue

        c_line_type.append(0)
        select_2d.append(cl_2d[i])
        select_3d.append(cl_3d[i])

    # print('LR_filter  time= %.4f ms' % ((time.time() - start_time) * 1000))
    if show_debug:
        image = image_in.copy()
        #水平线
        for l2d in rl_2d:
            cv2.line(image, (l2d[0], l2d[1]), (l2d[2], l2d[3]), (255, 0, 0), 1, cv2.LINE_AA)

        #垂直线
        for i in range(len(c_line_type)):
            t = c_line_type[i]
            l2d = cl_2d[i]
            if t==0:
                cv2.line(image, (l2d[0], l2d[1]), (l2d[2], l2d[3]), (0, 255, 0), 1, cv2.LINE_AA)
            elif t==1:
                cv2.line(image, (l2d[0], l2d[1]), (l2d[2], l2d[3]), (255, 0,255), 1, cv2.LINE_AA)
            else:
                cv2.line(image, (l2d[0], l2d[1]), (l2d[2], l2d[3]), (0, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow('RL_Line', image)
        #cv2.waitKey(1)
    return [np.array(select_2d),np.array(select_3d)]


#两点距离
@jit(nopython=True)
def cal_3Dpoint_distance(p1, p2):
    lenth = p2 - p1
    lenth = lenth * lenth
    dis = lenth.sum()
    dis = dis ** 0.5
    return dis


#直线合并，只是为了显示效果
def merge_line_group(lines_2d, lines_3d,bid):
    #初始化
    bl_2d =lines_2d[bid][:]
    bl_3d = lines_3d[bid][:]
    bl_x_pos = bl_3d[0][0]

    #合并及距离的直线
    for i in range(len(lines_3d)):
        l = lines_3d[i]
        l2d = lines_2d[i]
        x = l[0][0]
        if abs(x-bl_x_pos)>10:
            continue

        BN = bl_3d[0]
        BF = bl_3d[1]

        LN = l[0]
        LF = l[1]

        if LF[2]>BF[2]:
            if LN[2]<BN[2]:
                bl_2d = l2d
                bl_3d = l
            else:
                bl_2d = [bl_2d[0],bl_2d[1],l2d[2],l2d[3]]
                bl_3d = [bl_3d[0],l[1]]
        else:
            if LN[2]<BN[2]:
                bl_2d = [l2d[0],l2d[1],bl_2d[2],bl_2d[3]]
                bl_3d = [l[0],bl_3d[1]]

    return bl_2d,bl_3d


#从距离较近的一组直线中，挑选最合理的直线
@jit(nopython=True)
def select_best_line_from_group(line_g2d,line_g3d, rl_2d, rl_3d):
    #最大长度
    start = line_g3d[:,:1]
    end = line_g3d[:, 1:]
    dist = np.sqrt(np.sum((start - end) ** 2, axis=-1))
    maxl = np.argmax(dist)
    max_len = np.max(dist)

    best = 10000000.0
    best_id =maxl

    for i in range(len(line_g3d)):
        p1,p2 = line_g3d[i]
        l2d_a = line_g2d[i]
        sum_dis = 0
        cnt = 0
        for j in range(len(rl_3d)):
            PR1,PR2 = rl_3d[j]
            l2d_b = rl_2d[j]

            x1 = PR1[0]
            x2 = PR2[0]

            lx = min(x1, x2)
            rx = max(x1, x2)

            if (p1[0]<=0 and rx-p1[0]>70 ) or (p1[0] > 0 and p1[0]-lx > 70 ):
                dis_2d = cal_line2d_dis(l2d_a,l2d_b)
                min_dis_2d = min(abs(dis_2d[0]),abs(dis_2d[1]))
                sum_dis+=min_dis_2d
                cnt+=1
        if cnt>0:
            avg_dis = sum_dis/cnt

            if avg_dis<best and dist[i]*2>max_len:
                best = avg_dis
                best_id = i

    return [best_id,best,max_len*len(line_g3d)]


#按距离分组，dbscan算法
def line_group(image_in,cline_r,rline_r, g_thr,show_debug):
    start_time = time.time()

    # 直线
    cl_2d = cline_r[0]
    cl_3d = cline_r[1]

    rl_2d = rline_r[0]
    rl_3d_4 = rline_r[1]
    rl_3d = rl_3d_4[:,2:4]

    #提取近点X坐标
    cX = cl_3d[:,0:1,0:1]
    cX = cX.flatten()

    #排序
    # cX_sort = np.sort(cX)
    # order = np.argsort(cX)
    # cl_2d_sort = cl_2d[order]
    # cl_3d_sort = cl_3d[order]

    #按X分组
    #db = skc.DBSCAN(eps=g_thr, min_samples=1).fit(cX_sort)
    #labels = db.labels_
    labels = dbscan.MyDBSCAN(cX, g_thr, 1)
    labels = np.array(labels) - 1
    n_clusters = len(set(labels))

    #按分组加入列表
    line_g2d = []
    line_g3d = []
    for i in range(n_clusters):
        line_g2d.append([])
        line_g3d.append([])

    for i in range(len(cl_3d)):
        gid = labels[i]
        line_g2d[gid].append(cl_2d[i])
        line_g3d[gid].append(cl_3d[i])

    # 组内直线合并
    cl_2d_ret = []
    cl_3d_ret = []
    score = []
    mls = []
    for i in range(n_clusters):
        best_l = select_best_line_from_group(np.array(line_g2d[i]), np.array(line_g3d[i]), rl_2d, rl_3d)
        bid = int(best_l[0])
        cl_2d_ret.append(line_g2d[i][bid])
        cl_3d_ret.append(line_g3d[i][bid])
        score.append(best_l[2])

        # l2d = line_g2d[i][bid]
        # if show_debug:
        #      cv2.line(image, (l2d[0], l2d[1]), (l2d[2], l2d[3]), (0, 0, 255), 3, cv2.LINE_AA)

        # ml = merge_line_group(line_g2d[i], line_g3d[i],bid)
        # mls.append(ml)
        #
    # print('line_group time= %.4f ms   ' % ((time.time() - start_time) * 1000))

    if show_debug:
        image = image_in.copy()
        for l2d in cl_2d_ret:
            cv2.line(image, (l2d[0], l2d[1]), (l2d[2], l2d[3]), (0, 0, 255), 3, cv2.LINE_AA)

        for ml in mls:
            l2d = ml[0]
            cv2.line(image, (l2d[0], l2d[1]), (l2d[2], l2d[3]), (0, 255, 0), 3, cv2.LINE_AA)

        cv2.imshow('merge', image)
        #cv2.waitKey(1)

    return[cl_2d_ret,cl_3d_ret,score]

#查找配对
def find_best_pair(image_in, line_2d,line_3d,score,show_debug):
    #按左右分组
    lid = []
    rid =[]

    best_pair = [-1,-1]
    best_score = 0
    for i in range(len(line_3d)):
        if line_3d[i][0][0]<0:
            lid.append(i)
        else:
            rid.append(i)

    ret = [-1,0,0,-1,0,0]
    for i in lid:
        ll3d = line_3d[i][0]
        for j in rid:
            lr3d = line_3d[j][0]
            dis = lr3d[0]-ll3d[0]

            #距离过小或者距离过大
            if dis<550 or dis>1250:
                continue

            score_p = score[i]+score[j]
            if score_p>best_score:
                best_score = score_p
                best_pair=[i,j]

    if best_pair[0]!=-1  and best_pair[1]!=-1:
        l,r = best_pair
        pa1 = np.array(line_3d[l][0])
        pa2 = np.array(line_3d[l][1])
        pa1[1] = 0
        pa2[1] = 0
        disl = point_2_line_dis(np.array([0,0,0]), pa1,pa2)

        pa1 = np.array(line_3d[r][0])
        pa2 = np.array(line_3d[r][1])
        pa1[1] = 0
        pa2[1] = 0

        disr = point_2_line_dis(np.array([0,0,0]), pa1,pa2)

        ret = [l,disl,score[l],r,disr,score[r]]
    else:
        sort_l = np.argsort(-np.array(score))

        for id in sort_l:
            mid = id
            l = line_3d[mid]
            pa1 = np.array(l[0])
            pa2 = np.array(l[1])
            pa1[1] = 0
            pa2[1] = 0
            dis = point_2_line_dis(np.array([0,0,0]), pa1,pa2)
            if dis<1350:
                break

        if line_3d[mid][0][0]<0:
            ret = [mid,dis,score[mid],-1,0,0]
        else:
            ret = [-1,0,0,mid,dis,score[mid]]

    if show_debug:
        image = image_in.copy()
        if ret[0]!=-1:
            id = int(ret[0])
            l2d = line_2d[id]
            cv2.line(image, (l2d[0], l2d[1]), (l2d[2], l2d[3]), (0, 0, 255), 2, cv2.LINE_AA)
            txt = format("disL=%.1f" % (ret[1]))
            cv2.putText(image, txt, (0, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        if ret[3]!=-1:
            id = int(ret[3])
            l2d = line_2d[id]
            cv2.line(image, (l2d[0], l2d[1]), (l2d[2], l2d[3]), (255, 0, 0), 2, cv2.LINE_AA)

            txt = format("disR=%.1f" % (ret[4]))
            cv2.putText(image, txt, (0, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        if ret[0]!=-1 and ret[3]!=-1:
            txt = format("W=%.1f" % (ret[1]+ret[4]))
            cv2.putText(image, txt, (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.imshow('RL_Line_ret', image)
        cv2.waitKey(1)

    return ret


#偏航角及左右直线检测线程
def yaw_and_line_detect_thread(queue_line, install_height, Lthr, Hthr, Xthr, Zthr, show_debug_img, q_v2a, q_yi, q_v2s):
    frame_cnt=0
    while True:
        # 获取对齐图像帧及直线检测结果
        data = queue_line.get()
        frame_cnt+=1
        # print("frame_cnt=%d"%(frame_cnt))

        #获取图像，深度，直线检测结果
        img_color, img_depth, vtx, dlines = data[0], data[1], data[2], data[3]

        #没有直线
        if len(dlines)==0:
            continue

        #直线过滤
        lines_filter = line_filter(img_color, dlines, vtx, install_height,Lthr, Hthr, Xthr, Zthr ,show_debug_img)

        #偏航角计算，并分别找出水平及垂直
        ret = cal_derection(img_color, lines_filter, show_debug_img)

        if ret is not None:
            yaw, cline, rline = ret

            txt = format("YAW=%.2f" % (yaw))
            cv2.putText(img_color, txt, (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            #根据偏航角，转换直线坐标，方便后面的计算
            cline_r = rot_yaw_3D_line(cline, yaw)
            rline_r = rot_yaw_3D_line(rline, yaw)

            #左右直线过滤
            cline_rs = LR_filter(img_color, rline_r, cline_r, show_debug_img)

            #分组融合
            if len(cline_rs[0])>0:
                cl_2d,cl_3d,score = line_group(img_color,cline_rs,rline_r,40,show_debug_img)
                ret = find_best_pair(img_color, cl_2d, cl_3d, score, True)
                send_list = [yaw, ret[1], ret[4]]
                ret = [yaw,ret]
            else:
                ret = [yaw,-1,0,0,-1,0,0]
                send_list = [yaw, -999.9, -999.9]
            q_yi.put(yaw)
            q_yi.get() if q_yi.qsize() > 1 else time.sleep(0.001)
            q_v2a.put(send_list)
            q_v2a.get() if q_v2a.qsize() > 1 else time.sleep(0.001)
            q_v2s.put(send_list)
            q_v2s.get() if q_v2s.qsize() > 1 else time.sleep(0.001)
        else:
            ret = None

        #print(ret)


#初始化函数，创建采集线程、直线检测线程、 偏航角及距离计算 3个线程
def yaw_and_line_detect_init(install_height, Lthr, Hthr, Xthr, Zthr, show_debug_img, q_v2a, q_yi, q_v2s):
    #队列
    queue_image = queue.Queue(2)
    queue_line = queue.Queue(2)

    # 采集线程
    show_deth_image = show_debug_img
    cap_thread = threading.Thread(name='real_cap_thread', target=realsense_thread, args=(queue_image, show_deth_image))
    cap_thread.setDaemon(True)  # 把子进程设置为守护线程，必须在start()之前设置
    cap_thread.start()

    # 直线检测线程
    line_thread = threading.Thread(name='line_alg_thread', target=line_detect_thread,
                                   args=(queue_image, queue_line))
    line_thread.setDaemon(True)  # 把子进程设置为守护线程，必须在start()之前设置
    line_thread.start()

    #偏航角及左右直线检测线程
    line_thread = threading.Thread(name='yaw_and_line_detect_thread', target=yaw_and_line_detect_thread,
                                   args=(queue_line,install_height, Lthr, Hthr, Xthr, Zthr, show_debug_img, q_v2a, q_yi, q_v2s))
    line_thread.setDaemon(True)  # 把子进程设置为守护线程，必须在start()之前设置
    line_thread.start()
    return


# if __name__ == '__main__':
def distance_get(q_v2a, q_yi, q_v2s):
    # 初始化realsense深度相机，传入分辨率参数， 支持实时数据及录像数据模式
    # realsense_init(1280, 720, replay_file=None)
    realsense_init(640, 480, replay_file=None)
    #realsense_init(1280, 720, replay_file="D:\\hgh\\data\\8.bag")
    #realsense_init(640, 480, replay_file="/mnt/usb/data/1.bag")
    #realsense_init(1280, 720, replay_file="E:\\hgh data\\6.bag")

    # 安装高度，毫米为单位
    install_height = 260
    Lthr = 50  # 直线长度的阈值，像素为单位

    Xthr=1500   #左右的阈值，毫米为单位
    Ythr = 80   #平面高度的阈值，毫米为单位
    Zthr = 1300 #前方的阈值，毫米为单位

    show_debug_img = False
    yaw_and_line_detect_init(install_height, Lthr, Ythr, Xthr, Zthr, show_debug_img, q_v2a, q_yi, q_v2s)

    # 主循环
    while True:
        time.sleep(0.1)
        continue


