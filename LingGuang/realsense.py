# -*- coding: UTF-8 -*-
import numpy as np
import pyrealsense2 as rs
import time
import math

# 设置
pipeline = rs.pipeline()  # 定义流程pipeline，创建一个管道
config = rs.config()  # 定义配置config

decimation = rs.decimation_filter()
spatial = rs.spatial_filter()
temporal = rs.temporal_filter(0.5, 5, 1)
hole_filling = rs.hole_filling_filter()

depth_to_disparity = rs.disparity_transform(True)
disparity_to_depth = rs.disparity_transform(False)

# 创建对齐对象与color流对齐
align_to = rs.stream.color  # align_to 是计划对齐深度帧的流类型
align = rs.align(align_to)  # rs.align 执行深度帧与其他帧的对齐

enable_imu = False
imu_pipeline = None


# 深度图滤波， 不启用的话，对检测造成一定的影响，不是很大
def depth_filter(frame):
    return frame
    # frame = decimation.process(frame)
    # frame = depth_to_disparity.process(frame)
    frame = spatial.process(frame)
    frame = temporal.process(frame)
    # frame = disparity_to_depth.process(frame)
    # frame = hole_filling.process(frame)
    return frame

#单纯获取帧
def get_frames():
    frames = pipeline.wait_for_frames()
    return frames

#帧对齐
def frame_align(frames):
    # PC端约4ms， nano约12ms
    start_time = time.time()
    aligned_frames = align.process(frames)  # 获取对齐帧，将深度框与颜色框对齐
    # print('aligned_images %.2f ms' % ((time.time() - start_time) * 1000))

    aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的的depth帧
    aligned_color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的的color帧

    # 获取相机参数
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）
    color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参

    # PC端  nano约108ms，不滤波会对检测造成一定的影响，不是很大
    start_time = time.time()
    aligned_depth_frame = depth_filter(aligned_depth_frame)
    # print('depth_filter %.2f ms' % ((time.time() - start_time) * 1000))

    # 将images转为numpy arrays
    img_color = np.asanyarray(aligned_color_frame.get_data())  # RGB图
    img_depth = np.asanyarray(aligned_depth_frame.get_data())  # 深度图（默认16位）

    h, w, _ = img_color.shape
    pc = rs.pointcloud()

    # PC端约16ms， nano约30ms
    start_time = time.time()
    # pc.map_to(img_color)
    points = pc.calculate(aligned_depth_frame)

    vtx = np.asanyarray(points.get_vertices(dims=3))
    vtx = np.reshape(vtx, (h, w, -1)) * 1000
    # vtx = vtx.astype(int)
    # print('vtx %.2f ms\n' % ((time.time() - start_time) * 1000))

    return color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame, vtx


# 获取对齐图像帧与相机参数
def get_aligned_images():
    # 等待获取图像帧，获取颜色和深度的框架集, 基本不耗时
    frames = pipeline.wait_for_frames()

    if enable_imu:
        imu_frames = imu_pipeline.wait_for_frames(200)
        accel_frame = imu_frames.first_or_default(rs.stream.accel, rs.format.motion_xyz32f)
        gyro_frame = imu_frames.first_or_default(rs.stream.gyro, rs.format.motion_xyz32f)

        accel = accel_frame.as_motion_frame().get_motion_data()
        # 重力加速度在X-Y面上的投影
        pitch_xy = math.sqrt(accel.x * accel.x + accel.y * accel.y)
        # 重力加速度在Z轴上的分量与在X-Y面上的投影的正切，即俯仰角
        pitch = math.atan2(-accel.z, pitch_xy) * 57.3  # 57.3 = 180/3.1415
        # 重力加速度在Z-Y面上的投影
        roll_zy = math.sqrt(accel.z * accel.z + accel.y * accel.y)
        # 重力加速度在X轴上的分量与在Z-Y面上的投影的正切，即翻滚角
        roll = math.atan2(-accel.x, roll_zy) * 57.3

        # 打印姿态角信息
        print("roll:%.3f, pitch:%.3f" % (roll, pitch))

    # PC端约4ms， nano约12ms
    start_time = time.time()
    aligned_frames = align.process(frames)  # 获取对齐帧，将深度框与颜色框对齐
    # print('aligned_images %.2f ms' % ((time.time() - start_time) * 1000))

    aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的的depth帧
    aligned_color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的的color帧

    # 获取相机参数
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）
    color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参

    # PC端  nano约108ms，不滤波会对检测造成一定的影响，不是很大
    start_time = time.time()
    aligned_depth_frame = depth_filter(aligned_depth_frame)
    # print('depth_filter %.2f ms' % ((time.time() - start_time) * 1000))

    # 将images转为numpy arrays
    img_color = np.asanyarray(aligned_color_frame.get_data())  # RGB图
    img_depth = np.asanyarray(aligned_depth_frame.get_data())  # 深度图（默认16位）

    h, w, _ = img_color.shape
    pc = rs.pointcloud()

    # PC端约16ms， nano约30ms
    start_time = time.time()
    # pc.map_to(img_color)
    points = pc.calculate(aligned_depth_frame)

    vtx = np.asanyarray(points.get_vertices(dims=3))
    vtx = np.reshape(vtx, (h, w, -1)) * 1000
    # vtx = vtx.astype(int)
    # print('vtx %.2f ms\n' % ((time.time() - start_time) * 1000))

    return color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame, vtx


def realsense_init(width, height, replay_file=None):
    if enable_imu:
        global imu_pipeline
        imu_pipeline = rs.pipeline()
        imu_config = rs.config()
        imu_config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f)  # acceleration
        imu_config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f)  # gyroscope
        imu_pipeline.start(imu_config)

    if replay_file != None:
        config.enable_device_from_file(replay_file, repeat_playback=False)
        device = config.resolve(pipeline).get_device()
        if not device:
            print("Cannot open playback device. Verify that the ROSBag input file is not corrupted")
            exit(-1)
        else:
            device.as_playback().set_real_time(False)  # Set to False to read each frame sequentially
    else:
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 15)  # 配置depth流
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 15)  # 配置color流
    pipeline.start(config)  # streaming流开始

    # 激光功耗控制，范围0-360 MW， 默认150， 室外情况，设置为250
    # if replay_file == None:
    #     sensor = pipeline.get_active_profile().get_device().query_sensors()[0]
    #     sensor.set_option(rs.pyrealsense2.option.laser_power,150)

    # 可以打印支持的选项
    # print(sensor.get_supported_options())
    # get_option_description = sensor.get_option_description(rs.option.exposure)
