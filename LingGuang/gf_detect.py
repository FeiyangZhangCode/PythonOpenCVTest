import cv2
import torch
import time

import platform
import numpy as np
import pycuda.driver as cuda

from gf_detect_common import Colors, letterbox, non_max_suppression, Annotator, process_mask, scale_boxes

colors = Colors()

gf_model = None
use_gpu = True

USE_TENSORTRT = False

plat = platform.system()
if plat == 'Windows':
    USE_TENSORTRT = False
else:
    USE_TENSORTRT = True

if USE_TENSORTRT:
    import tensorrt as trt
    import trt_common


    class TrtModel():
        def __init__(self, trt_path):
            cuda.init()
            self.cfx = cuda.Device(0).make_context()

            TRT_LOGGER = trt.Logger(trt.Logger.INFO)
            runtime = trt.Runtime(TRT_LOGGER)

            # Deserialize the engine from file
            with open(trt_path, "rb") as f:
                engine = runtime.deserialize_cuda_engine(f.read())

            self.context = engine.create_execution_context()
            self.inputs, self.outputs, self.bindings, self.stream = trt_common.allocate_buffers(engine)
            # print(self.inputs, self.outputs, self.bindings, self.stream)

        def __call__(self, img_np_nchw):
            self.inputs[0].host = np.ascontiguousarray(img_np_nchw.cpu())
            # self.inputs[0].host = img_np_nchw

            self.cfx.push()
            trt_outputs = trt_common.do_inference_v2(self.context, bindings=self.bindings, inputs=self.inputs,
                                                     outputs=self.outputs, stream=self.stream)
            self.cfx.pop()
            # 模型转换，导致输出反序
            return trt_outputs[::-1]

            # trt_output = [output.reshape(shape) for output, shape in zip(trt_outputs, [(1, 512, 512, 2)])]
            # return trt_output


    def gf_mod_init(m_path="model.trt"):
        global gf_model
        gf_model = TrtModel(m_path)
else:
    def gf_mod_init(m_path="gf.pkl"):
        global gf_model
        gf_model = torch.jit.load(m_path, "cuda")
        gf_model.eval()


# 将mask合并，方便直线判断边界及所属框
def masks_ids(masks):
    flags = [1 << id for id in range(len(masks))]
    flags = torch.tensor(flags, device="cuda", dtype=torch.float32)
    flags = flags[:, None, None]  # shape(n,1,1)

    masks_flag = masks * flags  # shape(n,h,w)
    masks_flag = masks_flag.sum(0).cpu().numpy()

    # cv2.imshow("mask", mcs)
    # cv2.waitKey(0)
    return masks_flag


def gf_detect(im0, score_thr, iou_thr, overlay_box, overlay_mask):
    with torch.no_grad():
        start_time = time.time()

        # 预处理
        im = letterbox(im0, [640, 640], stride=32, auto=False)[0]  # padded resize
        im = im.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        im = np.ascontiguousarray(im)  # contiguous

        # 转到gpu，并归一化
        im = torch.from_numpy(im).cuda().float()
        im /= 255  # 0 - 255 to 0.0 - 1.0
        im = im[None]  # expand for batch dim
        #print('pre pro= %.4f ms' % ((time.time() - start_time) * 1000))

        # 推理
        y = gf_model(im)
        #print('gf_model= %.4f ms' % ((time.time() - start_time) * 1000))

        pred, proto = y
        proto = proto.cuda()
        pred = pred.cuda()
        # print(pred.size())
        # print(proto.size())

        pred = pred.reshape(1, 25200, 38)
        proto = proto.reshape(1, 32, 160, 160)
        det = non_max_suppression(pred, conf_thres=score_thr, iou_thres=iou_thr, classes=None, agnostic=False,
                                  max_det=1000, nm=32)

        if len(det) <= 0 or det[0].shape[0] == 0:
            return None, None

        det = det[0]
        proto = proto[0]
        #print('non_max_suppression= %.4f ms' % ((time.time() - start_time) * 1000))

        # 掩码处理，PC端约1到2ms
        masks = process_mask(proto, det[:, 6:], det[:, :4], im.shape[2:], upsample=True)  # HWC
        det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()  # rescale boxes to im0 size

        # 将mask合并，方便直线判断边界及所属框
        masks_flag = masks_ids(masks)

        # 打印处理耗时
        #print('gf detect= %.4f ms' % ((time.time() - start_time) * 1000))

        # 不用叠加效果
        if overlay_box == False and overlay_mask == False:
            return det[:, :5], masks_flag
        else:
            # 显示
            annotator = Annotator(im0, line_width=2, example=str(["gf"]))

        if overlay_mask:
            annotator.masks(
                masks,
                colors=[colors(x, True) for x in range(len(det[:, 5]))],  # hgh
                im_gpu=im[0])

        # 框叠加
        if overlay_box:
            for j, (*xyxy, conf, cls) in enumerate(reversed(det[:, :6])):
                label = f'{conf:.2f}'
                annotator.box_label(xyxy, label, color=colors(j, True))  # hgh

        return det[:, :5], masks_flag


def scale_line_to_mask(lines, img0_shape, masks_flag):
    img1_shape = masks_flag.shape
    gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])  # gain  = old / new
    pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2  # wh padding

    lines = lines.astype(np.float)
    lines[..., :4] *= gain
    lines[..., [0, 2]] += pad[0]  # x padding
    lines[..., [1, 3]] += pad[1]  # y padding

    lines = lines.round().astype(np.int)

    lines[..., [0, 2]] = lines[..., [0, 2]].clip(0, img1_shape[1])  # x1, x2
    lines[..., [1, 3]] = lines[..., [1, 3]].clip(0, img1_shape[0])  # y1, y2

    # image = masks_flag.copy()
    # for l in lines:
    #     cv2.line(image, (l[0], l[1]), (l[2], l[3]), (0, 0, 0), 1, cv2.LINE_AA)
    # cv2.imshow('mask Line', image)

    return lines, pad[0], pad[1]


import os

if __name__ == '__main__':
    if USE_TENSORTRT:
        gf_mod_init("./model/gf.trt")
    else:
        gf_mod_init("./model/best.torchscript")

    path = r"./image"
    filenames = os.listdir(path)
    for filename in filenames:
        img_fn = path + "/" + filename
        img = cv2.imread(img_fn)
        gf_detect(img, 0.25, 0.45)
        print(filename)
    cv2.waitKey(0)
