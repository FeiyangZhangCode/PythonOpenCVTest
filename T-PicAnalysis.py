import cv2
import numpy as np
import math

def readtest():
    RGB_Img = cv2.imread("part.jpg")
    Gra_Img = RGB_Img
    cv2.cvtColor(RGB_Img, cv2.COLOR_BGR2GRAY, Gra_Img)
    cv2.imshow("Test", Gra_Img)


def getDist_P2P(x1_d, y1_d, x2_d, y2_d):
    distance = math.pow((x1_d - x2_d), 2) + math.pow((y1_d - y2_d), 2)
    distance = math.sqrt(distance)
    return distance


if __name__ == '__main__':
    rgb_org = cv2.imread('./TestData/C0-112445-864513.jpg')

    # can_min = cv2.Canny(rgb_org, 40, 150)
    # cv2.imwrite('./TestData/Can-min.jpg', can_min)
    # can_max = cv2.Canny(rgb_org, 100, 400)
    # cv2.imwrite('./TestData/Can-max.jpg', can_max)
    # can_all = cv2.add(can_min, can_max)
    # cv2.imwrite('./TestData/Can-all.jpg', can_all)
    # for i in range(20, 120, 20):
    #     for j in range(50, 450, 50):
    #         gra_can = cv2.Canny(rgb_org, i, j)
    #         cv2.imwrite('./TestData/Can-' + str(i) + '-' + str(j) + '.jpg', gra_can)
            # cv2.imshow('test', gra_can)

    gra_can = cv2.Canny(rgb_org, 40, 100)
    rgb_hou = rgb_org.copy()

    gra_lines = np.zeros((gra_can.shape[0], gra_can.shape[1]), np.uint8)  # 创建个全0的黑背景
    lines = cv2.HoughLinesP(gra_can, rho=1.0, theta=np.pi / 180, threshold=30, minLineLength=30, maxLineGap=5)
    num_lines = 0
    for line in lines:
        for x1_p, y1_p, x2_p, y2_p in line:
            if getDist_P2P(x1_p, y1_p, x2_p, y2_p) > 30.0:
                num_lines += 1
                cv2.line(gra_lines, (x1_p, y1_p), (x2_p, y2_p), 255, 1)
                cv2.line(rgb_hou, (x1_p, y1_p), (x2_p, y2_p), (0, 0, 255), 1)

    print(len(lines), num_lines)

    cv2.imshow('test', rgb_hou)
    cv2.waitKey(0)

    cv2.destroyAllWindows()
