import cv2
import CVFunc
import numpy as np

min_threshold = 10
max_threshold = 100
img_org = cv2.imread('./TestData/30.jpg')
img_test = img_org.copy()
img_height = int(img_org.shape[0])
img_width = int(img_org.shape[1])
mid_height = int(img_height / 2)
mid_width = int(img_width / 2)

b, g, r = cv2.split(img_org)
for j in range(0, img_width, 1):
    for i in range(0, img_height, 1):
        b_value = b[i, j]
        g_value = g[i, j]
        r_value = r[i, j]
        if b_value < 110 and g_value < 110 and r_value > 130 and abs(float(b_value) - float(g_value)) < 30.0:
            continue
        else:
            img_test[i, j] = (0, 0, 0)

gra_test = cv2.cvtColor(img_test, cv2.COLOR_BGR2GRAY)
gra_test = CVFunc.func_noise_1(gra_test, 100)
gra_org = gra_test.copy()

def update_threshold(x):
    global min_threshold, max_threshold, gra_test, gra_org
    min_threshold = cv2.getTrackbarPos('min', 'test')
    max_threshold = cv2.getTrackbarPos('max', 'test')
    print(min_threshold, max_threshold)
    gra_test = cv2.Canny(gra_org, min_threshold, max_threshold)


cv2.namedWindow('test', cv2.WINDOW_NORMAL)
cv2.createTrackbar('min', 'test', 0, 200, update_threshold)
cv2.createTrackbar('max', 'test', 50, 400, update_threshold)

cv2.setTrackbarPos('min', 'test', 20)
cv2.setTrackbarPos('max', 'test', 100)

while True:
    cv2.imshow('test', gra_test)
    if cv2.waitKey(1) == ord('q'):
        break

# gra_lines = np.zeros((img_org.shape[0], img_org.shape[1]), np.uint8)  # 创建个全0的黑背景
# lines = cv2.HoughLinesP(gra_can_test, rho=1.0, theta=np.pi / 180, threshold=60, minLineLength=50, maxLineGap=20)
# for line in lines:
#     for x1, y1, x2, y2 in line:
#         cv2.line(gra_lines, (x1, y1), (x2, y2), 255, 1)


# cv2.imshow('test', gra_can_test)
# cv2.imshow('can', gra_can)

# cv2.waitKey(0)
cv2.destroyAllWindows()
