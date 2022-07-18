import cv2
import datetime

# 开始主程序
cap = cv2.VideoCapture(0)
cap.set(6, 1196444237)
cap.set(3, 1920)
cap.set(4, 1080)
cap.set(5, 30)
# 获取图像位置参数
img_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
img_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
mid_height = int(img_height / 2)
mid_width = int(img_width / 2)

while True:
    ret, frame = cap.read()

    if cv2.waitKey(1) & 0xFF == ord('c'):
        str_time = datetime.datetime.now().strftime('%H%M%S')
        str_address = './TestData/'
        cv2.imwrite(str_address + str_time + '.jpg', frame)
        print('Get', str_time)
    elif cv2.waitKey(1) & 0xFF == ord('q'):
        break

    cv2.line(frame, (0, mid_height), (img_width, mid_height), (0, 255, 0), 1)
    cv2.line(frame, (mid_width, 0), (mid_width, img_height), (0, 255, 0), 1)
    cv2.imshow('Cap', frame)

