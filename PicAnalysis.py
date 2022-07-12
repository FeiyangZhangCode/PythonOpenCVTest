import cv2
import numpy as np

def readtest():
    RGB_Img = cv2.imread("part.jpg")
    Gra_Img = RGB_Img
    cv2.cvtColor(RGB_Img, cv2.COLOR_BGR2GRAY, Gra_Img)
    cv2.imshow("Test",Gra_Img)

readtest()
cv2.destroyAllWindows()