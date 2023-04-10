import cv2
import numpy as np


def nothing(x):
    pass

cv2.namedWindow("setting", cv2.WINDOW_AUTOSIZE); # 命名一個視窗
cv2.createTrackbar('H_low','setting',0,360,nothing)
cv2.createTrackbar('S_low','setting',0,255,nothing)
cv2.createTrackbar('V_low','setting',0,255,nothing)
cv2.createTrackbar('H_high','setting',360,360,nothing)
cv2.createTrackbar('S_high','setting',255,255,nothing)
cv2.createTrackbar('V_high','setting',255,255,nothing)

# 讀取圖片並轉HSV
# img = cv2.imread('C:/Users/weng/Downloads/f.jpg')
# hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

cap = cv2.VideoCapture(0)
cap.set(3,5000)
cap.set(4,5000)
cap.set(cv2.CAP_PROP_BRIGHTNESS,1)


while True:

    ret, img = cap.read()
    img = cv2.resize(img,(320,180))
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    # 讀取拉桿數值
    H_low = cv2.getTrackbarPos('H_low','setting')
    S_low = cv2.getTrackbarPos('S_low','setting')
    V_low = cv2.getTrackbarPos('V_low','setting')
    H_high = cv2.getTrackbarPos('H_high','setting')
    S_high = cv2.getTrackbarPos('S_high','setting')
    V_high = cv2.getTrackbarPos('V_high','setting')

    

    # 一號遮罩
    lower1 = np.array([H_low,S_low,V_low])
    upper1 = np.array([H_high,S_high,V_high])
    mask1 = cv2.inRange(hsv,lower1,upper1)

    # 輸出原圖&成果
    # cv2.namedWindow( "dog", cv2.WINDOW_NORMAL )
    # cv2.resizeWindow("dog", 800, 600)
    cv2.imshow("dog", img)

    # cv2.namedWindow( "dog_mask", cv2.WINDOW_NORMAL )
    # cv2.resizeWindow("dog_mask", 800, 600)
    cv2.imshow("dog_mask", mask1)
    # cv2.imwrite('C:/Users/weng/Downloads/mask.jpg',mask1)

    k=cv2.waitKey(1)
    if k==ord('q'):
        cv2.destroyAllWindows()
        break

# cv2.resize(img,(640,360))
# cv2.imshow('img',img)
# cv2.waitKey(0)