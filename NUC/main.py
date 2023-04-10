
import cv2
import serial
import numpy as np
import time
from math import fabs, radians, floor, cos, sin
# from adafruit_rplidar import RPLidar, RPLidarException
from rplidar import RPLidar
import copy



'''
    序列埠設定
    開啟權限: sudo chmod 666 /dev/ttyACM0
    https://www.itdaan.com/tw/1b422ec424e1c3519f7cee4c2ad05274
'''
ser=serial.Serial('/dev/ttyACM0',9600,timeout=0.001)
# ser.flushInput()            # 清除輸入緩存區，放棄所有內容
# ser.flushOutput()           # 清除輸出緩衝區，放棄輸出
# ser.open()

"""
    光達設定
"""
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar('/dev/ttyUSB0')
max_distance = 320
lidar.connect()



'''
    相機設定
'''
cap = cv2.VideoCapture(0)
cap.set(3,5000)
cap.set(4,5000)
# cap.set(cv2.CAP_PROP_BRIGHTNESS,1)


'''
    變數設定
'''

# 左右線HSV遮色閥值
# L_H_low = 0
# L_S_low = 17
# L_V_low = 239
# L_H_high = 360
# L_S_high = 255
# L_V_high = 255
L_H_low = 16
L_S_low = 23
L_V_low = 239
L_H_high = 360
L_S_high = 255
L_V_high = 255

# R_H_low = 0
# R_S_low = 0
# R_V_low = 249
# R_H_high = 25
# R_S_high = 255
# R_V_high = 255

R_H_low = 0
R_S_low = 0
R_V_low = 236
R_H_high = 360
R_S_high = 23
R_V_high = 255

# 紅綠燈遮罩
G_Low = [35,90,90]
G_high = [85,255,255]


# 採樣間距
W_sampling_1 = 305
W_sampling_2 = 270
W_sampling_3 = 235
W_sampling_4 = 200


# 模式
mode = '0'    # 0=雙循線,1=左循線,2=右循線
old_mode = '0'

# 光達變數
lidar_error = 0
total_point = 0     # 距離小於max_distance的點數量
scan_data = [0]*360
# img = np.zeros((720,1280),dtype=np.uint8)






'''
    紅綠燈辨識
'''
def HoughCircles():

    # 回傳值(0=沒看到綠燈,1=有看到綠燈)
    look_green = 0

    # 讀取圖片並轉HSV
    ret, img = cap.read()
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    

    # 遮罩
    low_G = np.array(G_Low)
    up_G = np.array(G_high)
    mask_G = cv2.inRange(hsv,low_G,up_G)
    
    guess_img = cv2.GaussianBlur(mask_G,(25,25), 0)

    #param1的具體實現，用於邊緣檢測    
    canny = cv2.Canny(guess_img, 20, 45)

    kernel = np.ones((2,2),np.uint8)
    gradient = cv2.morphologyEx(canny, cv2.MORPH_GRADIENT, kernel)


    #霍夫變換圓檢測
    circles= cv2.HoughCircles(gradient,cv2.HOUGH_GRADIENT,2,200,param1=45,param2=85,minRadius=1,maxRadius=80)

    
    final_ing = img.copy()
    print('-------------我是條分割線-----------------')
    if (circles) is not None:
        look_green = 1
        #根據檢測到圓的信息，畫出每一個圓
        circles = np.uint16(np.around(circles))
        for circle in circles[0,:]:

            #坐標行列(就是圓心)
            x=int(circle[0])
            y=int(circle[1])
            #半徑
            r=int(circle[2])
            #在原圖用指定顏色圈出圓，參數設定為int所以圈畫存在誤差
            cv2.circle(final_ing,(x,y),r,(255,0,0),2)
            cv2.circle(final_ing,(x,y),5,(0,0,255),3)

        
    #顯示新圖像
    final_ing = cv2.resize(final_ing,(640,360))
    cv2.imshow('final',final_ing)

    #按任意鍵退出
    cv2.waitKey(1)

    return look_green



'''
    雙循線
'''
def double_line(img, kernel_size=25, low_threshold=10, high_threshold=20, close_size=5):

    # 左右線極限X值(需重置)
    R_min_300 = 640
    R_min_240 = 640
    R_min_180 = 640
    R_min_140 = 640

    L_min_300 = 0
    L_min_240 = 0
    L_min_180 = 0
    L_min_140 = 0

    # 影像預處理
    # img = copy(img)
    img = cv2.resize(img,(640,360))
    # img = cv2.GaussianBlur(img, (11, 11), 0)
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    # 右線遮罩
    lower_R = np.array([R_H_low,R_S_low,R_V_low])
    upper_R = np.array([R_H_high,R_S_high,R_V_high])
    mask_R = cv2.inRange(hsv,lower_R,upper_R)

    # 左線遮罩
    lower_L = np.array([L_H_low,L_S_low,L_V_low])
    upper_L = np.array([L_H_high,L_S_high,L_V_high])
    mask_L = cv2.inRange(hsv,lower_L,upper_L)



    # 右線運算
    # 右線運算 - Canny邊緣運算
    blur_gray = cv2.GaussianBlur(mask_R,(kernel_size, kernel_size), 0)
    canny_img = cv2.Canny(blur_gray, low_threshold, high_threshold)

    # 右線運算 - 閉運算(解緩Canny斷線問題)
    kernel = np.ones((close_size,close_size),np.uint8)
    gradient = cv2.morphologyEx(canny_img, cv2.MORPH_GRADIENT, kernel)

    # 右線運算 - 霍夫變換
    lines = cv2.HoughLinesP(gradient,1,np.pi/180,8,5,2)
    # print("error")
    if type(lines) == np.ndarray:
        for line in lines:
            x1,y1,x2,y2 = line[0]
            if ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_1:
                # cv2.line(img,(x1,y1),(x2,y2),(255,0,0),1)
                if ((x1+x2)/2)<R_min_300:
                    R_min_300 = int((x1+x2)/2)
            elif ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_2:
                # cv2.line(img,(x1,y1),(x2,y2),(0,255,0),1)
                if ((x1+x2)/2)<R_min_240:
                    R_min_240 = int((x1+x2)/2)
            # elif ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_3:
            #     # cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)
            #     if ((x1+x2)/2)<R_min_180:
            #         R_min_180 = int((x1+x2)/2)
            # elif ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_4:
            #     # cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)
            #     if ((x1+x2)/2)<R_min_140:
            #         R_min_140 = int((x1+x2)/2)
    else:
        print("error")
        pass
    


    # 左線運算
    # 左線運算 - Canny邊緣運算
    blur_gray = cv2.GaussianBlur(mask_L,(kernel_size, kernel_size), 0)
    canny_img = cv2.Canny(blur_gray, low_threshold, high_threshold)

    # 左線運算 - 閉運算(解緩Canny斷線問題)
    kernel = np.ones((close_size,close_size),np.uint8)
    gradient = cv2.morphologyEx(canny_img, cv2.MORPH_GRADIENT, kernel)

    # 左線運算 - 霍夫變換
    lines = cv2.HoughLinesP(gradient,1,np.pi/180,8,5,2)

    if type(lines) == np.ndarray:
        for line in lines:
            x1,y1,x2,y2 = line[0]
            if ((x1+x2)/2)<250 and ((y1+y2)/2)>W_sampling_1:
                # cv2.line(img,(x1,y1),(x2,y2),(255,0,0),1)
                if ((x1+x2)/2)>L_min_300:
                    L_min_300 = int((x1+x2)/2)
            elif ((x1+x2)/2)<250 and ((y1+y2)/2)>W_sampling_2:
                # cv2.line(img,(x1,y1),(x2,y2),(0,255,0),1)
                if ((x1+x2)/2)>L_min_240:
                    L_min_240 = int((x1+x2)/2)
            # elif ((x1+x2)/2)<250 and ((y1+y2)/2)>W_sampling_3:
            #     # cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)
            #     if ((x1+x2)/2)>L_min_180:
            #         L_min_180 = int((x1+x2)/2)
            # elif ((x1+x2)/2)<250 and ((y1+y2)/2)>W_sampling_4:
            #     # cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)
            #     if ((x1+x2)/2)>L_min_140:
            #         L_min_140 = int((x1+x2)/2)    
    else:
        pass




    # cv2.rectangle(img, (L_min_300, W_sampling_1), (R_min_300, 360), (255,0,0), 0) 
    # cv2.rectangle(img, (L_min_240, W_sampling_2), (R_min_240, W_sampling_1), (0,255,0), 0) 
    # cv2.rectangle(img, (L_min_180, W_sampling_3), (R_min_180, W_sampling_2), (0,0,255), 0) 
    # cv2.rectangle(img, (L_min_140, W_sampling_4), (R_min_140, W_sampling_3), (0,255,255), 0) 

    
    pts = np.array([[L_min_300,(360+W_sampling_1)/2], [L_min_240,(W_sampling_1+W_sampling_2)/2], [L_min_180,(W_sampling_2+W_sampling_3)/2],[L_min_140,(W_sampling_3+W_sampling_4)/2]], np.int32)
    pts = pts.reshape((-1, 1, 2))
    img = cv2.polylines(img, [pts], False,(255,200,0),3)

    pts = np.array([[R_min_300,(360+W_sampling_1)/2], [R_min_240,(W_sampling_1+W_sampling_2)/2], [R_min_180,(W_sampling_2+W_sampling_3)/2],[R_min_140,(W_sampling_3+W_sampling_4)/2]], np.int32)
    pts = pts.reshape((-1, 1, 2))
    img = cv2.polylines(img, [pts], False,(255,200,0),3)

    # 計算結果(車頭偏左負號)
    L_min = 320-((L_min_300+L_min_240)/4)
    R_min = ((R_min_300+R_min_240)/4)-320
    target_line = int(L_min-R_min)
    print(target_line)

    
    # 輸出原圖&成果
    cv2.imshow("img", img)
    cv2.imshow("mask_R", mask_R)
    cv2.imshow("mask_L", mask_L)
    cv2.waitKey(1)

    return target_line, img



'''
    右循線
'''
def right_line(img, kernel_size=25, low_threshold=10, high_threshold=20, close_size=5):

    # 左右線極限X值(需重置)
    R_min_300 = 640
    R_min_240 = 640
    R_min_180 = 640
    R_min_140 = 640

    # 影像預處理
    # img = copy(img)
    img = cv2.resize(img,(640,360))
    # img = cv2.GaussianBlur(img, (11, 11), 0)
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    # 右線遮罩
    lower_R = np.array([R_H_low,R_S_low,R_V_low])
    upper_R = np.array([R_H_high,R_S_high,R_V_high])
    mask_R = cv2.inRange(hsv,lower_R,upper_R)



    # 右線運算
    # 右線運算 - Canny邊緣運算
    blur_gray = cv2.GaussianBlur(mask_R,(kernel_size, kernel_size), 0)
    canny_img = cv2.Canny(blur_gray, low_threshold, high_threshold)

    # 右線運算 - 閉運算(解緩Canny斷線問題)
    kernel = np.ones((close_size,close_size),np.uint8)
    gradient = cv2.morphologyEx(canny_img, cv2.MORPH_GRADIENT, kernel)

    # 右線運算 - 霍夫變換
    lines = cv2.HoughLinesP(gradient,1,np.pi/180,8,5,2)
    # print("error")
    if type(lines) == np.ndarray:
        for line in lines:
            x1,y1,x2,y2 = line[0]
            if ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_1:
                # cv2.line(img,(x1,y1),(x2,y2),(255,0,0),1)
                if ((x1+x2)/2)<R_min_300:
                    R_min_300 = int((x1+x2)/2)
            elif ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_2:
                # cv2.line(img,(x1,y1),(x2,y2),(0,255,0),1)
                if ((x1+x2)/2)<R_min_240:
                    R_min_240 = int((x1+x2)/2)
            elif ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_3:
                # cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)
                if ((x1+x2)/2)<R_min_180:
                    R_min_180 = int((x1+x2)/2)
            elif ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_4:
                # cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)
                if ((x1+x2)/2)<R_min_140:
                    R_min_140 = int((x1+x2)/2)
    else:
        print("error")
        pass



    # cv2.rectangle(img, (L_min_300, W_sampling_1), (R_min_300, 360), (255,0,0), 0) 
    # cv2.rectangle(img, (L_min_240, W_sampling_2), (R_min_240, W_sampling_1), (0,255,0), 0) 
    # cv2.rectangle(img, (L_min_180, W_sampling_3), (R_min_180, W_sampling_2), (0,0,255), 0)
    # cv2.rectangle(img, (L_min_140, W_sampling_4), (R_min_140, W_sampling_3), (0,255,255), 0) 

    pts = np.array([[R_min_300,(360+W_sampling_1)/2], [R_min_240,(W_sampling_1+W_sampling_2)/2], [R_min_180,(W_sampling_2+W_sampling_3)/2],[R_min_140,(W_sampling_3+W_sampling_4)/2]], np.int32)
    pts = pts.reshape((-1, 1, 2))
    img = cv2.polylines(img, [pts], False,(255,200,0),3)

    # 計算結果(車頭偏左負號)
    R_min = ((R_min_300+R_min_240+R_min_180+R_min_140)/4)-320
    target_line = int(R_min-265)
    print(-target_line)

    
    # 輸出原圖&成果
    cv2.imshow("img", img)
    cv2.imshow("mask_R", mask_R)
    cv2.waitKey(1)

    return -target_line, img



'''
    左循線
'''
def left_line(img, kernel_size=25, low_threshold=10, high_threshold=20, close_size=5):

    # 左右線極限X值(需重置)
    L_min_300 = 0
    L_min_240 = 0
    L_min_180 = 0
    L_min_140 = 0

    # 影像預處理
    # img = copy(img)
    img = cv2.resize(img,(640,360))
    # img = cv2.GaussianBlur(img, (11, 11), 0)
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    # 左線遮罩
    lower_L = np.array([L_H_low,L_S_low,L_V_low])
    upper_L = np.array([L_H_high,L_S_high,L_V_high])
    mask_L = cv2.inRange(hsv,lower_L,upper_L)



    # 左線運算
    # 左線運算 - Canny邊緣運算
    blur_gray = cv2.GaussianBlur(mask_L,(kernel_size, kernel_size), 0)
    canny_img = cv2.Canny(blur_gray, low_threshold, high_threshold)

    # 左線運算 - 閉運算(解緩Canny斷線問題)
    kernel = np.ones((close_size,close_size),np.uint8)
    gradient = cv2.morphologyEx(canny_img, cv2.MORPH_GRADIENT, kernel)

    # 左線運算 - 霍夫變換
    lines = cv2.HoughLinesP(gradient,1,np.pi/180,8,5,2)

    if type(lines) == np.ndarray:
        for line in lines:
            x1,y1,x2,y2 = line[0]
            if ((x1+x2)/2)<250 and ((y1+y2)/2)>W_sampling_1:
                # cv2.line(img,(x1,y1),(x2,y2),(255,0,0),1)
                if ((x1+x2)/2)>L_min_300:
                    L_min_300 = int((x1+x2)/2)
            elif ((x1+x2)/2)<250 and ((y1+y2)/2)>W_sampling_2:
                # cv2.line(img,(x1,y1),(x2,y2),(0,255,0),1)
                if ((x1+x2)/2)>L_min_240:
                    L_min_240 = int((x1+x2)/2)
            elif ((x1+x2)/2)<250 and ((y1+y2)/2)>W_sampling_3:
                # cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)
                if ((x1+x2)/2)>L_min_180:
                    L_min_180 = int((x1+x2)/2)
            elif ((x1+x2)/2)<250 and ((y1+y2)/2)>W_sampling_4:
                # cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)
                if ((x1+x2)/2)>L_min_140:
                    L_min_140 = int((x1+x2)/2)    
    else:
        pass




    # cv2.rectangle(img, (L_min_300, W_sampling_1), (R_min_300, 360), (255,0,0), 0) 
    # cv2.rectangle(img, (L_min_240, W_sampling_2), (R_min_240, W_sampling_1), (0,255,0), 0) 
    # cv2.rectangle(img, (L_min_180, W_sampling_3), (R_min_180, W_sampling_2), (0,0,255), 0) 
    # cv2.rectangle(img, (L_min_140, W_sampling_4), (R_min_140, W_sampling_3), (0,255,255), 0) 

    
    pts = np.array([[L_min_300,(360+W_sampling_1)/2], [L_min_240,(W_sampling_1+W_sampling_2)/2], [L_min_180,(W_sampling_2+W_sampling_3)/2],[L_min_140,(W_sampling_3+W_sampling_4)/2]], np.int32)
    pts = pts.reshape((-1, 1, 2))
    img = cv2.polylines(img, [pts], False,(255,200,0),3)


    # 計算結果(車頭偏左負號)
    L_min = 320-((L_min_300+L_min_240+L_min_180+L_min_140)/4)
    target_line = int(L_min-220)
    print(target_line)

    
    # 輸出原圖&成果
    cv2.imshow("img", img)
    cv2.imshow("mask_L", mask_L)
    cv2.waitKey(1)

    return target_line, img

'''
    雙黃循線
'''
def double_yellow_line(img, kernel_size=25, low_threshold=10, high_threshold=20, close_size=5):
    
    # 左右線極限X值(需重置)
    R_min_300 = 640
    R_min_240 = 640
    R_min_180 = 640
    R_min_140 = 640

    L_min_300 = 0
    L_min_240 = 0
    L_min_180 = 0
    L_min_140 = 0

    # 影像預處理
    # img = copy(img)
    img = cv2.resize(img,(640,360))
    # img = cv2.GaussianBlur(img, (11, 11), 0)
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    # 右線遮罩
    lower_R = np.array([L_H_low,L_S_low,L_V_low])
    upper_R = np.array([L_H_high,L_S_high,L_V_high])
    mask_R = cv2.inRange(hsv,lower_R,upper_R)

    # 左線遮罩
    lower_L = np.array([L_H_low,L_S_low,L_V_low])
    upper_L = np.array([L_H_high,L_S_high,L_V_high])
    mask_L = cv2.inRange(hsv,lower_L,upper_L)



    # 右線運算
    # 右線運算 - Canny邊緣運算
    blur_gray = cv2.GaussianBlur(mask_R,(kernel_size, kernel_size), 0)
    canny_img = cv2.Canny(blur_gray, low_threshold, high_threshold)

    # 右線運算 - 閉運算(解緩Canny斷線問題)
    kernel = np.ones((close_size,close_size),np.uint8)
    gradient = cv2.morphologyEx(canny_img, cv2.MORPH_GRADIENT, kernel)

    # 右線運算 - 霍夫變換
    lines = cv2.HoughLinesP(gradient,1,np.pi/180,8,5,2)
    # print("error")
    if type(lines) == np.ndarray:
        for line in lines:
            x1,y1,x2,y2 = line[0]
            if ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_1:
                # cv2.line(img,(x1,y1),(x2,y2),(255,0,0),1)
                if ((x1+x2)/2)<R_min_300:
                    R_min_300 = int((x1+x2)/2)
            elif ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_2:
                # cv2.line(img,(x1,y1),(x2,y2),(0,255,0),1)
                if ((x1+x2)/2)<R_min_240:
                    R_min_240 = int((x1+x2)/2)
            elif ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_3:
                # cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)
                if ((x1+x2)/2)<R_min_180:
                    R_min_180 = int((x1+x2)/2)
            elif ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_4:
                # cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)
                if ((x1+x2)/2)<R_min_140:
                    R_min_140 = int((x1+x2)/2)
    else:
        print("error")
        pass
    


    # 左線運算
    # 左線運算 - Canny邊緣運算
    blur_gray = cv2.GaussianBlur(mask_L,(kernel_size, kernel_size), 0)
    canny_img = cv2.Canny(blur_gray, low_threshold, high_threshold)

    # 左線運算 - 閉運算(解緩Canny斷線問題)
    kernel = np.ones((close_size,close_size),np.uint8)
    gradient = cv2.morphologyEx(canny_img, cv2.MORPH_GRADIENT, kernel)

    # 左線運算 - 霍夫變換
    lines = cv2.HoughLinesP(gradient,1,np.pi/180,8,5,2)

    if type(lines) == np.ndarray:
        for line in lines:
            x1,y1,x2,y2 = line[0]
            if ((x1+x2)/2)<250 and ((y1+y2)/2)>W_sampling_1:
                # cv2.line(img,(x1,y1),(x2,y2),(255,0,0),1)
                if ((x1+x2)/2)>L_min_300:
                    L_min_300 = int((x1+x2)/2)
            elif ((x1+x2)/2)<250 and ((y1+y2)/2)>W_sampling_2:
                # cv2.line(img,(x1,y1),(x2,y2),(0,255,0),1)
                if ((x1+x2)/2)>L_min_240:
                    L_min_240 = int((x1+x2)/2)
            elif ((x1+x2)/2)<250 and ((y1+y2)/2)>W_sampling_3:
                # cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)
                if ((x1+x2)/2)>L_min_180:
                    L_min_180 = int((x1+x2)/2)
            elif ((x1+x2)/2)<250 and ((y1+y2)/2)>W_sampling_4:
                # cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)
                if ((x1+x2)/2)>L_min_140:
                    L_min_140 = int((x1+x2)/2)    
    else:
        pass


    
    

    # cv2.rectangle(img, (L_min_300, W_sampling_1), (R_min_300, 360), (255,0,0), 0) 
    # cv2.rectangle(img, (L_min_240, W_sampling_2), (R_min_240, W_sampling_1), (0,255,0), 0) 
    # cv2.rectangle(img, (L_min_180, W_sampling_3), (R_min_180, W_sampling_2), (0,0,255), 0) 
    # cv2.rectangle(img, (L_min_140, W_sampling_4), (R_min_140, W_sampling_3), (0,255,255), 0) 

    
    pts = np.array([[L_min_300,(360+W_sampling_1)/2], [L_min_240,(W_sampling_1+W_sampling_2)/2], [L_min_180,(W_sampling_2+W_sampling_3)/2],[L_min_140,(W_sampling_3+W_sampling_4)/2]], np.int32)
    pts = pts.reshape((-1, 1, 2))
    img = cv2.polylines(img, [pts], False,(255,200,0),3)

    pts = np.array([[R_min_300,(360+W_sampling_1)/2], [R_min_240,(W_sampling_1+W_sampling_2)/2], [R_min_180,(W_sampling_2+W_sampling_3)/2],[R_min_140,(W_sampling_3+W_sampling_4)/2]], np.int32)
    pts = pts.reshape((-1, 1, 2))
    img = cv2.polylines(img, [pts], False,(255,200,0),3)

    # 計算結果(車頭偏左負號)
    L_min = 320-((L_min_300+L_min_240+L_min_180+L_min_140)/4)
    R_min = ((R_min_300+R_min_240+R_min_180+R_min_140)/4)-320
    target_line = int(L_min-R_min)

    print("right= ",R_min_140+R_min_180+R_min_240+R_min_300,", left= ",L_min_140+L_min_180+L_min_240+L_min_300)
    print("target_line= ",target_line)




    if ((R_min_140+R_min_180+R_min_240+R_min_300)>=2200) and ((L_min_140+L_min_180+L_min_240+L_min_300)<=200):
        print('parking')
        target_line=-777
    else:
        print("雙黃線還沒分岔")
    

    
    # 輸出原圖&成果
    cv2.imshow("img", img)
    cv2.imshow("mask_R", mask_R)
    cv2.imshow("mask_L", mask_L)
    cv2.waitKey(1)

    return target_line, img


'''
    避障
'''
def Avoidance(img, scan, kernel_size=25, low_threshold=10, high_threshold=20, close_size=5):


    # 雙循線
    target_line, img = double_line(img, kernel_size=kernel_size, low_threshold=low_threshold, high_threshold=high_threshold, close_size=close_size)

    # 光達
    total_point = 0
    lidar_error = 0
    for (_, angle, distance) in scan:
        # 距離小於max_distance才加入參考
        if distance<max_distance:
            total_point += 1
            if angle>0 and angle<80:
                lidar_error += (0.004*(max_distance-distance))+(0.03*(80-angle))
                angle = radians(angle-90)
                scan_data[min([359, floor(angle)])] = distance
                cv2.circle(img, (int(cos(angle)*distance*0.5+320),int(sin(angle)*distance*0.5+160)), 1, (255,255,255), 4)
            if angle<360 and angle>270:
                lidar_error -= (0.004*(max_distance-distance))+(0.03*(angle-270))
                angle = radians(angle-90)
                scan_data[min([359, floor(angle)])] = distance
                cv2.circle(img, (int(cos(angle)*distance*0.5+320),int(sin(angle)*distance*0.5+160)), 1, (255,255,255), 4)
        
            

    # if total_point>40 and fabs(lidar_error)<120:
    #     if target_line>0:
    #         target_line = target_line*1.5
    #         print('lidar_error+target_line= ', target_line, ', lidar_error= ',0)
    #         # time.sleep(0.1)
    #     else:
    #         target_line = target_line*1.5
    #         # time.sleep(0.1)
    # else:
    target_line=int(lidar_error)+target_line
    print('lidar_error+target_line= ', target_line, ', lidar_error= ',lidar_error)

    
    # 輸出原圖&成果
    # cv2.imshow("img", img)

    return target_line, img


def parking(img, scan):

    img = cv2.resize(img,(640,360))

    # 光達
    total_point = 0
    lidar_error = 0
    for (_, angle, distance) in scan:
        # 距離小於max_distance才加入參考
        if distance<500:
            total_point += 1
            if angle>10 and angle<45:
                lidar_error += 1
                cv2.circle(img, (int(cos(angle)*distance*0.5+320),int(sin(angle)*distance*0.5+160)), 5, (255,255,255), 4)
            if angle<350 and angle>310:
                lidar_error -= 1
                cv2.circle(img, (int(cos(angle)*distance*0.5+320),int(sin(angle)*distance*0.5+160)), 5, (255,255,255), 4)
        

    target_line=int(lidar_error)
    print('lidar_error+target_line= ', target_line, ', lidar_error= ',lidar_error)

    
    # 輸出原圖&成果
    cv2.imshow("img", img)
    cv2.waitKey(1)

    return target_line, img




'''
    主程式
'''


lidar.stop()
lidar.disconnect()


while True:
    # data = -9876
    # t = time.time()

    # 讀取圖片並轉HSV
    ret, img = cap.read()

    # mode = ser.read().decode("utf-8")
    mode = '0'
    print("=======mode= ", mode)
    
    
    
    if mode=='0':
        while True:
            t = time.time()
            print(" 雙循線 ")
            ret, img = cap.read()
            data, img = double_line(img, kernel_size=25, low_threshold=10, high_threshold=20, close_size=5)

            # ser.write(str(data).encode("gbk"))
            cv2.waitKey(1)

            # mode = ser.read().decode("utf-8")
            # print("mode= ", mode)
            print('t=', 1/(time.time()-t))
            # if mode == '8' or mode!='0':
            #     break
    elif mode=='1':
        while True:
            print(" 左循線 ")
            ret, img = cap.read()
            data, img = left_line(img, kernel_size=25, low_threshold=10, high_threshold=20, close_size=5)

            ser.write(str(data).encode("gbk"))
            cv2.waitKey(1)

            mode = ser.read().decode("utf-8")
            print("mode= ", mode)
            if mode == '8' or (type(mode)!=str and mode!='1'):
                break
    elif mode=='2':
        while True:
            ret, img = cap.read()
            print(" 右循線 ")
            data, img = right_line(img, kernel_size=25, low_threshold=10, high_threshold=20, close_size=5)

            ser.write(str(data).encode("gbk"))
            cv2.waitKey(1)

            mode = ser.read().decode("utf-8")
            print("mode= ", mode)
            if mode == '8' or (type(mode)!=str and mode!='2'):
                break
    elif mode=='3':
        print(" 避障 ")
        lidar.connect()
        for i, scan in enumerate(lidar.iter_scans()):
            ret, img = cap.read()
            data, img = Avoidance(img, scan, kernel_size=25, low_threshold=10, high_threshold=20, close_size=5)
            ser.write(str(data).encode("gbk"))
            cv2.imshow("img", img)
            k=cv2.waitKey(1)
            mode = ser.read().decode("utf-8")
            if mode == '8':
                data = -9876    # 不再次發送訊號
                break
        lidar.stop()
        lidar.disconnect()
    elif mode=='4':
        for i in range(80):
            print(" 左循線 ")
            ret, img = cap.read()
            data, img = left_line(img, kernel_size=25, low_threshold=10, high_threshold=20, close_size=5)
            ser.write(str(data*1.5).encode("gbk"))
            cv2.waitKey(1)
            mode = ser.read().decode("utf-8")

        while True:
            print(" 雙黃線 ")
            ret, img = cap.read()
            data, img = double_yellow_line(img, kernel_size=25, low_threshold=10, high_threshold=20, close_size=5)
            if data==-777:
                ser.write(str('-777').encode("gbk"))
                ser.write(str('-777').encode("gbk"))
                ser.write(str('-777').encode("gbk"))
                break
            ser.write(str(data).encode("gbk"))
    elif mode=='9':
        print(" 停車 ")
        lidar.connect()
        for i, scan in enumerate(lidar.iter_scans()):
            ret, img = cap.read()
            data, img = parking(img, scan)

            if data<-5:
                ser.write(str('-99').encode("gbk"))
            elif data>5:
                ser.write(str('-88').encode("gbk"))
            else:
                ser.write(str('1').encode("gbk"))

            ser.read().decode("utf-8")
            if data<-5 or data>5:
                data = -9876    # 不再次發送訊號
                break
        print("test")
        lidar.stop()
        lidar.disconnect()


    if data != -9876:
        ser.write(str(data).encode("gbk"))


    # cv2.imshow("img", img)
    k=cv2.waitKey(1)
    if k==ord('q'):
        cv2.destroyAllWindows()
        break


# out.release()
cap.release()
ser.close()
cv2.destroyAllWindows()
lidar.stop()
lidar.disconnect()