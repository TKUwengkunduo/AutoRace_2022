import cv2
import matplotlib.pyplot as plt 
import numpy as np
import time
import serial

'''
    序列埠設定
    開啟權限: sudo chmod 666 /dev/ttyACM0
    https://www.itdaan.com/tw/1b422ec424e1c3519f7cee4c2ad05274
'''
ser=serial.Serial('/dev/ttyACM0',9600,timeout=0.5)
# ser.flushInput()            # 清除輸入緩存區，放棄所有內容
# ser.flushOutput()           # 清除輸出緩衝區，放棄輸出
# ser.open()


# 相機設定
cap = cv2.VideoCapture(0)
cap.set(3,5000)
cap.set(4,5000)
cap.set(cv2.CAP_PROP_BRIGHTNESS,1)

# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# out = cv2.VideoWriter('/home/iclab/autorace/line/output3.avi', fourcc, 30.0, (640, 360))

# 左右線HSV遮色閥值
L_H_low = 8
L_S_low = 8
L_V_low = 240
L_H_high = 36
L_S_high = 80
L_V_high = 255

R_H_low = 0
R_S_low = 0
R_V_low = 230
R_H_high = 0
R_S_high = 0
R_V_high = 255

# 右線遮罩
lower_R = np.array([R_H_low,R_S_low,R_V_low])
upper_R = np.array([R_H_high,R_S_high,R_V_high])

# 左線遮罩
lower_L = np.array([L_H_low,L_S_low,L_V_low])
upper_L = np.array([L_H_high,L_S_high,L_V_high])

#紅綠燈遮罩
low_G = np.array([35,90,90])
up_G = np.array([85,255,255])


# 採樣間距
W_sampling_1 = 325
W_sampling_2 = 290
W_sampling_3 = 255
W_sampling_4 = 220


# 看紅綠燈
def HoughCircles():

    # 回傳值(0=沒看到綠燈,1=有看到綠燈)
    look_green = 0

    # 讀取圖片並轉HSV
    ret, img = cap.read()
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    

    # 遮罩
    mask_G = cv2.inRange(hsv,low_G,up_G)
    
    guess_img = cv2.GaussianBlur(mask_G,(25,25), 0)

    #param1的具體實現，用於邊緣檢測    
    canny = cv2.Canny(guess_img, 20, 45)

    kernel = np.ones((2,2),np.uint8)
    gradient = cv2.morphologyEx(canny, cv2.MORPH_GRADIENT, kernel)


    #霍夫變換圓檢測
    circles= cv2.HoughCircles(gradient,cv2.HOUGH_GRADIENT,2,200,param1=45,param2=85,minRadius=1,maxRadius=80)

    
    final_ing = img.copy()
    print('no look green light')
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
    cv2.waitKey(1)

    return look_green








# 等待紅綠燈
while True:
    look_green = HoughCircles()
    if look_green==1:
        break



# 正式開始
while True:

    t = time.time()

    # 左右線X值(需重置)
    R_min_300 = 640
    R_min_240 = 640
    R_min_180 = 640
    R_min_140 = 640

    L_min_300 = 0
    L_min_240 = 0
    L_min_180 = 0
    L_min_140 = 0

    # 讀取圖片並轉HSV
    # img = cv2.imread('C:/Users/weng/Downloads/f.jpg')
    ret, img = cap.read()
    img = cv2.resize(img,(640,360))
    # img = cv2.GaussianBlur(img, (11, 11), 0)
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    # 右線遮罩
    mask_R = cv2.inRange(hsv,lower_R,upper_R)

    # 左線遮罩
    mask_L = cv2.inRange(hsv,lower_L,upper_L)


    # Canny邊緣運算
    kernel_size = 25
    blur_gray = cv2.GaussianBlur(mask_R,(kernel_size, kernel_size), 0)
    low_threshold = 10
    high_threshold = 20
    canny_img = cv2.Canny(blur_gray, low_threshold, high_threshold)

    # 閉運算(解緩Canny斷線問題)
    kernel = np.ones((5,5),np.uint8)
    gradient = cv2.morphologyEx(canny_img, cv2.MORPH_GRADIENT, kernel)

    #霍夫變換
    lines = cv2.HoughLinesP(gradient,1,np.pi/180,8,5,2)
    
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
        pass



    

    

    # Canny邊緣運算
    kernel_size = 25
    blur_gray = cv2.GaussianBlur(mask_L,(kernel_size, kernel_size), 0)
    low_threshold = 10
    high_threshold = 20
    canny_img = cv2.Canny(blur_gray, low_threshold, high_threshold)

    # 閉運算(解緩Canny斷線問題)
    kernel = np.ones((5,5),np.uint8)
    gradient = cv2.morphologyEx(canny_img, cv2.MORPH_GRADIENT, kernel)

    #霍夫變換
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


    L_min = 320-((L_min_300+L_min_240+L_min_180+L_min_140)/4)
    R_min = ((R_min_300+R_min_240+R_min_180+R_min_140)/4)-320
    target_line = int(L_min-R_min)
    print(target_line)
    # ser.write(str.encode(str(target_line)))
    ser.write(str(target_line).encode("gbk"))



    # 輸出原圖&成果
    cv2.imshow("img", img)
    # out.write(img)

    cv2.imshow("mask_R", mask_R)
    cv2.imshow("mask_L", mask_L)
    # cv2.waitKey(0)

     

    k=cv2.waitKey(1)
    if k==ord('q'):
        cv2.destroyAllWindows()
        break


cap.release()
ser.close()
cv2.destroyAllWindows()
