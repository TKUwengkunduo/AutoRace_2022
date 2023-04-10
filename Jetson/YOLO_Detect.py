"""
    YOLO & Python 環境需求
        1. .data
        2. .name
        3. .cfg
        4. .weight
        5. darknet(shared library)
        6. darknet.py
        7. libdarknet.so
"""


import time
from turtle import delay
import cv2
import numpy as np
import darknet
import serial  # 引用pySerial模組



# 紅綠燈遮罩
G_Low = [44,0,181]
G_high = [101,229,255]
G_height = [114,99]
G_width = [285,267]

# G_Low = [44,0,181]
# G_high = [101,229,255]
# G_height = [114,100]
# G_width = [282,266]




COM_PORT = '/dev/ttyTHS0'    # 指定通訊埠名稱
BAUD_RATES = 9600    # 設定傳輸速率
ser = serial.Serial(COM_PORT, BAUD_RATES, timeout=0.01)   # 初始化序列通訊埠


cap = cv2.VideoCapture(0)


# 神經網路檔案位置
data_path = '/home/iclab/AutoRace/cfg/AutoRace.data'
cfg_path = '/home/iclab/AutoRace/cfg/yolov4-tiny.cfg'
weights_path = '/home/iclab/AutoRace/cfg/yolov4-tiny_best_old.weights'


lable_choose = [0, 0, 0, 0, 0, 0]
SW = 1





"""
載入神經網路
"""
network, class_names, class_colors = darknet.load_network(
        cfg_path,
        data_path,
        weights_path,
        batch_size=1
)




"""
影像檢測
    輸入:(影像位置,神經網路,物件名稱集,信心值閥值(0.0~1.0))
    輸出:(檢測後影像,檢測結果)



    
    註記:
"""
def image_detection(image, network, class_names, class_colors, thresh):
    width = darknet.network_width(network)
    height = darknet.network_height(network)
    darknet_image = darknet.make_image(width, height, 3)

    
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_resized = cv2.resize(image_rgb, (width, height),
                               interpolation=cv2.INTER_LINEAR)

    darknet.copy_image_from_bytes(darknet_image, image_resized.tobytes())
    detections = darknet.detect_image(network, class_names, darknet_image, thresh=thresh)
    darknet.free_image(darknet_image)
    image = darknet.draw_boxes(detections, image_resized, class_colors)

    # print(detections)nnnnnnnnnnnnnnnjj

    return detections



"""
座標轉換
    輸入:(YOLO座標,原圖寬度,原圖高度)
    輸出:(框的左上座標,框的右下座標)
    註記:
"""
def bbox2points(bbox,W,H):
    """
    From bounding box yolo format
    to corner points cv2 rectangle
    """ 
    width = darknet.network_width(network)      # YOLO壓縮圖片大小(寬)
    height = darknet.network_height(network)    # YOLO壓縮圖片大小(高)

    x, y, w, h = bbox                           # (座標中心x,座標中心y,寬度比值,高度比值)
    x = x*W/width
    y = y*H/height
    w = w*W/width
    h = h*H/height
    x1 = int(round(x - (w / 2)))
    x2 = int(round(x + (w / 2)))
    y1 = int(round(y - (h / 2)))
    y2 = int(round(y + (h / 2)))
    
    return x1, y1, x2, y2



"""
原圖繪製檢測框線
    輸入:(檢測結果,原圖位置,框線顏色集)
    輸出:(影像結果)
    註記:
"""
def draw_boxes(detections, image, colors):
    H,W,_ = image.shape
    img = image.copy()

    label=0

    data = []
    check=0 
    confidence_max = -1
    for label, confidence, bbox in detections:
        if confidence_max==-1 or confidence_max<confidence:
            data = []
            x1, y1, x2, y2 = bbox2points(bbox,W,H)
            data.append([x1, y1, x2, y2, label, confidence, (x1+x2)/2, (y1+y2)/2])
            confidence_max = confidence

    for i in range(len(data)):
        check=0
        for j in range(len(data)):
            if i!=j and abs(data[i][6]-data[j][6])<20 and abs(data[i][7]-data[j][7])<20:
                check=1
                break
            else:
                pass

        if check==0:
            cv2.rectangle(img, (data[i][0], data[i][1]), (data[i][2], data[i][3]), colors[data[i][4]], 1)
            cv2.putText(img, "{} [{:.2f}]".format(label, float(confidence)),
                        (data[i][0], data[i][1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        colors[data[i][4]], 2)
            # 輸出框座標_加工格式座標(左上點座標,右上點座標)
            # print("\t{}\t: {:3.2f}%    (x1: {:4.0f}   y1: {:4.0f}   x2: {:4.0f}   y2: {:4.0f})".format(label, float(confidence), x1, y1, x2, y2))



    return img, label





'''
    紅綠燈辨識
'''
def HoughCircles():

    # 回傳值(0=沒看到綠燈,1=有看到綠燈)
    look_green = 0

    # 讀取圖片並轉HSV
    ret, img = cap.read()
    img = cv2.resize(img,(320,180))
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    
    # 遮罩
    low_G = np.array(G_Low)
    up_G = np.array(G_high)
    mask_G = cv2.inRange(hsv,low_G,up_G)

    # 限制範圍
    for i in range(180):
        for j in range(320):
            if i>G_height[0] or i<G_height[1] or j>G_width[0] or j<G_width[1]:
                mask_G[i][j]=0
                # mask_G[i][j]=mask_G[i][j]


    brightness = mask_G.sum()           # 亮度
    print(brightness)

    if brightness>7000:
        look_green=1
        print("look green light")

    #顯示新圖像
    # cv2.imshow('mask_G',mask_G)
    # cv2.imshow('img',img)

    #按任意鍵退出
    # cv2.waitKey(1)

    return look_green








"""
主程式
    程式流程:
    1. 檢測影像
    2. 在原圖繪製結果
    3. 輸出影像
"""
if __name__ == "__main__":

    # wait OpenCR
    # while True:
    #     cap.read()
    #     cap.read()
    #     ret, img = cap.read()
    #     mode = ser.read().decode("utf-8")
    #     print('wait')
    #     if mode=='0':
    #         print('start')
    #         break
    
    time.sleep(2)

    # 等待綠燈
    t = time.time()
    # while True:

    #     print('wait green light')
    #     look_green = HoughCircles()

    #     ser.read().decode("utf-8")

    #     if look_green==1 or time.time()-t>15:
    #         print('start')
    #         ser.write(str(9).encode())
    #         cv2.destroyAllWindows()
    #         break


    choose_time = time.time()
    delet_time = time.time()
    while True:
        t = time.time()

        ret, img = cap.read()

        key = cv2.waitKey(1)
        if key == ord('q') or key == 27 : # Esc
            print('break')
            break
        
        # print(len(img), len(img[0]))
        detections = image_detection(img,network, class_names, class_colors, thresh=0.75)
        out_img, label = draw_boxes(detections, img, class_colors)
        # print(len(out_img), len(out_img[0]))

        cv2.imshow('out', out_img)
        

        if int(label)>0:
            lable_choose[int(label)-1] += 1


        mode = ser.read().decode("utf-8")
        # print(mode,ser.read(),'=1')
        if mode == '8':
            print("=========================")
            if max(lable_choose)>=3 and SW==1:
                choose_time = time.time()
                ser.write(str(lable_choose.index(max(lable_choose))+1).encode())
                print(lable_choose.index(max(lable_choose))+1)
                print(lable_choose)
                lable_choose = [0]*6
                SW=0
                delet_time=time.time()
            else:
                ser.write(str(0).encode())

                
        if time.time()-choose_time>2.0:
            choose_time = time.time()
            print('dd')
            lable_choose = [0]*6
            
        
        if time.time()-delet_time>9.0:
            SW=1
        

        print(1/(time.time()-t))
            

        
    cap.release()
    cv2.destroyAllWindows()

