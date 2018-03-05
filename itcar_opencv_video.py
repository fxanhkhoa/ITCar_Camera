import cv2
import numpy as np

center = 400
left = center - 50
right = center + 50

cap = cv2.VideoCapture('itcar_line3.mp4')

while(True):
    ret, frame = cap.read()

    frame = cv2.resize(frame, (800, 600))

    crop_img = frame[500:600, 200:600]

    height, width = crop_img.shape[:2]
    
    gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray,(15,15),0)

    ret, thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY)
    edge = cv2.Canny(thresh,100, 200)
    #_,contours,hierarchy  = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)

    #con_img = cv2.drawContours(thresh, contours, -1, (0,0,255), 3)

    #if len(contours) > 0:
        #c = max(contours, key = cv2.contourArea)
        #print(c)
        #M = cv2.moments(c)
        #cx = int(M['m10']/M['m00'])
        #cy = int(M['m01']/M['m00'])
        #print(cx,cy)

        #out1 = cv2.circle(crop_img,(cx,cy), 10, (0,0,255), -1)

    sumaryX = []
    for i in range(1,height):
        #print(i)
        sumaryY = []
        for j in range(1,width):
            if (edge[i][j] !=0):
                #print(edge[i][j])
                sumaryY.append(j)
                #print(j)
                
        sumaryX.append(int(np.average(sumaryY)))
        #print(sumaryX[i-1])
        #print(int(np.average(sumaryY)))
    output = int(np.average(sumaryX))
    #out2 = cv2.circle(crop_img, (output, 50), 10, (0,0,255), -1)
    out2 = cv2.circle(frame, (output + 200, 50 + 500), 10, (0,0,255), -1)

    if output > right:
        print('turn left')
    elif output < right:
        print('turn right')
    
    cv2.imshow('out2',out2)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
