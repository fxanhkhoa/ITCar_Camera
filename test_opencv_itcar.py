import cv2
import numpy as np

img = cv2.imread('itcar_line.jpg')
img = cv2.resize(img, (800, 600))

crop_img = img[500:600, 100:700]
gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray,(5,5),0)

ret, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY)
edge = cv2.Canny(thresh,100, 200)
_,contours,hierarchy  = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)

con_img = cv2.drawContours(thresh, contours, -1, (0,0,255), 3)

if len(contours) > 0:
    c = max(contours, key = cv2.contourArea)
    #print(c)
    M = cv2.moments(c)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    print(cx,cy)

    #out1 = cv2.circle(crop_img,(cx,cy), 10, (0,0,255), -1)
    
sumaryX = []
for i in range(1,100):
    #print(i)
    sumaryY = []
    for j in range(1,600):
        if (edge[i][j] !=0):
            #print(edge[i][j])
            sumaryY.append(j)
            #print(j)
    sumaryX.append(int(np.average(sumaryY)))
    #print(sumaryX[i-1])
    #print(int(np.average(sumaryY)))

#out2 = cv2.circle(crop_img, (int(np.average(sumaryX)), 50), 10, (0,0,255), -1)
out2 = cv2.circle(img, (int(np.average(sumaryX)) + 100, 50 + 500), 10, (0,0,255), -1)


cv2.imshow('out1',edge)
cv2.imshow('out2',out2)
cv2.waitKey(0)
cv2.destroyAllWindows()
