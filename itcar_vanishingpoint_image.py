import cv2
import numpy as np

RadToDree = 57.2957795

img = cv2.imread('1.jpg')
img = cv2.resize(img, (800, 600))
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
gray_not = cv2.bitwise_not(gray)
blur = cv2.GaussianBlur(gray,(5,5),0)
blur_not = cv2.GaussianBlur(gray_not,(5,5),0)
ret,thresh = cv2.threshold(blur_not,150,255,cv2.THRESH_BINARY)
edges = cv2.Canny(blur,125,200,apertureSize = 3)

minLineLength = 30
maxLineGap = 10

lines = cv2.HoughLines(edges,1,np.pi/180,200)

i = 1
if lines is not None:
    for line in lines:
        for rho,theta in line:
            #print(i)
            i=i+1
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))

            cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)

cv2.imshow('edge', edges)
cv2.imshow('hough',img)
cv2.imwrite('houghlines3.jpg',img)
