import cv2
import numpy as np
import math
from gpiozero import Motor
from gpiozero import LED
from gpiozero import Button
import RPi.GPIO as GPIO
from gpiozero import PWMOutputDevice
import time

RadToDree = 57.2957795

#duty for center servo
duty_central = 7.5
duty_min = 5
duty_max = 10.5

ratio = 0.3
ratio_turn = 0
ratio_angle = 0.2

cap = cv2.VideoCapture(0)

#Motor Init
motorleft = Motor(17, 27)
motorright = Motor(4, 14)
#Current Speed
Cur_Speed_Left = 50
Cur_Speed_Right = 50

#button Init 25
GPIO.setup(25, GPIO.IN, pull_up_down = GPIO.PUD_UP)
#Servo Init
GPIO.setup(12, GPIO.OUT)
pwm = GPIO.PWM(12,50)
pwm.start(0)
Cur_Angle = 45

def SetAngle(angle):
    duty = float(angle) / 17 + duty_min
    GPIO.output(12, True)
    pwm.ChangeDutyCycle(float(duty))
    #Speed(0,0)
    #time.sleep(0.2)
    #GPIO.output(12, False)
    #pwm.ChangeDutyCycle(0)
    return True

def Speed(left, right):
    if (left < -100):
        left = -100
    if (left > 100):
        left = 100
    if (right < -100):
        right = -100
    if (right > 100):
        right = 100
    if (left < 0):
        motorleft.backward((float(-left)/100) * ratio)
    else:
        motorleft.forward((float(left)/100) * ratio)

    if (right < 0):
        motorright.backward((float(-right)/100) * ratio)
    else:
        motorright.forward((float(right)/100) * ratio)

#cua qua phai
def CalSpeed_left(middle, left, right):
    #print(middle, left, right)
    left_speed = ((600 - middle) * 100) / (600 - right)
    right_speed = ((600 - middle) *100) / (600 - left)
    #print(left_speed,right_speed)
    Speed(100, right_speed)
# cua qua trai
def CalSpeed_right(middle, left, right):
    #print(middle, left, right)
    left_speed = (middle * 100) / right
    right_speed = (middle * 100) / left
    #print(left_speed,right_speed)
    Speed(left_speed, 100)

def GetDeviation(x,y,center):
    x = x - middle
    a = math.atan(x/y)
    print(a * RadToDree)
    return a * RadToDree

while not (SetAngle(Cur_Angle)):
    print('not')
    pass
time.sleep(1)

print('set angle')

#button.wait_for_press()
while (GPIO.input(25)):
    pass
while not (GPIO.input(25)):
    pass
state=1
while (state):
#while (True):
    if not (GPIO.input(25)):
        state = 0
    ret, frame = cap.read()

    frame = cv2.resize(frame, (800, 600))

    crop_img = frame[100:300, 100:700]

    height, width = crop_img.shape[:2]

    middle = width/2
    leftside = middle - 100
    rightside =  middle + 100

    gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)

    blur = cv2.GaussianBlur(gray,(5,5),0)

    ret,thresh = cv2.threshold(blur,150,255,cv2.THRESH_BINARY)

    contours,hierarchy  = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)

    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        cx = float(M['m10']/M['m00'])
        cy = float(M['m01']/M['m00'])
        cv2.line(crop_img,(int(cx),0),(int(cx),720),(255,0,0),1)
        cv2.line(crop_img,(0,int(cy)),(1280,int(cy)),(255,0,0),1)
        cv2.drawContours(crop_img, contours, -1, (0,255,0), 1)

        output = cx
        #print(int(cx), int(cy), middle)
        deviation = GetDeviation(cx, cy, middle)
        if output > rightside:
            print('turn left')
            CalSpeed_left(output, leftside, rightside)
            SetAngle(Cur_Angle + int(deviation * ratio_angle))
        elif output < leftside:
            print('turn right')
            CalSpeed_right(output, leftside, rightside)
            SetAngle(Cur_Angle + int(deviation * ratio_angle))
        else:
            print('on road')
            Speed(50,50)


    #cv2.imshow('frame',crop_img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
SetAngle(45)
cap.release()
cv2.destroyAllWindows()
