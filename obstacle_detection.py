import cv2
import numpy as np
from matplotlib import pyplot as plt
from start_cameras import Start_Cameras
from datetime import datetime
import time
import os
from os import path
import Jetson.GPIO as GPIO
import time

brake_pin = 18

photo_height = 360
photo_width = 1280
img_height = 360
img_width = 640

np.seterr(divide='ignore', invalid='ignore')

def img_capture():
    def calcs():
        cnt = 0
        x = 0
        y = 0
        map = []
        while(x < 640):
            x += 20
            while(y < 360):
                y += 20
                div = 11
                try:
                    div = float(609.6/displ[x,y])
                    cnt += 1
                    map.append(div)
                except:
                    map.append(99)
        return map

    def calc(idx):
        x = 0
        y = 0
        map = []
        while(x < 640):
            x += 20
            while(y < 360):
                y += 20
                div = 11
                try:
                    div = float(609.6/displ[x,y])
                    global map_cnt
                    map_cnt += 1
                    map.append(div)
                except:
                    map.append(99)
        return map[idx]

    def brake():
        idx = calcs()
        GPIO.setmode(GPIO.BOARD) 
        GPIO.setup(brake_pin, GPIO.OUT, initial=GPIO.HIGH)
        i = 0
        for i in range(len(calcs())):
            if(i < 36):
                if(float(idx[i]) <= 10):
                    GPIO.output(brake_pin, GPIO.HIGH)
                    # print(float(idx[i]))
                    print("o")
                else:
                    GPIO.output(brake_pin, GPIO.LOW)
                    # print(float(idx[i]))
                    print("x")
            elif(i > 36):
                if(
                    float(idx[i]) <= 10 and float(idx[i-1]) <= 10 and float(idx[i+1]) <= 10
                    and float(idx[i-18]) <= 10 and float(idx[i-19]) <= 10 and float(idx[i-17]) <= 10
                    and float(calc(idx+18)) <= 10 and float(idx[i+19]) <= 10 and float(idx[i+17]) <= 10
                ):
                    GPIO.output(brake_pin, GPIO.HIGH) 
                    # print(float(idx[i]))
                    print("o")
                else:
                    GPIO.output(brake_pin, GPIO.LOW)
                    # print(float(idx[i]))
                    print("x")
            else:
                GPIO.output(brake_pin, GPIO.LOW)
                # print(float(idx[i]))
                print("x")
            i += 1

        GPIO.cleanup()
        cv2.destroyAllWindows
        time.sleep(1)

    # start
    left_camera = Start_Cameras(0).start()
    right_camera = Start_Cameras(1).start()
    
    left_grabbed, left_frame = left_camera.read()
    right_grabbed, right_frame = right_camera.read()
    
    if left_grabbed and right_grabbed:
        images = np.hstack((left_frame, right_frame))
        if path.isdir('../images') == True:
            filename = "../images/image_" + str(1).zfill(2) + ".png"
            cv2.imwrite(filename, images)
        else:
            os.makedirs("../images")
            filename = "../images/image_" + str(1).zfill(2) + ".png"
            cv2.imwrite(filename, images)
                
            t2 = datetime.now()
            next   
    filename = '../images/image_'+ str(1).zfill(2) + '.png'
    pair_img = cv2.imread(filename, -1)
    imgLeft = pair_img[0:img_height, 0:img_width]
    imgRight = pair_img[0:img_height, img_width:photo_width]
    leftName = '../pairs/left_' + str(1).zfill(2) + '.png'
    rightName = '../pairs/right_' + str(1).zfill(2) + '.png'
    cv2.imwrite(leftName, imgLeft)
    cv2.imwrite(rightName, imgRight)
    
    left_camera.stop()
    left_camera.release()
    right_camera.stop()
    right_camera.release()
    cv2.destroyAllWindows()
    
    imgL = cv2.imread(leftName, 1)
    imgR = cv2.imread(rightName, 1)
    
    window_size = 3
    min_disp = 0
    num_disp = 128-min_disp
    matcher_left = cv2.StereoSGBM_create(
		blockSize = 5,
		numDisparities = num_disp,
		minDisparity = min_disp,
		P1 = 8*3*window_size**2,
		P2 = 32*3*window_size**2,
		disp12MaxDiff = 1,
		uniquenessRatio = 15,
		speckleWindowSize = 0,
		speckleRange = 5,
		preFilterCap = 63,
		mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
		)
    matcher_right = cv2.ximgproc.createRightMatcher(matcher_left)
    
    lmbda = 80000
    sigma = 1.2
    
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=matcher_left)
    wls_filter.setLambda(lmbda)
    wls_filter.setSigmaColor(sigma)
    
    displ = matcher_left.compute(imgL, imgR) .astype(np.float32)
    displ = np.int16(displ)
    dispr = matcher_right.compute(imgR, imgL) .astype(np.float32)
    dispr = np.int16(dispr)
    
    filteredImg = wls_filter.filter(displ, imgL, None, dispr)
    filteredImg = cv2.normalize(
		src=filteredImg,
		dst=filteredImg,
		beta=1,
		alpha=255,
		norm_type=cv2.NORM_MINMAX,
        dtype=cv2.CV_8U
        )
    filteredImg = np.uint8(filteredImg)

    brake()


if __name__ == "__main__":
    while True:
        img_capture()