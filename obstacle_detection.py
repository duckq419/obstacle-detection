import cv2
import numpy as np
from matplotlib import pyplot as plt
from start_cameras import Start_Cameras
from datetime import datetime
import time
import os
from os import path
import RPi.GPIO as GPIO
import time

brake_pin = 12
photo_height = 360
photo_width = 1280
img_height = 360
img_width = 640

# search 1
def calc(idx):
    x = 0
    y = 0
    map = []
    while(x < 640):
        x += 20
        while(y < 360):
            y += 20
            div = 11
            if(float(509.6/displ[x,y]) > 0): # to do not div 0
                div = float(609.6/displ[x,y])
            map.append(div)
    return map[idx]
    
while True:
    left_camera = Start_Cameras(0).start()
    right_camera = Start_Cameras(1).start()
    cv2.namedWindow("Images", cv2.WINDOW_NORMAL)
    
    t2 = datetime.now()
    t1 = datetime.now()
    
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
    
    # cv2.imshow('Disparity', filteredImg)

    # search 2
    GPIO.setmode(GPIO.BOARD)
    cnt = 0
    idx = -1
    min = 99999
    try:
        while(calc(cnt) != None):
            if(calc(cnt) < min and calc(cnt) >= 0):
                min = calc(cnt)
                if(calc(cnt) == min):
                    idx = cnt
                print(calc(cnt), min)
            cnt += 1
    except:
        print(-1)

    if(calc(idx) <= 10 and calc(idx) >= 8):
        GPIO.setup(brake_pin, GPIO.OUT)
    elif(
        calc(idx) <= 10 and calc(idx-1) <= 10 and calc(idx+1) <= 10
        and calc(idx-18) <= 10 and calc(idx-19) <= 10 and calc(idx-17) <= 10
        and calc(idx+18) <= 10 and calc(idx+19) <= 10 and calc(idx+17) <= 10
    ):
        GPIO.setup(brake_pin, GPIO.OUT)
    else:
        GPIO.output(brake_pin, GPIO.LOW)
    print(min)
    GPIO.cleanup()
    cv2.waitKey(0)
    cv2.destroyAllWindows
    time.sleep(1)