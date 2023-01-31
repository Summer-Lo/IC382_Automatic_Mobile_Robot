#!/usr/bin/env python3
import cv2
import numpy as np

cap = cv2.VideoCapture(0)
currentFrame = 0
currentFlag = 0

if cap.isOpened():
    while True:
        ret, img = cap.read()
        if(ret == True):
            # Convert the image into grayscale
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Apply edge detection method on the image
            edges = cv2.Canny(gray, 50, 150, apertureSize=3)
            # Return an array of r and theta values
            lines = cv2.HoughLines(edges, 1, np.pi/180, 200)
            cv2.imshow('Captured image',img)
            cv2.imshow('Canny Filter', edges)
            if(currentFrame%4==0):
                currentFlag+=1
            currentFrame+=1
            if(cv2.waitKey(1)&0xFF == ord('q')):
                break
        else:
            break

    cap.release()
    cv2.destroyAllWindows()

else:
    print("Unable to open camera!")
