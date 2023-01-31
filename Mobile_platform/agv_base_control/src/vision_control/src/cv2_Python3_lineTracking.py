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
            
            img_line = img
            
            # The below for loop runs till r and theta values
            # are in the range of the 2d array
            try:
                for r_theta in lines:
                    arr = np.array(r_theta[0], dtype=np.float64)
                    r, theta = arr
                    # Stores the value of cos(theta) in a
                    a = np.cos(theta)

                    # Stores the value of sin(theta) in b
                    b = np.sin(theta)

                    # x0 stores the value rcos(theta)
                    x0 = a*r

                    # y0 stores the value rsin(theta)
                    y0 = b*r

                    # x1 stores the rounded off value of (rcos(theta)-1000sin(theta))
                    x1 = int(x0 + 1000*(-b))

                    # y1 stores the rounded off value of (rsin(theta)+1000cos(theta))
                    y1 = int(y0 + 1000*(a))

                    # x2 stores the rounded off value of (rcos(theta)+1000sin(theta))
                    x2 = int(x0 - 1000*(-b))

                    # y2 stores the rounded off value of (rsin(theta)-1000cos(theta))
                    y2 = int(y0 - 1000*(a))

                    # cv2.line draws a line in img from the point(x1,y1) to (x2,y2).
                    # (0,0,255) denotes the colour of the line to be
                    # drawn. In this case, it is red.
                    cv2.line(img_line, (x1, y1), (x2, y2), (0, 0, 255), 2)
            except:
                print("ERROR!")
            cv2.imshow('Captured image',img)
            cv2.imshow('Canny Filter', edges)
            #cv2.imshow('Line detection',img_line)

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
