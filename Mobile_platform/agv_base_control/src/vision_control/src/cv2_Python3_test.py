#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from PIL import ImageFont, ImageDraw
import PIL

msgsTopic = "/vision"
data = ""
bbox = []

def capture_camera():
    global data, bbox, img_gray
    # Checks and deletes the output file
    # You cant have a existing file or it will through an er
    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    
    # ROS config
    # Publish messages with Image type
    pub = rospy.Publisher(msgsTopic,Image, queue_size=10)

    # Set rospy node name
    rospy.init_node('vision_control', anonymous=True)

    # Go through the loop 10 times per second
    rate = rospy.Rate(10)
    
    # Create a Video Capture Object
    cap = cv2.VideoCapture(0)

    # Create a QRCode reader
    qrcode= cv2.QRCodeDetector()

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    currentFrame = 0
    currentFlag = 0
    # Get current width of frame
    width = cap.get(3)   # float
    # Get current height of frame
    height = cap.get(4) # float
    # Define the codec and create VideoWriter object
    
    if cap.isOpened():
        while not rospy.is_shutdown():
            ret_val, img = cap.read()
            #print("Captured image is: ",img)
            #print("Captured image type is: ", type(img))
           
            if(ret_val == True):
                # Print bugging information to the terminal
                # rospy.loginfo('publishing video frame')

                # Resize the captured image for speed up the process (QRCode)
                #img_resized = cv2.resize(img,(720,420))
                #img_resized = cv2.resize(img,(176,120))
                img_resized = cv2.resize(img,(400,300))
                # Recognizing the QRCode
                img_gray = cv2.cvtColor(img_resized,cv2.COLOR_BGR2GRAY)
                ready,data,bbox,rectified = qrcode.detectAndDecodeMulti(img_gray)
                if(bbox is not None):
                    #print(f"[QRCode] QRCode is detected with data {data}, bbox {bbox}, rectified {rectified}")
                    pass
                if ready:
                    for i in range(len(data)):
                        text = data[i]      	# QRCode content
                        print(f"[QRCode] Detected text is : {data[i]}")
                        box = boxSize(bbox[i])	# QRCode cordinate
                        #print(f"[QRCode] Detected Box is : {bbox[i]}")
                        cv2.rectangle(img_gray,(box[0],box[1]),(box[2],box[3]),(0,0,255),5)	# Draw the boundary of QRCode
                        #putText(box[0],box[3],text,color=(0,0,255))
                cv2.imshow('CSI Camera',img_gray)
                #print("[INFO] Frame Number:",currentFrame)
                #out.write(img)
                #print("Type of image before publishing is: ", type(img_gray))
                # Converts Opencv image to ROS image message
                pub.publish(br.cv2_to_imgmsg(img_gray,"passthrough"))

                if(currentFrame%4==0):
                    currentFlag+=1
                currentFrame+=1
                if cv2.waitKey(1)&0xFF == ord('q'):
                    break
            else:
                break
        rate.sleep()
        cap.release()
        cv2.destroyAllWindows()
    else:
        print('Unable to open camera')

def putText(x,y,text,color=(0,0,0)):
    global img_gray
    #fontpath = 'arial.ttf'
    #font = ImageFont.truetype(fontpath, 20)
    imgPil = PIL.Image.fromarray(img_gray)
    draw = ImageDraw.Draw(imgPil)
    #draw.text((x,y), text, fill=color, font=font)
    draw.text((x,y), text) 
    img_resized = np.array(imgPil)

def boxSize(arr):
    global data
    box_roll = np.rollaxis(arr,1,0)
    xmax = int(np.amax(box_roll[0]))
    xmin = int(np.amin(box_roll[0]))
    ymax = int(np.amax(box_roll[1]))
    ymin = int(np.amin(box_roll[1]))
    return (xmin,ymin,xmax,ymax)


if __name__ == '__main__':
    capture_camera()
