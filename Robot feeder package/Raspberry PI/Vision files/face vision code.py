#!/usr/local/bin/python
# -*- coding: utf-8 -*-

from niryo_one_python_api.niryo_one_api import *
import rospy
import time
import math
import os, sys
import cv2
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import Float64

n = NiryoOne()
n.set_arm_max_velocity(10)

rospy.init_node('CameraV2')
pubstart = rospy.Publisher('vision_init',Bool,queue_size=10)

#this program will do the following
#move to an observation pose
#attempt to detect a face

def Camera():
    count = 0
    #height and width of real object taken from gordon
    widthReal = 85
    heightReal = 120

    #Camera focals
    xfocal = 507.80557389
    yfocal = 507.56957351

    #target points of image
    z_move, x_move = 0, 0
    ztarget = 360
    xtarget = 360

    #initialise cascade type
    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_alt.xml')

    # Open the device at the ID 1 (USB)
    cap = cv2.VideoCapture(0)

    #To set the resolution
    #remove cv.cv depeding on version
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    obs =   [-0.058297595633625035, 0.6146183522286197, -0.7619608848706673, 0.00460766922526503, 0.20120155616990631, -2.4]
    n.move_joints(obs)

        #repeat to update stream until a face is found
        while x == 0:
            #setup frame
            ret, frame = cap.read()

            #convert frame to grayscale as haarcascades only works on greyscaled images
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #convert to grey for haarcascade as per documentation

            #run classifier
            faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5) #1.5, 5 standard values ; scaling factor increase for higher accuracy
            # x,y = x,y coordinates, h,w = height, weight unit pixel

            if faces != 0:
                x = 1
            else:
                pass

        for (x, y, w, h) in faces:
            pixelcoord =[x,y,w,h]
            print(pixelcoord)

            #roi = region of interest
            roi_gray = gray[0:720, 0:720]
            img_item = "saved-img.png"

            imgx = x
            imgy = y
            imgwidthpixel = w
            imgheightpixel = h

            #Find distance between object and robot
            distance = ( widthReal * xfocal ) / imgwidthpixel
            distance2 = ( heightReal * yfocal ) / imgheightpixel
            print(distance,distance2)

            #Centre of face
            xPixel_centre = imgx + ( imgwidthpixel / 2 )
            zPixel_centre = imgy + ( imgheightpixel / 2 )
            print'x,z pixel centre'
            print(xPixel_centre,zPixel_centre )

            #pixel difference
            xDiff = ( xtarget - xPixel_centre )
            zDiff = ( ztarget - zPixel_centre )
            print('xDiff and zDiff are (in pixels):')
            print( xDiff,zDiff )

            #finding real distance to move
            #we have to multiply z to inverse its sign as Pixel y-axes and Niryo Z-Axes are inverted
            z_move = (-1 ) * ( zDiff * distance ) /  yfocal
            x_move = ( xDiff * distance ) /  xfocal
            print'phyiscal distance z,x is (in mm)'
            print(z_move, x_move)
            z_move /= 1000
            x_move /= 1000

            print'phyiscal distance is (in m)'
            print(z_move, x_move)

            n.wait(1)

            #limit movement if misclassify
            if z_move > 0.20:
                z_move =0.15
            else:
                pass
            if x_move > 0.15:
                x_move = 0.15
            else:
                pass
            n.shift_pose(AXIS_Z, (z_move))
            n.shift_pose(AXIS_Y, x_move)
            n.wait(3)

            distancetemp = distance / 1000
            #limit movement if misclassify
            if distancetemp > 0.30:
                distancetemp = 0.3
            else:
                pass

            n.shift_pose(AXIS_X, distancetemp)

            #publish a message to main loop that program has ended
            start = True
            pubstart.publish(start)
            n.wait(10)
            break

        if cv2.waitKey(20) & count > 20:
            break
    #Check whether user selected camera is opened successfully.
    if not (cap.isOpened()):
        print('Could not open video device')

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        Camera()
    except rospy.ROSInterruptException:
        pubstart.publish(False)
        cap.release()
        cv2.destroyAllWindows()
        n.activate_learning_mode(True)
        rospy.shutdown()#!/usr/local/bin/python

