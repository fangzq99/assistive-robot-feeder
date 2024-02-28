#!/usr/local/bin/python
# -*- coding: utf-8 -*-
import os, sys
import cv2
import numpy as np
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import String
import subprocess

#process for this node is as follows

#1) take a photo and divide it up into 4 region of interest
#2) send the images to laptop for processing
#3) the program will continously try to check if a file 'emptydoc.txt' exists
#4) This file will only be sent when the processing on the laptop has been completed
#5) Once the file can be read, then the prediction are stored and used

rospy.init_node('FoodSend')
pub2 = rospy.Publisher('food_location_vision',Int16,queue_size=10)


def sender():
    print("sending..")

    ####This part is for python2.7 as subprocess module
    ####previously did not contain CompletedProcess class
    try:
        from subprocess import CompletedProcess
    except ImportError:
        # Python 2
        class CompletedProcess:

            def __init__(self, args, returncode, stdout=None, stderr=None):
                self.args = args
                self.returncode = returncode
                self.stdout = stdout
                self.stderr = stderr

            def check_returncode(self):
                if self.returncode != 0:
                    err = subprocess.CalledProcessError(self.returncode, self.args, output=self.stdout)
                    raise err
                return self.returncode

        def sp_run(*popenargs, **kwargs):
            input = kwargs.pop("input", None)
            check = kwargs.pop("handle", False)
            if input is not None:
                if 'stdin' in kwargs:
                    raise ValueError('stdin and input arguments may not both be used.')
                kwargs['stdin'] = subprocess.PIPE
            process = subprocess.Popen(*popenargs, **kwargs)
            try:
                outs, errs = process.communicate(input)
            except:
                process.kill()
                process.wait()
                raise
            returncode = process.poll()
            if check and returncode:
                raise subprocess.CalledProcessError(returncode, popenargs, output=outs)
            return CompletedProcess(popenargs, returncode, stdout=outs, stderr=errs)

        #initialise
        subprocess.run = sp_run

    #emptydoc.txt helps to let the laptop know that all files has been sent
    #try and except does the following:
    #if not file create one
    #however, when python try to create a file that already exists
    #an error will be raised, thus if an error is raised
    #we know that a file is already present and can skip
    try:
            print("No file creating...")
            f3 = open('emptydoc.txt', 'wx')
            f3.close()
    except:
            pass
    print("trying to send...")

    #send files
    subprocess.run(["scp", 'originalimg.jpeg', 'gordon@169.254.200.201:/home/gordon/test/test3.5/src'])
    subprocess.run(["scp", imgname[0], 'gordon@169.254.200.201:/home/gordon/test/test3.5/src'])
    subprocess.run(["scp", imgname[1], 'gordon@169.254.200.201:/home/gordon/test/test3.5/src'])
    subprocess.run(["scp", imgname[2], 'gordon@169.254.200.201:/home/gordon/test/test3.5/src'])
    subprocess.run(["scp", imgname[3], 'gordon@169.254.200.201:/home/gordon/test/test3.5/src'])
    subprocess.run(["scp", 'emptydoc.txt', 'gordon@169.254.200.201:/home/gordon/test/test3.5/src'])
    print("Sent...")

    #remove file so that program does not continue
    os.remove("emptydoc.txt")
def takepic():
        print("taking pic..")
        cap = cv2.VideoCapture(0)

        #Set frame to 1280x960
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)

        ret, frame = cap.read()
        #capture an image with name originalimg.jpeg
        cv2.imwrite('originalimg.jpeg', frame)

        #load the image
        image = cv2.imread('originalimg.jpeg')

        #divide the image
        roi_1 = image[200:380, 250:700]
        roi_2 = image[0:200, 280:400]
        roi_3 = image[0:200, 400:510]
        roi_4 = image[0:200, 510:615]

        img_item1 = imgname[0]
        img_item2 = imgname[1]
        img_item3 = imgname[2]
        img_item4 = imgname[3]

        #write divided images
        cv2.imwrite(img_item1, roi_1)
        cv2.imwrite(img_item2, roi_2)
        cv2.imwrite(img_item3, roi_3)
        cv2.imwrite(img_item4, roi_4)
        # When everything done, release the capture

        cap.release()
        cv2.destroyAllWindows()
        print("Pics taken...")
def main(msg_data):

        global food_choice
        sec_passed=0
        #received food choice fro main loop
        food_choice = msg_data.data
        print(food_choice)

        #take pic and call sender fnc
        takepic()
        sender()

        print('now waiting for files')

        #Check if file is here
        #if file exist run process fnc which reads file
        #if not, continue to read
        processcheck = False
        while processcheck == False:
            if os.path.exists("emptydoc.txt"):
                print("My images are here...")
                process()
                processcheck = True
            else:
                print("waiting for images...")
                pass
            rospy.sleep(1)

            #send again if 60 seconds has passed and no mesage returned
            #else do nothing
            if sec_passed > 60:
                sender()
                sec_passed=0
            else:
                pass
            sec_passed += 1
def process():

    #fnc does the following
    #read files
    #store values in food_dict
    #check food_dict values for food_choice
    #return food_dict key
    #publish to main loop

    global food_choice
    print("reading prediction..")
    f = open("predictions.txt", "r")

    #read file, split by new line
    predictions = f.read().split("\n")
    print(predictions)
    f.close()

    #store prediction in food_dict
    food_dict["Area 1"] = predictions[0]
    food_dict["Area 2"] = predictions[1]
    food_dict["Area 3"] = predictions[2]
    food_dict["Area 4"] = predictions[3]
    print(food_choice)

    #check where food_choice is
    if food_choice == food_dict["Area 1"]:
        data = 1
        pub2.publish(data)
    elif food_choice == food_dict["Area 2"]:
        data = 2
        pub2.publish(data)
    elif food_choice == food_dict["Area 3"]:
        data = 3
        pub2.publish(data)
    elif food_choice == food_dict["Area 4"]:
        data = 4
        pub2.publish(data)
    else:
        data = 0
        pub2.publish(data)

global food_dict
global food_choice
global imgname

#define a dict to store food predictions are location
food_dict = {
    "Area 1": "somefood",
    "Area 2": "somefood",
    "Area 3": "somefood",
    "Area 4": "somefood"
}

#image names
imgname = ['area1.jpeg','area2.jpeg','area3.jpeg','area4.jpeg']


if __name__ == '__main__':
    try:
        print("Waiting")
        rospy.Subscriber('food_name_vision', String, main)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.shutdown()
####

