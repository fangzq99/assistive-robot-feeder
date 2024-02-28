#!/usr/bin/env python3


from tensorflow.keras.models import load_model
from PIL import Image, ImageOps
import numpy as np
import subprocess
import os, sys
#import cv2
#import numpy as np

from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import String
import rospy

rospy.init_node('Processor')


def classify(image_name):
	#this fnc will process the image provided
	#the model has to be in the same folder
	
	
	# Load the model
	model = load_model('keras_model.h5', compile=False)
	# Create the array of the right shape to feed into the keras model
	# The 'length' or number of images you can put into the array is
	# determined by the first position in the shape tuple, in this case 1.
	data = np.ndarray(shape=(1, 224, 224, 3), dtype=np.float32)
	# Replace this with the path to your image
	image = Image.open(image_name)
	#resize the image to a 224x224 with the same strategy as in TM2:
	#resizing the image to be at least 224x224 and then cropping from the center
	size = (224, 224)
	image = ImageOps.fit(image, size, Image.ANTIALIAS)

	#turn the image into a numpy array
	image_array = np.asarray(image)
	# Normalize the image
	normalized_image_array = (image_array.astype(np.float32) / 127.0) - 1
	# Load the image into the array
	data[0] = normalized_image_array

	# run the inference
	prediction = model.predict(data)
	#convert to list from np array
	listpredict = prediction.tolist()
	#find the index of highest probabilty
	print("Probability of food for area:")
	print(max(listpredict))
	index = listpredict.index(max(listpredict))
	return index

def writefile(index):

	#write to file
	#Find index of highest probailty
	#Locate value given the index

	global f,labels
	f.write(labels[index])
	f.write('\n')
	print("\n\nFood for area is: ")
	print(labels[index])
	return 'completed'


def main():
		global labels,f ,f2

		print("starting... main prog")
		try:
			print("Opening files..")

			#create predictions file
			f = open("predictions.txt","w")
			#read the labels file and store values
			f2 = open("labels.txt", 'r+')
			labels = f2.read().split("\n")
			print("Class labels are: ")
			print(labels)


		
		print("Processing now..")
		#classify the image by running the model 
		#write to predictions.txt file
		writefile(classify('area1.jpeg'))
		writefile(classify('area2.jpeg'))
		writefile(classify('area3.jpeg'))
		writefile(classify('area4.jpeg'))

		try:
			print("closing files..")
			f.close()
			f2.close()
		except:
			print('cant close file')
		print("Trying to send..")
		sender()
	# else:
	# 	pass
		return print("completed")

#sends prediciton text file
def sender():

	### for python2.7
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

	    subprocess.run = sp_run

	#send files to niryo
	subprocess.run(["scp", 'predictions.txt', 'niryo@169.254.200.200:/home/niryo'])
	subprocess.run(["scp", 'emptydoc.txt', 'niryo@169.254.200.200:/home/niryo'])
	os.remove("emptydoc.txt")
	print("sent..")


def listener():

	while True:
		print("Node Name: Processing \nTopic Name: Start \n Topic Type: String \n Keyword: no emptydoc.txt")

		#if file exists (images has been transferred)
		#execute main
		if os.path.exists("emptydoc.txt"):
			main()
		else:
			pass
		rospy.sleep(1)
if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		rospy.shutdown()
####
