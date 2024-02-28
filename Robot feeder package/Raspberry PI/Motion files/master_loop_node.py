#!/usr/bin/env python

import rospy
import time
import math
from niryo_one_python_api.niryo_one_api import *
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Bool


rospy.init_node('main_loop', anonymous=False)


n = NiryoOne()
n.calibrate_auto()


food_choice_range = [1,2,3,4]	# Declaring the number of valid food choices to be received from rosserial, the numbers corresponds to their bowl numbers
food_options = int()    
vision_initiation_signal = Bool()
new_food_signal = Bool()
move_on = Bool()
# recalibration_move_on = Bool() # Recalibration variable which belongs to the recalibration function that was not implemented


arduinoInitPub = rospy.Publisher('system_init_arduino', Bool, queue_size=10)	# Topic that communicates with rosserial to initialize the control box
foodchoicePub = rospy.Publisher('food_choice_main', Int16, queue_size=10)	# For testing input purposes
foodVisionPub = rospy.Publisher('food_name_vision', String, queue_size=10) # Publisher which communicates with the food vision node
# recalibrateFeedingPosePub = rospy.Publisher('feeding_pose_calibration', Bool, queue_size=10)	# Recalibration topic which communicates with the face vision node, however was not implemented


default_jt = [-0.124,0.471,-1.029,0,0.312,-2.51]    # Default joint target which the arm will first move to
food_observation = [-1.57,0.333,-0.479,0.143,-1.021,1.493]
feeding_pose_joints = []	# Declaration of feeding_pose_joints to be recorded by using the face vision node as array

joint_target11 = [-1.57,0.333,-0.479,0.143,-1.021,1.493]    # Specific joint positions for bowl 1 obtained from Niryoone Studio
joint_target12 = [-1.602,0.254,-0.269,0.014,-1.338,0.182]
joint_target13 = [-1.678,0.205,-0.768,0.028,-0.258,0.025]
joint_target14 = [-1.614,0.238,-1.105,-2.906,-0.084,0.142]
joint_target15 = [-1.829,-0.332,-0.702,-2.748,-0.172,-0.552]
joint_target16 = [-1.729,-0.77,-0.355,-2.835,-0.465,-0.339]
joint_target17 = [-1.435,-0.783,-0.411,-2.597,-0.607,-0.334]
joint_target18 = [-1.137,-0.899,-0.517,-2.694,-1.184,0]
joint_target19 = [-1.156,-0.839,-0.554,-2.685,-1.204,0.1]
joint_target110 = [-1.121,-0.791,-0.79,-2.685,-1.418,0.1]
joint_target111 = [-1.323,0.435,-0.801,-2.932,-0.195,0.02]

joint_target21 = [-1.57,0.333,-0.479,0.143,-1.021,1.493]    # Specific joint positions for bowl 2 obtained from Niryoone Studio
joint_target22 = [-1.602,0.254,-0.269,0.014,-1.338,0.182]
joint_target23 = [-1.678,0.205,-0.768,0.028,-0.258,0.025]
joint_target24 = [-1.614,0.238,-1.105,-2.906,-0.084,0.142]
joint_target25 = [-1.378,-0.614,-0.142,-2.861,-0.083,-0.208]
joint_target26 = [-1.41,-0.976,0.287,-2.864,0.086,-0.046]
joint_target27 = [-1.408,-1.234,0.531,-2.874,-0.155,-0.132]
joint_target28 = [-1.163,-0.443,-0.723,-2.629,-1.08,-0.066]
joint_target29 = [-1.381,0.567,-0.908,-2.955,-0.246,0.046]
joint_target210 = [-0.554,0.612,-0.817,-2.869,0.057,-0.111]

joint_target31 = [-1.57,0.333,-0.479,0.143,-1.021,1.493]    # Specific joint positions for bowl 3 obtained from Niryoone Studio
joint_target32 = [-1.602,0.254,-0.269,0.014,-1.338,0.182]
joint_target33 = [-1.678,0.205,-0.768,0.028,-0.258,0.025]
joint_target34 = [-1.614,0.238,-1.105,-2.906,-0.084,0.142]
joint_target35 = [-1.601,-0.649,-0.057,-2.952,0.011,-0.015]
joint_target36 = [-1.588,-0.808,-0.098,-2.912,-0.23,0]
joint_target37 = [-1.584,-1.215,0.477,-3.018,-0.163,0.243]
joint_target38 = [-1.594,-1.221,0.364,-3.01,-0.452,-0.177]
joint_target39 = [-1.503,-1.228,0.363,-3.01,-0.465,-0.116]
joint_target310 = [-1.546,-1.228,0.249,-3.016,-0.842,0.081]
joint_target311 = [-1.56,-0.773,-0.514,-3.046,1.147,0.051]
joint_target312 = [-1.581,0.3,-0.608,-3.044,-0.178,0.005]
joint_target313 = [-0.304,0.603,-0.785,-3.044,0.08,0.04]

joint_target41 = [-1.57,0.333,-0.479,0.143,-1.021,1.493]    # Specific joint positions for bowl 4 obtained from Niryoone Studio
joint_target42 = [-1.602,0.254,-0.269,0.014,-1.338,0.182]
joint_target43 = [-1.678,0.205,-0.768,0.028,-0.258,0.025]
joint_target44 = [-1.614,0.238,-1.105,-2.906,-0.084,0.142]
joint_target45 = [-1.701,-0.771,-0.081,-2.904,-0.111,-0.197]
joint_target46 = [-1.656,-1.264,0.497,-2.898,-0.273,-0.106]
joint_target47 = [-1.656,-1.248,0.433,-2.99,-0.404,-0.339]
joint_target48 = [-1.555,-1.247,0.398,-2.99,-0.41,-0.364]
joint_target49 = [-1.563,-1.245,0.396,-2.944,-0.43,-0.061]
joint_target410 = [-1.562,-1.194,0.254,-2.955,-0.762,0.01]
joint_target411 = [-1.452,-0.793,-0.401,-2.691,-0.011,-0.273]
joint_target412= [0.009,0.128,-0.473,-2.499,-0.071,-0.283]


# Forward and backward movement feature during nozzle feeding that was unable to be implemented on time
# water_nozzle_offset_forward = []
# water_nozzle_offset_backward = []


def visionInitCallback(initState):
	global vision_initiation_signal
	global new_food_signal
	if (initState.data == True):
		vision_initiation_signal = True
	else:
		temp_bool = False # probably not needed
def visionInit():	# Subscriber function to allow the initialization of this main loop node upon receiving the true signal from the face vision node
	rospy.Subscriber('vision_init', Bool, visionInitCallback)


def foodChoiceCallback(choiceNum):
	global food_options
	global move_on
	if (choiceNum.data == 1):
		food_options = choiceNum.data
		move_on = True
	elif (choiceNum.data == 2):
		food_options = choiceNum.data
		move_on = True
	elif (choiceNum.data == 3):
		food_options = choiceNum.data
		move_on = True
	elif (choiceNum.data == 4):
		food_options = choiceNum.data
		move_on = True
	elif (choiceNum.data == 5):
		food_options = choiceNum.data
		move_on = True
	else:
		rospy.loginfo("CODE ERROR")	# Technically should never happen
def foodChoice():	# Subscriber function to receive the button number that was pressed from the rosserial node
	rospy.Subscriber('food_selection_arduino', Int16, foodChoiceCallback)


# Recalibration function that was unable to be implemented on time
# def poseRecalibrationCallback(RecaliChoice):
# 	global recalibration_move_on
# 	if (RecaliChoice.data == True):
# 		recalibration_move_on == True
# 	elif (RecaliChoice.data == False):
# 		recalibration_move_on == False
# def poseRecalibration():
# 	rospy.Subscriber('feeding_pose_calibration', Bool, queue_size=10)


def foodVisionCallback(choiceNum):
	global food_options
	global move_on
	if (choiceNum.data == 1):
		food_options = choiceNum.data
		move_on = True
	elif (choiceNum.data == 2):
		food_options = choiceNum.data
		move_on = True
	elif (choiceNum.data == 3):
		food_options = choiceNum.data
		move_on = True
	elif (choiceNum.data == 4):
		food_options = choiceNum.data
		move_on = True
	elif (choiceNum.data == 0):
		food_options = choiceNum.data
		move_on = True
	else:
		rospy.loginfo("CODE ERROR")	# Technically should never happen
def foodVision():	# Subscriber function to receive the bowl number that the food is in from the food vision node
	rospy.Subscriber('food_location_vision', Int16, foodVisionCallback)



if __name__ == '__main__':
	rate = rospy.Rate(1)
	n.set_arm_max_velocity(20)  # Setting of arm movement speed
	n.change_tool(TOOL_GRIPPER_1_ID)    
	n.move_joints(default_jt)   # Observation pose
	new_food_signal = True  # Initiatilization signal
	vision_initiation_signal = False    # Initialiazation signal
	# recalibration_move_on = True # Initialization signal
	food_options = 0
	n.close_gripper(TOOL_GRIPPER_1_ID, 200)	# Ensure the spoon starts in the correct orienation
	try:
		while (vision_initiation_signal != True):   # While loop to wait for calibration complete signal from the face vision ndoe
			rospy.loginfo_throttle(1,"Waiting for camera signal")
			visionInit()
		rospy.sleep(3)
		feeding_pose_joints = n.get_joints()	# Get the falibrated joint position that the robot is currently at right now immediately after the execution of the face vision node
		print("Feeding Location:")
		print(feeding_pose_joints)


		while True:
			if (new_food_signal == True):   # Loop that will run if new_food_signal is True, which will always be the first loop to be run due to the initial condition of new_food_signal == True
				time.sleep(3)
				rospy.loginfo_throttle(5,"Select new food!")
				n.move_joints(food_observation) # Observation pose
				# Pose recalibration feature that was unable to be implemented in time
				# arduinoInitPub.publish(True)
				# # if (recalibration_move_on == False):
				# # 	recalibrateFeedingPosePub.Publish(True)
				# # while (recalibration_move_on != True):
				# # 	rospy.loginfo_throttle(1,"Re-calibration in progress...")
				# # 	poseRecalibration()
				# while (move_on != True):
				# 	 rospy.loginfo_throttle(1,"Waiting for food signal...")
				# 	 foodChoice()
			 # 	move_on = False # Ensure food select loop will run in the next iteration

			 	while (food_options not in food_choice_range):
			 		if (vision_initiation_signal == True):
						arduinoInitPub.publish(True)    # If face vision node calibration signal is true, initiate the rosserial node
					else:
						rospy.loginfo("CODE ERROR") # Technically should never happen
					while (move_on != True):    # While loop to run the subscriber function foodChoice() until move_on signal is True
						 rospy.loginfo_throttle(1,"Waiting for food signal...")
						 foodChoice()   # Subscriber function to constantly listen for updates to the topic /food_selection_arduino
					if (food_options == 1):
						 rospy.loginfo("Plate 1 selected!")
						 foodchoicePub.publish(food_choice_range[0]) # Testing purpose
						 n.move_joints(joint_target11)  # Python API commands that is sent to the Niryoone package to move the Niryoone's joints
						 n.move_joints(joint_target12)
						 n.move_joints(joint_target13)
						 n.move_joints(joint_target14)
						 n.move_joints(joint_target15)
						 n.move_joints(joint_target16)
						 n.move_joints(joint_target17)
						 n.move_joints(joint_target18)
						 n.move_joints(joint_target19)
						 n.move_joints(joint_target110)
						 n.move_joints(joint_target111)
						 n.move_joints(feeding_pose_joints)
						 break
					elif (food_options == 2):
						rospy.loginfo("Plate 2 selected!")
						foodchoicePub.publish(food_choice_range[1]) # Testing purpose
						n.move_joints(joint_target21)   # Python API commands that is sent to the Niryoone package to move the Niryoone's joints
						n.move_joints(joint_target22)
						n.move_joints(joint_target23)
						n.move_joints(joint_target24)
						n.move_joints(joint_target25)
						n.move_joints(joint_target26)
						n.move_joints(joint_target27)
						n.move_joints(joint_target28)
						n.move_joints(joint_target29)
						n.move_joints(joint_target210)
						n.move_joints(feeding_pose_joints)
						break
					elif (food_options == 3):
						rospy.loginfo("Plate 3 selected!")
						foodchoicePub.publish(food_choice_range[2]) # Testing purpose
						n.move_joints(joint_target31)   # Python API commands that is sent to the Niryoone package to move the Niryoone's joints
						n.move_joints(joint_target32)
						n.move_joints(joint_target33)
						n.move_joints(joint_target34)
						n.move_joints(joint_target35)
						n.move_joints(joint_target36)
						n.move_joints(joint_target37)
						n.move_joints(joint_target38)
						n.move_joints(joint_target39)
						n.move_joints(joint_target310)
						n.move_joints(joint_target311)
						n.move_joints(joint_target312)
						n.move_joints(joint_target313)
						n.move_joints(feeding_pose_joints)
						break
					elif (food_options == 4):
						rospy.loginfo("Plate 4 selected!")
						foodchoicePub.publish(food_choice_range[3]) # Testing purpose
						n.move_joints(joint_target41)   # Python API commands that is sent to the Niryoone package to move the Niryoone's joints
						n.move_joints(joint_target42)
						n.move_joints(joint_target43)
						n.move_joints(joint_target44)
						n.move_joints(joint_target45)
						n.move_joints(joint_target46)
						n.move_joints(joint_target47)
						n.move_joints(joint_target48)
						n.move_joints(joint_target49)
						n.move_joints(joint_target410)
						n.move_joints(joint_target411)
						n.move_joints(joint_target412)
						n.move_joints(joint_target413)
						n.move_joints(joint_target414)
						n.move_joints(feeding_pose_joints)
						break
					elif (food_options == 5):	# Communication with food vision node
						rospy.loginfo("Rice!")
						food_choice.data = 'Rice'
						foodVisionPub.publish(food_choice)
						seconds = 10
						while(seconds > 0):
							foodVision()
							if (food_options == 1):
								 rospy.loginfo("Food is in plate 1!")
								 foodchoicePub.publish(food_choice_range[0]) # Testing purpose
								 n.move_joints(joint_target11)  # Python API commands that is sent to the Niryoone package to move the Niryoone's joints
								 n.move_joints(joint_target12)
								 n.move_joints(joint_target13)
								 n.move_joints(joint_target14)
								 n.move_joints(joint_target15)
								 n.move_joints(joint_target16)
								 n.move_joints(joint_target17)
								 n.move_joints(joint_target18)
								 n.move_joints(joint_target19)
								 n.move_joints(joint_target110)
								 n.move_joints(joint_target111)
								 n.move_joints(feeding_pose_joints)
								 break
							elif (food_options == 2):
								rospy.loginfo("Food is in plate 2!")
								foodchoicePub.publish(food_choice_range[1]) # Testing purpose
								n.move_joints(joint_target21)   # Python API commands that is sent to the Niryoone package to move the Niryoone's joints
								n.move_joints(joint_target22)
								n.move_joints(joint_target23)
								n.move_joints(joint_target24)
								n.move_joints(joint_target25)
								n.move_joints(joint_target26)
								n.move_joints(joint_target27)
								n.move_joints(joint_target28)
								n.move_joints(joint_target29)
								n.move_joints(joint_target210)
								n.move_joints(feeding_pose_joints)
								break
							elif (food_options == 3):
								rospy.loginfo("Food is in plate 3!")
								foodchoicePub.publish(food_choice_range[2]) # Testing purpose
								n.move_joints(joint_target31)   # Python API commands that is sent to the Niryoone package to move the Niryoone's joints
								n.move_joints(joint_target32)
								n.move_joints(joint_target33)
								n.move_joints(joint_target34)
								n.move_joints(joint_target35)
								n.move_joints(joint_target36)
								n.move_joints(joint_target37)
								n.move_joints(joint_target38)
								n.move_joints(joint_target39)
								n.move_joints(joint_target310)
								n.move_joints(joint_target311)
								n.move_joints(joint_target312)
								n.move_joints(joint_target313)
								n.move_joints(feeding_pose_joints)
								break
							elif (food_options == 4):
								rospy.loginfo("Food is in plate 4!")
								foodchoicePub.publish(food_choice_range[3]) # Testing purpose
								n.move_joints(joint_target41)   # Python API commands that is sent to the Niryoone package to move the Niryoone's joints
								n.move_joints(joint_target42)
								n.move_joints(joint_target43)
								n.move_joints(joint_target44)
								n.move_joints(joint_target45)
								n.move_joints(joint_target46)
								n.move_joints(joint_target47)
								n.move_joints(joint_target48)
								n.move_joints(joint_target49)
								n.move_joints(joint_target410)
								n.move_joints(joint_target411)
								n.move_joints(joint_target412)
								n.move_joints(joint_target413)
								n.move_joints(joint_target414)
								n.move_joints(feeding_pose_joints)
								break
							elif (food_options == 0):
								rospy.loginfo_throttle(1,"Food not detected in any bowl!Awaiting reselection...")
								break
							else:
								seconds-=1
								time.sleep(1)
								print("Waiting for vision node to publish plate number...%s seconds left...",seconds)


				move_on = False	# Ensure food select loop will run in the next iteration


				pin1 = GPIO_1A	# Water button pin
				pin2 = GPIO_1B  # New food button pin
				pin3 = GPIO_1C  # Old food button pin
				n.pin_mode(pin1, PIN_MODE_INPUT)
				n.pin_mode(pin2, PIN_MODE_INPUT)
				n.pin_mode(pin3, PIN_MODE_INPUT)
				input1 = n.digital_read(pin1)
				input2 = n.digital_read(pin2)
				input3 = n.digital_read(pin3)
				seconds = 15 # Time left before picking up old food again

				while ( seconds > 0 ):
					input1 = n.digital_read(pin1)
					input2 = n.digital_read(pin2)
					input3 = n.digital_read(pin3)
					if (input1 == PIN_LOW):	# Engage water nozzle
						while (input1 == PIN_LOW):
							n.close_gripper(TOOL_GRIPPER_1_ID, 50)  # Retracting of spoon
							# n.move_joints(water_nozzle_offset_forward)
							rospy.loginfo("Water!")
							time.sleep(5)
							input1 = n.digital_read(pin1)
						n.open_gripper(TOOL_GRIPPER_1_ID, 50)   # Extending of spoon
						# n.move_joints(water_nozzle_offset_backward)
						seconds = 15 # Reset timer
					elif (input2 == PIN_LOW):	# New food signal
						new_food_signal = True
						food_options = 0
						seconds = 0
						rospy.loginfo("New food!")
					elif (input3 == PIN_LOW):	# Old food signal
						new_food_signal = False
						rospy.loginfo("Old food!")
						seconds = 0
					# elif (input4 == PIN_LOW):	# Recalibrate feeding pose location
					# 	new_food_signal = True
					# 	food_options = 0
					# 	seconds = 0
					# 	recalibration_move_on = False
					else:
						rospy.loginfo("Time left to re selecting old food is %s seconds",seconds)
						time.sleep(1)
						new_food_signal = False
						seconds-=1


			elif (new_food_signal == False):    # Loop that will run if new_food_signal is False, which will only be possible to run from the second loop onwards
				rospy.loginfo_throttle(5,"Same food will be served again!")
				if (food_options == 1):
					rospy.loginfo("Plate 1 selected!")
					foodchoicePub.publish(food_choice_range[0]) # Testing purpose
					n.move_joints(joint_target11)   # Python API commands that is sent to the Niryoone package to move the Niryoone's joints
					n.move_joints(joint_target12)
					n.move_joints(joint_target13)
					n.move_joints(joint_target14)
					n.move_joints(joint_target15)
					n.move_joints(joint_target16)
					n.move_joints(joint_target17)
					n.move_joints(joint_target18)
					n.move_joints(joint_target19)
					n.move_joints(joint_target110)
					n.move_joints(joint_target111)
					n.move_joints(feeding_pose_joints)
				elif (food_options == 2):
					rospy.loginfo("Plate 2 selected!")
					foodchoicePub.publish(food_choice_range[1]) # Testing purpose
					n.move_joints(joint_target21)   # Python API commands that is sent to the Niryoone package to move the Niryoone's joints
					n.move_joints(joint_target22)
					n.move_joints(joint_target23)
					n.move_joints(joint_target24)
					n.move_joints(joint_target25)
					n.move_joints(joint_target26)
					n.move_joints(joint_target27)
					n.move_joints(joint_target28)
					n.move_joints(joint_target29)
					n.move_joints(joint_target210)
					n.move_joints(feeding_pose_joints)
				elif (food_options == 3):
					rospy.loginfo("Plate 3 selected!")
					foodchoicePub.publish(food_choice_range[2]) # Testing purpose
					n.move_joints(joint_target31)   # Python API commands that is sent to the Niryoone package to move the Niryoone's joints
					n.move_joints(joint_target32)
					n.move_joints(joint_target33)
					n.move_joints(joint_target34)
					n.move_joints(joint_target35)
					n.move_joints(joint_target36)
					n.move_joints(joint_target37)
					n.move_joints(joint_target38)
					n.move_joints(joint_target39)
					n.move_joints(joint_target310)
					n.move_joints(joint_target311)
					n.move_joints(joint_target312)
					n.move_joints(joint_target313)
					n.move_joints(joint_target314)
					n.move_joints(feeding_pose_joints)
				elif (food_options == 4):
					rospy.loginfo("Plate 4 selected!")
					foodchoicePub.publish(food_choice_range[3]) # Testing purpose
					n.move_joints(joint_target41)   # Python API commands that is sent to the Niryoone package to move the Niryoone's joints
					n.move_joints(joint_target42)
					n.move_joints(joint_target43)
					n.move_joints(joint_target44)
					n.move_joints(joint_target45)
					n.move_joints(joint_target46)
					n.move_joints(joint_target47)
					n.move_joints(joint_target48)
					n.move_joints(joint_target49)
					n.move_joints(joint_target410)
					n.move_joints(joint_target411)
					n.move_joints(joint_target412)
					n.move_joints(joint_target413)
					n.move_joints(joint_target414)
					n.move_joints(feeding_pose_joints)
				move_on = False	# Ensure food select loop will run in the next tieration


				pin1 = GPIO_1A	# Declaration of all pins on the Niryoone RPI
				pin2 = GPIO_1B
				pin3 = GPIO_1C
				n.pin_mode(pin1, PIN_MODE_INPUT)
				n.pin_mode(pin2, PIN_MODE_INPUT)
				n.pin_mode(pin3, PIN_MODE_INPUT)
				input1 = n.digital_read(pin1)
				input2 = n.digital_read(pin2)
				input3 = n.digital_read(pin3)
				seconds = 15

				while ( seconds > 0 ):
					input1 = n.digital_read(pin1)	# Checking for button inputs
					input2 = n.digital_read(pin2)
					input3 = n.digital_read(pin3)
					if (input1 == PIN_LOW):	# Engage water nozzle
						while (input1 == PIN_LOW):
							n.close_gripper(TOOL_GRIPPER_1_ID, 200) # Retracting of spoon
							# n.shift_pose(water_nozzle_offset_forward)
							rospy.loginfo("Water!")
							input1 = n.digital_read(pin1)
						n.open_gripper(TOOL_GRIPPER_1_ID, 200)  # Extending of spoon
						# n.shift_pose(water_nozzle_offset_backward)
						seconds = 15 # Reset timer
					elif (input2 == PIN_LOW):	# New food signal
						new_food_signal = True
						food_options = 0
						seconds = 0
						rospy.loginfo("New food!")
					elif (input3 == PIN_LOW):	# Old food signal
						new_food_signal = False
						rospy.loginfo("Old food!")
						seconds = 0
					# elif (input4 == PIN_LOW):	# Recalibrate feeding pose location
					# 	new_food_signal = True
					# 	food_options = 0
					# 	seconds = 0
					# 	recalibration_move_on = False
					else:
						rospy.loginfo("Time left to re-selecting old food is %s seconds",seconds)
						time.sleep(1)
						new_food_signal = False
						seconds-=1
			rate.sleep()
	except rospy.ROSInterruptException:
		rospy.on_shutdown()
		n.learning_mode(True)