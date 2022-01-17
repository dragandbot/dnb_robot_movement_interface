#!/usr/bin/python3

#TOOD: incremental_joints to be added and tested in the future as currently its not working

PKG = 'dnb_tf'
NAME = 'autotest_dnb_tf'

import roslib; roslib.load_manifest(PKG)
import sys
import tf
import time
import json                 # Json API
import rospy                # ROS API
import rostest				# ROS specific testing library
import unittest				# Python generic testing library used by ROS
from dnb_tf.srv import *
from robot_movement_interface.msg import *
import geometry_msgs.msg 


class UnitTest(unittest.TestCase):
        

	def test_ptp_joints(self):	# Point To Point motion with Joints values
		
		rospy.wait_for_service('/transform_commandlist')
		client = rospy.ServiceProxy('/transform_commandlist', Transform)

		req = TransformRequest()

		req.input_commandlist.commands.append(Command())
		req.input_commandlist.commands[0].command_type = "PTP"
		req.input_commandlist.commands[0].pose_type = "JOINTS"
		req.input_commandlist.commands[0].pose = [0.1,0.5,0.2,0.15,0,0.1] # 6-Array (Joints) 
		req.input_commandlist.commands[0].velocity_type = "RAD/S"
		req.input_commandlist.commands[0].velocity = [0.1] # 1-Array
		req.input_commandlist.commands[0].acceleration_type = "RAD/S^2"
		req.input_commandlist.commands[0].acceleration = [0.1] # 1-Array
		req.input_commandlist.commands[0].blending_type = "M"
		req.input_commandlist.commands[0].blending = [0.04]
		req.input_commandlist.replace_previous_commands = True

		response = client(req)

		assert ((response.output_commandlist.commands[0].command_type) == "PTP")
		assert ((response.output_commandlist.commands[0].pose_reference) == "")
		assert ((response.output_commandlist.commands[0].pose_reference_frame.x) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.y) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.z) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.alpha) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.beta) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.gamma) == 0)
		assert ((response.output_commandlist.commands[0].pose_type) == "JOINTS")
		assert ((response.output_commandlist.commands[0].pose[0]) == 0.10000000149011612)
		assert ((response.output_commandlist.commands[0].pose[1]) == 0.5)
		assert (round(response.output_commandlist.commands[0].pose[2],5) == 0.2)
		assert (round(response.output_commandlist.commands[0].pose[3],5) == 0.15)
		assert (round(response.output_commandlist.commands[0].pose[4],3) == 0)
		assert ((response.output_commandlist.commands[0].pose[5]) ==  0.10000000149011612)
		assert ((response.output_commandlist.commands[0].velocity_type) == "RAD/S")
		assert (round(response.output_commandlist.commands[0].velocity[0],5) == 0.1) # 1-Array
		assert ((response.output_commandlist.commands[0].acceleration_type) == "RAD/S^2")
		assert (round(response.output_commandlist.commands[0].acceleration[0],5) == 0.1) # 1-Array
		assert ((response.output_commandlist.commands[0].effort_type) == "")
		assert (not(response.output_commandlist.commands[0].effort) )
		assert ((response.output_commandlist.commands[0].blending_type) == "M")
		assert ((response.output_commandlist.commands[0].blending[0]) == 0.03999999910593033)
		assert (not(response.output_commandlist.commands[0].additional_parameters))
		assert (not(response.output_commandlist.commands[0].additional_values) )
		assert ((response.output_commandlist.replace_previous_commands) == True)


	def test_ptp_quaternion_given_ref(self):	# Point To Point motion with Quaternion with given frame as ref.
		
		rospy.wait_for_service('/transform_commandlist')
		client = rospy.ServiceProxy('/transform_commandlist', Transform)

		req = TransformRequest()

		req.input_commandlist.commands.append(Command())
		req.input_commandlist.commands[0].command_type = "PTP"
		req.input_commandlist.commands[0].pose_type = "QUATERNION"
		req.input_commandlist.commands[0].pose_reference_frame.x = 1.57
		req.input_commandlist.commands[0].pose_reference_frame.y = 5.47
		req.input_commandlist.commands[0].pose_reference_frame.z = 0.8
		req.input_commandlist.commands[0].pose_reference_frame.alpha = 4.87
		req.input_commandlist.commands[0].pose_reference_frame.beta = 3.45
		req.input_commandlist.commands[0].pose_reference_frame.gamma = 0.89
		req.input_commandlist.commands[0].pose = [0.1, -0.4, 0.2, 3.14, 0.0, 3.14] # 7-Array (Frame) x,y,z,w,a,b,c
		req.input_commandlist.commands[0].velocity_type = "RAD/S"
		req.input_commandlist.commands[0].velocity = [0.1] # 1-Array
		req.input_commandlist.commands[0].acceleration_type = "RAD/S^2"
		req.input_commandlist.commands[0].acceleration = [0.1] # 1-Array
		req.input_commandlist.commands[0].blending_type = "M"
		req.input_commandlist.commands[0].blending = [0]
		req.input_commandlist.replace_previous_commands = True

		response = client(req)

		assert ((response.output_commandlist.commands[0].command_type) == "PTP")
		assert ((response.output_commandlist.commands[0].pose_reference) == "")
		assert (round(response.output_commandlist.commands[0].pose_reference_frame.x,5) == 1.57)
		assert (round(response.output_commandlist.commands[0].pose_reference_frame.y,0) == 5)
		assert (round(response.output_commandlist.commands[0].pose_reference_frame.z,5) == 0.8)
		assert (round(response.output_commandlist.commands[0].pose_reference_frame.alpha,2) == 4.87)
		assert (round(response.output_commandlist.commands[0].pose_reference_frame.beta,5) == 3.45)
		assert (round(response.output_commandlist.commands[0].pose_reference_frame.gamma,2) == 0.89)
		assert ((response.output_commandlist.commands[0].pose_type) == "QUATERNION")
		assert ((response.output_commandlist.commands[0].pose[0]) == 1.1617239713668823)
		assert ((response.output_commandlist.commands[0].pose[1]) == 5.444746971130371)
		assert ((response.output_commandlist.commands[0].pose[2]) == 1.0065743923187256)
		assert ((response.output_commandlist.commands[0].pose[3]) == -1.412132978439331)
		assert ((response.output_commandlist.commands[0].pose[4]) == 0.30964475870132446)
		assert ((response.output_commandlist.commands[0].pose[5]) ==  -0.8912724852561951)
		assert ((response.output_commandlist.commands[0].velocity_type) == "RAD/S")
		assert (round(response.output_commandlist.commands[0].velocity[0],5) == 0.1) # 1-Array
		assert ((response.output_commandlist.commands[0].acceleration_type) == "RAD/S^2")
		assert (round(response.output_commandlist.commands[0].acceleration[0],5) == 0.1) # 1-Array
		assert ((response.output_commandlist.commands[0].effort_type) == "")
		assert (not(response.output_commandlist.commands[0].effort) )
		assert ((response.output_commandlist.commands[0].blending_type) == "M")
		assert ((response.output_commandlist.commands[0].blending[0]) == 0)
		assert (not(response.output_commandlist.commands[0].additional_parameters))
		assert (not(response.output_commandlist.commands[0].additional_values) )
		assert ((response.output_commandlist.replace_previous_commands) == True)	


	def test_ptp_euler(self):	# Point To Point motion with Euler
		
		rospy.wait_for_service('/transform_commandlist')
		client = rospy.ServiceProxy('/transform_commandlist', Transform)

		req = TransformRequest()

		req.input_commandlist.commands.append(Command())
		req.input_commandlist.commands[0].command_type = "PTP"
		req.input_commandlist.commands[0].pose_type = "EULER_INTRINSIC_ZYX"
		req.input_commandlist.commands[0].pose = [0.1, -0.4, 0.2, 3.14, 0.0, 3.14] # 6-Array (Frame) x,y,z,a,b,c
		req.input_commandlist.commands[0].velocity_type = "RAD/S"
		req.input_commandlist.commands[0].velocity = [0.1] # 1-Array
		req.input_commandlist.commands[0].acceleration_type = "RAD/S^2"
		req.input_commandlist.commands[0].acceleration = [0.1] # 1-Array
		req.input_commandlist.commands[0].blending_type = "M"
		req.input_commandlist.commands[0].blending = [0.02]
		req.input_commandlist.replace_previous_commands = True

		response = client(req)

		assert ((response.output_commandlist.commands[0].command_type) == "PTP")
		assert ((response.output_commandlist.commands[0].pose_reference) == "")
		assert ((response.output_commandlist.commands[0].pose_reference_frame.x) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.y) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.z) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.alpha) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.beta) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.gamma) == 0)
		assert ((response.output_commandlist.commands[0].pose_type) == "EULER_INTRINSIC_ZYX")
		assert ((response.output_commandlist.commands[0].pose[0]) == 0.10000000149011612)
		assert ((response.output_commandlist.commands[0].pose[1]) == -0.4000000059604645)
		assert ((response.output_commandlist.commands[0].pose[2]) == 0.20000000298023224)
		assert ((response.output_commandlist.commands[0].pose[3]) == 3.140000104904175)
		assert ((response.output_commandlist.commands[0].pose[4]) == -4.235164736271502e-22)
		assert ((response.output_commandlist.commands[0].pose[5]) ==  3.140000104904175)
		assert ((response.output_commandlist.commands[0].velocity_type) == "RAD/S")
		assert (round(response.output_commandlist.commands[0].velocity[0],5) == 0.1) # 1-Array
		assert ((response.output_commandlist.commands[0].acceleration_type) == "RAD/S^2")
		assert (round(response.output_commandlist.commands[0].acceleration[0],5) == 0.1) # 1-Array
		assert ((response.output_commandlist.commands[0].effort_type) == "")
		assert (not(response.output_commandlist.commands[0].effort) )
		assert ((response.output_commandlist.commands[0].blending_type) == "M")
		assert ((response.output_commandlist.commands[0].blending[0]) == 0.019999999552965164)
		assert (not(response.output_commandlist.commands[0].additional_parameters))
		assert (not(response.output_commandlist.commands[0].additional_values) )
		assert ((response.output_commandlist.replace_previous_commands) == True)		

	
	def test_lineal_time(self):	# Lineal motion with time Constrained

		rospy.wait_for_service('/transform_commandlist')
		client = rospy.ServiceProxy('/transform_commandlist', Transform)

		req = TransformRequest()

		req.input_commandlist.commands.append(Command())
		req.input_commandlist.commands[0].command_type = "LIN_TIMED"
		req.input_commandlist.commands[0].pose_type = "JOINTS"
		req.input_commandlist.commands[0].pose = [1,3,2,9,1.78,8.59] # 6-Array (Joints) 
		req.input_commandlist.commands[0].blending_type = "M"
		req.input_commandlist.commands[0].blending = [0.5]
		req.input_commandlist.commands[0].additional_values = [5.4875] # 1-Array (Sec.)
		req.input_commandlist.replace_previous_commands = True

		response = client(req)

		assert ((response.output_commandlist.commands[0].command_type) == "LIN_TIMED")
		assert ((response.output_commandlist.commands[0].pose_reference) == "")
		assert ((response.output_commandlist.commands[0].pose_reference_frame.x) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.y) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.z) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.alpha) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.beta) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.gamma) == 0)
		assert ((response.output_commandlist.commands[0].pose_type) == "JOINTS")
		assert ((response.output_commandlist.commands[0].pose[0]) == 1)
		assert ((response.output_commandlist.commands[0].pose[1]) == 3)
		assert ((response.output_commandlist.commands[0].pose[2]) == 2)
		assert ((response.output_commandlist.commands[0].pose[3]) == 9)
		assert ((response.output_commandlist.commands[0].pose[4]) == 1.7799999713897705)
		assert ((response.output_commandlist.commands[0].pose[5]) ==  8.59000015258789)
		assert ((response.output_commandlist.commands[0].velocity_type) == "")
		assert (not(response.output_commandlist.commands[0].velocity)) # 1-Array
		assert ((response.output_commandlist.commands[0].acceleration_type) == "")
		assert (not(response.output_commandlist.commands[0].acceleration)) # 1-Array
		assert ((response.output_commandlist.commands[0].effort_type) == "")
		assert (not(response.output_commandlist.commands[0].effort) )
		assert ((response.output_commandlist.commands[0].blending_type) == "M")
		assert ((response.output_commandlist.commands[0].blending[0]) == 0.5)
		assert (not(response.output_commandlist.commands[0].additional_parameters) )
		assert (round(response.output_commandlist.commands[0].additional_values[0],5) == 5.4875)
		assert ((response.output_commandlist.replace_previous_commands) == True)		


	def test_lineal_joints_base_ref(self):	# Lineal motion with Joints values and base link as ref. frame

		rospy.wait_for_service('/transform_commandlist')
		client = rospy.ServiceProxy('/transform_commandlist', Transform)

		req = TransformRequest()

		req.input_commandlist.commands.append(Command())
		req.input_commandlist.commands[0].command_type = "LIN"
		req.input_commandlist.commands[0].pose_type = "JOINTS"
		req.input_commandlist.commands[0].pose = [0.1,2.8,0.2,2,3,5.8] # 6-Array (Joints) 
		req.input_commandlist.commands[0].velocity_type = "M/S"
		req.input_commandlist.commands[0].velocity = [0.2] # 1-Array
		req.input_commandlist.commands[0].acceleration_type = "M/S^2"
		req.input_commandlist.commands[0].acceleration = [0.1] # 1-Array
		req.input_commandlist.commands[0].blending_type = "M"
		req.input_commandlist.commands[0].blending = [0]
		req.input_commandlist.replace_previous_commands = True

		response = client(req)

		assert ((response.output_commandlist.commands[0].command_type) == "LIN")
		assert ((response.output_commandlist.commands[0].pose_reference) == "")
		assert ((response.output_commandlist.commands[0].pose_reference_frame.x) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.y) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.z) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.alpha) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.beta) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.gamma) == 0)
		assert ((response.output_commandlist.commands[0].pose_type) == "JOINTS")
		assert ((response.output_commandlist.commands[0].pose[0]) == 0.10000000149011612)
		assert ((response.output_commandlist.commands[0].pose[1]) == 2.799999952316284)
		assert ((response.output_commandlist.commands[0].pose[2]) == 0.20000000298023224)
		assert ((response.output_commandlist.commands[0].pose[3]) == 2)
		assert ((response.output_commandlist.commands[0].pose[4]) == 3)
		assert ((response.output_commandlist.commands[0].pose[5]) ==  5.800000190734863)
		assert ((response.output_commandlist.commands[0].velocity_type) == "M/S")
		assert (round(response.output_commandlist.commands[0].velocity[0],5) == 0.2) # 1-Array
		assert ((response.output_commandlist.commands[0].acceleration_type) == "M/S^2")
		assert (round(response.output_commandlist.commands[0].acceleration[0],5) == 0.1) # 1-Array
		assert ((response.output_commandlist.commands[0].effort_type) == "")
		assert (not(response.output_commandlist.commands[0].effort) )
		assert ((response.output_commandlist.commands[0].blending_type) == "M")
		assert ((response.output_commandlist.commands[0].blending[0]) == 0)
		assert (not(response.output_commandlist.commands[0].additional_parameters) )
		assert (not(response.output_commandlist.commands[0].additional_values))
		assert ((response.output_commandlist.replace_previous_commands) == True)


	def test_lineal_quaternion_base_ref(self):	# Lineal motion with Quaternion and base link as ref. frame

		rospy.wait_for_service('/transform_commandlist')
		client = rospy.ServiceProxy('/transform_commandlist', Transform)

		req = TransformRequest()

		req.input_commandlist.commands.append(Command())
		req.input_commandlist.commands[0].command_type = "LIN"
		req.input_commandlist.commands[0].pose_type = "QUATERNION"
		req.input_commandlist.commands[0].pose = [0.1,0.1,0.2,3.14,0.8,1.5] # 7-Array (Frame) x,y,z,w,a,b,c
		req.input_commandlist.commands[0].velocity_type = "M/S"
		req.input_commandlist.commands[0].velocity = [0.2] # 1-Array
		req.input_commandlist.commands[0].acceleration_type = "M/S^2"
		req.input_commandlist.commands[0].acceleration = [0.1] # 1-Array
		req.input_commandlist.commands[0].blending_type = "M"
		req.input_commandlist.commands[0].blending = [0]
		req.input_commandlist.replace_previous_commands = True

		response = client(req)

		assert ((response.output_commandlist.commands[0].command_type) == "LIN")
		assert ((response.output_commandlist.commands[0].pose_reference) == "")
		assert ((response.output_commandlist.commands[0].pose_reference_frame.x) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.y) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.z) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.alpha) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.beta) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.gamma) == 0)
		assert ((response.output_commandlist.commands[0].pose_type) == "QUATERNION")
		assert ((response.output_commandlist.commands[0].pose[0]) == 0.10000000149011612)
		assert ((response.output_commandlist.commands[0].pose[1]) == 0.10000000149011612)
		assert ((response.output_commandlist.commands[0].pose[2]) == 0.20000000298023224)
		assert ((response.output_commandlist.commands[0].pose[3]) == 3.140000104904175)
		assert ((response.output_commandlist.commands[0].pose[4]) == 0.800000011920929)
		assert ((response.output_commandlist.commands[0].pose[5]) ==  1.5)
		assert ((response.output_commandlist.commands[0].velocity_type) == "M/S")
		assert (round(response.output_commandlist.commands[0].velocity[0],5) == 0.2) # 1-Array
		assert ((response.output_commandlist.commands[0].acceleration_type) == "M/S^2")
		assert (round(response.output_commandlist.commands[0].acceleration[0],5) == 0.1) # 1-Array
		assert ((response.output_commandlist.commands[0].effort_type) == "")
		assert (not(response.output_commandlist.commands[0].effort) )
		assert ((response.output_commandlist.commands[0].blending_type) == "M")
		assert ((response.output_commandlist.commands[0].blending[0]) == 0)
		assert (not(response.output_commandlist.commands[0].additional_parameters) )
		assert (not(response.output_commandlist.commands[0].additional_values))
		assert ((response.output_commandlist.replace_previous_commands) == True)


	def test_lineal_quaternion_given_ref(self):	# Lineal motion with Quaternion and given frame as ref. frame

		rospy.wait_for_service('/transform_commandlist')
		client = rospy.ServiceProxy('/transform_commandlist', Transform)

		req = TransformRequest()

		req.input_commandlist.commands.append(Command())
		req.input_commandlist.commands[0].command_type = "LIN"
		req.input_commandlist.commands[0].pose_reference == ""
		req.input_commandlist.commands[0].pose_reference_frame.x = 2
		req.input_commandlist.commands[0].pose_reference_frame.y = 0.5
		req.input_commandlist.commands[0].pose_reference_frame.z = 4
		req.input_commandlist.commands[0].pose_reference_frame.alpha = 3.14
		req.input_commandlist.commands[0].pose_reference_frame.beta = 0.5
		req.input_commandlist.commands[0].pose_reference_frame.gamma = 2
		req.input_commandlist.commands[0].pose_type = "QUATERNION"
		req.input_commandlist.commands[0].pose = [0.1,0.1,0.2,3.14,0.8,1.5] # 7-Array (Frame) x,y,z,w,a,b,c
		req.input_commandlist.commands[0].velocity_type = "M/S"
		req.input_commandlist.commands[0].velocity = [0.2] # 1-Array
		req.input_commandlist.commands[0].acceleration_type = "M/S^2"
		req.input_commandlist.commands[0].acceleration = [0.1] # 1-Array
		req.input_commandlist.commands[0].blending_type = "M"
		req.input_commandlist.commands[0].blending = [0]
		req.input_commandlist.replace_previous_commands = True

		response = client(req)

		assert ((response.output_commandlist.commands[0].command_type) == "LIN")
		assert ((response.output_commandlist.commands[0].pose_reference) == "")
		assert ((response.output_commandlist.commands[0].pose_reference_frame.x) == 2)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.y) == 0.5)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.z) == 4)
		assert (round(response.output_commandlist.commands[0].pose_reference_frame.alpha,5) == 3.14)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.beta) == 0.5)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.gamma) == 2)
		assert ((response.output_commandlist.commands[0].pose_type) == "QUATERNION")
		assert ((response.output_commandlist.commands[0].pose[0]) == 1.9089059829711914)
		assert ((response.output_commandlist.commands[0].pose[1]) == 0.7236195206642151)
		assert ((response.output_commandlist.commands[0].pose[2]) == 3.958815097808838)
		assert ((response.output_commandlist.commands[0].pose[3]) == -0.9498870372772217)
		assert ((response.output_commandlist.commands[0].pose[4]) == -0.6396126747131348)
		assert ((response.output_commandlist.commands[0].pose[5]) ==  0.04210380092263222)
		assert ((response.output_commandlist.commands[0].velocity_type) == "M/S")
		assert (round(response.output_commandlist.commands[0].velocity[0],5) == 0.2) # 1-Array
		assert ((response.output_commandlist.commands[0].acceleration_type) == "M/S^2")
		assert (round(response.output_commandlist.commands[0].acceleration[0],5) == 0.1) # 1-Array
		assert ((response.output_commandlist.commands[0].effort_type) == "")
		assert (not(response.output_commandlist.commands[0].effort) )
		assert ((response.output_commandlist.commands[0].blending_type) == "M")
		assert ((response.output_commandlist.commands[0].blending[0]) == 0)
		assert (not(response.output_commandlist.commands[0].additional_parameters) )
		assert (not(response.output_commandlist.commands[0].additional_values))
		assert ((response.output_commandlist.replace_previous_commands) == True)

	
	def test_lineal_euler_base_ref(self):	# Lineal motion with Euler Intrinsic ZYX and base link as ref. frame

		rospy.wait_for_service('/transform_commandlist')
		client = rospy.ServiceProxy('/transform_commandlist', Transform)

		req = TransformRequest()

		req.input_commandlist.commands.append(Command())
		req.input_commandlist.commands[0].command_type = "LIN"
		req.input_commandlist.commands[0].pose_type = "EULER_INTRINSIC_ZYX"
		req.input_commandlist.commands[0].pose = [0.1, -0.4, 0.2, 3.14, 0.0, 3.14] # 6-Array (Frame) x,y,z,a,b,c
		req.input_commandlist.commands[0].velocity_type = "M/S"
		req.input_commandlist.commands[0].velocity = [0.2] # 1-Array
		req.input_commandlist.commands[0].acceleration_type = "M/S^2"
		req.input_commandlist.commands[0].acceleration = [0.1] # 1-Array
		req.input_commandlist.commands[0].blending_type = "M"
		req.input_commandlist.commands[0].blending = [0]
		req.input_commandlist.replace_previous_commands = True

		response = client(req)

		assert ((response.output_commandlist.commands[0].command_type) == "LIN")
		assert ((response.output_commandlist.commands[0].pose_reference) == "")
		assert ((response.output_commandlist.commands[0].pose_reference_frame.x) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.y) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.z) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.alpha) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.beta) == 0)
		assert ((response.output_commandlist.commands[0].pose_reference_frame.gamma) == 0)
		assert ((response.output_commandlist.commands[0].pose_type) == "EULER_INTRINSIC_ZYX")
		assert ((response.output_commandlist.commands[0].pose[0]) == 0.10000000149011612)
		assert ((response.output_commandlist.commands[0].pose[1]) == -0.4000000059604645)
		assert ((response.output_commandlist.commands[0].pose[2]) == 0.20000000298023224)
		assert ((response.output_commandlist.commands[0].pose[3]) == 3.140000104904175)
		assert ((response.output_commandlist.commands[0].pose[4]) == -4.235164736271502e-22)
		assert ((response.output_commandlist.commands[0].pose[5]) ==  3.140000104904175)
		assert ((response.output_commandlist.commands[0].velocity_type) == "M/S")
		assert (round(response.output_commandlist.commands[0].velocity[0],5) == 0.2) # 1-Array
		assert ((response.output_commandlist.commands[0].acceleration_type) == "M/S^2")
		assert (round(response.output_commandlist.commands[0].acceleration[0],5) == 0.1) # 1-Array
		assert ((response.output_commandlist.commands[0].effort_type) == "")
		assert (not(response.output_commandlist.commands[0].effort) )
		assert ((response.output_commandlist.commands[0].blending_type) == "M")
		assert ((response.output_commandlist.commands[0].blending[0]) == 0)
		assert (not(response.output_commandlist.commands[0].additional_parameters) )
		assert (not(response.output_commandlist.commands[0].additional_values))
		assert ((response.output_commandlist.replace_previous_commands) == True)


	def test_lineal_euler_given_ref(self):	# Lineal motion with Euler Intrinsic ZYX and given frame as ref. frame

		rospy.wait_for_service('/transform_commandlist')
		client = rospy.ServiceProxy('/transform_commandlist', Transform)

		req = TransformRequest()

		req.input_commandlist.commands.append(Command())
		req.input_commandlist.commands[0].command_type = "LIN"
		req.input_commandlist.commands[0].pose_reference = "frame_a"
		req.input_commandlist.commands[0].pose_reference_frame.x = 0.1
		req.input_commandlist.commands[0].pose_reference_frame.y = 0.5
		req.input_commandlist.commands[0].pose_reference_frame.z = -0.1
		req.input_commandlist.commands[0].pose_reference_frame.alpha = 0.8
		req.input_commandlist.commands[0].pose_reference_frame.beta = 0
		req.input_commandlist.commands[0].pose_reference_frame.gamma = 3.14
		req.input_commandlist.commands[0].pose_type = "EULER_INTRINSIC_ZYX"
		req.input_commandlist.commands[0].pose = [0.1, -0.4, 0.2, 3.14, 0.5, 3.14] # 6-Array (Frame) x,y,z,a,b,c
		req.input_commandlist.commands[0].velocity_type = "M/S"
		req.input_commandlist.commands[0].velocity = [0.3] # 1-Array
		req.input_commandlist.commands[0].acceleration_type = "M/S^2"
		req.input_commandlist.commands[0].acceleration = [0.2] # 1-Array
		req.input_commandlist.commands[0].blending_type = "M"
		req.input_commandlist.commands[0].blending = [0]
		req.input_commandlist.replace_previous_commands = True

		response = client(req)

		assert ((response.output_commandlist.commands[0].command_type) == "LIN")
		assert ((response.output_commandlist.commands[0].pose_reference) == "frame_a")
		assert (round(response.output_commandlist.commands[0].pose_reference_frame.x,5) == 0.10000)
		assert (round(response.output_commandlist.commands[0].pose_reference_frame.y,5) == 0.5)
		assert (round(response.output_commandlist.commands[0].pose_reference_frame.z,5) == -0.10000)
		assert (round(response.output_commandlist.commands[0].pose_reference_frame.alpha,5) == 0.80000)
		assert (round(response.output_commandlist.commands[0].pose_reference_frame.beta,5) == 0)
		assert (round(response.output_commandlist.commands[0].pose_reference_frame.gamma,5) == 3.14000)
		assert ((response.output_commandlist.commands[0].pose_type) == "EULER_INTRINSIC_ZYX")
		assert ((response.output_commandlist.commands[0].pose[0]) == -0.1170429214835167)
		assert ((response.output_commandlist.commands[0].pose[1]) == 0.8501960635185242)
		assert ((response.output_commandlist.commands[0].pose[2]) == -0.30063676834106445)
		assert ((response.output_commandlist.commands[0].pose[3]) == -2.3408701419830322)
		assert ((response.output_commandlist.commands[0].pose[4]) == -0.500001847743988)
		assert ((response.output_commandlist.commands[0].pose[5]) ==  0.0002221506292698905)
		assert ((response.output_commandlist.commands[0].velocity_type) == "M/S")
		assert ((response.output_commandlist.commands[0].velocity[0]) == 0.30000001192092896) # 1-Array
		assert ((response.output_commandlist.commands[0].acceleration_type) == "M/S^2")
		assert ((response.output_commandlist.commands[0].acceleration[0]) == 0.20000000298023224) # 1-Array
		assert ((response.output_commandlist.commands[0].effort_type) == "")
		assert (not(response.output_commandlist.commands[0].effort) )
		assert ((response.output_commandlist.commands[0].blending_type) == "M")
		assert ((response.output_commandlist.commands[0].blending[0]) == 0)
		assert (not(response.output_commandlist.commands[0].additional_parameters) )
		assert (not(response.output_commandlist.commands[0].additional_values))
		assert ((response.output_commandlist.replace_previous_commands) == True)


	def test_general(self):	# Lineal motion with Euler Intrinsic ZYX and given frame as ref. frame

		rospy.wait_for_service('/transform_commandlist')
		client = rospy.ServiceProxy('/transform_commandlist', Transform)

		req = TransformRequest()

		req.input_commandlist.commands.append(Command())
		req.input_commandlist.commands[0].command_type = "LIN"
		req.input_commandlist.commands[0].pose_reference = "frame_a"
		req.input_commandlist.commands[0].pose_reference_frame.x = 0.1
		req.input_commandlist.commands[0].pose_reference_frame.y = 0.5
		req.input_commandlist.commands[0].pose_reference_frame.z = -0.1
		req.input_commandlist.commands[0].pose_reference_frame.alpha = 0.8
		req.input_commandlist.commands[0].pose_reference_frame.beta = 0
		req.input_commandlist.commands[0].pose_reference_frame.gamma = 3.14
		req.input_commandlist.commands[0].pose_type = "EULER_INTRINSIC_ZYX"
		req.input_commandlist.commands[0].pose = [0.1, -0.4, 0.2, 3.14, 0.5, 3.14] # 6-Array (Frame) x,y,z,a,b,c
		req.input_commandlist.commands[0].velocity_type = "M/S"
		req.input_commandlist.commands[0].velocity = [0.3,0.5,0.154] # 1-Array
		req.input_commandlist.commands[0].acceleration_type = "M/S^2"
		req.input_commandlist.commands[0].acceleration = [0.2] # 1-Array
		req.input_commandlist.commands[0].effort_type = "EFFORT" 
		req.input_commandlist.commands[0].effort = [5] 
		req.input_commandlist.commands[0].blending_type = "M"
		req.input_commandlist.commands[0].blending = [1.5,0.2]
		req.input_commandlist.commands[0].additional_parameters = ['item1','item2']
		req.input_commandlist.commands[0].additional_values = [55,98]
		req.input_commandlist.replace_previous_commands = False

		response = client(req)

		assert ((response.output_commandlist.commands[0].command_type) == "LIN")
		assert ((response.output_commandlist.commands[0].pose_reference) == "frame_a")
		assert (round(response.output_commandlist.commands[0].pose_reference_frame.x,5) == 0.10000)
		assert (round(response.output_commandlist.commands[0].pose_reference_frame.y,5) == 0.5)
		assert (round(response.output_commandlist.commands[0].pose_reference_frame.z,5) == -0.10000)
		assert (round(response.output_commandlist.commands[0].pose_reference_frame.alpha,5) == 0.80000)
		assert (round(response.output_commandlist.commands[0].pose_reference_frame.beta,5) == 0)
		assert (round(response.output_commandlist.commands[0].pose_reference_frame.gamma,5) == 3.14000)
		assert ((response.output_commandlist.commands[0].pose_type) == "EULER_INTRINSIC_ZYX")
		assert (len(response.output_commandlist.commands[0].pose)==6)
		assert ((response.output_commandlist.commands[0].pose[0]) == -0.1170429214835167)
		assert ((response.output_commandlist.commands[0].pose[1]) == 0.8501960635185242)
		assert ((response.output_commandlist.commands[0].pose[2]) == -0.30063676834106445)
		assert ((response.output_commandlist.commands[0].pose[3]) == -2.3408701419830322)
		assert ((response.output_commandlist.commands[0].pose[4]) == -0.500001847743988)
		assert ((response.output_commandlist.commands[0].pose[5]) ==  0.0002221506292698905)
		assert ((response.output_commandlist.commands[0].velocity_type) == "M/S")
		assert (len(response.output_commandlist.commands[0].velocity) == 3) 
		assert (round(response.output_commandlist.commands[0].velocity[0],3) == 0.3) 
		assert (round(response.output_commandlist.commands[0].velocity[1],3) == 0.5) 
		assert (round(response.output_commandlist.commands[0].velocity[2],3) == 0.154) 
		assert ((response.output_commandlist.commands[0].acceleration_type) == "M/S^2")
		assert (len(response.output_commandlist.commands[0].acceleration) == 1)
		assert (round(response.output_commandlist.commands[0].acceleration[0],5) == 0.2) # 1-Array
		assert ((response.output_commandlist.commands[0].effort_type) == "EFFORT")
		assert (len(response.output_commandlist.commands[0].effort)==1 )
		assert ((response.output_commandlist.commands[0].effort[0])==5 )
		assert ((response.output_commandlist.commands[0].blending_type) == "M")
		assert (len(response.output_commandlist.commands[0].blending) == 2)
		assert (round(response.output_commandlist.commands[0].blending[0],3) == 1.5)
		assert (round(response.output_commandlist.commands[0].blending[1],3) == 0.2)
		assert (len(response.output_commandlist.commands[0].additional_parameters)== 2)
		assert ((response.output_commandlist.commands[0].additional_parameters[0])=='item1')
		assert ((response.output_commandlist.commands[0].additional_parameters[1])=='item2')
		assert (len(response.output_commandlist.commands[0].additional_values )== 2)
		assert ((response.output_commandlist.commands[0].additional_values[0])==55)
		assert ((response.output_commandlist.commands[0].additional_values[1])==98)
		assert ((response.output_commandlist.replace_previous_commands) == False)


if __name__ == '__main__':
	rospy.init_node('dnb_tf_autotest')
	try :
		rostest.rosrun(PKG, NAME, UnitTest, sys.argv) 
	except	KeyboardInterrupt as e :
		rospy.logger (e)
		pass

	print("exiting")

