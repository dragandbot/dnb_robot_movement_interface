#!/usr/bin/python3

import time
import rospy
import _thread
import threading
import actionlib

from dnb_tf.srv import *
from dnb_tool_manager.srv import TransformGoal
from robot_movement_interface.msg import *

# Information: a result code != 0 means abnormal trajectory 
# Information: a result code == -1 means that an error was produced in the trajectory, and therefore will finish
# If result code == -1 the rest of the message is invalid

# TODO: add commond_id control

# ------------------------------------------------------------------------
# Variables
# ------------------------------------------------------------------------
transformServiceName = '/transform_commandlist' # dnb_tf/Transform service form dnb_tf package
toolManagerServiceName = '/dnb_tool_manager/transform_goal' # dnb_tool_manager/TransformGoal from dnb_tool_manager package
last_finished_id = None
last_msg = None

lock = threading.Lock() # Just in case
command_id = 0			# Actionizer will increase the id now

# ------------------------------------------------------------------------
# Callback function executed after the publication of new result
# If the driver is cancelled with /stop_robot_right_now or killed, it is not
# guaranteed to receive a /command_result after cancellation
# ------------------------------------------------------------------------
def result_callback(msg):
	global last_msg
	global last_finished_id
	rospy.logdebug("Result Callback")
	with lock:
		last_finished_id = msg.command_id
		last_msg = msg

class Actionizer(object):

	_feedback = CommandsFeedback()
	_result = CommandsResult()

	# Action initialisation
	def __init__(self, name):
		self.publisher = rospy.Publisher('/command_list', CommandList, queue_size=10)
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, CommandsAction, execute_cb=self.execute_cb, auto_start = False)

		rospy.loginfo("Starting actionizer")

		# wait some seconds until both transformation services are loaded
		while not rospy.is_shutdown():
			try:
				rospy.wait_for_service(transformServiceName, timeout=5)
				rospy.wait_for_service(toolManagerServiceName, timeout=5)
				break
			except:
				rospy.logwarn("Could not start actionizer. Check if service " + transformServiceName + " is running.")
				rospy.logwarn("Could not start actionizer. Check if service " + toolManagerServiceName + " is running.")

		# create the service client when waiting for it was successful
		self._transformService = rospy.ServiceProxy(transformServiceName, Transform)
		self._toolManagerService = rospy.ServiceProxy(toolManagerServiceName, TransformGoal)
		self._as.start() # Start the action

	# Action callback
	def execute_cb(self, goal):
		global command_id
		global last_msg
		global last_finished_id

		# last finished id and msg must come after the new command is sent.
		last_finished_id = None
		last_msg = None
		self._result.result.result_code = Result.FAILURE_EXECUTION # Error by default

		# Replace the command ids with the sequencer in the actionizer
		for x in goal.commands.commands:
			x.command_id = command_id
			command_id = (command_id + 1) % 2047483647

		rospy.logdebug("Execute Callback with {} commands".format(len(goal.commands.commands)))

		# setting a global variable for result callback method
		try:
			goal_id = goal.commands.commands[-1].command_id # Goal Id is the target ID we are expecting arrive
		except:
			rospy.logerr("Error trying to access to last command ID")
			self._as.set_aborted(self._result) # Aborted proxy
			return

		# The service will provide a transformed command list if success or an empty list if not
		# transform command before publishing the commands
		# pose_reference_frame is used for the transformations
		try:
			transformedCommandList = self._transformService(goal.commands).output_commandlist
		except:
			rospy.logerr("Error using relative position transform service.")
			self._as.set_aborted(self._result) # Aborted proxy
			return

		# when the response is not empty, in case of a successful transformation
		# set this new transformation as a goal. or just publish on topic
		if transformedCommandList.commands:
			rospy.logdebug('Transformation successful.')
			goal.commands = transformedCommandList
		else:
			rospy.logerr("Error transforming relative positions. Empty list of positions.")
			self._as.set_aborted(self._result) # Aborted proxy
			return

		# The service will provide a transformed command list if success or an empty list if not
		try:
			toolMangerCommandList = self._toolManagerService(goal.commands).output_commandlist
		except:
			rospy.logerr("Error using TCP transform service.")
			self._as.set_aborted(self._result) # Aborted proxy
			return

		if toolMangerCommandList.commands:
			rospy.logdebug('TCP Transformation successful.')
			goal.commands = toolMangerCommandList
		else:
			rospy.logerr("Error transforming Tool Center Point. Empty list of positions.")
			self._as.set_aborted(self._result) # Aborted proxy
			return

		# publish the goal to the command_list topic
		self.publisher.publish(goal.commands)

		# loop until last goal was reached or
		# cancelation of the commands was requested
		rospy.logdebug("Waiting for last_result message...")
		while True:

			with lock:
				if self._as.is_preempt_requested():
					rospy.loginfo("Command list aborted (preempted).")
					last_finished_id = None
					if last_msg: self._result.result = last_msg
					self._result.result.result_code = Result.FAILURE_EXECUTION
					self._as.set_preempted(self._result)
					return
				elif last_msg and last_msg.result_code != Result.SUCCESS:
					rospy.loginfo("Command list aborted (error executing trajectory).")
					last_finished_id = None
					if last_msg: self._result.result = last_msg
					self._result.result.result_code = last_msg.result_code
					self._as.set_aborted(self._result)
					return
				elif goal_id == last_finished_id:
					rospy.logdebug("Last trajectory command reached.")
					if last_msg: self._result.result = last_msg
					self._as.set_succeeded(self._result)
					return

			rospy.sleep(0.05)


if __name__ == '__main__':
	rospy.init_node('commands_action_server')
	rospy.Subscriber('/command_result', Result, result_callback)
	actionizerInstance = Actionizer(rospy.get_name())

	rospy.loginfo("Action server started with name '/commands_action_server'")
	rospy.spin()
