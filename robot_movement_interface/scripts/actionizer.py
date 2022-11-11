#!/usr/bin/python3

import time
import rospy
import _thread
import threading
import actionlib

from dnb_tf.srv import *
from dnb_tool_manager.srv import TransformGoal
from robot_movement_interface.msg import * 	# Get FK
from robot_movement_interface.srv import *
from std_srvs.srv import *
from dnb_collision_detection.srv import *	# CheckWaypoints
from dnb_virtual_walls.srv import *			# CheckWalls

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
		self._srv_set_collision_checking = rospy.Service('~set_collision_checking', SetBool, self.cb_set_collision_checking)
		self._srv_set_virtual_walls = rospy.Service('~set_virtual_walls', SetBool, self.cb_set_virtual_walls)
		self._srv_set_move_despite_collision = rospy.Service('~set_move_despite_collision', SetBool, self.cb_set_move_despite_collision)
		self._collision_checking = False
		self._virtual_walls = False
		self._virtual_walls_service_proxy = None
		self._fk_service_proxy = None
		self._move_despite_collision = False

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

	def cb_set_collision_checking(self, req):
		self._collision_checking = req.data
		return SetBoolResponse(success = True)

	def cb_set_virtual_walls(self, req):
		#rospy.logerr("Actionizer: walls set to " + str(req.data))
		if req.data:
			try:
				rospy.wait_for_service('/get_fk', 3.0)
				rospy.wait_for_service('/dnb_virtual_walls/check_walls', 3.0)
				self._fk_service_proxy = rospy.ServiceProxy('get_fk', GetFK)
				self._virtual_walls_service_proxy = rospy.ServiceProxy('/dnb_virtual_walls/check_walls', CheckWalls)
				self._virtual_walls = True
			except Exception as e:
				rospy.logerr("Actionizer: walls check cannot be activated due to " + str(e))
				self._virtual_walls = False
				self._virtual_walls_service_proxy = None
				self._fk_service_proxy = None
		else:
			self._virtual_walls = False
			self._virtual_walls_service_proxy = None
			self._fk_service_proxy = None
		return SetBoolResponse(success = self._virtual_walls)

	def cb_set_move_despite_collision(self, req):
		self._move_despite_collision = req.data
		return SetBoolResponse(success = True)

	def cb_set_need_ik_when_collision_checking(self, req):
		self._need_ik_when_collision_checking = req.data
		return SetBoolResponse(success = True)

	def check_for_collision(self, commands): # returns service_success, ik_success, have_collision
		try:
			rospy.wait_for_service('/dnb_check_trajectory_collision/check_waypoints', 1.0)
			srv_check_waypoints = rospy.ServiceProxy('/dnb_check_trajectory_collision/check_waypoints', CheckWaypoints)

			request = CheckWaypointsRequest()
			request.commands = commands

			result = srv_check_waypoints(request)

			if result.result_code == CheckWaypointsResponse.SUCCESS:
				return True, True, len(result.collisions.collisions)>0
			elif result.result_code == CheckWaypointsResponse.ERROR_IK_FAILED:
				return True, False, False
			else:
				return False, False, False
		except (rospy.ServiceException, rospy.ROSException):
			return False, False, False

	# Given a list of joints returns its euler pose by calling the fk service
	# PRE: FK Service must exist and should be checked before, assert walls checking active
	def get_euler_from_joints(self, joints):
		assert(self._virtual_walls and self._virtual_walls_service_proxy and self._fk_service_proxy)
		fk_request = GetFKRequest()
		fk_request.joints = joints
		fk_request.result_robot_based = True
		fk_request.result_flange_based = True
		fk_response = self._fk_service_proxy(fk_request)
		p = fk_response.pose
		assert(fk_response.error == 0)
		return [p.x, p.y, p.z, p.alpha, p.beta, p.gamma]

	# Returns success = true if command crashes with no wall. If das system is deactivated, returns always success
	def check_for_walls(self, commands):
		if self._virtual_walls:
			for command in commands.commands:
				if command.command_type == "LIN" or command.command_type == "PTP":
					if command.pose_type == "JOINTS":
						target_pose = self.get_euler_from_joints(command.pose)
					elif command.pose_type == "EULER_INTRINSIC_ZYX":
						target_pose = command.pose
					else:
						rospy.logerr("Actionizer: Unidentified pose type, cannot check for walls")
						return False
					wall_request = CheckWallsRequest()
					wall_request.flange.x = target_pose[0]
					wall_request.flange.y = target_pose[1]
					wall_request.flange.z = target_pose[2]
					wall_request.flange.alpha = target_pose[3]
					wall_request.flange.beta = target_pose[4]
					wall_request.flange.gamma = target_pose[5]
					wall_response = self._virtual_walls_service_proxy(wall_request)
					if wall_response.crash: return False
				else:
					rospy.logerr("Actionizer: Walls accepts only LIN or PTP moves. Cancelling")
					return False
		return True	

	# Action callback
	def execute_cb(self, goal):
		global command_id
		global last_msg
		global last_finished_id

		rospy.loginfo("<ActionizerInformed> <> <>")

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

		if self._collision_checking:
			service_success, ik_success, have_collision = self.check_for_collision(goal.commands)
		else:
			service_success = True
			ik_success = True
			have_collision = False

		if self._virtual_walls:
			if not self.check_for_walls(goal.commands):
				rospy.loginfo("Command list aborted (wall crash).")
				last_finished_id = None
				self._result.result.result_code = Result.FAILURE_COLLISION_CHECKING
				self._as.set_preempted(self._result)
				return

		if self._collision_checking and not service_success:
			rospy.loginfo("Command list aborted (collision checking failure).")
			last_finished_id = None
			self._result.result.result_code = Result.FAILURE_COLLISION_CHECKING
			self._as.set_preempted(self._result)
			return

		if self._collision_checking and not ik_success:
			rospy.loginfo("Command list aborted (ik failure).")
			last_finished_id = None
			self._result.result.result_code = Result.FAILURE_IK
			self._as.set_preempted(self._result)
			return

		if (not have_collision or (have_collision and self._move_despite_collision)):
			# publish the goal to the command_list topic
			rospy.loginfo("<ActionizerFinished> <> <>")
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
		else:
			rospy.loginfo("Command list aborted (possible collision).")
			last_finished_id = None
			self._result.result.result_code = Result.FAILURE_POSSIBLE_COLLISION
			self._as.set_preempted(self._result)
			return


if __name__ == '__main__':
	rospy.init_node('commands_action_server')
	rospy.Subscriber('/command_result', Result, result_callback)
	actionizerInstance = Actionizer(rospy.get_name())

	rospy.loginfo("Action server started with name '/commands_action_server'")
	rospy.spin()
