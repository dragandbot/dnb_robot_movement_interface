#!/usr/bin/python3
import time
import rospy
import actionlib
import random

from robot_movement_interface.msg import *
from robot_movement_interface.srv import *
from std_srvs.srv import *

###########################
# HELPERS: Move Cartesian #
###########################

# Returns str including non-ascii characters
def safe_string(s):
	if (sys.version_info > (3, 0)):
		return s
	else:
		return s.encode("utf-8") if isinstance(s, unicode) else s

class ActionManager(object):
	debug = False
	def __init__(self, action_name, action_type):
		if self.debug: print("ActionManager - Initializing")
		self.client = actionlib.SimpleActionClient(action_name, action_type)
		self.client.wait_for_server()
		if self.debug: print("ActionManager - Initialized")

	def __del__(self):
		if self.debug: print("ActionManager - Destructor called")
		self.cleaner(self.client)
		if self.debug: print("ActionManager - Destructor finished")

	def cleaner(self, client):
		if self.debug: print("ActionManager - Cleaner called")
		if not client: return
		client.action_client.status_sub.unregister()
		client.action_client.result_sub.unregister()
		client.action_client.feedback_sub.unregister()
		del client.action_client.status_sub
		del client.action_client.result_sub
		del client.action_client.feedback_sub
		del client.action_client.pub_goal
		del client.action_client.pub_cancel
		del client
		if self.debug: print("ActionManager - Cleaner finished")

	# This functions does a blocking call to the action, waiting until any result (also cancellation) is received
	# It returns the result message received
	def call(self, goal):
		if self.debug: print("ActionManager - Call called")
		self.client.send_goal(goal)
		self.client.wait_for_result()
		if self.debug: print("ActionManager - Call finished with result:")
		if self.debug: print(self.client.get_result())
		return self.client.get_result()
	# This function
	def abort(self):
		if self.debug: print("ActionManager - Abort called")
		self.client.cancel_all_goals()
		self.client.wait_for_result()
		if self.debug: print("ActionManager - Aborted")

aborted = False

###############################
# END HELPERS: Move Cartesian #
###############################

###############################
# RMI: API                    #
###############################

def move_to(x: float, y: float, z: float, rz: float = 0, ry: float = 0, rx: float = 0):
	global aborted
	aborted = False

    # IGNORE ORIENTATION!
	rz = -1.5707963705062866
	ry = 0
	rx = -3.141592502593994

	command_action_goal = robot_movement_interface.msg.CommandsGoal()
	command_list = CommandList()
	command_list.commands = []
	command_list.replace_previous_commands = True
	command_id = random.randint(0,2047483647)

	# create the reference and prefill
	ref = EulerFrame()
	ref.x = float(0.0)
	ref.y = float(0.0)
	ref.z = float(0.0)
	# alpha, beta, gamma is essentially rz, ry, rx
	ref.alpha = float(0.0)
	ref.beta =  float(0.0)
	ref.gamma = float(0.0)

	command = Command()
	command.command_id = command_id
	command.pose_reference_frame = ref
	command.command_type = "LIN"
	command.pose_type = 'EULER_INTRINSIC_ZYX'
	command.pose = [x/1000, y/1000, z/1000, rz, ry, rx]
	print(command.pose)
	command.velocity_type = "M/S"
	command.velocity = [0.15]
	command.blending_type = "PERCENT"
	command.blending = [50.0]
	command.acceleration_type = "PERCENT"
	command.acceleration = [50.0]
	command.pose_reference = 'world'

	command_list.commands.append(command)
	command_id += 1

	command_action_goal.commands = command_list
	result_msg = move_manager.call(command_action_goal)

	if not aborted and not result_msg:
		raise Exception("COMMAND_FAILED")
	if not aborted and result_msg and result_msg.result.result_code != Result.SUCCESS:
		if result_msg.result.result_code == Result.FAILURE_COMMUNICATION:
			raise Exception("COMMUNICATION_ERROR")
		elif result_msg.result.result_code == Result.FAILURE_TARGET_NOT_REACHABLE:
			raise Exception("POSE_NOT_REACHABLE")
		elif result_msg.result.result_code == Result.FAILURE_OUT_OF_REACH:
			raise Exception("POSE_NOT_REACHABLE")
		elif result_msg.result.result_code == Result.FAILURE_ROBOT_CONFIGURATION:
			raise Exception("POSE_NOT_REACHABLE")
		elif result_msg.result.result_code == Result.FAILURE_JOINT_LIMIT:
			raise Exception("POSE_NOT_REACHABLE")
		elif result_msg.result.result_code == Result.FAILURE_CROSSING_SINGULARITY:
			raise Exception("POSE_NOT_REACHABLE")
		elif result_msg.result.result_code == Result.FAILURE_EMERGENCY_STOP:
			raise Exception("SAFETY_STOP")
		elif result_msg.result.result_code == Result.FAILURE_COMPONENT_NOT_READY:
			raise Exception("ROBOT_NOT_READY")
		elif result_msg.result.result_code == Result.FAILURE_POSSIBLE_COLLISION:
			raise Exception("PROBABLE_COLLISION")
		elif result_msg.result.result_code == Result.FAILURE_IK:
			raise Exception("POSE_NOT_REACHABLE")
		else:
			raise Exception("COMMAND_ERROR")

def set_io(io: int, value: bool):
	global set_io_srv

	device = 'robot'
	service_name = device + '/set_digital_ios'

	try:
		rospy.wait_for_service(service_name, 1.0)
	except rospy.ROSException:
		raise Exception('ERROR_UNKNOWN_DEVICE')

	set_io_srv = rospy.ServiceProxy(service_name, SetDigitalIOs)

	request = SetDigitalIOsRequest()
	dio = DigitalIOValue()
	dio.id.group_id = "DO"
	dio.id.pin_number = int(io)
	dio.value = value
	request.ios.append(dio)

	try:
		response = set_io_srv(request)

		if not response.success:
			if len(response.errors) > 0:
				if (response.errors[0].error == IOConstants.SUCCESS):
					raise Exception('ERROR_UNKNOWN')
				elif (response.errors[0].error == IOConstants.ERR_INVALID_IO or response.errors[0].error == IOConstants.ERR_INVALID_GROUP):
					raise Exception('ERROR_NO_DIGITAL_IO')
				elif (response.errors[0].error == IOConstants.ERR_NO_OUTPUT):
					raise Exception('ERROR_NO_DIGITAL_OUTPUT')
				elif (response.errors[0].error == IOConstants.ERR_SETTING_FAILED):
					raise Exception('ERROR_SET_FAILED')
				else:
					raise Exception('ERROR_UNKNOWN')
	except rospy.ROSException:
		raise Exception('ERROR_UNKNOWN')

def get_io(io: int) -> bool:
	"""Get a boolean status of a specific signal or I/O.

	Parameters
	----------
	io : int
		The number of a signal which can be an input or an output.

	Returns
	-------
	bool
		Returns the signal status which is True or False.

	Raises
	------
	ERROR_UNKNOWN_DEVICE
		If the device is not known where the signal comes from.

	"""
	global get_io_srv

	device = 'robot'
	service_name = device + '/get_digital_ios'

	try:
		rospy.wait_for_service(service_name, 1.0)
	except rospy.ROSException:
		raise Exception('ERROR_UNKNOWN_DEVICE')

	get_io_srv = rospy.ServiceProxy(service_name, GetDigitalIOs)

	request = GetDigitalIOsRequest()
	ioid = IOID()
	ioid.group_id = "DO"
	ioid.pin_number = io
	request.ios.append(ioid)

	try:
		response = get_io_srv(request)

		if response.success:
			if len(response.ios) > 0:
				return bool(response.ios[0].value)
		else:
			if len(response.errors) > 0:
				if (response.errors[0].error == IOConstants.SUCCESS):
					pass
				elif (response.errors[0].error == IOConstants.ERR_INVALID_IO or response.errors[0].error == IOConstants.ERR_INVALID_GROUP):
					raise Exception('ERROR_NO_DIGITAL_IO')
				else:
					raise Exception('ERROR_UNKNOWN')
	except rospy.ROSException:
		raise Exception('ERROR_UNKNOWN')

def check_cube():
    cube_check = bool(random.getrandbits(1))
    time.sleep(1)
    if not cube_check: print("Cube was NOK")
    return cube_check

def grab():
	time.sleep(0.1)
	set_io(1, True)
	time.sleep(0.1)
	set_io(2, False)
	time.sleep(0.7)

def release():
	time.sleep(0.1)
	set_io(1, False)
	time.sleep(0.1)
	set_io(2, True)
	time.sleep(0.9)

rospy.init_node('rmi_api')
move_manager = ActionManager('commands_action_server', robot_movement_interface.msg.CommandsAction)
#rospy.spin()

