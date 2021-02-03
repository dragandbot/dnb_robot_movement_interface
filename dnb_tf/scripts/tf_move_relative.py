#!/usr/bin/env python
"""
    File: tf_move_relative.py
    Author: Witalij Siebert
    Python Version: 2.7

    Description: This file provides two interfaces for transformations.
        transform_commandlist: An interface for relative movement
        skills which are used by the dragandbot frontend. Inputs are a
        trajectory and the input frame. The Interface will then transform
        the trajectory into the appropriate frame (/base) and send it back
        to the actionizer.
        transform_pose: An interface for float[] poses. Inputs are the pose,
        the input frame and a boolean 'invert'. If set true the interface does
        invert the given transformation and uses the inverted Transformation
        on the pose.
"""
import roslib

import rospy
import threading
import math
import tf
import geometry_msgs.msg

from copy import deepcopy
# import our own services and messages
from std_msgs.msg import Empty
from dnb_tf.srv import *
from robot_movement_interface.msg import *

# ROS PATHS
URI_FIXED_FRAME = "/dnb_remote_robot_control/robot_data/fixed_frame"
URI_WORLD_FRAME = "/dnb_remote_robot_control/robot_data/world_frame"

# that frames are used for calculations
BASE_HELPER = "base_helper"
TARGET_HELPER = "target_helper"

########################
### Helper Functions ###
########################

def dnb_euler_list_to_pose_stamped(euler, timestamp, frame):
    ps = geometry_msgs.msg.PoseStamped()
    ps.header.frame_id = frame
    ps.header.stamp = timestamp
    ps.pose.position.x = euler[0]
    ps.pose.position.y = euler[1]
    ps.pose.position.z = euler[2]

    # Calculate the orientation from RPY
    q = tf.transformations.quaternion_from_euler(euler[5], euler[4], euler[3], axes='sxyz')

    ps.pose.orientation.x = q[0]
    ps.pose.orientation.y = q[1]
    ps.pose.orientation.z = q[2]
    ps.pose.orientation.w = q[3]
    return ps

def dnb_euler_frame_to_transformation(euler, timestamp, parent_frame, child_frame):
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = parent_frame
    t.header.stamp = timestamp
    t.child_frame_id = child_frame
    t.transform.translation.x = euler.x
    t.transform.translation.y = euler.y
    t.transform.translation.z = euler.z

    # Calculate the orientation from RPY
    q = tf.transformations.quaternion_from_euler(euler.gamma, euler.beta, euler.alpha, axes='sxyz')

    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t

def pose_stamped_to_dnb_euler_list(pstamped):
    el = [0] * 6
    el[0] = pstamped.pose.position.x
    el[1] = pstamped.pose.position.y
    el[2] = pstamped.pose.position.z

    # Calculate RPY from the quaternion
    euler = tf.transformations.euler_from_quaternion([
        pstamped.pose.orientation.x,
        pstamped.pose.orientation.y,
        pstamped.pose.orientation.z,
        pstamped.pose.orientation.w],
        axes='sxyz')

    el[3] = euler[2]
    el[4] = euler[1]
    el[5] = euler[0]
    return el


########################
###   DNBTF Class    ###
########################

class DNBTF:
    def __init__(self):
        self.lock = threading.Lock()
        self.tf_listener = tf.TransformListener(True, rospy.Duration(10.0))

        self.robot_fixed_frame = rospy.get_param(URI_FIXED_FRAME, "base_link")
        self.world_frame = rospy.get_param(URI_WORLD_FRAME, "world")
        self.transformation_world_to_fixed_frame = None        

        service_transform_cmdlist = rospy.Service('transform_commandlist', Transform, self.handle_transform)
        service_transform_pose = rospy.Service("transform_pose", TransformPose, self.handle_transform_pose)
        subscriber_notify_changed_transforms = rospy.Subscriber("notify_changed_system_transformations", Empty, self.callback_notify_changed_transformations)

        rospy.loginfo("TFMoveRelative: Ready to transform poses.")

    # --------------------------------------------------------
    # This will listen to a specified tf transforamtion
    #  and return the values as EulerFrame
    # --------------------------------------------------------
    def listen_transformation(self, source_frame, target_frame):
        pos, rot = self.tf_listener.lookupTransform(target_frame, source_frame, self.tf_listener.getLatestCommonTime(target_frame, source_frame))

        ef = EulerFrame()
        ef.x = pos[0]
        ef.y = pos[1]
        ef.z = pos[2]

        euler = tf.transformations.euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]], axes='sxyz')
        ef.alpha = euler[2]
        ef.beta = euler[1]
        ef.gamma = euler[0]
        return ef

    # --------------------------------------------------------
    # This will set a transformation to the internal tf tree.
    # This is used to transform the incoming commands.
    # --------------------------------------------------------
    def publish_transformation(self, eulerPose, transformer, timestamp):
        t = dnb_euler_frame_to_transformation(eulerPose, timestamp, BASE_HELPER, TARGET_HELPER)
        transformer.setTransform(t)

    # --------------------------------------------------------
    # Transform a complete incoming pose and send it back in the
    # proper form.
    # child_pose is the position to attached to the parent_pose
    # parent_pose is the base pose
    # timestamp is now, at what time we want to transform
    # invert False: attaches the child_pose to the parent_pose
    # invert True:  attaches the parent_pose to the child_pose
    # Return: complete transformed pose: the child_pose combined with the parent pose
    # --------------------------------------------------------
    def transform(self, child_pose, parent_pose, timestamp, invert):

        source = BASE_HELPER
        target = TARGET_HELPER

        if invert:
            target = BASE_HELPER
            source = TARGET_HELPER

        # the transformer
        tr = tf.TransformerROS()

        # create the transformation frame in tr and publish it
        self.publish_transformation(parent_pose, tr, timestamp)

        ps_input = dnb_euler_list_to_pose_stamped(child_pose, timestamp, target)

        # transform pose from input frame to base
        ps_output = tr.transformPose(source, ps_input)

        return pose_stamped_to_dnb_euler_list(ps_output)

    ##################################
    ### Service Callback Functions ###
    ##################################

    # --------------------------------------------------------
    # Handle incoming transformation requests.
    # --------------------------------------------------------
    def handle_transform(self, req):
        with self.lock:
            # the timestamp equal on each called service request
            transformTime = rospy.Time.now()

            # make a copy for the result
            res = TransformResponse()
            res.output_commandlist = deepcopy(req.input_commandlist)

            rospy.logdebug('TFMoveRelative: Transforming relative frames for a command list')

            # iterate over the commands and transform the pose
            for i in range(0, len(req.input_commandlist.commands)):

                # Joint moves are not transformed because they are absolute
                if req.input_commandlist.commands[i].pose_type == "JOINTS": continue

                rospy.logdebug("TFMoveRelative: Transforming command with ID: " + str(req.input_commandlist.commands[i].command_id))

                try:
                    # Backward compatibility: empty pose_reference is taken as "world"
                    if req.input_commandlist.commands[i].pose_reference == "":
                        req.input_commandlist.commands[i].pose_reference = self.world_frame

                    # step 1: calculate the absolute pose of the pose_reference_frame in the given frame
                    res.output_commandlist.commands[i].pose = self.transform(req.input_commandlist.commands[i].pose, req.input_commandlist.commands[i].pose_reference_frame, transformTime, False)

                    # step 2: convert the absolute pose from the given frame into the robot frame
                    if req.input_commandlist.commands[i].pose_reference == self.world_frame and self.transformation_world_to_fixed_frame != None:
                        # step 2.1: reference is world, so use world_to_fixed_frame from cache
                        robot_base_to_given_pose_reference = self.transformation_world_to_fixed_frame
                    else:
                        # step 2.1: listen the transformation from the robot base frame to the given frame on the public tf tree
                        robot_base_to_given_pose_reference = self.listen_transformation(self.robot_fixed_frame, req.input_commandlist.commands[i].pose_reference)

                    # step 2.2: "substract" the transformation from the robot base frame to the given frame from the given pose
                    res.output_commandlist.commands[i].pose = self.transform(res.output_commandlist.commands[i].pose, robot_base_to_given_pose_reference, transformTime, True)
                except Exception as ex:
                        rospy.logerr("TFMoveRelative: Transformation error: " + str(ex))
                        return TransformResponse()

            return res


    # --------------------------------------------------------
    # Handle incoming transform pose requests.
    # --------------------------------------------------------
    def handle_transform_pose(self, req):
        with self.lock:
            # create empty response
            res = TransformPoseResponse()

            try:
                res.output_pose = self.transform(req.input_pose, req.transform, rospy.Time.now(), req.invert)
            except Exception as ex:
                rospy.logerr("TFMoveRelative: Transformation error: " + str(ex))
                return TransformPoseResponse()

            return res

    # --------------------------------------------------------
    # Handle incoming robot has reloaded notification
    # --------------------------------------------------------
    def callback_notify_changed_transformations(self, msg):
        rospy.logdebug("TFMoveRelative: Robot restart notified")
        loop_rate = rospy.Duration(1.0)

        with self.lock:
            while not rospy.is_shutdown():

                self.robot_fixed_frame = rospy.get_param(URI_FIXED_FRAME, "base_link")
                self.world_frame = rospy.get_param(URI_WORLD_FRAME, "world")

                try:
                    self.transformation_world_to_fixed_frame = self.listen_transformation(self.robot_fixed_frame, self.world_frame)
                    return
                except Exception as ex:
                    rospy.logerr("TFMoveRelative: Transformation error: " + str(ex))
                    self.transformation_world_to_fixed_frame = None   

                rospy.sleep(loop_rate)
# starts the server
if __name__ == '__main__':
    rospy.init_node('transform_pose_server')
    dnb_tf = DNBTF()
    rospy.spin()
