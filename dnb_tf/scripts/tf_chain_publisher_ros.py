#!/usr/bin/python3

import rospy
import tf
import PyKDL as kdl
from geometry_msgs.msg import PoseStamped, Point, Quaternion

""" Lookup a chain of transforms in tf and publish as EulerFrame.

Params:
    ~root_frame: root frame of the chain.
    ~tip_frame: tip frame of the chain.

Topics:
    tf_chain: publish ROS Standard Frame


Rosrun usage:

    rosrun dnb_tf tf_chain_publisher.py _root_frame:=base_link _tip_frame:=ee_link tf_chain:=tool_state

Roslaunch usage:

    <node pkg="dnb_tf" type="tf_chain_publisher.py" name="TODO">
        <param name="root_frame" value="base_link"/>
        <param name="tip_frame" value="ee_link"/>
        <remap from="tf_chain" to="tool_state"/>
    </node>
"""

if __name__ == '__main__':
    rospy.init_node('tf_chain_publisher_ros')

    rate = rospy.Rate(100)
    tf = tf.TransformListener()

    topic = rospy.resolve_name('pose_state')
    publish = rospy.Publisher(topic, PoseStamped, queue_size=1).publish

    # Note that frame tip in root = transform root to tip
    target_frame = rospy.get_param('~root_frame', 'base_link')
    source_frame = rospy.get_param('~tip_frame', 'ee_link')

    rospy.loginfo('Publishing {}->{} on {}'.format(source_frame, target_frame, topic))

    seq = 0
    while not rospy.is_shutdown():
        try:
            if tf.frameExists(source_frame) and tf.frameExists(target_frame):
                t = tf.getLatestCommonTime(source_frame, target_frame)
                p, q = tf.lookupTransform(target_frame, source_frame, t)

                point = Point()
                point.x = p[0]
                point.y = p[1]
                point.z = p[2]

                quaternion = Quaternion()
                quaternion.x = q[0]
                quaternion.y = q[1]
                quaternion.z = q[2]
                quaternion.w = q[3]

                pose = PoseStamped()
                pose.header.seq = seq
                pose.header.frame_id = source_frame
                pose.header.stamp = rospy.Time.now()
                pose.pose.position = point
                pose.pose.orientation = quaternion

                publish(pose)

                seq = seq + 1
        except:
            rospy.logwarn("No available tf connection between " + source_frame + " and " + target_frame + ". Retrying...")
        rate.sleep() # comment for testing memory leak
