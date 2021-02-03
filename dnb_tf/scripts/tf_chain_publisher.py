#!/usr/bin/env python
import rospy
import tf2_ros
import PyKDL as kdl

from robot_movement_interface.msg import EulerFrame

""" Lookup a chain of transforms in tf and publish as EulerFrame.

Params:
    ~root_frame: root frame of the chain.
    ~tip_frame: tip frame of the chain.

Topics:
    tf_chain: publish EulerFrame


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
    rospy.init_node('tf_chain_publisher')
    rate = rospy.Rate(100)

    tr = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tr)

    topic = rospy.resolve_name('tf_chain')
    topic2 = rospy.resolve_name('tool_frame')
    publish = rospy.Publisher(topic, EulerFrame, queue_size=1).publish
    publish2 = rospy.Publisher(topic2, EulerFrame, queue_size=1).publish

    # Note that frame tip in root = transform root to tip
    target_frame = rospy.get_param('~root_frame', 'base_link')
    source_frame = rospy.get_param('~tip_frame', 'ee_link')

    # wait a second for frames to accumulate. buf.lookup_transform seems to fail immediately if it hasn't yet
    # gotten any transfroms for the 'odom' frame, instead of waiting for the timeout
    rospy.sleep(1)

    while not rospy.is_shutdown():

        try:
            t = tr.lookup_transform(target_frame, source_frame, rospy.Time(), rospy.Duration(1.0))
            p, q = t.transform.translation, t.transform.rotation
            a, b, c = kdl.Rotation.Quaternion(q.x, q.y, q.z, q.w).GetEulerZYX()
            publish(EulerFrame(p.x,p.y,p.z, a, b, c))
            publish2(EulerFrame(p.x,p.y,p.z, a, b, c))

        except:
            rospy.logwarn("No available tf connection between " + source_frame + " and " + target_frame + ". Retrying...")

        rate.sleep() # comment for testing memory leak
