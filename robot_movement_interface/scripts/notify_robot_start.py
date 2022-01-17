#!/usr/bin/python3

import rospy
from std_msgs.msg import Empty

# service is setup
def robot_start_notify():
    rospy.init_node('notify_robot_start')
    publisher_notify_changed_transforms = rospy.Publisher('/notify_changed_system_transformations', Empty, queue_size=1)
    delay = rospy.Duration(rospy.get_param('~delay', 1.0))

    rospy.sleep(delay)

    message = Empty()
    publisher_notify_changed_transforms.publish(message)
    
    rospy.sleep(delay)

# starts the server
if __name__ == '__main__':
    robot_start_notify()
