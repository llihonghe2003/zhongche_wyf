#! /usr/bin/env python

import rospy

# include <geometry_msgs/Twist.h>
from geometry_msgs.msg import Twist

# import geometry_msgs


def main():
    rospy.init_node("test_yaw")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    msgControl = Twist()
    msgControl.linear.x = 0
    msgControl.angular.z = 1

    while not rospy.is_shutdown():
        cmd = raw_input("cmd_vel:")
        print(msgControl.angular.z)
        msgControl.angular.z = msgControl.angular.z + 0.1
        pub.publish(msgControl)


if __name__ == "__main__":
    main()
