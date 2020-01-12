#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Point

class publish_dynamic_goal():
    def __init__(self):
        rospy.init_node('publish_dynamic_goal', anonymous=True)

        self.listener = tf.TransformListener()

        rospy.loginfo("Initiating publish_trajectory...")
        self.marker_publisher = rospy.Publisher('dynamic_goal', Point, queue_size=10)
        self.point = Point(-0.4, 0.0, 0.75)

    def generate_trajectory(self):
        dec = 0.001
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.point.z < 0.4:
                dec = -1*dec
            elif self.point.z > 0.75:
                dec = -1*dec

            self.point.z += dec

            rate.sleep()
            self.marker_publisher.publish(self.point)

if __name__ == '__main__':
    try:
        publish_dyn = publish_dynamic_goal()
        publish_dyn.generate_trajectory()

    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted before completion")
