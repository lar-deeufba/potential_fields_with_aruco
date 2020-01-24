#!/usr/bin/env python
import roslib
import rospy
import tf


if __name__ == '__main__':
    rospy.init_node('ar_marker_0_broadcaster')
    # turtlename = rospy.get_param('~turtle')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(125)
    print "Publishing ar_marker_0 link"
    while not rospy.is_shutdown():
        br.sendTransform((-0.5, -0.1, 0.033),
                         tf.transformations.quaternion_from_euler(0.0, 0.0, -0.6),
                         rospy.Time.now(),
                         "ar_marker_0",
                         "base_link")
        rate.sleep()
