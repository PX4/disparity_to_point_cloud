#!/usr/bin/env python
import roslib
roslib.load_manifest('disparity_to_point_cloud')

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('my_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 1.0),
                         ( -0.70710678118654, 0.0,0.0, 0.70710678118654),
                         rospy.Time.now(),
                         "map",
                         "world")
        rate.sleep()
