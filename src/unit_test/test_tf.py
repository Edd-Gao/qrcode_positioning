#! /usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
import thread
import sys

rospy.init_node("test_tf")

br = tf2_ros.TransformBroadcaster()
t = geometry_msgs.msg.TransformStamped()

while True:
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "test"
    t.transform.translation.x = 0
    t.transform.translation.y = 10
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 1.2)

    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

    rospy.sleep(rospy.Duration(1, 0))
