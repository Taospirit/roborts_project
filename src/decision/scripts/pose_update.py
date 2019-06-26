#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
import math
import tf
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf import TransformListener

if __name__ == '__main__':
    rospy.init_node('pose_update')
    robot_pose_pub = rospy.Publisher('robot_pose', Odometry, queue_size=1)
    robot_gimbal_pub = rospy.Publisher('gimbal_pose', Quaternion, queue_size=1)
    pose = Odometry()
    node_ns = rospy.get_namespace()
    tf1 = tf.TransformListener()
    tf2 = tf.TransformListener()
    pose.header.frame_id = 'map'
    pose.child_frame_id = node_ns + '/base_link'
    rate = rospy.Rate(40.0)
    while not rospy.is_shutdown():
        try:
            t, q = tf1.lookupTransform("/map", 'base_link', rospy.Time(0))
            pose.pose.pose.position.x = t[0]
            pose.pose.pose.position.y = t[1]
            pose.pose.pose.orientation.x = q[0]
            pose.pose.pose.orientation.y = q[1]
            pose.pose.pose.orientation.z = q[2]
            pose.pose.pose.orientation.w = q[3]
            robot_pose_pub.publish(pose)
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as err:
            rospy.logwarn('no robot tf')
            rospy.logwarn(err)
        # try:
        #     t, r = tf2.lookupTransform(node_ns + 'base_link', node_ns + 'gimbal', rospy.Time(0))
        #     robot_gimbal_pub.publish(r[0], r[1], r[2], r[3])
        # except (tf.LookupException, tf.ConnectivityException,
        #         tf.ExtrapolationException) as err:
        #     rospy.logwarn('no gimbal tf')
        #     rospy.logwarn(err)
        rate.sleep()
