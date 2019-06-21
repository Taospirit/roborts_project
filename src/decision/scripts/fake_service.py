#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from roborts_msgs.srv import ShootCmd, GimbalMode, ChassisMode, FricWhl


def handle_fake_srv(req):
    
    return 0


if __name__ == "__main__":
    rospy.init_node('fake_service')
    cmd_shoot_srv = rospy.Service('cmd_shoot', ShootCmd, handle_fake_srv)
    cmd_shoot_srv = rospy.Service('set_gimbal_mode', GimbalMode,
                                  handle_fake_srv)
    cmd_shoot_srv = rospy.Service('set_chassis_mode', ChassisMode,
                                  handle_fake_srv)
    cmd_shoot_srv = rospy.Service('cmd_fric_wheel', FricWhl, handle_fake_srv)
    rospy.spin()