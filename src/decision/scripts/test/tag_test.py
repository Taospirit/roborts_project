#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from decision.msg import BulletAmount

def talker():
    pub = rospy.Publisher("bullet_amount",BulletAmount,queue_size = 0)
    rospy.init_node("talker")
    
    rospy.loginfo("talker started!!!")
    while not rospy.is_shutdown():
        rospy.loginfo("pub")
        a = 1
        #pub.publish("a")
    
if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
