#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l

i/k : move_forward / move_backward
j/l : move_left / move_right
u/o : turn_left / turn_right

anything else : stop
q/a : increase/decrease max speeds by 10%
w/s : increase/decrease only linear speed by 10%
e/d : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'u':(0, 0, 0, 1),
        'i':(1, 0, 0, 0),
        'o':(0, 0, 0, -1),
        'j':(0, 1, 0, 0),
        'k':(-1, 0, 0, 0),
        'l':(0, -1, 0, 0),

        # 'T':(1,0,0,0),
        # 'Y':(1,1,0,0),
        # 'U':(0,-1,0,0),
        # 'G':(1,1,0,0),
        # 'H':(-1,1,0,0),
        # 'J':(-1,-1,0,0),
        # 'B':(0,1,0,0),
        # 'N':(0,0,1,0),
        # 'M':(0,0,-1,0),
           }

speedBindings={
        'q':(1.1,1.1),
        'a':(.9,.9),
        'w':(1.1,1),
        's':(.9,1),
        'e':(1,1.1),
        'd':(1,.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print msg
        print vels(speed,turn)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print vels(speed,turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break
                    
            if key in moveBindings.keys():
                twist = Twist()
                twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
                pub.publish(twist)

    except:
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


