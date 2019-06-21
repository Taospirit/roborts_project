#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np
import rospy
import rospkg
import tf
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from pi_trees_ros.pi_trees_ros import *
from controller import Controller
from decision.msg import EnemyPos
from roborts_msgs.msg import GimbalAngle, GimbalRate, ShootInfo, ShootState, TwistAccel
from Battle import BattleEnv


# 慢速向前命令
cmdvel_slowfront = Twist()
cmdvel_slowfront.linear.x = 0.205
cmdvel_slowfront.linear.y = 0
cmdvel_slowfront.linear.z = 0
cmdvel_slowfront.angular.x = 0
cmdvel_slowfront.angular.y = 0
cmdvel_slowfront.angular.z = 0

# 慢速向后命令
cmdvel_slowback = Twist()
cmdvel_slowback.linear.x = -0.205
cmdvel_slowback.linear.y = 0
cmdvel_slowback.linear.z = 0
cmdvel_slowback.angular.x = 0
cmdvel_slowback.angular.y = 0
cmdvel_slowback.angular.z = 0

# 慢速正转
cmdvel_slowzuotwist = Twist()
cmdvel_slowzuotwist.linear.x = 0
cmdvel_slowzuotwist.linear.y = 0
cmdvel_slowzuotwist.linear.z = 0
cmdvel_slowzuotwist.angular.x = 0
cmdvel_slowzuotwist.angular.y = 0
cmdvel_slowzuotwist.angular.z = 1.2

# 慢速反转
cmdvel_slowyoutwist = Twist()
cmdvel_slowyoutwist.linear.x = 0
cmdvel_slowyoutwist.linear.y = 0
cmdvel_slowyoutwist.linear.z = 0
cmdvel_slowyoutwist.angular.x = 0
cmdvel_slowyoutwist.angular.y = 0
cmdvel_slowyoutwist.angular.z = -1.2

# 中速向前命令
cmdvel_middlefront = Twist()
cmdvel_middlefront.linear.x = 0.6
cmdvel_middlefront.linear.y = 0
cmdvel_middlefront.linear.z = 0
cmdvel_middlefront.angular.x = 0
cmdvel_middlefront.angular.y = 0
cmdvel_middlefront.angular.z = 0

# 中速向后命令
cmdvel_middleback = Twist()
cmdvel_middleback.linear.x = -0.5
cmdvel_middleback.linear.y = 0
cmdvel_middleback.linear.z = 0
cmdvel_middleback.angular.x = 0
cmdvel_middleback.angular.y = 0
cmdvel_middleback.angular.z = 0

# 最快速向前命令
cmdvel_fastfront = Twist()
cmdvel_fastfront.linear.x = 3.5
cmdvel_fastfront.linear.y = 0
cmdvel_fastfront.linear.z = 0
cmdvel_fastfront.angular.x = 0
cmdvel_fastfront.angular.y = 0
cmdvel_fastfront.angular.z = 0
# 最快速向左命令
cmdvel_fastleft = Twist()
cmdvel_fastleft.linear.x = 0
cmdvel_fastleft.linear.y = 3.5
cmdvel_fastleft.linear.z = 0
cmdvel_fastleft.angular.x = 0
cmdvel_fastleft.angular.y = 0
cmdvel_fastleft.angular.z = 0
# 定义静止命令
cmdvel_stop = Twist()
cmdvel_stop.linear.x = 0
cmdvel_stop.linear.y = 0
cmdvel_stop.linear.z = 0
cmdvel_stop.angular.x = 0
cmdvel_stop.angular.y = 0
cmdvel_stop.angular.z = 0

# 推车命令
cmdvel_blockedpush = Twist()
cmdvel_blockedretreat = Twist()

# for test
buff_is_cutted = False


# TODO reconstrcut the code


class BlackBoard():
    def __init__(self):
        self.sended_close = False
        self.sended_far = False
        self.sended_beated = False
        self.sended_follow = False
        self.sended_patrol = False
        self.sended_blocked = False
        self.follow_flag = 0  # 0:跟随；1：左转；2：右转
        self.patrol_flag = 0  # 0:第一个点; 1：第二个点； 2：第三个点...
        self.patrol_flag1 = 0  # 0:先自旋360； 1：开始ptl
        self.patrol_flag2 = 0  # ptl点
        self.leftarea = False
        self.search_state = False

        self.goallastforptl_x = 0
        self.goallastforptl_y = 0

        self.Blocked = False
        self.Blocked_Prevent_close = False
        self.Blocked_Prevent_far = False
        self.Blocked_Prevent_follow = False
        self.Blocked_Prevent_patrol = False
        self.Blocked_excuteflag = 0  # 0:转正；1：推；2：拯救点
        self.BlockedPoseSaveX = 0
        self.BlockedPoseSaveY = 0
        self.BlockedPoseSaveYaw = 0
        self.Blocked_GoalYaw = 0

        self.get_buff_time = 0  # 到了6s就执行抢符
        self.get_buff_time_flag = False  # 不要重复计时标志位

        # 定义拯救点数组:5个点
        self.PreRescuePoint = ((1.35, 2.6, 45), (2.25, 4.5, -45),
                               (6.65, 2.4, -135), (5.75, 0.5, 135))
        self.RescuePoint = ((4, 2.5, 0), (4.8, 1.8, 179), (3.2, 3.2, 0))
        self.avoidloop_far = 0
        self.avoidloop_follow = 0
        self.avoidloop_patrol = 0


class BuildTree():
    def __init__(self):
        self.blackboard = BlackBoard()
        rate = rospy.Rate(10)
        NORMALBEHAVE = Selector("NORMALBEHAVE")
        CLOSE = Sequence("CLOSE")
        FAR = Sequence("FAR")
        BEATED = Sequence("BEATED")
        PATROL = Sequence("PATROL")
        FOLLOW = Sequence("FOLLOW")
        SEARCH_REGION = Sequence("SEARCH")
        
        
        # NORMALBEHAVE.add_child(CLOSE)
        #NORMALBEHAVE.add_child(FAR)
        #NORMALBEHAVE.add_child(BEATED)
        #NORMALBEHAVE.add_child(PATROL)
        NORMALBEHAVE.add_child(FOLLOW)
        NORMALBEHAVE.add_child(SEARCH_REGION)
        
        ISCLOSE = IsClose("ISCLOSE", blackboard=self.blackboard)
        CLOSESHOOT = CloseShoot("CLOSESHOOT", blackboard=self.blackboard)
        CLOSE.add_child(ISCLOSE)
        CLOSE.add_child(CLOSESHOOT)

        ISPATROL = IsPatrol("ISPATROL", blackboard=self.blackboard)
        RANDOM = Random("RANDOM", blackboard=self.blackboard)
        PATROL.add_child(ISPATROL)
        PATROL.add_child(RANDOM)

        ISSEARCH = IsSearch("ISSERARCH", blackboard=self.blackboard)
        SEARCH = Search("SEARCH", blackboard=self.blackboard)
        SEARCH_REGION.add_child(ISSEARCH)
        SEARCH_REGION.add_child(SEARCH)

        IS_FAR = IsFar("IS_FAR", blackboard=self.blackboard)
        FARSHOOT = FarShoot("FARSHOOT", blackboard=self.blackboard)
        FAR.add_child(IS_FAR)
        FAR.add_child(FARSHOOT)

        IS_BEATED = IsBeated("IS_BEATED", blackboard=self.blackboard)
        BEATEDSHOOT = BeatedShoot("BEATEDSHOOT", blackboard=self.blackboard)
        BEATED.add_child(IS_BEATED)
        BEATED.add_child(BEATEDSHOOT)

        ISFOLLOW = IsFollow("ISFOLLOW", blackboard=self.blackboard)
        FOLLOW_SHOOT = Follow_Shoot("FOLLOW_SHOOT", blackboard=self.blackboard)
        FOLLOW.add_child(ISFOLLOW)
        FOLLOW.add_child(FOLLOW_SHOOT)

        rospy.loginfo("NORMALBEHAVE Tree")
        print_tree(NORMALBEHAVE)
        
        while not rospy.is_shutdown():
            NORMALBEHAVE.status = NORMALBEHAVE.run()
            rate.sleep()


class IsSearch(Task):
    def __init__(self, name, blackboard=None):
        super(IsSearch, self).__init__(name)
        self.name = name
        self.blackboard = blackboard

    def run(self):
        rospy.logwarn('search!!!!!!!!!!!!! distance:  {}, result: {}'.format(env.enemy_pose.enemy_dist,env.detection_result))
        if env.enemy_pose.enemy_dist == 0 and env.detection_result == False: #env.detection_result_stable == False:
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.FAILURE

    def reset(self):
        pass


class Search(Task):
    def __init__(self, name, blackboard=None):
        super(Search, self).__init__(name)
        self.name = name
        self.blackboard = blackboard
        self.location_num = 0
        self.search_region_index = 0
        self.blackboard.Blocked_Prevent_follow = False

    def run(self):# 功能说明:按照自定义顺序巡逻4个区域，进入区域前选取该区域内距当前位置最近的点作为进入点。
        rospy.logwarn("%s, %s" %(env.detection_result, env.enemy_pose.enemy_dist))
        if env.sudden_find == True or env.enemy_pose.enemy_dist != 0:
            controller.send_vel(cmdvel_stop)
            controller.send
            rospy.logwarn('return 1')
            return TaskStatus.FAILURE
        else:
                self.search_order = [np.random.randint(4)] # 自定义搜索顺序
                self.search_region = env.search_regions[self.search_order[0]] # 更新搜索区域
                result = self.searchRegion(self.search_region)
                if result:
                    return TaskStatus.SUCCESS
                else:
                    return TaskStatus.FAILURE

    def searchRegion(self, point_list):    
        for search_goal in point_list:
            if env.sudden_find == True or  env.enemy_pose.enemy_dist != 0:
                controller.send_vel(cmdvel_stop)
                rospy.logwarn('return 2')
                return False
            if env.isActionAvaliable(search_goal[0], search_goal[1], search_goal[2]):
                env.send_goal(env.navgoal)
            else:
                rospy.logwarn('cannot goto point')
        return True 
        
            
class IsClose(Task):
    def __init__(self, name, blackboard=None):
        super(IsClose, self).__init__(name)
        self.name = name
        self.blackboard = blackboard
        
    def run(self):
        if env.enemy_pose.enemy_dist < 4.5 and env.enemy_pose.enemy_dist > 0 and self.blackboard.Blocked_Prevent_close == False:
            self.blackboard.sended_far = False
            self.blackboard.sended_beated = False
            self.blackboard.sended_patrol = False
            self.blackboard.sended_follow = False
            self.blackboard.sended_blocked = False
            self.blackboard.patrol_flag = 0
            self.blackboard.follow_flag = 0
            self.blackboard.patrol_flag1 = 0
            self.blackboard.patrol_flag2 = 0

            self.blackboard.Blocked = False
            self.blackboard.Blocked_Prevent_close = False
            self.blackboard.Blocked_Prevent_far = False
            self.blackboard.Blocked_Prevent_follow = False
            self.blackboard.Blocked_Prevent_patrol = False
            self.blackboard.Blocked_excuteflag = 0

            self.blackboard.avoidloop_far = 0
            self.blackboard.avoidloop_follow = 0
            self.blackboard.avoidloop_patrol = 0
            rospy.loginfo('r0 dist: {} '.format(env.enemy_pose.enemy_dist))
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.FAILURE

    def reset(self):
                pass


class CloseShoot(Task):
    def __init__(self, name, blackboard=None):
        super(CloseShoot, self).__init__(name)
        self.name = name
        self.blackboard = blackboard
        self.goal_x = 0
        self.goal_y = 0
        self.goal_yaw = 0
        self.time1 = 0

    def run(self):
        self.blackboard.goallastforptl_x = env.followgoal_x
        self.blackboard.goallastforptl_y = env.followgoal_y
        if env.enemy_pose.enemy_dist < 4.5:  # 否则后退一些
            if self.blackboard.sended_close == False:
                self.goal_x = env.robot_pose['x']
                self.goal_y = env.robot_pose['y']
                if env.enemy_pose.enemy_yaw > 0:  #枪管在左
                    #self.goal_yaw = env.robot_pose[
                    #    'theta'] + env.gimbal_angle.yaw_angle - 35  #与枪管成30度
                    rospy.loginfo(
                        '---------------------theta-----------------------------------'
                    )
                else:
                    #self.goal_yaw = env.robot_pose[
                    #    'theta'] + env.gimbal_angle.yaw_angle + 35  #与枪管成30度
                    rospy.loginfo(
                        '+++++++++++++++++++++++++++++++++++theta+++++++++++++++++++++++++++++++++++'
                    )
                if self.goal_yaw > 180:
                    self.goal_yaw = self.goal_yaw - 360
                if self.goal_yaw < -180:
                    self.goal_yaw = self.goal_yaw + 360
                if env.isActionAvaliable(self.goal_x, self.goal_y,
                                         self.goal_yaw):  # 指向敌人
                    #env.send_goal(env.navgoal)
                    self.blackboard.sended_close = True
                    rospy.loginfo('r0： turn to enemy!!!!!!!!!! goal is %s, %s'
                                  % (env.enemy_pose.enemy_yaw, self.goal_yaw))
                else:
                    rospy.loginfo('r0: cant turn to enemy %s, %s, %s' %
                                  (self.goal_x, self.goal_y, self.goal_yaw))
            #if rospy.Time.now().secs - self.time1 > 1:
            else:
                self.blackboard.sended_close = False
        else:
            if self.blackboard.sended_close == False:
                #controller.send_vel(cmdvel_slowback)
                self.blackboard.sended_close = True
                self.time1 = rospy.Time.now().secs
                rospy.loginfo('r0: retreat!!!!!!!!!!!!!!!!%s, %s, %s' %
                              (self.goal_x, self.goal_y, self.goal_yaw))
            if rospy.Time.now().secs - self.time1 > 0.2:
                self.blackboard.sended_close = False
                controller.send_vel(cmdvel_stop)
                controller.send_vel(cmdvel_stop)
                rospy.loginfo('retreat end!!!!!!!!!!!!!!!')
        return TaskStatus.SUCCESS


class IsFar(Task):
    def __init__(self, name, blackboard=None):
        super(IsFar, self).__init__(name)
        self.name = name
        self.blackboard = blackboard

    def run(self):
        if env.enemy_pose.enemy_dist > 1.6 and self.blackboard.Blocked_Prevent_far == False:
            self.blackboard.sended_close = False
            self.blackboard.sended_beated = False
            self.blackboard.sended_patrol = False
            self.blackboard.sended_follow = False
            self.blackboard.sended_blocked = False
            self.blackboard.patrol_flag = 0
            self.blackboard.follow_flag = 0
            self.blackboard.patrol_flag1 = 0
            self.blackboard.patrol_flag2 = 0

            self.blackboard.Blocked = False
            self.blackboard.Blocked_Prevent_close = False
            self.blackboard.Blocked_Prevent_far = False
            self.blackboard.Blocked_Prevent_follow = False
            self.blackboard.Blocked_Prevent_patrol = False
            self.blackboard.Blocked_excuteflag = 0

            self.blackboard.avoidloop_follow = 0
            self.blackboard.avoidloop_patrol = 0
            rospy.loginfo('far dist: {}'.format(env.enemy_pose.enemy_dist))
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.FAILURE

    def reset(self):
        pass


class FarShoot(Task):
    def __init__(self, name, blackboard=None):
        super(FarShoot, self).__init__(name)
        self.name = name
        self.blackboard = blackboard
        self.goal_x = 0
        self.goal_y = 0
        self.goal_yaw = 0
        self.time1 = 0
        self.time2 = 0

    def run(self):
        #controller.send_vel(cmdvel_stop)
        self.blackboard.goallastforptl_x = env.followgoal_x
        self.blackboard.goallastforptl_y = env.followgoal_y
        if self.blackboard.sended_far == False and env.enemy_pose.enemy_dist > 0:
            if env.isActionAvaliable(env.goal_x, env.goal_y, env.goal_yaw):
                self.goal_x = env.goal_x
                self.goal_y = env.goal_y
                self.goal_yaw = env.goal_yaw
            else:
                self.goal_x = env.farchoosegoal_x
                self.goal_y = env.farchoosegoal_y
                self.goal_yaw = env.farchoosegoal_yaw
                rospy.loginfo('farchoose known point!!!!!!!!!!!!!!')
            self.blackboard.BlockedPoseSaveX = env.robot_pose['x']
            self.blackboard.BlockedPoseSaveY = env.robot_pose['y']
            self.blackboard.BlockedPoseSaveYaw = env.robot_pose['theta']
            if env.isActionAvaliable(self.goal_x, self.goal_y,
                                     self.goal_yaw):  # 指向敌人
                #env.send_goal(env.navgoal)
                self.time1 = rospy.Time.now().secs
                self.blackboard.sended_far = True
                rospy.loginfo(
                    'r0: Far track!!!!!!!!!!!!!!         goal is %s,%s,%s ;enemy is in %s, %s '
                    % (self.goal_x, self.goal_y, self.goal_yaw,
                       env.enemy_pose.enemy_dist, env.enemy_pose.enemy_yaw))
            else:
                self.time1 = rospy.Time.now().secs
                rospy.loginfo('r0: cant track %s, %s, %s' %
                              (self.goal_x, self.goal_y, self.goal_yaw))
        else:
                if rospy.Time.now().secs - self.time1 > 0.3:
                        self.blackboard.sended_far = False
           # env.enemy_pose.enemy_dist = 0

        return TaskStatus.SUCCESS

    def reset(self):
        pass


class IsBeated(Task):
    def __init__(self, name, blackboard=None):
        super(IsBeated, self).__init__(name)
        self.name = name
        self.blackboard = blackboard

    def run(self):
        if env.which_armor > 0:
            self.blackboard.sended_close = False
            self.blackboard.sended_far = False
            self.blackboard.sended_patrol = False
            self.blackboard.sended_follow = False
            self.blackboard.sended_blocked = False
            self.blackboard.patrol_flag = 0
            self.blackboard.follow_flag = 0
            self.blackboard.patrol_flag1 = 0
            self.blackboard.patrol_flag2 = 0

            self.blackboard.Blocked = False
            self.blackboard.Blocked_Prevent_close = False
            self.blackboard.Blocked_Prevent_far = False
            self.blackboard.Blocked_Prevent_follow = False
            self.blackboard.Blocked_Prevent_patrol = False
            self.blackboard.Blocked_excuteflag = 0

            self.blackboard.avoidloop_far = 0
            self.blackboard.avoidloop_follow = 0
            self.blackboard.avoidloop_patrol = 0
            rospy.loginfo('r0 beated!!!!!!!!!!!!!!!!!!!!!!!')
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.FAILURE

    def reset(self):
        pass


class BeatedShoot(Task):
    def __init__(self, name, blackboard=None):
        super(BeatedShoot, self).__init__(name)
        self.name = name
        self.blackboard = blackboard
        self.goal_x = 0
        self.goal_y = 0
        self.goal_yaw = 0
        self.time1 = 0

    def run(self):
        if self.blackboard.sended_beated == False:
            self.goal_x = env.robot_pose['x']
            self.goal_y = env.robot_pose['y']
            if env.which_armor == 0:  # 主 beated
                if env.enemy_pose_last >= 0:
                    self.goal_yaw = env.robot_pose['theta'] + 30
                else:
                    self.goal_yaw = env.robot_pose['theta'] - 30
            elif env.which_armor == 1:  # 左
                self.goal_yaw = env.robot_pose['theta'] + 120
            elif env.which_armor == 2:  # 后
                self.goal_yaw = env.robot_pose['theta'] + 178
            elif env.which_armor == 3:  #右
                self.goal_yaw = env.robot_pose['theta'] - 120
            elif env.which_armor == 4:  #左摄像头
                self.goal_yaw = env.robot_pose['theta'] + 130
                rospy.loginfo('camera zuo move!!!!!!!!!!11')
            elif env.which_armor == 5:  #右摄像头
                self.goal_yaw = env.robot_pose['theta'] - 130
                rospy.loginfo('camera you move!!!!!!!!!!')
            else:
                rospy.loginfo('armor detect error!!!!!!!!!!!!')
            # 防止过度
            if self.goal_yaw > 180:
                self.goal_yaw = self.goal_yaw - 360
            elif self.goal_yaw < -180:
                self.goal_yaw = self.goal_yaw + 360
            else:
                pass
            if env.isActionAvaliable(self.goal_x, self.goal_y,
                                     self.goal_yaw):  # 指向敌人
                env.send_goal(env.navgoal)
                self.blackboard.sended_beated = True
                self.time1 = rospy.Time.now().secs
                rospy.loginfo( 'r0： beated move!!!!!!!!!!')
            else:
                rospy.loginfo('r0: cant beated move %s, %s, %s' %
                              (self.goal_x, self.goal_y, self.goal_yaw))
        if rospy.Time.now().secs - self.time1 > 3.5:
            self.blackboard.sended_beated = False
            env.which_armor = -1
        return TaskStatus.SUCCESS

    def reset(self):
        pass


class IsFollow(Task):
    def __init__(self, name, blackboard=None):
        super(IsFollow, self).__init__(name)
        self.name = name
        self.blackboard = blackboard

    def run(self):
        if env.enemy_pose.enemy_dist > 0:#  and self.blackboard.Blocked_Prevent_follow == False:
            self.blackboard.Blocked = False
            self.blackboard.Blocked_Prevent_follow = False
            #rospy.loginfo('FALLOW: {}'.format(env.enemy_pose.enemy_dist))
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.FAILURE

    def reset(self):
        pass


class Follow_Shoot(Task):
    def __init__(self, name, blackboard=None):
        super(Follow_Shoot, self).__init__(name)
        self.name = name
        self.blackboard = blackboard
        self.goal_x = 0
        self.goal_y = 0
        self.goal_yaw = 0
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.keep_length = 0
        self.time1 = 0

    def run(self):
        #rospy.logwarn('enter follow_shoot')
        if self.blackboard.follow_flag == 0:
            if env.isActionAvaliable(env.robot_pose['x'],env.robot_pose['y'], 
                                            math.degrees(env.enemy_pose.enemy_yaw)):# 指向敌人
                env.send_goal(env.navgoal)
            self.count = -1
            self.change_angle = 0
            self.goal_x = 0
            self.goal_y = 0
            self.goal_yaw = 0
            # if self.blackboard.sended_follow == False:
            while env.isActionAvaliable(self.goal_x, self.goal_y, self.goal_yaw) == False:# and env.num_not_detected_enemy < 5:
                self.count += 1
                if self.count > 6:
                    rospy.logwarn('!!!!!!  cant follow  !!!!!!!')
                    return TaskStatus.FAILURE               
                self.change_angle = math.radians(15 * (int(self.count) / 2)) # 旋转角度 radian
                rospy.loginfo("%s" % math.degrees(self.change_angle))
                
                if env.enemy_pose.enemy_dist > 0 and env.enemy_pose.enemy_dist < 0.6:
                    self.keep_length = 0.6  # 跟随距离
                elif env.enemy_pose.enemy_dist >= 0.6 and env.enemy_pose.enemy_dist < 0.9:
                    return TaskStatus.SUCCESS
                elif env.enemy_pose.enemy_dist >= 0.9 and env.enemy_pose.enemy_dist < 4:
                    self.keep_length = 0.9
    
                if self.count % 2 == 1:      #顺时针
                    rospy.loginfo("what !!!!!!!!!!")
                    if env.enemy_pose.enemy_yaw > 0 and env.enemy_pose.enemy_yaw < math.pi/2:
                        self.y = env.robot_pose['y'] + env.enemy_pose.enemy_dist * (math.sin(self.yaw) - math.cos(self.yaw) * math.tanh(self.yaw - self.change_angle))
                        self.x = env.robot_pose['x']
                        self.dist = env.enemy_pose.enemy_dist * math.cos(self.yaw) / math.cos(self.yaw - self.change_angle)
                        self.yaw = env.enemy_pose.enemy_yaw - self.change_angle # radian

                    elif env.enemy_pose.enemy_yaw > math.pi/2 and env.enemy_pose.enemy_yaw < math.pi:
                        self.x = env.robot_pose['x'] - env.enemy_pose.enemy_dist * (math.sin(self.yaw - math.pi/2) - math.cos(self.yaw - math.pi/2) * math.tanh(self.yaw - math.pi/2 - self.change_angle))
                        self.y = env.robot_pose['y']
                        self.dist = env.enemy_pose.enemy_dist * math.cos(self.yaw - math.pi/2) / math.cos(self.yaw - math.pi/2 - self.change_angle)
                        self.yaw = env.enemy_pose.enemy_yaw - self.change_angle

                    elif env.enemy_pose.enemy_yaw > -math.pi/2 and env.enemy_pose.enemy_yaw < 0:
                        self.x = env.robot_pose['x'] + env.enemy_pose.enemy_dist * (math.sin(math.pi/2 + self.yaw) - math.cos(math.pi/2 + self.yaw) * math.tanh(math.pi/2 + self.yaw -self.change_angle))
                        self.y = env.robot_pose['y']
                        self.dist = env.enemy_pose.enemy_dist * math.cos(self.yaw + math.pi/2) / math.cos(self.yaw - self.change_angle)
                        self.yaw = env.enemy_pose.enemy_yaw - self.change_angle

                    else:
                        self.y = env.robot_pose['y'] - env.enemy_pose.enemy_dist * (math.sin(math.pi + self.yaw) - math.cos(math.pi + self.yaw) * math.tanh(math.pi + self.yaw -self.change_angle))
                        self.x = env.robot_pose['x']
                        self.dist = env.enemy_pose.enemy_dist * math.cos(self.yaw + math.pi) / math.cos(self.yaw + math.pi - self.change_angle)
                        self.yaw = env.enemy_pose.enemy_yaw - self.change_angle

                else:
                    rospy.loginfo("oh  yeah !!!!!!!!!!")
                    if env.enemy_pose.enemy_yaw > 0 and env.enemy_pose.enemy_yaw < math.pi/2:  #逆时针
                        self.x = env.robot_pose['x'] + env.enemy_pose.enemy_dist * (math.sin(math.pi/2 - self.yaw) - math.cos(math.pi/2 - self.yaw) * math.tanh(math.pi/2 - self.yaw - self.change_angle))
                        self.y = env.robot_pose['y']
                        self.dist = env.enemy_pose.enemy_dist * math.cos(-self.yaw + math.pi/2) / math.cos(math.pi/2 - self.yaw - self.change_angle)
                        self.yaw = env.enemy_pose.enemy_yaw + self.change_angle

                    elif env.enemy_pose.enemy_yaw > math.pi/2 and env.enemy_pose.enemy_yaw < math.pi:
                        self.y = env.robot_pose['y'] + env.enemy_pose.enemy_dist * (math.sin(math.pi - self.yaw) - math.cos(math.pi- self.yaw) * math.tanh(math.pi - self.yaw - self.change_angle))
                        self.x = env.robot_pose['x']
                        self.dist = env.enemy_pose.enemy_dist * math.cos(-self.yaw + math.pi) / math.cos(math.pi - self.yaw - self.change_angle)
                        self.yaw = env.enemy_pose.enemy_yaw + self.change_angle
                        
                    elif env.enemy_pose.enemy_yaw > -math.pi/2 and env.enemy_pose.enemy_yaw < 0:
                        self.y = env.robot_pose['y'] - env.enemy_pose.enemy_dist * (math.sin(-self.yaw) - math.cos(-self.yaw) * math.tanh(-self.yaw -self.change_angle))
                        self.x = env.robot_pose['x']
                        self.dist = env.enemy_pose.enemy_dist * math.cos(-self.yaw) / math.cos(-self.yaw - self.change_angle)
                        self.yaw = env.enemy_pose.enemy_yaw + self.change_angle
                        
                    else:
                        self.x = env.robot_pose['x'] - env.enemy_pose.enemy_dist * (math.sin(-self.yaw - math.pi/2) - math.cos(-self.yaw - math.pi/2) * math.tanh(-self.yaw - math.pi/2 -self.change_angle))
                        self.y = env.robot_pose['y']
                        self.dist = env.enemy_pose.enemy_dist * math.cos(-self.yaw - math.pi/2) / math.cos(-self.yaw - math.pi/2 - self.change_angle)
                        self.yaw = env.enemy_pose.enemy_yaw + self.change_angle
                self.dist = self.dist - self.keep_length
                self.dx = self.dist * math.cos(self.yaw)
                self.dy = self.dist * math.sin(self.yaw)
                self.goal_x = self.x + self.dx
                self.goal_y = self.y + self.dy
                self.goal_yaw = math.degrees(self.yaw)
            # if env.num_not_detected_enemy > 5:
            #     rospy.loginfo("mad !!!!")
            #     return TaskStatus.FAILURE
            if env.isActionAvaliable(self.goal_x, self.goal_y,
                                         self.goal_yaw):  # 判断目标点是否可行
                env.send_goal(env.navgoal)
            self.blackboard.BlockedPoseSaveX = env.robot_pose['x']
            self.blackboard.BlockedPoseSaveY = env.robot_pose['y']
            self.blackboard.BlockedPoseSaveYaw = env.robot_pose['theta']                                
            self.time1 = rospy.Time.now().secs
            rospy.loginfo('r0: is follow !!!!!!!     %s,%s,%s' %
                                  (self.goal_x, self.goal_y, self.goal_yaw))
            self.blackboard.sended_follow = True

            self.time1 = rospy.Time.now().secs
            # if rospy.Time.now().secs - self.time1 > 2:
            #     # 0.1m不走10度不转
            #     if np.square(self.blackboard.BlockedPoseSaveX -
            #                  env.robot_pose['x']) + np.square(
            #                      self.blackboard.BlockedPoseSaveY -
            #                      env.robot_pose['y']) < 0.0225 and np.abs(
            #                          self.blackboard.BlockedPoseSaveYaw -
            #                          env.robot_pose['theta']) < 10:
            #         self.blackboard.avoidloop_follow = self.blackboard.avoidloop_follow + 1
            #         rospy.logwarn('follow blocked!!!!!!!,%s' %
            #                       (self.blackboard.avoidloop_follow))
            #         # self.blackboard.Blocked_GoalYaw = np.arctan2(
            #         #     (self.goal_y - env.robot_pose['y']),
            #         #     self.goal_x - env.robot_pose['x']) * 180 / 3.1416
            #         # self.blackboard.Blocked = True
            #         # self.blackboard.Blocked_Prevent_follow = True
            #         return TaskStatus.FAILURE
            #     else:
            #         self.blackboard.sended_follow = False
            #         if np.square(
            #                 self.goal_x - env.robot_pose['x']) + np.square(
            #                     self.goal_y - env.robot_pose['y']) < 0.01:  #到了
            #             #self.blackboard.follow_flag = 1
            #             rospy.loginfo('has reached goal')
            return TaskStatus.SUCCESS
    '''    
        elif self.blackboard.follow_flag == 1:
            if self.blackboard.sended_follow == False:
                self.goal_x = env.goal_x
                self.goal_y = env.goal_y
                self.goal_yaw = env.goal_yaw + 30
                if self.goal_yaw > 180:
                    self.goal_yaw = self.goal_yaw - 360
                if env.isActionAvaliable(self.goal_x, self.goal_y,
                                         self.goal_yaw):  # 判断目标点是否可行
                    env.send_goal(env.navgoal)
                    rospy.loginfo('r0: is follow !!!!!!!!!!!!!!     %s,%s,%s' %
                                  (self.goal_x, self.goal_y, self.goal_yaw))
                    self.time1 = rospy.Time.now().secs
                    self.blackboard.sended_follow = True
                else:
                    rospy.loginfo('r0 cant move %s, %s, %s' %
                                  (self.goal_x, self.goal_y, self.goal_yaw))
            if rospy.Time.now().secs - self.time1 > 2.5:
                self.blackboard.sended_follow = False
                self.blackboard.follow_flag = 2
            else:
                return TaskStatus.SUCCESS
        elif self.blackboard.follow_flag == 2:
            if self.blackboard.sended_follow == False:
                self.goal_x = env.goal_x
                self.goal_y = env.goal_y
                self.goal_yaw = env.goal_yaw
                self.goal_yaw = self.goal_yaw - 30
                if self.goal_yaw < -180:
                    self.goal_yaw = self.goal_yaw + 360
                if env.isActionAvaliable(self.goal_x, self.goal_y,
                                         self.goal_yaw):  # 判断目标点是否可行
                    env.send_goal(env.navgoal)
                    rospy.loginfo('r0: is follow !!!!!!!!!!!!!!     %s,%s,%s' %
                                  (self.goal_x, self.goal_y, self.goal_yaw))
                    self.time1 = rospy.Time.now().secs
                    self.blackboard.sended_follow = True
                else:
                    rospy.loginfo('r0 cant move %s, %s, %s' %
                                  (self.goal_x, self.goal_y, self.goal_yaw))
            if rospy.Time.now().secs - self.time1 > 2.5:
                self.blackboard.sended_follow = False
                self.blackboard.follow_flag = 0
            else:
                return TaskStatus.SUCCESS
        else:
            rospy.logwarn('follow error!!!!!!!!!!!!!')
    '''

    def reset(self):
        pass



class IsPatrol(Task):
    def __init__(self, name, blackboard=None):
        super(IsPatrol, self).__init__(name)
        self.name = name
        self.blackboard = blackboard

    def run(self):
        if env.enemy_pose.enemy_dist == 0 and self.blackboard.Blocked_Prevent_patrol == False:
            self.blackboard.sended_far = False
            self.blackboard.sended_close = False
            self.blackboard.sended_beated = False
            self.blackboard.sended_follow = False
            self.blackboard.sended_blocked = False
            self.blackboard.follow_flag = 0

            self.blackboard.Blocked = False
            self.blackboard.Blocked_Prevent_close = False
            self.blackboard.Blocked_Prevent_far = False
            self.blackboard.Blocked_Prevent_follow = False
            self.blackboard.Blocked_Prevent_patrol = False
            self.blackboard.Blocked_excuteflag = 0

            self.blackboard.avoidloop_far = 0
            self.blackboard.avoidloop_follow = 0

            print 'r0 dist ==0; r1 dist == 0'
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.FAILURE

    def reset(self):
        pass


class Random(Task):
    def __init__(self, name, blackboard=None):
        super(Random, self).__init__(name)
        self.name = name
        self.count = 0
        self.patrol_yaw = True
        self.blackboard = blackboard
        self.goal_x = 0
        self.goal_y = 0
        self.goal_yaw = 0
        self.time1 = 0

    def run(self):
        if self.blackboard.patrol_flag1 == 0:  #先自转
            if self.blackboard.goallastforptl_x >= env.robot_pose[
                    'x'] and self.blackboard.goallastforptl_y >= env.robot_pose[
                        'y']:
                self.goal_x = env.robot_pose['x'] - 0.15
                self.goal_y = env.robot_pose['y'] - 0.15
                print 'ptl1 ttttttttttttttttttttttttttttttttttttttt'
            elif self.blackboard.goallastforptl_x >= env.robot_pose[
                    'x'] and self.blackboard.goallastforptl_y < env.robot_pose[
                        'y']:
                self.goal_x = env.robot_pose['x'] - 0.15
                self.goal_y = env.robot_pose['y'] + 0.15
                print 'ptl2 ttttttttttttttttttttttttttttttttttttttttttt'
            elif self.blackboard.goallastforptl_x < env.robot_pose[
                    'x'] and self.blackboard.goallastforptl_y >= env.robot_pose[
                        'y']:
                self.goal_x = env.robot_pose['x'] + 0.15
                self.goal_y = env.robot_pose['y'] - 0.15
                print 'ptl3 ttttttttttttttttttttttttttttttttttttttttttt'
            elif self.blackboard.goallastforptl_x < env.robot_pose[
                    'x'] and self.blackboard.goallastforptl_y < env.robot_pose[
                        'y']:
                self.goal_x = env.robot_pose['x'] + 0.15
                self.goal_y = env.robot_pose['y'] + 0.15
                print 'ptl4 tttttttttttttttttttttttttttttttttttttttt'
            else:
                print 'ptl1 error!!!!!!!!!!!!!!!!'
            if env.isActionAvaliable(self.goal_x, self.goal_y, 0):  # front1
                self.goal_x = env.robot_pose['x']
                self.goal_y = env.robot_pose['y']
            else:
                self.goal_x = env.robot_pose['x']
                self.goal_y = env.robot_pose['y']
            if env.enemy_pose_last >= 0:  #左转
                if self.blackboard.patrol_flag2 == 0:  # 第一次转60度
                    self.goal_yaw = env.robot_pose['theta'] + 135  #一次转90度；4次
                elif self.blackboard.patrol_flag2 == 1:  # 第2次转90度
                    self.goal_yaw = env.robot_pose['theta'] + 135  # 一次转90度；4次
                elif self.blackboard.patrol_flag2 == 2:  # 第3次转90度
                    self.goal_yaw = env.robot_pose['theta'] + 135  #一次转90度；4次
                else:
                    print 'zixuan error!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
            else:  #右转
                if self.blackboard.patrol_flag2 == 0:  # 第一次转60度
                    self.goal_yaw = env.robot_pose['theta'] - 135  #一次转90度；4次
                elif self.blackboard.patrol_flag2 == 1:  # 第2次转90度
                    self.goal_yaw = env.robot_pose['theta'] - 135  # 一次转90度；4次
                elif self.blackboard.patrol_flag2 == 2:  # 第3次转90度
                    self.goal_yaw = env.robot_pose['theta'] - 135  #一次转90度；4次
                else:
                    print 'zixuan error!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
            if self.goal_yaw > 180:
                self.goal_yaw = self.goal_yaw - 360
            if self.goal_yaw < -180:
                self.goal_yaw = self.goal_yaw + 360
            if self.blackboard.sended_patrol == False:
                if env.isActionAvaliable(self.goal_x, self.goal_y,
                                         self.goal_yaw):  # front1
                    env.send_goal(env.navgoal)
                    print 'r0 is patrol_spin; goal is %s, %s, %s' % (
                        self.goal_x, self.goal_y, self.goal_yaw)
                    self.time1 = rospy.Time.now().secs
                    self.blackboard.sended_patrol = True
            else:
                if rospy.Time.now().secs - self.time1 > 2.5 or np.abs(
                        env.robot_pose['theta'] - self.goal_yaw) < 10:
                    self.blackboard.sended_patrol = False
                    self.blackboard.patrol_flag2 = self.blackboard.patrol_flag2 + 1
                    if self.blackboard.patrol_flag2 == 3:
                        self.blackboard.patrol_flag2 = 0
                        self.blackboard.patrol_flag1 = 1  # 旋转结束
                        print 'zixuan end!!!!!!!!!!!!!!!!!!!!!!!!'
        else:  #ptl
            if self.blackboard.patrol_flag == 0:  # 第一点：移动
                if self.blackboard.sended_patrol == False:
                    if env.HP_robot1 > 0:  #另一台车是活的
                        self.goal_x = 6
                        self.goal_y = 1.8
                        self.goal_yaw = 155
                    else:  #另车是死的
                        if self.blackboard.leftarea == False:  #走you边
                            self.goal_x = 6
                            self.goal_y = 1.8
                            self.goal_yaw = 155
                        else:
                            self.goal_x = 2
                            self.goal_y = 3.2
                            self.goal_yaw = -35
                    self.blackboard.BlockedPoseSaveX = env.robot_pose['x']
                    self.blackboard.BlockedPoseSaveY = env.robot_pose['y']
                    self.blackboard.BlockedPoseSaveYaw = env.robot_pose[
                        'theta']
                    if env.isActionAvaliable(self.goal_x, self.goal_y,
                                             self.goal_yaw):  # front1
                        env.send_goal(env.navgoal)
                        print 'r0 is patrol_front; goal is %s, %s, %s' % (
                            self.goal_x, self.goal_y, self.goal_yaw)
                        self.time1 = rospy.Time.now().secs
                        self.blackboard.sended_patrol = True
                else:
                    if rospy.Time.now().secs - self.time1 > 2.5:
                        if np.square(self.blackboard.BlockedPoseSaveX -
                                     env.robot_pose['x']) + np.square(
                                         self.blackboard.BlockedPoseSaveY - env
                                         .robot_pose['y']) < 0.0225 and np.abs(
                                             self.blackboard.BlockedPoseSaveYaw
                                             - env.robot_pose['theta']) < 10:
                            self.blackboard.avoidloop_patrol = self.blackboard.avoidloop_patrol + 1
                            print 'ptlfront blocked!!!!!!!  %s' % (
                                self.blackboard.avoidloop_patrol)
                            self.blackboard.Blocked_GoalYaw = np.arctan2(
                                (self.goal_y - env.robot_pose['y']),
                                self.goal_x -
                                env.robot_pose['x']) * 180 / 3.1416
                            self.blackboard.Blocked = True
                            self.blackboard.Blocked_Prevent_patrol = True
                        else:
                            self.blackboard.sended_patrol = False
                            if np.square(self.goal_x -
                                         env.robot_pose['x']) + np.square(
                                             self.goal_y - env.robot_pose['y']
                                         ) < 0.0169:  # 到了
                                self.blackboard.patrol_flag = 1
            elif self.blackboard.patrol_flag == 1:  #转
                if self.blackboard.sended_patrol == False:
                    if env.HP_robot1 > 0:
                        self.goal_x = 6.2
                        self.goal_y = 1.8
                        self.goal_yaw = -145
                    else:  #另车是死的
                        if self.blackboard.leftarea == False:  #走you边
                            self.goal_x = 6.2
                            self.goal_y = 1.8
                            self.goal_yaw = -145
                        else:
                            self.goal_x = 1.8
                            self.goal_y = 3.2
                            self.goal_yaw = 35
                    if env.isActionAvaliable(self.goal_x, self.goal_y,
                                             self.goal_yaw):  # front1
                        env.send_goal(env.navgoal)
                        print 'r0 is patrol_front1; goal is %s, %s, %s' % (
                            self.goal_x, self.goal_y, self.goal_yaw)
                        self.time1 = rospy.Time.now().secs
                        self.blackboard.sended_patrol = True
                else:
                    if rospy.Time.now().secs - self.time1 > 3:
                        self.blackboard.sended_patrol = False
                        self.blackboard.patrol_flag = 2
            elif self.blackboard.patrol_flag == 2:  #转
                if self.blackboard.sended_patrol == False:
                    if env.HP_robot1 > 0:
                        self.goal_x = 6.1
                        self.goal_y = 1.8
                        self.goal_yaw = 0
                    else:  #另车是死的
                        if self.blackboard.leftarea == False:  #走you边
                            self.goal_x = 6.1
                            self.goal_y = 1.8
                            self.goal_yaw = 0
                        else:
                            self.goal_x = 1.9
                            self.goal_y = 3.2
                            self.goal_yaw = 178
                    if env.isActionAvaliable(self.goal_x, self.goal_y,
                                             self.goal_yaw):  # front1
                        env.send_goal(env.navgoal)
                        print 'r0 is patrol_front2; goal is %s, %s, %s' % (
                            self.goal_x, self.goal_y, self.goal_yaw)
                        self.time1 = rospy.Time.now().secs
                        self.blackboard.sended_patrol = True
                else:
                    if rospy.Time.now().secs - self.time1 > 3:
                        self.blackboard.sended_patrol = False
                        self.blackboard.patrol_flag = 3
            elif self.blackboard.patrol_flag == 3:  #移动
                if self.blackboard.sended_patrol == False:
                    if env.HP_robot1 > 0:
                        self.goal_x = 7.1
                        self.goal_y = 1.9
                        self.goal_yaw = -75
                    else:  #另车是死的
                        if self.blackboard.leftarea == False:  #走you边
                            self.goal_x = 7.1
                            self.goal_y = 1.9
                            self.goal_yaw = -75
                        else:
                            self.goal_x = 0.9
                            self.goal_y = 3.1
                            self.goal_yaw = 105
                    self.blackboard.BlockedPoseSaveX = env.robot_pose['x']
                    self.blackboard.BlockedPoseSaveY = env.robot_pose['y']
                    self.blackboard.BlockedPoseSaveYaw = env.robot_pose[
                        'theta']
                    if env.isActionAvaliable(self.goal_x, self.goal_y,
                                             self.goal_yaw):  # front1
                        env.send_goal(env.navgoal)
                        print 'r0 is patrol_back1; goal is %s, %s, %s' % (
                            self.goal_x, self.goal_y, self.goal_yaw)
                        self.time1 = rospy.Time.now().secs
                        self.blackboard.sended_patrol = True
                else:
                    if rospy.Time.now().secs - self.time1 > 2:
                        if np.square(self.blackboard.BlockedPoseSaveX -
                                     env.robot_pose['x']) + np.square(
                                         self.blackboard.BlockedPoseSaveY - env
                                         .robot_pose['y']) < 0.0225 and np.abs(
                                             self.blackboard.BlockedPoseSaveYaw
                                             - env.robot_pose['theta']) < 10:
                            self.blackboard.avoidloop_patrol = self.blackboard.avoidloop_patrol + 1
                            print 'ptlback1 blocked!!!!!!!  %s' % (
                                self.blackboard.avoidloop_patrol)
                            self.blackboard.Blocked_GoalYaw = np.arctan2(
                                (self.goal_y - env.robot_pose['y']),
                                self.goal_x -
                                env.robot_pose['x']) * 180 / 3.1416
                            self.blackboard.Blocked = True
                            self.blackboard.Blocked_Prevent_patrol = True
                        else:
                            self.blackboard.sended_patrol = False
                            if np.square(self.goal_x -
                                         env.robot_pose['x']) + np.square(
                                             self.goal_y - env.robot_pose['y']
                                         ) < 0.0225:  # 到了
                                self.blackboard.patrol_flag = 4

            elif self.blackboard.patrol_flag == 4:  #移动
                if self.blackboard.sended_patrol == False:
                    if env.HP_robot1 > 0:
                        self.goal_x = 6.65
                        self.goal_y = 3
                        self.goal_yaw = 70
                    else:  # 另车是死的
                        if self.blackboard.leftarea == False:  # 走you边
                            self.goal_x = 6.65
                            self.goal_y = 3
                            self.goal_yaw = 70
                        else:
                            self.goal_x = 1.35
                            self.goal_y = 2
                            self.goal_yaw = -110
                    self.blackboard.BlockedPoseSaveX = env.robot_pose['x']
                    self.blackboard.BlockedPoseSaveY = env.robot_pose['y']
                    self.blackboard.BlockedPoseSaveYaw = env.robot_pose[
                        'theta']
                    if env.isActionAvaliable(self.goal_x, self.goal_y,
                                             self.goal_yaw):  # front1
                        env.send_goal(env.navgoal)
                        print 'r0 is patrol_back2; goal is %s, %s, %s' % (
                            self.goal_x, self.goal_y, self.goal_yaw)
                        self.time1 = rospy.Time.now().secs
                        self.blackboard.sended_patrol = True
                else:
                    if rospy.Time.now().secs - self.time1 > 2.8:
                        if np.square(self.blackboard.BlockedPoseSaveX -
                                     env.robot_pose['x']) + np.square(
                                         self.blackboard.BlockedPoseSaveY - env
                                         .robot_pose['y']) < 0.0225 and np.abs(
                                             self.blackboard.BlockedPoseSaveYaw
                                             - env.robot_pose['theta']) < 10:
                            self.blackboard.avoidloop_patrol = self.blackboard.avoidloop_patrol + 1
                            print 'ptlback2 blocked!!!!!!!  %s' % (
                                self.blackboard.avoidloop_patrol)
                            self.blackboard.Blocked_GoalYaw = np.arctan2(
                                (self.goal_y - env.robot_pose['y']),
                                self.goal_x -
                                env.robot_pose['x']) * 180 / 3.1416
                            self.blackboard.Blocked = True
                            self.blackboard.Blocked_Prevent_patrol = True
                        else:
                            self.blackboard.sended_patrol = False
                            if np.square(self.goal_x -
                                         env.robot_pose['x']) + np.square(
                                             self.goal_y - env.robot_pose['y']
                                         ) < 0.0225:  # 到了
                                self.blackboard.patrol_flag = 5
            elif self.blackboard.patrol_flag == 5:  #移动
                if self.blackboard.sended_patrol == False:
                    if env.HP_robot1 > 0:
                        self.goal_x = 5.8
                        self.goal_y = 1.8
                        self.goal_yaw = 150
                    else:  # 另车是死的
                        if self.blackboard.leftarea == False:  # 走you边
                            self.goal_x = 5.8
                            self.goal_y = 1.8
                            self.goal_yaw = 150
                        else:
                            self.goal_x = 2.2
                            self.goal_y = 3.2
                            self.goal_yaw = -30
                    self.blackboard.BlockedPoseSaveX = env.robot_pose['x']
                    self.blackboard.BlockedPoseSaveY = env.robot_pose['y']
                    self.blackboard.BlockedPoseSaveYaw = env.robot_pose[
                        'theta']
                    if env.isActionAvaliable(self.goal_x, self.goal_y,
                                             self.goal_yaw):  # front1
                        env.send_goal(env.navgoal)
                        print 'r0 is patrol_behind; goal is %s, %s, %s' % (
                            self.goal_x, self.goal_y, self.goal_yaw)
                        self.time1 = rospy.Time.now().secs
                        self.blackboard.sended_patrol = True
                else:
                    if rospy.Time.now().secs - self.time1 > 2:
                        if np.square(self.blackboard.BlockedPoseSaveX -
                                     env.robot_pose['x']) + np.square(
                                         self.blackboard.BlockedPoseSaveY - env
                                         .robot_pose['y']) < 0.0225 and np.abs(
                                             self.blackboard.BlockedPoseSaveYaw
                                             - env.robot_pose['theta']) < 10:
                            self.blackboard.avoidloop_patrol = self.blackboard.avoidloop_patrol + 1
                            print 'ptlback blocked!!!!!!!  %s' % (
                                self.blackboard.avoidloop_patrol)
                            self.blackboard.Blocked_GoalYaw = np.arctan2(
                                (self.goal_y - env.robot_pose['y']),
                                self.goal_x -
                                env.robot_pose['x']) * 180 / 3.1416
                            self.blackboard.Blocked = True
                            self.blackboard.Blocked_Prevent_patrol = True
                        else:
                            self.blackboard.sended_patrol = False
                            if np.square(self.goal_x -
                                         env.robot_pose['x']) + np.square(
                                             self.goal_y - env.robot_pose['y']
                                         ) < 0.0225:  # 到了
                                self.blackboard.patrol_flag = 0
                                self.blackboard.leftarea = ~self.blackboard.leftarea
            else:
                print 'patrol error!!!'
        return TaskStatus.SUCCESS



if __name__ == '__main__':
    rospy.loginfo('init')
    rospy.init_node('decision_node')
    controller = Controller()
    env = BattleEnv()
    tflistener = tf.TransformListener()
    controller.global_path_planner_action_client.wait_for_server(rospy.Duration(0.5))
    controller.local_path_planner_action_client.wait_for_server(rospy.Duration(0.5))

    rospy.loginfo('Start the RM!!!!!!!!!!!!!!!!!!')
    goal_x = 6.3
    goal_y = 1.8
    goal_yaw = 90
        
    #goal_x = 4.12
    #goal_y = 0.62
    #goal_yaw = -90
    #if env.isActionAvaliable(goal_x, goal_y, goal_yaw):  # front1
    #    env.send_goal(env.navgoal)
    #    rospy.loginfo('seng goal success')
    #rospy.sleep(5)
    
    
    # goal_x = 1.7
    # goal_y = 3.2
    # goal_yaw = 90
    # if env.isActionAvaliable(goal_x, goal_y, goal_yaw):  # front1
    #    env.send_goal(env.navgoal)
    #    rospy.loginfo('seng goal success')
    
    


    rospy.loginfo('enter tree')
    tree = BuildTree()  # 已经加buff进入tree        
    rospy.spin()
    
    




