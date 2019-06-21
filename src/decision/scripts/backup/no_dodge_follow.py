class IsFollow(py_trees.Behaviour):
    def __init__(self, name, blackboard=None):
        super(IsFollow, self).__init__(name)
        self.name = name
        self.blackboard = blackboard

    def update(self):
        if self.blackboard.EnterFollow == True:
            #rospy.loginfo('FALLOW: {}'.format(env.enemy_pose.enemy_dist))
            return py_trees.Status.SUCCESS
        else:
            return py_trees.Status.FAILURE

class Follow_Shoot(py_trees.Behaviour):
    def __init__(self, name, blackboard=None):
        super(Follow_Shoot, self).__init__(name)
        self.name = name
        self.blackboard = blackboard
        self.goal_x = 0.01
        self.goal_y = 0.01
        self.goal_yaw = 0
        self.yaw = 0
        self.keep_length = 0
        self.time1 = 0
        self.change_angle_list = [0, 0, 5, -5, 10, -10, -17, 17, 24, -24, 32, -32, 40, -40, 50, -50, 60, -60]
        self.no_big_change = False
        self.first_enter_stable = True

    def update(self):
        #rospy.logwarn('enter follow_shoot')
        if env.detection_result == False:
            ctrl.send_vel(TwistControl(0, 0, 0, 0).Twist)
            self.blackboard.EnterFollow = False
            return py_trees.Status.FAILURE
            
        if self.enemy_big_change() == False and env.enemy_pose.enemy_dist < 1.5: 
            self.no_big_change = True
            if self.first_enter_stable == False:
                # self.blackboard.enterDodge()
                if env.beated_num_two_secs > 4:
                    self.blackboard.enterDodge()
                else:
                    self.blackboard.enterTracking()
                    if env.ARMOR_HIT_NUM != -1:
                        env.ARMOR_HIT_NUM = -1
                        if self.normalizeDegree(env.enemy_yaw_chassis) > 0:
                            self.goal_yaw = math.radians(env.robot_pose['theta']) + math.radians(35)
                        else:
                            self.goal_yaw = math.radians(env.robot_pose['theta']) - math.radians(35)
                    elif self.need_change_angle(0, 40) == True: # not hit & need ratate 
                        if self.normalizeDegree(env.enemy_yaw_chassis) > 0: # enemy at left
                            self.goal_yaw = env.enemy_pose.enemy_yaw - math.radians(35)
                        else:                                               # enemy at right
                            self.goal_yaw = env.enemy_pose.enemy_yaw + math.radians(35)
                    if env.isActionAvaliable(env.robot_pose['x'], env.robot_pose['y'], 
                                        self.normalizeDegree(self.goal_yaw)):
                        env.TwitsRotation(math.degrees(self.goal_yaw))
                return py_trees.Status.SUCCESS
            else:
                self.first_enter_stable = False
        else:
            self.first_enter_stable = True
            self.no_big_change = False
        
        self.blackboard.enterTracking()
        self.blackboard.enemy_last_x = env.enemy_position['x']
        self.blackboard.enemy_last_y = env.enemy_position['y']
        self.blackboard.enemy_last_yaw = env.enemy_pose.enemy_yaw
        self.blackboard.enemy_last_remaining_time = env.REMAINING_TIME

        if self.follow_goal(env.enemy_pose.enemy_dist, env.enemy_pose.enemy_yaw, env.enemy_position['x'], 
            env.enemy_position['y'], env.robot_pose['x'], env.robot_pose['y']) == False:
            if self.rescue_follow_goal(env.enemy_position['x'], env.enemy_position['y'], env.enemy_pose.enemy_yaw, 
                    env.robot_pose['x'], env.robot_pose['y']) == False:
                self.blackboard.EnterFollow = False
                return py_trees.Status.FAILURE # 无法跟随 

        if env.isActionAvaliable(self.goal_x, self.goal_y,
                                        self.normalizeDegree(self.goal_yaw)):  # 判断目标点是否可行
            env.enemy_follow_x = env.enemy_position['x']
            env.enemy_follow_y = env.enemy_position['y']
            env.send_goal_in_follow(env.navgoal)
            rospy.loginfo('r0: is follow !!!!!!!     %s,%s,%s' %
                                (self.goal_x, self.goal_y, self.normalizeDegree(self.goal_yaw)))
        if env.FOLLOW_cancel_flag == 1:
            self.blackboard.EnterFollow = False
            return py_trees.Status.FAILURE
        return py_trees.Status.SUCCESS

    def need_change_angle(self, angle_min, angle_max): # degrees
        angle = abs(math.degrees(env.enemy_pose.enemy_yaw) - env.robot_pose['theta'])
        if angle <= angle_max and angle >= angle_min:
            return False
        else:
            return True

    def follow_goal(self, enemy_dist, enemy_yaw, enemy_x, enemy_y, robot_x, robot_y):
        rospy.logerr("---------follow_goal---------")
        self.count = 0
        self.change_angle = 0
        self.goal_x = 0.01
        self.goal_y = 0.01
        self.goal_yaw = 0

        if self.no_big_change == True:
            self.keep_length = 1.2 
        elif self.close_obstacle(enemy_x, enemy_x, robot_x, robot_y, 0.3) == True:
            self.keep_length = 0.7
        elif enemy_dist > 0 and enemy_dist < 2.6:
            self.keep_length = 1.2 # 跟随距离
        while env.isActionAvaliable(self.goal_x, self.goal_y, self.normalizeDegree(self.goal_yaw)) == False:
            self.count += 1
            if self.count > (len(self.change_angle_list) - 1):
                rospy.logwarn('!!!!!!  cant follow  !!!!!!!')
                return False     # 无法跟随         
            self.change_angle = math.radians(self.change_angle_list[self.count]) # 旋转角度 radian
                    
            rospy.loginfo("%s" % math.degrees(self.change_angle))
            self.yaw = enemy_yaw + self.change_angle - math.pi
            self.goal_x = self.keep_length * math.cos(self.yaw) + enemy_x
            self.goal_y = self.keep_length * math.sin(self.yaw) + enemy_y
            self.goal_yaw = self.yaw - math.pi

            if self.obstacle_dis(self.goal_x, self.goal_y, 0.4) == False or env.isReachable(enemy_x, enemy_y, 
                 self.goal_x, self.goal_y) == False:
                self.goal_x = 0.01
                self.goal_y = 0.01
        return True
    
    def rescue_follow_goal(self, enemy_x, enemy_y, enemy_yaw, robot_x, robot_y):
        if enemy_yaw >= math.pi/2:
            self.goal_x = robot_x
            self.goal_y = enemy_y
            self.goal_yaw = math.radians(178)
            if env.isReachable(self.goal_x, self.goal_y, robot_x, robot_y) == False:
                self.goal_x = enemy_x
                self.goal_y = robot_y
                self.goal_yaw = math.radians(90)
        elif enemy_yaw >= 0:
            self.goal_x = robot_x
            self.goal_y = enemy_y
            self.goal_yaw = 0.01
            if env.isReachable(self.goal_x, self.goal_y, robot_x, robot_y) == False:
                self.goal_x = enemy_x
                self.goal_y = robot_y
                self.goal_yaw = math.radians(90)
        elif enemy_yaw >= -math.pi/2:
            self.goal_x = robot_x
            self.goal_y = enemy_y
            self.goal_yaw = 0.01
            if env.isReachable(self.goal_x, self.goal_y, robot_x, robot_y) == False:
                self.goal_x = enemy_x
                self.goal_y = robot_y
                self.goal_yaw = math.radians(-90)         
        else:
            self.goal_x = robot_x
            self.goal_y = enemy_y
            self.goal_yaw = math.radians(178)
            if env.isReachable(self.goal_x, self.goal_y, robot_x, robot_y) == False:
                self.goal_x = enemy_x
                self.goal_y = robot_y
                self.goal_yaw = math.radians(-90)
        if env.isActionAvaliable(self.goal_x, self.goal_y, self.normalizeDegree(self.goal_yaw)) == False:
            return False
        else:
            return True

    def normalizeDegree(self, theta): # 将角度radian归一化为[-180, 180]
        return math.degrees(math.atan2(math.sin(theta), math.cos(theta)))

    def enemy_big_change(self):
        if np.square(self.blackboard.enemy_last_x - env.enemy_position['x']) + np.square(self.blackboard.enemy_last_y - env.enemy_position['y']) < 0.025:
            return False
        if abs(math.degrees(env.enemy_pose.enemy_yaw) - env.robot_pose['theta']) < 40:
            return False
        return True

    def obstacle_dis(self, robot_x, robot_y, dis_set):
        dis_min = 10
        for i in env.obstacle_conner:
            dis = math.hypot(robot_x - i[0], robot_y - i[1])
            if dis < dis_min:
                dis_min = dis
        if dis_min < dis_set:
            return False
        else:
            return True
    
    def close_obstacle(self, enemy_x, enemy_y, robot_x, robot_y, dis_set):
        dis_min = 10
        for i in env.obstacle_conner:
            if self.judge(i[0], i[1], enemy_x, enemy_y, robot_x, robot_y):
                dis = self.dis_point_to_line(i[0], i[1], enemy_x, enemy_y, robot_x, robot_y)
                if dis_min > dis:
                    dis_min = dis
        if dis_min < dis_set:
            return True
        else:
            return False
    
    def dis_point_to_line(self, pointX, pointY, lineX1, lineY1, lineX2, lineY2): # 一般式
        a = lineY2 - lineY1
        b = lineX1 - lineX2
        c = lineX2 * lineY1 - lineX1 * lineY2
        dis = (math.fabs(a*pointX+b*pointY+c)) / (math.pow(a*a+b*b,0.5))
        return dis
    
    def judge(self, pointX, pointY, pointX1, pointY1, pointX2, pointY2):
        if (pointX >= pointX1 and pointX <= pointX2) or (pointX <= pointX1 and pointX >= pointX2):
            if (pointY >= pointY1 and pointY <= pointY2) or (pointY <= pointY1 and pointY >= pointY2):
                return True
            else:
                return False
        else:
            return False