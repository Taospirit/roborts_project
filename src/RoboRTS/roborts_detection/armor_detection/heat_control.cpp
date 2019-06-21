#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include "roborts_msgs/HeatControl.h"
#include "roborts_msgs/FricSpeed.h"
#include "roborts_msgs/ArmorDetection.h"
#include "roborts_msgs/ProjectileInfo.h"
#include "roborts_msgs/RobotHeat.h"
#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/RobotShoot.h"
#include "roborts_msgs/GameStatus.h"


#define MAX_SHOOT_SPEED 21
#define MIN_SHOOT_SPEED 16
#define MAX_ENEMY_DISTANCE 5000
#define MIN_ENEMY_DISTANCE 300

class HeatControl {
protected:
    ros::NodeHandle nh_;

    ros::Subscriber projectile_info_sub_;
    ros::Subscriber robot_status_sub_;
    ros::Subscriber robot_heat_sub_;
    ros::Subscriber enemy_info_sub_;
    ros::Subscriber robot_shoot_sub_;

    ros::Publisher heat_control_pub_;
    ros::Publisher fric_speed_pub_;

    // topic message
    roborts_msgs::HeatControl heat_control_msg_;
    roborts_msgs::FricSpeed fric_speed_msg_;

    // enemy info
    bool detected_enemy_;
    float gimbal_omega_;
    float enemy_distance_;
    float enemy_angle_;

    // projectile info
    int projectile_state_;

    // robot state
    int remain_hp_;

    // robot heat
    int shooter_heat_now_;
    int shooter_heat_last_;
    int real_heat_;

    std::string robot_name;
    char robot_name_;

public:
    HeatControl() {
        nh_.param<std::string>("/robot_name", robot_name, "blue");
        ROS_INFO("using model for %s", robot_name.c_str());
        robot_name_ = (robot_name == "blue")? 1 : 0;
        detected_enemy_ = false;
        enemy_distance_ = 0;
        enemy_angle_ = 0;
        gimbal_omega_ = 0;
        // 1 no projectile, 2 less, 3 many ,0 error shoot cmd, 4 initial state
        projectile_state_ = 4;
        remain_hp_ = 2000;
        real_heat_ = 0;

        projectile_info_sub_ =
                nh_.subscribe<roborts_msgs::ProjectileInfo>("projectile_info", 1, &HeatControl::projectile_callback,
                                                                this);
        robot_status_sub_ =
                nh_.subscribe<roborts_msgs::RobotStatus>("robot_status", 1, &HeatControl::robot_status_callback,
                                                             this);
        robot_heat_sub_ =
                nh_.subscribe<roborts_msgs::RobotHeat>("robot_heat", 1, &HeatControl::robot_heat_callback, this);
        enemy_info_sub_ = nh_.subscribe<roborts_msgs::ArmorDetection>("armor_detection_info",
                                                                          1,
                                                                          &HeatControl::enemy_info_callback,
                                                                          this);
        robot_shoot_sub_ =
                nh_.subscribe<roborts_msgs::RobotShoot>("robot_shoot", 1, &HeatControl::robot_shoot_callback, this);

        heat_control_pub_ = nh_.advertise<roborts_msgs::HeatControl>("heat_control_info", 1, true);
        fric_speed_pub_ = nh_.advertise<roborts_msgs::FricSpeed>("set_fric_speed", 1, true);
    }

    void projectile_callback(const roborts_msgs::ProjectileInfo::ConstPtr &msg) {
        projectile_state_ = msg->pjt_info;
    }

    void robot_status_callback(const roborts_msgs::RobotStatus::ConstPtr &msg) {
        remain_hp_ = msg->remain_hp;
    }

    void robot_heat_callback(const roborts_msgs::RobotHeat::ConstPtr &msg) {
        shooter_heat_now_ = msg->shooter_heat;
        if (shooter_heat_now_ != shooter_heat_last_ || (shooter_heat_now_ == 0 && shooter_heat_last_ == 0))
            real_heat_ = shooter_heat_now_;
        shooter_heat_last_ = shooter_heat_now_;
    }

    void enemy_info_callback(const roborts_msgs::ArmorDetection::ConstPtr &msg) {
        detected_enemy_ = msg->detected_enemy;
        enemy_distance_ = msg->distance;
        enemy_angle_ = msg->yaw_angle;
    }

    void robot_shoot_callback(const roborts_msgs::RobotShoot::ConstPtr &msg) {
        real_heat_ += msg->speed;
    }

    void calc_pwm_by_speed(void) {
        if(robot_name_){
            fric_speed_msg_.fric_speed = (int) (0.1685 * std::pow(heat_control_msg_.shoot_speed, 2) + 
                1.121 * heat_control_msg_.shoot_speed + 1170.8);
        }else{
            fric_speed_msg_.fric_speed = (int) (0.2817 * std::pow(heat_control_msg_.shoot_speed, 2) - 
                1.306 * heat_control_msg_.shoot_speed + 1156.1);
        }
        fric_speed_pub_.publish(fric_speed_msg_);
    }

    void calc_speed_and_num(void) {
        int remain_heat = 0;
        // heat check
        if (real_heat_ < 180) {
            if (remain_hp_ < 400)   // cooling 12 per period
                remain_heat = 180 - real_heat_ + 1.2;
            else                   // cooling 6 per period
                remain_heat = 180 - real_heat_ + 0.6;
        } else {
            heat_control_msg_.shoot_num = -1;
            heat_control_msg_.shoot_speed = (MAX_SHOOT_SPEED + MIN_SHOOT_SPEED) / 2.0;  // m/s
            heat_control_pub_.publish(heat_control_msg_);
            return;
        }
        // projectile check
        if (projectile_state_ == 1)  // no projectile
        {
            heat_control_msg_.shoot_num = 1;
            heat_control_msg_.shoot_speed = (MAX_SHOOT_SPEED + MIN_SHOOT_SPEED) / 2.0;  // m/s
            heat_control_pub_.publish(heat_control_msg_);
            return;
        } else if (projectile_state_ == 4 || !detected_enemy_) // initial state
        {
            heat_control_msg_.shoot_speed = (MAX_SHOOT_SPEED + MIN_SHOOT_SPEED) / 2.0;  // m/s
            if (remain_heat / heat_control_msg_.shoot_speed > 1)
                heat_control_msg_.shoot_num = 1;
            else
                heat_control_msg_.shoot_num = 0;
            heat_control_pub_.publish(heat_control_msg_);
            return;
        }

        // detection check
        if (detected_enemy_) {
            float delta_shoot_speed = MAX_SHOOT_SPEED - MIN_SHOOT_SPEED;
            float delta_enmy_distance = MAX_ENEMY_DISTANCE - MIN_ENEMY_DISTANCE;

            if (projectile_state_ == 2)    // less projectile
            {
                if (enemy_distance_ < MIN_ENEMY_DISTANCE)
                    heat_control_msg_.shoot_speed = MIN_SHOOT_SPEED;
                else if (enemy_distance_ > MAX_ENEMY_DISTANCE)
                    heat_control_msg_.shoot_speed = MAX_SHOOT_SPEED;
                else
                    heat_control_msg_.shoot_speed = MIN_SHOOT_SPEED +
                                                    (enemy_distance_ - MIN_ENEMY_DISTANCE) / delta_enmy_distance *
                                                    delta_shoot_speed;
                heat_control_msg_.shoot_num = std::floor(remain_heat / heat_control_msg_.shoot_speed);
            } else if (projectile_state_ == 3)  // many projectile
            {
                if (enemy_distance_ < MIN_ENEMY_DISTANCE)
                    heat_control_msg_.shoot_speed = MIN_SHOOT_SPEED;
                else if (enemy_distance_ > MAX_ENEMY_DISTANCE)
                    heat_control_msg_.shoot_speed = MAX_SHOOT_SPEED;
                else
                    heat_control_msg_.shoot_speed = MIN_SHOOT_SPEED +
                                                    (enemy_distance_ - MIN_ENEMY_DISTANCE) / delta_enmy_distance *
                                                    delta_shoot_speed;
                heat_control_msg_.shoot_num = std::floor(remain_heat / heat_control_msg_.shoot_speed);
            }
        }

        // std::cout << "detected:" << detected_enemy_ << "  e_distance:" << enemy_distance_ << "  e_angle:" << enemy_angle_
        // << "  g_omega:" << gimbal_omega_ << std::endl << "projectile:" << projectile_state_ << "  hp:" << remain_hp_ << " heat:" << real_heat_ << std::endl;

        heat_control_pub_.publish(heat_control_msg_);
    }

};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "heat_control");

    HeatControl heat_control;

    ros::Rate rate(100);
    while (ros::ok()) {
        heat_control.calc_speed_and_num();
        heat_control.calc_pwm_by_speed();
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
