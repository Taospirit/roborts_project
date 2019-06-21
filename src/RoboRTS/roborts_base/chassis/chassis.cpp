/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "chassis.h"
#include "../roborts_sdk/sdk.h"

roborts_msgs::Debug global_debug_msg;

namespace roborts_base {
    Chassis::Chassis(std::shared_ptr<roborts_sdk::Handle> handle, std::string odom_frame_name,
                     std::string robot_base_frame_name, roborts_base::Speed speed) :
            handle_(handle),
            odom_frame_(odom_frame_name),
            robot_base_frame_(robot_base_frame_name),
            speed_(speed) {
        SDK_Init();
        ROS_Init();
    }

    Chassis::~Chassis() {
        if (heartbeat_thread_.joinable()) {
            heartbeat_thread_.join();
        }
    }

    void Chassis::SDK_Init() {

        verison_client_ = handle_->CreateClient<roborts_sdk::cmd_version_id, roborts_sdk::cmd_version_id>
                (UNIVERSAL_CMD_SET, CMD_REPORT_VERSION,
                 MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
        roborts_sdk::cmd_version_id version_cmd;
        version_cmd.version_id = 0;
        auto version = std::make_shared<roborts_sdk::cmd_version_id>(version_cmd);
        verison_client_->AsyncSendRequest(version,
                                          [](roborts_sdk::Client<roborts_sdk::cmd_version_id,
                                                  roborts_sdk::cmd_version_id>::SharedFuture future) {
                                              ROS_INFO("Chassis Firmware Version: %d.%d.%d.%d",
                                                       int(future.get()->version_id >> 24 & 0xFF),
                                                       int(future.get()->version_id >> 16 & 0xFF),
                                                       int(future.get()->version_id >> 8 & 0xFF),
                                                       int(future.get()->version_id & 0xFF));
                                          });

        handle_->CreateSubscriber<roborts_sdk::cmd_chassis_info>(CHASSIS_CMD_SET, CMD_PUSH_CHASSIS_INFO,
                                                                 CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                                 std::bind(&Chassis::ChassisInfoCallback, this,
                                                                           std::placeholders::_1));
        handle_->CreateSubscriber<roborts_sdk::cmd_uwb_info>(COMPATIBLE_CMD_SET, CMD_PUSH_UWB_INFO,
                                                             CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                             std::bind(&Chassis::UWBInfoCallback, this,
                                                                       std::placeholders::_1));

        chassis_speed_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_chassis_speed>(CHASSIS_CMD_SET,
                                                                                      CMD_SET_CHASSIS_SPEED,
                                                                                      MANIFOLD2_ADDRESS,
                                                                                      CHASSIS_ADDRESS);
        chassis_spd_acc_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_chassis_spd_acc>(CHASSIS_CMD_SET,
                                                                                          CMD_SET_CHASSIS_SPD_ACC,
                                                                                          MANIFOLD2_ADDRESS,
                                                                                          CHASSIS_ADDRESS);
        // set chassis mode
        chassis_mode_pub_ = handle_->CreatePublisher<roborts_sdk::chassis_mode_e>(CHASSIS_CMD_SET, CMD_SET_CHASSIS_MODE,
                                                                                  MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);

        heartbeat_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_heartbeat>(UNIVERSAL_CMD_SET, CMD_HEARTBEAT,
                                                                              MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
        heartbeat_thread_ = std::thread([this] {
                                            roborts_sdk::cmd_heartbeat heartbeat;
                                            heartbeat.heartbeat = 0;
                                            while (ros::ok()) {
                                                heartbeat_pub_->Publish(heartbeat);
                                                std::this_thread::sleep_for(std::chrono::milliseconds(300));
                                            }
                                        }
        );

    }

    void Chassis::ROS_Init() {
        //ros publisher
        ros_odom_pub_ = ros_nh_.advertise<nav_msgs::Odometry>("odom", 30);
        ros_uwb_pub_ = ros_nh_.advertise<geometry_msgs::PoseStamped>("uwb", 30);
        ros_pub_pjt_ = ros_nh_.advertise<roborts_msgs::ProjectileInfo>("projectile_info", 1);
        ros_pub_debug_ = ros_nh_.advertise<roborts_msgs::Debug>("chassis_gimbal_mode", 1);
        //ros subscriber
        ros_sub_cmd_chassis_vel_ = ros_nh_.subscribe("cmd_vel", 1, &Chassis::ChassisSpeedCtrlCallback, this);
        ros_sub_cmd_chassis_vel_acc_ = ros_nh_.subscribe("cmd_vel_acc", 1, &Chassis::ChassisSpeedAccCtrlCallback, this);


        //ros service
        ros_chassis_mode_srv_ = ros_nh_.advertiseService("set_chassis_mode", &Chassis::SetChassisModeService, this);

        //ros_message_init
        odom_.header.frame_id = odom_frame_;
        odom_.child_frame_id = robot_base_frame_;

        odom_tf_.header.frame_id = odom_frame_;
        odom_tf_.child_frame_id = robot_base_frame_;

        uwb_data_.header.frame_id = "uwb";
    }

    void Chassis::ChassisInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_info> chassis_info) {

        ros::Time current_time = ros::Time::now();
        odom_.header.stamp = current_time;
        odom_.pose.pose.position.x = chassis_info->position_x_mm / 1000.;
        odom_.pose.pose.position.y = chassis_info->position_y_mm / 1000.;
        odom_.pose.pose.position.z = 0.0;
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(chassis_info->gyro_angle / 1800.0 * M_PI);
        odom_.pose.pose.orientation = q;
        odom_.twist.twist.linear.x = chassis_info->v_x_mm / 1000.0;
        odom_.twist.twist.linear.y = chassis_info->v_y_mm / 1000.0;
        odom_.twist.twist.angular.z = chassis_info->gyro_rate / 1800.0 * M_PI;
        ros_odom_pub_.publish(odom_);

        odom_tf_.header.stamp = current_time;
        odom_tf_.transform.translation.x = chassis_info->position_x_mm / 1000.;
        odom_tf_.transform.translation.y = chassis_info->position_y_mm / 1000.;

        odom_tf_.transform.translation.z = 0.0;
        odom_tf_.transform.rotation = q;
        tf_broadcaster_.sendTransform(odom_tf_);

        ros_pub_pjt_.publish(chassis_info->pjt_info);

        // global_debug_msg
        global_debug_msg.chassis_mode = chassis_info->chassis_mode;
        ros_pub_debug_.publish(global_debug_msg);
    }

    void Chassis::UWBInfoCallback(const std::shared_ptr<roborts_sdk::cmd_uwb_info> uwb_info) {

        uwb_data_.header.stamp = ros::Time::now();
        uwb_data_.pose.position.x = ((double) uwb_info->x) / 100.0;
        uwb_data_.pose.position.y = ((double) uwb_info->y) / 100.0;
        uwb_data_.pose.position.z = 0;
        uwb_data_.pose.orientation = tf::createQuaternionMsgFromYaw(uwb_info->yaw / 180.0 * M_PI);
        ros_uwb_pub_.publish(uwb_data_);

    }

    void Chassis::ChassisSpeedCtrlCallback(const geometry_msgs::Twist::ConstPtr &vel) {
        roborts_sdk::cmd_chassis_speed chassis_speed;

        if (vel->linear.x > speed_.speed_linear_max) {
            chassis_speed.vx = speed_.speed_linear_max;
        } else {
            chassis_speed.vx = vel->linear.x * speed_.speed_linear_ratio;
        }
        if (vel->linear.y > speed_.speed_linear_max) {
            chassis_speed.vy = speed_.speed_linear_max;
        } else {
            chassis_speed.vy = vel->linear.y * speed_.speed_linear_ratio;
        }
        if (vel->linear.y > speed_.speed_angular_max) {
            chassis_speed.vw = speed_.speed_angular_max;
        } else {
            chassis_speed.vw = vel->angular.z * speed_.speed_angular_ratio / M_PI;
        }

        chassis_speed.rotate_x_offset = 0;
        chassis_speed.rotate_y_offset = 0;
        chassis_speed_pub_->Publish(chassis_speed);
    }

    void Chassis::ChassisSpeedAccCtrlCallback(const roborts_msgs::TwistAccel::ConstPtr &vel_acc) {
        roborts_sdk::cmd_chassis_spd_acc chassis_spd_acc;

        if (vel_acc->twist.linear.x > speed_.speed_linear_max) {
            chassis_spd_acc.vx = speed_.speed_linear_max;
        } else {
            chassis_spd_acc.vx = vel_acc->twist.linear.x * speed_.speed_linear_ratio;
        }

        if (vel_acc->twist.linear.y > speed_.speed_linear_max) {
            chassis_spd_acc.vy = speed_.speed_linear_max;
        } else {
            chassis_spd_acc.vy = vel_acc->twist.linear.y * speed_.speed_linear_ratio;
        }

        if (vel_acc->twist.linear.z > speed_.speed_angular_max) {
            chassis_spd_acc.vw = speed_.speed_angular_max;
        } else {
            chassis_spd_acc.vw = vel_acc->twist.angular.z * speed_.speed_angular_ratio / M_PI;
        }

        if (vel_acc->accel.linear.x > speed_.speed_linear_max) {
            chassis_spd_acc.ax = speed_.speed_linear_acc_max;
        } else {
            chassis_spd_acc.ax = vel_acc->accel.linear.x * speed_.speed_linear_ratio;
        }

        if (vel_acc->accel.linear.y > speed_.speed_linear_max) {
            chassis_spd_acc.ay = speed_.speed_linear_acc_max;
        } else {
            chassis_spd_acc.ay = vel_acc->accel.linear.y * speed_.speed_linear_ratio;
        }

        if (vel_acc->accel.angular.z > speed_.speed_angular_max) {
            chassis_spd_acc.wz = speed_.speed_angular_acc_max;
        } else {
            chassis_spd_acc.wz = vel_acc->accel.angular.z * speed_.speed_angular_ratio / M_PI;
        }

        chassis_spd_acc.rotate_x_offset = 0;
        chassis_spd_acc.rotate_y_offset = 0;
        chassis_spd_acc_pub_->Publish(chassis_spd_acc);
    }

    bool Chassis::SetChassisModeService(roborts_msgs::ChassisMode::Request &req,
                                        roborts_msgs::ChassisMode::Response &res) {
        roborts_sdk::chassis_mode_e chassis_mode = static_cast<roborts_sdk::chassis_mode_e>(req.chassis_mode);
        chassis_mode_pub_->Publish(chassis_mode);
        res.received = true;
        return true;
    }

}
