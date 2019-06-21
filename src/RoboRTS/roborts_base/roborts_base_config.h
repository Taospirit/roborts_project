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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBORTS_BASE_CONFIG_H
#define ROBORTS_BASE_CONFIG_H

#include <ros/ros.h>

namespace roborts_base {

    struct Speed {
        int speed_linear_ratio;
        int speed_angular_ratio;
        int speed_linear_max;
        int speed_angular_max;
        int speed_linear_acc_max;
        int speed_angular_acc_max;
    };

    struct Config {
        void GetParam(ros::NodeHandle *nh) {
            node_name = ros::this_node::getName();
            nh->param<std::string>("/serial_port", serial_port, "/dev/serial_sdk");
            nh->param<std::string>("odom_frame_name", odom_frame_name, "odom");
            nh->param<std::string>("robot_base_frame_name", robot_base_frame_name, "base_link");
            nh->param<std::string>("gimbal_frame_name", gimbal_frame_name, "gimbal");
            nh->param<int>("speed_linear_ratio", speed.speed_linear_ratio, 1000);
            nh->param<int>("speed_angular_ratio", speed.speed_angular_ratio, 1800);
            nh->param<int>("speed_linear_max", speed.speed_linear_max, 1);
            nh->param<int>("speed_angular_max", speed.speed_angular_max, 1);
            nh->param<int>("speed_linear_acc_max", speed.speed_linear_acc_max, 1);
            nh->param<int>("speed_angular_acc_max", speed.speed_angular_acc_max, 1);
        }

        std::string serial_port;
        std::string node_name;
        std::string odom_frame_name;
        std::string robot_base_frame_name;
        std::string gimbal_frame_name;
        roborts_base::Speed speed;
    };

}
#endif //ROBORTS_BASE_CONFIG_H