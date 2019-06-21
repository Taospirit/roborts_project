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

#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include "camera_node.h"

namespace roborts_camera {
    CameraNode::CameraNode(std::string robot_name) :
        camera_param_(robot_name){
        camera_num_ = camera_param_.GetCameraParam().size();
        img_pubs_.resize(camera_num_);
        camera_threads_.resize(camera_num_);
        camera_driver_.resize(camera_num_);

        for (unsigned int i = 0; i < camera_num_; i++) {
            auto camera_info = camera_param_.GetCameraParam()[i];
            nhs_.push_back(ros::NodeHandle(camera_info.camera_name));
            image_transport::ImageTransport it(nhs_.at(i));
            img_pubs_[i] = it.advertiseCamera("image_raw", 1, true);
            //create the selected camera driver
            camera_driver_[i] =
                    roborts_common::AlgorithmFactory<CameraBase, CameraInfo>::CreateAlgorithm(camera_info.camera_type,
                                                                                              camera_info);
        }

        StartThread();
    }

    void CameraNode::StartThread() {
        running_ = true;
        for (unsigned int i = 0; i < camera_num_; i++) {
            camera_threads_[i] = std::thread(&CameraNode::Update, this, i);
        }
    }

    void CameraNode::Update(const unsigned int index) {
        cv::Mat img;
        bool camera_info_send = false;
        ros::Time last_time = ros::Time::now();
        while (running_) {

            ros::Duration period = ros::Time::now() - last_time;
            char sizestr[30] = "\0";
            if (!img.empty())
                sprintf(sizestr, "  Img_size=[%d %d]", img.cols, img.rows);
            if (1000000000 / period.nsec < 30)
                ROS_WARN_THROTTLE(1, "%s:\t FPS=%d\t%s", camera_param_.GetCameraParam()[index].camera_name.c_str(),
                                  1000000000 / period.nsec, sizestr);
            else
                ROS_INFO_THROTTLE(1, "%s:\t FPS=%d\t%s", camera_param_.GetCameraParam()[index].camera_name.c_str(),
                                  1000000000 / period.nsec, sizestr);
            last_time = ros::Time::now();
            camera_driver_[index]->StartReadCamera(img); //开启摄像头并调节曝光度
            if (!img.empty()) {
                sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
                img_msg->header.frame_id = camera_param_.GetCameraParam()[index].camera_name;
                img_msg->header.stamp = ros::Time::now();
                camera_param_.GetCameraParam()[index].ros_camera_info->header.stamp = img_msg->header.stamp;
                img_pubs_[index].publish(img_msg, camera_param_.GetCameraParam()[index].ros_camera_info);
            } else {
                ROS_ERROR_THROTTLE(1, "img is empty");
            }

        }
    }

    void CameraNode::StoptThread() {

    }

    CameraNode::~CameraNode() {
        running_ = false;
        for (auto &iter: camera_threads_) {
            if (iter.joinable())
                iter.join();
        }
    }
} //namespace roborts_camera

void SignalHandler(int signal) {
    if (ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()) {
        ros::shutdown();
    }
}

int main(int argc, char **argv) {
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);
    ros::init(argc, argv, "roborts_camera_node", ros::init_options::NoSigintHandler);

    ros::NodeHandle nh_;
    std::string robot_name;
    nh_.param<std::string>("/robot_name", robot_name, "blue");
    ROS_INFO("using camera param for %s", robot_name.c_str());

    roborts_camera::CameraNode camera_test(robot_name);
    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();
    ros::waitForShutdown();
    return 0;
}
