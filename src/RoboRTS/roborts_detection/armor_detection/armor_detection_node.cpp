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

#include <unistd.h>
#include "armor_detection_node.h"
#include <exception>

#include <iostream>
#include <fstream>
#include <string>

namespace roborts_detection {

    ArmorDetectionNode::ArmorDetectionNode(std::string robot_base_frame_name,
                                           std::string gimbal_frame_name,
                                           std::string camera_frame_name) :
            node_state_(roborts_common::IDLE),
            demensions_(3),
            initialized_(false),
            detected_enemy_(false),
            undetected_count_(0),
            robot_base_frame_(robot_base_frame_name),
            gimbal_frame_(gimbal_frame_name),
            camera_frame_(camera_frame_name),
            as_(nh_, "armor_detection_node_action", boost::bind(&ArmorDetectionNode::ActionCB, this, _1), false),
            armor_detection_actionlib_client_("armor_detection_node_action", true) {
        tf_ = std::make_shared<tf::TransformListener>(ros::Duration(1));
        enemy_nh_ = ros::NodeHandle();

        enemy_nh_.param<std::string>("/robot_name", robot_name, "blue");
        ROS_INFO("using detection param for %s", robot_name.c_str());

        initialized_ = false;
        side_detection_.resize(3);
        side_detection_sub_.resize(3);

        if (Init().IsOK()) {
            initialized_ = true;
            node_state_ = roborts_common::IDLE;
        } else {
            ROS_ERROR("armor_detection_node initalized failed!");
            node_state_ = roborts_common::FAILURE;
        }
        as_.start();

        armor_detection_actionlib_client_.waitForServer();
        armor_detection_goal_.command = 1;
        armor_detection_actionlib_client_.sendGoal(armor_detection_goal_,
                                                   actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleDoneCallback(),
                                                   actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleActiveCallback(),
                                                   boost::bind(&ArmorDetectionNode::ArmorDetectionFeedbackCallback,
                                                               this,
                                                               _1));

    }

    ErrorInfo ArmorDetectionNode::GimbalReload() {
        ArmorDetectionAlgorithms armor_detection_param;

        std::string
                file_name =
                ros::package::getPath("roborts_detection") + "/armor_detection/config/front_detection_" + robot_name + ".prototxt";
        bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &armor_detection_param);
        if (!read_state) {
            ROS_ERROR("Cannot open %s", file_name.c_str());
            return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
        }
        gimbal_control_.Init(armor_detection_param.camera_gimbal_transform().offset_x(),
                             armor_detection_param.camera_gimbal_transform().offset_z(),
                             armor_detection_param.camera_gimbal_transform().offset_y(),
                             armor_detection_param.camera_gimbal_transform().offset_pitch(),
                             armor_detection_param.camera_gimbal_transform().offset_yaw(),
                             armor_detection_param.projectile_model_info().init_v(),
                             armor_detection_param.projectile_model_info().init_k(),
                             armor_detection_param.gimbal_control_param().first_meet_kp(),
                             armor_detection_param.gimbal_control_param().first_meet_kd(),
                             armor_detection_param.gimbal_control_param().tracking_kp(),
                             armor_detection_param.gimbal_control_param().tracking_ki(),
                             armor_detection_param.gimbal_control_param().tracking_kd(),
                             armor_detection_param.gimbal_control_param().tracking_integral_limit());
        ROS_INFO("Reload the gimbal parameter from armor_detection.prototxt");
        return ErrorInfo(ErrorCode::OK);
    }

    ErrorInfo ArmorDetectionNode::Init() {
        enemy_info_pub_ = enemy_nh_.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 1);
        armor_detection_pub_ = enemy_nh_.advertise<roborts_msgs::ArmorDetection>("armor_detection_info", 1);
        shoot_speed_sub_ = enemy_nh_.subscribe<roborts_msgs::HeatControl>("heat_control_info", 1,
                                                                          boost::bind(
                                                                                  &ArmorDetectionNode::ShootSpeedCallback,
                                                                                  this,
                                                                                  _1));

        switch (side_detection_sub_.size()) {
            case 3:
                side_detection_sub_[2] = enemy_nh_.subscribe<roborts_msgs::SideDetection>(
                        "back_camera/detection_result", 1,
                        boost::bind(&ArmorDetectionNode::SideDetectionCallback, this, _1));
            case 2:
                side_detection_sub_[1] = enemy_nh_.subscribe<roborts_msgs::SideDetection>(
                        "right_camera/detection_result", 1,
                        boost::bind(&ArmorDetectionNode::SideDetectionCallback, this, _1));
            case 1:
                side_detection_sub_[0] = enemy_nh_.subscribe<roborts_msgs::SideDetection>(
                        "left_camera/detection_result", 1,
                        boost::bind(&ArmorDetectionNode::SideDetectionCallback, this, _1));
            default:
                break;
        }

        ArmorDetectionAlgorithms armor_detection_param;

        //gimbal control debug --> draw its curves
        // gimbal_info_pub_ = enemy_nh_.advertise<roborts_msgs::GimbalInfo>("debug_gimbal_info", 1);

        //KalmanFilter Initialization
        KalmanReset();

        std::string
                file_name =
                ros::package::getPath("roborts_detection") + "/armor_detection/config/front_detection_" + robot_name + ".prototxt";
        bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &armor_detection_param);
        if (!read_state) {
            ROS_ERROR("Cannot open %s", file_name.c_str());
            return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
        }
        gimbal_control_.Init(armor_detection_param.camera_gimbal_transform().offset_x(),
                             armor_detection_param.camera_gimbal_transform().offset_z(),
                             armor_detection_param.camera_gimbal_transform().offset_y(),
                             armor_detection_param.camera_gimbal_transform().offset_pitch(),
                             armor_detection_param.camera_gimbal_transform().offset_yaw(),
                             armor_detection_param.projectile_model_info().init_v(),
                             armor_detection_param.projectile_model_info().init_k(),
                             armor_detection_param.gimbal_control_param().first_meet_kp(),
                             armor_detection_param.gimbal_control_param().first_meet_kd(),
                             armor_detection_param.gimbal_control_param().tracking_kp(),
                             armor_detection_param.gimbal_control_param().tracking_ki(),
                             armor_detection_param.gimbal_control_param().tracking_kd(),
                             armor_detection_param.gimbal_control_param().tracking_integral_limit());

        //create the selected algorithms
        std::string selected_algorithm = armor_detection_param.selected_algorithm();
        // create image receiver
        cv_toolbox_ = std::make_shared<CVToolbox>(armor_detection_param.camera_name());
        // create armor detection algorithm
        armor_detector_ = roborts_common::AlgorithmFactory<ArmorDetectionBase, std::shared_ptr<CVToolbox>, std::string>::CreateAlgorithm
                (selected_algorithm, cv_toolbox_, "front_camera");

        undetected_armor_delay_ = armor_detection_param.undetected_armor_delay();
        if (armor_detector_ == nullptr) {
            ROS_ERROR("Create armor_detector_ pointer failed!");
            return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
        } else
            return ErrorInfo(ErrorCode::OK);
    }

    void ArmorDetectionNode::KalmanReset() {
        kalman_.init(4, 2, 0);
        kalman_.transitionMatrix = (cv::Mat_<float>(4, 4) << 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
        cv::setIdentity(kalman_.measurementMatrix);
        cv::setIdentity(kalman_.processNoiseCov, cv::Scalar::all(1e-5));
        cv::setIdentity(kalman_.measurementNoiseCov, cv::Scalar::all(1e-5));
        cv::setIdentity(kalman_.errorCovPost, cv::Scalar::all(1));
        measurement_ = cv::Mat::zeros(2, 1, CV_32F);
        // kalman_.statePost = cv::Mat(cv::Size(4,1),CV_32F);
        // cv::RNG r;
        // r.fill(kalman_.statePost, cv::RNG::UNIFORM,0,100);
        cv::randn(kalman_.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));
    }

    void ArmorDetectionNode::ActionCB(const roborts_msgs::ArmorDetectionGoal::ConstPtr &data) {
        roborts_msgs::ArmorDetectionFeedback feedback;
        roborts_msgs::ArmorDetectionResult result;
        bool undetected_msg_published = false;

        if (!initialized_) {
            feedback.error_code = error_info_.error_code();
            feedback.error_msg = error_info_.error_msg();
            as_.publishFeedback(feedback);
            as_.setAborted(result, feedback.error_msg);
            ROS_INFO("Initialization Failed, Failed to execute action!");
            return;
        }

        switch (data->command) {
            case 1:
                StartThread();
                break;
            case 2:
                PauseThread();
                break;
            case 3:
                StopThread();
                break;
            case 4:
                GimbalReload();
                break;
            default:
                break;
        }

        bool last_detect_result = false;

        ros::Rate rate(100);
        while (ros::ok()) {

            if (as_.isPreemptRequested()) {
                as_.setPreempted();
                return;
            }

            {
                std::lock_guard<std::mutex> guard(mutex_);
                //if (undetected_count_ != 0) {
                if (detected_enemy_) {
                    feedback.detected = true;
                    feedback.error_code = error_info_.error_code();
                    feedback.error_msg = error_info_.error_msg();
                    feedback.enemy_pos.header.frame_id = camera_frame_;
                    feedback.enemy_pos.header.stamp = ros::Time::now();
                    feedback.enemy_pos.pose.position.x = x_;
                    feedback.enemy_pos.pose.position.y = y_;
                    feedback.enemy_pos.pose.position.z = z_;
                    feedback.enemy_pos.pose.orientation.w = 1;
                    as_.publishFeedback(feedback);
                    // undetected_msg_published = false;
                } else {
                    //if(!undetected_msg_published) {
                    if(last_detect_result) {
                        feedback.detected = true;
                        feedback.error_code = error_info_.error_code();
                        feedback.error_msg = error_info_.error_msg();
                        feedback.enemy_pos.header.frame_id = camera_frame_;
                        feedback.enemy_pos.header.stamp = ros::Time::now();
                        feedback.enemy_pos.pose.position.x = x_;
                        feedback.enemy_pos.pose.position.y = y_;
                        feedback.enemy_pos.pose.position.z = z_;
                        feedback.enemy_pos.pose.orientation.w = 1;
                    }
                    else {
                        feedback.detected = false;
                        feedback.error_code = error_info_.error_code();
                        feedback.error_msg = error_info_.error_msg();
                        feedback.enemy_pos.header.frame_id = camera_frame_;
                        feedback.enemy_pos.header.stamp = ros::Time::now();
                        feedback.enemy_pos.pose.position.x = 0;
                        feedback.enemy_pos.pose.position.y = 0;
                        feedback.enemy_pos.pose.position.z = 0;
                        feedback.enemy_pos.pose.orientation.w = 1;
                    }
                    as_.publishFeedback(feedback);
                    // undetected_msg_published = true;
                }
                last_detect_result = detected_enemy_;
            }
            rate.sleep();
        }
    }

    void
    ArmorDetectionNode::ArmorDetectionFeedbackCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr &feedback) {

        double distance, yaw;
        bool detected_enemy, detected_by_front;
        if (feedback->detected) {
            detected_enemy = true;
            detected_by_front = true;
            tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
            geometry_msgs::PoseStamped camera_pose_msg, global_pose_msg;
            camera_pose_msg = feedback->enemy_pos;

            distance = std::sqrt(camera_pose_msg.pose.position.x * camera_pose_msg.pose.position.x +
                                 camera_pose_msg.pose.position.y * camera_pose_msg.pose.position.y);
            yaw = atan2(camera_pose_msg.pose.position.y, camera_pose_msg.pose.position.x);

        } else {
            detected_enemy = false;
            detected_by_front = false;
            distance = 0;
            yaw = 0;
        }

        //fusion of front and side detection
        if (detected_enemy) {
            armor_detection_.detected_enemy = detected_enemy;
            armor_detection_.detected_by_front = detected_by_front;
            armor_detection_.distance = distance;            // mm
            armor_detection_.yaw_angle = yaw + tf_yaw_;      // rad
        } else {
            std::vector<roborts_msgs::SideDetection>::iterator iter;

            roborts_msgs::ArmorDetection min_dist_msg, min_angle_msg;
            float min_distance = 5000;
            float min_angle = 360;
            int detected_num = 0;

            int armor_dist_threshold = 3500;  // mm

            // 得到检测到敌人的相机数量
            for (iter = side_detection_.begin(); iter != side_detection_.end(); iter++)
                if (iter->detected_enemy == true) detected_num++;
            //只有一个相机检测到
            if (detected_num == 1) {
                for (iter = side_detection_.begin(); iter != side_detection_.end(); iter++) {
                    if (iter->detected_enemy) {
                        detected_enemy = true;
                        yaw = iter->yaw;
                        distance = iter->distance;
                    }
                }
            } else if (detected_num >= 2) {
                detected_enemy = true;
                for (iter = side_detection_.begin(); iter != side_detection_.end(); iter++) {
                    if (!iter->detected_enemy)
                        continue;

                    if (min_distance > iter->distance) {
                        min_distance = iter->distance;
                        min_dist_msg.distance = iter->distance;
                        min_dist_msg.yaw_angle = iter->yaw;
                    }

                    if (std::abs(min_angle) > std::abs(iter->yaw)) {
                        min_angle = iter->yaw;
                        min_angle_msg.yaw_angle = iter->yaw;
                        min_angle_msg.distance = iter->distance;
                    }
                }

                if (min_angle_msg.distance == min_dist_msg.distance &&
                    min_angle_msg.distance != 0) {   // min_dist & min_angle are the same
                    yaw = min_dist_msg.yaw_angle;
                    distance = min_dist_msg.distance;

                } else {
                    if (min_angle_msg.distance < armor_dist_threshold && min_dist_msg.distance < armor_dist_threshold) {
                        yaw = min_angle_msg.yaw_angle;
                        distance = min_angle_msg.distance;

                    } else if (std::min(min_angle_msg.distance, min_dist_msg.distance) < armor_dist_threshold) {
                        yaw = min_dist_msg.yaw_angle;
                        distance = min_dist_msg.distance;

                    } else {
                        yaw = min_angle_msg.yaw_angle;
                        distance = min_angle_msg.distance;
                    }
                }
            }

            armor_detection_.detected_enemy = detected_enemy;
            armor_detection_.detected_by_front = detected_by_front;
            armor_detection_.yaw_angle = yaw;
            armor_detection_.distance = distance;
        }

        armor_detection_pub_.publish(armor_detection_);

        // std::cout << "Find:" << detected_enemy << " result(d,a):" << distance << "  " << yaw*180/CV_PI << std::endl;
    }

    void ArmorDetectionNode::ShootSpeedCallback(const roborts_msgs::HeatControl::ConstPtr &msg) {

        ROS_INFO_THROTTLE(1, "get shoot speed: %f", msg->shoot_speed);
        gimbal_control_.init_v_ = msg->shoot_speed;
    }

//side detection callback
    void ArmorDetectionNode::SideDetectionCallback(const roborts_msgs::SideDetection::ConstPtr &msg) {
        int camera_index;
        if (msg->camera_name == "left_camera")
            camera_index = 0;
        else if (msg->camera_name == "right_camera")
            camera_index = 1;
        else if (msg->camera_name == "back_camera")
            camera_index = 2;
        else {
            camera_index = -1;
            ROS_ERROR("No such side camera ~");
        }

        side_detection_[camera_index].detected_enemy = msg->detected_enemy;
        side_detection_[camera_index].distance = msg->distance;
        side_detection_[camera_index].pitch = msg->pitch;
        side_detection_[camera_index].yaw = msg->yaw;

        if(msg->detected_enemy)
            std::cout << msg->camera_name << "   dis:" << msg->distance << "   ang:" << msg->yaw << std::endl;
        // ROS_INFO_THROTTLE(1, "get side detection [%d] info", camera_index);
    }


    void ArmorDetectionNode::ExecuteLoop() {

        undetected_count_ = undetected_armor_delay_;

        ros::Rate rate(50);
        while (running_) {

            if (node_state_ == NodeState::RUNNING) {

                // ros::Time last_time = ros::Time::now();
                ErrorInfo error_info = armor_detector_->DetectArmor(detected_enemy_, target_3d_);
                // ros::Time current_time = ros::Time::now();
                // std::cout << "Time cost: " << current_time - last_time << std::endl;
                {
                    std::lock_guard<std::mutex> guard(mutex_);

                    y_ = -(target_3d_.x + gimbal_control_.offset_.x);
                    z_ = target_3d_.y + gimbal_control_.offset_.y;
                    x_ = target_3d_.z + gimbal_control_.offset_.z;
                    //  std::cout << target_3d_ << std::endl;
                    //  std::cout << "x, y, z : " << x_ << " " << y_ << " " << z_ << std::endl;
                    error_info_ = error_info;
                }

                try {
                    tf_->lookupTransform(robot_base_frame_, gimbal_frame_, ros::Time(0), transform_);
                }
                catch (tf::TransformException ex) {
                    ROS_ERROR("lookupTransform form %s to %s failed!", robot_base_frame_.c_str(),
                              gimbal_frame_.c_str());
                }

                float kalman_pitch, kalman_yaw;
                float pitch, yaw;

                // tf listener
                tf::Quaternion tf_q = transform_.getRotation();
                double tf_roll, tf_pitch;
                tf::Matrix3x3(tf_q).getRPY(tf_roll, tf_pitch, tf_yaw_);
                static float tf_yaw_last = 0, yaw_last = 0;
                gimbal_omega_ = tf_yaw_ - tf_yaw_last;            // rad/period
                tf_yaw_last = tf_yaw_;
                //std::cout << "gimbal_omega: " <<  gimbal_omega_ << std::endl;

                if (detected_enemy_) {

                    gimbal_control_.Transform(target_3d_, pitch, yaw);

                    // KF
                    cv::Mat prediction = kalman_.predict();
                    measurement_.at<float>(0) = pitch;
                    measurement_.at<float>(1) = yaw;
                    kalman_.correct(measurement_);

                    kalman_pitch = kalman_.statePost.at<float>(0);
                    kalman_yaw = kalman_.statePost.at<float>(1);

                    // motion estimation
                    static float yaw_last = 0;
                    float yaw_differential = yaw - yaw_last;         // rad/period
                    // float kalman_yaw_diff = kalman_yaw - kalman_yaw_last;
                    yaw_last = yaw;

                    // shoot estimation
                    // float distance = std::sqrt(x_*x_ + y_*y_) / 1000;  // mm -> m
                    // float fly_time = distance / gimbal_control_.init_v_;
                    // float fly_angle = fly_time * gimbal_omega_;
                    float fly_angle = 0.4 * (gimbal_omega_ / std::abs(gimbal_omega_)) / 180.0 * CV_PI;

                    if (std::abs(gimbal_omega_ * 180 / CV_PI) < 0.005)
                        fly_angle = 0;

                    static float yaw_integral = 0;

                    if ((yaw_integral * gimbal_control_.tracking_ki_ >=
                         gimbal_control_.tracking_integral_limit_ * CV_PI / 180.0
                         && yaw < 0) ||
                        (yaw_integral * gimbal_control_.tracking_ki_ <=
                         -gimbal_control_.tracking_integral_limit_ * CV_PI / 180.0
                         && yaw > 0) ||
                        (std::abs(yaw_integral) * gimbal_control_.tracking_ki_
                         < gimbal_control_.tracking_integral_limit_ * CV_PI / 180.0))
                        yaw_integral += (yaw + fly_angle);

                    gimbal_angle_.yaw_mode = true;
                    gimbal_angle_.pitch_mode = false;
                    if (armor_detector_->armor_first_meeting_ > 0) {
                        gimbal_angle_.yaw_angle = gimbal_control_.first_meet_kp_ * kalman_yaw +
                                                  gimbal_control_.first_meet_kd_ * yaw_differential;
                        yaw_integral = 0;
                    } else
                        gimbal_angle_.yaw_angle = gimbal_control_.tracking_kp_ * kalman_yaw  +
                                                  gimbal_control_.tracking_ki_ * yaw_integral +
                                                  gimbal_control_.tracking_kd_ * yaw_differential;
                    // gimbal_angle_.yaw_angle = kalman_yaw * gimbal_control_.tracking_kp_;
                    gimbal_angle_.pitch_angle = pitch;

                    // std::cout << "yaw_integral: " <<  yaw_integral * 180 / CV_PI << std::endl;


                    // //gimbal control debug --> draw its curves
                    // gimbal_info_.measured_yaw = yaw;
                    // gimbal_info_.kalman_yaw = kalman_yaw;
                    // gimbal_info_.output_yaw = gimbal_angle_.yaw_angle;
                    // gimbal_info_.tf_yaw_diff = gimbal_omega_;
                    // gimbal_info_pub_.publish(gimbal_info_);

                    std::lock_guard<std::mutex> guard(mutex_);
                    undetected_count_ = undetected_armor_delay_;

                    PublishMsgs();
                } else if (undetected_count_ != 0) {

                    gimbal_angle_.yaw_mode = true;
                    gimbal_angle_.pitch_mode = false;
                    gimbal_angle_.yaw_angle = 0;
                    gimbal_angle_.pitch_angle = pitch;

                    undetected_count_--;

                    // //gimbal control debug --> draw its curves
                    // gimbal_info_.measured_yaw = 0;
                    // gimbal_info_.kalman_yaw = 0;
                    // gimbal_info_.output_yaw = 0;
                    // gimbal_info_.tf_yaw_diff = 0;
                    // gimbal_info_pub_.publish(gimbal_info_);

                    KalmanReset();
                    PublishMsgs();
                }

            } else if (node_state_ == NodeState::PAUSE) {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_var_.wait(lock);
            }
            rate.sleep();
        }
    }

    void ArmorDetectionNode::PublishMsgs() {
        enemy_info_pub_.publish(gimbal_angle_);
    }

    void ArmorDetectionNode::StartThread() {
        ROS_INFO("Armor detection node started!");
        running_ = true;
        armor_detector_->SetThreadState(true);
        if (node_state_ == NodeState::IDLE) {
            armor_detection_thread_ = std::thread(&ArmorDetectionNode::ExecuteLoop, this);
        }
        node_state_ = NodeState::RUNNING;
        condition_var_.notify_one();
    }

    void ArmorDetectionNode::PauseThread() {
        ROS_INFO("Armor detection thread paused!");
        node_state_ = NodeState::PAUSE;
    }

    void ArmorDetectionNode::StopThread() {
        node_state_ = NodeState::IDLE;
        running_ = false;
        armor_detector_->SetThreadState(false);
        if (armor_detection_thread_.joinable()) {
            armor_detection_thread_.join();
        }
    }

    ArmorDetectionNode::~ArmorDetectionNode() {
        StopThread();
    }
} //namespace roborts_detection

void SignalHandler(int signal) {
    if (ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()) {
        ros::shutdown();
    }
}

int main(int argc, char **argv) {
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);
    ros::init(argc, argv, "armor_detection_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");
    std::string node_name;
    std::string robot_base_frame_name;
    std::string gimbal_frame_name;
    std::string camera_frame_name;
    node_name = ros::this_node::getName();
    nh.param<std::string>("robot_base_frame_name", robot_base_frame_name, "base_link");
    nh.param<std::string>("gimbal_frame_name", gimbal_frame_name, "gimbal");
    nh.param<std::string>("camera_frame_name", camera_frame_name, "front_camera");
    roborts_detection::ArmorDetectionNode armor_detection(robot_base_frame_name, gimbal_frame_name, camera_frame_name);
    ros::AsyncSpinner async_spinner(2);
    async_spinner.start();
    ros::waitForShutdown();
    armor_detection.StopThread();
    return 0;
}


