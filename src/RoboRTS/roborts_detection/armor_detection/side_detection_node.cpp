#include <unistd.h>
#include "side_detection_node.h"

#include <exception>
#include <sys/ioctl.h>
#include <iostream>
#include <fstream>
#include <string>

#include "../../roborts_base/ros_dep.h"

namespace roborts_detection {

    SideDetectionNode::SideDetectionNode(int camera_num) :
            detected_enemy_(false) {

        nh_.param<std::string>("/robot_name", robot_name, "blue");
        ROS_INFO("using side detection param for %s", robot_name.c_str());

        camera_number_ = camera_num;
        nh_ = ros::NodeHandle();
        gimbal_control_.resize(camera_number_);
        thread_.resize(camera_number_);
        enable_thread_.resize(camera_number_);
        toolbox_.resize(camera_number_);
        armor_detector_.resize(camera_number_);
        detection_res_pub_.resize(camera_number_);
        camera_name_.resize(camera_number_);

        for (int camera_index = 0; camera_index < camera_number_; camera_index++) {

            if (Init(camera_index).IsOK()) {

                if (enable_thread_[camera_index]) {
                    armor_detector_[camera_index]->SetThreadState(true);
                    thread_[camera_index] = std::thread(&SideDetectionNode::DetectionThread, this, camera_index);
                    running_ = true;
                } else {
                    armor_detector_[camera_index]->SetThreadState(false);
                    if (thread_[camera_index].joinable())
                        thread_[camera_index].join();
                }

            } else {
                ROS_ERROR("side_detection_node initalized failed!");
            }
        }
    }

    ErrorInfo SideDetectionNode::DetectionThread(int camera_index) {
        cv::Mat img;
        roborts_msgs::SideDetection detection_result;

        ros::Rate rate(30);
        while (running_) {

            armor_detector_[camera_index]->miniDetectArmor(detected_enemy_, target_3d_);

            // float pitch, yaw;
            float y, z, x, distance, yaw_angle;
            detection_result.camera_name = camera_name_[camera_index];
            if (detected_enemy_) {
                // gimbal_control_[camera_index].Transform(target_3d_, pitch, yaw);
                if(camera_name_[camera_index] == "right_camera")
                    y = -(target_3d_.x + gimbal_control_[camera_index].offset_.x);
                else
                    y = -target_3d_.x + gimbal_control_[camera_index].offset_.x;
                z = target_3d_.y + gimbal_control_[camera_index].offset_.y;
                x = target_3d_.z + gimbal_control_[camera_index].offset_.z;

                std::cout << target_3d_ << std::endl;
                std::cout << "x, y, z : " << x << " " << y << " " << z << std::endl;

                distance = std::sqrt(x * x + y * y);
                if(camera_name_[camera_index] == "back_camera")
                    yaw_angle = atan2(y, x) - gimbal_control_[camera_index].offset_yaw_ / 180.0 * CV_PI;
                else
                    yaw_angle = atan2(y, x) + gimbal_control_[camera_index].offset_yaw_ / 180.0 * CV_PI;
                detection_result.distance = distance;
                detection_result.detected_enemy = detected_enemy_;
                detection_result.pitch = 0;
                detection_result.yaw = yaw_angle;
                detection_res_pub_[camera_index].publish(detection_result);
            } else {
                detection_result.distance = 0;
                detection_result.detected_enemy = detected_enemy_;
                detection_result.pitch = 0;
                detection_result.yaw = 0;
                detection_res_pub_[camera_index].publish(detection_result);
            }

            rate.sleep();
        }
    }

    ErrorInfo SideDetectionNode::Init(int camera_index) {
        Side_Camera side_camera;
        std::string camera_prototxt;
        switch (camera_index) {
            case 0:
                camera_prototxt.assign("back_detection");
                break;
            case 1:
                camera_prototxt.assign("right_detection");
                break;
            case 2:
                camera_prototxt.assign("left_detection");
                break;
            default:
                ROS_WARN("Wrong camera number. No such camera ~");
                break;
        }

        std::string file_name = ros::package::getPath("roborts_detection")
                                + "/armor_detection/config/" + camera_prototxt + "_" + robot_name + ".prototxt";
        // int fd = open(file_name.data(), O_RDONLY);
        // if (fd == -1) {
        //   printf("cannot open %s prototxt file! in \n", camera_prototxt);
        //   return;
        // }
        // using google::protobuf::io::FileInputStream;
        // FileInputStream *input = new FileInputStream(fd);
        bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &side_camera);
        if (!read_state) {
            ROS_ERROR("Cannot open %s", file_name.c_str());
            return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
        }
        camera_name_[camera_index].assign(side_camera.camera_name());
        if(camera_name_[camera_index] == "back_camera")
            gimbal_control_[camera_index].Init(side_camera.camera_gimbal_transform().offset_x(), // (+)
                                            side_camera.camera_gimbal_transform().offset_z(),  // (+)
                                            side_camera.camera_gimbal_transform().offset_y(),  // (+)
                                            side_camera.camera_gimbal_transform().offset_pitch(),
                                            side_camera.camera_gimbal_transform().offset_yaw());
        else 
            gimbal_control_[camera_index].Init(side_camera.camera_gimbal_transform().offset_y(), // (+)
                                            side_camera.camera_gimbal_transform().offset_z(),  // (+)
                                            side_camera.camera_gimbal_transform().offset_x(),  // (+)
                                            side_camera.camera_gimbal_transform().offset_pitch(),
                                            side_camera.camera_gimbal_transform().offset_yaw());
        enable_thread_[camera_index] = side_camera.enable_thread();

        shoot_speed_sub_ = nh_.subscribe<roborts_msgs::HeatControl>("heat_control_info", 1,
                                                                    boost::bind(&SideDetectionNode::ShootSpeedCallback,
                                                                                this,
                                                                                _1));
        detection_res_pub_[camera_index] = nh_.advertise<roborts_msgs::SideDetection>(
                camera_name_[camera_index] + "/detection_result", 1);

        if (camera_index == 0)
            toolbox_[camera_index] = std::make_shared<CVToolbox>(camera_name_[camera_index] + "/color");
        else
            toolbox_[camera_index] = std::make_shared<CVToolbox>(camera_name_[camera_index]);

        armor_detector_[camera_index] = roborts_common::AlgorithmFactory<ArmorDetectionBase, std::shared_ptr<CVToolbox>, std::string>::CreateAlgorithm
                ("constraint_set", toolbox_[camera_index], camera_name_[camera_index]);

        if (armor_detector_[camera_index] == nullptr) {
            ROS_ERROR("Create side armor_detector_ pointer failed!");
            return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
        } else
            return ErrorInfo(ErrorCode::OK);
    }


    void SideDetectionNode::ShootSpeedCallback(const roborts_msgs::HeatControl::ConstPtr &msg) {

        ROS_INFO_THROTTLE(1, "get shoot speed: %f", msg->shoot_speed);
        for (int i = 0; i < camera_number_; i++)
            gimbal_control_[i].init_v_ = msg->shoot_speed;
    }

    SideDetectionNode::~SideDetectionNode() {
        running_ = false;
        for (int i = 0; i < camera_number_; i++) {
            armor_detector_[i]->SetThreadState(false);
            if (thread_[i].joinable())
                thread_[i].join();
        }
    }
}//namespace roborts_detection
void SignalHandler(int signal) {
    if (ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()) {
        ros::shutdown();
    }
}


int main(int argc, char **argv) {
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);
    ros::init(argc, argv, "side_detection_node", ros::init_options::NoSigintHandler);

    roborts_detection::SideDetectionNode side_cameras(3);

    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();
    ros::waitForShutdown();
    return 0;

}
