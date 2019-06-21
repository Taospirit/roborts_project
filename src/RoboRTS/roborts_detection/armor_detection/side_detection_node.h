#ifndef  SIDE_DETECTION_NODE_H
#define SIDE_DETECTION_NODE_H

#include <thread>
#include <mutex>
#include <vector>
#include <condition_variable>
#include <boost/thread.hpp>
#include "io/io.h"
#include "state/node_state.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "cv_toolbox.h"
#include "armor_detection_base.h"
#include "gimbal_control.h"
#include "armor_detection_algorithms.h"
#include "constraint_set/constraint_set.h"
#include "proto/side_detecton.pb.h"
#include "roborts_msgs/HeatControl.h"

typedef struct {
    bool enemy_detection;
    float angle;
} SideDetectionMsg;
namespace roborts_detection {
    using roborts_common::NodeState;

    class SideDetectionNode {
    public:
        /**
         * @brief constructor.
         * @param which camera is in using;
         *        0: left camera
         *        1: right camera
         *        2: back camera
         */
        SideDetectionNode(int camera_num);

        /**
         * @brief Initialize.
         * @param which camera is in using;
         *        0: left camera
         *        1: right camera
         *        2: back camera
         *        other: error
         */
        ErrorInfo Init(int camera_index);

        /**
         * @brief subscribe shoot speed from heat control to calc pitch angle.
         */
        void ShootSpeedCallback(const roborts_msgs::HeatControl::ConstPtr &msg);

        ErrorInfo DetectionThread(int camera_index);

        ~SideDetectionNode();

    private:

        std::vector<bool> enable_thread_;
        std::vector<std::thread> thread_;
        std::vector<std::shared_ptr<CVToolbox>> toolbox_;
        std::vector<std::string> camera_name_;

        int camera_number_;

        //! detection
        std::vector<std::shared_ptr<ArmorDetectionBase>> armor_detector_;
        bool detected_enemy_;
        cv::Point3f target_3d_;

        ros::NodeHandle nh_;
        std::vector<ros::Publisher> detection_res_pub_;
        ros::Subscriber shoot_speed_sub_;
        bool running_;

        std::string robot_name;

        //! control model
        std::vector<GimbalContrl> gimbal_control_;

    };

}

#endif
