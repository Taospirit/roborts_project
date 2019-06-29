/*
 * Copyright (c) 2019 Critical_HIT.
 * All rights reserved.
 */

#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
#include <vector>
#include "io/io.h"
#include "../proto/rgbd_scan_fusion.pb.h"

#include "roborts_msgs/SupplyDistance.h"
#include "roborts_msgs/FacePosition.h"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::Image> my_sync_policy;

class RGBDScanFusion {
protected:
    // Our NodeHandle
    ros::NodeHandle nh_;

    // Message Filter and Time Synchronizer
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_;
    message_filters::Synchronizer<my_sync_policy> sync_;

    // Depth image to calculate distance between realsense and wall
    message_filters::Subscriber<sensor_msgs::Image> supply_depth_image_sub_;
    message_filters::Subscriber<roborts_msgs::FacePosition> face_position_sub_;

    ros::Publisher supply_distance_pub_;

    // Components for publishing
    sensor_msgs::LaserScan fused_scan_;
    ros::Publisher output_pub_;

    // images
    cv::Mat depth_image_;
    cv::Mat supply_depth_image_;
    std::vector<cv::Vec2f> laser_vector_;

    // Camera Param
    float left_max_angle_; // degree
    float right_max_angle_;
    float top_max_angle_;
    float bottom_max_angle_;
    float max_distance_;

    // vertical/horizontal direction center length, for calculate different direction pixel angle
    float left_center_dist_;
    float right_center_dist_;
    float top_center_dist_;
    float bottom_center_dist_;

    // delta height/width between RGB-D camera & Laser_scan
    float delta_height_; // mm
    float delta_width_;

    bool enable_fusion_;

    int center_x;
    int center_y;
    int offset_x;
    int offset_y;

public:
    // Constructor
    RGBDScanFusion() : scan_sub_(nh_, "scan_filtered_before_fusion", 1),
                       depth_image_sub_(nh_, "back_camera/depth/image_rect_raw", 1),
                       supply_depth_image_sub_(nh_, "back_camera/depth/image_rect_raw", 1),
                       face_position_sub_(nh_, "face_position", 1),
                       sync_(my_sync_policy(10), scan_sub_, depth_image_sub_) {
        laser_filters::RGBDScanParam rgbd_scan_param;
        std::string file_name = ros::package::getPath("laser_filters") + "/config/rgbd_scan_fusion.prototxt";
        int fd = open(file_name.data(), O_RDONLY);
        if (fd == -1) {
            ROS_ERROR("cannot open prototxt file!\n");
            return;
        }
        google::protobuf::io::FileInputStream *input = new google::protobuf::io::FileInputStream(fd);
        bool success = google::protobuf::TextFormat::Parse(input, &rgbd_scan_param);
        if (success)
            ROS_INFO("prototxt load successfully\n");
        else
            ROS_ERROR("prototxt load failed\n");

        // camera param initialize
        left_max_angle_ = rgbd_scan_param.left_max_angle();
        right_max_angle_ = rgbd_scan_param.right_max_angle();
        top_max_angle_ = rgbd_scan_param.top_max_angle();
        bottom_max_angle_ = rgbd_scan_param.bottom_max_angle();

        left_center_dist_ = 320.0 / (tan(left_max_angle_ / 180.0 * CV_PI));
        right_center_dist_ = 320.0 / (tan(right_max_angle_ / 180.0 * CV_PI));
        top_center_dist_ = 240.0 / (tan(top_max_angle_ / 180.0 * CV_PI));
        bottom_center_dist_ = 240.0 / (tan(bottom_max_angle_ / 180.0 * CV_PI));

        delta_height_ = rgbd_scan_param.delta_height();
        delta_width_ = rgbd_scan_param.delta_width();
        max_distance_ = rgbd_scan_param.max_distance();
        enable_fusion_ = rgbd_scan_param.enable_fusion();

        center_x = 0;
        center_y = 0;
        offset_x = 0;
        offset_y = 0;

        // Advertise output
        output_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_filtered", 1);

        if (enable_fusion_)
            sync_.registerCallback(boost::bind(&RGBDScanFusion::sync_callback, this, _1, _2));
        else
            scan_sub_.registerCallback(boost::bind(&RGBDScanFusion::scan_callback, this, _1));

        face_position_sub_.registerCallback(boost::bind(&RGBDScanFusion::face_callback, this, _1));
        supply_depth_image_sub_.registerCallback(boost::bind(&RGBDScanFusion::depth_image_callback, this, _1));
        supply_distance_pub_ = nh_.advertise<roborts_msgs::SupplyDistance>("supply_distance", 1);
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        ROS_INFO_THROTTLE(1, "Publish Raw Laser_Scan");
        output_pub_.publish(scan_msg);
    }

    void face_callback(const roborts_msgs::FacePosition::ConstPtr &face_position){
        center_x = face_position->face_center_x;
        center_y = face_position->face_center_y;
        offset_x = face_position->face_offset_x / 2;
        offset_y = face_position->face_offset_y / 2;
    }

    // depth image callback
    void depth_image_callback(const sensor_msgs::Image::ConstPtr &depth_msg) {
        
        try {
            supply_depth_image_ = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image.clone();
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("Could not convert from ‘%s‘ to ‘32fc1‘.", depth_msg->encoding.c_str());
        }

        float supply_distance = 0; 
        // int depth_filter_size = 50;

        if(center_x != 0)
        {
            int pixel_size = 0;
            float temp_depth = 0;
            if(!supply_depth_image_.empty())
            {
                for(int i = -offset_y; i < offset_y + 1; i++)
                {
                    for(int j = -offset_x; j < offset_x + 1; j++)
                    {
                        if(supply_depth_image_.at<float>(center_y+i, center_x+j) != 0){
                            temp_depth = supply_depth_image_.at<float>(center_y+i, center_x+j);
                            supply_distance = (supply_distance * pixel_size + temp_depth) / (pixel_size + 1);
                            pixel_size ++;
                        }
                        // std::cout << supply_distance;
                    }
                }

                // if(pixel_size != 0)
                //     supply_distance = supply_distance / pixel_size;
                // else
                //     supply_distance = -1;
            }

        }
        // else
        //     supply_distance = 0
        
        // std::cout << "depth image callback: " << supply_depth_image_.size() << std::endl;

        roborts_msgs::SupplyDistance supply_distance_msg;
        supply_distance_msg.supply_distance = supply_distance;
        supply_distance_pub_.publish(supply_distance_msg);
    }

    // Callback
    void
    sync_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg, const sensor_msgs::Image::ConstPtr &depth_msg) {
        // Fusion rgbd_scan.
        try {
            depth_image_ = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image.clone();
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("Could not convert from ‘%s‘ to ‘32fc1‘.", depth_msg->encoding.c_str());
        }

        laser_vector_ = depth_image_to_laser_image(depth_image_);
        fused_scan_ = fusion_laser_scan(scan_msg, laser_vector_);

        output_pub_.publish(fused_scan_);
        ROS_INFO_THROTTLE(1, "Fusion!");
    }

    // calculate scan data from depth image
    std::vector<cv::Vec2f> depth_image_to_laser_image(cv::Mat &depth_image) {
        int width = depth_image.cols;
        int height = depth_image.rows;
        std::vector<cv::Vec2f> laser_vector;

        for (int i = 0; i < width; i++) {
            float depth_angle;
            float laser_angle;
            float laser_distance; // rad / mm  left - right +
            float min_delta_height = 10000;
            int index_i = i - width / 2; // horizontal

            if (index_i > 0) // right + left -
                depth_angle = atan(index_i / right_center_dist_);
            else
                depth_angle = atan(index_i / left_center_dist_);

            // find pixel whose height is close to laser_scan
            for (int j = 0; j < height; j++) {
                float v_angle;                // rad
                int index_j = j - height / 2; // vertical

                if (index_j > 0) // bottom + top -
                    v_angle = atan(index_j / bottom_center_dist_);
                else
                    v_angle = atan(index_j / top_center_dist_);

                float depth = depth_image.at<float>(j, i);
                if (depth == 0)
                    depth = 10000;

                float delta_height = fabs(delta_height_ - depth * tan(v_angle) / cos(depth_angle));
                if (min_delta_height > delta_height) {
                    min_delta_height = delta_height;
                    laser_distance = depth / cos(depth_angle);
                }
            }

            // convert rgbd_position distance & angle to laser_position
            float phy_ctr_dist = laser_distance * sin(depth_angle);
            float lsr_dist = laser_distance * cos(depth_angle) + delta_width_;
            laser_angle = atan(phy_ctr_dist / lsr_dist);
            if (fabs(laser_angle) < 0.25 / 180.0 * CV_PI)
                laser_distance = lsr_dist;
            else
                laser_distance = phy_ctr_dist / sin(laser_angle);

            int int_angle = (int) (laser_angle / CV_PI * 180.0 * 2); //not real angel, sequence in 720 data point
            if (int_angle < 0)
                int_angle += 720;

            laser_vector.push_back(cv::Vec2f(laser_distance, int_angle));
        }

        return laser_vector;
    }

    // fusion laser image and laser scan raw
    sensor_msgs::LaserScan fusion_laser_scan(const sensor_msgs::LaserScan::ConstPtr &scan_msg,
                                             std::vector<cv::Vec2f> &laser_vector) {
        std::vector<cv::Vec2f> laser_vector_filtered;
        cv::Vec2f laser_min_distance = laser_vector[0];
        for (int i = 0; i < laser_vector.size(); i++) {
            cv::Vec2f laser_current = laser_vector[i];
            if (laser_current[1] == laser_min_distance[1] && laser_current[0] < laser_min_distance[0]) {
                laser_min_distance = laser_current;
            }
            if (laser_current[1] != laser_min_distance[1]) {
                laser_vector_filtered.push_back(laser_min_distance);
                laser_min_distance = laser_current;
            }
        }

        // after filtered, scan_msg.ranges changed!
        sensor_msgs::LaserScan fused_scan;
        fused_scan = *scan_msg;

        sensor_msgs::LaserScan::_ranges_type ranges(720);

        for (int i = 0; i < fused_scan.ranges.size(); i++) {
            if (fused_scan.ranges[i] > fused_scan.range_max) {
                ROS_INFO_THROTTLE(1, "fusing depth image to laser");
                for (int j = 0; j < laser_vector_filtered.size(); j++) {
                    if (laser_vector_filtered[j][1] == i)
                        if (laser_vector_filtered[j][0] < max_distance_)
                            ranges[720 - i] = laser_vector_filtered[j][0] / 1000; //not clockwise direction
                }
            } else
                ranges[i] = fused_scan.ranges[i];
        }
        fused_scan.ranges = ranges;
        return fused_scan;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "rgbd_scan_fusion");

    RGBDScanFusion t;
    ros::spin();

    return 0;
}
