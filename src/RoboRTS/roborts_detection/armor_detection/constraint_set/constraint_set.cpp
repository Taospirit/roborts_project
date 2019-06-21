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
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include "constraint_set.h"
#include <highgui.h>
#include <cv.h>
#include "timer/timer.h"
#include "io/io.h"

#define FIRST_MEET_COUNT 15

namespace roborts_detection {

    ConstraintSet::ConstraintSet(std::shared_ptr<CVToolbox> cv_toolbox, std::string camera_name) :
            ArmorDetectionBase(cv_toolbox),
            camera_name_(camera_name) {
        filter_x_count_ = 0;
        filter_y_count_ = 0;
        filter_z_count_ = 0;
        filter_distance_count_ = 0;
        filter_pitch_count_ = 0;
        filter_yaw_count_ = 0;
        read_index_ = -1;
        detection_time_ = 0;
        thread_running_ = false;
        detecte_by_tracking_ = false;
        // when firsty meet armor, choose P controller in N frames
        armor_first_meeting_ = FIRST_MEET_COUNT;

        last_armor_center_ = cv::Point2f(0, 0);

        LoadParam();
        error_info_ = ErrorInfo(roborts_common::OK);
    }

    void ConstraintSet::LoadParam() {
        //read parameters
        std::string file_name = ros::package::getPath("roborts_detection") + "/armor_detection/constraint_set/config/";
        file_name += (camera_name_ + "_constraint_set.prototxt");
        bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &constraint_set_config_);
        ROS_ASSERT_MSG(read_state, "Cannot open %s", file_name.c_str());

        enable_debug_ = constraint_set_config_.enable_debug();
        // enemy_color_ = constraint_set_config_.enemy_color();
        nh.param<std::string>("enemy_color", enemy_color, "BLUE");
        enemy_color_ = enemy_color == "BLUE" ? 0 : 1;
        // nh_.param<std::string>("global_frame_name", global_frame_name, "odom");
        ROS_WARN_ONCE("enemy color is ", enemy_color_);
        using_hsv_ = constraint_set_config_.using_hsv();

        //armor info
        float armor_width = constraint_set_config_.armor_size().width();
        float armor_height = constraint_set_config_.armor_size().height();
        SolveArmorCoordinate(armor_width, armor_height);

        //algorithm threshold parameters
        light_max_aspect_ratio_ = constraint_set_config_.threshold().light_max_aspect_ratio();
        light_min_area_ = constraint_set_config_.threshold().light_min_area();
        light_min_angle_ = constraint_set_config_.threshold().light_min_angle();
        light_max_angle_ = 180.0 - light_min_angle_;
        light_max_angle_diff_ = constraint_set_config_.threshold().light_max_angle_diff();
        armor_max_angle_ = constraint_set_config_.threshold().armor_max_angle();
        armor_min_area_ = constraint_set_config_.threshold().armor_min_area();
        armor_max_aspect_ratio_ = constraint_set_config_.threshold().armor_max_aspect_ratio();
        armor_max_pixel_val_ = constraint_set_config_.threshold().armor_max_pixel_val();
        armor_max_stddev_ = constraint_set_config_.threshold().armor_max_stddev();
        armor_max_mean_ = constraint_set_config_.threshold().armor_max_mean();

        color_thread_ = constraint_set_config_.threshold().color_thread();
        blue_thread_ = constraint_set_config_.threshold().blue_thread();
        red_thread_ = constraint_set_config_.threshold().red_thread();

        int get_intrinsic_state = -1;
        int get_distortion_state = -1;

        while ((get_intrinsic_state < 0) || (get_distortion_state < 0)) {
            ROS_WARN("Wait for camera driver launch %d", get_intrinsic_state);
            usleep(50000);
            ros::spinOnce();
            get_intrinsic_state = cv_toolbox_->GetCameraMatrix(intrinsic_matrix_);
            get_distortion_state = cv_toolbox_->GetCameraDistortion(distortion_coeffs_);
        }
    }

    ErrorInfo ConstraintSet::miniDetectArmor(bool &detected, cv::Point3f &target_3d) {

        std::vector<cv::RotatedRect> lights;
        std::vector<ArmorInfo> armors;

        auto img_begin = std::chrono::high_resolution_clock::now();
        bool sleep_by_diff_flag = true;
        while (true) {
            // Ensure exit this thread while call Ctrl-C
            if (!thread_running_) {
                ErrorInfo error_info(ErrorCode::STOP_DETECTION);
                return error_info;
            }

            read_index_ = cv_toolbox_->NextImage(src_img_);

            if (read_index_ < 0) {
                //ROS_INFO("detection_time=%f",detection_time_);
                // Reducing lock and unlock when accessing function 'NextImage'
                if (detection_time_ == 0) {
                    usleep(20000);
                    continue;
                } else {
                    double capture_time = 0;
                    cv_toolbox_->GetCaptureTime(capture_time);
                    //ROS_INFO("capture_time=%f",capture_time);
                    if (capture_time == 0) {
                        // Make sure the driver is launched and the image callback is called
                        usleep(20000);
                        continue;
                    } else if (capture_time > detection_time_ && sleep_by_diff_flag) {
                        // ROS_WARN("time sleep %lf", (capture_time - detection_time_));
                        usleep((unsigned int) (capture_time - detection_time_));
                        sleep_by_diff_flag = false;
                        continue;
                    } else {
                        //For real time request when image call back called, the function 'NextImage' should be called.
                        usleep(500);
                        continue;
                    }
                }
            } else {
                break;
            }
        }

        if(!src_img_.empty()) {

            if(camera_name_ != "back_camera")
                cv::resize(src_img_, src_img_, cv::Size(), 0.5, 0.5);

            if(camera_name_ == "back_camera")
                resize_ratio_ = 1;
            else
                resize_ratio_ = 720.0 / src_img_.rows;

            auto detection_begin = std::chrono::high_resolution_clock::now();

            if (!detecte_by_tracking_)  // searching armor in whole image
            {
                cv::cvtColor(src_img_, gray_img_, CV_BGR2GRAY);
                if (enable_debug_) {
                    show_lights_before_filter_ = src_img_.clone();
                    show_lights_after_filter_ = src_img_.clone();
                    show_armors_befor_filter_ = src_img_.clone();
                    show_armors_after_filter_ = src_img_.clone();
                }

                if (enemy_color_ == RED)
                    red_thread_ = constraint_set_config_.threshold().red_thread();
                else
                    blue_thread_ = constraint_set_config_.threshold().blue_thread();

                DetectLights(src_img_, lights);
                PossibleArmors(lights, armors);
                FilterArmors(armors);

                if (!armors.empty()) {

                    detected = true;
                    detecte_by_tracking_ = true;
                    // when firsty meet armor, choose P controller in N frames
                    if (--armor_first_meeting_ < 0)
                        armor_first_meeting_ = 0;

                    final_armor_ = SelectFinalArmor(armors);

                    float armor_x = (final_armor_.vertex[0].x + final_armor_.vertex[1].x +
                                    final_armor_.vertex[2].x + final_armor_.vertex[3].x) / 4;
                    float armor_y = (final_armor_.vertex[0].y + final_armor_.vertex[1].y +
                                    final_armor_.vertex[2].y + final_armor_.vertex[3].y) / 4;
                    last_armor_center_ = cv::Point2f(armor_x, armor_y);

                    if (enable_debug_) {
                        cv_toolbox_->DrawRotatedRect(src_img_, final_armor_.vertex, cv::Scalar(0, 255, 0), 2);
                    }

                    CalcControlInfo(final_armor_, target_3d);

                    // KCF tracking init
                    cv::Rect armor_rect = final_armor_.rect.boundingRect();
                    int armor_rect_width = armor_rect.width;
                    int armor_rect_height = armor_rect.height;
                    cv::Rect tracker_init_rect(armor_rect.tl().x - armor_rect_width / 2,
                                            armor_rect.tl().y - armor_rect_height / 2,
                                            armor_rect_width * 2, armor_rect_height * 2);

                    tracker_.init(tracker_init_rect, src_img_);
                } else {
                    detected = false;
                    detecte_by_tracking_ = false;
                    // when firsty meet armor, choose P controller in N frames
                    if (--armor_first_meeting_ < 0)
                        armor_first_meeting_ = 0;
                }
            } else   // KCF tracking , searching armor in tracking rect
            {
                cv::Rect tracker_result_rect = tracker_.update(src_img_);

                if (tracker_result_rect.width != 0) {

                    // estimate a ROI region and continue detecting armor again
                    int tl_x = tracker_result_rect.tl().x;
                    int tl_y = tracker_result_rect.tl().y;
                    int br_x = tracker_result_rect.br().x;
                    int br_y = tracker_result_rect.br().y;

                    if (tl_x < 0) tl_x = 0;
                    if (tl_y < 0) tl_y = 0;
                    if (br_x > src_img_.cols) br_x = src_img_.cols;
                    if (br_y > src_img_.rows) br_y = src_img_.rows;

                    tracker_result_rect = cv::Rect(cv::Point(tl_x, tl_y), cv::Point(br_x, br_y));
                    estimated_roi_ = src_img_(cv::Rect(tracker_result_rect)).clone();

                    lights.clear();
                    armors.clear();
                    cv::cvtColor(estimated_roi_, gray_img_, CV_BGR2GRAY);
                    if (enable_debug_) {
                        show_lights_before_filter_ = estimated_roi_.clone();
                        show_lights_after_filter_ = estimated_roi_.clone();
                        show_armors_befor_filter_ = estimated_roi_.clone();
                        show_armors_after_filter_ = estimated_roi_.clone();
                    }

                    if (enemy_color_ == RED)
                        red_thread_ /= 2;
                    else
                        blue_thread_ /= 2;

                    DetectLights(estimated_roi_, lights);
                    PossibleArmors(lights, armors);
                    FilterArmors(armors);
                    if (!armors.empty()) {

                        detected = true;
                        detecte_by_tracking_ = true;
                        // when firsty meet armor, choose P controller in N frames
                        if (--armor_first_meeting_ < 0)
                            armor_first_meeting_ = 0;

                        final_armor_ = SelectFinalArmor(armors);

                        final_armor_.vertex[0] += (cv::Point2f) (tracker_result_rect.tl());
                        final_armor_.vertex[1] += (cv::Point2f) (tracker_result_rect.tl());
                        final_armor_.vertex[2] += (cv::Point2f) (tracker_result_rect.tl());
                        final_armor_.vertex[3] += (cv::Point2f) (tracker_result_rect.tl());

                        float armor_x = (final_armor_.vertex[0].x + final_armor_.vertex[1].x +
                                        final_armor_.vertex[2].x + final_armor_.vertex[3].x) / 4;
                        float armor_y = (final_armor_.vertex[0].y + final_armor_.vertex[1].y +
                                        final_armor_.vertex[2].y + final_armor_.vertex[3].y) / 4;
                        last_armor_center_ = cv::Point2f(armor_x, armor_y);

                        CalcControlInfo(final_armor_, target_3d);

                        if (enable_debug_) {
                            cv_toolbox_->DrawRotatedRect(src_img_, final_armor_.vertex, cv::Scalar(0, 255, 0), 2);
                            cv::rectangle(src_img_, tracker_result_rect, cv::Scalar(0, 0, 255), 2, 8);
                        }
                    } else {
                        detected = false;
                        detecte_by_tracking_ = false;
                        // when firsty meet armor, choose P controller in N frames
                        armor_first_meeting_ = FIRST_MEET_COUNT;
                    }
                }
            }

            // search HP_Bar with final_armor_.vertex
            // if(detected)
            // {
            //   armor_hp_ = CalcArmorHP(final_armor_);
            //   ROS_INFO("HP: %d", armor_hp_);
            // }

            // detect armor number
            // if(detected)
            // {
            //   DetectArmorNum(final_armor_);
            // }


            if (enable_debug_) {
                if (camera_name_ == "front_camera")
                    cv::imshow("front_result_img_", src_img_);
                else if (camera_name_ == "right_camera")
                    cv::imshow("right_result_img_", src_img_);
                else if (camera_name_ == "left_camera")
                    cv::imshow("left_result_img_", src_img_);
                else if (camera_name_ == "back_camera")
                    cv::imshow("back_result_img_", src_img_);
            }

            lights.clear();
            armors.clear();
            cv_toolbox_->ReadComplete(read_index_);
            detection_time_ = std::chrono::duration<double, std::ratio<1, 1000000>>
                    (std::chrono::high_resolution_clock::now() - detection_begin).count();
        }

        return error_info_;
    }

    ErrorInfo ConstraintSet::DetectArmor(bool &detected, cv::Point3f &target_3d) {
        std::vector<cv::RotatedRect> lights;
        std::vector<ArmorInfo> armors;

        auto img_begin = std::chrono::high_resolution_clock::now();
        bool sleep_by_diff_flag = true;
        while (true) {
            // Ensure exit this thread while call Ctrl-C
            if (!thread_running_) {
                ErrorInfo error_info(ErrorCode::STOP_DETECTION);
                return error_info;
            }

            read_index_ = cv_toolbox_->NextImage(src_img_);
            if(camera_name_ == "front_camera")
                resize_ratio_ = 600.0 / src_img_.rows;

            if (read_index_ < 0) {
                //ROS_INFO("detection_time=%f",detection_time_);
                // Reducing lock and unlock when accessing function 'NextImage'
                if (detection_time_ == 0) {
                    usleep(20000);
                    continue;
                } else {
                    double capture_time = 0;
                    cv_toolbox_->GetCaptureTime(capture_time);
                    //ROS_INFO("capture_time=%f",capture_time);
                    if (capture_time == 0) {
                        // Make sure the driver is launched and the image callback is called
                        usleep(20000);
                        continue;
                    } else if (capture_time > detection_time_ && sleep_by_diff_flag) {
                        // ROS_WARN("time sleep %lf", (capture_time - detection_time_));
                        usleep((unsigned int) (capture_time - detection_time_));
                        sleep_by_diff_flag = false;
                        continue;
                    } else {
                        //For real time request when image call back called, the function 'NextImage' should be called.
                        usleep(500);
                        continue;
                    }
                }
            } else {
                break;
            }
        }


        if(!src_img_.empty()) {

            auto detection_begin = std::chrono::high_resolution_clock::now();

            if (!detecte_by_tracking_)  // searching armor in whole image
            {
                cv::cvtColor(src_img_, gray_img_, CV_BGR2GRAY);
                if (enable_debug_) {
                    show_lights_before_filter_ = src_img_.clone();
                    show_lights_after_filter_ = src_img_.clone();
                    show_armors_befor_filter_ = src_img_.clone();
                    show_armors_after_filter_ = src_img_.clone();
                }

                if (enemy_color_ == RED)
                    red_thread_ = constraint_set_config_.threshold().red_thread();
                else
                    blue_thread_ = constraint_set_config_.threshold().blue_thread();

                DetectLights(src_img_, lights);
                PossibleArmors(lights, armors);
                FilterArmors(armors);

                if (!armors.empty()) {

                    detected = true;
                    detecte_by_tracking_ = true;
                    // when firsty meet armor, choose P controller in N frames
                    if (--armor_first_meeting_ < 0)
                        armor_first_meeting_ = 0;

                    final_armor_ = SelectFinalArmor(armors);

                    // creat armor mask
                    // mask_image_ = src_img_(cv::Rect(mask_rect_)).clone();

                    float armor_x = (final_armor_.vertex[0].x + final_armor_.vertex[1].x +
                                    final_armor_.vertex[2].x + final_armor_.vertex[3].x) / 4;
                    float armor_y = (final_armor_.vertex[0].y + final_armor_.vertex[1].y +
                                    final_armor_.vertex[2].y + final_armor_.vertex[3].y) / 4;
                    last_armor_center_ = cv::Point2f(armor_x, armor_y);

                    if (enable_debug_) {
                        cv_toolbox_->DrawRotatedRect(src_img_, final_armor_.vertex, cv::Scalar(0, 255, 0), 2);
                        // cv_toolbox_->DrawRotatedRect(src_img_, mask_rect_, cv::Scalar(255, 0, 0), 2);
                    }

                    CalcControlInfo(final_armor_, target_3d);

                    // KCF tracking init
                    cv::Rect armor_rect = final_armor_.rect.boundingRect();
                    int armor_rect_width = armor_rect.width;
                    int armor_rect_height = armor_rect.height;
                    cv::Rect tracker_init_rect(armor_rect.tl().x - armor_rect_width / 2,
                                            armor_rect.tl().y - armor_rect_height / 2,
                                            armor_rect_width * 2, armor_rect_height * 2);

                    tracker_.init(tracker_init_rect, src_img_);
                } else {
                    detected = false;
                    detecte_by_tracking_ = false;
                    // when firsty meet armor, choose P controller in N frames
                    if (--armor_first_meeting_ < 0)
                        armor_first_meeting_ = 0;
                }
            } else   // KCF tracking , searching armor in tracking rect
            {
                cv::Rect tracker_result_rect = tracker_.update(src_img_);

                if (tracker_result_rect.width != 0) {

                    // estimate a ROI region and continue detecting armor again
                    int tl_x = tracker_result_rect.tl().x;
                    int tl_y = tracker_result_rect.tl().y;
                    int br_x = tracker_result_rect.br().x;
                    int br_y = tracker_result_rect.br().y;

                    if (tl_x < 0) tl_x = 0;
                    if (tl_y < 0) tl_y = 0;
                    if (br_x > src_img_.cols) br_x = src_img_.cols;
                    if (br_y > src_img_.rows) br_y = src_img_.rows;

                    tracker_result_rect = cv::Rect(cv::Point(tl_x, tl_y), cv::Point(br_x, br_y));
                    estimated_roi_ = src_img_(cv::Rect(tracker_result_rect)).clone();

                    lights.clear();
                    armors.clear();
                    cv::cvtColor(estimated_roi_, gray_img_, CV_BGR2GRAY);
                    if (enable_debug_) {
                        show_lights_before_filter_ = estimated_roi_.clone();
                        show_lights_after_filter_ = estimated_roi_.clone();
                        show_armors_befor_filter_ = estimated_roi_.clone();
                        show_armors_after_filter_ = estimated_roi_.clone();
                    }

                    if (enemy_color_ == RED)
                        red_thread_ /= 2;
                    else
                        blue_thread_ /= 2;

                    DetectLights(estimated_roi_, lights);
                    PossibleArmors(lights, armors);
                    FilterArmors(armors);
                    if (!armors.empty()) {

                        detected = true;
                        detecte_by_tracking_ = true;
                        // when firsty meet armor, choose P controller in N frames
                        if (--armor_first_meeting_ < 0)
                            armor_first_meeting_ = 0;

                        final_armor_ = SelectFinalArmor(armors);

                        final_armor_.vertex[0] += (cv::Point2f) (tracker_result_rect.tl());
                        final_armor_.vertex[1] += (cv::Point2f) (tracker_result_rect.tl());
                        final_armor_.vertex[2] += (cv::Point2f) (tracker_result_rect.tl());
                        final_armor_.vertex[3] += (cv::Point2f) (tracker_result_rect.tl());
                        
                        // cv::Point mask_tl = mask_rect_.tl();
                        // cv::Point mask_br = mask_rect_.br();
                        // mask_rect_.tl() = mask_tl + tracker_result_rect.tl();
                        // mask_rect_.br() = mask_br + tracker_result_rect.tl();
                        // mask_rect_.x += tracker_result_rect.x;
                        // mask_rect_.y += tracker_result_rect.y;

                        // if(mask_rect_.tl().x)

                        // std::cout << mask_rect_.x << " " << mask_rect_.y << "  tl:" << mask_rect_.tl() << "  br:" << mask_rect_.br() << std::endl;

                        // creat armor mask
                        // mask_image_ = src_img_(cv::Rect(mask_rect_)).clone();

                        float armor_x = (final_armor_.vertex[0].x + final_armor_.vertex[1].x +
                                        final_armor_.vertex[2].x + final_armor_.vertex[3].x) / 4;
                        float armor_y = (final_armor_.vertex[0].y + final_armor_.vertex[1].y +
                                        final_armor_.vertex[2].y + final_armor_.vertex[3].y) / 4;
                        last_armor_center_ = cv::Point2f(armor_x, armor_y);

                        CalcControlInfo(final_armor_, target_3d);

                        if (enable_debug_) {
                            cv_toolbox_->DrawRotatedRect(src_img_, final_armor_.vertex, cv::Scalar(0, 255, 0), 2);
                            cv::rectangle(src_img_, tracker_result_rect, cv::Scalar(0, 0, 255), 2, 8);
                            // cv_toolbox_->DrawRotatedRect(src_img_, mask_rect_, cv::Scalar(255, 0, 0), 2);
                        }
                    } else {  // tracking, no armor

                        // try to match mask
                        // cv::Mat match_result;
                        // cv::matchTemplate(estimated_roi_, mask_image_, match_result, CV_TM_CCOEFF_NORMED);
                        
                        // double min_match_value, max_match_value;
                        // cv::Point min_match_location, max_match_location;
                        // cv::minMaxLoc(match_result, &min_match_value, &max_match_value, &min_match_location, &max_match_location);

                        // std::cout << "min: " << min_match_value << "  max: " << max_match_value << std::endl;

                        // if(mask_image_.rows > 0 && mask_image_.cols > 0) {
                        //     cv::imshow("template", mask_image_);
                        //     cv::imshow("result", match_result);
                        //     cv::waitKey(0);
                        // }

                        detected = false;
                        detecte_by_tracking_ = false;
                        // when firsty meet armor, choose P controller in N frames
                        armor_first_meeting_ = FIRST_MEET_COUNT;
                    }
                }
            }

            // search HP_Bar with final_armor_.vertex
            // if(detected)
            // {
            //   armor_hp_ = CalcArmorHP(final_armor_);
            //   ROS_INFO("HP: %d", armor_hp_);
            // }

            // detect armor number
            // if(detected)
            // {
            //   DetectArmorNum(final_armor_);
            // }


            if (enable_debug_) {
                if (camera_name_ == "front_camera")
                    cv::imshow("front_result_img_", src_img_);
                else if (camera_name_ == "right_camera")
                    cv::imshow("right_result_img_", src_img_);
                else if (camera_name_ == "left_camera")
                    cv::imshow("left_result_img_", src_img_);
                else if (camera_name_ == "back_camera")
                    cv::imshow("back_result_img_", src_img_);
            }

            lights.clear();
            armors.clear();
            cv_toolbox_->ReadComplete(read_index_);
            detection_time_ = std::chrono::duration<double, std::ratio<1, 1000000>>
                    (std::chrono::high_resolution_clock::now() - detection_begin).count();
        }

        return error_info_;
    }

    void ConstraintSet::DetectLights(const cv::Mat &src, std::vector<cv::RotatedRect> &lights) {
        //std::cout << "********************************************DetectLights********************************************" << std::endl;
        if (using_hsv_) {
            binary_color_img_ = cv_toolbox_->DistillationColor(src, enemy_color_, using_hsv_);
            cv::threshold(gray_img_, binary_brightness_img_, color_thread_, 255, CV_THRESH_BINARY);
        } else {
            auto light = cv_toolbox_->DistillationColor(src, enemy_color_, using_hsv_);

            if(enable_debug_)
            {
              if(camera_name_ == "front_camera")
                cv::imshow("front_light", light);
              else if(camera_name_ == "right_camera")
                cv::imshow("right_light", light);
              else if(camera_name_ == "left_camera")
                cv::imshow("left_light", light);
              else if(camera_name_ == "back_camera")
                cv::imshow("back_light", light);
            }

            // // ---------------------------------------
            // cv::blur(light, light, cv::Size(3, 3));
            // // ---------------------------------------

            cv::threshold(gray_img_, binary_brightness_img_, color_thread_, 255, CV_THRESH_BINARY);

            cv::Mat close_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
            cv::morphologyEx(binary_brightness_img_, binary_brightness_img_, cv::MORPH_CLOSE, close_element);
            // cv::morphologyEx(binary_brightness_img_, binary_brightness_img_, cv::MORPH_OPEN, close_element);


            float thresh;
            if (enemy_color_ == BLUE)
                thresh = blue_thread_;
            else
                thresh = red_thread_;
            cv::threshold(light, binary_color_img_, thresh, 255, CV_THRESH_BINARY);

            // cv::Mat color_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
            // cv::morphologyEx(binary_color_img_, binary_color_img_, cv::MORPH_OPEN, color_element);
        }
        if (enable_debug_) {

          if(camera_name_ == "front_camera") {
            cv::imshow("front_binary_brightness", binary_brightness_img_);
            cv::imshow("front_binary_color", binary_color_img_);
          }
          else if(camera_name_ == "right_camera") {
            cv::imshow("right_binary_brightness", binary_brightness_img_);
            cv::imshow("right_binary_color", binary_color_img_);
          }
          else if(camera_name_ == "left_camera") {
            cv::imshow("left_binary_brightness", binary_brightness_img_);
            cv::imshow("left_binary_color", binary_color_img_);
          }
          else if(camera_name_ == "back_camera") {
            cv::imshow("back_binary_brightness", binary_brightness_img_);
            cv::imshow("back_binary_color", binary_color_img_);
          }
        }

        // auto contours_light = cv_toolbox_->FindContours(binary_color_img_);
        auto contours_brightness = cv_toolbox_->FindContours(binary_brightness_img_);   // white light contours
        // std::cout << "brightness_size:" << contours_brightness.size() << "     light_size:" << contours_light.size() << std::endl;

        lights.clear();
        lights_info_.clear();

        for (int i = 0; i < contours_brightness.size(); ++i) {

            // Filter lights
            cv::RotatedRect single_light = cv::minAreaRect(contours_brightness[i]);
            cv::Point2f vertices_point[4];
            single_light.points(vertices_point);
            LightInfo light_info(vertices_point);

            auto light_aspect_ratio = std::max(single_light.size.width, single_light.size.height) /
                                      std::min(single_light.size.width, single_light.size.height);

            if (light_info.angle_ < light_max_angle_ && light_info.angle_ > light_min_angle_ &&
                light_aspect_ratio < light_max_aspect_ratio_ && light_aspect_ratio > 2.0 &&
                single_light.size.area() >= light_min_area_) {

                cv::Rect contour_rect_brightness = single_light.boundingRect();
                int edge = std::min(contour_rect_brightness.width, contour_rect_brightness.height);
                cv::Point2i color_rect_tl = contour_rect_brightness.tl() - cv::Point2i(0.5 * edge, 0.5 * edge);
                int color_rect_width = contour_rect_brightness.width + edge;
                int color_rect_height = contour_rect_brightness.height + edge;

                if (color_rect_tl.x < 0) {
                    color_rect_width += color_rect_tl.x;
                    color_rect_tl.x = 0;
                }
                if (color_rect_tl.y < 0) {
                    color_rect_height += color_rect_tl.y;
                    color_rect_tl.y = 0;
                }
                if (color_rect_tl.x + color_rect_width > binary_brightness_img_.cols)
                    color_rect_width += (binary_brightness_img_.cols - color_rect_tl.x - color_rect_width);
                if (color_rect_tl.y + color_rect_height > binary_brightness_img_.rows)
                    color_rect_height += (binary_brightness_img_.rows - color_rect_tl.y - color_rect_height);

                cv::Rect contour_rect_color(color_rect_tl.x, color_rect_tl.y, color_rect_width, color_rect_height);

                cv::Mat roi_image_color = cv::Mat::zeros(binary_brightness_img_.size(), CV_8UC1);
                cv::Mat roi_rect_color = roi_image_color(cv::Rect(contour_rect_color));
                cv::Mat roi_binary_color_img = binary_color_img_(cv::Rect(contour_rect_color));
                roi_binary_color_img.copyTo(roi_rect_color);

                auto contours_light_color = cv_toolbox_->FindContours(roi_image_color);

                // cv::imshow("roi_image_color", roi_image_color);
                // std::cout << "brightness_size:" << contours_brightness.size() << "     light_size:" << contours_light_color.size() << std::endl;
                // cv::waitKey(0);

                for (int j = 0; j < contours_light_color.size(); ++j) {
                    cv::Rect contour_rect_color = cv::boundingRect(contours_light_color[j]);
                    int in_num = 0;
                    for (int in = 0; in < contours_brightness.at(i).size(); in += 3) {
                        if (contour_rect_color.contains(contours_brightness[i][in]))
                            in_num++;
                    }
                    if (in_num * 3 > 0.8 * contours_brightness.at(i).size()) //绝大多数（阈值80%）被包围，则判定为装甲板灯带
                    {
                        lights_info_.push_back(light_info);
                        lights.push_back(single_light);

                        if (enable_debug_)
                            cv_toolbox_->DrawRotatedRect(show_lights_before_filter_,
                                                         single_light,
                                                         cv::Scalar(0, 255, 0),
                                                         1,
                                                         light_info.angle_);

                        break;
                    }
                }
            }
        }

        if (enable_debug_)
        {
          if(camera_name_ == "front_camera")
            cv::imshow("front_lights_before_filter", show_lights_before_filter_);
          else if(camera_name_ == "right_camera")
            cv::imshow("right_lights_before_filter", show_lights_before_filter_);
          else if(camera_name_ == "left_camera")
            cv::imshow("left_lights_before_filter", show_lights_before_filter_);
          else if(camera_name_ == "back_camera")
            cv::imshow("back_lights_before_filter", show_lights_before_filter_);
        }

        auto c = cv::waitKey(1);
        if (c == 'a') {
            cv::waitKey(0);
        }
    }

    void ConstraintSet::FilterLights(std::vector<cv::RotatedRect> &lights) {
        //std::cout << "********************************************FilterLights********************************************" << std::endl;
        std::vector<cv::RotatedRect> rects;
        rects.reserve(lights.size());

        // --------------------------------------------
        // use the lights_info.angle_,
        // dont ues the RotatedRect.angle
        std::vector<LightInfo> lights_angle;
        lights_angle.reserve(lights.size());

        for (int i = 0; i < lights.size(); i++) {

            auto light_aspect_ratio =
                    std::max(lights[i].size.width, lights[i].size.height) /
                    std::min(lights[i].size.width, lights[i].size.height);

            if (lights_info_[i].angle_ < light_max_angle_ && lights_info_[i].angle_ > light_min_angle_ &&
                light_aspect_ratio < light_max_aspect_ratio_ && light_aspect_ratio > 1.0 &&  // cancel square light
                lights[i].size.area() >= light_min_area_) { //angle < light_max_angle_ &&
                rects.push_back(lights[i]);

                lights_angle.push_back(lights_info_[i]);

                if (enable_debug_)
                    cv_toolbox_->DrawRotatedRect(show_lights_after_filter_,
                                                 lights[i],
                                                 cv::Scalar(0, 255, 0),
                                                 1,
                                                 lights_info_[i].angle_);
                // -------------------------------------------

            }
        }

        lights = rects;
        lights_info_ = lights_angle;

        if (enable_debug_) {

          if(camera_name_ == "front_camera")
            cv::imshow("front_lights_after_filter", show_lights_after_filter_);
          else if(camera_name_ == "right_camera")
            cv::imshow("right_lights_after_filter", show_lights_after_filter_);
          else if(camera_name_ == "left_camera")
            cv::imshow("left_lights_after_filter", show_lights_after_filter_);
          else if(camera_name_ == "back_camera")
            cv::imshow("back_lights_after_filter", show_lights_after_filter_);
        }

    }

    void ConstraintSet::PossibleArmors(const std::vector<cv::RotatedRect> &lights, std::vector<ArmorInfo> &armors) {
        //std::cout << "********************************************PossibleArmors********************************************" << std::endl;
        for (unsigned int i = 0; i < lights.size(); i++) {
            for (unsigned int j = i + 1; j < lights.size(); j++) {
                cv::RotatedRect light1 = lights[i];
                cv::RotatedRect light2 = lights[j];
                auto edge1 = std::minmax(light1.size.width, light1.size.height);
                auto edge2 = std::minmax(light2.size.width, light2.size.height);
                auto lights_dis = std::sqrt((light1.center.x - light2.center.x) * (light1.center.x - light2.center.x) +
                                            (light1.center.y - light2.center.y) * (light1.center.y - light2.center.y));
                // -------------------------------------------------------------------------------
                float center_angle_for_rect;
                float k = (light1.center.y - light2.center.y) / (light1.center.x - light2.center.x);
                float armor_angle = std::atan((light1.center.y - light2.center.y) /
                                              (light1.center.x - light2.center.x)) / CV_PI * 180.0;

                if (k > 0)
                    center_angle_for_rect = (float) (std::atan(k) * 180.0 / CV_PI);
                else
                    center_angle_for_rect = (float) (180.0 + std::atan(k) * 180.0 / CV_PI);
                // -------------------------------------------------------------------------------

                cv::RotatedRect rect;
                rect.angle = center_angle_for_rect;
                rect.center.x = (light1.center.x + light2.center.x) / 2;
                rect.center.y = (light1.center.y + light2.center.y) / 2;

                float armor_width_min = std::abs(static_cast<float>(lights_dis) - std::max(edge1.first, edge2.first));
                float armor_width_max = std::abs(static_cast<float>(lights_dis) - std::min(edge1.first, edge2.first));
                float armor_height_min = std::min<float>(edge1.second, edge2.second);
                float armor_height_max = std::max<float>(edge1.second, edge2.second);

                rect.size.width = std::max<float>(armor_width_min, armor_height_max);
                rect.size.height = std::min<float>(armor_width_min, armor_height_max);

                float light1_angle = lights_info_[i].angle_;
                float light2_angle = lights_info_[j].angle_;
                auto angle_diff = std::abs(light1_angle - light2_angle);

                // Avoid incorrect calculation at 180 and 0.
                if (angle_diff > 175) {
                    angle_diff = 180 - angle_diff;
                }

                float edge_second_ration =
                        std::max<float>(edge1.second, edge2.second) / std::min<float>(edge1.second, edge2.second);
                float armor_aspect_ratio_max = armor_width_max / armor_height_min;
                float armor_aspect_ratio_min = armor_width_min / armor_height_max;
                float armor_area = std::abs(rect.size.area());
                // int armor_center_pixel_val = gray_img_.at<uchar>(static_cast<int>(rect.center.y), static_cast<int>(rect.center.x));


                // std::cout << std::endl << lights_info_[i].angle_ << " - " << lights_info_[j].angle_ << std::endl;
                // std::cout << "angle_diff: " << angle_diff << std::endl;
                // std::cout << "edge_second_ration: " << edge_second_ration << std::endl;
                // std::cout << "armor_aspect_ratio: " << armor_aspect_ratio_max << "  " << armor_aspect_ratio_min << std::endl;
                // std::cout << "armor_area: " << armor_area << std::endl;
                // std::cout << "std::abs(armor_angle): " << std::abs(armor_angle) << std::endl;

                if (angle_diff < light_max_angle_diff_ && edge_second_ration < 1.8 &&
                    armor_aspect_ratio_max < armor_max_aspect_ratio_ && armor_aspect_ratio_min >= 1.0 &&
                    armor_area > armor_min_area_ && std::abs(armor_angle) < armor_max_angle_) {
                    // armor_center_pixel_val < armor_max_pixel_val_

                    std::vector<cv::Point2f> armor_points;
                    if (light1.center.x < light2.center.x) {

                        // add the index to find light angle
                        CalcArmorInfo(armor_points, light1, light2, i, j);
                    } else {

                        // add the index to find light angle
                        CalcArmorInfo(armor_points, light2, light1, j, i);
                    }

                    // filter wrong vertices rectangle
                    // if(armor_points[0].y < armor_points[3].y && armor_points[1].y < armor_points[2].y)
                    armors.emplace_back(ArmorInfo(rect, armor_points, armor_angle, i, j));

                    if (enable_debug_)
                    {
                        cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, armor_points, cv::Scalar(0, 255, 0), 2);
                        cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, rect, cv::Scalar(255, 0, 0), 2);
                    }

                    armor_points.clear();
                }
            }
        }

        if(camera_name_ == "front_camera")
        {
        
            cv::waitKey(1);
        }

        if (enable_debug_)
        {
          if(camera_name_ == "front_camera")
            cv::imshow("front_armors_befor_filter", show_armors_befor_filter_);
          else if(camera_name_ == "right_camera")
            cv::imshow("right_armors_befor_filter", show_armors_befor_filter_);
          else if(camera_name_ == "left_camera")
            cv::imshow("left_armors_befor_filter", show_armors_befor_filter_);
          else if(camera_name_ == "back_camera")
            cv::imshow("back_armors_befor_filter", show_armors_befor_filter_);
        }

    }

    void ConstraintSet::FilterArmors(std::vector<ArmorInfo> &armors) {
        //std::cout << "********************************************FilterArmors********************************************" << std::endl;
        // cv::Mat mask = cv::Mat::zeros(gray_img_.size(), CV_8UC1);
        // for (auto armor_iter = armors.begin(); armor_iter != armors.end();) {
        //   cv::Point pts[4];
        //   for (unsigned int i = 0; i < 4; i++) {
        //     pts[i].x = (int) armor_iter->vertex[i].x;
        //     pts[i].y = (int) armor_iter->vertex[i].y;
        //   }
        //   cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255), 8, 0);

        //   cv::Mat mat_mean;
        //   cv::Mat mat_stddev;
        //   cv::meanStdDev(gray_img_, mat_mean, mat_stddev, mask);

        //   auto stddev = mat_stddev.at<double>(0, 0);
        //   auto mean = mat_mean.at<double>(0, 0);

        //   if (stddev > armor_max_stddev_ || mean > armor_max_mean_) {
        //     armor_iter = armors.erase(armor_iter);
        //   } else {
        //     armor_iter++;
        //   }
        // }

        // nms
        std::vector<bool> is_armor(armors.size(), true);
        for (int i = 0; i < armors.size() && is_armor[i] == true; i++) {
            for (int j = i + 1; j < armors.size() && is_armor[j]; j++) {

                float dx = armors[i].rect.center.x - armors[j].rect.center.x;
                float dy = armors[i].rect.center.y - armors[j].rect.center.y;
                float dis = std::sqrt(dx * dx + dy * dy);
                //filter the armor reflected by the ground
                if (std::abs(dy * 1.0 / dx) > 20) {
                    if (dy > 0) //delete the lower armor
                        is_armor[i] = false;
                    else
                        is_armor[j] = false;
                }
                //---
                if (dis < armors[i].rect.size.width + armors[j].rect.size.width) {
                    if (std::abs(armors[i].angle) > std::abs(armors[j].angle))
                        is_armor[i] = false;
                    else
                        is_armor[j] = false;
                }

                // check if two armors contain the same light
                if (armors[i].light1 == armors[j].light1 ||
                    armors[i].light2 == armors[j].light2 ||
                    armors[i].light1 == armors[j].light2 ||
                    armors[i].light2 == armors[j].light1) {
                    if (std::abs(armors[i].angle) > std::abs(armors[j].angle))
                        is_armor[i] = false;
                    else
                        is_armor[j] = false;
                }
            }
        }

        for (unsigned int i = 0; i < armors.size(); i++) {
            if (!is_armor[i]) {
                armors.erase(armors.begin() + i);
                is_armor.erase(is_armor.begin() + i);
                //after erasing, i points to next element
                i--;
            }
        }
        if (enable_debug_) {
          for (int i = 0; i < armors.size(); i++) {
            cv_toolbox_->DrawRotatedRect(show_armors_after_filter_, armors[i].vertex, cv::Scalar(0, 255, 0), 2);
            // std::cout << "armors[" << i << "].light1, 2: " << armors[i].light1 << "  " << armors[i].light2 << std:: endl;
          }

          if(camera_name_ == "front_camera")
            cv::imshow("front_armors_after_filter", show_armors_after_filter_);
          else if(camera_name_ == "right_camera")
            cv::imshow("right_armors_after_filter", show_armors_after_filter_);
          else if(camera_name_ == "left_camera")
            cv::imshow("left_armors_after_filter", show_armors_after_filter_);
          else if(camera_name_ == "back_camera")
            cv::imshow("back_armors_after_filter", show_armors_after_filter_);
        }
    }

// -------------------
// Choose closet armor
// -------------------
    ArmorInfo ConstraintSet::SelectFinalArmor(std::vector<ArmorInfo> &armors) {

        cv::Point2f current_armor_center;
        float center_dis, min_center_dis = 10000;
        int return_index;

        for (int i = 0; i < armors.size(); i++) {
            float armor_x = (armors[i].vertex[0].x + armors[i].vertex[1].x +
                             armors[i].vertex[2].x + armors[i].vertex[3].x) / 4;
            float armor_y = (armors[i].vertex[0].y + armors[i].vertex[1].y +
                             armors[i].vertex[2].y + armors[i].vertex[3].y) / 4;
            current_armor_center = cv::Point2f(armor_x, armor_y);

            center_dis = std::sqrt(std::pow(current_armor_center.x - last_armor_center_.x, 2) +
                                   std::pow(current_armor_center.y - last_armor_center_.y, 2));

            if (center_dis < min_center_dis) {
                min_center_dis = center_dis;
                return_index = i;
            }
        }
        return armors[return_index];
    }

    int ConstraintSet::CalcArmorHP(ArmorInfo &final_armor) {  // uncompleted

        // float delta_x_03 = (final_armor.vertex[0].x - final_armor.vertex[3].x) / 6.0;
        // float delta_y_03 = (final_armor.vertex[0].y - final_armor.vertex[3].y) / 6.0;
        // float delta_x_01 = (final_armor.vertex[0].x - final_armor.vertex[1].x) / 12.0;
        // float delta_y_01 = (final_armor.vertex[0].y - final_armor.vertex[1].y) / 12.0;
        // float delta_x_12 = (final_armor.vertex[1].x - final_armor.vertex[2].x) / 6.0;
        // float delta_y_12 = (final_armor.vertex[1].y - final_armor.vertex[2].y) / 6.0;

        // // estimate armor hp bar rect
        // std::vector<cv::Point2f> vertices(4);
        // if (final_armor_.angle < 0) {
        //   vertices[0].x = final_armor.vertex[0].x + delta_x_03 * 13 + delta_x_01 * 10;
        //   vertices[0].y = final_armor.vertex[0].y + delta_y_03 * 13 + delta_y_01 * 10;
        //   vertices[1] = vertices[0] - cv::Point2f(32 * delta_x_01, 32 * delta_y_01);
        //   vertices[3] = vertices[0] - cv::Point2f(6 * delta_x_03, 6 * delta_y_03);
        //   vertices[2] = vertices[1] - cv::Point2f(6 * delta_x_03, 6 * delta_y_03);
        // } else {
        //   vertices[1].x = final_armor.vertex[1].x + delta_x_12 * 13 - delta_x_01 * 10;
        //   vertices[1].y = final_armor.vertex[1].y + delta_y_12 * 13 - delta_y_01 * 10;
        //   vertices[0] = vertices[1] - cv::Point2f(-32 * delta_x_01, -32 * delta_y_01);
        //   vertices[2] = vertices[1] - cv::Point2f(6 * delta_x_12, 6 * delta_y_12);
        //   vertices[3] = vertices[0] - cv::Point2f(6 * delta_x_12, 6 * delta_y_12);
        // }

        // int img_width = src_img_.cols;
        // int img_height = src_img_.rows;

        // // avoid estimated rect out of image
        // for (int index = 0; index < 4; index++) {
        //   if (vertices[index].x < 0)
        //     vertices[index].x = 0;
        //   else if (vertices[index].x > img_width)
        //     vertices[index].x = img_width;

        //   if (vertices[index].y < 0)
        //     vertices[index].y = 0;
        //   else if (vertices[index].y > img_height)
        //     vertices[index].y = img_height;
        // }
        // cv::RotatedRect hp_bar_rotated_rect = cv::minAreaRect(vertices);
        // cv::Rect hp_bar_rect = hp_bar_rotated_rect.boundingRect();

        // cv::Mat hp_bar_gray_image;
        // cv::Mat hp_bar_brightness_image;

        // cv::Mat estimated_roi = src_img_(cv::Rect(hp_bar_rect)).clone();
        // cv::cvtColor(estimated_roi, hp_bar_gray_image, CV_BGR2GRAY);
        // cv::threshold(hp_bar_gray_image, hp_bar_brightness_image, 0, color_thread_, CV_THRESH_OTSU);
        // cv::Mat close_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        // cv::morphologyEx(hp_bar_brightness_image, hp_bar_brightness_image, cv::MORPH_CLOSE, close_element);

        // auto contours_brightness = cv_toolbox_->FindContours(hp_bar_brightness_image);   // white light contours
        // if (contours_brightness.size() != 0) {
        //   int longest_contours = 0;
        //   int contour_index = 0;
        //   for (int i = 0; i < contours_brightness.size(); i++) {
        //     int contours_size = contours_brightness[i].size();
        //     if (longest_contours < contours_size) {
        //       longest_contours = contours_size;
        //       contour_index = i;
        //     }
        //   }
        //   cv::RotatedRect hp_bar_light = cv::minAreaRect(contours_brightness[contour_index]);
        //   cv::Point2f hp_bar_light_vertices[4];
        //   hp_bar_light.points(hp_bar_light_vertices);
        //   float hp_bar_width, armor_width, hp_ratio;

        //   // hp_bar
        //   float angle_1 = std::abs(std::atan((hp_bar_light_vertices[0].x - hp_bar_light_vertices[1].x) /
        //     (hp_bar_light_vertices[0].y - hp_bar_light_vertices[1].y)));
        //   float angle_2 = std::abs(std::atan((hp_bar_light_vertices[0].x - hp_bar_light_vertices[3].x) /
        //     (hp_bar_light_vertices[0].y - hp_bar_light_vertices[3].y)));
        //   // large angle is height
        //   if (angle_1 < angle_2)
        //     hp_bar_width = hp_bar_light.size.width;
        //   else
        //     hp_bar_width = hp_bar_light.size.height;

        //   // armor
        //   float width_1 = std::sqrt(std::pow(final_armor.vertex[0].x - final_armor.vertex[1].x, 2) +
        //     std::pow(final_armor.vertex[0].y - final_armor.vertex[1].y, 2));
        //   float width_2 = std::sqrt(std::pow(final_armor.vertex[2].x - final_armor.vertex[3].x, 2) +
        //     std::pow(final_armor.vertex[2].y - final_armor.vertex[3].y, 2));
        //   armor_width = (width_1 + width_2) / 2.0;

        //   hp_ratio = hp_bar_width / armor_width;   // full hp ratio is 2.0
        //   int hp = hp_ratio * 1000;
        //   if (hp > 2000)
        //     hp = 2000;

        //   cv_toolbox_->DrawRotatedRect(estimated_roi, hp_bar_light, cv::Scalar(0, 0, 255), 2);
        //   cv::imshow("HP_Bar_bri", hp_bar_brightness_image);
        //   cv::imshow("src_img", estimated_roi);
        //   return hp;
        // } else
        //   return -1;
    }

    void ConstraintSet::DetectArmorNum(ArmorInfo &final_armor) {
        // dst rect 60 * 120 mm
        cv::Point2f src_points[3];
        cv::Point2f dst_points[3];
        src_points[0] = final_armor.vertex[0];
        src_points[1] = final_armor.vertex[1];
        src_points[2] = final_armor.vertex[2];
        dst_points[0] = cv::Point2d(0, 0);
        dst_points[1] = cv::Point2d(120, 0);
        dst_points[2] = cv::Point2d(120, 60);

        cv::Mat affine_transform = cv::getAffineTransform(src_points, dst_points);

        cv::Mat affine_image;
        cv::warpAffine(src_img_, affine_image, affine_transform, src_img_.size());
        cv::Mat armor_image = affine_image(cv::Rect(30, 0, 60, 60)).clone();


        cv::Mat armor_image_gray, armor_image_binary, armor_num_mask;
        cv::cvtColor(armor_image, armor_image_gray, CV_BGR2GRAY);


        cv::Mat mat_mean;
        cv::Mat mat_stddev;
        cv::meanStdDev(armor_image_gray, mat_mean, mat_stddev);
        double stddev = mat_stddev.at<double>(0, 0);
        double mean = mat_mean.at<double>(0, 0);
        std::cout << "before: " << stddev << " std  --  mean " << mean << std::endl;

        cv::equalizeHist(armor_image_gray, armor_image_gray);

        cv::meanStdDev(armor_image_gray, mat_mean, mat_stddev);
        stddev = mat_stddev.at<double>(0, 0);
        mean = mat_mean.at<double>(0, 0);
        std::cout << "after: " << stddev << " std  --  mean " << mean << std::endl;

        cv::threshold(armor_image_gray, armor_image_binary, 0, 255, CV_THRESH_OTSU);
        cv::Mat close_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(armor_image_binary, armor_image_binary, cv::MORPH_CLOSE, close_element);

        cv::imshow("before preocess", armor_image);


        NumImage = armor_image;
        cv::imshow("affine whole", affine_image);

        cv::imshow("after affine", armor_image_binary);
        cv::waitKey(0);
    }

    void ConstraintSet::CalcControlInfo(ArmorInfo &armor, cv::Point3f &target_3d) {
        cv::Mat rvec;
        cv::Mat tvec;

        armor.vertex[0] *= resize_ratio_;
        armor.vertex[1] *= resize_ratio_;
        armor.vertex[2] *= resize_ratio_;
        armor.vertex[3] *= resize_ratio_;

        cv::solvePnP(armor_points_,
                     armor.vertex,
                     intrinsic_matrix_,
                     distortion_coeffs_,
                     rvec,
                     tvec);
        target_3d = cv::Point3f(tvec);

        armor.vertex[0] /= resize_ratio_;
        armor.vertex[1] /= resize_ratio_;
        armor.vertex[2] /= resize_ratio_;
        armor.vertex[3] /= resize_ratio_;
    }

// --------------------------------------------------
// add the light index to find angle, and fix DJI Bug
// --------------------------------------------------
    void ConstraintSet::CalcArmorInfo(std::vector<cv::Point2f> &armor_points,
                                      cv::RotatedRect left_light,
                                      cv::RotatedRect right_light,
                                      int left_light_index,
                                      int right_light_index) {
        cv::Point2f left_points[4], right_points[4];
        left_light.points(left_points);
        right_light.points(right_points);

        float left_light_angle = lights_info_[left_light_index].angle_;
        float left_light_width = lights_info_[left_light_index].width_;
        float left_light_height = lights_info_[left_light_index].height_;

        float right_light_angle = lights_info_[right_light_index].angle_;
        float right_light_width = lights_info_[right_light_index].width_;
        float right_light_height = lights_info_[right_light_index].height_;

        float left_beta = (float) (std::atan(left_light_height / left_light_width) * 180.0 / CV_PI);
        float right_beta = (float) (std::atan(right_light_height / right_light_width) * 180.0 / CV_PI);

        SortLightvertices(left_points, left_light_angle, left_beta);
        SortLightvertices(right_points, right_light_angle, right_beta);

        armor_points.push_back(left_points[1]);
        armor_points.push_back(right_points[0]);
        armor_points.push_back(right_points[3]);
        armor_points.push_back(left_points[2]);

        std::vector<cv::Point2f> mask_points;
        mask_points.push_back(left_points[0]);
        mask_points.push_back(right_points[1]);
        mask_points.push_back(right_points[2]);
        mask_points.push_back(left_points[3]);
        
        // mask_rect_ = cv::boundingRect(mask_points);
    }

// ---------------------------------------
// @brief  Sort the vertices of light rectangle
// @param  vertices is four vertices of light rectangle
// @param  Alpha angle is the angle of rectangle with 0 degree towards to left and the range of (0,180)
// @param  Beta angle is the minimal inside angle of rectangle
// ---------------------------------------
    void ConstraintSet::SortLightvertices(cv::Point2f vertices[], float alpha_angle, float beta_angle) {

        cv::Point2f vertices_sorted[4];
        std::sort(vertices, vertices + 4, [](const cv::Point2f &p1, const cv::Point2f &p2) { return p1.x < p2.x; });

        if (alpha_angle == 90.0) {

            if (vertices[0].y < vertices[1].y) {
                vertices_sorted[0] = vertices[0];
                vertices_sorted[3] = vertices[1];
            } else {
                vertices_sorted[0] = vertices[1];
                vertices_sorted[3] = vertices[0];
            }

            if (vertices[2].y < vertices[3].y) {
                vertices_sorted[1] = vertices[2];
                vertices_sorted[2] = vertices[3];
            } else {
                vertices_sorted[1] = vertices[3];
                vertices_sorted[2] = vertices[2];
            }

        } else {

            // alpha_angle < 90
            if (alpha_angle < 90.0) {

                if (alpha_angle + beta_angle < 90.0) {
                    vertices_sorted[0] = vertices[0];
                    vertices_sorted[1] = vertices[1];
                    vertices_sorted[2] = vertices[3];
                    vertices_sorted[3] = vertices[2];
                } else if (alpha_angle + beta_angle > 90.0) {
                    vertices_sorted[0] = vertices[0];
                    vertices_sorted[1] = vertices[2];
                    vertices_sorted[2] = vertices[3];
                    vertices_sorted[3] = vertices[1];
                } else {
                    // alpha + beta = 90 , sort vertex.y
                    std::sort(vertices, vertices + 4,
                              [](const cv::Point2f &p1, const cv::Point2f &p2) { return p1.y < p2.y; });
                    vertices_sorted[0] = vertices[1];
                    vertices_sorted[1] = vertices[0];
                    vertices_sorted[2] = vertices[2];
                    vertices_sorted[3] = vertices[3];
                }
            } else {

                // alpha_angle > 90
                if (alpha_angle - beta_angle > 90.0) {
                    vertices_sorted[0] = vertices[2];
                    vertices_sorted[1] = vertices[3];
                    vertices_sorted[2] = vertices[1];
                    vertices_sorted[3] = vertices[0];
                } else if (alpha_angle - beta_angle < 90.0) {
                    vertices_sorted[0] = vertices[1];
                    vertices_sorted[1] = vertices[3];
                    vertices_sorted[2] = vertices[2];
                    vertices_sorted[3] = vertices[0];
                } else {
                    // alpha_angle - beta_angle = 90 , sort vertex.y
                    std::sort(vertices, vertices + 4,
                              [](const cv::Point2f &p1, const cv::Point2f &p2) { return p1.y < p2.y; });
                    vertices_sorted[0] = vertices[0];
                    vertices_sorted[1] = vertices[1];
                    vertices_sorted[2] = vertices[3];
                    vertices_sorted[3] = vertices[2];
                }
            }
        }
        vertices[0] = vertices_sorted[0];
        vertices[1] = vertices_sorted[1];
        vertices[2] = vertices_sorted[2];
        vertices[3] = vertices_sorted[3];
    }

    void ConstraintSet::SolveArmorCoordinate(const float width,
                                             const float height) {
        armor_points_.emplace_back(cv::Point3f(-width / 2, height / 2, 0.0));
        armor_points_.emplace_back(cv::Point3f(width / 2, height / 2, 0.0));
        armor_points_.emplace_back(cv::Point3f(width / 2, -height / 2, 0.0));
        armor_points_.emplace_back(cv::Point3f(-width / 2, -height / 2, 0.0));
    }

    void ConstraintSet::SignalFilter(double &new_num, double &old_num, unsigned int &filter_count, double max_diff) {
        if (fabs(new_num - old_num) > max_diff && filter_count < 2) {
            filter_count++;
            new_num += max_diff;
        } else {
            filter_count = 0;
            old_num = new_num;
        }
    }

    void ConstraintSet::SetThreadState(bool thread_state) {
        thread_running_ = thread_state;
    }

    ConstraintSet::~ConstraintSet() {

    }
} //namespace roborts_detection
