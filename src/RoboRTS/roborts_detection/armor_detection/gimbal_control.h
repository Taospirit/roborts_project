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
#ifndef ROBORTS_DETECTION_GIMBAL_CONTROL_H
#define ROBORTS_DETECTION_GIMBAL_CONTROL_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

namespace roborts_detection {

    const double PI = 3.1415926535;
    const float GRAVITY = 9.78;

/**
 * @brief The class can make a transformation: the 3D position of enemy -->  pitch,yaw angle of gimbal.
 * For more derails, see projectile_model.pdf
 * TODO: add enemy motion estimation
 */

    class GimbalContrl {
    private:
        /**
         * @brief Calculate the actual y value with air resistance
         * @param x the distanc
         * @param v Projectile velocity
         * @param angle Pitch angle
         * @return The actual y value in the gimbal coordinate
         */
        float BulletModel(float x, float v, float angle);

        /**
         * @brief Get the gimbal control angle
         * @param x Distance from enemy(the armor selected to shoot) to gimbal
         * @param y Value of y in gimbal coordinate.
         * @param v Projectile velocity
         * @return Gimbal pitch angle
         */
        float GetPitch(float x, float y, float v);

    public:
        /**
         * @brief Init the Transformation matrix from camera to gimbal //TODO: write in ros tf
         * @param x Translate x
         * @param y Translate y
         * @param z Translate z
         * @param pitch Rotate pitch
         * @param yaw Rotate yaw
         */
        void Init(float x, float y, float z, float pitch, float yaw,
                  float init_v = 20,
                  float init_k = 0.036,
                  float first_meet_kp = 0,
                  float first_meet_kd = 0,
                  float tracking_kp = 0,
                  float tracking_ki = 0,
                  float tracking_kd = 0,
                  float tracking_integral_limit = 0
        );

        /**
         * @brief Get the gimbal control info.
         * @param postion Enemy position(actually it should be the target armor).
         * @param pitch Input and output actual pitch angle
         * @param yaw Input and output actual yaw angle
         */
        void Transform(cv::Point3f &postion, float &pitch, float &yaw);

    public:
        //! Transformation matrix between camera coordinate system and gimbal coordinate system.
        //! Translation unit: cm
        cv::Point3f offset_;
        //! Rotation matrix unit: degree
        float offset_pitch_;
        float offset_yaw_;

        //! Initial value
        float init_v_;
        float init_k_;

        // control param
        float first_meet_kp_;
        float first_meet_kd_;
        float tracking_kp_;
        float tracking_ki_;
        float tracking_kd_;
        float tracking_integral_limit_;

    };

} //namespace roborts_detection

#endif //ROBORTS_DETECTION_GIMBAL_CONTROL_H
