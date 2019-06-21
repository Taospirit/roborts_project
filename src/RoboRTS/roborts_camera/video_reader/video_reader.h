#ifndef ROBORTS_VIDEO_READER_H
#define ROBORTS_VIDEO_READER_H

#include <thread>

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "actionlib/server/simple_action_server.h"

#include "../camera_param.h"
#include "../camera_base.h"
#include "alg_factory/algorithm_factory.h"
#include "io/io.h"

namespace roborts_camera {
    class Video_Reader : public CameraBase {
    public:

        explicit Video_Reader(CameraInfo camera_info);

        /**
         * @brief Start to read uvc camera
         * @param img Image data in form of cv::Mat to be read
         */
        void StartReadCamera(cv::Mat &img) override;

        /**
         * @brief Stop to read uvc camera
         */
        void StopReadCamera();

        ~Video_Reader() override;

    private:
        /**
         * @brief Set camera exposure
         * @param id Camera path
         * @param val Camera exposure value
         */
        void SetCameraExposure(std::string id, int val);

        //! Initialization of camera read
        bool read_camera_initialized_;

    };

    roborts_common::REGISTER_ALGORITHM(CameraBase, "video_reader", Video_Reader, CameraInfo);

}

#endif
