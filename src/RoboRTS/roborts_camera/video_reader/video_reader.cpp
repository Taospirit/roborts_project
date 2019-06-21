
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include "video_reader.h"

namespace roborts_camera {
    Video_Reader::Video_Reader(CameraInfo camera_info) :
            CameraBase(camera_info) {
        ROS_INFO("Creat video_reader to read video.");
    }

    void Video_Reader::StartReadCamera(cv::Mat &img) {
        if (!camera_initialized_) {
            camera_initialized_ = true;
            camera_info_.cap_handle.open(camera_info_.camera_path);
            ROS_ASSERT_MSG(camera_info_.cap_handle.isOpened(), "Cannot open %s .",
                           cameras_[camera_num].video_path.c_str());
        } else {
            bool f = camera_info_.cap_handle.read(img);
            ROS_ASSERT_MSG(f, "video was at the end");

//        camera_info_.cap_handle >> img;
        }
    }

    void Video_Reader::StopReadCamera() {
        //TODO: To be implemented
    }

    void Video_Reader::SetCameraExposure(std::string id, int val) {
    }

    Video_Reader::~Video_Reader() {
    }

}
