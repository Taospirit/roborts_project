/****************************************************************************
 *  Copyright (C) HITSZ
 *  Reference to 2019 RoboMaster
 ***************************************************************************/

#ifndef ROBORTS_CAMERA_MVSUA_DRIVER_H
#define ROBORTS_CAMERA_MVSUA_DRIVER_H

#include <thread>

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "actionlib/server/simple_action_server.h"

#include "../camera_param.h"
#include "../camera_base.h"
#include "alg_factory/algorithm_factory.h"
#include "io/io.h"
#include "MVSDK/CameraApi.h"
#include "../proto/MVCamera_para.pb.h"

namespace roborts_camera {
/**
 * @brief UVC Camera class, product of the camera factory inherited from CameraBase
 */
    class MVSUADriver : public CameraBase {
    public:
        /**
         * @brief Constructor of MVSUADriver
         * @param camera_info  Information and parameters of camera
         */
        explicit MVSUADriver(CameraInfo camera_info);

        /**
         * @brief Start to read uvc camera
         * @param img Image data in form of cv::Mat to be read
         */
        void StartReadCamera(cv::Mat &img) override;

        /**
         * @brief Stop to read uvc camera
         */
        void StopReadCamera();

        ~MVSUADriver() override;

    private:
        /**
         * @brief Set camera exposure
         * @param id Camera path
         * @param val Camera exposure value
         */
        cv::Mat CaptureImage();

        void LoadParaFromProto();

        bool MVSUA_Init();

        int MVSetResolution(int width, int height, int w_offset, int h_offset);
        /**
         * @brief 设置相机分辨率，截取中间部分
         */
        //! Initialization of camera read
        bool read_camera_initialized_;
        bool firstimg_Flag;                               //用于输出第一帧图像的信息。
        int m_hCamera;
        tSdkCameraCapbility g_tCapability;      //设备描述信息
        unsigned char *g_pRawBuffer = NULL;  //raw数据
        unsigned char *g_pRgbBuffer = NULL;  //处理后数据缓存区
        BYTE *g_readBuf = NULL;     //画板显示数据区
        tSdkFrameHead g_tFrameHead;       //图像帧头信息
        double mImgResizeRatio;

    };

    roborts_common::REGISTER_ALGORITHM(CameraBase, "mvsua", MVSUADriver, CameraInfo);

} //namespace roborts_camera
#endif //ROBORTS_CAMERA_UVC_DRIVER_H
