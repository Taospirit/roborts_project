/****************************************************************************
 *  Copyright (C) HITSZ.
 *  Reference to 2019 RoboMaster
 *  该驱动为迈德威视摄像头驱动，型号为MVSUA133GC-T
 *  迈德威视SDK包：http://www.mindvision.com.cn/rjxz/list_12.aspx?lcid=138
 ***************************************************************************/

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include "mvsua_driver.h"

namespace roborts_camera {
    MVSUADriver::MVSUADriver(CameraInfo camera_info) :
            CameraBase(camera_info) {
        firstimg_Flag = true;
        camera_initialized_ = MVSUA_Init();
        if (camera_initialized_) LoadParaFromProto();
        ROS_INFO("Create MVSUADriver to read image from MVSUA Camera.");

    }

    int MVSUADriver::MVSetResolution(int width, int height, int w_offset, int h_offset) {
        //用于居中
//    int WidthMax=g_tCapability.sResolutionRange.iWidthMax;
//    int HeightMax=g_tCapability.sResolutionRange.iHeightMax;
//    int w_offset = (WidthMax-width)/2;
//    int h_offset = (HeightMax-height)/2;
        tSdkImageResolution *p = g_tCapability.pImageSizeDesc;
        p->iWidth = width;
        p->iHeight = height;
        p->iHOffsetFOV = w_offset;
        p->iVOffsetFOV = h_offset;
        return CameraSetImageResolution(m_hCamera, p);
    }

    cv::Mat MVSUADriver::CaptureImage() {
        using namespace cv;
        Mat img;
        static int error_counter=0;
        if (CameraGetImageBuffer(m_hCamera, &g_tFrameHead, &g_pRawBuffer, 1000) == CAMERA_STATUS_SUCCESS) {
            CameraImageProcess(m_hCamera, g_pRawBuffer, g_pRgbBuffer, &g_tFrameHead);
            IplImage *iplimage = cvCreateImageHeader(cvSize(g_tFrameHead.iWidth, g_tFrameHead.iHeight), IPL_DEPTH_8U,
                                                     3);
            cvSetData(iplimage, g_pRgbBuffer, g_tFrameHead.iWidth * 3);
            CameraReleaseImageBuffer(m_hCamera, g_pRawBuffer);
            img = cvarrToMat(iplimage);
            int tWidth = img.cols * mImgResizeRatio;
            int tHeight = img.rows * mImgResizeRatio;
            resize(img, img, Size(tWidth, tHeight));
            if (firstimg_Flag) {
                firstimg_Flag = false;
                printf("Image Width:%d,Image Height:%d\n", g_tFrameHead.iWidth, g_tFrameHead.iHeight);
            }
        } else {
            std::cout << "mvsua camera can not CaptureImage" << std::endl;
            error_counter++;
            if(error_counter>10)
            {
                error_counter=0;
                CameraUnInit(m_hCamera);    // 释放相机，准备重新启动
                usleep(20);
                camera_initialized_=MVSUA_Init();
                if (camera_initialized_) LoadParaFromProto();
                ROS_INFO("MUSUA Camera can not CaptureImage, try to restart it...");
            }
        }
        return img;
    }

    void MVSUADriver::LoadParaFromProto() {
        using namespace std;
        MVCamera::MVSUA133 MVSUAPara;
        std::string file_name = ros::package::getPath("roborts_camera") + "/config/mvsua_param.prototxt";
        int fd = open(file_name.data(), O_RDONLY);
        if (fd == -1) {
            printf("cannot open prototxt file!\n");
            return;
        }
        using google::protobuf::io::FileInputStream;
        FileInputStream *input = new FileInputStream(fd);
        bool success = google::protobuf::TextFormat::Parse(input, &MVSUAPara);
        if (success) printf("prototxt load successfully\n");
        else printf("prototxt load failed\n");
        //设置曝光模式：手动或者自动
        CameraSetAeState(m_hCamera, MVSUAPara.aestate());
        if (MVSUAPara.aestate())//自动曝光时目标亮度值
        {
            CameraSetAeTarget(m_hCamera, MVSUAPara.aetarget());
            CameraSetAntiFlick(m_hCamera, MVSUAPara.antiflick());
            //是否抑制频闪
            if (MVSUAPara.antiflick()) {
                CameraSetLightFrequency(m_hCamera, MVSUAPara.lightfrequency());
            };

        } else {

            CameraSetAnalogGain(m_hCamera, MVSUAPara.analoggain());//手动曝光增益
            CameraSetExposureTime(m_hCamera, MVSUAPara.exposuretime() * MVSUAPara.exposurelinetime());//手动曝光时间
        }
        CameraSetGamma(m_hCamera, MVSUAPara.gamma());
        CameraSetSaturation(m_hCamera, MVSUAPara.saturation());
        CameraSetSharpness(m_hCamera, MVSUAPara.shaprpness());
        CameraSetMirror(m_hCamera, 0, MVSUAPara.mirror_h());
        CameraSetMirror(m_hCamera, 1, MVSUAPara.mirror_v());
        CameraSetGain(m_hCamera, MVSUAPara.rgain(), MVSUAPara.ggain(), MVSUAPara.bgain());
        if (MVSUAPara.has_width() && MVSUAPara.has_height()) {
            MVSetResolution(MVSUAPara.width(), MVSUAPara.height(), MVSUAPara.w_offset(), MVSUAPara.h_offset());
            ROS_INFO("Init MV Camera [Width,Height]=[%d,%d] [w_offset,h_offset]=[%d,%d]",
                     MVSUAPara.width(),
                     MVSUAPara.height(),
                     MVSUAPara.w_offset(),
                     MVSUAPara.h_offset());
        }
        if (MVSUAPara.has_speed()) {
            CameraSetFrameSpeed(m_hCamera, MVSUAPara.speed()); //0-2,2为最高帧率输出
            ROS_INFO("Init MV Camera Speed Mode=%d", MVSUAPara.speed());
        }
        if (MVSUAPara.has_resize_ratio()) {
            mImgResizeRatio = MVSUAPara.resize_ratio();
        } else {
            mImgResizeRatio = 1;
        }
        CameraPlay(m_hCamera);
        delete input;
        close(fd);
    }

    bool MVSUADriver::MVSUA_Init() {
        int iCameraCounts = 4;
        int iStatus = -1;
        tSdkCameraDevInfo tCameraEnumList[4];
        CameraSdkInit(1);//初始化，内置SDK中文提示（0为英文）
        CameraEnumerateDevice(tCameraEnumList, &iCameraCounts);
        if (iCameraCounts == 0)//没有连接设备
        {
            ROS_INFO("No MV Driver connected!");
            ROS_INFO("You should check the connection or /etc/udev/rules.d/88-mvusb.rules");
            return false;
        }
        //初始化第一个相机，并使用上次退出时的参数
        iStatus = CameraInit(&tCameraEnumList[0], -1, -1, &m_hCamera);
        //初始化失败
        if (iStatus != CAMERA_STATUS_SUCCESS) {
            ROS_INFO("Camera Init failed!");
            return false;
        }
        CameraGetCapability(m_hCamera, &g_tCapability);
        g_pRgbBuffer = (unsigned char *) malloc(g_tCapability.sResolutionRange.iHeightMax
                                                * g_tCapability.sResolutionRange.iWidthMax * 3);
        g_readBuf = (unsigned char *) malloc(g_tCapability.sResolutionRange.iHeightMax
                                             * g_tCapability.sResolutionRange.iWidthMax * 3);
        //设置相机为彩色输出
        CameraSetIspOutFormat(m_hCamera, CAMERA_MEDIA_TYPE_BGR8);

        return true;
    }

    void MVSUADriver::StartReadCamera(cv::Mat &img) {
        if (!camera_initialized_) {
            camera_initialized_ = MVSUA_Init();
            if (camera_initialized_) LoadParaFromProto();
        } else {
            img = CaptureImage();
        }
    }

    void MVSUADriver::StopReadCamera() {
        //TODO: To be implemented
        CameraUnInit(m_hCamera);
    }

    MVSUADriver::~MVSUADriver() {
        ROS_INFO("Release MV Camera");
        CameraUnInit(m_hCamera);    //必须释放相机
    }

} //namespace roborts_camera
