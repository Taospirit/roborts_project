#ifndef PROTOLOADER_H
#define PROTOLOADER_H

#include <google/protobuf/message.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include "proto/MVCamera_para.pb.h"
#include <sys/time.h>
#include "inc/CameraApi.h"
#include "stdio.h"
#include "CameraApi.h"
#include "mainwindow.h"

extern int g_hCamera;           //设备句柄
extern unsigned char *g_pRgbBuffer;      //处理后数据缓存区
extern tSdkFrameHead g_tFrameHead;        //图像帧头信息
extern tSdkCameraCapbility g_tCapability;       //设备描述信息


extern pthread_t g_thread_id;            //线程
extern pthread_t g_thread_getcounts;     //线程
extern BYTE *g_readBuf;             //画板显示数据区
extern int g_display_state;        //显示状态
extern int g_disply_fps;           //统计帧率
extern int g_SaveParameter_num;    //保存参数组
extern int g_SaveImage_type;       //保存图像格式



/*加载相机参数*/
/*2019-3-10*/
int MVCameraParaLoad(char save_path[], Width_Height *g_W_H_INFO) {
    using namespace std;
    MVCamera::MVSUA133 MVSUAPara;
    int fd = open(save_path, O_RDONLY);
    if (fd == -1) {
        printf("cannot creat prototxt file!\n");
        return -1;
    }
    using google::protobuf::io::FileInputStream;
    FileInputStream *input = new FileInputStream(fd);
    bool success = google::protobuf::TextFormat::Parse(input, &MVSUAPara);
    if (success) printf("prototxt load to %s successfully\n", save_path);
    else printf("prototxt save failed\n");

    tSdkImageResolution sLResolution;  //获取当前设置到分辨率

    CameraGetResolutionForSnap(g_hCamera, &sLResolution);
    sLResolution.iWidth = MVSUAPara.width();
    sLResolution.iHeight = MVSUAPara.height();
    sLResolution.iWidthFOV = MVSUAPara.w_offset();
    sLResolution.iHeightFOV = MVSUAPara.h_offset();
    CameraSetImageResolution(g_hCamera, &sLResolution);
    CameraSetFrameSpeed(g_hCamera, MVSUAPara.speed());

    g_W_H_INFO->sensor_width = MVSUAPara.width();
    g_W_H_INFO->sensor_height = MVSUAPara.height();
    g_W_H_INFO->buffer_size = g_W_H_INFO->sensor_width * g_W_H_INFO->sensor_height;
    CameraSetAeState(g_hCamera, MVSUAPara.aestate());
    if (MVSUAPara.aestate()) {
        CameraSetAeTarget(g_hCamera, MVSUAPara.aetarget());
        CameraSetAntiFlick(g_hCamera, MVSUAPara.antiflick());
        if (MVSUAPara.antiflick()) {
            CameraSetLightFrequency(g_hCamera, MVSUAPara.lightfrequency());
        };

    } else {
        CameraSetAnalogGain(g_hCamera, MVSUAPara.analoggain());
        CameraSetExposureTime(g_hCamera, MVSUAPara.exposuretime() * MVSUAPara.exposurelinetime());
//        CameraSetExposureLineTime(g_hCamera,MVSUAPara.exposurelinetime());
    }
    CameraSetGamma(g_hCamera, MVSUAPara.gamma());
    CameraSetSaturation(g_hCamera, MVSUAPara.saturation());
    CameraSetSharpness(g_hCamera, MVSUAPara.shaprpness());
    CameraSetMirror(g_hCamera, 0, MVSUAPara.mirror_h());
    CameraSetMirror(g_hCamera, 1, MVSUAPara.mirror_v());
    CameraSetGain(g_hCamera, MVSUAPara.rgain(), MVSUAPara.ggain(), MVSUAPara.bgain());

    delete input;
    close(fd);
    return 1;
}

/*保存相机参数*/
/*2019-3-10*/
void save_asprototxt(char save_path[50]) {
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    MVCamera::MVSUA133 MVCameraPara;
    BOOL AEstate, AntiFlick;
    tSdkImageResolution sResolution;  //获取当前设置到分辨率
    int ResolutionIndex, AeTarget, LightFrequency, AnalogGain;
    double ExposureTime, ExposureLineTime;
    CameraGetResolutionForSnap(g_hCamera, &sResolution);

    MVCameraPara.set_resolutionindex(sResolution.iIndex);
    MVCameraPara.set_width(sResolution.iWidth);
    MVCameraPara.set_height(sResolution.iHeight);
    MVCameraPara.set_w_offset(sResolution.iHOffsetFOV);
    MVCameraPara.set_h_offset(sResolution.iVOffsetFOV);

    int speed_mode;
    CameraGetFrameSpeed(g_hCamera, &speed_mode);
    MVCameraPara.set_speed(speed_mode);
    CameraGetAeState(g_hCamera, &AEstate);
    MVCameraPara.set_aestate(AEstate);

    if (AEstate) {
        CameraGetAeTarget(g_hCamera, &AeTarget);
        CameraGetAntiFlick(g_hCamera, &AntiFlick);
        MVCameraPara.set_aetarget(AeTarget);
        MVCameraPara.set_antiflick(AntiFlick);
        if (AntiFlick) {
            CameraGetLightFrequency(g_hCamera, &LightFrequency);
            MVCameraPara.set_lightfrequency(LightFrequency);
        }
    } else {
        CameraGetAnalogGain(g_hCamera, &AnalogGain);
        CameraGetExposureTime(g_hCamera, &ExposureTime);
        CameraGetExposureLineTime(g_hCamera, &ExposureLineTime);
        MVCameraPara.set_analoggain(AnalogGain);
        MVCameraPara.set_exposuretime(ExposureTime / ExposureLineTime);
        MVCameraPara.set_exposurelinetime(ExposureLineTime);

        CameraSetAnalogGain(g_hCamera, AnalogGain);
        CameraSetExposureTime(g_hCamera, ExposureTime);
//         CameraSetExposureLineTime(g_hCamera,  &ExposureLineTime);
    }
    int Gamma, Contrast;
    CameraGetGamma(g_hCamera, &Gamma);
    CameraGetContrast(g_hCamera, &Contrast);
    MVCameraPara.set_gamma(Gamma);
    MVCameraPara.set_contrast(Contrast);

    int RPos, GPos, BPos, Saturation, Sharpness, Dir;
    BOOL H_Dir, V_Dir;
    CameraGetGain(g_hCamera, &RPos, &GPos, &BPos);
    CameraGetSaturation(g_hCamera, &Saturation);
    CameraGetSharpness(g_hCamera, &Sharpness);
    CameraGetMirror(g_hCamera, 0, &H_Dir); //水平
    CameraGetMirror(g_hCamera, 1, &V_Dir);
    MVCameraPara.set_ggain(GPos);
    MVCameraPara.set_rgain(RPos);
    MVCameraPara.set_bgain(BPos);
    MVCameraPara.set_saturation(Saturation);
    MVCameraPara.set_shaprpness(Sharpness);
    MVCameraPara.set_mirror_h(H_Dir);
    MVCameraPara.set_mirror_v(V_Dir);

    int fd = open(save_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd == -1) {
        printf("cannot open prototxt file!\n");
        return;
    }
    using google::protobuf::io::FileOutputStream;
    FileOutputStream *output = new FileOutputStream(fd);
    bool success = google::protobuf::TextFormat::Print(MVCameraPara, output);
    if (success) printf("prototxt open successfully\n");
    else printf("prototxt open failed\n");
    delete output;
    close(fd);

}

#endif // PROTOLOADER_H
