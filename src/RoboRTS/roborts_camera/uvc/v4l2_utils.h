#ifndef V4L2_UTISL_H
#define V4L2_UTILS_H

#include "linux/videodev2.h"

#include "io/io.h"
#include <thread>
#include <sys/ioctl.h>

int enum_frame_intervals(int dev, __u32 pixfmt, __u32 width, __u32 height) {
    int ret;
    struct v4l2_frmivalenum fival;

    memset(&fival, 0, sizeof(fival));
    fival.index = 0;
    fival.pixel_format = pixfmt;
    fival.width = width;
    fival.height = height;
    printf("\tTime interval between frame: ");
    while ((ret = ioctl(dev, VIDIOC_ENUM_FRAMEINTERVALS, &fival)) == 0) {
        if (fival.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
            printf("%u/%u, ",
                   fival.discrete.numerator, fival.discrete.denominator); //输出分数
        } else if (fival.type == V4L2_FRMIVAL_TYPE_CONTINUOUS) {
            printf("{min { %u/%u } .. max { %u/%u } }, ",
                   fival.stepwise.min.numerator, fival.stepwise.min.numerator,
                   fival.stepwise.max.denominator, fival.stepwise.max.denominator);
            break;
        } else if (fival.type == V4L2_FRMIVAL_TYPE_STEPWISE) {
            printf("{min { %u/%u } .. max { %u/%u } / "
                   "stepsize { %u/%u } }, ",
                   fival.stepwise.min.numerator, fival.stepwise.min.denominator,
                   fival.stepwise.max.numerator, fival.stepwise.max.denominator,
                   fival.stepwise.step.numerator, fival.stepwise.step.denominator);
            break;
        }
        fival.index++;
    }
    printf("\n");
    if (ret != 0 && errno != EINVAL) {
        printf("ERROR enumerating frame intervals: %d\n", errno);
        return errno;
    }

    return 0;
}

int enum_frame_sizes(int dev, __u32 pixfmt) {
    int ret;
    struct v4l2_frmsizeenum fsize;

    memset(&fsize, 0, sizeof(fsize));
    fsize.index = 0;
    fsize.pixel_format = pixfmt;
    while ((ret = ioctl(dev, VIDIOC_ENUM_FRAMESIZES, &fsize)) == 0) {
        if (fsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
            printf("  {discrete: width = %u, height = %u}\n",
                   fsize.discrete.width, fsize.discrete.height);
            ret = enum_frame_intervals(dev, pixfmt,
                                       fsize.discrete.width, fsize.discrete.height);  //查找设备支持的 帧的间隔时间
            if (ret != 0)
                printf("  Unable to enumerate frame sizes.\n");
        } else if (fsize.type == V4L2_FRMSIZE_TYPE_CONTINUOUS) {
            printf("  {continuous: min { width = %u, height = %u} .. "
                   "  max { width = %u, height = %u } }\n",
                   fsize.stepwise.min_width, fsize.stepwise.min_height,
                   fsize.stepwise.max_width, fsize.stepwise.max_height);
            printf("  Refusing to enumerate frame intervals.\n");
            break;
        } else if (fsize.type == V4L2_FRMSIZE_TYPE_STEPWISE) {
            printf("{  stepwise: min { width = %u, height = %u} .. "
                   "  max { width = %u, height = %u } / "
                   "  stepsize { width = %u, height = %u } }\n",
                   fsize.stepwise.min_width, fsize.stepwise.min_height,
                   fsize.stepwise.max_width, fsize.stepwise.max_height,
                   fsize.stepwise.step_width, fsize.stepwise.step_height);
            printf("  Refusing to enumerate frame intervals.\n");
            break;
        }
        fsize.index++;
    }
    if (ret != 0 && errno != EINVAL) {
        printf("  ERROR enumerating frame sizes: %d\n", errno);
        return errno;
    }

    return 0;
}

void printUVCProperty(int cam_fd) {

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_fmtdesc fmt;
    struct v4l2_frmsizeenum frmsize;
    struct v4l2_format format;

    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    struct v4l2_fmtdesc fmtdesc;
    fmtdesc.index = 0;
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    //获取当前摄像头支持的格式
    while (ioctl(cam_fd, VIDIOC_ENUM_FMT, &fmtdesc) != -1) {
        fmtdesc.index++;
        printf("{ pixelformat = '%c%c%c%c', description = '%s' }\n",
               fmtdesc.pixelformat & 0xFF, (fmtdesc.pixelformat >> 8) & 0xFF,
               (fmtdesc.pixelformat >> 16) & 0xFF, (fmtdesc.pixelformat >> 24) & 0xFF,
               fmtdesc.description);
        int ret = enum_frame_sizes(cam_fd, fmtdesc.pixelformat); // 列举该格式下的帧大小
        if (ret != 0)
            printf("  Unable to enumerate frame sizes.\n");
    }


//    format.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE; //这里必须填这个
//    format.fmt.pix.width       = 1280;   //用户希望设置的宽
//    format.fmt.pix.height      = 720;    //用户希望设置的高
//    format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;//选择格式：V4L2_PIX_FMT_YUYV或V4L2_PIX_FMT_MJPEG
//    format.fmt.pix.field       = V4L2_FIELD_INTERLACED;
    int ret = ioctl(cam_fd, VIDIOC_G_FMT, &format);

    if (ret < 0) {
        printf("VIDIOC_G_FMT failed (%d)\n", ret);
        return;
    } else {
        printf("type=%d width=%d height=%d pixelformat='%c%c%c%c',filed=%d",
               format.type, format.fmt.pix.width, format.fmt.pix.height,
               format.fmt.pix.pixelformat & 0xFF, (format.fmt.pix.pixelformat >> 8) & 0xFF,
               (format.fmt.pix.pixelformat >> 16) & 0xFF, (format.fmt.pix.pixelformat >> 24) & 0xFF,
               format.fmt.pix.field);
    }


}

#endif
