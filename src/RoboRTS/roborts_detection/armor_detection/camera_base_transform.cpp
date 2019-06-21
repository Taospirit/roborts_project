#include "armor_detection_node.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "armor_detection_node");

    ros::NodeHandle nh("~");
    std::string node_name;
    std::string robot_base_frame_name;
    std::string gimbal_frame_name;
    std::string camera_frame_name;
    node_name = ros::this_node::getName();
    nh.param<std::string>("robot_base_frame_name", robot_base_frame_name, "base_link");
    nh.param<std::string>("gimbal_frame_name", gimbal_frame_name, "gimbal");
    nh.param<std::string>("camera_frame_name", camera_frame_name, "industry_camera");

    tf::TransformListener listener;
    tf::StampedTransform transform;
    geometry_msgs::Quaternion quat_msg;
    tf2::Quaternion Quaternion;
    tf::Quaternion q_;
    tf::Transform tmp;
    tf::Vector3 t_;
    double g_roll, g_pitch, g_yaw;
    tf::TransformBroadcaster br;

    roborts_detection::ArmorDetectionAlgorithms armor_detection_param;
    std::string
            file_name = ros::package::getPath("roborts_detection") + "/armor_detection/config/armor_detection.prototxt";
    bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &armor_detection_param);
    if (!read_state) {
        ROS_ERROR("Cannot open %s", file_name.c_str());
    }
    float x = armor_detection_param.camera_gimbal_transform().offset_x();
    float y = armor_detection_param.camera_gimbal_transform().offset_y();
    float z = armor_detection_param.camera_gimbal_transform().offset_z();
    float pitch_offset = armor_detection_param.camera_gimbal_transform().offset_pitch();
    float yaw_offset = armor_detection_param.camera_gimbal_transform().offset_yaw();

    ros::Rate rate(100);
    while (ros::ok()) {
        try {
            listener.lookupTransform(robot_base_frame_name, gimbal_frame_name, ros::Time(0), transform);
            auto q = transform.getRotation();
            auto t = transform.getOrigin();
            tf::Matrix3x3(q).getRPY(g_roll, g_pitch, g_yaw);
            auto camera_pitch = 0;
            auto camera_yaw = g_yaw;
            auto camera_x = t.x() + x;
            auto camera_y = t.y() + y;
            auto camera_z = t.z() + z;
            Quaternion.setRPY(0, 0, camera_yaw);
            Quaternion.normalize();
            quat_msg = tf2::toMsg(Quaternion);
            tf::quaternionMsgToTF(quat_msg, q_);
            t_.setX(camera_x);
            t_.setY(camera_y);
            t_.setZ(camera_z);
            tmp.setOrigin(t_);
            tmp.setRotation(q_);
            br.sendTransform(tf::StampedTransform(tmp, ros::Time::now(), robot_base_frame_name, camera_frame_name));
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
        }
        rate.sleep();
    }
    ros::spin();
    return 0;
}
