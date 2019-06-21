#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include "../camera_node.h"
#include "video_reader.h"

void SignalHandler(int signal) {
    if (ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()) {
        ros::shutdown();
    }
}

int main(int argc, char **argv) {
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);
    ros::init(argc, argv, "robo_video_test", ros::init_options::NoSigintHandler);

    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();
    ros::waitForShutdown();
    return 0;
}
