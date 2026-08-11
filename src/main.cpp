#include <cstdio>

#include <ros/ros.h>

#include "deep_bridge/CmdVelBridge.h"

int main(int argc, char** argv) {
    // 同 unitree_bridge/src/main.cpp：stdout 重定向到文件（tools/deep_bridge.sh 就是这么
    // 启动的）时会从行缓冲变成全缓冲，进程被杀时缓冲区里的 INFO 日志会直接丢失，这里强制
    // 行缓冲，保证日志实时落盘。
    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    ros::init(argc, argv, "deep_bridge_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try {
        deep_bridge::CmdVelBridge bridge(nh, pnh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("[deep_bridge] fatal error: %s", e.what());
        return 1;
    }
    return 0;
}
