#include <cstdio>

#include <ros/ros.h>
#include <unitree/robot/channel/channel_factory.hpp>

#include "unitree_bridge/CmdVelBridge.h"

int main(int argc, char** argv) {
    // ROS_INFO/ROS_DEBUG go to stdout, ROS_WARN/ERROR/FATAL go to stderr. glibc
    // line-buffers stdout when it's a tty but switches to full block buffering
    // (~4KB) the moment it's redirected to a file or pipe, which is exactly
    // what tools/unitree_bridge.sh does. The effect: while the node is
    // running, the log file only shows WARN+ (stderr is always unbuffered)
    // and every INFO line sits invisible in the buffer until the process
    // exits cleanly or the buffer happens to fill up — killed abruptly
    // (SIGKILL, a crash) and the buffered INFO lines are lost outright.
    // Force line buffering unconditionally so INFO shows up promptly
    // whether stdout is a tty or a redirected file.
    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    ros::init(argc, argv, "unitree_bridge_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 进程内只能调用一次，必须在任何 SportClient/Publisher/Subscriber 构造之前完成
    std::string network_interface = "eth0";
    pnh.param("network_interface", network_interface, network_interface);
    ROS_INFO_STREAM("[unitree_bridge] ChannelFactory Init on interface: " << network_interface);
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

    unitree_bridge::CmdVelBridge bridge(nh, pnh);
    ros::spin();
    return 0;
}
