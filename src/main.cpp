#include <ros/ros.h>
#include <unitree/robot/channel/channel_factory.hpp>

#include "unitree_bridge/CmdVelBridge.h"

int main(int argc, char** argv) {
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
