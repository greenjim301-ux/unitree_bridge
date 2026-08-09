/*
 * unitree_bridge_node
 *
 * 订阅 cmd_vel（geometry_msgs::Twist），按固定频率通过 unitree_sdk2 的
 * go2::SportClient::Move(vx, vy, vyaw) 下发给 Go2。
 * 独立于 hand-lio：这是控制下发链路，跟 hand-lio 的感知/定位链路职责不同，
 * 不共用节点、不共用生命周期。
 *
 * ChannelFactory::Instance()->Init(...) 必须在构造 CmdVelBridge 之前、
 * main() 里先调用一次（进程内只能 Init 一次），所以这里不放在类里做。
 */
#pragma once

#include <mutex>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

namespace unitree_bridge {

class CmdVelBridge {
public:
    CmdVelBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~CmdVelBridge();

private:
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void controlTimerCallback(const ros::TimerEvent&);

    // 开机自动站立：RecoveryStand() 对起始姿态没有要求（趴着/蹲着都能站起来，
    // 不像 StandUp() 那样只适合已经处于蹲姿的情况），是构造函数里阻塞调用的，
    // 站稳之前不会创建 cmd_vel 订阅者，保证站立完成前不会响应任何速度指令。
    void autoStandOnStart();

    // 站立完成后设置机器人的初始运动模式：关闭机身自带的避障，切到经典步态。
    // 自带避障必须关掉——它会拦截/改写我们下发的 Move()，跟 SCAN-Planner 的
    // 局部避障抢控制权，表现为狗"不听话地绕路"或原地卡住。
    // 经典步态相比 AI 步态速度跟随更线性、侧移更稳，适合闭环速度控制。
    void applyInitialMotionMode();

    ros::Subscriber cmd_vel_sub_;
    ros::Timer control_timer_;
    unitree::robot::go2::SportClient sport_client_;
    unitree::robot::go2::ObstaclesAvoidClient obstacles_avoid_client_;

    std::mutex cmd_mutex_;
    double vx_ = 0.0;
    double vy_ = 0.0;
    double vyaw_ = 0.0;
    ros::Time last_cmd_time_;
    bool have_cmd_ = false;

    // ---- 参数 ----
    double control_rate_hz_ = 20.0;   // 控制循环频率：每个周期都重发 Move()/StopMove()，不是收到一次发一次
    double cmd_timeout_sec_ = 0.5;    // 超过这么久没收到新 cmd_vel 就调用 StopMove()（安全看门狗）
    double max_vx_ = 0.6;             // [m/s] 限幅，先给保守默认值，按实际测试需要调
    double max_vy_ = 0.6;             // [m/s]
    double max_vyaw_ = 0.6;           // [rad/s]
    bool auto_stand_on_start_ = true; // 开机是否自动 RecoveryStand()
    double stand_settle_sec_ = 3.0;   // 发完 RecoveryStand() 后阻塞等待这么久，给站立动作留出物理执行时间

    bool disable_obstacle_avoid_on_start_ = true; // 开机是否关闭机身自带避障（ObstaclesAvoidClient::SwitchSet(false)）
    bool classic_walk_on_start_ = true;           // 开机是否切换到经典步态（SportClient::ClassicWalk(true)）
    double mode_settle_sec_ = 1.0;                // 切模式/步态后阻塞等待这么久，给切换动作留出执行时间
};

}  // namespace unitree_bridge
