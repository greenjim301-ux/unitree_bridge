#include "unitree_bridge/CmdVelBridge.h"

#include <algorithm>

namespace unitree_bridge {

namespace {
double Clamp(double v, double limit) { return std::max(-limit, std::min(limit, v)); }
}  // namespace

CmdVelBridge::CmdVelBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    std::string cmd_vel_topic = "cmd_vel";
    double sport_client_timeout_sec = 10.0;
    pnh.param("cmd_vel_topic", cmd_vel_topic, cmd_vel_topic);
    pnh.param("control_rate_hz", control_rate_hz_, control_rate_hz_);
    pnh.param("cmd_timeout_sec", cmd_timeout_sec_, cmd_timeout_sec_);
    pnh.param("max_vx", max_vx_, max_vx_);
    pnh.param("max_vy", max_vy_, max_vy_);
    pnh.param("max_vyaw", max_vyaw_, max_vyaw_);
    pnh.param("sport_client_timeout_sec", sport_client_timeout_sec, sport_client_timeout_sec);
    pnh.param("auto_stand_on_start", auto_stand_on_start_, auto_stand_on_start_);
    pnh.param("stand_settle_sec", stand_settle_sec_, stand_settle_sec_);
    pnh.param("disable_obstacle_avoid_on_start", disable_obstacle_avoid_on_start_, disable_obstacle_avoid_on_start_);
    pnh.param("classic_walk_on_start", classic_walk_on_start_, classic_walk_on_start_);
    pnh.param("mode_settle_sec", mode_settle_sec_, mode_settle_sec_);

    // ChannelFactory::Instance()->Init(...) 已经在 main() 里调用过，这里只初始化 client 本身
    sport_client_.SetTimeout(static_cast<float>(sport_client_timeout_sec));
    sport_client_.Init();
    obstacles_avoid_client_.SetTimeout(static_cast<float>(sport_client_timeout_sec));
    obstacles_avoid_client_.Init();

    if (auto_stand_on_start_) {
        autoStandOnStart();
    }

    // 放在站立之后：步态切换需要机器人已经处于正常站立状态
    applyInitialMotionMode();

    // 站立完成之后才创建订阅者/定时器，站立过程中不会有 cmd_vel 被处理
    cmd_vel_sub_ = nh.subscribe(cmd_vel_topic, 1, &CmdVelBridge::cmdVelCallback, this);
    control_timer_ =
        nh.createTimer(ros::Duration(1.0 / control_rate_hz_), &CmdVelBridge::controlTimerCallback, this);

    ROS_INFO_STREAM("[unitree_bridge] cmd_vel_topic=" << cmd_vel_topic << " control_rate_hz=" << control_rate_hz_
                                                        << " cmd_timeout_sec=" << cmd_timeout_sec_
                                                        << " max_vx=" << max_vx_ << " max_vy=" << max_vy_
                                                        << " max_vyaw=" << max_vyaw_
                                                        << " auto_stand_on_start=" << auto_stand_on_start_
                                                        << " disable_obstacle_avoid_on_start=" << disable_obstacle_avoid_on_start_
                                                        << " classic_walk_on_start=" << classic_walk_on_start_);
}

void CmdVelBridge::applyInitialMotionMode() {
    // 先关避障再切步态：避障模块开着的时候会接管运动指令，切步态可能被它拦下
    if (disable_obstacle_avoid_on_start_) {
        ROS_INFO("[unitree_bridge] disabling built-in obstacle avoidance ...");
        const int32_t ret = obstacles_avoid_client_.SwitchSet(false);
        if (ret != 0) {
            ROS_ERROR("[unitree_bridge] ObstaclesAvoid SwitchSet(false) failed, error code=%d. "
                      "机身自带避障可能仍然开着，会和 SCAN-Planner 抢控制权，请手动用 App 关闭后再跑导航。",
                      ret);
        } else {
            // 回读确认：SwitchSet 返回 0 只代表 RPC 成功，以实际状态为准
            bool enabled = true;
            const int32_t get_ret = obstacles_avoid_client_.SwitchGet(enabled);
            if (get_ret != 0) {
                ROS_WARN("[unitree_bridge] ObstaclesAvoid SwitchGet() failed, error code=%d, 无法确认避障是否已关闭",
                         get_ret);
            } else if (enabled) {
                ROS_ERROR("[unitree_bridge] 避障开关回读仍为 ON，关闭没有生效，请检查机器人端避障服务状态");
            } else {
                ROS_INFO("[unitree_bridge] built-in obstacle avoidance is OFF");
            }
        }
    }

    if (classic_walk_on_start_) {
        ROS_INFO("[unitree_bridge] switching to classic walk gait ...");
        const int32_t ret = sport_client_.ClassicWalk(true);
        if (ret != 0) {
            ROS_ERROR("[unitree_bridge] ClassicWalk(true) failed, error code=%d, 机器人可能仍处于 AI 步态", ret);
        } else {
            ROS_INFO("[unitree_bridge] classic walk gait enabled");
        }
    }

    if ((disable_obstacle_avoid_on_start_ || classic_walk_on_start_) && mode_settle_sec_ > 0.0) {
        ros::Duration(mode_settle_sec_).sleep();
    }
}

void CmdVelBridge::autoStandOnStart() {
    ROS_WARN("[unitree_bridge] auto_stand_on_start=true: 确认机器人已放置在安全、周围无障碍的地面上");
    ROS_INFO("[unitree_bridge] calling RecoveryStand() ...");
    const int32_t ret = sport_client_.RecoveryStand();
    if (ret != 0) {
        ROS_ERROR("[unitree_bridge] RecoveryStand() failed, error code=%d. 请确认 network_interface "
                   "是否正确、机器人运动控制服务(sport_mode)是否已启动，再手动重启本节点。",
                   ret);
        return;
    }
    ROS_INFO_STREAM("[unitree_bridge] RecoveryStand() issued, waiting " << stand_settle_sec_
                                                                         << "s for it to physically settle...");
    ros::Duration(stand_settle_sec_).sleep();
    ROS_INFO("[unitree_bridge] stand settle wait done, ready to accept cmd_vel");
}

CmdVelBridge::~CmdVelBridge() {
    // 节点退出前主动刹停一次，不完全依赖机器人自己的看门狗
    sport_client_.StopMove();
}

void CmdVelBridge::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    vx_ = Clamp(msg->linear.x, max_vx_);
    vy_ = Clamp(msg->linear.y, max_vy_);
    vyaw_ = Clamp(msg->angular.z, max_vyaw_);
    last_cmd_time_ = ros::Time::now();
    have_cmd_ = true;
}

void CmdVelBridge::controlTimerCallback(const ros::TimerEvent&) {
    double vx = 0.0;
    double vy = 0.0;
    double vyaw = 0.0;
    bool timed_out = true;
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        vx = vx_;
        vy = vy_;
        vyaw = vyaw_;
        timed_out = !have_cmd_ || (ros::Time::now() - last_cmd_time_).toSec() > cmd_timeout_sec_;
    }

    if (timed_out) {
        sport_client_.StopMove();
    } else {
        sport_client_.Move(static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(vyaw));
    }
}

}  // namespace unitree_bridge
