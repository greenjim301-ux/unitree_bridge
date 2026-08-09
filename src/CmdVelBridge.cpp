#include "unitree_bridge/CmdVelBridge.h"

#include <algorithm>
#include <unordered_map>

namespace unitree_bridge {

namespace {
double Clamp(double v, double limit) { return std::max(-limit, std::min(limit, v)); }

// rt/sportmodestate 的 error_code 字段回报的运动状态机取值，来自 Unitree
// 《运动服务接口 V2.0》文档。注意这套编号跟 sport_api.hpp 里的 API ID
// 不是一回事（比如 ClassicWalk 的 API ID 是 2049，而经典模式号是 2010）。
const char* SportModeName(uint32_t code) {
    static const std::unordered_map<uint32_t, const char*> kNames = {
        {100, "灵动"},        {1001, "阻尼"},     {1002, "站立锁定"},
        {1004, "蹲下"},       {2006, "蹲下"},     {1006, "打招呼/伸懒腰/舞蹈/拜年/比心/开心"},
        {1007, "坐下"},       {1008, "前跳"},     {1009, "扑人"},
        {1013, "平衡站立"},   {1015, "常规行走"}, {1016, "常规跑步"},
        {1017, "常规续航"},   {1091, "摆姿势"},   {2007, "闪避"},
        {2008, "并腿跑"},     {2009, "跳跃跑"},   {2010, "经典"},
        {2011, "倒立"},       {2012, "前空翻"},   {2013, "后空翻"},
        {2014, "左空翻"},     {2016, "交叉步"},   {2017, "直立"},
        {2019, "牵引"},
    };
    const auto it = kNames.find(code);
    return it == kNames.end() ? "未知" : it->second;
}
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
    pnh.param("sport_state_topic", sport_state_topic_, sport_state_topic_);
    pnh.param("state_wait_timeout_sec", state_wait_timeout_sec_, state_wait_timeout_sec_);
    pnh.param("classic_walk_retry", classic_walk_retry_, classic_walk_retry_);
    pnh.param("expected_mode", expected_mode_, expected_mode_);

    // ChannelFactory::Instance()->Init(...) 已经在 main() 里调用过，这里只初始化 client 本身
    sport_client_.SetTimeout(static_cast<float>(sport_client_timeout_sec));
    sport_client_.Init();
    obstacles_avoid_client_.SetTimeout(static_cast<float>(sport_client_timeout_sec));
    obstacles_avoid_client_.Init();

    // 状态回读通道，要在 applyInitialMotionMode() 之前建好
    sport_state_sub_ =
        std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(sport_state_topic_);
    sport_state_sub_->InitChannel([this](const void* msg) { sportStateHandler(msg); }, 1);

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
        confirmClassicWalk();
    }

    // 走了 confirmClassicWalk() 的话里面已经等过 mode_settle_sec_ 了，不再重复等
    if (disable_obstacle_avoid_on_start_ && !classic_walk_on_start_ && mode_settle_sec_ > 0.0) {
        ros::Duration(mode_settle_sec_).sleep();
    }

    logFinalMotionMode();
}

void CmdVelBridge::logFinalMotionMode() {
    unitree_go::msg::dds_::SportModeState_ state;
    if (!waitForFreshSportState(state_wait_timeout_sec_, state)) {
        ROS_WARN("[unitree_bridge] ===== 最终运动模式: 读取 %s 超时(%.1fs)，未能确认 =====",
                 sport_state_topic_.c_str(), state_wait_timeout_sec_);
        return;
    }

    const uint32_t mode = state.error_code();
    const char* name = SportModeName(mode);

    if (expected_mode_ >= 0 && mode != static_cast<uint32_t>(expected_mode_)) {
        ROS_WARN("[unitree_bridge] ===== 最终运动模式: %u(%s)，与期望的 %d(%s) 不符 =====", mode, name,
                 expected_mode_, SportModeName(static_cast<uint32_t>(expected_mode_)));
        return;
    }

    ROS_INFO("[unitree_bridge] ===== 最终运动模式: %u(%s) =====", mode, name);
}

void CmdVelBridge::sportStateHandler(const void* msg) {
    if (msg == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_state_ = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(msg);
    have_state_ = true;
}

bool CmdVelBridge::waitForFreshSportState(double timeout_sec, unitree_go::msg::dds_::SportModeState_& out) {
    {
        // 先丢掉旧帧，保证等到的是切换之后产生的状态
        std::lock_guard<std::mutex> lock(state_mutex_);
        have_state_ = false;
    }

    const ros::Time deadline = ros::Time::now() + ros::Duration(timeout_sec);
    while (ros::ok() && ros::Time::now() < deadline) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (have_state_) {
                out = last_state_;
                return true;
            }
        }
        ros::Duration(0.02).sleep();
    }
    return false;
}

bool CmdVelBridge::confirmClassicWalk() {
    // 总共发 1 + classic_walk_retry_ 次，每次发完都回读一帧状态确认
    for (int attempt = 0; attempt <= classic_walk_retry_; ++attempt) {
        ROS_INFO("[unitree_bridge] switching to classic walk gait (attempt %d/%d) ...", attempt + 1,
                 classic_walk_retry_ + 1);
        const int32_t ret = sport_client_.ClassicWalk(true);
        if (ret != 0) {
            ROS_WARN("[unitree_bridge] ClassicWalk(true) failed, error code=%d", ret);
            continue;
        }

        // 先给切换动作留出执行时间再回读：状态帧是 ~50Hz 发的，切换刚发出去
        // 就读下一帧的话读到的还是切换前的模式
        if (mode_settle_sec_ > 0.0) {
            ros::Duration(mode_settle_sec_).sleep();
        }

        unitree_go::msg::dds_::SportModeState_ state;
        if (!waitForFreshSportState(state_wait_timeout_sec_, state)) {
            ROS_WARN("[unitree_bridge] 等待 %s 状态超时(%.1fs)，无法确认步态。检查 network_interface "
                     "是否正确、机器人运动控制服务是否在跑。",
                     sport_state_topic_.c_str(), state_wait_timeout_sec_);
            continue;
        }

        // error_code 在运动服务接口里被复用为"当前模式"，不是字面的错误码；
        // mode/gait_type 一并打印，纯诊断用
        const uint32_t current_mode = state.error_code();
        ROS_INFO("[unitree_bridge] sport state readback: error_code(当前模式)=%u(%s) fsm_mode=%d gait_type=%d",
                 current_mode, SportModeName(current_mode), static_cast<int>(state.mode()),
                 static_cast<int>(state.gait_type()));

        if (expected_mode_ < 0) {
            // 显式关掉校验：只把实测值打出来
            ROS_WARN("[unitree_bridge] expected_mode=-1，只回读不校验");
            return true;
        }

        if (current_mode == static_cast<uint32_t>(expected_mode_)) {
            ROS_INFO("[unitree_bridge] classic walk confirmed: %u(%s)", current_mode, SportModeName(current_mode));
            return true;
        }

        ROS_WARN("[unitree_bridge] 模式回读不符：当前 %u(%s)，期望 %d(%s)", current_mode, SportModeName(current_mode),
                 expected_mode_, SportModeName(static_cast<uint32_t>(expected_mode_)));
    }

    ROS_ERROR("[unitree_bridge] 经典步态切换未能确认，机器人可能仍处于其他模式。速度跟随和侧移特性会和"
              "调参时不一致，建议停下检查后再跑导航。");
    return false;
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
