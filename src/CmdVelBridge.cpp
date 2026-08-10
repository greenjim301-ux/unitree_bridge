#include "unitree_bridge/CmdVelBridge.h"

#include <algorithm>
#include <unordered_map>

namespace unitree_bridge {

namespace {
double Clamp(double v, double limit) { return std::max(-limit, std::min(limit, v)); }

// rt/sportmodestate 的 error_code 字段回报的运动状态机取值，来自 Unitree
// 《运动服务接口 V2.0》文档。注意这套编号跟 sport_api.hpp 里的 API ID
// 不是一回事（比如 ClassicWalk 的 API ID 是 2049，而经典模式号是 2010）。
// 打印用的名字是英文翻译，行尾注释保留文档原文的中文名，方便对照。
const char* SportModeName(uint32_t code) {
    static const std::unordered_map<uint32_t, const char*> kNames = {
        {100, "Agile"},                                    // 灵动
        {1001, "Damping"},                                 // 阻尼
        {1002, "Standing Lock"},                            // 站立锁定
        {1004, "Squat"},                                    // 蹲下
        {2006, "Squat"},                                    // 蹲下
        {1006, "Greeting/Stretch/Dance/NewYear/Heart/Happy"}, // 打招呼/伸懒腰/舞蹈/拜年/比心/开心
        {1007, "Sit"},                                      // 坐下
        {1008, "Front Jump"},                                // 前跳
        {1009, "Pounce"},                                    // 扑人
        {1013, "Balance Stand"},                             // 平衡站立
        {1015, "Normal Walk"},                               // 常规行走
        {1016, "Normal Run"},                                // 常规跑步
        {1017, "Normal Endurance"},                          // 常规续航
        {1091, "Pose"},                                      // 摆姿势
        {2007, "Dodge"},                                     // 闪避
        {2008, "Bound Run"},                                 // 并腿跑
        {2009, "Jump Run"},                                  // 跳跃跑
        {2010, "Classic"},                                   // 经典
        {2011, "Handstand"},                                 // 倒立
        {2012, "Front Flip"},                                // 前空翻
        {2013, "Back Flip"},                                 // 后空翻
        {2014, "Left Flip"},                                 // 左空翻
        {2016, "Cross Step"},                                // 交叉步
        {2017, "Upright"},                                   // 直立
        {2019, "Traction"},                                  // 牵引
    };
    const auto it = kNames.find(code);
    return it == kNames.end() ? "Unknown" : it->second;
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
    pnh.param("classic_walk_on_start", classic_walk_on_start_, classic_walk_on_start_);
    pnh.param("mode_settle_sec", mode_settle_sec_, mode_settle_sec_);
    pnh.param("sport_state_topic", sport_state_topic_, sport_state_topic_);
    pnh.param("state_wait_timeout_sec", state_wait_timeout_sec_, state_wait_timeout_sec_);
    pnh.param("classic_walk_retry", classic_walk_retry_, classic_walk_retry_);
    pnh.param("expected_mode", expected_mode_, expected_mode_);

    // ChannelFactory::Instance()->Init(...) 已经在 main() 里调用过，这里只初始化 client 本身
    sport_client_.SetTimeout(static_cast<float>(sport_client_timeout_sec));
    sport_client_.Init();

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
                                                        << " classic_walk_on_start=" << classic_walk_on_start_);
}

void CmdVelBridge::applyInitialMotionMode() {
    if (classic_walk_on_start_) {
        confirmClassicWalk();
    }

    logFinalMotionMode();
}

void CmdVelBridge::logFinalMotionMode() {
    unitree_go::msg::dds_::SportModeState_ state;
    if (!waitForFreshSportState(state_wait_timeout_sec_, state)) {
        ROS_WARN("[unitree_bridge] ===== Final motion mode: timed out (%.1fs) reading %s, could not confirm =====",
                 state_wait_timeout_sec_, sport_state_topic_.c_str());
        return;
    }

    const uint32_t mode = state.error_code();
    const char* name = SportModeName(mode);

    if (expected_mode_ >= 0 && mode != static_cast<uint32_t>(expected_mode_)) {
        ROS_WARN("[unitree_bridge] ===== Final motion mode: %u(%s), does not match expected %d(%s) =====", mode,
                 name, expected_mode_, SportModeName(static_cast<uint32_t>(expected_mode_)));
        return;
    }

    ROS_INFO("[unitree_bridge] ===== Final motion mode: %u(%s) =====", mode, name);
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
            ROS_WARN("[unitree_bridge] Timed out (%.1fs) waiting for %s state, unable to confirm the gait. "
                     "Check network_interface and whether the robot's sport_mode service is running.",
                     state_wait_timeout_sec_, sport_state_topic_.c_str());
            continue;
        }

        // error_code 在运动服务接口里被复用为"当前模式"，不是字面的错误码；
        // mode/gait_type 一并打印，纯诊断用
        const uint32_t current_mode = state.error_code();
        ROS_INFO("[unitree_bridge] sport state readback: error_code(current mode)=%u(%s) fsm_mode=%d gait_type=%d",
                 current_mode, SportModeName(current_mode), static_cast<int>(state.mode()),
                 static_cast<int>(state.gait_type()));

        if (expected_mode_ < 0) {
            // 显式关掉校验：只把实测值打出来
            ROS_WARN("[unitree_bridge] expected_mode=-1, readback only, no verification");
            return true;
        }

        if (current_mode == static_cast<uint32_t>(expected_mode_)) {
            ROS_INFO("[unitree_bridge] classic walk confirmed: %u(%s)", current_mode, SportModeName(current_mode));
            return true;
        }

        ROS_WARN("[unitree_bridge] Mode readback mismatch: current %u(%s), expected %d(%s)", current_mode,
                 SportModeName(current_mode), expected_mode_, SportModeName(static_cast<uint32_t>(expected_mode_)));
    }

    ROS_ERROR("[unitree_bridge] Could not confirm the classic walk switch; the robot may still be in another mode. "
              "Speed tracking and lateral motion will differ from what was tuned for. Stop and check before "
              "running navigation.");
    return false;
}

void CmdVelBridge::autoStandOnStart() {
    ROS_WARN("[unitree_bridge] auto_stand_on_start=true: make sure the robot is on safe, obstacle-free ground "
             "before it stands up");
    ROS_INFO("[unitree_bridge] calling RecoveryStand() ...");
    const int32_t ret = sport_client_.RecoveryStand();
    if (ret != 0) {
        ROS_ERROR("[unitree_bridge] RecoveryStand() failed, error code=%d. Check that network_interface is "
                  "correct and the robot's sport_mode service is running, then restart this node manually.",
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
