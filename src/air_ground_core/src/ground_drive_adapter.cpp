// ground_drive_adapter —— kinematic_sim 地面执行后端（M0 接口契约 v1.1，M5 增控制权核查）。
//
// 与 scripts/ground_drive_adapter.py 语义逐条对齐：
// - 任务运行期唯一写本方车（agc_car）位姿的节点；绝不写无人机或目标位姿。
//   任务非运行期（IDLE/终态）不写位姿（由 scenario_reset 独占写入）。
// - 订阅 /agc/ground/cmd_vel，20 Hz 运动学积分（先转向再沿新朝向平移）：
//     yaw += wz*dt; x += vx*cos(yaw)*dt; y += vx*sin(yaw)*dt; z = drive.start_z
//   并用 set_model_state 写 agc_car（reference_frame=world）。
// - 命令 watchdog：命令年龄（仿真时间）> cmd_max_age_s 时本拍零速，计数写
//   /agc/drive/watchdog_count（首个命令到达前不计数，仅记一条日志）。
// - 服务失败计数写 /agc/drive/service_failures；连续失败 >=5 -> backend_ok=false，恢复置 true。
// - 每拍写 drive_log.jsonl。
// - M5 写入权核查：本节点是任务运行期唯一允许写本方车位姿的节点。除合规复位
//   （scenario_reset，会自增 /agc/reset/epoch）外，若 Gazebo 实际车位姿出现本节点
//   无法产生的突跳，判定存在外部写入者（set_model_state 写入权冲突），置
//   /agc/drive/pose_authority_ok=false、自增 /agc/drive/external_write_count，
//   并写 pose_authority.jsonl。复位换序时把积分器重同步到实际位姿（合法写方）。
#include <cmath>
#include <algorithm>
#include <string>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/TwistStamped.h>

#include <air_ground_core/AgcTaskState.h>

#include "agc/agc_common.hpp"

namespace {

// Gazebo 命名空间（默认 /gazebo），去尾斜杠并补前导斜杠；空串保持空。
std::string gzNs() {
  std::string ns = agc::pgetStr("/agc/gazebo/ns", "/gazebo");
  if (ns.empty()) return "";
  while (!ns.empty() && ns.back() == '/') ns.pop_back();
  if (!ns.empty() && ns.front() != '/') ns = "/" + ns;
  return ns;
}

}  // namespace

class GroundDriveAdapter {
 public:
  GroundDriveAdapter() : nh_() {
    hz_ = agc::cfgD("rates/drive_hz", 20.0);
    cmd_max_age_ = agc::cfgD("freshness/cmd_max_age_s", 0.2);
    model_ = agc::cfgS("gazebo/car_model", "agc_car");
    start_x_ = agc::cfgD("drive/start_x", 0.0);
    start_y_ = agc::cfgD("drive/start_y", 0.0);
    start_z_ = agc::cfgD("drive/start_z", 0.11);
    start_yaw_ = agc::cfgD("drive/start_yaw", 0.0);
    // M5：写入权核查阈值
    ext_jump_m_ = agc::cfgD("drive/external_write_jump_m", 0.30);
    ext_consec_need_ = agc::cfgI("drive/external_write_consecutive", 3);
    resync_holdoff_s_ = agc::cfgD("drive/resync_holdoff_s", 1.0);

    x_ = start_x_;
    y_ = start_y_;
    z_ = start_z_;
    yaw_ = start_yaw_;

    const std::string ns = gzNs();
    cmd_sub_ = nh_.subscribe("/agc/ground/cmd_vel", 1, &GroundDriveAdapter::cmdCb, this);
    models_sub_ = nh_.subscribe(ns + "/model_states", 1, &GroundDriveAdapter::modelsCb, this);
    // 运行期判定：非 IDLE 非终态才积分/写位姿（与 air_control_bridge 一致）
    task_sub_ = nh_.subscribe("/agc/task/state", 10, &GroundDriveAdapter::taskStateCb, this);

    // 启动时探一下 set_model_state；不可用不致命，运行期重试
    if (!agc::serviceReady(ns + "/set_model_state", 3.0)) {
      ROS_WARN("ground_drive_adapter: set_model_state 暂不可用，运行中重试");
    }

    ros::param::set("/agc/drive/backend_ok", true);
    // M5：写入权初值（本节点即唯一写入者）
    ros::param::set("/agc/drive/pose_authority_ok", true);
    ros::param::set("/agc/drive/external_write_count", 0);
    timer_ = nh_.createTimer(ros::Duration(1.0 / hz_), &GroundDriveAdapter::tick, this);
    ROS_INFO("ground_drive_adapter 就绪：backend=kinematic_sim start=(%.2f,%.2f,%.2f) yaw=%.2f",
             start_x_, start_y_, start_z_, start_yaw_);
  }

 private:
  // ------------------------------------------------------------ 回调
  void cmdCb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    cmd_vx_ = msg->twist.linear.x;
    cmd_wz_ = msg->twist.angular.z;
    // 以到达的仿真时间戳为命令年龄起点；零时间戳退回当前仿真时刻
    cmd_stamp_ = msg->header.stamp.isZero() ? agc::simNow() : msg->header.stamp;
    cmd_received_ = true;
  }

  void modelsCb(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == model_) {
        const geometry_msgs::Pose& p = msg->pose[i];
        actual_x_ = p.position.x;
        actual_y_ = p.position.y;
        actual_yaw_ = agc::yawFromQuat(p.orientation);
        has_actual_ = true;
        return;
      }
    }
  }

  // 镜像 air_control_bridge：任务运行期 = 非终态且非 IDLE
  void taskStateCb(const air_ground_core::AgcTaskState::ConstPtr& msg) {
    const int state = static_cast<int>(msg->state);
    task_running_ = (!agc::isTerminal(state) && state != agc::ST_IDLE);
  }

  // ------------------------------------------------------------ 写模型
  bool write(std::string* err) {
    const std::string name = gzNs() + "/set_model_state";
    gazebo_msgs::SetModelState srv;
    gazebo_msgs::ModelState& ms = srv.request.model_state;
    ms.model_name = model_;
    ms.pose.position.x = x_;
    ms.pose.position.y = y_;
    ms.pose.position.z = z_;
    const std::array<double, 4> q = agc::quatFromYaw(yaw_);
    ms.pose.orientation.x = q[0];
    ms.pose.orientation.y = q[1];
    ms.pose.orientation.z = q[2];
    ms.pose.orientation.w = q[3];
    ms.reference_frame = agc::FRAME_WORLD;
    const bool called = ros::service::call(name, srv);
    if (!called) {
      if (err) *err = "set_model_state 调用失败";
      return false;
    }
    if (!srv.response.success) {
      if (err) *err = "set_model_state 返回失败: " + srv.response.status_message;
      return false;
    }
    return true;
  }

  // 更新连续失败计数与 backend_ok
  void reportService(bool ok) {
    if (ok) {
      consec_fail_ = 0;
      if (!agc::pgetB("/agc/drive/backend_ok", true)) {
        ros::param::set("/agc/drive/backend_ok", true);
        ROS_INFO("ground_drive_adapter: 服务恢复，backend_ok=true");
      }
      return;
    }
    consec_fail_ += 1;
    last_fail_wall_ = agc::wallNow();
    ros::param::set("/agc/drive/service_failures",
                    agc::pgetI("/agc/drive/service_failures", 0) + 1);
    ROS_WARN_THROTTLE(2.0, "ground_drive_adapter: set_model_state 失败（连续 %d）", consec_fail_);
    const bool stalled = last_fail_wall_ > 0.0 && consec_fail_ >= 5;
    if (stalled && agc::pgetB("/agc/drive/backend_ok", true)) {
      ros::param::set("/agc/drive/backend_ok", false);
      ROS_ERROR("ground_drive_adapter: 连续服务失败 >=5，backend_ok=false");
    }
  }

  // ------------------------------------------------------------ 写入权核查（M5）
  void setAuthority(bool ok) {
    if (has_authority_ && authority_ok_ == ok) return;
    has_authority_ = true;
    authority_ok_ = ok;
    ros::param::set("/agc/drive/pose_authority_ok", ok);
  }

  // 复位换序（epoch 变化）时把积分器重同步到 Gazebo 实际位姿——这是合法写方
  // （scenario_reset，仅非运行期）；其余时段检测本节点无法产生的实际位姿突跳。
  void checkPoseAuthority(double t, double vx, double dt) {
    const int epoch = agc::pgetI("/agc/reset/epoch", 0);
    const double wall = agc::wallNow();
    if (epoch != reset_epoch_) {
      reset_epoch_ = epoch;
      resync_wall_ = wall;      // 进入重同步保持窗
      has_prev_actual_ = false;  // 不跨复位比较（避免把瞬移误判为外部写入）
      ext_consec_ = 0;
      if (has_actual_) {
        x_ = actual_x_;
        y_ = actual_y_;
        yaw_ = actual_yaw_;
      }
      setAuthority(true);
    }
    if (!has_actual_) return;

    if (wall - resync_wall_ < resync_holdoff_s_) {
      // 保持窗内持续对齐，且不做突跳判定
      x_ = actual_x_;
      y_ = actual_y_;
      yaw_ = actual_yaw_;
    } else if (has_prev_actual_) {
      const double jump = std::hypot(actual_x_ - prev_actual_x_, actual_y_ - prev_actual_y_);
      const double expect = std::fabs(vx) * dt + 0.10;  // 本节点一步位移 + 裕度
      if (jump > std::max(ext_jump_m_, expect + 0.10)) {
        ext_consec_ += 1;
      } else {
        ext_consec_ = 0;
      }
      if (ext_consec_ >= ext_consec_need_) {
        external_write_count_ += 1;
        ros::param::set("/agc/drive/external_write_count", external_write_count_);
        setAuthority(false);
        ext_consec_ = 0;  // 记一次后清零，避免每拍重复计数
        ROS_WARN_THROTTLE(1.0, "ground_drive_adapter: 检测到外部写车位姿 (jump=%.3f m)", jump);
        agc::JsonObj o;
        o.n("t_sim", t).v("event", "EXTERNAL_POSE_WRITE")
            .n("jump_m", jump).n("expected_step_m", expect)
            .n("actual_x", actual_x_).n("actual_y", actual_y_)
            .iv("external_write_count", external_write_count_);
        agc::appendJsonl("pose_authority.jsonl", o);
      }
    }
    prev_actual_x_ = actual_x_;
    prev_actual_y_ = actual_y_;
    has_prev_actual_ = true;
  }

  // ------------------------------------------------------------ 初始化
  bool maybeInit() {
    if (initialized_) return true;
    init_ticks_ += 1;
    if (has_actual_) {
      // 以 Gazebo 实际位姿为积分初值
      x_ = actual_x_;
      y_ = actual_y_;
      yaw_ = actual_yaw_;
      initialized_ = true;
      ROS_INFO("ground_drive_adapter: 积分初值取 Gazebo 实际位姿 (%.2f,%.2f,yaw=%.2f)", x_, y_, yaw_);
      return true;
    }
    // 等 Gazebo 约 2 s，仍无则回退 drive.start_*
    if (init_ticks_ > static_cast<int>(2.0 * hz_)) {
      initialized_ = true;
      ROS_WARN("ground_drive_adapter: 未取得 Gazebo 车体位姿，回退 drive.start_*");
      return true;
    }
    return false;
  }

  // ------------------------------------------------------------ 定时器
  void tick(const ros::TimerEvent&) {
    const double t = agc::simNow().toSec();
    if (t <= 0.0) {
      ROS_WARN_THROTTLE(5.0, "ground_drive_adapter: 仿真时钟未就绪，等待 /clock");
      return;
    }
    if (!maybeInit()) return;

    double dt = last_t_ < 0.0 ? 0.0 : (t - last_t_);
    last_t_ = t;
    if (dt < 0.0 || dt > 0.5) dt = 0.0;  // 时钟停滞或跳变：本拍不推进

    // ---- 任务非运行期：scenario_reset 独占写位姿，本节点不积分、不写 ----
    if (!task_running_) {
      // 仅在非运行期跟踪复位 epoch（合法写方）并简化处理：复位换序即恢复写入权、
      // 清掉跨复位比较记录，避免下个运行期首拍把瞬移误判为外部写入。
      const int epoch = agc::pgetI("/agc/reset/epoch", 0);
      if (epoch != reset_epoch_) {
        reset_epoch_ = epoch;
        setAuthority(true);
        has_prev_actual_ = false;
        ext_consec_ = 0;
      }
      if (has_actual_) {  // 积分器重同步到 Gazebo 实际车体位姿
        x_ = actual_x_;
        y_ = actual_y_;
        yaw_ = actual_yaw_;
      }
      z_ = start_z_;

      // 仍写一行日志，保留复盘连续性
      agc::JsonObj pose;
      pose.n("x", x_).n("y", y_).n("z", z_).n("yaw", yaw_);
      agc::JsonObj o;
      o.n("t_sim", t).n("cmd_vx", 0.0).n("cmd_wz", 0.0).raw("pose", pose.dump())
          .bv("watchdog", false);
      o.bv("pose_authority_ok", authority_ok_).n("pose_dev_m", 0.0)
          .iv("external_write_count", external_write_count_).iv("reset_epoch", reset_epoch_)
          .bv("task_running", false);
      agc::appendJsonl("drive_log.jsonl", o);
      return;
    }

    // 命令 watchdog
    bool watchdog = false;
    double vx = 0.0, wz = 0.0;
    if (!cmd_received_) {
      if (!wait_logged_) {
        wait_logged_ = true;
        ROS_INFO("ground_drive_adapter: 尚未收到命令，等待首个 cmd_vel（不计数）");
      }
    } else {
      const double age = agc::stampAge(cmd_stamp_);
      if (age > cmd_max_age_) {
        watchdog = true;
        ros::param::set("/agc/drive/watchdog_count",
                        agc::pgetI("/agc/drive/watchdog_count", 0) + 1);
        ROS_WARN_THROTTLE(1.0, "ground_drive_adapter: 命令年龄 %.3f > %.2f，零速", age, cmd_max_age_);
      } else {
        vx = cmd_vx_;
        wz = cmd_wz_;
      }
    }

    // 运动学积分（先转向再沿新朝向平移）
    yaw_ = agc::normalizeAngle(yaw_ + wz * dt);
    x_ += vx * std::cos(yaw_) * dt;
    y_ += vx * std::sin(yaw_) * dt;
    z_ = start_z_;

    // M5 写入权核查（含复位换序重同步）
    checkPoseAuthority(t, vx, dt);

    // 与 Gazebo 实际位姿偏差只告警，不强行纠正（避免与复位抢写）
    double pose_dev = 0.0;
    if (has_actual_) {
      pose_dev = std::hypot(x_ - actual_x_, y_ - actual_y_);
      if (pose_dev > 0.5) {
        ROS_WARN_THROTTLE(2.0, "ground_drive_adapter: 积分位姿与 Gazebo 偏差 %.2f m > 0.5 m", pose_dev);
      }
    }

    std::string err;
    reportService(write(&err));

    agc::JsonObj pose;
    pose.n("x", x_).n("y", y_).n("z", z_).n("yaw", yaw_);
    agc::JsonObj o;
    o.n("t_sim", t).n("cmd_vx", vx).n("cmd_wz", wz).raw("pose", pose.dump()).bv("watchdog", watchdog);
    o.bv("pose_authority_ok", authority_ok_).n("pose_dev_m", pose_dev)
        .iv("external_write_count", external_write_count_).iv("reset_epoch", reset_epoch_)
        .bv("task_running", true);
    agc::appendJsonl("drive_log.jsonl", o);
  }

  ros::NodeHandle nh_;
  ros::Subscriber cmd_sub_, models_sub_, task_sub_;
  ros::Timer timer_;

  double hz_ = 20.0, cmd_max_age_ = 0.2;
  std::string model_ = "agc_car";
  double start_x_ = 0.0, start_y_ = 0.0, start_z_ = 0.11, start_yaw_ = 0.0;

  double x_ = 0.0, y_ = 0.0, z_ = 0.11, yaw_ = 0.0;
  bool initialized_ = false;
  int init_ticks_ = 0;
  double last_t_ = -1.0;

  double cmd_vx_ = 0.0, cmd_wz_ = 0.0;
  ros::Time cmd_stamp_;
  bool cmd_received_ = false;
  bool wait_logged_ = false;

  // 任务运行期判定（/agc/task/state）
  bool task_running_ = false;

  int consec_fail_ = 0;
  double last_fail_wall_ = 0.0;
  bool has_actual_ = false;
  double actual_x_ = 0.0, actual_y_ = 0.0, actual_yaw_ = 0.0;

  // M5 写入权核查
  double ext_jump_m_ = 0.30, resync_holdoff_s_ = 1.0;
  int ext_consec_need_ = 3;
  int reset_epoch_ = -1;
  double resync_wall_ = 0.0;
  bool has_prev_actual_ = false;
  double prev_actual_x_ = 0.0, prev_actual_y_ = 0.0;
  int ext_consec_ = 0;
  int external_write_count_ = 0;
  bool authority_ok_ = true;
  bool has_authority_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ground_drive_adapter");
  GroundDriveAdapter node;
  ros::spin();
  return 0;
}