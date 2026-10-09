// scenario_reset —— 场景复位服务（M0 接口契约 v1.1）。
//
// 与 scripts/scenario_reset.py 语义逐条对齐：
// - 服务 /agc/scenario/reset（std_srvs/Trigger）：仅当前任务为 IDLE 或终态时允许复位。
// - 复位内容：把 agc_car 写回 scenario car_start，把 intruder_uav 写回 target_start；
//   **绝不写无人机位姿**（无人机位姿所有权属于空中控制链）。
// - 静态障碍不复位（本就固定不动）。
// - 通过自增 /agc/target/reset_epoch 通知 intruder_driver 归零内部相位。
// - 复位失败只属基础设施失败（INFRA_FAILURE），绝不进入任务失败分子。
// - 启动时也做一次初始复位（等 /gazebo/model_states 出现后执行）。
// - 热切换（M0 v1.2）：dynamic_obstacles=true 时轮询 /agc/scenario_file，变化即
//   删除上一场景 spawn 的障碍并按新 scene obstacles 用 spawn_sdf_model 重建，
//   完成后自增 /agc/scenario/epoch；dynamic_obstacles=false 时行为与改动前一致。
#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <std_srvs/Trigger.h>
#include <air_ground_core/AgcTaskState.h>

#include "agc/agc_common.hpp"

namespace {

// 允许复位的任务状态：IDLE 与全部终态
bool allowedResetState(int state) {
  return state == agc::ST_IDLE || state == agc::ST_TRACKING_SUCCESS ||
         state == agc::ST_MISSION_FAILED || state == agc::ST_ABORTED ||
         state == agc::ST_INFRA_FAILURE;
}

// Gazebo 命名空间（默认 /gazebo），去尾斜杠并补前导斜杠
std::string gzNs() {
  std::string ns = agc::pgetStr("/agc/gazebo/ns", "/gazebo");
  if (ns.empty()) return "";
  while (!ns.empty() && ns.back() == '/') ns.pop_back();
  if (!ns.empty() && ns.front() != '/') ns = "/" + ns;
  return ns;
}

std::vector<double> toVec(const std::array<double, 4>& a) {
  return std::vector<double>(a.begin(), a.end());
}

// 子串包含（大小写不敏感），用于容忍 delete_model 的 "does not exist"
bool containsNoCase(const std::string& s, const std::string& sub) {
  std::string a = s, b = sub;
  std::transform(a.begin(), a.end(), a.begin(), ::tolower);
  std::transform(b.begin(), b.end(), b.begin(), ::tolower);
  return a.find(b) != std::string::npos;
}

// 由场景障碍生成静态 box SDF（位姿由 spawn 服务参数给出，SDF 内 pose 置零）
std::string obstacleSdf(const std::string& name, const agc::Obstacle& o) {
  char buf[640];
  std::snprintf(buf, sizeof(buf),
                "<sdf version='1.4'><model name='%s'><static>true</static>"
                "<pose>0 0 0 0 0 0</pose><link name='link'>"
                "<collision name='collision'><geometry><box><size>%g %g %g</size></box></geometry></collision>"
                "<visual name='visual'><geometry><box><size>%g %g %g</size></box></geometry>"
                "<material><ambient>1 0.5 0 1</ambient><diffuse>1 0.5 0 1</diffuse></material>"
                "</visual></link></model></sdf>",
                name.c_str(), o.sx, o.sy, o.sz, o.sx, o.sy, o.sz);
  return buf;
}

}  // namespace

class ScenarioReset {
 public:
  ScenarioReset() : nh_() {
    max_retries_ = agc::cfgI("reset/max_retries", 3);
    car_model_ = agc::cfgS("gazebo/car_model", "agc_car");
    target_model_ = agc::cfgS("gazebo/target_model", "intruder_uav");

    // 读取场景文件（路径来自 parameter server，不自己解析 YAML）
    scenario_file_ = agc::cfgS("scenario_file", "");
    dynamic_obstacles_ = agc::cfgB("scenario/dynamic_obstacles", false);
    sc_ = agc::loadScenario(scenario_file_);
    car_start_ = sc_.car_start;
    target_start_ = sc_.target_start;
    // target_start z 以 /agc/target/default_height_m 为准
    target_start_[2] = agc::cfgD("target/default_height_m", 2.0);

    // 启动期 Gazebo 服务可能尚未就绪：给足重试次数（0.5s 一次），不因早期竞态放弃
    init_max_attempts_ = agc::cfgI("reset/startup_attempts", 60);

    state_sub_ = nh_.subscribe("/agc/task/state", 1, &ScenarioReset::stateCb, this);
    models_sub_ = nh_.subscribe(gzNs() + "/model_states", 1, &ScenarioReset::modelStatesCb, this);

    srv_ = nh_.advertiseService("/agc/scenario/reset", &ScenarioReset::handleReset, this);
    // 0.5s 轮询：既是启动初始复位，也是场景文件热切换检测（变化即重载）
    timer_ = nh_.createTimer(ros::Duration(0.5), &ScenarioReset::tick, this);
    ROS_INFO("scenario_reset 就绪：car_start=[%.2f, %.2f, %.2f, %.2f] target_start=[%.2f, %.2f, %.2f, %.2f] max_retries=%d dynamic_obstacles=%d",
             car_start_[0], car_start_[1], car_start_[2], car_start_[3],
             target_start_[0], target_start_[1], target_start_[2], target_start_[3],
             max_retries_, dynamic_obstacles_ ? 1 : 0);
  }

 private:
  // ------------------------------------------------------------ 回调
  void stateCb(const air_ground_core::AgcTaskState::ConstPtr& msg) {
    state_ = static_cast<int>(msg->state);
    has_state_ = true;
    state_name_ = msg->state_name;
    if (state_name_.empty()) {
      const std::string n = agc::stateName(state_);
      state_name_ = (n == "UNKNOWN") ? "unknown" : n;
    }
  }

  void modelStatesCb(const gazebo_msgs::ModelStates::ConstPtr&) { model_states_seen_ = true; }

  // ------------------------------------------------------------ 工具
  // 调用 set_model_state 写模型位姿（world 系）；pose4=[x,y,z,yaw]
  bool setModel(const std::string& model_name, const std::array<double, 4>& pose4, std::string* err) {
    const std::string name = gzNs() + "/set_model_state";
    if (!agc::serviceReady(name, 2.0)) {
      if (err) *err = "set_model_state 服务不可用";
      return false;
    }
    gazebo_msgs::SetModelState srv;
    gazebo_msgs::ModelState& ms = srv.request.model_state;
    ms.model_name = model_name;
    ms.pose.position.x = pose4[0];
    ms.pose.position.y = pose4[1];
    ms.pose.position.z = pose4[2];
    const std::array<double, 4> q = agc::quatFromYaw(pose4[3]);
    ms.pose.orientation.x = q[0];
    ms.pose.orientation.y = q[1];
    ms.pose.orientation.z = q[2];
    ms.pose.orientation.w = q[3];
    ms.twist.linear.x = ms.twist.linear.y = ms.twist.linear.z = 0.0;
    ms.twist.angular.x = ms.twist.angular.y = ms.twist.angular.z = 0.0;
    ms.reference_frame = agc::FRAME_WORLD;
    if (!ros::service::call(name, srv)) {
      if (err) *err = "set_model_state 调用失败";
      return false;
    }
    if (!srv.response.success) {
      if (err) *err = "set_model_state 返回失败: " + srv.response.status_message;
      return false;
    }
    return true;
  }

  void bumpEpoch(const std::string& param) {
    ros::param::set(param, agc::pgetI(param, 0) + 1);
  }

  // 若空中重标定服务存在则调用；不存在则忽略并记录
  void tryRecalibrate() {
    const std::string name = "/agc/air/recalibrate";
    if (!agc::serviceReady(name, 2.0)) {
      ROS_INFO("scenario_reset: /agc/air/recalibrate 不存在，跳过空中重标定");
      return;
    }
    std_srvs::Trigger srv;
    if (ros::service::call(name, srv)) {
      ROS_INFO("scenario_reset: 已调用 /agc/air/recalibrate -> %s", srv.response.message.c_str());
    } else {
      ROS_INFO("scenario_reset: /agc/air/recalibrate 不可用，跳过");
    }
  }

  // ------------------------------------------------------------ 动态障碍热切换
  // 查询 Gazebo 当前模型名；失败返回 false
  bool worldModelNames(std::vector<std::string>* out) {
    const std::string name = gzNs() + "/get_world_properties";
    if (!agc::serviceReady(name, 1.0)) {
      ROS_WARN("scenario_reset: get_world_properties 不可用");
      return false;
    }
    gazebo_msgs::GetWorldProperties srv;
    if (!ros::service::call(name, srv)) return false;
    *out = srv.response.model_names;
    return true;
  }

  // 删除上一场景 spawn 的障碍 model；容忍 "does not exist"，并等待删除生效
  void deleteSpawnedObstacles() {
    if (spawned_models_.empty()) return;
    const std::string name = gzNs() + "/delete_model";
    if (!agc::serviceReady(name, 2.0)) {
      ROS_WARN("scenario_reset: delete_model 不可用，保留待下次重试");
      return;
    }
    for (const auto& m : spawned_models_) {
      gazebo_msgs::DeleteModel srv;
      srv.request.model_name = m;
      if (!ros::service::call(name, srv) || !srv.response.success) {
        const std::string msg = srv.response.status_message;
        if (!containsNoCase(msg, "does not exist"))
          ROS_WARN("scenario_reset: 删除动态障碍 %s 失败: %s", m.c_str(), msg.c_str());
      }
    }
    // 删除可能延迟生效，轮询直到旧模型从世界中消失（避免同名复用被延迟删除吃掉）
    for (int i = 0; i < 30; ++i) {
      std::vector<std::string> names;
      if (!worldModelNames(&names)) break;
      bool any = false;
      for (const auto& m : spawned_models_)
        if (std::find(names.begin(), names.end(), m) != names.end()) any = true;
      if (!any) break;
      ros::WallDuration(0.1).sleep();
    }
    spawned_models_.clear();
  }

  // 按场景 obstacles 用 spawn_sdf_model 生成静态 box；轮询直到全部可见
  bool spawnObstacles(const agc::Scenario& sc) {
    const std::string name = gzNs() + "/spawn_sdf_model";
    if (!agc::serviceReady(name, 2.0)) {
      ROS_WARN("scenario_reset: spawn_sdf_model 不可用");
      return false;
    }
    std::vector<std::string> deployed;
    for (size_t i = 0; i < sc.obstacles.size(); ++i) {
      const agc::Obstacle& o = sc.obstacles[i];
      // 名前缀用 scenario 里的 name 字段（空则回退 obstacle_<i>）
      const std::string mname = o.name.empty() ? ("obstacle_" + std::to_string(i)) : o.name;
      gazebo_msgs::SpawnModel srv;
      srv.request.model_name = mname;
      srv.request.model_xml = obstacleSdf(mname, o);
      srv.request.robot_namespace = "";
      srv.request.reference_frame = agc::FRAME_WORLD;
      srv.request.initial_pose.position.x = o.x;
      srv.request.initial_pose.position.y = o.y;
      srv.request.initial_pose.position.z = o.z;
      const std::array<double, 4> q = agc::quatFromYaw(o.yaw);
      srv.request.initial_pose.orientation.x = q[0];
      srv.request.initial_pose.orientation.y = q[1];
      srv.request.initial_pose.orientation.z = q[2];
      srv.request.initial_pose.orientation.w = q[3];
      if (!ros::service::call(name, srv) || !srv.response.success) {
        ROS_ERROR("scenario_reset: spawn 动态障碍 %s 失败: %s", mname.c_str(),
                  srv.response.status_message.c_str());
        spawned_models_ = deployed;  // 记录已成功者，供下次删除
        return false;
      }
      deployed.push_back(mname);
    }
    // spawn 可能先返回后可见：轮询 get_world_properties 直到全部出现
    for (int i = 0; i < 30; ++i) {
      std::vector<std::string> names;
      if (worldModelNames(&names)) {
        bool all = true;
        for (const auto& m : deployed)
          if (std::find(names.begin(), names.end(), m) == names.end()) { all = false; break; }
        if (all) break;
      }
      ros::WallDuration(0.1).sleep();
    }
    spawned_models_ = deployed;
    return true;
  }

  // 一次完整部署：删旧 -> 放新 -> 自增 /agc/scenario/epoch
  void deployDynamicObstacles() {
    dyn_deployed_once_ = true;
    deleteSpawnedObstacles();
    if (spawnObstacles(sc_)) {
      bumpEpoch("/agc/scenario/epoch");
      ROS_INFO("scenario_reset: 动态障碍已切换 (%zu 个)，scenario/epoch=%d",
               spawned_models_.size(), agc::pgetI("/agc/scenario/epoch", 0));
    } else {
      ROS_ERROR("scenario_reset: 动态障碍切换失败，scenario/epoch 未推进");
    }
  }

  // 轮询 /agc/scenario_file：变化即重载 car/target 起点；dynamic 时切换障碍
  bool checkScenarioReload() {
    const std::string f = agc::cfgS("scenario_file", "");
    if (f == scenario_file_) return false;
    scenario_file_ = f;
    const agc::Scenario sc = agc::loadScenario(f);
    if (!sc.loaded) {
      ROS_ERROR("scenario_reset: 无法读取新场景文件 %s", f.c_str());
      return false;
    }
    sc_ = sc;
    car_start_ = sc_.car_start;
    target_start_ = sc_.target_start;
    target_start_[2] = agc::cfgD("target/default_height_m", 2.0);
    ROS_INFO("scenario_reset: 检测到场景文件变更 %s，重载 car_start/target_start", f.c_str());
    if (dynamic_obstacles_) deployDynamicObstacles();
    return true;
  }

  // ------------------------------------------------------------ 复位核心
  // 执行一次复位（含重试）。成功返回 true。
  // 注意：复位失败属基础设施失败，绝不进入任务失败分子。
  bool performReset(const std::string& reason) {
    bool ok = false;
    std::string err;
    int attempts = 0;
    for (int attempt = 1; attempt <= max_retries_; ++attempt) {
      attempts = attempt;
      std::string e1, e2;
      if (setModel(car_model_, car_start_, &e1) && setModel(target_model_, target_start_, &e2)) {
        ok = true;
        break;
      }
      err = e1.empty() ? e2 : e1;
      ROS_WARN("scenario_reset: 第 %d/%d 次复位失败：%s", attempt, max_retries_, err.c_str());
    }

    agc::JsonObj detail;
    detail.v("reason", reason).iv("attempts", attempts);
    detail.raw("car_start", agc::JsonObj::arrNum(toVec(car_start_)));
    detail.raw("target_start", agc::JsonObj::arrNum(toVec(target_start_)));
    detail.v("obstacles", dynamic_obstacles_ ? "动态障碍由 scenario_reset 热切换管理"
                                             : "静态障碍不复位（本就固定不动）");
    detail.iv("scenario_epoch", agc::pgetI("/agc/scenario/epoch", 0));
    if (ok) {
      // 通知 intruder_driver 归零内部相位；保持 target 使能
      bumpEpoch("/agc/target/reset_epoch");
      // 任一次复位成功即自增复位序号
      bumpEpoch("/agc/reset/epoch");
      ros::param::set("/agc/reset/last_result", "SUCCESS");
      tryRecalibrate();
      ROS_INFO("scenario_reset: 复位成功（%s）", reason.c_str());
    } else {
      ros::param::set("/agc/reset/last_result", "RESET_FAILURE");
      detail.v("error", err);
      ROS_ERROR("scenario_reset: 复位失败（%s）：%s", reason.c_str(), err.c_str());
    }

    agc::JsonObj o;
    o.n("t_sim", agc::simNow().toSec()).n("t_wall", agc::wallNow());
    if (has_state_) o.iv("task_state", state_); else o.nullv("task_state");
    o.v("task_state_name", state_name_)
        .bv("success", ok)
        .iv("reset_epoch", agc::pgetI("/agc/reset/epoch", 0))
        .raw("detail", detail.dump());
    agc::appendJsonl("reset_log.jsonl", o);
    return ok;
  }

  // ------------------------------------------------------------ 服务
  bool handleReset(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    // 复位前先吸收场景文件变更（消除「批跑器已换场景但轮询未到」的竞态）
    checkScenarioReload();
    // 门禁：仅 IDLE 或终态允许复位
    if (!has_state_) {
      ROS_WARN("scenario_reset: 尚未收到 /agc/task/state，拒绝复位");
      res.success = false;
      res.message = "task state unknown";
      return true;
    }
    if (!allowedResetState(state_)) {
      std::string name = agc::stateName(state_);
      if (name == "UNKNOWN") name = state_name_;
      ROS_WARN("scenario_reset: 任务运行中，拒绝复位（state=%s）", name.c_str());
      res.success = false;
      res.message = "task running: " + name;
      return true;
    }
    const bool ok = performReset("service");
    res.success = ok;
    res.message = ok ? "SUCCESS" : "RESET_FAILURE";
    return true;
  }

  // ------------------------------------------------------------ 定时器
  // 每拍：先吸收场景文件热切换，再推进启动初始复位
  void tick(const ros::TimerEvent&) {
    checkScenarioReload();
    initialTick();
  }

  // ------------------------------------------------------------ 启动复位
  void initialTick() {
    if (init_done_ && dyn_deployed_once_) return;
    if (!model_states_seen_) return;
    // dynamic：启动即部署初始场景障碍（动态 world 无固定障碍），default 时不触发
    if (dynamic_obstacles_ && !dyn_deployed_once_) deployDynamicObstacles();
    if (init_done_) return;
    if (init_attempts_ >= init_max_attempts_) {
      init_done_ = true;
      ros::param::set("/agc/reset/last_result", "RESET_FAILURE");
      ROS_ERROR("scenario_reset: 启动复位连续 %d 次失败，放弃", init_attempts_);
      return;
    }
    init_attempts_ += 1;
    ROS_WARN_THROTTLE(5.0, "scenario_reset: 等待 Gazebo set_model_state 就绪并启动复位（第 %d 次）", init_attempts_);
    if (performReset("startup")) init_done_ = true;
  }

  ros::NodeHandle nh_;
  ros::Subscriber state_sub_, models_sub_;
  ros::ServiceServer srv_;
  ros::Timer timer_;

  int max_retries_ = 3;
  std::string car_model_ = "agc_car", target_model_ = "intruder_uav";
  std::array<double, 4> car_start_{{0.0, 0.0, 0.11, 0.0}};
  std::array<double, 4> target_start_{{0.0, 0.0, 2.0, 0.0}};

  // 热切换状态
  std::string scenario_file_;
  bool dynamic_obstacles_ = false;
  agc::Scenario sc_;
  std::vector<std::string> spawned_models_;
  bool dyn_deployed_once_ = false;

  int state_ = 0;
  bool has_state_ = false;
  std::string state_name_ = "unknown";
  bool model_states_seen_ = false;
  bool init_done_ = false;
  int init_attempts_ = 0;
  int init_max_attempts_ = 60;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "scenario_reset");
  ScenarioReset node;
  ros::spin();
  return 0;
}