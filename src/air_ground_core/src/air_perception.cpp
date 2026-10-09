// air_perception —— 真值派生的感知前端（M0 接口契约 v1.2 第八节，C++ 实时链路）
//
// 职责：把 Gazebo 真值（目标 intruder_uav + 场景静态障碍）按多个"传感器通道"
// 转成带来源/坐标系/时间戳/置信度的观测，发布 /agc/air/obs（AirObservationArray）。
// 说明（诚实口径）：这是**真值派生的感知仿真**，不是真实视觉/雷达识别。
//
// 关键约束：
// 1. 每通道独立 RNG 噪声 + 独立延迟/丢包/离群偏置；不共享随机流。
// 2. 观测 header.stamp = 原始观测时刻；延迟只推迟"发布"，因此消费者看到的
//    观测年龄 = now - stamp 会随延迟增长（延迟注入即用此项）。
// 3. confidence 由噪声模型解析给出：2D 径向误差覆盖 P(|err|<=tol)，并与
//    /agc/calibration/confidence_tolerance_m 一致；离线做经验覆盖核验。
// 4. 故障通过在线 rosparam 注入（run 前固定或运行期按时序）：
//    /agc/perception/fault/<sensor>/{enabled,drop_prob,latency_s,outlier_bias_m}
// 5. 落 perception_log.jsonl（每条样本的通道、序号、故障参数、观测结果）。
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <yaml-cpp/yaml.h>

#include <air_ground_core/AirObservationArray.h>
#include <air_ground_core/SensorObservation.h>
#include <air_ground_core/TargetObservation.h>
#include <air_ground_core/ObstacleObservation.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <map>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "agc/agc_common.hpp"

namespace {

const char* const SOURCE = "air_perception";

// 单通道配置（来自 perception_sensors.yaml）
struct SensorCfg {
  std::string id;
  double sigma_xy = 0.10, sigma_z = 0.12;
  double latency_s = 0.0, drop_prob = 0.0;
  double confidence = 1.0;  // 由 sigma_xy 与 tol 解析得到
  // N1 视场/距离门控：机体系朝向（boresight）+ 全视场（弧度，水平/垂直）+ 距离量程
  std::array<double, 3> boresight{{1.0, 0.0, 0.0}};
  double hfov_rad = M_PI / 2.0, vfov_rad = M_PI / 3.0;
  double range_min = 0.30, range_max = 12.0;
  // ---- A0 观测模型字段（默认 0/关，与 N1 逐位等价）----
  double corr_xy = 0.0;                        // 通道内 x/y 噪声相关系数 ρ
  std::string corr_group;                      // 通道间公共模态组名（空=不参与）
  double corr_rho = 0.0;                       // 公共模态权重 ρ_c
  double noise_tau_s = 0.0;                    // 噪声时间相关常数(s)，0=逐拍独立
  std::array<double, 3> bias_m{{0.0, 0.0, 0.0}};  // 确定性偏置（world 系，m）
  double drop_burst_prob = 0.0;                // 进入突发态的概率缩放（∈(0,1]，0=关闭）
  double drop_burst_mean_s = 0.5;              // 突发态平均持续(s)
  double drop_burst_mean_gap_s = 5.0;          // 正常态平均持续(s)
  std::string drop_group;                      // 跨通道共同丢失组名（空=独立）
  double false_alarm_rate_hz = 0.0;            // 虚警发生率(Hz)
};

// 一次采样（已加噪），延迟释放后转成观测消息
struct ObsSample {
  double t_obs = 0.0;      // 原始观测时刻（仿真时钟）
  bool has_target = false;
  double tx = 0, ty = 0, tz = 0, tyaw = 0;
  unsigned int tseq = 0;
  // N1 门控结果（仅目标；用于 perception_log 记录拒收原因与诊断量）
  bool gate_ok = false;
  std::string gate_reason = "no_target";
  double gate_range = 0.0, gate_az_deg = 0.0, gate_el_deg = 0.0;
  // A0 评估日志扩展字段（仅离线日志，绝不进入发布消息）
  std::string drop_source;             // "independent" | "burst" | ""
  bool burst_state = false;            // 本拍突发丢失态
  std::string occluder;                // 目标遮挡障碍名（未遮挡为空）
  bool is_false_alarm = false;         // 本拍是否产生虚警（幻影目标）
  std::array<double, 3> fa_pos{{0.0, 0.0, 0.0}};  // 虚警位置
  struct Ob {
    std::string id;
    double x = 0, y = 0, z = 0, yaw = 0, sx = 0, sy = 0, sz = 0;
    unsigned int seq = 0;
  };
  std::vector<Ob> obs;
};

struct Pending {
  double release_t = 0.0;  // 释放（可发布）时刻 = t_obs + latency
  ObsSample s;
};

// 协方差：对角 x/y/z + x-y 相关项（ρ=0 时退化为纯对角，与现状一致）
void cov3(double sxy, double sz, double rho, boost::array<double, 36>& c) {
  c.assign(0.0);
  const double v = sxy * sxy;
  c[0] = c[7] = v;
  c[1] = c[6] = rho * v;
  c[14] = sz * sz;
}

// 3D 点乘序列
std::string seq3(double x, double y, double z) {
  return agc::JsonObj::arrNum({x, y, z});
}

// 线段 p0→p1 与障碍 OBB（按 pad 外扩）是否相交（局部 slab/AABB 测试）。
// 线段两端绕 yaw 反向旋转到障碍局部系，再做三轴 slab 相交。
bool segHitsObb(const std::array<double, 3>& p0, const std::array<double, 3>& p1,
                const agc::Obstacle& o, double pad) {
  const double cy = std::cos(-o.yaw), sy = std::sin(-o.yaw);
  auto toLocal = [&](const std::array<double, 3>& p) -> std::array<double, 3> {
    const double dx = p[0] - o.x, dy = p[1] - o.y, dz = p[2] - o.z;
    return {{cy * dx - sy * dy, sy * dx + cy * dy, dz}};
  };
  const std::array<double, 3> a = toLocal(p0), b = toLocal(p1);
  const std::array<double, 3> h{{o.sx * 0.5 + pad, o.sy * 0.5 + pad, o.sz * 0.5 + pad}};
  double tmin = 0.0, tmax = 1.0;  // 线段参数区间
  for (int i = 0; i < 3; ++i) {
    const double d = b[i] - a[i];
    if (std::fabs(d) < 1e-9) {
      if (a[i] < -h[i] || a[i] > h[i]) return false;  // 平行且在该轴 slab 外
    } else {
      double t1 = (-h[i] - a[i]) / d, t2 = (h[i] - a[i]) / d;
      if (t1 > t2) std::swap(t1, t2);
      tmin = std::max(tmin, t1);
      tmax = std::min(tmax, t2);
      if (tmin > tmax) return false;
    }
  }
  return tmin <= tmax;
}

// 公共模态噪声组（同 corr_group 共享一条独立流，每拍每组每轴抽一次）
struct CommonGroup {
  std::mt19937 rng;
  long long drawn_tick = -1;  // 本拍是否已抽取（tick 序号）
  double gx = 0.0, gy = 0.0;
};

// 成簇丢失组（同 drop_group 共享流与两态状态，每拍每组只更新一次）
struct BurstGroup {
  std::mt19937 rng;
  bool state = false;
  bool init = false;
  double last_t = 0.0;
  long long last_tick = -1;
};

// reseed 模式下按 (通道,角色) 维护的 bucket 栅格 AR(1) 状态
struct ArReseedState {
  std::array<double, 3> st{{0.0, 0.0, 0.0}};
  long long last_bucket = 0;
  bool init = false;
};

// 确定性重播种用的混合函数（雪崩性）；同 (seed, key, bucket) 恒得同一 seed
uint32_t mix32(uint32_t a, uint32_t b, uint32_t c) {
  uint32_t h = a * 2654435761u ^ (b * 2246822519u) ^ (c * 3266489917u);
  h ^= h >> 16;
  h *= 0x7feb352du;
  h ^= h >> 15;
  h *= 0x846ca68bu;
  h ^= h >> 16;
  return h;
}

// 组名稳定哈希（FNV-1a），用于 drop_group 的重播种 key
uint32_t fnv1a(const std::string& s) {
  uint32_t h = 2166136261u;
  for (unsigned char ch : s) {
    h ^= ch;
    h *= 16777619u;
  }
  return h;
}

}  // namespace

class AirPerception {
 public:
  AirPerception() : nh_() {
    ns_ = agc::pgetStr("/agc/gazebo/ns", "/gazebo");
    while (!ns_.empty() && ns_.back() == '/') ns_.pop_back();
    target_model_ = agc::pgetStr("/agc/gazebo/target_model", "intruder_uav");
    drone_model_ = agc::pgetStr("/agc/gazebo/drone_model", "iris");
    target_id_ = agc::pgetStr("/agc/target/id", "intruder_uav");
    obs_hz_ = agc::pgetD("/agc/perception/obs_hz", 20.0);
    tol_m_ = agc::pgetD("/agc/calibration/confidence_tolerance_m",
                        agc::pgetD("/agc/perception/tol_m", 0.25));
    sensors_file_ = agc::pgetStr("/agc/perception/sensors_file", "");
    seed_ = agc::pgetStr("/agc/log/seed", "unseeded");
    loadSensors();

    scenario_file_ = agc::pgetStr("/agc/scenario_file", "");
    obstacles_ = agc::loadScenario(scenario_file_).obstacles;

    nh_.setParam("/agc/perception/source", std::string(SOURCE));
    nh_.setParam("/agc/perception/confidence_tolerance_m", tol_m_);
    for (const auto& s : sensors_) {
      nh_.setParam("/agc/perception/confidence/" + s.id, s.confidence);
    }

    pub_ = nh_.advertise<air_ground_core::AirObservationArray>("/agc/air/obs", 10);
    sub_ = nh_.subscribe(ns_ + "/model_states", 1, &AirPerception::onModelStates, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(obs_hz_, 1e-3)),
                             &AirPerception::tick, this);
    // 启动审计汇总：通道数与各 A0 观测模型开关使用情况
    bool has_common = false, has_tcorr = false, has_burst = false, has_fa = false;
    for (const auto& s : sensors_) {
      if (!s.corr_group.empty() && s.corr_rho != 0.0) has_common = true;
      if (s.noise_tau_s > 0.0) has_tcorr = true;
      if (s.drop_burst_prob > 0.0) has_burst = true;
      if (s.false_alarm_rate_hz > 0.0) has_fa = true;
    }
    ROS_INFO("air_perception 就绪：通道=%zu obs_hz=%.1f tol=%.3f m seed=%s 遮挡=%s(%.3f m) "
             "公共模态=%s 时间相关=%s 成簇丢失=%s 虚警=%s 确定性重播种=%s（真值派生的感知仿真，非真实识别）",
             sensors_.size(), obs_hz_, tol_m_, seed_.c_str(), occlusion_enabled_ ? "on" : "off",
             occlusion_pad_m_, has_common ? "on" : "off", has_tcorr ? "on" : "off",
             has_burst ? "on" : "off", has_fa ? "on" : "off", reseed_enabled_ ? "on" : "off");
  }

 private:
  // 解析通道配置；confidence 由解析式给出
  void loadSensors() {
    if (sensors_file_.empty()) {
      ROS_ERROR("air_perception: /agc/perception/sensors_file 未设置，无法读取通道配置");
      return;
    }
    YAML::Node root;
    try {
      root = YAML::LoadFile(sensors_file_);
    } catch (const std::exception& e) {
      ROS_ERROR("air_perception: 读取 %s 失败: %s", sensors_file_.c_str(), e.what());
      return;
    }
    const YAML::Node list = root["sensors"];
    if (!list || !list.IsSequence()) {
      ROS_ERROR("air_perception: %s 缺 sensors 序列", sensors_file_.c_str());
      return;
    }
    // 基线种子上溯：seed 可解析为整数时用其值，否则用确定性默认 1（可复现）
    long base_seed = 1;
    {
      char* endp = nullptr;
      const long v = std::strtol(seed_.c_str(), &endp, 10);
      if (endp != seed_.c_str() && *endp == '\0') base_seed = v;
    }
    base_seed_ = base_seed;
    size_t idx = 0;
    for (const auto& n : list) {
      SensorCfg c;
      if (n["id"]) c.id = n["id"].as<std::string>();
      if (c.id.empty()) { ++idx; continue; }
      if (n["sigma_xy_m"]) c.sigma_xy = n["sigma_xy_m"].as<double>();
      if (n["sigma_z_m"]) c.sigma_z = n["sigma_z_m"].as<double>();
      if (n["latency_s"]) c.latency_s = n["latency_s"].as<double>();
      if (n["drop_prob"]) c.drop_prob = n["drop_prob"].as<double>();
      // N1 视场/距离门控参数（缺省：前视半视场 90°/60°，量程 0.3~12 m）
      if (n["boresight_body"] && n["boresight_body"].IsSequence() && n["boresight_body"].size() >= 3) {
        c.boresight = {{n["boresight_body"][0].as<double>(),
                        n["boresight_body"][1].as<double>(),
                        n["boresight_body"][2].as<double>()}};
      }
      if (n["hfov_deg"]) c.hfov_rad = n["hfov_deg"].as<double>() * M_PI / 180.0;
      if (n["vfov_deg"]) c.vfov_rad = n["vfov_deg"].as<double>() * M_PI / 180.0;
      if (n["range_min_m"]) c.range_min = n["range_min_m"].as<double>();
      if (n["range_max_m"]) c.range_max = n["range_max_m"].as<double>();
      // ---- A0 观测模型字段（全部缺省即 0/关，保证默认逐位等价）----
      if (n["corr_xy"]) c.corr_xy = n["corr_xy"].as<double>();
      if (n["corr_group"]) c.corr_group = n["corr_group"].as<std::string>();
      if (n["corr_rho"]) c.corr_rho = n["corr_rho"].as<double>();
      if (n["noise_tau_s"]) c.noise_tau_s = n["noise_tau_s"].as<double>();
      if (n["bias_m"] && n["bias_m"].IsSequence() && n["bias_m"].size() >= 3) {
        c.bias_m = {{n["bias_m"][0].as<double>(), n["bias_m"][1].as<double>(),
                     n["bias_m"][2].as<double>()}};
      }
      if (n["drop_burst_prob"]) c.drop_burst_prob = n["drop_burst_prob"].as<double>();
      if (n["drop_burst_mean_s"]) c.drop_burst_mean_s = n["drop_burst_mean_s"].as<double>();
      if (n["drop_burst_mean_gap_s"]) c.drop_burst_mean_gap_s = n["drop_burst_mean_gap_s"].as<double>();
      if (n["drop_group"]) c.drop_group = n["drop_group"].as<std::string>();
      if (n["false_alarm_rate_hz"]) c.false_alarm_rate_hz = n["false_alarm_rate_hz"].as<double>();
      // 2D 径向（Rayleigh）覆盖：P(R<=tol)=1-exp(-tol^2/(2 sigma^2))
      const double s2 = c.sigma_xy * c.sigma_xy;
      c.confidence = (s2 > 1e-12) ? (1.0 - std::exp(-(tol_m_ * tol_m_) / (2.0 * s2))) : 1.0;
      c.confidence = std::min(1.0, std::max(0.0, c.confidence));
      // 噪声流：种子与现状完全相同（base_seed + 101*(idx+1)）
      rng_noise_.emplace_back(static_cast<unsigned>(base_seed + 101 * (idx + 1)));
      // 成簇/虚警：不同命名空间派生，默认关闭时不在其上抽样，故不影响噪声流序列
      rng_burst_.emplace_back(static_cast<unsigned>(base_seed + 1000003 + 101 * (idx + 1)));
      rng_fa_.emplace_back(static_cast<unsigned>(base_seed + 2000003 + 101 * (idx + 1)));
      burst_state_.push_back(false);
      a_prev_target_.push_back({{0.0, 0.0, 0.0}});
      a_prev_obs_.push_back({{0.0, 0.0, 0.0}});
      has_prev_t_.push_back(false);
      last_t_.push_back(0.0);
      // 公共模态组：首次遇到时创建独立流（种子按组出现次序派生）
      if (!c.corr_group.empty() && c.corr_rho != 0.0 && !common_groups_.count(c.corr_group)) {
        CommonGroup g;
        g.rng.seed(static_cast<unsigned>(base_seed + 3000003 + 101 * (++common_group_seq_)));
        common_groups_.emplace(c.corr_group, g);
      }
      // 成簇丢失组：首次遇到时用该通道的成簇流种子初始化，组内共用流与状态
      if (!c.drop_group.empty() && !burst_groups_.count(c.drop_group)) {
        BurstGroup g;
        g.rng.seed(static_cast<unsigned>(base_seed + 1000003 + 101 * (idx + 1)));
        burst_groups_.emplace(c.drop_group, g);
      }
      sensors_.push_back(c);
      pending_.emplace_back();
      tseq_.push_back(0);
      oseq_.push_back(0);
      ++idx;
    }
    // 顶层 perception_model.occlusion（缺省回退 rosparam）
    bool oc_en = agc::pgetB("/agc/perception/occlusion_enabled", false);
    double oc_pad = agc::pgetD("/agc/perception/occlusion_pad_m", 0.0);
    const YAML::Node pm = root["perception_model"];
    if (pm && pm["occlusion"]) {
      const YAML::Node oc = pm["occlusion"];
      if (oc["enabled"]) oc_en = oc["enabled"].as<bool>();
      if (oc["pad_m"]) oc_pad = oc["pad_m"].as<double>();
    }
    occlusion_enabled_ = oc_en;
    occlusion_pad_m_ = oc_pad;
    // 顶层 perception_model.deterministic_reseed（缺省回退 rosparam，默认 false=关闭）
    bool rs = agc::pgetB("/agc/perception/deterministic_reseed", false);
    if (pm && pm["deterministic_reseed"]) rs = pm["deterministic_reseed"].as<bool>();
    reseed_enabled_ = rs;
  }

  // 真值缓存：目标位置/朝向 + 无人机位姿（N1 门控按“原始观测时刻”的无人机位姿判定）
  void onModelStates(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    target_present_ = false;
    for (size_t i = 0; i < msg->name.size() && i < msg->pose.size(); ++i) {
      const geometry_msgs::Pose& p = msg->pose[i];
      if (msg->name[i] == target_model_) {
        truth_ = {p.position.x, p.position.y, p.position.z};
        truth_yaw_ = agc::yawFromQuat(p.orientation);
        target_present_ = true;
      } else if (msg->name[i] == drone_model_) {
        drone_pos_ = {p.position.x, p.position.y, p.position.z};
        drone_quat_ = {{p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w}};
        drone_pose_ok_ = true;
      }
    }
  }

  // N1 视场/距离门控：传感器原点 = 无人机位置，朝向 = 无人机姿态 × 机体系 boresight。
  // 返回 true=在视场内；reason 记 "ok"/"no_drone_pose"/"out_of_range_min"/
  // "out_of_range_max"/"behind"/"out_of_fov"；range/az_deg/el_deg 为诊断量。
  bool gatePoint(const SensorCfg& c, const std::array<double, 3>& pt, std::string* reason,
                 double* range, double* az_deg, double* el_deg) const {
    if (!drone_pose_ok_) { *reason = "no_drone_pose"; return false; }
    const std::array<double, 3> d{
        {pt[0] - drone_pos_[0], pt[1] - drone_pos_[1], pt[2] - drone_pos_[2]}};
    const double r = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    *range = r;
    if (r < c.range_min) { *reason = "out_of_range_min"; return false; }
    if (r > c.range_max) { *reason = "out_of_range_max"; return false; }
    const std::array<double, 3> u{{d[0] / r, d[1] / r, d[2] / r}};
    const std::array<double, 3> bore = agc::rotateVecByQuat(drone_quat_, c.boresight);
    // 传感器横轴 = boresight × world_up；纵轴 = 横轴 × boresight
    const std::array<double, 3> up{{0.0, 0.0, 1.0}};
    std::array<double, 3> right{{bore[1] * up[2] - bore[2] * up[1],
                                 bore[2] * up[0] - bore[0] * up[2],
                                 bore[0] * up[1] - bore[1] * up[0]}};
    double rn = std::sqrt(right[0] * right[0] + right[1] * right[1] + right[2] * right[2]);
    if (rn < 1e-6) { right = {{0.0, 1.0, 0.0}}; rn = 1.0; }
    for (double& v : right) v /= rn;
    const std::array<double, 3> up2{{right[1] * bore[2] - right[2] * bore[1],
                                     right[2] * bore[0] - right[0] * bore[2],
                                     right[0] * bore[1] - right[1] * bore[0]}};
    const double fwd = u[0] * bore[0] + u[1] * bore[1] + u[2] * bore[2];
    const double rc = u[0] * right[0] + u[1] * right[1] + u[2] * right[2];
    const double uc = u[0] * up2[0] + u[1] * up2[1] + u[2] * up2[2];
    const double az = std::atan2(rc, fwd);
    const double el = std::atan2(uc, std::hypot(fwd, rc));
    *az_deg = az * 180.0 / M_PI;
    *el_deg = el * 180.0 / M_PI;
    if (fwd <= 0.0) { *reason = "behind"; return false; }
    if (std::fabs(az) > c.hfov_rad / 2.0 || std::fabs(el) > c.vfov_rad / 2.0) {
      *reason = "out_of_fov";
      return false;
    }
    *reason = "ok";
    return true;
  }

  // 读取某通道本拍的故障参数（enabled=true 时用 fault 值，否则用文件默认值）
  void faultParams(const SensorCfg& c, double* drop, double* lat, double* bias) const {
    const std::string base = "/agc/perception/fault/" + c.id;
    const bool en = agc::pgetB(base + "/enabled", false);
    *drop = en ? agc::pgetD(base + "/drop_prob", c.drop_prob) : c.drop_prob;
    *lat = en ? agc::pgetD(base + "/latency_s", c.latency_s) : c.latency_s;
    *bias = en ? agc::pgetD(base + "/outlier_bias_m", 0.0) : 0.0;
  }

  // 是否启用噪声相关模型（任一相关/时间相关字段非默认）
  static bool noiseModelOn(const SensorCfg& c) {
    return c.corr_xy != 0.0 || c.noise_tau_s > 0.0 ||
           (!c.corr_group.empty() && c.corr_rho != 0.0);
  }

  // 单位噪声 u（逐轴）→ 公共模态 → AR(1)；naxes=2（障碍）时不处理 z 轴
  void applyNoiseModel(size_t si, const SensorCfg& c, double dt, int naxes,
                       std::array<double, 3>& u, std::array<double, 3>* ar_state) {
    // 通道间公共模态：同组每拍每轴只抽一次，组内共用同一 g
    if (!c.corr_group.empty() && c.corr_rho != 0.0) {
      auto it = common_groups_.find(c.corr_group);
      if (it != common_groups_.end()) {
        CommonGroup& g = it->second;
        if (g.drawn_tick != tick_id_) {
          std::normal_distribution<double> gn(0.0, 1.0);
          g.gx = gn(g.rng);
          g.gy = gn(g.rng);
          g.drawn_tick = tick_id_;
        }
        const double rc = c.corr_rho, sc = std::sqrt(std::max(0.0, 1.0 - rc * rc));
        u[0] = rc * g.gx + sc * u[0];
        u[1] = rc * g.gy + sc * u[1];
      }
    }
    // AR(1) 时间相关：a = ρ_t*a_prev + sqrt(1-ρ_t²)*u（τ=0 时不介入，保持 a=u）
    // 目标与障碍使用各自独立的 AR 状态缓冲，避免同拍内互相覆盖时间相关链
    if (c.noise_tau_s > 0.0) {
      (void)si;
      const double rt = (dt > 0.0) ? std::exp(-dt / c.noise_tau_s) : 0.0;
      const double q = std::sqrt(std::max(0.0, 1.0 - rt * rt));
      for (int i = 0; i < naxes; ++i) {
        u[i] = rt * (*ar_state)[i] + q * u[i];
        (*ar_state)[i] = u[i];
      }
    }
  }

  // reseed：某 (si, role) 在 bucket b 的单位白噪声（含 corr_xy 混合与公共模态叠加）。
  // role: -1=目标；>=0=障碍序号 j。全部由 (base_seed_, key, bucket) 纯函数派生。
  void unitNoiseReseed(size_t si, int role, const SensorCfg& c, long long b, int naxes,
                       std::array<double, 3>* out) {
    const uint32_t key = (role < 0)
                             ? static_cast<uint32_t>(si)
                             : static_cast<uint32_t>(si * 100000u + static_cast<size_t>(role));
    std::mt19937 rng(mix32(static_cast<uint32_t>(base_seed_), key, static_cast<uint32_t>(b)));
    std::normal_distribution<double> n(0.0, 1.0);
    std::array<double, 3> u{{0.0, 0.0, 0.0}};
    u[0] = n(rng);  // w1
    u[1] = n(rng);  // w2
    if (naxes >= 3) u[2] = n(rng);  // wz（顺序在 x,y 之后）
    // corr_xy 通道内相关
    if (c.corr_xy != 0.0) {
      const double r = c.corr_xy;
      u[1] = r * u[0] + std::sqrt(std::max(0.0, 1.0 - r * r)) * u[1];
    }
    // 公共模态：按 (组,bucket) 纯函数派生，同组同 bucket 恒得同一 g（等价于以 bucket 为缓存键）
    if (!c.corr_group.empty() && c.corr_rho != 0.0) {
      std::mt19937 grng(mix32(static_cast<uint32_t>(base_seed_ + 0xC001u), fnv1a(c.corr_group),
                              static_cast<uint32_t>(b)));
      std::normal_distribution<double> gn(0.0, 1.0);
      const double gx = gn(grng), gy = gn(grng);
      const double rc = c.corr_rho, sc = std::sqrt(std::max(0.0, 1.0 - rc * rc));
      u[0] = rc * gx + sc * u[0];
      u[1] = rc * gy + sc * u[1];
    }
    *out = u;
  }

  // reseed：在 bucket 栅格上确定性递推 AR(1)，返回 bucket b 的单位噪声 a。
  // 栅格步长 dt_b=1/obs_hz_，衰减 rho=exp(-dt_b/τ)；τ<=0 时 a=u。
  void arNoiseReseed(size_t si, int role, const SensorCfg& c, long long b, int naxes,
                     std::array<double, 3>* out) {
    std::array<double, 3> u;
    unitNoiseReseed(si, role, c, b, naxes, &u);
    if (c.noise_tau_s <= 0.0) {  // 无时间相关：a=u
      *out = u;
      return;
    }
    ArReseedState& s = ar_reseed_[std::make_pair(si, role)];
    const double dt_b = 1.0 / std::max(obs_hz_, 1e-9);
    const double rho = std::exp(-dt_b / c.noise_tau_s);
    const double q = std::sqrt(std::max(0.0, 1.0 - rho * rho));
    if (!s.init) {  // 首桶：零记忆做一步
      s.st = {{0.0, 0.0, 0.0}};
      for (int i = 0; i < naxes; ++i) s.st[i] = q * u[i];
      s.last_bucket = b;
      s.init = true;
      *out = s.st;
      return;
    }
    if (b <= s.last_bucket) {  // 同桶/回退：不重复递推
      *out = s.st;
      return;
    }
    long long span = b - s.last_bucket;
    if (span > 100000) {  // 递推跨度上限，超限重置并把 last 设为 b-1（本桶再走一步）
      ROS_WARN_THROTTLE(10.0, "air_perception: reseed AR(1) 递推跨度过大(%lld)，重置状态", span);
      s.st = {{0.0, 0.0, 0.0}};
      s.last_bucket = b - 1;
      span = 1;
    }
    (void)span;
    for (long long k = s.last_bucket + 1; k <= b; ++k) {
      std::array<double, 3> uk;
      unitNoiseReseed(si, role, c, k, naxes, &uk);
      for (int i = 0; i < naxes; ++i) s.st[i] = rho * s.st[i] + q * uk[i];
    }
    s.last_bucket = b;
    *out = s.st;
  }

  // 成簇丢失两态状态机；返回本拍是否处于突发丢失态。
  // drop_burst_prob<=0 时永不为 burst 且不抽样；同 drop_group 组共用流与状态，每拍每组只更新一次。
  // reseed_enabled_ 时转移抽样改用按 (key,bucket) 派生的局部生成器；否则用持久流。
  bool burstThisTick(size_t si, const SensorCfg& c, double t, double dt, long long bucket) {
    if (c.drop_burst_prob <= 0.0) return false;
    const double eps = 1e-6;
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    std::mt19937 reseed_rng;
    if (!c.drop_group.empty()) {
      auto it = burst_groups_.find(c.drop_group);
      if (it == burst_groups_.end()) return false;
      BurstGroup& g = it->second;
      if (g.last_tick != tick_id_) {
        const double gdt = g.init ? (t - g.last_t) : 0.0;
        std::mt19937* brng = &g.rng;
        if (reseed_enabled_) {  // key = 组名哈希
          reseed_rng.seed(mix32(static_cast<uint32_t>(base_seed_ + 0xB001u),
                                fnv1a(c.drop_group), static_cast<uint32_t>(bucket)));
          brng = &reseed_rng;
        }
        if (g.state) {
          const double p_bg = 1.0 - std::exp(-std::max(gdt, 0.0) / std::max(c.drop_burst_mean_s, eps));
          if (uni(*brng) < p_bg) g.state = false;
        } else {
          // 进入突发态：基础转移率按 drop_burst_prob 缩放
          const double base = 1.0 - std::exp(-std::max(gdt, 0.0) / std::max(c.drop_burst_mean_gap_s, eps));
          const double p_gb = std::min(1.0, std::max(0.0, c.drop_burst_prob * base));
          if (uni(*brng) < p_gb) g.state = true;
        }
        g.last_t = t;
        g.last_tick = tick_id_;
        g.init = true;
      }
      return g.state;
    }
    std::mt19937* brng = &rng_burst_[si];
    if (reseed_enabled_) {  // key = 通道序号
      reseed_rng.seed(mix32(static_cast<uint32_t>(base_seed_ + 0xB001u),
                            static_cast<uint32_t>(si), static_cast<uint32_t>(bucket)));
      brng = &reseed_rng;
    }
    if (burst_state_[si]) {
      const double p_bg = 1.0 - std::exp(-std::max(dt, 0.0) / std::max(c.drop_burst_mean_s, eps));
      if (uni(*brng) < p_bg) burst_state_[si] = false;
    } else {
      // 进入突发态：基础转移率按 drop_burst_prob 缩放
      const double base = 1.0 - std::exp(-std::max(dt, 0.0) / std::max(c.drop_burst_mean_gap_s, eps));
      const double p_gb = std::min(1.0, std::max(0.0, c.drop_burst_prob * base));
      if (uni(*brng) < p_gb) burst_state_[si] = true;
    }
    return burst_state_[si];
  }

  // 无人机→pt 线段是否被任一真值障碍遮挡；返回遮挡障碍名（无则空）
  std::string firstOccluder(const std::array<double, 3>& pt) const {
    for (const auto& o : obstacles_) {
      if (segHitsObb(drone_pos_, pt, o, occlusion_pad_m_))
        return o.name.empty() ? std::string("obstacle") : o.name;
    }
    return std::string();
  }

  // 生成一拍的观测样本（已加噪）；丢包命中返回 false
  // RNG 兼容：默认配置下噪声流抽样顺序=独立丢包 uniform → 目标 w1,w2,w3 → 每障碍 w1,w2，与 N1 现状逐位一致。
  bool makeSample(size_t si, double t, double drop, double lat, double bias, ObsSample* out) {
    const SensorCfg& c = sensors_[si];
    // 确定性重播种：按 (base_seed, si, bucket) 派生本拍局部生成器；关闭时用持久流（默认等价）
    const long long bucket = std::llround(t * obs_hz_);
    std::mt19937 tick_rng;
    std::mt19937* rng_ptr = &rng_noise_[si];
    if (reseed_enabled_) {
      tick_rng.seed(mix32(static_cast<uint32_t>(base_seed_), static_cast<uint32_t>(si),
                          static_cast<uint32_t>(bucket)));
      rng_ptr = &tick_rng;
    }
    std::mt19937& rng = *rng_ptr;
    std::uniform_real_distribution<double> uni(0.0, 1.0);

    // 与上一拍的仿真时间差（首拍 dt=0），并更新通道时间基准
    const double dt = has_prev_t_[si] ? (t - last_t_[si]) : 0.0;
    last_t_[si] = t;
    has_prev_t_[si] = true;

    out->t_obs = t;
    out->gate_ok = false;
    out->gate_reason = "no_target";
    out->gate_range = out->gate_az_deg = out->gate_el_deg = 0.0;
    out->occluder.clear();
    out->is_false_alarm = false;
    out->drop_source.clear();

    // 1) 独立丢包（噪声流，抽样顺序与现状完全一致）
    const bool indep_hit = (drop > 0.0) && (uni(rng) < std::min(1.0, drop));
    // 2) 成簇丢失状态（独立流；关闭时不抽样）
    const bool burst = burstThisTick(si, c, t, dt, bucket);
    out->burst_state = burst;
    if (indep_hit || (burst && c.drop_burst_prob > 0.0)) {
      out->drop_source = indep_hit ? "independent" : "burst";
      return false;
    }

    std::normal_distribution<double> nx(0.0, c.sigma_xy), ny(0.0, c.sigma_xy), nz(0.0, c.sigma_z);
    const bool model_on = noiseModelOn(c);

    // 目标：几何门控通过 → 遮挡判定 → 产生观测；被遮挡不产生观测（gate_reason=occluded）
    if (target_present_) {
      std::string greason;
      double gr = 0, gaz = 0, gel = 0;
      const bool ok = gatePoint(c, truth_, &greason, &gr, &gaz, &gel);
      out->gate_ok = ok;
      out->gate_reason = greason;
      out->gate_range = gr;
      out->gate_az_deg = gaz;
      out->gate_el_deg = gel;
      if (ok) {
        const std::string occ = occlusion_enabled_ ? firstOccluder(truth_) : std::string();
        if (!occ.empty()) {
          out->gate_ok = false;
          out->gate_reason = "occluded";
          out->occluder = occ;
        } else {
          double ox, oy, oz;
          if (reseed_enabled_) {  // 确定性重播种：目标噪声按 (base_seed_,si,bucket) 纯函数派生
            std::array<double, 3> a;
            arNoiseReseed(si, -1, c, bucket, 3, &a);
            ox = truth_[0] + c.sigma_xy * a[0] + c.bias_m[0];
            oy = truth_[1] + c.sigma_xy * a[1] + c.bias_m[1];
            oz = truth_[2] + c.sigma_z * a[2] + c.bias_m[2];
          } else {
            // 抽样顺序：x,y,z（与现状一致）
            const double vx = nx(rng), vy = ny(rng), vz = nz(rng);
            if (model_on) {
              std::array<double, 3> u{{vx / c.sigma_xy, vy / c.sigma_xy, vz / c.sigma_z}};
              if (c.corr_xy != 0.0) {
                const double r = c.corr_xy;
                u[1] = r * u[0] + std::sqrt(std::max(0.0, 1.0 - r * r)) * u[1];
              }
              applyNoiseModel(si, c, dt, 3, u, &a_prev_target_[si]);
              ox = truth_[0] + c.sigma_xy * u[0] + c.bias_m[0];
              oy = truth_[1] + c.sigma_xy * u[1] + c.bias_m[1];
              oz = truth_[2] + c.sigma_z * u[2] + c.bias_m[2];
            } else {  // 默认路径：raw 正态 + 偏置(默认 0)，与现状逐位等价
              ox = truth_[0] + vx + c.bias_m[0];
              oy = truth_[1] + vy + c.bias_m[1];
              oz = truth_[2] + vz + c.bias_m[2];
            }
          }
          out->has_target = true;
          out->tx = ox + bias;  // 故障离群偏置（沿 +x，沿用现状）
          out->ty = oy;
          out->tz = oz;
          out->tyaw = truth_yaw_;
          out->tseq = ++tseq_[si];
        }
      }
    }

    // 障碍：几何门控通过 → 遮挡判定（排除自身）→ 产生观测
    for (size_t j = 0; j < obstacles_.size(); ++j) {
      const auto& o = obstacles_[j];
      std::string greason;
      double gr = 0, gaz = 0, gel = 0;
      const std::array<double, 3> oc{{o.x, o.y, o.z}};
      if (!gatePoint(c, oc, &greason, &gr, &gaz, &gel)) continue;
      if (occlusion_enabled_) {
        bool blocked = false;
        for (const auto& o2 : obstacles_) {
          if (&o2 == &o) continue;
          if (segHitsObb(drone_pos_, oc, o2, occlusion_pad_m_)) { blocked = true; break; }
        }
        if (blocked) continue;
      }
      double ox, oy;
      if (reseed_enabled_) {  // 确定性重播种：每个障碍用各自独立派生生成器（与门控/其他障碍无关）
        std::array<double, 3> a;
        arNoiseReseed(si, static_cast<int>(j), c, bucket, 2, &a);
        ox = o.x + c.sigma_xy * a[0] + c.bias_m[0];
        oy = o.y + c.sigma_xy * a[1] + c.bias_m[1];
      } else {
        const double vx = nx(rng), vy = ny(rng);
        if (model_on) {
          std::array<double, 3> u{{vx / c.sigma_xy, vy / c.sigma_xy, 0.0}};
          if (c.corr_xy != 0.0) {
            const double r = c.corr_xy;
            u[1] = r * u[0] + std::sqrt(std::max(0.0, 1.0 - r * r)) * u[1];
          }
          applyNoiseModel(si, c, dt, 2, u, &a_prev_obs_[si]);
          ox = o.x + c.sigma_xy * u[0] + c.bias_m[0];
          oy = o.y + c.sigma_xy * u[1] + c.bias_m[1];
        } else {
          ox = o.x + vx + c.bias_m[0];
          oy = o.y + vy + c.bias_m[1];
        }
      }
      ObsSample::Ob ob;
      ob.id = o.name.empty() ? "obstacle" : o.name;
      ob.x = ox; ob.y = oy; ob.z = o.z;
      ob.yaw = o.yaw;
      ob.sx = o.sx; ob.sy = o.sy; ob.sz = o.sz;
      ob.seq = ++oseq_[si];
      out->obs.push_back(ob);
    }

    // 虚警（仅评估日志，绝不进入发布消息；率<=0 时不抽样）
    if (c.false_alarm_rate_hz > 0.0) {
      const double p_fa = std::min(1.0, c.false_alarm_rate_hz / std::max(obs_hz_, 1e-9));
      std::mt19937 fa_rng;
      std::mt19937* fa_ptr = &rng_fa_[si];
      if (reseed_enabled_) {  // 确定性重播种：按 (si,bucket) 派生局部生成器
        fa_rng.seed(mix32(static_cast<uint32_t>(base_seed_ + 0xFA01u),
                          static_cast<uint32_t>(si), static_cast<uint32_t>(bucket)));
        fa_ptr = &fa_rng;
      }
      if (uni(*fa_ptr) < p_fa) {
        out->is_false_alarm = true;
        const std::array<double, 3>& b = target_present_ ? truth_ : drone_pos_;
        out->fa_pos = {{b[0] + 12.0, b[1] + 8.0, b[2] + 4.0}};  // 固定较大偏移，与真值不相邻
      }
    }
    return true;
  }

  air_ground_core::SensorObservation toMsg(size_t si, const ObsSample& s, double now) {
    const SensorCfg& c = sensors_[si];
    air_ground_core::SensorObservation so;
    so.header.stamp = ros::Time(now);          // 该通道观测发布（释放）时刻
    so.header.frame_id = agc::FRAME_WORLD;
    so.sensor_id = c.id;
    if (s.has_target) {
      air_ground_core::TargetObservation to;
      to.header.stamp = ros::Time(s.t_obs);    // 原始观测时刻（延迟不回填）
      to.header.frame_id = agc::FRAME_WORLD;
      to.target_id = target_id_;
      to.sensor_id = c.id;
      to.pose.pose.position.x = s.tx;
      to.pose.pose.position.y = s.ty;
      to.pose.pose.position.z = s.tz;
      const std::array<double, 4> q = agc::quatFromYaw(s.tyaw);
      to.pose.pose.orientation.x = q[0];
      to.pose.pose.orientation.y = q[1];
      to.pose.pose.orientation.z = q[2];
      to.pose.pose.orientation.w = q[3];
      cov3(c.sigma_xy, c.sigma_z, c.corr_xy, to.pose.covariance);
      to.confidence = static_cast<float>(c.confidence);
      to.obs_seq = s.tseq;
      so.targets.push_back(to);
    }
    for (const auto& o : s.obs) {
      air_ground_core::ObstacleObservation oo;
      oo.header.stamp = ros::Time(s.t_obs);
      oo.header.frame_id = agc::FRAME_WORLD;
      oo.obstacle_id = o.id;
      oo.sensor_id = c.id;
      oo.pose.pose.position.x = o.x;
      oo.pose.pose.position.y = o.y;
      oo.pose.pose.position.z = o.z;
      const std::array<double, 4> q = agc::quatFromYaw(o.yaw);
      oo.pose.pose.orientation.x = q[0];
      oo.pose.pose.orientation.y = q[1];
      oo.pose.pose.orientation.z = q[2];
      oo.pose.pose.orientation.w = q[3];
      cov3(c.sigma_xy, c.sigma_z, c.corr_xy, oo.pose.covariance);
      oo.size.x = o.sx; oo.size.y = o.sy; oo.size.z = o.sz;
      oo.confidence = static_cast<float>(c.confidence);
      oo.obs_seq = o.seq;
      so.obstacles.push_back(oo);
    }
    return so;
  }

  // 轮询通道配置与场景文件：变化即重载（热切换）
  void checkReloads() {
    const std::string sf = agc::pgetStr("/agc/perception/sensors_file", "");
    if (sf != sensors_file_) {
      sensors_file_ = sf;
      // 先清空通道状态再重建，避免重复追加
      sensors_.clear();
      rng_noise_.clear();
      rng_burst_.clear();
      rng_fa_.clear();
      a_prev_target_.clear();  // 目标 AR(1) 时间相关状态
      a_prev_obs_.clear();     // 障碍 AR(1) 时间相关状态
      has_prev_t_.clear();
      last_t_.clear();
      burst_state_.clear();   // 成簇丢失状态
      common_groups_.clear(); // 公共模态组（含流与缓存）
      burst_groups_.clear();  // 成簇丢失组（含流与状态）
      ar_reseed_.clear();     // reseed 的 bucket 栅格 AR(1) 状态
      common_group_seq_ = 0;
      pending_.clear();
      tseq_.clear();
      oseq_.clear();
      loadSensors();
      for (const auto& s : sensors_)
        nh_.setParam("/agc/perception/confidence/" + s.id, s.confidence);
      ROS_INFO("air_perception: 通道配置变更 %s，重载通道=%zu", sf.c_str(), sensors_.size());
    }
    const std::string cf = agc::pgetStr("/agc/scenario_file", "");
    if (cf != scenario_file_) {
      scenario_file_ = cf;
      obstacles_ = agc::loadScenario(cf).obstacles;
      ROS_INFO("air_perception: 场景文件变更 %s，重载障碍=%zu", cf.c_str(), obstacles_.size());
    }
  }

  void tick(const ros::TimerEvent&) {
    checkReloads();
    const double t = agc::simNow().toSec();
    if (t <= 0.0) return;  // /clock 未就绪
    ++tick_id_;  // 本拍序号（供公共模态/成簇组缓存判定）

    air_ground_core::AirObservationArray arr;
    arr.header.stamp = ros::Time(t);
    arr.header.frame_id = agc::FRAME_WORLD;

    for (size_t si = 0; si < sensors_.size(); ++si) {
      const SensorCfg& c = sensors_[si];
      double drop = 0.0, lat = 0.0, bias = 0.0;
      faultParams(c, &drop, &lat, &bias);

      // 1) 生成本拍样本并入队（release = t + latency）
      ObsSample s;
      const bool made = makeSample(si, t, drop, lat, bias, &s);
      if (made) {
        pending_[si].push_back(Pending{t + std::max(0.0, lat), s});
        // 落盘样本（含故障参数与 A0 扩展字段，供离线核验与标定）
        agc::JsonObj o;
        o.n("t_sim", t).v("sensor_id", c.id).iv("obs_seq", s.tseq)
            .bv("dropped", false).n("drop_prob", drop).n("latency_s", lat)
            .n("outlier_bias_m", bias).bv("has_target", s.has_target)
            .bv("gate_ok", s.gate_ok).v("gate_reason", s.gate_reason)
            .n("target_range_m", s.gate_range).n("target_az_deg", s.gate_az_deg)
            .n("target_el_deg", s.gate_el_deg);
        if (s.has_target) {
          o.raw("target_truth", seq3(truth_[0], truth_[1], truth_[2]))
              .raw("target_obs", seq3(s.tx, s.ty, s.tz))
              .n("confidence", c.confidence);
        }
        o.iv("n_obstacles", static_cast<long long>(s.obs.size()));
        // A0 扩展字段：送达时刻 / 来源 / 真值 ID（仅评估日志，绝不进入发布消息）
        o.n("t_send_s", t + std::max(0.0, lat)).v("source", SOURCE);
        if (s.has_target) o.v("truth_id", target_id_); else o.nullv("truth_id");
        o.bv("is_false_alarm", s.is_false_alarm);
        if (s.is_false_alarm) o.raw("fa_pos", seq3(s.fa_pos[0], s.fa_pos[1], s.fa_pos[2]));
        o.iv("burst_state", s.burst_state ? 1 : 0).v("drop_source", s.drop_source)
            .v("occluder", s.occluder);
        agc::appendJsonl("perception_log.jsonl", o);
      } else {
        agc::JsonObj o;
        o.n("t_sim", t).v("sensor_id", c.id).iv("obs_seq", 0)
            .bv("dropped", true).n("drop_prob", drop).n("latency_s", lat)
            .n("outlier_bias_m", bias).bv("has_target", false).iv("n_obstacles", 0)
            .n("t_send_s", t + std::max(0.0, lat)).v("source", SOURCE).nullv("truth_id")
            .bv("is_false_alarm", false).iv("burst_state", s.burst_state ? 1 : 0)
            .v("drop_source", s.drop_source).v("occluder", "");
        agc::appendJsonl("perception_log.jsonl", o);
      }

      // 2) 释放到期的待发样本，取最新一条；丢弃更旧的（避免积压）
      bool have = false;
      ObsSample latest;
      while (!pending_[si].empty() && pending_[si].front().release_t <= t) {
        latest = pending_[si].front().s;
        have = true;
        pending_[si].pop_front();
      }
      if (have) arr.sensors.push_back(toMsg(si, latest, t));
    }

    pub_.publish(arr);
  }

  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Timer timer_;

  std::string ns_, target_model_, drone_model_, target_id_, sensors_file_, seed_;
  std::string scenario_file_;  // 场景文件缓存（变化即重载障碍真值）
  double obs_hz_ = 20.0, tol_m_ = 0.25;
  bool occlusion_enabled_ = false;  // A0 遮挡门控总开关
  double occlusion_pad_m_ = 0.0;    // 遮挡判定障碍外扩（m）
  bool reseed_enabled_ = false;     // 确定性重播种模式（默认关，关闭时逐位等价）
  long base_seed_ = 1;
  long long tick_id_ = 0;           // 本拍序号（组缓存判定）
  int common_group_seq_ = 0;        // 公共模态组出现次序（派生种子）

  std::vector<SensorCfg> sensors_;
  std::vector<std::mt19937> rng_noise_;  // 噪声流（种子/抽样顺序与 N1 现状完全相同）
  std::vector<std::mt19937> rng_burst_;  // 成簇丢失流（默认关闭不抽样）
  std::vector<std::mt19937> rng_fa_;     // 虚警流（默认关闭不抽样）
  std::vector<std::array<double, 3>> a_prev_target_;  // 目标 AR(1) 上一拍单位噪声
  std::vector<std::array<double, 3>> a_prev_obs_;     // 障碍 AR(1) 上一拍单位噪声
  std::vector<char> has_prev_t_;               // 是否已有上一拍
  std::vector<double> last_t_;                 // 上一拍仿真时刻
  std::vector<char> burst_state_;              // 每通道突发丢失态（无 drop_group 时使用）
  std::map<std::string, CommonGroup> common_groups_;  // 公共模态组流与缓存
  std::map<std::string, BurstGroup> burst_groups_;    // 成簇丢失组流与状态
  std::map<std::pair<size_t, int>, ArReseedState> ar_reseed_;  // reseed 的 bucket 栅格 AR(1) 状态
  std::vector<std::deque<Pending>> pending_;
  std::vector<unsigned int> tseq_, oseq_;
  std::vector<agc::Obstacle> obstacles_;

  std::array<double, 3> truth_{{0.0, 0.0, 0.0}};
  double truth_yaw_ = 0.0;
  bool target_present_ = false;
  // N1：无人机位姿缓存（门控用），来自同一 /gazebo/model_states
  std::array<double, 3> drone_pos_{{0.0, 0.0, 0.0}};
  std::array<double, 4> drone_quat_{{0.0, 0.0, 0.0, 1.0}};
  bool drone_pose_ok_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "air_perception");
  AirPerception node;
  ros::spin();
  return 0;
}