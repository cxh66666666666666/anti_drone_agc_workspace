// air_fusion —— 目标与障碍多源融合（M0 接口契约 v1.2 第八节，C++ 实时链路）
//
// 输入 /agc/air/obs（air_perception 的多通道观测），输出：
//   * /agc/targets   (TargetTrackArray，source=air_fusion) —— 被 M3 链路消费
//   * /agc/obstacles (ObstacleTrackArray，source=air_fusion) —— 障碍源为 air_fusion 时被规划消费
//
// 门控（契约 8.2）：新鲜度（age ≤ fusion/target_max_age_s，默认 0.35 s，严于消费者 0.5 s）、
// 置信度（≥ fusion/confidence_min）、空间一致性（与参考估计距离 > fusion/gate_m 判离群）。
// 融合：仅对 inlier 做逆方差加权；因各通道同源、误差相关，协方差以 correlated_pad_m 设下界，
// 不按 1/N 缩小，也不跨源取最大。无有效观测发空数组，绝不刷新旧观测时间。
// 障碍：按 obstacle_id 关联 inlier，取并集 + 保守外扩，不做 yaw 环绕下的加权平均。
// 日志：fusion_log.jsonl（输入观测、门控决策、离群标记、融合输出、拒收原因）。
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <air_ground_core/AirObservationArray.h>
#include <air_ground_core/TargetTrack.h>
#include <air_ground_core/TargetTrackArray.h>
#include <air_ground_core/ObstacleTrack.h>
#include <air_ground_core/ObstacleTrackArray.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "agc/agc_common.hpp"

namespace {

const char* const SOURCE = "air_fusion";

struct SensorInfo {
  double sigma_xy = 0.1;
  double confidence = 1.0;
};

void diagCov3(double sxy, double sz, boost::array<double, 36>& c) {
  c.assign(0.0);
  c[0] = c[7] = sxy * sxy;
  c[14] = sz * sz;
}

}  // namespace

class AirFusion {
 public:
  AirFusion() : nh_() {
    target_id_ = agc::pgetStr("/agc/target/id", "intruder_uav");
    hz_ = agc::cfgD("fusion/hz", 10.0);
    target_max_age_ = agc::cfgD("fusion/target_max_age_s", 0.35);
    obs_max_age_ = agc::cfgD("fusion/obs_max_age_s", 0.35);
    gate_m_ = agc::cfgD("fusion/gate_m", 1.5);
    confidence_min_ = agc::cfgD("fusion/confidence_min", 0.5);
    min_sources_ = agc::cfgI("fusion/min_target_sources", 1);
    obstacle_match_m_ = agc::cfgD("fusion/obstacle_match_m", 1.0);
    obstacle_pad_m_ = agc::cfgD("fusion/obstacle_pad_m", 0.05);
    correlated_pad_m_ = agc::cfgD("fusion/correlated_pad_m", 0.05);
    cold_start_min_agree_ = agc::cfgI("fusion/cold_start_min_agree", 2);
    tol_m_ = agc::cfgD("calibration/confidence_tolerance_m", 0.25);
    ref_hold_s_ = agc::cfgD("fusion/reference_hold_s", 1.0);
    target_source_ = agc::pgetStr("/agc/target/source", "gt_adapter");
    obstacle_source_ = agc::pgetStr("/agc/obstacles/source", "scenario");

    loadSensors();
    nh_.setParam("/agc/fusion/source", std::string(SOURCE));
    nh_.setParam("/agc/fusion/target_max_age_s", target_max_age_);
    nh_.setParam("/agc/fusion/calibration_tolerance_m", tol_m_);

    // 仅在对应源被选中时发布，保证唯一发布者
    if (target_source_ == SOURCE)
      pub_targets_ = nh_.advertise<air_ground_core::TargetTrackArray>("/agc/targets", 10);
    else
      ROS_WARN("air_fusion: target/source=%s，本节点不发布 /agc/targets", target_source_.c_str());
    if (obstacle_source_ == SOURCE)
      pub_obstacles_ = nh_.advertise<air_ground_core::ObstacleTrackArray>("/agc/obstacles", 10);
    else
      ROS_WARN("air_fusion: obstacles/source=%s，本节点不发布 /agc/obstacles", obstacle_source_.c_str());

    sub_ = nh_.subscribe("/agc/air/obs", 1, &AirFusion::onObs, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(hz_, 1e-3)), &AirFusion::tick, this);
    ROS_INFO("air_fusion 就绪：%zu 通道 hz=%.1f age<=%.2f gate=%.2f conf_min=%.2f（source=%s/%s）",
             sensors_.size(), hz_, target_max_age_, gate_m_, confidence_min_,
             target_source_.c_str(), obstacle_source_.c_str());
  }

 private:
  void loadSensors() {
    const std::string file = agc::pgetStr("/agc/perception/sensors_file", "");
    if (file.empty()) {
      ROS_ERROR("air_fusion: /agc/perception/sensors_file 未设置");
      return;
    }
    YAML::Node root;
    try {
      root = YAML::LoadFile(file);
    } catch (const std::exception& e) {
      ROS_ERROR("air_fusion: 读取 %s 失败: %s", file.c_str(), e.what());
      return;
    }
    const YAML::Node list = root["sensors"];
    if (!list || !list.IsSequence()) return;
    const double tol = tol_m_;
    for (const auto& n : list) {
      if (!n["id"]) continue;
      SensorInfo si;
      if (n["sigma_xy_m"]) si.sigma_xy = n["sigma_xy_m"].as<double>();
      const double s2 = si.sigma_xy * si.sigma_xy;
      si.confidence = (s2 > 1e-12) ? (1.0 - std::exp(-(tol * tol) / (2.0 * s2))) : 1.0;
      sensors_[n["id"].as<std::string>()] = si;
      order_.push_back(n["id"].as<std::string>());
    }
  }

  // ------------------------------------------------------------ 输入
  void onObs(const air_ground_core::AirObservationArray::ConstPtr& msg) {
    for (const auto& so : msg->sensors) {
      SensorState& st = state_[so.sensor_id];
      for (const auto& t : so.targets) st.target = t, st.has_target = true;
      for (const auto& o : so.obstacles) st.obstacles[o.obstacle_id] = o;
    }
  }

  double sigmaOf(const std::string& sensor) const {
    const auto it = sensors_.find(sensor);
    return it == sensors_.end() ? 0.15 : it->second.sigma_xy;
  }

  // 2D 径向覆盖 = 1 - exp(-tol^2/(2 sigma^2))
  double coverage(double sigma) const {
    const double s2 = sigma * sigma;
    if (s2 <= 1e-12) return 1.0;
    return std::min(1.0, std::max(0.0, 1.0 - std::exp(-(tol_m_ * tol_m_) / (2.0 * s2))));
  }

  // ------------------------------------------------------------ 目标融合
  void fuseTargets(double now) {
    struct Cand {
      std::string sensor;
      air_ground_core::TargetObservation o;
      double age = 0.0;
      double sigma = 0.1;
    };
    std::vector<Cand> cands;
    agc::JsonObj log;
    log.n("t_sim", now).v("kind", "target");
    std::vector<std::string> inputs;
    for (const auto& kv : state_) {
      if (!kv.second.has_target) continue;
      const auto& o = kv.second.target;
      const double age = now - o.header.stamp.toSec();
      const double sigma = sigmaOf(kv.first);
      std::string verdict = "inlier";
      if (age > target_max_age_) verdict = "stale";
      else if (o.confidence < confidence_min_) verdict = "low_conf";
      agc::JsonObj e;
      e.v("sensor", kv.first).n("age_s", age).n("confidence", o.confidence)
          .raw("pos", agc::JsonObj::arrNum({o.pose.pose.position.x, o.pose.pose.position.y}))
          .v("verdict", verdict);
      inputs.push_back(e.dump());
      if (verdict == "inlier")
        cands.push_back(Cand{kv.first, o, age, sigma});
    }
    log.raw("inputs", agc::JsonObj::arr(inputs));

    if (static_cast<int>(cands.size()) < std::max(1, min_sources_)) {
      publishEmptyTargets(now);
      log.v("result", "no_target").iv("n_inliers", 0).iv("n_candidates",
                                                         static_cast<long long>(cands.size()))
          .bv("cold_start", false).v("ref_source", "no_target");
      agc::appendJsonl("fusion_log.jsonl", log);
      setDiag(0, 0, static_cast<int>(state_.size()));
      has_last_fused_ = false;
      return;
    }

    // 参考点：优先用最近融合估计（保证注入切换期的门控稳定）；
    // 冷启动（无历史）改用鲁棒关联：按 gate_m 单链聚类取最大簇作参考，
    // 候选≥2 且最大簇成员数 < cold_start_min_agree 时判冷启动歧义，不发高置信目标（N4）。
    bool cold_start = false;
    std::string ref_source = "history";
    std::vector<double> cluster_sizes;
    double rx, ry;
    bool have_ref = false;
    if (has_last_fused_ && (now - last_fused_sim_) <= ref_hold_s_) {
      rx = last_fused_x_; ry = last_fused_y_; have_ref = true;
    } else {
      cold_start = true;
      const int n = static_cast<int>(cands.size());
      std::vector<int> label(n, -1);  // 单链聚类标签
      int nc = 0;
      for (int i = 0; i < n; ++i) {
        if (label[i] != -1) continue;
        label[i] = nc;
        for (int j = i + 1; j < n; ++j) {
          if (label[j] != -1) continue;
          const double d = std::hypot(cands[i].o.pose.pose.position.x - cands[j].o.pose.pose.position.x,
                                      cands[i].o.pose.pose.position.y - cands[j].o.pose.pose.position.y);
          if (d <= gate_m_) label[j] = nc;
        }
        ++nc;
      }
      int best_c = -1, best_sz = 0;
      for (int c = 0; c < nc; ++c) {
        const int sz = static_cast<int>(std::count(label.begin(), label.end(), c));
        cluster_sizes.push_back(static_cast<double>(sz));
        if (sz > best_sz) { best_sz = sz; best_c = c; }
      }
      if (n >= 2 && best_sz < std::max(2, cold_start_min_agree_)) {
        // 冷启动歧义：不把无法判定的冲突当作高置信输出
        publishEmptyTargets(now);
        log.v("result", "cold_start_ambiguous").bv("cold_start", true).v("ref_source", "ambiguous")
            .raw("cluster_sizes", agc::JsonObj::arrNum(cluster_sizes));
        agc::appendJsonl("fusion_log.jsonl", log);
        setDiag(0, n, static_cast<int>(state_.size()));
        has_last_fused_ = false;
        return;
      }
      double sx = 0, sy = 0;
      int cnt = 0;
      for (int i = 0; i < n; ++i) {
        if (label[i] != best_c) continue;
        sx += cands[i].o.pose.pose.position.x; sy += cands[i].o.pose.pose.position.y; ++cnt;
      }
      rx = sx / std::max(1, cnt); ry = sy / std::max(1, cnt);
      have_ref = true;
      ref_source = (n >= 2) ? "cluster" : "single";
    }

    // 空间一致性门控
    std::vector<Cand> inliers;
    std::vector<std::string> outliers;
    for (const auto& c : cands) {
      const double d = std::hypot(c.o.pose.pose.position.x - rx, c.o.pose.pose.position.y - ry);
      if (d <= gate_m_) inliers.push_back(c);
      else outliers.push_back(c.sensor);
    }
    if (static_cast<int>(inliers.size()) < std::max(1, min_sources_)) {
      publishEmptyTargets(now);
      log.v("result", "gate_rejected_all").iv("n_inliers", 0);
      log.raw("outliers", agc::JsonObj::arrStr(outliers)).n("ref_x", rx).n("ref_y", ry);
      log.bv("cold_start", cold_start).v("ref_source", ref_source);
      if (!cluster_sizes.empty()) log.raw("cluster_sizes", agc::JsonObj::arrNum(cluster_sizes));
      agc::appendJsonl("fusion_log.jsonl", log);
      setDiag(0, static_cast<int>(outliers.size()), static_cast<int>(state_.size()));
      has_last_fused_ = false;
      return;
    }

    // 逆方差加权 + 相关噪声协方差下界
    double sw = 0, swx = 0, swy = 0, swz = 0, best_stamp = 0;
    const Cand* bestc = &inliers[0];
    for (const auto& c : inliers) {
      const double w = 1.0 / std::max(1e-6, c.sigma * c.sigma);
      sw += w;
      swx += w * c.o.pose.pose.position.x;
      swy += w * c.o.pose.pose.position.y;
      swz += w * c.o.pose.pose.position.z;
      best_stamp = std::max(best_stamp, c.o.header.stamp.toSec());
      if (c.o.confidence > bestc->o.confidence) bestc = &c;
    }
    const double fx = swx / sw, fy = swy / sw, fz = swz / sw;
    const double sigma_fused = std::max(std::sqrt(1.0 / sw), correlated_pad_m_);
    const double conf_fused = coverage(sigma_fused);

    // 速度：对融合位置做短时差分（观测消息不含 twist）
    std::array<double, 3> v{{0.0, 0.0, 0.0}};
    if (has_last_fused_ && (now - last_fused_sim_) > 1e-3) {
      const double dt = now - last_fused_sim_;
      v = {{(fx - last_fused_x_) / dt, (fy - last_fused_y_) / dt, (fz - last_fused_z_) / dt}};
    }

    air_ground_core::TargetTrackArray arr;
    arr.header.stamp = ros::Time(now);
    arr.header.frame_id = agc::FRAME_WORLD;
    air_ground_core::TargetTrack tr;
    tr.header.stamp = ros::Time(best_stamp);  // 最新真实观测时刻
    tr.header.frame_id = agc::FRAME_WORLD;
    tr.target_id = target_id_;
    tr.pose.pose.position.x = fx;
    tr.pose.pose.position.y = fy;
    tr.pose.pose.position.z = fz;
    const std::array<double, 4> q = agc::quatFromYaw(agc::yawFromQuat(bestc->o.pose.pose.orientation));
    tr.pose.pose.orientation.x = q[0];
    tr.pose.pose.orientation.y = q[1];
    tr.pose.pose.orientation.z = q[2];
    tr.pose.pose.orientation.w = q[3];
    diagCov3(sigma_fused, std::max(sigma_fused, correlated_pad_m_), tr.pose.covariance);
    tr.twist.twist.linear.x = v[0];
    tr.twist.twist.linear.y = v[1];
    tr.twist.twist.linear.z = v[2];
    diagCov3(0.05, 0.05, tr.twist.covariance);
    tr.confidence = static_cast<float>(conf_fused);
    tr.source = SOURCE;
    arr.tracks.push_back(tr);
    if (pub_targets_) pub_targets_.publish(arr);

    last_fused_sim_ = now;
    last_fused_x_ = fx; last_fused_y_ = fy; last_fused_z_ = fz;
    has_last_fused_ = true;

    log.v("result", "fused").iv("n_inliers", static_cast<long long>(inliers.size()));
    log.raw("outliers", agc::JsonObj::arrStr(outliers));
    log.n("ref_x", rx).n("ref_y", ry);
    log.bv("cold_start", cold_start).v("ref_source", ref_source);
    if (!cluster_sizes.empty()) log.raw("cluster_sizes", agc::JsonObj::arrNum(cluster_sizes));
    log.raw("fused_pos", agc::JsonObj::arrNum({fx, fy, fz}))
        .n("sigma_fused", sigma_fused).n("confidence_fused", conf_fused)
        .n("stamp_s", best_stamp);
    std::vector<std::string> ins;
    for (const auto& c : inliers) ins.push_back(c.sensor);
    log.raw("inliers", agc::JsonObj::arrStr(ins));
    agc::appendJsonl("fusion_log.jsonl", log);
    setDiag(static_cast<int>(inliers.size()), static_cast<int>(outliers.size()),
            static_cast<int>(state_.size()));
  }

  void publishEmptyTargets(double now) {
    if (!pub_targets_) return;
    air_ground_core::TargetTrackArray arr;
    arr.header.stamp = ros::Time(now);
    arr.header.frame_id = agc::FRAME_WORLD;
    pub_targets_.publish(arr);  // 空数组：不复制旧姿态、不刷新旧 stamp
  }

  // ------------------------------------------------------------ 障碍融合
  void fuseObstacles(double now) {
    if (!pub_obstacles_) return;
    std::map<std::string, std::vector<const air_ground_core::ObstacleObservation*>> by_id;
    for (const auto& kv : state_) {
      for (auto& okv : kv.second.obstacles) {
        auto& o = okv.second;
        const double age = now - o.header.stamp.toSec();
        if (age > obs_max_age_ || o.confidence < confidence_min_) continue;
        by_id[okv.first].push_back(&o);
      }
    }

    air_ground_core::ObstacleTrackArray arr;
    arr.header.stamp = ros::Time(now);
    arr.header.frame_id = agc::FRAME_WORLD;
    for (const auto& kv : by_id) {
      if (kv.second.empty()) continue;
      double sw = 0, swx = 0, swy = 0, swz = 0, best_stamp = 0, best_conf = -1, best_yaw = 0;
      double sx = 0, sy = 0, sz = 0;
      for (const auto* o : kv.second) {
        const std::string sensor = o->sensor_id;
        const double sigma = sigmaOf(sensor);
        const double w = 1.0 / std::max(1e-6, sigma * sigma);
        sw += w;
        swx += w * o->pose.pose.position.x;
        swy += w * o->pose.pose.position.y;
        swz += w * o->pose.pose.position.z;
        best_stamp = std::max(best_stamp, o->header.stamp.toSec());
        sx = std::max(sx, static_cast<double>(o->size.x));
        sy = std::max(sy, static_cast<double>(o->size.y));
        sz = std::max(sz, static_cast<double>(o->size.z));
        if (o->confidence > best_conf) {
          best_conf = o->confidence;
          best_yaw = agc::yawFromQuat(o->pose.pose.orientation);
        }
      }
      air_ground_core::ObstacleTrack t;
      t.header.stamp = ros::Time(best_stamp);
      t.header.frame_id = agc::FRAME_WORLD;
      t.obstacle_id = kv.first;
      t.pose.position.x = swx / sw;
      t.pose.position.y = swy / sw;
      t.pose.position.z = swz / sw;
      const std::array<double, 4> q = agc::quatFromYaw(best_yaw);
      t.pose.orientation.x = q[0]; t.pose.orientation.y = q[1];
      t.pose.orientation.z = q[2]; t.pose.orientation.w = q[3];
      // 并集 + 保守外扩（不做 yaw 环绕加权平均）
      t.size.x = sx + obstacle_pad_m_;
      t.size.y = sy + obstacle_pad_m_;
      t.size.z = sz;
      const double sigma_fused = std::max(std::sqrt(1.0 / std::max(sw, 1e-9)), correlated_pad_m_);
      t.confidence = static_cast<float>(coverage(sigma_fused));
      t.source = SOURCE;
      arr.tracks.push_back(t);
    }
    pub_obstacles_.publish(arr);
    agc::JsonObj log;
    log.n("t_sim", now).v("kind", "obstacle").iv("n_tracks",
                                                  static_cast<long long>(arr.tracks.size()));
    agc::appendJsonl("fusion_log.jsonl", log);
  }

  void setDiag(int n_in, int n_out, int n_sensors) {
    nh_.setParam("/agc/fusion/n_input_sensors", n_sensors);
    nh_.setParam("/agc/fusion/last_n_inliers", n_in);
    nh_.setParam("/agc/fusion/last_n_outliers", n_out);
  }

  void tick(const ros::TimerEvent&) {
    const double now = agc::simNow().toSec();
    if (now <= 0.0) return;
    if (pub_targets_) fuseTargets(now);
    fuseObstacles(now);
  }

  ros::NodeHandle nh_;
  ros::Publisher pub_targets_, pub_obstacles_;
  ros::Subscriber sub_;
  ros::Timer timer_;

  struct SensorState {
    bool has_target = false;
    air_ground_core::TargetObservation target;
    std::map<std::string, air_ground_core::ObstacleObservation> obstacles;
  };
  std::map<std::string, SensorState> state_;
  std::map<std::string, SensorInfo> sensors_;
  std::vector<std::string> order_;

  std::string target_id_, target_source_, obstacle_source_;
  double hz_ = 10.0, target_max_age_ = 0.35, obs_max_age_ = 0.35, gate_m_ = 1.5;
  double confidence_min_ = 0.5, obstacle_pad_m_ = 0.05, correlated_pad_m_ = 0.05;
  double tol_m_ = 0.25, ref_hold_s_ = 1.0, obstacle_match_m_ = 1.0;
  int min_sources_ = 1;
  int cold_start_min_agree_ = 2;

  bool has_last_fused_ = false;
  double last_fused_sim_ = 0, last_fused_x_ = 0, last_fused_y_ = 0, last_fused_z_ = 0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "air_fusion");
  AirFusion node;
  ros::spin();
  return 0;
}