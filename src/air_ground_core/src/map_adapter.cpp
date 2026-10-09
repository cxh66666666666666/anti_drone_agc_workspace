// map_adapter —— 场景地图适配器（M0 接口契约 v1.1）
//
// C++ 等价实现，行为基准为 scripts/map_adapter.py。发布 /agc/map（OccupancyGrid），
// frame_id=world；header.stamp 用仿真时钟；info.origin 取 origin_x/origin_y、z=0。
// 关键设计：
// 1. 用 agc::loadScenario 的 map 段与 obstacles 构建栅格：障碍 OBB 覆盖的格子置 100，
//    其余置 0（含 2D SAT 判据，0.3 m 细支柱也能覆盖到）。
// 2. 内容哈希：对 data(int8) 字节做 sha1 写入 /agc/map/version_hash（只放参数与日志）。
// 3. 按 /agc/rates/map_hz 且内容变更即发；空图（尺寸<=0 或全未知）不发布并 logerr。
// 4. 有效时写 /agc/map/valid=true；场景读不到时不发布地图并报出原因。
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <air_ground_core/ObstacleTrackArray.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "agc/agc_common.hpp"

namespace {

// 轴对齐方格 [center ± half] 与 yaw 对齐 OBB 是否相交（2D 分离轴）。
// 用面积相交判「覆盖」，避免细长障碍落在格子中心之间被漏判。
bool rectObbOverlap(double cx, double cy, double half, double ox, double oy, double hx,
                    double hy, double yaw) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double ux = c, uy = s;    // OBB 局部 x 轴
  const double vx = -s, vy = c;   // OBB 局部 y 轴
  const double dx = cx - ox, dy = cy - oy;
  const double axes[4][2] = {{1.0, 0.0}, {0.0, 1.0}, {ux, uy}, {vx, vy}};
  for (const auto& ax : axes) {
    const double r_aabb = half * (std::fabs(ax[0]) + std::fabs(ax[1]));
    const double r_obb =
        hx * std::fabs(ax[0] * ux + ax[1] * uy) + hy * std::fabs(ax[0] * vx + ax[1] * vy);
    if (std::fabs(dx * ax[0] + dy * ax[1]) > r_aabb + r_obb) return false;
  }
  return true;
}

struct Grid {
  bool valid = false;
  std::string reason;
  std::vector<int8_t> data;
  int width = 0, height = 0;
  double resolution = 0.0, origin_x = 0.0, origin_y = 0.0;
  bool unknown_is_obstacle = false;
  long long occupied = 0;
  std::string version_hash;
};

// 由场景构建栅格；障碍几何来自 obs（source=scenario 时为场景障碍，air_fusion 时为融合障碍）。
// 无效时 valid=false 并给出 reason
Grid buildGrid(const agc::Scenario& sc, const std::vector<agc::Obstacle>& obs) {
  Grid g;
  const double res = sc.map_resolution;
  const double sx = sc.map_sx, sy = sc.map_sy;
  const double ox = sc.map_ox, oy = sc.map_oy;
  g.unknown_is_obstacle = sc.map_unknown_is_obstacle;
  if (res <= 0.0 || sx <= 0.0 || sy <= 0.0) {
    char buf[160];
    std::snprintf(buf, sizeof(buf), "invalid map geometry: resolution=%g size=(%g,%g)", res, sx,
                  sy);
    g.reason = buf;
    return g;
  }
  const int width = static_cast<int>(std::lround(sx / res));
  const int height = static_cast<int>(std::lround(sy / res));
  if (width <= 0 || height <= 0) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "empty map: width=%d height=%d", width, height);
    g.reason = buf;
    return g;
  }

  // 0=free / 100=occupied；当前场景全部已知，故无 -1 未知格
  std::vector<int8_t> data(static_cast<size_t>(width) * height, 0);
  const double half = res * 0.5;
  for (const auto& o : obs) {
    const double radius = std::hypot(o.hx(), o.hy());
    const int ix0 = static_cast<int>(std::floor((o.x - radius - ox) / res));
    const int ix1 = static_cast<int>(std::ceil((o.x + radius - ox) / res));
    const int iy0 = static_cast<int>(std::floor((o.y - radius - oy) / res));
    const int iy1 = static_cast<int>(std::ceil((o.y + radius - oy) / res));
    for (int iy = std::max(0, iy0); iy < std::min(height, iy1 + 1); ++iy) {
      const double cy = oy + (iy + 0.5) * res;
      for (int ix = std::max(0, ix0); ix < std::min(width, ix1 + 1); ++ix) {
        const double cx = ox + (ix + 0.5) * res;
        if (rectObbOverlap(cx, cy, half, o.x, o.y, o.hx(), o.hy(), o.yaw))
          data[static_cast<size_t>(iy) * width + ix] = 100;
      }
    }
  }

  if (data.empty()) {
    g.reason = "empty data array";
    return g;
  }
  bool all_unknown = true;
  for (int8_t v : data)
    if (v != -1) {
      all_unknown = false;
      break;
    }
  if (all_unknown) {
    g.reason = "all cells unknown";
    return g;
  }

  // 内容哈希：对 data 字节做 sha1（与 Python sha1_bytes(v & 0xFF) 一致）
  std::string bytes;
  bytes.resize(data.size());
  for (size_t i = 0; i < data.size(); ++i) bytes[i] = static_cast<char>(data[i] & 0xFF);

  g.valid = true;
  g.data = std::move(data);
  g.width = width;
  g.height = height;
  g.resolution = res;
  g.origin_x = ox;
  g.origin_y = oy;
  g.occupied = std::count(g.data.begin(), g.data.end(), static_cast<int8_t>(100));
  g.version_hash = agc::sha1Text(bytes);
  return g;
}

}  // namespace

class MapAdapter {
 public:
  MapAdapter() : nh_() {
    scenario_file_ = agc::pgetStr("/agc/scenario_file", "");
    rate_hz_ = agc::pgetD("/agc/rates/map_hz", 2.0);
    obstacle_source_ = agc::pgetStr("/agc/obstacles/source", "scenario");

    pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/agc/map", 1, true);
    load();
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(rate_hz_, 1e-3)),
                             &MapAdapter::tick, this);
    tick(ros::TimerEvent());  // 首次立即发布（内容变更即发的即时性）
  }

 private:
  void load() {
    if (scenario_file_.empty()) {
      ROS_ERROR("map_adapter: /agc/scenario_file 未设置，无法读取场景地图");
      nh_.setParam("/agc/map/valid", false);
      return;
    }
    sc_ = agc::loadScenario(scenario_file_);
    if (!sc_.loaded) {
      ROS_ERROR("map_adapter: 无法读取场景文件 %s，不发布地图", scenario_file_.c_str());
      nh_.setParam("/agc/map/valid", false);
      return;
    }
    if (obstacle_source_ == "air_fusion") {
      // 障碍源=融合：先建空障碍基线图（地图几何仍取场景 map 段），随后由 /agc/obstacles 重建
      ROS_INFO("map_adapter: obstacles/source=air_fusion，障碍格改由 /agc/obstacles 提供");
      applyObstacles({});
      sub_obstacles_ = nh_.subscribe("/agc/obstacles", 1, &MapAdapter::obstacleCb, this);
    } else {
      // 默认：与 v1.1 完全一致的构造期路径（构造性等价，内容哈希不变）
      applyObstacles(sc_.obstacles);
    }
  }

  // 用给定障碍集合重建栅格并立即发布一次（内容哈希用于变更检测）
  void applyObstacles(const std::vector<agc::Obstacle>& obs) {
    Grid g = buildGrid(sc_, obs);
    if (!g.valid) {
      ROS_ERROR("map_adapter: 场景 %s 地图无效：%s", scenario_file_.c_str(), g.reason.c_str());
      nh_.setParam("/agc/map/valid", false);
      return;
    }
    obstacles_ = obs;
    grid_ = std::move(g);
    ROS_INFO("map_adapter: %dx%d @ %.3f m, occupied=%lld, hash=%s (obstacles=%zu source=%s)",
             grid_.width, grid_.height, grid_.resolution, grid_.occupied,
             grid_.version_hash.c_str(), obstacles_.size(), obstacle_source_.c_str());
    tick(ros::TimerEvent());  // 变更即发
  }

  // 障碍集合是否实质相同（防止融合噪声引发的重规划风暴）
  static bool sameObstacles(const std::vector<agc::Obstacle>& a,
                            const std::vector<agc::Obstacle>& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
      const double eps = 1e-3;
      if (a[i].name != b[i].name || std::fabs(a[i].x - b[i].x) > eps ||
          std::fabs(a[i].y - b[i].y) > eps || std::fabs(a[i].yaw - b[i].yaw) > eps ||
          std::fabs(a[i].sx - b[i].sx) > eps || std::fabs(a[i].sy - b[i].sy) > eps)
        return false;
    }
    return true;
  }

  void obstacleCb(const air_ground_core::ObstacleTrackArray::ConstPtr& msg) {
    std::vector<agc::Obstacle> obs;
    obs.reserve(msg->tracks.size());
    for (const auto& t : msg->tracks) {
      agc::Obstacle o;
      o.name = t.obstacle_id;
      o.x = t.pose.position.x;
      o.y = t.pose.position.y;
      o.z = t.pose.position.z;
      o.yaw = agc::yawFromQuat(t.pose.orientation);
      o.sx = t.size.x;
      o.sy = t.size.y;
      o.sz = t.size.z;
      obs.push_back(o);
    }
    if (sameObstacles(obs, obstacles_)) return;  // 未实质变化：不重建
    applyObstacles(obs);
  }

  // 轮询 /agc/scenario_file：变化即重读场景并重建栅格（复用 load/applyObstacles）
  void checkScenarioReload() {
    const std::string f = agc::pgetStr("/agc/scenario_file", "");
    if (f == scenario_file_) return;
    scenario_file_ = f;
    if (scenario_file_.empty()) return;
    const agc::Scenario sc = agc::loadScenario(scenario_file_);
    if (!sc.loaded) {
      ROS_ERROR("map_adapter: 场景文件变更后无法读取 %s，不发布地图", scenario_file_.c_str());
      nh_.setParam("/agc/map/valid", false);
      return;
    }
    sc_ = sc;
    // 障碍源=融合时地图几何重建、障碍仍由 /agc/obstacles 提供（空基线）；
    // 默认源则用新场景 obstacles 重建
    applyObstacles(obstacle_source_ == "air_fusion" ? std::vector<agc::Obstacle>{}
                                                    : sc_.obstacles);
    ROS_INFO("map_adapter: 场景文件变更 %s，已重建栅格", scenario_file_.c_str());
  }

  void tick(const ros::TimerEvent&) {
    checkScenarioReload();
    if (!grid_.valid) {
      ROS_ERROR_THROTTLE(5.0, "map_adapter: 无有效地图，不发布（scenario_file=%s）",
                         scenario_file_.c_str());
      return;
    }
    const Grid& g = grid_;
    nav_msgs::OccupancyGrid msg;
    msg.header.stamp = agc::simNow();
    msg.header.frame_id = agc::FRAME_WORLD;
    msg.info.resolution = g.resolution;
    msg.info.width = static_cast<uint32_t>(g.width);
    msg.info.height = static_cast<uint32_t>(g.height);
    msg.info.origin.position.x = g.origin_x;
    msg.info.origin.position.y = g.origin_y;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;
    msg.data = g.data;
    pub_.publish(msg);

    if (g.version_hash != last_hash_) {
      const bool first = !has_last_hash_;
      last_hash_ = g.version_hash;
      has_last_hash_ = true;
      nh_.setParam("/agc/map/version_hash", g.version_hash);
      ROS_INFO("map published: %dx%d hash=%s occupied=%lld (reason=%s)", g.width, g.height,
               g.version_hash.c_str(), g.occupied, first ? "first" : "content_change");
    }
    nh_.setParam("/agc/map/valid", true);
  }

  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Timer timer_;
  ros::Subscriber sub_obstacles_;

  std::string scenario_file_;
  std::string obstacle_source_ = "scenario";
  double rate_hz_;
  agc::Scenario sc_;
  std::vector<agc::Obstacle> obstacles_;
  Grid grid_;
  std::string last_hash_;
  bool has_last_hash_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_adapter");
  MapAdapter node;
  ros::spin();
  return 0;
}