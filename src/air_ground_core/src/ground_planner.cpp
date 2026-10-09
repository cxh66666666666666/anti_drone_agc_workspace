// ground_planner —— 地面路径规划（M0 接口契约 v1.1，C++ 实现；基准 scripts/ground_planner.py）
//
// - 发布 /agc/ground/path（nav_msgs/Path，frame_id=world，非空才发）。
// - 输入：/agc/ground/goal、/agc/map、/agc/ground/odom。
// - 膨胀量 dilate_m = max(planner.inflation_m, clearance.protection_m)：
//   它保证路径中心与障碍表面的距离 ≥ protection_m（保护距离），
//   而不是「机身半径 + 一点余量」这种不说明含义的叠加。
// - 纯 C++ 8 邻域 A* + 视线剪枝；重规划触发：首次 / goal 位移 / 地图版本 /
//   路径不可行 / 周期；发布按 rates.path_hz 重发当前路径。
// - 连续失败达 planner.plan_retry_max 置 /agc/planner/unreachable=true，成功置回 false。
#include "agc/agc_common.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <air_ground_core/ObstacleTrackArray.h>

namespace {

const double SQRT2 = std::sqrt(2.0);

// 8 邻域：(dx, dy, cost)
struct Neighbor { int dx, dy; double cost; };
const Neighbor NEIGHBORS[8] = {
    {1, 0, 1.0}, {-1, 0, 1.0}, {0, 1, 1.0}, {0, -1, 1.0},
    {1, 1, SQRT2}, {1, -1, SQRT2}, {-1, 1, SQRT2}, {-1, -1, SQRT2}};

// 与 agc::Obstacle 等价的只读视图：让下列纯函数不依赖 ROS，便于 /tmp 单测。
struct ObsView { double x, y, yaw, hx, hy; };

// 点到 yaw 对齐矩形的平面距离；点在内部返回负值（与 agc::pointObbClearance 同式）。
double obbClearance(double px, double py, const ObsView& o) {
  const double dx = px - o.x, dy = py - o.y;
  const double c = std::cos(-o.yaw), s = std::sin(-o.yaw);
  const double lx = c * dx - s * dy;
  const double ly = s * dx + c * dy;
  const double ex = std::fabs(lx) - o.hx;
  const double ey = std::fabs(ly) - o.hy;
  if (ex > 0.0 && ey > 0.0) return std::sqrt(ex * ex + ey * ey);
  return std::max(ex, ey);
}

using Grid = std::vector<std::vector<bool>>;  // blocked[ix][iy]

// ================================================================ 纯函数（可单测）
// 由障碍 OBB 构造膨胀后的阻塞栅格：cell 中心到障碍表面 < dilate_m 即阻塞。
Grid buildGrid(const std::vector<ObsView>& obs, double res, int nx, int ny,
               double ox, double oy, double dilate_m) {
  Grid blocked(nx, std::vector<bool>(ny, false));
  for (int ix = 0; ix < nx; ++ix) {
    const double cx = ox + (ix + 0.5) * res;
    for (int iy = 0; iy < ny; ++iy) {
      const double cy = oy + (iy + 0.5) * res;
      for (const auto& ob : obs) {
        if (obbClearance(cx, cy, ob) < dilate_m) {
          blocked[ix][iy] = true;
          break;
        }
      }
    }
  }
  return blocked;
}

// 8 邻域 A*；blocked[ix][iy]=True 为障碍。成功返回 true 并写出 cell 列表。
bool astar(const Grid& blocked, const std::pair<int, int>& start,
           const std::pair<int, int>& goal, std::vector<std::pair<int, int>>* out) {
  const int nx = static_cast<int>(blocked.size());
  const int ny = nx > 0 ? static_cast<int>(blocked[0].size()) : 0;
  auto inb = [&](const std::pair<int, int>& c) {
    return c.first >= 0 && c.first < nx && c.second >= 0 && c.second < ny;
  };
  if (!inb(start) || !inb(goal)) return false;
  if (blocked[goal.first][goal.second]) return false;  // 终点落在障碍内：不可达

  const int n = nx * ny;
  auto idx = [&](int x, int y) { return y * nx + x; };
  auto h = [&](const std::pair<int, int>& c) {
    const int dx = std::abs(c.first - goal.first);
    const int dy = std::abs(c.second - goal.second);
    return (dx + dy) + (SQRT2 - 2.0) * std::min(dx, dy);
  };

  if (start == goal) { out->assign(1, start); return true; }

  struct Node { double f, g; int x, y; };
  struct Cmp {  // priority_queue 默认大顶堆：这里让 f 小者优先
    bool operator()(const Node& a, const Node& b) const {
      if (a.f != b.f) return a.f > b.f;
      if (a.g != b.g) return a.g > b.g;
      if (a.x != b.x) return a.x > b.x;
      return a.y > b.y;
    }
  };
  std::priority_queue<Node, std::vector<Node>, Cmp> open;
  std::vector<char> closed(n, 0);
  std::vector<double> g(n, agc::INF);
  std::vector<int> came(n, -1);

  g[idx(start.first, start.second)] = 0.0;
  open.push(Node{h(start), 0.0, start.first, start.second});

  while (!open.empty()) {
    const Node cur = open.top();
    open.pop();
    const int ci = idx(cur.x, cur.y);
    if (closed[ci]) continue;
    closed[ci] = 1;
    if (cur.x == goal.first && cur.y == goal.second) {
      std::vector<std::pair<int, int>> path;
      int c = ci;
      while (c != -1) {
        path.emplace_back(c % nx, c / nx);
        c = came[c];
      }
      std::reverse(path.begin(), path.end());
      *out = path;
      return true;
    }
    for (const auto& nb : NEIGHBORS) {
      const int jx = cur.x + nb.dx, jy = cur.y + nb.dy;
      if (jx < 0 || jx >= nx || jy < 0 || jy >= ny) continue;
      const int ji = idx(jx, jy);
      if (blocked[jx][jy] || closed[ji]) continue;
      // 对角移动不允许穿角
      if (nb.dx != 0 && nb.dy != 0 &&
          (blocked[cur.x + nb.dx][cur.y] || blocked[cur.x][cur.y + nb.dy])) {
        continue;
      }
      const double ng = cur.g + nb.cost;
      if (ng < g[ji]) {
        g[ji] = ng;
        came[ji] = ci;
        open.push(Node{ng + h({jx, jy}), ng, jx, jy});
      }
    }
  }
  return false;
}

// 栅格线段 p0->p1 是否穿过障碍（采样判定）；越界视为穿过。
bool lineBlocked(const Grid& blocked, const std::pair<int, int>& p0,
                 const std::pair<int, int>& p1) {
  const int nx = static_cast<int>(blocked.size());
  const int ny = nx > 0 ? static_cast<int>(blocked[0].size()) : 0;
  const int steps =
      static_cast<int>(std::max(std::abs(p1.first - p0.first),
                                std::abs(p1.second - p0.second)) * 2) + 1;
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    const int ix = static_cast<int>(std::lround(p0.first + t * (p1.first - p0.first)));
    const int iy = static_cast<int>(std::lround(p0.second + t * (p1.second - p0.second)));
    if (ix < 0 || ix >= nx || iy < 0 || iy >= ny) return true;
    if (blocked[ix][iy]) return true;
  }
  return false;
}

// 视线剪枝：用尽量远的可见点串起路径。
std::vector<std::pair<int, int>> pruneLos(const std::vector<std::pair<int, int>>& cells,
                                          const Grid& blocked) {
  if (cells.size() <= 2) return cells;
  std::vector<std::pair<int, int>> out;
  out.push_back(cells.front());
  size_t i = 0;
  while (i + 1 < cells.size()) {
    size_t j = cells.size() - 1;
    while (j > i + 1 && lineBlocked(blocked, cells[i], cells[j])) --j;
    out.push_back(cells[j]);
    i = j;
  }
  return out;
}

}  // namespace

// ================================================================ 节点
// N3 目标格可行化：目标格被膨胀图阻塞时，在切比雪夫半径 R 内按欧氏距离取最近的可行格。
// 「可行」由 buildBlocked 定义为「格中心净空 ≥ dilate_m(=max(inflation, protection))」，
// 因此不会降低保护阈值。返回 false 表示半径内无任何可行格（确实无路）。
bool nearestFeasibleCell(const Grid& blocked, const std::pair<int, int>& goal_c, int R,
                         std::pair<int, int>* out) {
  const int nx = static_cast<int>(blocked.size());
  const int ny = nx > 0 ? static_cast<int>(blocked[0].size()) : 0;
  bool found = false;
  double best = agc::INF;
  for (int dx = -R; dx <= R; ++dx) {
    for (int dy = -R; dy <= R; ++dy) {
      const int ix = goal_c.first + dx, iy = goal_c.second + dy;
      if (ix < 0 || ix >= nx || iy < 0 || iy >= ny) continue;
      if (blocked[ix][iy]) continue;
      const double d2 = static_cast<double>(dx * dx + dy * dy);
      if (d2 < best) { best = d2; *out = {ix, iy}; found = true; }
    }
  }
  return found;
}

class GroundPlanner {
 public:
  GroundPlanner() : nh_(), pnh_("~") {
    replan_disp_ = agc::cfgD("planner/replan_displacement_m", 1.0);
    replan_period_ = agc::cfgD("planner/replan_period_s", 2.0);
    inflation_ = agc::cfgD("planner/inflation_m", 0.30);
    protection_ = agc::cfgD("clearance/protection_m", 0.80);
    retry_max_ = agc::cfgI("planner/plan_retry_max", agc::cfgI("limits/plan_retry_max", 3));
    path_hz_ = agc::cfgD("rates/path_hz", 2.0);
    p_res_ = agc::cfgD("planner/grid_resolution_m", 0.5);
    p_sx_ = agc::cfgD("planner/grid_size_x_m", 40.0);
    p_sy_ = agc::cfgD("planner/grid_size_y_m", 40.0);
    p_ox_ = agc::cfgD("planner/grid_origin_x", -20.0);
    p_oy_ = agc::cfgD("planner/grid_origin_y", -20.0);
    unknown_blocked_ = agc::cfgB("planner/unknown_is_obstacle", true);
    dilate_m_ = std::max(inflation_, protection_);
    goal_adjust_max_m_ = agc::cfgD("planner/goal_adjust_max_m", 1.0);

    const agc::Scenario sc = agc::loadScenario(agc::cfgS("scenario_file", ""));
    // M4：障碍源在 run 前固定。source=air_fusion 时膨胀改用 /agc/obstacles 融合障碍；
    // source=scenario（默认）时与 v1.1 完全一致（构造性等价）。
    obstacle_fusion_ = (agc::cfgS("obstacles/source", "scenario") == "air_fusion");
    if (obstacle_fusion_) {
      sub_obstacles_ = nh_.subscribe("/agc/obstacles", 1, &GroundPlanner::obstacleCb, this);
      ROS_INFO("ground_planner: obstacles/source=air_fusion，膨胀改用 /agc/obstacles 融合障碍");
    } else {
      for (const auto& o : sc.obstacles) {
        obstacles_.push_back(ObsView{o.x, o.y, o.yaw, o.hx(), o.hy()});
      }
    }

    sub_goal_ = nh_.subscribe("/agc/ground/goal", 1, &GroundPlanner::goalCb, this);
    sub_map_ = nh_.subscribe("/agc/map", 1, &GroundPlanner::mapCb, this);
    sub_odom_ = nh_.subscribe("/agc/ground/odom", 1, &GroundPlanner::odomCb, this);
    pub_ = nh_.advertise<nav_msgs::Path>("/agc/ground/path", 1);
    // 10 Hz 检查触发，高于 path_hz 以支持重规划即发
    timer_ = nh_.createTimer(ros::Duration(0.1), &GroundPlanner::tick, this);
    ROS_INFO("ground_planner 就绪：dilate_m=%.2f (max(inflation=%.2f, protection=%.2f))",
             dilate_m_, inflation_, protection_);
  }

 private:
  // ------------------------------------------------------------ 回调
  void goalCb(const geometry_msgs::PoseStamped::ConstPtr& m) { goal_ = m; has_goal_ = true; }
  void mapCb(const nav_msgs::OccupancyGrid::ConstPtr& m) { map_ = m; has_map_ = true; }
  void odomCb(const nav_msgs::Odometry::ConstPtr& m) { odom_ = m; has_odom_ = true; }

  // ------------------------------------------------------------ 输入有效性
  bool inputsValid(std::string* reason) const {
    if (!has_map_) { *reason = "no map"; return false; }
    if (!agc::cfgB("map/valid", false)) { *reason = "map invalid"; return false; }
    if (!has_goal_) { *reason = "no goal"; return false; }
    if (!has_odom_) { *reason = "no odom"; return false; }
    return true;
  }

  // ------------------------------------------------------------ 栅格
  void gridSpec(double* res, int* nx, int* ny, double* ox, double* oy) const {
    const auto& info = map_->info;
    if (info.resolution > 0.0 && info.width > 0 && info.height > 0) {
      *res = info.resolution;
      *nx = static_cast<int>(info.width);
      *ny = static_cast<int>(info.height);
      *ox = info.origin.position.x;
      *oy = info.origin.position.y;
      return;
    }
    *res = p_res_;
    *nx = static_cast<int>(std::lround(p_sx_ / p_res_));
    *ny = static_cast<int>(std::lround(p_sy_ / p_res_));
    *ox = p_ox_;
    *oy = p_oy_;
  }

  Grid buildBlocked(double res, int nx, int ny, double ox, double oy) const {
    Grid blocked(nx, std::vector<bool>(ny, false));
    const auto& data = map_->data;
    const int total = static_cast<int>(data.size());
    for (int ix = 0; ix < nx; ++ix) {
      for (int iy = 0; iy < ny; ++iy) {
        const int i = iy * nx + ix;  // OccupancyGrid 行主序：index = y*width + x
        if (i >= total) continue;
        const int v = data[i];
        bool occ = v >= 50;
        if (v < 0 && unknown_blocked_) occ = true;
        blocked[ix][iy] = occ;
      }
    }
    // 膨胀：有场景障碍时用 OBB 精确净空，否则对占用格做形态学膨胀
    if (!obstacles_.empty()) {
      for (int ix = 0; ix < nx; ++ix) {
        const double cx = ox + (ix + 0.5) * res;
        for (int iy = 0; iy < ny; ++iy) {
          if (blocked[ix][iy]) continue;
          const double cy = oy + (iy + 0.5) * res;
          for (const auto& ob : obstacles_) {
            if (obbClearance(cx, cy, ob) < dilate_m_) {
              blocked[ix][iy] = true;
              break;
            }
          }
        }
      }
    } else {
      const int r = static_cast<int>(std::ceil(dilate_m_ / res));
      const Grid src = blocked;
      for (int ix = 0; ix < nx; ++ix) {
        for (int iy = 0; iy < ny; ++iy) {
          if (!src[ix][iy]) continue;
          for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
              const int jx = ix + dx, jy = iy + dy;
              if (jx >= 0 && jx < nx && jy >= 0 && jy < ny) blocked[jx][jy] = true;
            }
          }
        }
      }
    }
    return blocked;
  }

  std::pair<int, int> worldToCell(double x, double y, double res, int nx, int ny,
                                  double ox, double oy) const {
    int ix = static_cast<int>(std::floor((x - ox) / res));
    int iy = static_cast<int>(std::floor((y - oy) / res));
    ix = std::min(std::max(ix, 0), nx - 1);
    iy = std::min(std::max(iy, 0), ny - 1);
    return {ix, iy};
  }

  std::pair<double, double> cellToWorld(const std::pair<int, int>& c, double res,
                                        double ox, double oy) const {
    return {ox + (c.first + 0.5) * res, oy + (c.second + 0.5) * res};
  }

  // ------------------------------------------------------------ 规划
  bool plan(std::vector<std::pair<double, double>>* pts, std::string* reason,
            double* car_x, double* car_y, double* goal_x, double* goal_y,
            bool* goal_adjusted) const {
    const geometry_msgs::Point& cp = odom_->pose.pose.position;
    const geometry_msgs::Point& gp = goal_->pose.position;
    *car_x = cp.x; *car_y = cp.y;
    *goal_x = gp.x; *goal_y = gp.y;

    double res, ox, oy;
    int nx, ny;
    gridSpec(&res, &nx, &ny, &ox, &oy);
    const Grid blocked = buildBlocked(res, nx, ny, ox, oy);

    const auto start_c = worldToCell(cp.x, cp.y, res, nx, ny, ox, oy);
    auto goal_c = worldToCell(gp.x, gp.y, res, nx, ny, ox, oy);

    // N3：目标格可行化。目标格被膨胀图阻塞时，在半径 R 内就近取可行目标格（不降低保护阈值）；
    // 半径内无可行格才判不可达（区分「可达但栅格落点错误」与「确实无路」）。
    *goal_adjusted = false;
    bool adjusted = false;
    if (blocked[goal_c.first][goal_c.second]) {
      const int R = std::max(1, static_cast<int>(std::ceil(goal_adjust_max_m_ / std::max(res, 1e-6))));
      std::pair<int, int> adj;
      if (nearestFeasibleCell(blocked, goal_c, R, &adj)) {
        goal_c = adj;
        adjusted = true;
        *goal_adjusted = true;
      } else {
        *reason = "goal_cell_blocked_no_feasible";
        return false;
      }
    }

    std::vector<std::pair<int, int>> cells;
    if (!astar(blocked, start_c, goal_c, &cells)) {
      *reason = "astar_no_path";
      return false;
    }
    const auto pruned = pruneLos(cells, blocked);
    pts->clear();
    pts->reserve(pruned.size());
    for (const auto& c : pruned) pts->push_back(cellToWorld(c, res, ox, oy));
    (*pts)[0] = {cp.x, cp.y};    // 起点=规划时刻车位置
    // 终点：未调整时用 goal 精确世界点；调整后用可行格中心
    pts->back() = adjusted ? cellToWorld(goal_c, res, ox, oy)
                           : std::make_pair(gp.x, gp.y);
    return true;
  }

  // ------------------------------------------------------------ 路径可行性
  bool pathInfeasible() const {
    if (!has_path_ || path_.poses.empty()) return true;
    double res, ox, oy;
    int nx, ny;
    gridSpec(&res, &nx, &ny, &ox, &oy);
    const Grid blocked = buildBlocked(res, nx, ny, ox, oy);
    std::vector<std::pair<double, double>> pts;
    pts.reserve(path_.poses.size());
    for (const auto& p : path_.poses) {
      pts.push_back({p.pose.position.x, p.pose.position.y});
    }
    if (pts.empty()) return true;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
      const auto c0 = worldToCell(pts[i].first, pts[i].second, res, nx, ny, ox, oy);
      const auto c1 = worldToCell(pts[i + 1].first, pts[i + 1].second, res, nx, ny, ox, oy);
      if (lineBlocked(blocked, c0, c1)) return true;
    }
    const auto& nx0 = pts.size() > 1 ? pts[1] : pts[0];
    const auto c = worldToCell(nx0.first, nx0.second, res, nx, ny, ox, oy);
    return blocked[c.first][c.second];
  }

  // ------------------------------------------------------------ 触发判定
  std::string triggerOf(double t) const {
    if (!has_last_plan_goal_) return "first";
    const double gx = goal_->pose.position.x, gy = goal_->pose.position.y;
    if (std::hypot(gx - last_plan_goal_.first, gy - last_plan_goal_.second) > replan_disp_) {
      return "displacement";
    }
    const std::string cur_hash = agc::cfgS("map/version_hash", "");
    if (!cur_hash.empty() && cur_hash != last_hash_) return "map_version";
    if (pathInfeasible()) return "infeasible";
    if (has_last_plan_sim_ && (t - last_plan_sim_) >= replan_period_) return "period";
    return "";
  }

  double pathLen(const std::vector<std::pair<double, double>>& pts) const {
    double s = 0.0;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
      s += std::hypot(pts[i + 1].first - pts[i].first, pts[i + 1].second - pts[i].second);
    }
    return s;
  }

  // 把世界点序列转为 Path（frame_id=world，朝向取航段方向）。
  void setPath(const std::vector<std::pair<double, double>>& pts) {
    path_ = nav_msgs::Path();
    path_.header.frame_id = agc::FRAME_WORLD;
    path_.header.stamp = agc::simNow();
    path_.poses.resize(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) {
      geometry_msgs::PoseStamped& ps = path_.poses[i];
      ps.header.frame_id = agc::FRAME_WORLD;
      ps.pose.position.x = pts[i].first;
      ps.pose.position.y = pts[i].second;
      ps.pose.position.z = 0.0;
      double yaw = 0.0;
      if (i + 1 < pts.size()) {
        yaw = std::atan2(pts[i + 1].second - pts[i].second, pts[i + 1].first - pts[i].first);
      }
      const auto q = agc::quatFromYaw(yaw);
      ps.pose.orientation.x = q[0];
      ps.pose.orientation.y = q[1];
      ps.pose.orientation.z = q[2];
      ps.pose.orientation.w = q[3];
    }
    has_path_ = true;
  }

  void publishPath() {
    if (!has_path_ || path_.poses.empty()) return;  // 非空才发
    path_.header.frame_id = agc::FRAME_WORLD;
    path_.header.stamp = agc::simNow();
    pub_.publish(path_);
    has_last_pub_sim_ = true;
    last_pub_sim_ = agc::simNow().toSec();
  }

  // ------------------------------------------------------------ 定时器
  void tick(const ros::TimerEvent&) {
    std::string reason;
    if (!inputsValid(&reason)) {
      if (reason != last_invalid_reason_) {
        last_invalid_reason_ = reason;
        ROS_WARN("ground_planner: 输入无效(%s)，不发布路径", reason.c_str());
        agc::JsonObj o;
        o.n("t_sim", agc::simNow().toSec()).v("trigger", "invalid").bv("success", false);
        o.v("failure_reason", reason);
        agc::appendJsonl("planner_log.jsonl", o);
      }
      return;
    }
    last_invalid_reason_.clear();

    const double t = agc::simNow().toSec();
    const std::string trigger = triggerOf(t);
    if (trigger.empty()) {
      // 未触发重规划：按 path_hz 重发当前路径
      if (has_path_ && (!has_last_pub_sim_ || (t - last_pub_sim_) >= 1.0 / path_hz_)) {
        publishPath();
      }
      return;
    }

    const double t0 = agc::wallNow();  // 单调墙钟，仅测耗时
    std::vector<std::pair<double, double>> pts;
    std::string fail_reason;
    double car_x = 0, car_y = 0, goal_x = 0, goal_y = 0;
    bool goal_adjusted = false;
    const bool ok = plan(&pts, &fail_reason, &car_x, &car_y, &goal_x, &goal_y,
                         &goal_adjusted);
    const double plan_ms = (agc::wallNow() - t0) * 1000.0;
    const std::string cur_hash = agc::cfgS("map/version_hash", "");

    if (ok) {
      fail_count_ = 0;
      ros::param::set("/agc/planner/unreachable", false);
      setPath(pts);
      has_last_plan_goal_ = true;
      last_plan_goal_ = {goal_x, goal_y};
      has_last_plan_sim_ = true;
      last_plan_sim_ = t;
      last_hash_ = cur_hash;
      path_len_ = pathLen(pts);
      ROS_INFO("ground_planner: 规划成功 trigger=%s waypoints=%zu len=%.2fm %.1fms",
               trigger.c_str(), pts.size(), path_len_, plan_ms);
      publishPath();
      agc::JsonObj o;
      o.n("t_sim", t).v("trigger", trigger);
      o.raw("goal", agc::JsonObj::arrNum({goal_x, goal_y}));
      o.raw("car_pose", agc::JsonObj::arrNum({car_x, car_y}));
      o.v("map_version_hash", cur_hash).n("plan_time_ms", plan_ms).n("path_len_m", path_len_);
      o.iv("n_waypoints", static_cast<long long>(pts.size())).bv("success", true)
          .bv("goal_adjusted", goal_adjusted);
      o.v("failure_reason", "");
      agc::appendJsonl("planner_log.jsonl", o);
    } else {
      ++fail_count_;
      if (fail_count_ >= retry_max_) ros::param::set("/agc/planner/unreachable", true);
      has_path_ = false;  // 失败时不发布新路径
      ROS_WARN("ground_planner: 规划失败 trigger=%s (%s) 连续=%d", trigger.c_str(),
               fail_reason.c_str(), fail_count_);
      agc::JsonObj o;
      o.n("t_sim", t).v("trigger", trigger);
      o.raw("goal", agc::JsonObj::arrNum({goal_x, goal_y}));
      o.raw("car_pose", agc::JsonObj::arrNum({car_x, car_y}));
      o.v("map_version_hash", cur_hash).n("plan_time_ms", plan_ms).n("path_len_m", 0.0);
      o.iv("n_waypoints", 0).bv("success", false).v("failure_reason", fail_reason);
      agc::appendJsonl("planner_log.jsonl", o);
    }
  }

  // M4：融合障碍更新（仅 obstacle_fusion_ 时订阅）。用并集后的保守外扩尺寸做膨胀。
  void obstacleCb(const air_ground_core::ObstacleTrackArray::ConstPtr& msg) {
    std::vector<ObsView> obs;
    obs.reserve(msg->tracks.size());
    for (const auto& t : msg->tracks) {
      obs.push_back(ObsView{t.pose.position.x, t.pose.position.y,
                            agc::yawFromQuat(t.pose.orientation),
                            t.size.x * 0.5, t.size.y * 0.5});
    }
    obstacles_ = obs;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_goal_, sub_map_, sub_odom_, sub_obstacles_;
  ros::Publisher pub_;
  ros::Timer timer_;

  // 参数
  double replan_disp_, replan_period_, inflation_, protection_, path_hz_;
  double p_res_, p_sx_, p_sy_, p_ox_, p_oy_;
  int retry_max_;
  bool unknown_blocked_;
  bool obstacle_fusion_ = false;
  double dilate_m_;
  double goal_adjust_max_m_ = 1.0;  // N3 目标格可行化搜索半径(m)
  std::vector<ObsView> obstacles_;

  // 输入缓存
  nav_msgs::OccupancyGrid::ConstPtr map_;
  geometry_msgs::PoseStamped::ConstPtr goal_;
  nav_msgs::Odometry::ConstPtr odom_;
  bool has_map_ = false, has_goal_ = false, has_odom_ = false;

  // 规划状态
  nav_msgs::Path path_;
  bool has_path_ = false;
  double path_len_ = 0.0;
  bool has_last_plan_goal_ = false;
  std::pair<double, double> last_plan_goal_;
  bool has_last_plan_sim_ = false;
  double last_plan_sim_ = 0.0;
  std::string last_hash_;
  bool has_last_pub_sim_ = false;
  double last_pub_sim_ = 0.0;
  int fail_count_ = 0;
  std::string last_invalid_reason_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ground_planner");
  GroundPlanner node;
  ros::spin();
  return 0;
}