#include "avoid_nav/planner.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <queue>

namespace avoid_nav
{
GridPlanner::GridPlanner(const PlannerParams& p) : p_(p)
{
  nx_ = static_cast<int>(std::ceil((p_.x_max - p_.x_min) / p_.res)) + 1;
  ny_ = static_cast<int>(std::ceil((p_.y_max - p_.y_min) / p_.res)) + 1;
}

void GridPlanner::setFootprint(const Polygon& fp)
{
  fp_ = fp;
  front_ = -1e9;
  for (const Pt& q : fp_)
    front_ = std::max(front_, q.x);
}

bool GridPlanner::cell(double x, double y, int& r, int& c) const
{
  c = static_cast<int>(std::lround((x - p_.x_min) / p_.res));
  r = static_cast<int>(std::lround((y - p_.y_min) / p_.res));
  return r >= 0 && r < ny_ && c >= 0 && c < nx_;
}

void GridPlanner::build(const std::vector<Pt>& obstacles, double theta, const Keepout& k, const std::vector<Pt>& far)
{
  keep_ = k;
  occ_ = cv::Mat::zeros(ny_, nx_, CV_8U);
  for (const Pt& q : obstacles)
  {
    int r, c;
    if (cell(q.x, q.y, r, c))
      occ_.at<uint8_t>(r, c) = 1;
  }
  const int infl = static_cast<int>(std::lround(std::min(p_.hard_inflation, p_.obstacle_inflation) / p_.res));
  if (infl > 0)
    cv::dilate(occ_, occ_, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * infl + 1, 2 * infl + 1)));

  // keep-out zones (touching a red line loses; the low wall bounds the course front and back)
  const double lim = k.red_inner - k.line_margin;
  for (int r = 0; r < ny_; ++r)
  {
    const double y = p_.y_min + r * p_.res;
    if (std::fabs(y) >= lim)
      occ_.row(r).setTo(1);
  }
  for (int c = 0; c < nx_; ++c)
  {
    const double x = p_.x_min + c * p_.res;
    if (x >= k.front_wall_x || x <= k.back_wall_x)
      occ_.col(c).setTo(1);
  }

  // footprint kernel at the current heading, anchored at the robot origin
  double ext = 0;
  for (const Pt& q : fp_)
    ext = std::max(ext, std::hypot(q.x, q.y));
  const int half = static_cast<int>(std::ceil(ext / p_.res)) + 1;
  cv::Mat kernel = cv::Mat::zeros(2 * half + 1, 2 * half + 1, CV_8U);
  std::vector<cv::Point> pts;
  const double c = std::cos(theta), s = std::sin(theta);
  for (const Pt& q : fp_)
    pts.emplace_back(static_cast<int>(std::lround((c * q.x - s * q.y) / p_.res)) + half,
                     static_cast<int>(std::lround((s * q.x + c * q.y) / p_.res)) + half);
  cv::fillPoly(kernel, std::vector<std::vector<cv::Point>>{pts}, cv::Scalar(1));
  cv::dilate(occ_, hard_, kernel, cv::Point(half, half), 1, cv::BORDER_CONSTANT, cv::Scalar(0));

  // far C-space: the same dilation, stamped at each of the few far cells (a full-grid dilate costs as
  // much as the obstacles' C-space, for a few dozen points); robot positions p whose footprint covers
  // s: p = s - (k - half)
  far_.release();
  if (!far.empty())
  {
    far_ = cv::Mat::zeros(ny_, nx_, CV_8U);
    cv::Mat fk = kernel;
    const int sh = static_cast<int>(std::lround(p_.far_shrink / p_.res));
    if (sh > 0)
      cv::erode(kernel, fk, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * sh + 1, 2 * sh + 1)),
                cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, cv::Scalar(0));
    for (const Pt& q : far)
    {
      int r, c;
      if (!cell(q.x, q.y, r, c))
        continue;
      for (int kr = 0; kr < fk.rows; ++kr)
      {
        const int rr = r - (kr - half);
        if (rr < 0 || rr >= ny_)
          continue;
        const uint8_t* krow = fk.ptr<uint8_t>(kr);
        uint8_t* frow = far_.ptr<uint8_t>(rr);
        for (int kc = 0; kc < fk.cols; ++kc)
        {
          const int cc = c - (kc - half);
          if (krow[kc] && cc >= 0 && cc < nx_)
            frow[cc] = 1;
        }
      }
    }
  }

  // soft C-space = hard C-space grown by the band (Minkowski sums commute: inflating the obstacles
  // first and then dilating by the footprint is the same as growing the C-space)
  cv::Mat freemask = (hard_ == 0);
  cv::Mat dist_hard;
  cv::distanceTransform(freemask, dist_hard, cv::DIST_L2, 3);
  dist_hard *= p_.res;                                     // the first free ring is one cell (res) away
  const float band = static_cast<float>(std::max(0.0, p_.obstacle_inflation - p_.hard_inflation));
  cspace_ = (dist_hard < band + 0.5f * static_cast<float>(p_.res)) / 255;   // CV_8U 0/1
  dist_ = cv::max(dist_hard - band, 0.0f);
  dist_.setTo(0.0f, cspace_);
}

bool GridPlanner::free(double x, double y) const
{
  int r, c;
  return cell(x, y, r, c) && cspace_.at<uint8_t>(r, c) == 0;
}

bool GridPlanner::farCost(double x, double y) const
{
  int r, c;
  return !far_.empty() && cell(x, y, r, c) && far_.at<uint8_t>(r, c);
}

double GridPlanner::clearance(double x, double y) const
{
  int r, c;
  if (!cell(x, y, r, c))
    return 0.0;
  return dist_.at<float>(r, c);
}

double GridPlanner::freeDistance(double x, double y, double ux, double uy, double max_d) const
{
  const double step = p_.res * 0.5;
  double d = 0;
  // the robot origin may sit just inside the inflated C-space (perception margin): step out first
  while (d <= p_.start_snap && !free(x + ux * d, y + uy * d))
    d += step;
  if (d > p_.start_snap)
    return 0.0;
  for (; d <= max_d; d += step)
    if (!free(x + ux * d, y + uy * d))
      return std::max(0.0, d - step);
  return max_d;
}

double GridPlanner::reachSpeed(int r, int c, int sr, int sc, double v0) const
{
  const double d = std::hypot(r - sr, c - sc) * p_.res;                      // <= the path distance
  const double to_goal = std::max(0.0, keep_.goal_x - front_ - (p_.x_min + c * p_.res));
  const double v_goal2 = 2 * p_.a_brake * std::max(0.0, keep_.front_wall_x - keep_.goal_x - 0.03);
  return std::min(p_.v_max, std::min(std::sqrt(v0 * v0 + 2 * p_.a_max * d), std::sqrt(v_goal2 + 2 * p_.a_brake * to_goal)));
}

float GridPlanner::stepFactor(int r, int c, double v_reach) const
{
  const float d = dist_.at<float>(r, c);
  const float pref = static_cast<float>(p_.clearance_pref);
  float f = 1.f + (d < pref ? static_cast<float>(p_.clearance_weight) * (pref - d) / pref : 0.f);
  const double v = std::min(v_reach, p_.v_tight + p_.v_per_clearance * d);
  if (v > 1e-3)
    f = std::max(f, static_cast<float>(v_reach / v));       // travel time relative to the fastest possible
  if (cspace_.at<uint8_t>(r, c))
    f *= static_cast<float>(p_.band_cost);                  // between the hard and the soft boundary
  if (!far_.empty() && far_.at<uint8_t>(r, c))
    f *= static_cast<float>(p_.far_cost);                   // an obstacle seen far away, roughly
  return f;
}

bool GridPlanner::lineOfSight(const std::vector<int>& cells, const std::vector<float>& need,
                              const std::vector<uint8_t>& far_ok, size_t i, size_t j) const
{
  const int r0 = cells[i] / nx_, c0 = cells[i] % nx_, r1 = cells[j] / nx_, c1 = cells[j] % nx_;
  const int n = std::max(std::abs(r1 - r0), std::abs(c1 - c0));
  for (int m = 0; m <= n; ++m)
  {
    const int r = r0 + (n ? (r1 - r0) * m / n : 0);
    const int c = c0 + (n ? (c1 - c0) * m / n : 0);
    const size_t k = i + (n ? (j - i) * static_cast<size_t>(m) / static_cast<size_t>(n) : 0);
    if (cspace_.at<uint8_t>(r, c) || dist_.at<float>(r, c) < need[k])
      return false;
    if (!far_.empty() && far_.at<uint8_t>(r, c) && !far_ok[k])
      return false;
  }
  return true;
}

bool GridPlanner::plan(double x, double y, std::vector<Pt>& path, const std::vector<Pt>* previous, double v0) const
{
  path.clear();
  int sr, sc;
  if (!cell(x, y, sr, sc))
    return false;
  if (hard_.at<uint8_t>(sr, sc))                           // snap to the nearest cell outside the hard C-space
  {
    const int k = static_cast<int>(std::lround(p_.start_snap / p_.res));
    double best = 1e9;
    int br = -1, bc = -1;
    for (int r = std::max(0, sr - k); r <= std::min(ny_ - 1, sr + k); ++r)
      for (int c = std::max(0, sc - k); c <= std::min(nx_ - 1, sc + k); ++c)
        if (!hard_.at<uint8_t>(r, c) && std::hypot(r - sr, c - sc) < best)
        {
          best = std::hypot(r - sr, c - sc);
          br = r;
          bc = c;
        }
    if (br < 0)
      return false;
    sr = br;
    sc = bc;
  }
  // hysteresis: cells near the path being followed
  cv::Mat keep;
  if (previous && previous->size() >= 2 && p_.keep_factor < 1.0)
  {
    keep = cv::Mat::zeros(ny_, nx_, CV_8U);
    std::vector<cv::Point> poly;
    for (const Pt& q : *previous)
      poly.emplace_back(static_cast<int>(std::lround((q.x - p_.x_min) / p_.res)),
                        static_cast<int>(std::lround((q.y - p_.y_min) / p_.res)));
    cv::polylines(keep, std::vector<std::vector<cv::Point>>{poly}, false, cv::Scalar(1),
                  std::max(1, static_cast<int>(std::lround(2 * p_.keep_radius / p_.res))));
  }
  const float keep_f = static_cast<float>(p_.keep_factor);
  const int goal_col = static_cast<int>(std::ceil((keep_.goal_x - front_ - p_.x_min) / p_.res));
  const int N = nx_ * ny_;
  std::vector<float> g(N, 1e30f);
  std::vector<int> parent(N, -1);
  std::vector<uint8_t> closed(N, 0);
  using Node = std::pair<float, int>;
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
  // admissible: no step costs less than res x keep_factor
  const float h_scale = static_cast<float>(p_.res) * (keep.empty() ? 1.f : std::min(1.f, keep_f));
  auto h = [&](int c) { return static_cast<float>(std::max(0, goal_col - c)) * h_scale; };
  const int s = sr * nx_ + sc;
  g[s] = 0;
  open.push({h(sc), s});
  static const int dr[8] = {-1, -1, -1, 0, 0, 1, 1, 1}, dc[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
  int goal = -1;
  while (!open.empty())
  {
    const int u = open.top().second;
    open.pop();
    if (closed[u])
      continue;
    closed[u] = 1;
    const int ur = u / nx_, uc = u % nx_;
    if (uc >= goal_col)
    {
      goal = u;
      break;
    }
    for (int k = 0; k < 8; ++k)
    {
      const int vr = ur + dr[k], vc = uc + dc[k];
      if (vr < 0 || vr >= ny_ || vc < 0 || vc >= nx_ || hard_.at<uint8_t>(vr, vc))
        continue;
      const int v = vr * nx_ + vc;
      if (closed[v])
        continue;
      float f = stepFactor(vr, vc, reachSpeed(vr, vc, sr, sc, v0));
      if (!keep.empty() && keep.at<uint8_t>(vr, vc))
        f *= keep_f;
      const float step = static_cast<float>(p_.res) * ((dr[k] && dc[k]) ? 1.41421356f : 1.f) * f;
      if (g[u] + step < g[v])
      {
        g[v] = g[u] + step;
        parent[v] = u;
        open.push({g[v] + h(vc), v});
      }
    }
  }
  if (goal < 0)
    return false;
  std::vector<int> cells;
  for (int v = goal; v >= 0; v = parent[v])
    cells.push_back(v);
  std::reverse(cells.begin(), cells.end());
  // Shortcut smoothing: keep a waypoint only where the straight line to a later one would be
  // blocked. Every point of a shortcut must keep the clearance the A* path had around the
  // corresponding cells (up to clearance_pref), less one cell. Requiring only the smallest clearance
  // of the replaced cells - as before - let a shortcut from a tight gap's exit run along the next
  // obstacle at that gap's 1-2 cm, crawling at v_tight. Not more than clearance_pref: A* has no
  // turning cost, and keeping the travel-time clearance made the path weave between boxes.
  const float floor_need = static_cast<float>(p_.res * 1.5);
  std::vector<float> need(cells.size());
  const int w = 3;                                          // +-3 cells: lenient where A* came close nearby
  for (size_t k = 0; k < cells.size(); ++k)
  {
    const float want = static_cast<float>(p_.clearance_pref);
    float local = 1e9f;
    for (size_t q = (k >= static_cast<size_t>(w) ? k - w : 0); q <= std::min(cells.size() - 1, k + w); ++q)
      local = std::min(local, dist_.at<float>(cells[q] / nx_, cells[q] % nx_));
    need[k] = std::max(floor_need, std::min(want, local) - static_cast<float>(p_.res));
  }
  // a shortcut may cross the far obstacles' C-space only where A* (which paid for it) did, +-w cells
  std::vector<uint8_t> far_ok(cells.size(), 1);
  if (!far_.empty())
    for (size_t k = 0; k < cells.size(); ++k)
    {
      far_ok[k] = 0;
      for (size_t q = (k >= static_cast<size_t>(w) ? k - w : 0); q <= std::min(cells.size() - 1, k + w) && !far_ok[k]; ++q)
        far_ok[k] = far_.at<uint8_t>(cells[q] / nx_, cells[q] % nx_);
    }
  std::vector<int> keep_cells{cells.front()};
  size_t i = 0;
  while (i + 1 < cells.size())
  {
    size_t j = cells.size() - 1;
    while (j > i + 1 && !lineOfSight(cells, need, far_ok, i, j))
      --j;
    keep_cells.push_back(cells[j]);
    i = j;
  }
  for (int v : keep_cells)
    path.push_back({p_.x_min + (v % nx_) * p_.res, p_.y_min + (v / nx_) * p_.res});
  path.front() = {x, y};
  return true;
}
}  // namespace avoid_nav
