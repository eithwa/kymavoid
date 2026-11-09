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

void GridPlanner::build(const std::vector<Pt>& obstacles, double theta, const Keepout& k)
{
  keep_ = k;
  occ_ = cv::Mat::zeros(ny_, nx_, CV_8U);
  for (const Pt& q : obstacles)
  {
    int r, c;
    if (cell(q.x, q.y, r, c))
      occ_.at<uint8_t>(r, c) = 1;
  }
  const int infl = static_cast<int>(std::lround(p_.obstacle_inflation / p_.res));
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
  cv::dilate(occ_, cspace_, kernel, cv::Point(half, half), 1, cv::BORDER_CONSTANT, cv::Scalar(0));

  cv::Mat freemask = (cspace_ == 0);
  cv::distanceTransform(freemask, dist_, cv::DIST_L2, 3);
  dist_ *= p_.res;
}

bool GridPlanner::free(double x, double y) const
{
  int r, c;
  return cell(x, y, r, c) && cspace_.at<uint8_t>(r, c) == 0;
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

bool GridPlanner::lineOfSight(int r0, int c0, int r1, int c1) const
{
  // a shortcut must keep half the preferred clearance, so smoothing does not undo A*'s spacing;
  // in gaps tighter than that the A* cells are kept as they are
  const float need = static_cast<float>(std::max(p_.res * 1.5, 0.5 * p_.clearance_pref));
  const int n = std::max(std::abs(r1 - r0), std::abs(c1 - c0));
  for (int i = 0; i <= n; ++i)
  {
    const int r = r0 + (n ? (r1 - r0) * i / n : 0);
    const int c = c0 + (n ? (c1 - c0) * i / n : 0);
    if (cspace_.at<uint8_t>(r, c) || dist_.at<float>(r, c) < need)
      return false;
  }
  return true;
}

bool GridPlanner::plan(double x, double y, std::vector<Pt>& path) const
{
  path.clear();
  int sr, sc;
  if (!cell(x, y, sr, sc))
    return false;
  if (cspace_.at<uint8_t>(sr, sc))                         // snap to the nearest free cell
  {
    const int k = static_cast<int>(std::lround(p_.start_snap / p_.res));
    double best = 1e9;
    int br = -1, bc = -1;
    for (int r = std::max(0, sr - k); r <= std::min(ny_ - 1, sr + k); ++r)
      for (int c = std::max(0, sc - k); c <= std::min(nx_ - 1, sc + k); ++c)
        if (!cspace_.at<uint8_t>(r, c) && std::hypot(r - sr, c - sc) < best)
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
  const int goal_col = static_cast<int>(std::ceil((keep_.goal_x - front_ - p_.x_min) / p_.res));
  const int N = nx_ * ny_;
  std::vector<float> g(N, 1e30f);
  std::vector<int> parent(N, -1);
  std::vector<uint8_t> closed(N, 0);
  using Node = std::pair<float, int>;
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
  auto h = [&](int c) { return static_cast<float>(std::max(0, goal_col - c) * p_.res); };
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
      if (vr < 0 || vr >= ny_ || vc < 0 || vc >= nx_ || cspace_.at<uint8_t>(vr, vc))
        continue;
      const int v = vr * nx_ + vc;
      if (closed[v])
        continue;
      const float d = dist_.at<float>(vr, vc);
      const float pen = d < p_.clearance_pref ? p_.clearance_weight * (p_.clearance_pref - d) / p_.clearance_pref : 0.f;
      const float step = p_.res * ((dr[k] && dc[k]) ? 1.41421356f : 1.f) * (1.f + pen);
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
  // shortcut smoothing: keep a waypoint only where the straight line to the next would be blocked
  std::vector<int> keep{cells.front()};
  size_t i = 0;
  while (i + 1 < cells.size())
  {
    size_t j = cells.size() - 1;
    while (j > i + 1 && !lineOfSight(cells[i] / nx_, cells[i] % nx_, cells[j] / nx_, cells[j] % nx_))
      --j;
    keep.push_back(cells[j]);
    i = j;
  }
  for (int v : keep)
    path.push_back({p_.x_min + (v % nx_) * p_.res, p_.y_min + (v / nx_) * p_.res});
  path.front() = {x, y};
  return true;
}
}  // namespace avoid_nav
