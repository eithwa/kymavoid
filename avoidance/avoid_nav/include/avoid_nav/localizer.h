// Field-frame localisation: EKF over (x, y, theta, sx, sy).
//
//   predict : body velocity from wheel odometry, heading change from the IMU. sx, sy are the
//             odometry scale errors (forward / sideways). Omni-wheel odometry errors are mostly such
//             scale errors, which grow linearly with distance; modelling them as white noise would
//             make the filter far too confident and its gates would reject the corrections.
//   update  : known straight lines of the field seen by the camera. Points near where a line is
//             expected are fitted with a line; its distance and angle in the base frame correct the
//             pose along the line's normal and the heading.
//               red lines (from /boundary)        -> y and theta
//               low wall faces (from /scan, black) -> x (front/back wall), y (side walls), theta
//             The lidars see neither, so in the lidar configuration these updates simply never fire.
//
// The lines are absolute references: the pose does not drift along their normals while one is in
// view, and a crooked start (placement error) is corrected as soon as a line is seen.
//
// x is corrected only by the front / back low walls. A front / back wall that should be in view but
// has not matched for lost_scans scans is searched for in a wider band (lost_gate) and, if a long
// straight fit turns up, the pose and the scale uncertainty along it are widened first so that the
// fit is accepted (2026-11-10). Before, once the x error exceeded the 35 cm association gate the
// wall never matched again and x drifted to -0.6 m: the robot was past the goal line without knowing
// it and pushed into the wall. (Most of that error came from a camera latency 40-55 ms longer than
// camera_adapter assumed: camera.yaml latency, avoid_sim's virtual camera.)
#pragma once
#include <Eigen/Dense>

#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "avoid_nav/polygon.h"

namespace avoid_nav
{
inline double wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

// A field line n . p = c (n unit, field frame), seen from the side where n . p < c.
struct FieldLine
{
  std::string name;
  double nx, ny, c;
  double t_min, t_max;   // extent along the line (coordinate along (-ny, nx))
};

struct LineFit
{
  std::string name;
  bool ok = false;
  int n = 0;                  // points used
  double rho = 0, alpha = 0;  // base frame: normal angle alpha, n . p = rho
  double rms = 0, spread = 0;
  bool accepted = false;      // passed the innovation gate
  double mahalanobis2 = 0;
};

struct LocalizerParams
{
  double sigma_x0 = 0.02, sigma_y0 = 0.02, sigma_theta0 = 2.0 * M_PI / 180;
  double sigma_sx0 = 0.05, sigma_sy0 = 0.08;   // odometry scale error prior (sideways is worse)
  double scale_walk = 0.002;         // 1 / sqrt(s): slow change of the scale errors
  double vel_noise_const = 0.01;     // m/s, white
  double vel_noise_long = 0.02;      // fraction of |v| along the motion, white
  double vel_noise_lat = 0.04;       // fraction of |v| sideways, white
  double gyro_noise = 0.002;         // rad / sqrt(s)
  double line_gate_min = 0.06, line_gate_max = 0.35;
  int line_min_points = 5;
  double line_min_spread = 0.20;
  double sigma_rho_floor = 0.015, sigma_rho_quad = 0.015;   // + quad * rho^2: the camera coarsens with range
  double sigma_alpha_floor = 0.6 * M_PI / 180;
  double innovation_gate = 9.21;     // chi-square, 2 dof, 99 %
  int lost_scans = 15;               // front / back wall expected in view, unmatched this many scans: ...
  double lost_gate = 0.6;            // [m] ... search this far (boxes stay >= 0.75 m from those walls)
  double lost_min_spread = 0.5;      // [m] ... and accept only a long,
  double lost_max_rms = 0.03;        // [m] ... straight fit
};

class Localizer
{
public:
  using Mat5 = Eigen::Matrix<double, 5, 5>;
  explicit Localizer(const LocalizerParams& p = LocalizerParams()) : p_(p) { reset(0, 0, 0); }

  void reset(double x, double y, double theta)
  {
    s_ << x, y, theta, 0, 0;
    P_.setZero();
    P_(0, 0) = p_.sigma_x0 * p_.sigma_x0;
    P_(1, 1) = p_.sigma_y0 * p_.sigma_y0;
    P_(2, 2) = p_.sigma_theta0 * p_.sigma_theta0;
    P_(3, 3) = p_.sigma_sx0 * p_.sigma_sx0;
    P_(4, 4) = p_.sigma_sy0 * p_.sigma_sy0;
    lost_.clear();
    reacquired_ = 0;
  }

  // vx, vy: body velocity [m/s]; dtheta: heading change from the IMU [rad]
  void predict(double vx, double vy, double dtheta, double dt)
  {
    if (dt <= 0)
      return;
    const double th = s_(2), c = std::cos(th), s = std::sin(th);
    const double vxs = vx * (1 + s_(3)), vys = vy * (1 + s_(4));   // scale-corrected body velocity
    s_(0) += (c * vxs - s * vys) * dt;
    s_(1) += (s * vxs + c * vys) * dt;
    s_(2) = wrapAngle(th + dtheta);
    Mat5 F = Mat5::Identity();
    F(0, 2) = (-s * vxs - c * vys) * dt;
    F(1, 2) = (c * vxs - s * vys) * dt;
    F(0, 3) = c * vx * dt;
    F(0, 4) = -s * vy * dt;
    F(1, 3) = s * vx * dt;
    F(1, 4) = c * vy * dt;
    const double v = std::hypot(vx, vy);
    const double sl = p_.vel_noise_const + p_.vel_noise_long * v;
    const double st = p_.vel_noise_const + p_.vel_noise_lat * v;
    Eigen::Matrix<double, 5, 2> G = Eigen::Matrix<double, 5, 2>::Zero();
    G << c * dt, -s * dt, s * dt, c * dt, 0, 0, 0, 0, 0, 0;
    Eigen::Matrix2d Qv = Eigen::Vector2d(sl * sl, st * st).asDiagonal();
    Mat5 Q = G * Qv * G.transpose();
    Q(2, 2) += p_.gyro_noise * p_.gyro_noise * dt;
    Q(3, 3) += p_.scale_walk * p_.scale_walk * dt;
    Q(4, 4) += p_.scale_walk * p_.scale_walk * dt;
    P_ = F * P_ * F.transpose() + Q;
  }

  // Points (base frame, metres) that may belong to any of `lines`; points farther than max_range
  // are ignored (too coarse). Returns one fit per line, for diagnostics.
  std::vector<LineFit> updateLines(const std::vector<Pt>& pts, const std::vector<FieldLine>& lines, double max_range)
  {
    std::vector<LineFit> fits;
    for (const FieldLine& L : lines)
    {
      const bool x_ref = std::fabs(L.nx) > 0.9;                // the low wall in front of / behind the course
      const double rho_hat = L.c - (L.nx * s_(0) + L.ny * s_(1));
      const bool expected = rho_hat > 0.3 && rho_hat < max_range - 0.2;
      int& lost = lost_[L.name];
      const bool wide = x_ref && lost >= p_.lost_scans;
      LineFit f = fit(pts, L, max_range, wide);
      if (f.ok)
        apply(L, f, wide);
      if (f.accepted && wide)
        ++reacquired_;
      lost = (f.accepted || !expected) ? 0 : lost + 1;
      fits.push_back(f);
    }
    return fits;
  }

  int reacquired() const { return reacquired_; }   // wide-band re-matches of a lost front / back wall

  double x() const { return s_(0); }
  double y() const { return s_(1); }
  double theta() const { return s_(2); }
  double scaleX() const { return s_(3); }
  double scaleY() const { return s_(4); }
  const Mat5& cov() const { return P_; }

private:
  LineFit fit(const std::vector<Pt>& pts, const FieldLine& L, double max_range, bool wide = false) const
  {
    LineFit f;
    f.name = L.name;
    const double th = s_(2), c = std::cos(th), s = std::sin(th);
    // predicted uncertainty along the line normal: pose part + heading lever arm (per point below)
    Eigen::Matrix<double, 5, 1> g;
    g << L.nx, L.ny, 0, 0, 0;
    const double var_n = g.dot(P_ * g);
    std::vector<Pt> sel;
    for (const Pt& b : pts)
    {
      const double r = std::hypot(b.x, b.y);
      if (!(r <= max_range))
        continue;
      const double fx = s_(0) + c * b.x - s * b.y;
      const double fy = s_(1) + s * b.x + c * b.y;
      const double off = L.nx * fx + L.ny * fy - L.c;                     // signed distance to the line
      const double along = -L.ny * fx + L.nx * fy;
      const double sig = std::sqrt(var_n + r * r * P_(2, 2));
      const double gate = wide ? p_.lost_gate : std::min(p_.line_gate_max, std::max(p_.line_gate_min, 3.0 * sig + 0.03));
      if (std::fabs(off) <= gate && along >= L.t_min - 0.1 && along <= L.t_max + 0.1)
        sel.push_back(b);
    }
    for (int pass = 0; pass < 2; ++pass)                    // fit, drop > 3 cm outliers, refit
    {
      if (static_cast<int>(sel.size()) < p_.line_min_points)
        return f;
      Eigen::Vector2d m(0, 0);
      for (const Pt& q : sel)
        m += Eigen::Vector2d(q.x, q.y);
      m /= sel.size();
      Eigen::Matrix2d C = Eigen::Matrix2d::Zero();
      for (const Pt& q : sel)
      {
        Eigen::Vector2d d(q.x - m(0), q.y - m(1));
        C += d * d.transpose();
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(C);
      Eigen::Vector2d n = es.eigenvectors().col(0);           // smallest eigenvalue: line normal
      Eigen::Vector2d dir = es.eigenvectors().col(1);
      double rho = n.dot(m);
      if (rho < 0)
      {
        n = -n;
        rho = -rho;
      }
      double lo = 1e9, hi = -1e9, ss = 0;
      std::vector<Pt> keep;
      for (const Pt& q : sel)
      {
        const double res = n.dot(Eigen::Vector2d(q.x, q.y)) - rho;
        const double t = dir.dot(Eigen::Vector2d(q.x, q.y));
        lo = std::min(lo, t);
        hi = std::max(hi, t);
        ss += res * res;
        if (std::fabs(res) <= 0.03 + 0.02 * std::fabs(rho))
          keep.push_back(q);
      }
      f.n = sel.size();
      f.rho = rho;
      f.alpha = std::atan2(n(1), n(0));
      f.rms = std::sqrt(ss / sel.size());
      f.spread = hi - lo;
      if (pass == 0 && keep.size() < sel.size())
        sel = keep;
      else
        break;
    }
    f.ok = f.n >= p_.line_min_points && f.spread >= p_.line_min_spread;
    if (wide)
      f.ok = f.ok && f.spread >= p_.lost_min_spread && f.rms <= p_.lost_max_rms;
    return f;
  }

  void apply(const FieldLine& L, LineFit& f, bool wide = false)
  {
    const double th = s_(2);
    const double rho_hat = L.c - (L.nx * s_(0) + L.ny * s_(1));          // robot to line along n
    const double alpha_hat = wrapAngle(std::atan2(L.ny, L.nx) - th);      // n seen from the robot
    Eigen::Vector2d innov(f.rho - rho_hat, wrapAngle(f.alpha - alpha_hat));
    if (wide)
    {
      // re-matching a lost wall: the filter was too sure of itself. Widen the pose along the normal
      // by the innovation, and the scale errors back to their prior, so the fit is taken.
      Eigen::Matrix<double, 5, 1> gn;
      gn << L.nx, L.ny, 0, 0, 0;
      P_ += gn * gn.transpose() * (innov(0) * innov(0));
      P_(3, 3) = std::max(P_(3, 3), p_.sigma_sx0 * p_.sigma_sx0);
      P_(4, 4) = std::max(P_(4, 4), p_.sigma_sy0 * p_.sigma_sy0);
    }
    Eigen::Matrix<double, 2, 5> H;
    H << -L.nx, -L.ny, 0, 0, 0, 0, 0, -1, 0, 0;
    const double sr = std::max(p_.sigma_rho_floor, f.rms / std::sqrt(f.n)) + p_.sigma_rho_quad * f.rho * f.rho;
    const double sa = std::max(p_.sigma_alpha_floor, f.rms / std::max(f.spread, 0.05));
    Eigen::Matrix2d R = Eigen::Vector2d(sr * sr, sa * sa).asDiagonal();
    Eigen::Matrix2d S = H * P_ * H.transpose() + R;
    f.mahalanobis2 = innov.dot(S.ldlt().solve(innov));
    if (f.mahalanobis2 > p_.innovation_gate)
      return;                                                // e.g. a red object outside the field
    Eigen::Matrix<double, 5, 2> K = P_ * H.transpose() * S.inverse();
    s_ += K * innov;
    s_(2) = wrapAngle(s_(2));
    P_ = (Mat5::Identity() - K * H) * P_;
    P_ = 0.5 * (P_ + P_.transpose());
    f.accepted = true;
  }

  LocalizerParams p_;
  Eigen::Matrix<double, 5, 1> s_;
  Mat5 P_;
  std::map<std::string, int> lost_;    // per line: scans in a row expected in view but not matched
  int reacquired_ = 0;
};
}  // namespace avoid_nav
