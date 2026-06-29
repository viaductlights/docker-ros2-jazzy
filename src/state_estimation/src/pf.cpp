#include "rclcpp/rclcpp.hpp"

#include <random>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;

struct Particle {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double weight = 1.0;
};

class ParticleFilter : public rclcpp::Node {
public:
    ParticleFilter() : Node("pf"){
	this->declare_parameter<int>("n_particles", 500); // test variables: 500, 100, 1000
	num_particles_ = this->get_parameter("n_particles").as_int();

        // Initialize random engine
        std::random_device rd;
        gen_ = std::mt19937(rd());

        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&ParticleFilter::odomCallback, this, std::placeholders::_1));
 scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ParticleFilter::scanCallback, this, _1));

        // map_server publishes /map once, latched (transient_local). Subscribing
        // with matching QoS means we still get it even if map_server published
        // before this node came up.
        rclcpp::QoS map_qos(rclcpp::KeepLast(1));
        map_qos.transient_local();
        map_qos.reliable();
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", map_qos,
            std::bind(&ParticleFilter::mapCallback, this, _1));

        // Publisher to visualize particles in RViz2
        particle_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/particles", 10);
	// Single best-estimate pose, for evaluation against ground truth (e.g.
	// the tb4 pose bridged from gz sim) and for tools like rqt_plot/PlotJuggler.
	pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_pf", 10);
	tb4_pf_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("tb4_pf_path", 10); // initialize tb4 pf path publisher

        initialize_particles();

        RCLCPP_INFO(get_logger(), "PF initialised: %d particles. Waiting on /map to build the likelihood field...", num_particles_);
    }

  private:
    	void initialize_particles() {
          std::normal_distribution<double> dist_x(0.0, 0.5);
          std::normal_distribution<double> dist_y(0.0, 0.5);
          std::normal_distribution<double> dist_theta(0.0, 0.2);

          particles_.resize(num_particles_);
          for (auto& p : particles_) {
		p.x = dist_x(gen_);
            	p.y = dist_y(gen_);
            	p.theta = dist_theta(gen_);
            	p.weight = 1.0 / num_particles_;
        	}
    	}

    	void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
	  static nav_msgs::msg::Odometry last_odom = *msg;
          static bool initialized = false;

	  if (!initialized) {
            last_odom   = *msg;
            initialized = true;
            return;
	  }
          
	  // Calculate delta from previous odom
          double dx = msg->pose.pose.position.x - last_odom.pose.pose.position.x;
          double dy = msg->pose.pose.position.y - last_odom.pose.pose.position.y;
	  double prev_yaw = extractYaw(last_odom.pose.pose.orientation);
          double curr_yaw = extractYaw(msg->pose.pose.orientation);
          double dtheta   = wrapAngle(curr_yaw - prev_yaw);
          double trans    = std::hypot(dx, dy);
        
          double delta_rot1  = (trans > 1e-4) ? wrapAngle(std::atan2(dy, dx) - prev_yaw) : 0.0;
          double delta_trans = trans;
          double delta_rot2  = wrapAngle(dtheta - delta_rot1);

          predict(delta_rot1, delta_trans, delta_rot2);
	  last_odom = *msg;
          publish_particles();
          publish_pose_estimate(msg->header.stamp);
     	}

    	void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          extract_scan_(*msg);   // populates ScanPoints_ in base_link frame

          if (ScanPoints_.empty()) return;

          if (!map_received_) return;  // nothing to score particles against yet

          update();
          resample();
          publish_particles();
          publish_pose_estimate(msg->header.stamp);
    	}

	void predict(double delta_rot1, double delta_trans, double delta_rot2) {
          // Motion noise coefficients — start large and tighten once the filter
          // is converging. Mirror the EKF's Q_ magnitude as a starting point.
          const double alpha1 = 0.05;   // rot  noise ∝ |rot|
          const double alpha2 = 0.01;   // rot  noise ∝ |trans|
          const double alpha3 = 0.02;   // trans noise ∝ |trans|
          const double alpha4 = 0.01;   // trans noise ∝ |rot|

	  // EMA of recent rotation — update() reads this to temporarily relax
          // matching tolerance after a big turn. 0.9 ≈ a few-tick memory;
          // tune toward your actual /odom rate (closer to 1.0 = longer memory).
          const double rotation_decay = 0.9;
          recent_rotation_ = rotation_decay * recent_rotation_
                            + (std::abs(delta_rot1) + std::abs(delta_rot2));

	  for (auto& p : particles_) {
            	double std_rot1  = std::sqrt(alpha1 * delta_rot1  * delta_rot1 +
                                          alpha2 * delta_trans * delta_trans) + 1e-9;
            	double std_trans = std::sqrt(alpha3 * delta_trans * delta_trans +
                                          alpha4 * (delta_rot1 * delta_rot1 +
                                                    delta_rot2 * delta_rot2)) + 1e-9;
            	double std_rot2  = std::sqrt(alpha1 * delta_rot2  * delta_rot2 +
                                          alpha2 * delta_trans * delta_trans) + 1e-9;

            	std::normal_distribution<double> n1(0.0, std_rot1);
            	std::normal_distribution<double> n2(0.0, std_trans);
            	std::normal_distribution<double> n3(0.0, std_rot2);

            	double r1 = delta_rot1  - n1(gen_);
            	double t  = delta_trans - n2(gen_);
            	double r2 = delta_rot2  - n3(gen_);

            	p.x     += t * std::cos(p.theta + r1);
            	p.y     += t * std::sin(p.theta + r1);
            	p.theta  = wrapAngle(p.theta + r1 + r2);
          }
	}

void update() {
        // Likelihood-field observation model (Thrun/Burgard/Fox, "likelihood
        // field range finder model"; this is the same model AMCL uses).
        // Instead of extracting corners and matching them to a handful of
        // hand-surveyed landmarks, every particle scores the raw scan
        // directly against the precomputed distance-to-nearest-obstacle
        // field built from the known occupancy grid. No clustering, no line
        // fitting, no corner intersection, no nearest-landmark association —
        // the whole map is the landmark.

        // How tightly a "good" endpoint should hug an occupied cell, the
        // per-beam outlier floor, and how many beams to skip — all relaxed
        // automatically right after a big turn (recent_rotation_ large),
        // tightened back up once things settle. Without this, one fixed
        // setting has to compromise between staying sharp for normal small
        // motion and surviving a near-180° spin — this gets both.
        const double sigma_hit_base = 0.2,  sigma_hit_max_mult = 2.5;
        const double lambda_base    = 0.15, lambda_max         = 0.35;
        const int    stride_base    = 5,    stride_max_extra   = 10;
        const double rotation_window = 2.5;  // [rad] of accumulated recent
                                              // rotation counted as "fully relaxed" — tune
                                              // this against the turns that were giving you trouble

        const double relax = std::clamp(recent_rotation_ / rotation_window, 0.0, 1.0);

        const double sigma_hit   = sigma_hit_base * (1.0 + relax * (sigma_hit_max_mult - 1.0));
        const double var_hit     = sigma_hit * sigma_hit;
        const double lambda      = lambda_base + relax * (lambda_max - lambda_base);
        const int    beam_stride = stride_base + static_cast<int>(relax * stride_max_extra);

	// Robust mixture, à la the AMCL likelihood-field model:
        //   p(beam) = (1−λ)·gaussian(dist)  +  λ
        // Applied PER BEAM. This is the part that matters: a floor applied
        // once, after summing every beam's log-likelihood for the whole
        // scan, doesn't actually protect anything — dozens of capped-out
        // beams (e.g. every far point on a particle with a slightly wrong
        // heading, since they're all rotated by the same wrong Δθ) can sum
        // past -700 in log-likelihood and underflow std::exp() to exactly
        // 0.0 before the floor ever gets a say. That silent all-particles-
        // to-zero underflow on sharp turns was the actual collapse.

	double weight_sum = 0.0;

        for (auto& p : particles_) {
            double log_likelihood = 0.0;
            int    matched_count  = 0;

            for (size_t i = 0; i < ScanPoints_.size(); i += beam_stride) {
                const auto& pt = ScanPoints_[i];

                // Transform scan point from base_link (robot frame) → map
                // (world frame) using this particle's pose hypothesis.
                // Standard 2-D rigid body: p_world = R(theta) * p_robot + t
                double wx = p.x + pt.x() * std::cos(p.theta) - pt.y() * std::sin(p.theta);
                double wy = p.y + pt.x() * std::sin(p.theta) + pt.y() * std::cos(p.theta);

                // O(1) lookup into the precomputed distance transform —
                // this is the step that replaces the entire nearest-landmark
                // search loop.
                double dist;
                int mx, my;
                if (worldToMap(wx, wy, mx, my)) {
                    dist = likelihood_field_[static_cast<size_t>(my) * map_.info.width + mx];
                } else {
                    // Off the known map entirely: treat like "as far from an
                    // obstacle as the field allows" rather than discarding —
                    // keeps the gate "soft" the same way the old Cartesian
                    // gate was, without a hard cutoff.
                    dist = max_occ_dist_;
                }

                // Per-beam robust mixture — never less than log(lambda),
                // no matter how bad this single point is.
                double point_gauss = std::exp(-0.5 * (dist * dist) / var_hit);
                double point_prob  = (1.0 - lambda) * point_gauss + lambda;
                log_likelihood += std::log(point_prob);
                ++matched_count;
            }

            double gauss_w = (matched_count > 0) ? std::exp(log_likelihood) : 0.0;
            p.weight *= gauss_w;   // robustness already applied per-beam above
            weight_sum += p.weight;
        }

        // Normalise
        if (weight_sum < 1e-15) {
            // Total weight collapse: warn and reset to uniform rather than
            // produce NaN. If this fires often, increase sigma_hit or lambda,
            // or check that the map frame / particle frame actually agree
            // (e.g. a bad initial pose putting every particle off-map).
            RCLCPP_WARN(get_logger(),
                "Weight collapse — resetting to uniform. "
                "Check: initial pose | sigma_hit | map alignment");
            const double w = 1.0 / num_particles_;
            for (auto& p : particles_) p.weight = w;
        } else {
            for (auto& p : particles_) p.weight /= weight_sum;
        }
    }

    // World (map frame) → grid cell indices. Handles a rotated map origin
    // (rare for map_server output, but cheap to do correctly with the same
    // extractYaw() helper already used for odometry).
    bool worldToMap(double wx, double wy, int& mx, int& my) const {
        double ox   = map_.info.origin.position.x;
        double oy   = map_.info.origin.position.y;
        double oyaw = extractYaw(map_.info.origin.orientation);

        double dx = wx - ox;
        double dy = wy - oy;
        double c  = std::cos(-oyaw), s = std::sin(-oyaw);
        double lx =  c * dx - s * dy;
        double ly =  s * dx + c * dy;

        mx = static_cast<int>(std::floor(lx / map_.info.resolution));
        my = static_cast<int>(std::floor(ly / map_.info.resolution));

        return mx >= 0 && my >= 0 &&
               mx < static_cast<int>(map_.info.width) &&
               my < static_cast<int>(map_.info.height);
    }

    // Multi-source Dijkstra distance transform (AMCL-style "brushfire"):
    // expand outward from every occupied cell simultaneously. Each queue
    // entry carries the *original* source cell, so the distance compared at
    // every step is the true Euclidean distance to that source — not an
    // accumulated path length, which would just be a chamfer/Manhattan
    // approximation. Runs once, when the map arrives, not per scan.
    void computeLikelihoodField() {
        const int    w   = static_cast<int>(map_.info.width);
        const int    h   = static_cast<int>(map_.info.height);
        const double res = map_.info.resolution;

        std::vector<double> dist_cells(static_cast<size_t>(w) * h,
                                        std::numeric_limits<double>::infinity());

        struct QEntry { double dist; int x, y, sx, sy; };
        auto cmp = [](const QEntry& a, const QEntry& b) { return a.dist > b.dist; };
        std::priority_queue<QEntry, std::vector<QEntry>, decltype(cmp)> pq(cmp);

        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                if (map_.data[static_cast<size_t>(y) * w + x] >= occ_threshold_) {
                    size_t idx = static_cast<size_t>(y) * w + x;
                    dist_cells[idx] = 0.0;
                    pq.push({0.0, x, y, x, y});
                }
            }
        }

        const int    dx8[8] = {-1, -1, -1,  0, 0, 1, 1, 1};
        const int    dy8[8] = {-1,  0,  1, -1, 1,-1, 0, 1};
        const double max_cells = max_occ_dist_ / res;

        while (!pq.empty()) {
            QEntry cur = pq.top(); pq.pop();
            size_t idx = static_cast<size_t>(cur.y) * w + cur.x;
            if (cur.dist > dist_cells[idx] + 1e-9) continue;  // stale entry
            if (cur.dist > max_cells) continue;               // beyond cap — stop expanding here

            for (int k = 0; k < 8; ++k) {
                int nx = cur.x + dx8[k];
                int ny = cur.y + dy8[k];
                if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;

                size_t nidx = static_cast<size_t>(ny) * w + nx;
                double nd = std::hypot(static_cast<double>(nx - cur.sx),
                                        static_cast<double>(ny - cur.sy));
                if (nd < dist_cells[nidx]) {
                    dist_cells[nidx] = nd;
                    pq.push({nd, nx, ny, cur.sx, cur.sy});
                }
            }
        }

        likelihood_field_.resize(dist_cells.size());
        for (size_t i = 0; i < dist_cells.size(); ++i) {
            likelihood_field_[i] = std::min(dist_cells[i] * res, max_occ_dist_);
        }
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (map_received_) return;  // depot map is static — build the field once

        map_ = *msg;
        computeLikelihoodField();
        map_received_ = true;

        RCLCPP_INFO(get_logger(),
            "Likelihood field built: %ux%u cells @ %.3f m/cell, capped at %.2f m",
            map_.info.width, map_.info.height, map_.info.resolution, max_occ_dist_);
    }

    void resample() {
        // ESS = 1/Σwᵢ² — equals N when weights are uniform, 1 when one
        // particle holds all weight. Resample only when diversity has dropped.
        double sum_sq = 0.0;
        for (const auto& p : particles_) sum_sq += p.weight * p.weight;
        double ess = (sum_sq > 1e-30) ? 1.0 / sum_sq : 0.0;
        if (ess > 0.5 * num_particles_) return;

        std::vector<Particle> new_particles;
        new_particles.reserve(num_particles_);

        const double step = 1.0 / num_particles_;
        std::uniform_real_distribution<double> dist_uniform(0.0, step);
        double r = dist_uniform(gen_);
        double c = particles_[0].weight;   // BUG FIX #1: was particles_.weight
        int i = 0;

	// roughening noise to prevent sample impoverishment/diversity collapse and non realistic covariance
	std::normal_distribution<double> roughen_xy(0.0, 0.01);     // relative to sigma_hit
        std::normal_distribution<double> roughen_theta(0.0, 0.01);

        for (int m = 0; m < num_particles_; ++m) {
            double u = r + m * step;
            while (u > c && i < num_particles_ - 1) {
                ++i;
                c += particles_[i].weight;
            }
            Particle np = particles_[i];
	    np.x     += roughen_xy(gen_);
            np.y     += roughen_xy(gen_);
            np.theta  = wrapAngle(np.theta + roughen_theta(gen_));
            np.weight   = step;   // reset to uniform after resampling
            new_particles.push_back(np);
        }

        particles_ = std::move(new_particles);
    }

 void extract_scan_(const sensor_msgs::msg::LaserScan& scan) {
        ScanPoints_.clear();
        ScanPoints_.reserve(scan.ranges.size());

        for (size_t i = 0; i < scan.ranges.size(); i++) {
            double range = scan.ranges[i];
            if (std::isnan(range) || std::isinf(range) ||
                range < scan.range_min || range > scan.range_max)
                continue;

            double angle   = scan.angle_min + (i * scan.angle_increment);
            double x_lidar = range * std::cos(angle); // RPLidar → base_link transform (from EKF)
            double y_lidar = range * std::sin(angle);
            double x_base = -y_lidar - 0.04; 
            double y_base =  x_lidar;
            ScanPoints_.push_back(Eigen::Vector2d(x_base, y_base));
        }
    }
      	void publish_particles() {
          geometry_msgs::msg::PoseArray arr;
          arr.header.stamp = this->get_clock()->now();
          arr.header.frame_id = "map";

          for (const auto& p : particles_) {
            	geometry_msgs::msg::Pose pose;
            	pose.position.x = p.x;
            	pose.position.y = p.y;
		tf2::Quaternion q;
            q.setRPY(0.0, 0.0, p.theta);
            pose.orientation = tf2::toMsg(q);
            	arr.poses.push_back(pose);
          }
          particle_pub_->publish(arr);
      	}

      	// Collapses the particle set down to a single pose for evaluation
      	// against ground truth. x/y are a normal weighted average; theta needs
      	// a circular mean — averaging raw angles breaks down across the ±π
      	// wrap (e.g. +179° and −179° should average to ±180°, not 0°).
      	void publish_pose_estimate(const builtin_interfaces::msg::Time& stamp) {
          double mean_x = 0.0, mean_y = 0.0;
          double sum_sin = 0.0, sum_cos = 0.0;

          for (const auto& p : particles_) {
            	mean_x  += p.weight * p.x;
            	mean_y  += p.weight * p.y;
            	sum_sin += p.weight * std::sin(p.theta);
            	sum_cos += p.weight * std::cos(p.theta);
          }
          double mean_theta = std::atan2(sum_sin, sum_cos);

          // Weighted sample covariance (x, y, theta). Angle residuals are
          // wrapped for the same reason the mean needed the circular form.
          double cov_xx = 0.0, cov_yy = 0.0, cov_tt = 0.0;
          double cov_xy = 0.0, cov_xt = 0.0, cov_yt = 0.0;

          for (const auto& p : particles_) {
            	double dx = p.x - mean_x;
            	double dy = p.y - mean_y;
            	double dt = wrapAngle(p.theta - mean_theta);

            	cov_xx += p.weight * dx * dx;
            	cov_yy += p.weight * dy * dy;
            	cov_tt += p.weight * dt * dt;
            	cov_xy += p.weight * dx * dy;
            	cov_xt += p.weight * dx * dt;
            	cov_yt += p.weight * dy * dt;
          }

          geometry_msgs::msg::PoseWithCovarianceStamped est;
          est.header.stamp    = stamp;             // use the triggering message's
                                                    // stamp, not wall time, so this
                                                    // lines up with ground truth later
          est.header.frame_id = "map";             // same frame particle_pub_ uses

          est.pose.pose.position.x = mean_x;
          est.pose.pose.position.y = mean_y;

          tf2::Quaternion q;
          q.setRPY(0.0, 0.0, mean_theta);
          est.pose.pose.orientation = tf2::toMsg(q);

          // 6x6 row-major covariance over (x, y, z, roll, pitch, yaw). We only
          // have a 2-D belief, so only the x/y/yaw block is non-zero — same
          // index layout AMCL uses for /amcl_pose, so this is a drop-in
          // comparison if you've used that before.
          auto& c = est.pose.covariance;
          std::fill(c.begin(), c.end(), 0.0);
          c[0]  = cov_xx;  c[1]  = cov_xy;  c[5]  = cov_xt;
          c[6]  = cov_xy;  c[7]  = cov_yy;  c[11] = cov_yt;
          c[30] = cov_xt;  c[31] = cov_yt;  c[35] = cov_tt;

          pose_pub_->publish(est);
	  // path for rviz visualization
	  geometry_msgs::msg::PoseStamped path_pose;
	  path_pose.header = est.header;
	  path_pose.pose = est.pose.pose;

	  accumulated_pf_path_.header.stamp = est.header.stamp;
	  accumulated_pf_path_.header.frame_id = "map";
	  accumulated_pf_path_.poses.push_back(path_pose);

	  tb4_pf_path_publisher_->publish(accumulated_pf_path_);
      	}


    static double wrapAngle(double a) {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    static double extractYaw(const geometry_msgs::msg::Quaternion& q_msg) {
        tf2::Quaternion q;
        tf2::fromMsg(q_msg, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return yaw;
    }
      	int num_particles_;
    	std::vector<Particle> particles_;
    	std::mt19937 gen_;

	std::vector<Eigen::Vector2d> ScanPoints_;

	// Likelihood-field state, built once from /map.
	nav_msgs::msg::OccupancyGrid map_;
	std::vector<double> likelihood_field_;  // [m] distance to nearest occupied cell, indexed [y*width + x]
	bool   map_received_  = false;
	double max_occ_dist_  = 2.0;  // [m] cap on the distance field (and the sensor model's worst case)
	int    occ_threshold_ = 50;   // occupancy value (0-100) at/above which a cell counts as an obstacle	
	double recent_rotation_ = 0.0;  // decaying |rot1|+|rot2| accumulator — see predict()/update()

	nav_msgs::msg::Path accumulated_pf_path_;

	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tb4_pf_path_publisher_;
    	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticleFilter>());
    rclcpp::shutdown();
    return 0;
}
