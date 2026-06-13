
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <functional>
//#include <sstream>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <math.h>
#include <map>
#include <vector>
#include <random>

#include "message_filters/subscriber.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
/*#include geometry_msgs/msg/pose_with_covariance_stamped.hpp*/
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

struct Landmark { // data structure for landmarks
  double x;
  double y;
};

/*struct EndPoints { // data structure for edge detection
  bool found = false;
  Eigen::Vector2d endpoint_min;
  Eigen::Vector2d endpoint_max;
};*/

class KalmanFilter : public rclcpp::Node{
  public:
	KalmanFilter() : Node("ekf"){
	  rclcpp::QoS qos = rclcpp::QoS(10);

	  cmd_vel_sub_.subscribe(this, "cmd_vel", qos.get_rmw_qos_profile());
	  odom_sub_.subscribe(this, "odom", qos.get_rmw_qos_profile());
	  scan_sub_.subscribe(this, "scan", qos.get_rmw_qos_profile());
//	  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped("pose_ekf", 10);
//	  tb4_gt_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>("tb4_dynamic_pose", 10, std::bind(&KalmanFilter::gtPoseCallback, this, std::placeholders::_1)); // initialize tb4 ground truth pose subscriber from bridged gz sim msg
//	  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped("tb4_stamped", 10);
//	  tb4_gt_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("tb4_gt_path", 10); // initialize tb4 ground truth path publisher
//	  tb4_ekf_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("tb4_ekf_path", 10); // initialize tb4 ekf filter path publisher
	  detected_points_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("detected_points", 10); // initialize detected corner publisher (for debugging)	  

//  	  this->declare_parameter<int>("tb4_object_index", 1); // param for tb4 ground truth publisher for testing
//	  accumulated_path_.header.frame_id = "map";	  
	  
	  uint32_t queue_size = 10;
	  sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::TwistStamped, nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan>>>(message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::TwistStamped, nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan>(queue_size), cmd_vel_sub_, odom_sub_, scan_sub_); // initialize approximate time message filter for odom and cmd_vel msgs
	  
	  sync->setAgePenalty(0.50);
	  sync->registerCallback(std::bind(&KalmanFilter::syncCallback, this, _1, _2, _3)); // sync callback for message filter

	  // initialize known landmarks in global map	  
	   known_landmark_[1] = {6.3, 3.3}; // pallet corner 1
	   known_landmark_[2] = {9.48, -0.17}; // post 1
	}

  private:
	void syncCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr & cmd_vel, const nav_msgs::msg::Odometry::ConstSharedPtr & odom, const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan){
	  RCLCPP_INFO(this->get_logger(), "cmdl %f, cmda %f, odom_x %f", cmd_vel->twist.linear.x, cmd_vel->twist.linear.z, odom->pose.pose.orientation.w); // debugging
	  double cv_l = cmd_vel->twist.linear.x;
	  double cv_a = cmd_vel->twist.angular.z;
//	  double odom_x = odom->pose.pose.position.x;
//	  double odom_y = odom->pose.pose.position.y;
//	  double odom_theta = odom->pose.pose.orientation.w;
	  Eigen::Vector3d State_bar_;
	  Eigen::Matrix3d Sigma_bar_;
//	  Eigen::Matrix3d Kalman_gain_;
//	  Eigen::Vector3d Sensor_data(odom_x, odom_y, odom_theta); 

	  State_ = pose_expected_t(cv_l, cv_a);	// predict tb4 state by applying motion model
	  Sigma_bar_ = covariance_t(cv_l);	// incorporate process noise into covariance matrix

/*	  Kalman_gain_ = kalman_gain_t(Sigma_bar_); // incorporate measurement noise, covariance for sensor correction
	  State_ = pose_updated_t(State_bar_, Kalman_gain_, Sensor_data); // corrected tb4 state
	  Sigma_ = covariance_updated_t(Kalman_gain_, Sigma_bar_); // corrected state covariance matrix*/
	  extract_scan_(*scan);

	  RCLCPP_INFO(this->get_logger(), "expectedx: %f, theta: %f", State_(0), State_(2)); // debugging
// Local_points.reserve(scan->ranges.size());
	}

	// helper functions for ekf algo
	Eigen::Vector3d pose_expected_t(const double & linear_vel, const double & angular_vel){
	  Eigen::Vector3d tb4_expected;

	  RCLCPP_INFO(this->get_logger(), "received linear cmd_vel: %f, angular cmd_vel: %f, state_x: %f, state_theta: %f", linear_vel, angular_vel, State_(0), State_(2));

	  tb4_expected(0) = State_(0) + linear_vel * cos(State_(2)) * 0.05;
	  tb4_expected(1) = State_(1) + linear_vel * sin(State_(2)) * 0.05;
	  tb4_expected(2) = State_(2) + angular_vel * 0.05;
	  return tb4_expected;
	}

	Eigen::Matrix3d covariance_t(const double & linear_vel){
	  Eigen::Matrix3d tb4_covariance_expected;
	  G_(0,2) = -linear_vel*sin(State_(2)) * 0.05;
	  G_(1,2) = linear_vel*cos(State_(2)) * 0.05;
	  tb4_covariance_expected = G_ * Sigma_ * G_.transpose() + R_;
	  return tb4_covariance_expected;
	}

	// convert scan range data to coordinates
	void extract_scan_(const sensor_msgs::msg::LaserScan & scan){
	  std::vector<Eigen::Vector2d> local_points;
	  local_points.reserve(scan.ranges.size());

	  for (size_t i = 0; i < scan.ranges.size(); i++){
	  	double range = scan.ranges[i];

		// filter out implausible readings
		if (std::isnan(range) || std::isinf(range) || range < scan.range_min || range > scan.range_max){
			continue;
		}

		double angle = scan.angle_min + ( i * scan.angle_increment); // calculate beam angle in tb4 coordinate frame
		double x = range * cos(angle); // covert to cartesian coordinates in tb4 coordinate frame
		double y = range * sin(angle);
		
		local_points.push_back(Eigen::Vector2d(x, y));
	  }
	  
	  RCLCPP_INFO(this->get_logger(), "processed %zu points out of %zu beams", local_points.size(), scan.ranges.size());

	  if (!local_points.empty()){
		Clusters_ = extract_clusters_(local_points);
		std::vector<Eigen::Vector3d> Lines_;
		Eigen::Vector2d Intersection_;
		visualization_msgs::msg::Marker corner_marker;

		int corner_num = 0;
		double margin = 0.1;

		for (size_t i = 0; i < Clusters_.size(); i++){
			Lines_.push_back(extract_lines_(Clusters_[i]));
		}

		RCLCPP_INFO(this->get_logger(), "%zu lines extracted", Lines_.size());

		for (size_t j = 0; j < Lines_.size(); j++){
			for (size_t k = 0; k < j; k++){
				if (intersect_(Lines_[j], Lines_[k], Intersection_)){
					if (inside_cluster_(Intersection_, Clusters_[j], margin)){
						Corners_.push_back(Intersection_);
						corner_num += 1;
						RCLCPP_INFO(this->get_logger(), "corner #%i found at %f , and %f", corner_num, Intersection_.x(), Intersection_.y());
					}
				}
				else{
					RCLCPP_INFO(this->get_logger(), "no intersection");
				}
			}
		}
		
		corner_marker.header.frame_id = "base_link";
		corner_marker.header.stamp = this->now();
		corner_marker.ns = "detected_corners";
		corner_marker.id = 0;
		corner_marker.type = visualization_msgs::msg::Marker::POINTS;
		corner_marker.action = visualization_msgs::msg::Marker::ADD;
		corner_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
		corner_marker.scale.x = 0.3;
		corner_marker.scale.y = 0.3;
		corner_marker.color.r = 1.0f;
		corner_marker.color.g = 0.5f;
		corner_marker.color.b = 0.0f;
		corner_marker.color.a = 1.0f;

		for (const auto& Intersection_ : Corners_){
			geometry_msgs::msg::Point p;
			p.x = Intersection_.x();
			p.y = Intersection_.y();
			p.z = 0.0;
			corner_marker.points.push_back(p);
		}
		
		if (!Corners_.empty()){
			detected_points_publisher_->publish(corner_marker);
		}
	  }

	  else {
		RCLCPP_INFO(this->get_logger(), "no scan results");
	  }
	}

	// sort points into clusters
	std::vector<std::vector<Eigen::Vector2d>> extract_clusters_(const std::vector<Eigen::Vector2d> & points){
		double max_gap = 0.5;
		std::vector<std::vector<Eigen::Vector2d>> clusters;
		std::vector<Eigen::Vector2d> current;
		
		current.push_back(points[0]);
		for (size_t i = 1; i < points.size(); i++){
			if ((points[i] - points[i-1]).norm() > max_gap){
				if (current.size() >= 5 && current.size() <= 50){
				       	clusters.push_back(current);
					current.clear();
				}
			}
			current.push_back(points[i]);
		}
		if (current.size() >= 5 && current.size() <= 75){
		       	clusters.push_back(current);
		}
		RCLCPP_INFO(this->get_logger(), "%zu clusters found", clusters.size());

		return clusters;
	}

	// fit clusters into line
	Eigen::Vector3d extract_lines_(const std::vector<Eigen::Vector2d> & cluster){
		Eigen::Vector2d mean = Eigen::Vector2d::Zero();
		Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();

		for (const auto& p : cluster){
			mean += p;
		}
		mean /= cluster.size();
		for (const auto&p : cluster){
			Eigen::Vector2d d = p - mean;
			cov += d * d.transpose();
		}

		Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov, Eigen::ComputeFullU);
		Eigen::Vector2d normal = svd.matrixU().col(1);

		double c = -normal.dot(mean);
		Eigen::Vector3d line(normal.x(), normal.y(), c);
		return line;

	}

	// check for line intersections
	bool intersect_(const Eigen::Vector3d & l1, const Eigen::Vector3d & l2, Eigen::Vector2d & intersection){
		double det = l1[0] * l2[1] - l2[0] * l1[1];
		if (std::abs(det) < 1e-3){ // implies lines are parallel, do not intersect in a corner
			return false;
		}
		intersection[0] = (l1[1] * l2[2] - l2[1] * l1[2]) / det;
		intersection[1] = (l2[0] * l1[2] - l1[0] * l2[2]) / det;
		return true;
	}

	// check if line intersection is within point cluster
	bool inside_cluster_(const Eigen::Vector2d & p, const std::vector<Eigen::Vector2d> & cluster, double & margin){
    		double min_x = p.x(), max_x = p.x(), min_y = p.y(), max_y = p.y();
	    	for (const auto& pt : cluster) {
			min_x = std::min(min_x, pt.x()); max_x = std::max(max_x, pt.x());
		        min_y = std::min(min_y, pt.y()); max_y = std::max(max_y, pt.y());
    		}
		return (p.x() >= min_x - margin && p.x() <= max_x + margin && p.y() >= min_y - margin && p.y() <= max_y + margin);
	}

/*	Eigen::Matrix3d kalman_gain_t(Eigen::Matrix3d & expected_covariance){
	  Eigen::Matrix3d kg_t;

	  kg_t = expected_covariance * C_.transpose() * (C_ * expected_covariance * C_.transpose() + Q_).inverse() ;
	  return kg_t;
	}

	Eigen::Vector3d pose_updated_t(Eigen::Vector3d & tb4_expected, Eigen::Matrix3d & kalman_gain, Eigen::Vector3d & odom_data ){
	  Eigen::Vector3d tb4_corrected;
	  tb4_corrected = tb4_expected + kalman_gain * (odom_data - C_ * tb4_expected);
	  return tb4_corrected;
	}

	Eigen::Matrix3d covariance_updated_t(Eigen::Matrix3d & kalman_gain, Eigen::Matrix3d & expected_covariance){
	  Eigen::Matrix3d covariance_corrected;
	  covariance_corrected = (Eigen::Matrix3d::Identity(3,3) - kalman_gain * C_) * expected_covariance;
	  return covariance_corrected;
	}
	
	//temporary ekf path publisher for testing purposes. will comment out in favour of posestampedwithcovariance pub/sub
	void publish_pose_t(const geometry_msgs::msg::PoseStamped & msg){

	  geometry_msgs::msg::PoseStamped current_pose_stamped;
	  current_pose_stamped.header.stamp = msg.header.stamp;
	  current_pose_stamped.header.frame_id = "base_link";
	  current_pose_stamped.pose = msg.pose;

	  accumulated_kf_path_.header.stamp = this->now();
	  accumulated_kf_path_.header.frame_id = "map";
	  accumulated_kf_path_.poses.push_back(current_pose_stamped);

	  tb4_kf_path_publisher_->publish(accumulated_kf_path_);
	
	}

	// tb4 ground_truth path publisher for ekf testing purposes. will comment out in favour of ground_truth publisher in test_trajectory node
	void gtPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
	  int tb4_index = this->get_parameter("tb4_object_index").as_int();
	  if (tb4_index < 0 || static_cast<size_t>(tb4_index) >= msg->poses.size()){
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "tb4 index %d out of bounds", tb4_index);
                return;
          }
	  
	  const auto& tb4_pose = msg->poses[tb4_index];

          geometry_msgs::msg::PoseStamped current_pose_stamped;
          current_pose_stamped.header.stamp = msg->header.stamp;
          current_pose_stamped.header.frame_id = "base_link";
          current_pose_stamped.pose.position.x = tb4_pose.position.x + 8.0; // offset for difference in rviz and gz sim's coordinate system
          current_pose_stamped.pose.position.y = tb4_pose.position.y;
          current_pose_stamped.pose.position.z = tb4_pose.position.z;
          current_pose_stamped.pose.orientation.x = tb4_pose.orientation.x;
          current_pose_stamped.pose.orientation.y = tb4_pose.orientation.y;
          current_pose_stamped.pose.orientation.z = tb4_pose.orientation.z;
          current_pose_stamped.pose.orientation.w = tb4_pose.orientation.w;
          accumulated_path_.header.stamp = this->now();
          accumulated_path_.header.frame_id = "map";
          accumulated_path_.poses.push_back(current_pose_stamped);

	  tb4_gt_path_publisher_->publish(accumulated_path_);
	}

	nav_msgs::msg::Path accumulated_path_;
	nav_msgs::msg::Path accumulated_ekf_path_;*/

	std::map<int, Landmark> known_landmark_;

/*	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tb4_gt_path_publisher_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tb4_ekf_path_publisher_;*/
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr detected_points_publisher_;
//	rclcpp::Publisher<geometry_msgs::msg::PoseStampedWithCovariance>::SharedPtr pose_publisher_;
	message_filters::Subscriber<geometry_msgs::msg::TwistStamped> cmd_vel_sub_;
	message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
	message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
//	rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr tb4_gt_pose_subscriber_;
	std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::TwistStamped, nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan>>> sync;

	Eigen::Matrix3d G_ = Eigen::Matrix3d::Identity(3, 3);
/*	Eigen::Matrix3d B_ = Eigen::Matrix3d::Identity(3, 3) * 0.05; // message filter syncing cmd_vel msgs elapses approximately 50 ms btw msgs
	Eigen::Matrix3d C_ = Eigen::Matrix3d::Identity(3, 3);*/
	Eigen::Vector3d State_ = Eigen::Vector3d(0, 0, 0); // initialize robot state vector at time = 0
	Eigen::Matrix3d Sigma_ = Eigen::Matrix3d::Identity(3, 3); // initialize starting covariance to one since tb4's state is known at t = 0

	Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity(3, 3) * 0.03 ; // initial test value
	Eigen::Matrix3d Q_ = Eigen::Matrix3d::Identity(3, 3) * 0.01; // initial test value

	std::vector<std::vector<Eigen::Vector2d>> Clusters_;
	std::vector<Eigen::Vector2d> Corners_;
};

int main (int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KalmanFilter>());
  rclcpp::shutdown();
  return 0;
}
