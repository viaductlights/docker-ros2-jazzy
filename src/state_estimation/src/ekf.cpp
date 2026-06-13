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
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

struct Landmark { // data structure for landmarks
  double x;
  double y;
};

struct Segment {
  Eigen::Vector2d p1;
  Eigen::Vector2d p2;
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
	  cluster_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("clusters", 10); // cluster publisher (for debugging)
	  detected_points_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("detected_points", 10); // initialize detected corner publisher (for debugging)	  
	
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

	// helper functions for feature extraction from /scan
	void extract_scan_(const sensor_msgs::msg::LaserScan & scan){ // convert scan range data to x-y coordinates
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
		//RCLCPP_INFO(this->get_logger(), "x: %f, y: %f \n", x, y);		
		local_points.push_back(Eigen::Vector2d(x, y));
	  }
	  
	  RCLCPP_INFO(this->get_logger(), "processed %zu points out of %zu beams", local_points.size(), scan.ranges.size());

	  if (!local_points.empty()){
		Clusters_ = extract_clusters_(local_points);
		publishClusters(Clusters_, "rplidar_link", this->now()); // debugging cluster extraction
		publishEndpoints(Endpoints_, "rplidar_link", this->now()); // debugging endpoint extraction 
	  }

	  else {
		RCLCPP_INFO(this->get_logger(), "no scan results");
	  }
	}

	// sort points into clusters
	std::vector<std::vector<Eigen::Vector2d>> extract_clusters_(const std::vector<Eigen::Vector2d> & points){
	  double max_gap = 0.5;
	  double max_distance = 4.0;
	  std::vector<std::vector<Eigen::Vector2d>> clusters;
	  std::vector<Eigen::Vector2d> current;

	  current.push_back(points[0]);
	  for (size_t i = 1; i < points.size(); i++){
		if ((points[i] - points[i-1]).norm() > max_gap){
			if (current.size() >= 10){ // discard small clusters
			       	clusters.push_back(current);
				current.clear();
			}
		}
		current.push_back(points[i]);
	  }

	  if (current.size() >= 10){ // discard last detected cluster if small
	       	clusters.push_back(current);
	  }

	  auto it = clusters.begin();
	  while (it != clusters.end()) { // discard large clusters
	  	if (it->size() >= 50) {
			it = clusters.erase(it);
		} else {
			++it;
		}
	  }

	 Endpoints_.clear();

	 for (const auto& cluster : clusters){
		 if (cluster.empty()){
			 Endpoints_.push_back({Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()});
		 }
		 else{
			 Endpoints_.push_back({cluster.front(), cluster.back()});
		 }
	 }

	  for (size_t j = Endpoints_.size(); j-- > 0;) { // discard long segments
		if (too_long_(Endpoints_[j], max_distance)) {
			Endpoints_.erase(Endpoints_.begin() + j);
			clusters.erase(clusters.begin() + j);
		}
	  }

	  RCLCPP_INFO(this->get_logger(), "%zu valid clusters found, %zu endpoints found", clusters.size(), Endpoints_.size());

	  return clusters;
	}


	bool too_long_(const std::vector<Eigen::Vector2d> & segment, double max_distance) const {
		if (segment.size() < 2){
			return false;
		}
		return (segment[1] - segment[0]).norm() > max_distance;
	}

	// debugging for successful scan extraction and clustering
	void publishClusters(std::vector<std::vector<Eigen::Vector2d>> & clusters, const std::string & frame_id, rclcpp::Time stamp){
	  visualization_msgs::msg::MarkerArray marker_array;

    	  for (size_t i = 0; i < clusters.size(); ++i) {

	  if (clusters[i].empty()) continue;

          visualization_msgs::msg::Marker marker;
          marker.header.frame_id = frame_id;
          marker.header.stamp = stamp;
          marker.ns = "clusters";
          marker.id = i;                       // unique ID per cluster
          marker.type = visualization_msgs::msg::Marker::POINTS;
          marker.action = visualization_msgs::msg::Marker::ADD;
          marker.scale.x = 0.2;               // point size (meters)
          marker.scale.y = 0.2;

          // Assign a color based on cluster index (rainbow)
          // Using HSL: hue = i / clusters.size()
          float hue = static_cast<float>(i) / clusters.size();
          // Convert hue to RGB (simple approach)
          float r = std::sin(2 * M_PI * hue);
          float g = std::sin(2 * M_PI * (hue + 1.0f/3.0f));
          float b = std::sin(2 * M_PI * (hue + 2.0f/3.0f));
          // Map from [-1,1] to [0,1]
          marker.color.r = (r + 1.0f) / 2.0f;
          marker.color.g = (g + 1.0f) / 2.0f;
          marker.color.b = (b + 1.0f) / 2.0f;
          marker.color.a = 1.0f;

          // Add all points in this cluster
          for (const auto& pt : clusters[i]) {
          	geometry_msgs::msg::Point p;
		p.x = pt.x();
            	p.y = pt.y();
            	p.z = 0.0;
            	marker.points.push_back(p);
          }

          marker_array.markers.push_back(marker);
    	  }

    	  cluster_publisher_->publish(marker_array);
	}

	// debugging for endpoint extraction 
	void publishEndpoints(std::vector<std::vector<Eigen::Vector2d>> & endpoints, const std::string & frame_id, rclcpp::Time stamp){
	  visualization_msgs::msg::MarkerArray marker_array;

	  for (size_t m = 0; m < endpoints.size(); m++){
		  if (endpoints[m].empty()) continue;

		  visualization_msgs::msg::Marker marker;
		  marker.header.frame_id = frame_id;
		  marker.header.stamp = stamp;
		  marker.ns = "detected_points";
		  marker.id = m;
		  marker.type = visualization_msgs::msg::Marker::POINTS;
		  marker.action = visualization_msgs::msg::Marker::ADD;
		  marker.scale.x = 0.15;
		  marker.scale.y = 0.15;

		  float hue = static_cast<float>(m) / endpoints.size();
	          // Convert hue to RGB (simple approach)
	          float r = std::sin(2 * M_PI * hue);
	          float g = std::sin(2 * M_PI * (hue + 1.0f/3.0f));
	          float b = std::sin(2 * M_PI * (hue + 2.0f/3.0f));
	          // Map from [-1,1] to [0,1]
	          marker.color.r = (r + 1.0f) / 2.0f;
	          marker.color.g = (g + 1.0f) / 2.0f;
	          marker.color.b = (b + 1.0f) / 2.0f;
	          marker.color.a = 1.0f;

		  for (const auto& pt : endpoints[m]){
			geometry_msgs::msg::Point p;
			p.x = pt.x();
			p.y = pt.y();
			p.z = 0.0;
			marker.points.push_back(p);
		  }	
		
		  marker_array.markers.push_back(marker);
	  }

	  detected_points_publisher_->publish(marker_array);
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
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr detected_points_publisher_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_publisher_;
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
	std::vector<std::vector<Eigen::Vector2d>> Endpoints_;
};

int main (int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KalmanFilter>());
  rclcpp::shutdown();
  return 0;
}
