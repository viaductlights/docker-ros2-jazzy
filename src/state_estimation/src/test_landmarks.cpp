
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <functional>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <math.h>
#include <map>
#include <vector>
#include <random>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>

#include "message_filters/subscriber.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

struct Landmark { // data structure for landmarks
  double x;
  double y;
};

struct MatchedObservation{
  Eigen::VectorXd matched_point;
  int landmark_index;
  Eigen::MatrixXd S_inv; // 2x2
  Eigen::VectorXd nu; // 2x1
  Eigen::MatrixXd H; // 2x3
};

class LandmarkDebug : public rclcpp::Node{
  public:
	LandmarkDebug() : Node("test_landmarks"){
	  rclcpp::QoS qos = rclcpp::QoS(10);

	  odom_sub_.subscribe(this, "odom", qos.get_rmw_qos_profile());
	  scan_sub_.subscribe(this, "scan", qos.get_rmw_qos_profile());
	  cluster_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("clusters", 10); // cluster publisher (for debugging)
	  detected_points_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("detected_points", 10); // initialize detected corner publisher (for debugging)	  
	  landmark_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("landmarks", 10);
	  match_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("matches", 10); // initialize detected matches publisher (for debugging)	
	  
	  uint32_t queue_size = 10;
	  sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan>>>(message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan>(queue_size), odom_sub_, scan_sub_); // initialize approximate time message filter for odom and scan msgs
	  
	  sync->setAgePenalty(0.50);
	  sync->registerCallback(std::bind(&LandmarkDebug::syncCallback, this, _1, _2)); // sync callback for message filter
	  }

  private:
	void syncCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & odom, const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan){
	  RCLCPP_INFO(this->get_logger(), "odom_x %f", odom->pose.pose.position.x); // dummy for unused variable compilation error to test message filter scan results
//	  double odom_x = odom->pose.pose.position.x;
//	  double odom_y = odom->pose.pose.position.y;
//	  double odom_z = odom->pose.pose.orientation.w;
	  Eigen::Vector3d State_bar_;
	  Eigen::Matrix3d Sigma_bar_;
	  std::vector<MatchedObservation> Matched_Set_;
	  std::string lidar_frame = scan->header.frame_id;
	  geometry_msgs::msg::TransformStamped transform_stamped;

	  State_ << 0.0, 0.0, 0.0;	// only testing scan at start state
	  Sigma_bar_ = covariance_t(0.0);	//
	  extract_scan_(*scan);

	  associate_landmarks_(Known_Landmarks_, Endpoints_, State_, Sigma_bar_, Q_);
	}

	void associate_landmarks_(const std::vector<Landmark> & Known_Landmarks_, const std::vector<std::vector<Eigen::Vector2d>> & endpoints, const Eigen::Vector3d & robot_state, const Eigen::Matrix3d & covariance, const Eigen::Matrix2d & process_noise){
	  int best_match;
	  double best_distance, dx, dy, dtheta, q, distance;
      	  double CHI_GATE_THRESHOLD = 1.1; // ultra low mahalanobis gate -> high match probability necessary
	  Eigen::MatrixXd best_S_inv, best_nu, best_H;
	  Eigen::Vector2d delta, zed_hat, zed;
	  std::vector<MatchedObservation> matched_set;

	  for (size_t i = 0; i < endpoints.size() ; i++){ // for each endpoint
		for (size_t j = 0; j < endpoints[i].size(); j++){ 
			
			best_match = 0; // initializing matched_set struct
			best_distance = CHI_GATE_THRESHOLD;
			best_S_inv = Eigen::MatrixXd::Zero(2, 2);
			best_nu = Eigen::MatrixXd::Zero(2, 1);
			best_H = Eigen::MatrixXd::Zero(2, 3);
			delta = Eigen::VectorXd::Zero(2);

			for (size_t k = 0; k < Known_Landmarks_.size(); k++){ // for each landmark
				dx = Known_Landmarks_[k].x - robot_state.x();
				dy = Known_Landmarks_[k].y - robot_state.y();
				dtheta = atan2(dy, dx) - robot_state.z();
				delta << dx, dy;
				q = delta.transpose() * delta;
				zed_hat << std::sqrt(q), dtheta; // predicted range, bearing
				zed << std::sqrt(endpoints[i][j].x() * endpoints[i][j].x() + endpoints[i][j].y() * endpoints[i][j].y()), atan2(endpoints[i][j].y(), endpoints[i][j].x()); // measured ranged, bearing
				Eigen::Matrix<double, 2, 3> H = Jacobian_(q, dx, dy);
				Eigen::Matrix2d S_inv = (H * covariance * H.transpose() + process_noise).inverse();
				Eigen::Vector2d Innovation = zed - zed_hat;
				Innovation.y() = atan2(sin(Innovation.y()), cos(Innovation.y())); // bearing wrap
				distance = Innovation.transpose() * S_inv * Innovation; // Mahalanobis distance
				if (distance < best_distance){ // nearest neighbour gating
					best_match = k + 1;
					best_distance = distance;
					best_S_inv = S_inv;
					best_nu = Innovation;
					best_H = H;
				}
			}
			if (best_match != 0){
				matched_set.push_back(MatchedObservation{endpoints[i][j], best_match - 1, best_S_inv, best_nu, best_H});
				RCLCPP_INFO(this->get_logger(), "landmark %i matched to x: %f, y: %f \n", best_match - 1 , endpoints[i][j].x(), endpoints[i][j].y()); // debugging
			}
		}
	  }
	  for (const auto & match : matched_set){ // debugging matches
		  publishMatches(match.matched_point, "base_link", this->now());
	  }
	  
	  publishKnownLandmarks(Known_Landmarks_, "base_link", this->now());
	 
	}

	Eigen::Matrix<double, 2, 3> Jacobian_(double q, double dx, double dy){
	  Eigen::Matrix<double, 2, 3> H;
	  H(0,0) = -dx * std::sqrt(q);
	  H(0,1) = -dy * std::sqrt(q);
	  H(0,2) = 0;
	  H(1,0) = dy;
	  H(1,1) = -dx;
	  H(1,2) = -q;
	  return (1/q) * H;
	}

	// helper functions for mahalanobis gate covariance
	Eigen::Matrix3d covariance_t(const double & linear_vel){
	  Eigen::Matrix3d tb4_covariance_expected;
	  G_(0,2) = -linear_vel*sin(State_(2)) * 0.05;
	  G_(1,2) = linear_vel*cos(State_(2)) * 0.05;
	  tb4_covariance_expected = G_ * Sigma_ * G_.transpose() + R_;
	  return tb4_covariance_expected;
	}

	// helper functions for feature extraction from /scan
	void extract_scan_(const sensor_msgs::msg::LaserScan & scan){ // convert scan range data to x-y coordinates suitable for landmark association
	  std::vector<Eigen::Vector2d> local_points;
	  local_points.reserve(scan.ranges.size());

	  for (size_t i = 0; i < scan.ranges.size(); i++){ // populate local_points from scan data
	  	double range = scan.ranges[i];

		// filter out implausible readings
		if (std::isnan(range) || std::isinf(range) || range < scan.range_min || range > scan.range_max){
			continue;
		}

		double angle = scan.angle_min + ( i * scan.angle_increment); // calculate beam angle in tb4 coordinate frame
		double x_lidar = range * cos(angle); // covert to cartesian coordinates in tb4 coordinate frame
		double y_lidar = range * sin(angle);
		double x_base = -y_lidar - 0.04;
		double y_base = x_lidar;
		//RCLCPP_INFO(this->get_logger(), "x: %f, y: %f \n", x_base, y_base);		
		local_points.push_back(Eigen::Vector2d(x_base, y_base));
	  }
	  
	  RCLCPP_INFO(this->get_logger(), "processed %zu points out of %zu beams", local_points.size(), scan.ranges.size());

	  if (!local_points.empty()){
		Clusters_ = extract_clusters_(local_points);
		publishClusters(Clusters_, "base_link", this->now()); // debugging cluster extraction
		publishEndpoints(Endpoints_, "base_link", this->now()); // debugging endpoint extraction 
	  }

	  else {
		RCLCPP_INFO(this->get_logger(), "no scan results");
	  }
	}

	// sort points into clusters and then segment each cluster's endpoints
	std::vector<std::vector<Eigen::Vector2d>> extract_clusters_(const std::vector<Eigen::Vector2d> & points){
	  double max_gap = 0.2;
	  double max_distance = 333.0;
	  std::vector<std::vector<Eigen::Vector2d>> clusters;
	  std::vector<Eigen::Vector2d> current;

	  current.push_back(points[0]);
	  for (size_t i = 1; i < points.size(); i++){
		if ((points[i] - points[i-1]).norm() > max_gap){
			if (current.size() >= 10){ // discard small clusters dissimilar to pallet corners
			       	clusters.push_back(current);
			}
			current.clear();
		}
		current.push_back(points[i]);
	  }

	  if (current.size() >= 10){ // discard last detected cluster if small
	       	clusters.push_back(current);
	  }

	  auto it = clusters.begin();
	  while (it != clusters.end()) { // discard large clusters dissimilar to pallet corners
	  	if (it->size() >= 50) {
			it = clusters.erase(it);
		} else {
			++it;
		}
	  }

	 Endpoints_.clear();

	 for (const auto& cluster : clusters){ // extract endpoints of cluster segments
		 if (cluster.empty()){
			 Endpoints_.push_back({Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()});
		 }
		 else{
			 Endpoints_.push_back({cluster.front(), cluster.back()});
		 }
	 }

	  for (size_t j = Endpoints_.size(); j-- > 0;) { // discard long segments dissimilar to pallet corners
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


	// debugging for successful scan extraction and clustering via visualization_msgs
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

          // assign a color based on cluster index (rainbow)
          // using HSL: hue = i / clusters.size()
          float hue = static_cast<float>(i) / clusters.size();
          // convert hue to RGB 
          float r = std::sin(2 * M_PI * hue);
          float g = std::sin(2 * M_PI * (hue + 1.0f/3.0f));
          float b = std::sin(2 * M_PI * (hue + 2.0f/3.0f));
          // map from [-1,1] to [0,1]
          marker.color.r = (r + 1.0f) / 2.0f;
          marker.color.g = (g + 1.0f) / 2.0f;
          marker.color.b = (b + 1.0f) / 2.0f;
          marker.color.a = 1.0f;

          // add all points in this cluster
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

	// debugging for endpoint extraction via visualization_msgs
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
		  marker.scale.x = 0.3;
		  marker.scale.y = 0.3;

		  float hue = static_cast<float>(m) / endpoints.size();
	          // convert hue to RGB
	          float r = std::sin(2 * M_PI * hue);
	          float g = std::sin(2 * M_PI * (hue + 1.0f/3.0f));
	          float b = std::sin(2 * M_PI * (hue + 2.0f/3.0f));
	          // map from [-1,1] to [0,1]
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

	// debugging for landmark assocation
	void publishMatches(const Eigen::VectorXd & point, const std::string & frame_id, rclcpp::Time stamp){
	  visualization_msgs::msg::Marker marker;
	  
	  marker.header.frame_id = frame_id;
	  marker.header.stamp = stamp;
	  marker.ns = "matches";
	  marker.id = 0;
	  marker.type = visualization_msgs::msg::Marker::POINTS;
	  marker.action = visualization_msgs::msg::Marker::ADD;
	  marker.scale.x = 0.5;
	  marker.scale.y = 0.5;
	  marker.color.r = 1.0f; // orange for matches
	  marker.color.g = 0.5f;
	  marker.color.b = 0.0f;
	  marker.color.a = 1.0f;
	  geometry_msgs::msg::Point p;
	  p.x = point(0);
	  p.y = point(1);
	  p.z = 0.0;
	  marker.points.push_back(p);
	  
	  match_publisher_->publish(marker);
	}

	// debugging for known landmark readings
	void publishKnownLandmarks(const std::vector<Landmark> & landmarks, const std::string & frame_id, rclcpp::Time stamp){
	  visualization_msgs::msg::MarkerArray marker_array;
	  
	  for (size_t i = 0; i <landmarks.size(); ++i){
		visualization_msgs::msg::Marker marker;
	  	marker.header.frame_id = frame_id;
	  	marker.header.stamp = stamp;
	  	marker.ns = "landmarks";
	  	marker.id = static_cast<int>(i);
	  	marker.type = visualization_msgs::msg::Marker::POINTS;
	  	marker.action = visualization_msgs::msg::Marker::ADD;
	  	marker.scale.x = 0.5;
	  	marker.scale.y = 0.5;
	  	marker.color.r = 0.0f; // blue for all known landmarks
	  	marker.color.g = 0.5f;
	  	marker.color.b = 1.0f;
	  	marker.color.a = 1.0f;
		geometry_msgs::msg::Point p;
		p.x = landmarks[i].x;
		p.y = landmarks[i].y;
		p.z = 0.0;
		marker.points.push_back(p);
		marker_array.markers.push_back(marker);
	  }
	  
	  landmark_publisher_->publish(marker_array);
	}

	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr detected_points_publisher_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_publisher_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr match_publisher_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_publisher_;
	message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
	message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
	std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan>>> sync;

	Eigen::Matrix3d G_ = Eigen::Matrix3d::Identity(3, 3);
	Eigen::Vector3d State_ = Eigen::Vector3d(0, 0, 0); // initialize robot state vector at time = 0
	Eigen::Matrix3d Sigma_ = Eigen::Matrix3d::Identity(3, 3) * 0.0001; // initialize starting covariance to one since tb4's state is known at t = 0

	Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity(3, 3) * 0.00005 ; // measurement noise value
	Eigen::Matrix2d Q_ = Eigen::Matrix2d::Identity(2, 2) * 0.01; // process noise value

	std::vector<std::vector<Eigen::Vector2d>> Clusters_;
	std::vector<std::vector<Eigen::Vector2d>> Endpoints_;
	std::vector<Landmark> Known_Landmarks_ = { // landmarks are selected pallet corners
	  {5.94, 4.3}, // p1 - p5
	  {7.76, 3.94},
	  {8.93, -1.67},
	  {11.8, -1.78},
	  {17.2, -1.65},
	};
	std::vector<Landmark> Landmarks_tb4_;
};

int main (int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandmarkDebug>());
  rclcpp::shutdown();
  return 0;
}
