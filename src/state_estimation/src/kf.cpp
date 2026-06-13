#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <functional>
//#include <sstream>
#include <Eigen/Dense>

#include "message_filters/subscriber.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
/*#include geometry_msgs/msg/pose_with_covariance_stamped.hpp*/
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class KalmanFilter : public rclcpp::Node{
  public:
	KalmanFilter() : Node("kf"){
	  rclcpp::QoS qos = rclcpp::QoS(10);

	  cmd_vel_sub_.subscribe(this, "cmd_vel", qos.get_rmw_qos_profile());
	  odom_sub_.subscribe(this, "odom", qos.get_rmw_qos_profile());
//	  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped("pose_kf", 10);
	  tb4_gt_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>("tb4_dynamic_pose", 10, std::bind(&KalmanFilter::gtPoseCallback, this, std::placeholders::_1)); // initialize tb4 ground truth pose subscriber from bridged gz sim msg
//	  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped("tb4_stamped", 10);
	  tb4_gt_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("tb4_gt_path", 10); // initialize tb4 ground truth path publisher
	  tb4_kf_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("tb4_kf_path", 10); // initialize tb4 kf filter path publisher
	
  	  this->declare_parameter<int>("tb4_object_index", 1); // param for tb4 ground truth publisher for testing
	  accumulated_path_.header.frame_id = "map";	  
	  
	  uint32_t queue_size = 10;
	  sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::TwistStamped, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::TwistStamped, nav_msgs::msg::Odometry>(queue_size), cmd_vel_sub_, odom_sub_); // initialize approximate time message filter for odom and cmd_vel msgs
	  
	  sync->setAgePenalty(0.50);
	  sync->registerCallback(std::bind(&KalmanFilter::syncCallback, this, _1, _2)); // sync callback for message filter
	}

  private:
	void syncCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr & cmd_vel, const nav_msgs::msg::Odometry::ConstSharedPtr & odom){
	  RCLCPP_INFO(this->get_logger(), "sync callback with cmdl time %u and odom time %u \n cmdl %f, cmda %f, odomx %f, y %f, theta %f", cmd_vel->header.stamp.nanosec, odom->header.stamp.nanosec, cmd_vel->twist.linear.x, cmd_vel->twist.linear.z, odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.orientation.w); // debugging
	  double cv_l = cmd_vel->twist.linear.x;
	  double cv_a = cmd_vel->twist.angular.z;
	  double odom_x = odom->pose.pose.position.x;
	  double odom_y = odom->pose.pose.position.y;
	  double odom_theta = odom->pose.pose.orientation.w;
	  Eigen::Vector3d State_bar_;
	  Eigen::Matrix3d Sigma_bar_;
	  Eigen::Matrix3d Kalman_gain_;
	  Eigen::Vector3d Sensor_data(odom_x, odom_y, odom_theta); 

	  State_bar_ = pose_expected_t(cv_l, cv_a);	// predict tb4 state by applying motion model
	  Sigma_bar_ = covariance_t();	// incorporate process noise into covariance matrix
	  Kalman_gain_ = kalman_gain_t(Sigma_bar_); // incorporate measurement noise, covariance for sensor correction
	  State_ = pose_updated_t(State_bar_, Kalman_gain_, Sensor_data); // corrected tb4 state
	  Sigma_ = covariance_updated_t(Kalman_gain_, Sigma_bar_); // corrected state covariance matrix

	  RCLCPP_INFO(this->get_logger(), "expectedx: %f, y: %f, theta: %f, pose x: %f, y: %f, theta: %f, covariance 1: %f, 2: %f, 3: %f", State_bar_(0), State_bar_(1), State_bar_(2), State_(0), State_(1), State_(2), Sigma_(0,0), Sigma_(1,1), Sigma_(2,2)); // debugging

	  // temporary pose publisher for testing. will comment out in favour of posestampedwithcovariance msg pub
	  geometry_msgs::msg::PoseStamped pose_msg;
	  pose_msg.header.stamp = this->now();
	  pose_msg.header.frame_id = "map";
	  pose_msg.pose.position.x = State_(0);
	  pose_msg.pose.position.y = State_(1);
	  pose_msg.pose.orientation.w = State_(2);
	  publish_pose_t(pose_msg);
	}

	// helper functions for kf algo
	Eigen::Vector3d pose_expected_t(const double & linear_vel, const double & angular_vel){
	  Eigen::Vector3d tb4_expected;
	  Control_(0) = linear_vel;
	  Control_(2) = angular_vel;
	  tb4_expected = A_ * State_ + B_ * Control_;
	  return tb4_expected;
	}

	Eigen::Matrix3d covariance_t(){
	  Eigen::Matrix3d tb4_covariance_expected;
	  tb4_covariance_expected = A_ * Sigma_ * A_.transpose() + R_;
	  return tb4_covariance_expected;
	}

	Eigen::Matrix3d kalman_gain_t(Eigen::Matrix3d & expected_covariance){
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
	
	//temporary kf path publisher for testing purposes. will comment out in favour of posestampedwithcovariance pub/sub
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

	// tb4 ground_truth path publisher for kf testing purposes. will comment out in favour of ground_truth publisher in test_trajectory node
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
	nav_msgs::msg::Path accumulated_kf_path_;

	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tb4_gt_path_publisher_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tb4_kf_path_publisher_;
//	rclcpp::Publisher<geometry_msgs::msg::PoseStampedWithCovariance>::SharedPtr pose_publisher_;
	message_filters::Subscriber<geometry_msgs::msg::TwistStamped> cmd_vel_sub_;
	message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr tb4_gt_pose_subscriber_;
	std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::TwistStamped, nav_msgs::msg::Odometry>>> sync;

	Eigen::Matrix3d A_ = Eigen::Matrix3d::Identity(3, 3);
	Eigen::Matrix3d B_ = Eigen::Matrix3d::Identity(3, 3) * 0.05; // message filter syncing cmd_vel msgs elapses approximately 50 ms btw msgs
	Eigen::Matrix3d C_ = Eigen::Matrix3d::Identity(3, 3);
	Eigen::Vector3d State_ = Eigen::Vector3d(0, 0, 1); // initialize robot state vector at time = 0
	Eigen::Vector3d Control_ = Eigen::Vector3d(0, 0, 0); // initialize control vector at time = 0						   
	Eigen::Matrix3d Sigma_ = Eigen::Matrix3d::Identity(3, 3); // initialize starting covariance to one since tb4's state is known at t = 0

	Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity(3, 3) * 0.03 ; // initial test value
	Eigen::Matrix3d Q_ = Eigen::Matrix3d::Identity(3, 3) * 0.01; // initial test value
};

int main (int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KalmanFilter>());
  rclcpp::shutdown();
  return 0;
}

// depreciated functions from early stages testing
/*	void cmdVelTimerCallback(){
	  RCLCPP_INFO(this->get_logger(), "cmd_vel callback");
	}

	void odomTimerCallback(){
	  nav_msgs::msg::Odometry odom;
	  RCLCPP_INFO(this->get_logger(), "odom_x callback");
	  auto now = this->get_clock()->now();
	  odom.header.stamp = now;
	  odom.header.frame_id = "kf";
	}*/


