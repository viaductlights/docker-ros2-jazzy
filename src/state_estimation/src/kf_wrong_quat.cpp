#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <functional>
//#include <sstream>
#include <Eigen/Dense>

#include "message_filters/subscriber.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class KalmanFilterQuatError : public rclcpp::Node{
  public:
	KalmanFilterQuatError() : Node("kf_qe"){
	  rclcpp::QoS qos = rclcpp::QoS(10);

	  cmd_vel_sub_.subscribe(this, "cmd_vel", qos.get_rmw_qos_profile());
	  odom_sub_.subscribe(this, "odom", qos.get_rmw_qos_profile());
	  kt_publisher_ = this->create_publisher<std_msgs::msg::Float32>("kt", 10);
	  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_kf_qe", 10);
//	  tb4_gt_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>("tb4_dynamic_pose", 10, std::bind(&KalmanFilterQuatError::gtPoseCallback, this, std::placeholders::_1)); // initialize tb4 ground truth pose subscriber from bridged gz sim msg
//	  tb4_gt_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("tb4_gt_path", 10); // initialize tb4 ground truth path publisher
	  tb4_kf_noquat_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("tb4_kf_qe_path", 10); // initialize tb4 kf filter path publisher
	
  	  this->declare_parameter<int>("tb4_object_index", 1); // param for tb4 ground truth publisher for testing
	  this->declare_parameter<double>("gz_x_offset", 8.0); // param for offsetting different between rviz and gz coordinates
	  
	  uint32_t queue_size = 10;
	  sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::TwistStamped, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::TwistStamped, nav_msgs::msg::Odometry>(queue_size), cmd_vel_sub_, odom_sub_); // initialize approximate time message filter for odom and cmd_vel msgs
	  
	  sync->setAgePenalty(0.50);
	  sync->registerCallback(std::bind(&KalmanFilterQuatError::syncCallback, this, _1, _2)); // sync callback for message filter
	}

  private:
	void syncCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr & cmd_vel, const nav_msgs::msg::Odometry::ConstSharedPtr & odom){
//	  RCLCPP_INFO(this->get_logger(), "sync callback with cmdl time %u and odom time %u \n cmdl %f, cmda %f, odomx %f, y %f, theta %f", cmd_vel->header.stamp.nanosec, odom->header.stamp.nanosec, cmd_vel->twist.linear.x, cmd_vel->twist.angular.z, odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.orientation.w); // debugging
	   if (!initialized_){ // initializing check for first run
		  last_stamp_ = rclcpp::Time(cmd_vel->header.stamp);
		  initialized_ = true;
		  return;
	  }
	  double cv_l = cmd_vel->twist.linear.x;
	  double cv_a = cmd_vel->twist.angular.z;
	  double odom_x = odom->pose.pose.position.x;
	  double odom_y = odom->pose.pose.position.y;
	  double odom_theta = odom->pose.pose.orientation.w;

	  double dt = (rclcpp::Time(cmd_vel->header.stamp) - last_stamp_).seconds();
	  last_stamp_ = rclcpp::Time(cmd_vel->header.stamp);

	  Eigen::Vector3d State_bar_;
	  Eigen::Matrix3d Sigma_bar_;
	  Eigen::Matrix3d Kalman_gain_;
	  Eigen::Vector3d Sensor_data(odom_x, odom_y, odom_theta); 

	  State_bar_ = pose_expected_t(cv_l, cv_a, dt);	// predict tb4 state by applying motion model
	  Sigma_bar_ = covariance_t();	// incorporate process noise into covariance matrix
	  Kalman_gain_ = kalman_gain_t(Sigma_bar_); // incorporate measurement noise, covariance for sensor correction
	  State_ = pose_updated_t(State_bar_, Kalman_gain_, Sensor_data); // corrected tb4 state
	  State_(2) = atan2(sin(State_(2)), cos(State_(2))); // normalize in case of drift beyond [-pi, pi]								  // 
	  Sigma_ = covariance_updated_t(Kalman_gain_, Sigma_bar_); // corrected state covariance matrix

	  publish_pose_covariance(); // publishes to /pose_kf_qe

	  //RCLCPP_INFO(this->get_logger(), "expectedx: %f, y: %f, theta: %f, pose x: %f, y: %f, theta: %f, covariance 1: %f, 2: %f, 3: %f", State_bar_(0), State_bar_(1), State_bar_(2), State_(0), State_(1), State_(2), Sigma_(0,0), Sigma_(1,1), Sigma_(2,2)); // debugging
	  // kalman gain publisher
	  double kt = Kalman_gain_(0,0);
	  std_msgs::msg::Float32 kt_msg;
	  kt_msg.data = kt;
	  kt_publisher_->publish(kt_msg);

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
	Eigen::Vector3d pose_expected_t(const double & linear_vel, const double & angular_vel, const double & dt){
	  Eigen::Vector3d tb4_expected;
	  Control_(0) = linear_vel;
	  Control_(2) = angular_vel;
	  tb4_expected = A_ * State_ + B_ * dt * Control_;
	  return tb4_expected;
	}

	Eigen::Matrix3d covariance_t(){
	  Eigen::Matrix3d tb4_covariance_expected;
	  tb4_covariance_expected = A_ * Sigma_ * A_.transpose() + Q_;
	  return tb4_covariance_expected;
	}

	Eigen::Matrix3d kalman_gain_t(Eigen::Matrix3d & expected_covariance){
	  Eigen::Matrix3d kg_t;

	  kg_t = expected_covariance * C_.transpose() * (C_ * expected_covariance * C_.transpose() + R_).inverse() ;
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

	void publish_pose_covariance(){
          geometry_msgs::msg::PoseWithCovarianceStamped msg;

          msg.header.stamp = this->now();
          msg.header.frame_id = "map";
          msg.pose.pose.position.x = State_(0);
          msg.pose.pose.position.y = State_(1);
          msg.pose.pose.orientation.w = State_(2); // wrong: yaw assigned directly to quaternion w component

          std::fill(
                msg.pose.covariance.begin(),
                msg.pose.covariance.end(),
                0.0);

          msg.pose.covariance[0]  = Sigma_(0,0);
          msg.pose.covariance[7]  = Sigma_(1,1);
          msg.pose.covariance[35] = Sigma_(2,2);

          pose_pub_->publish(msg);
        }
	
	//kf path publisher for visualization purposes
	void publish_pose_t(const geometry_msgs::msg::PoseStamped & msg){

	  geometry_msgs::msg::PoseStamped current_pose_stamped;
	  current_pose_stamped.header.stamp = msg.header.stamp;
	  current_pose_stamped.header.frame_id = "map";
	  current_pose_stamped.pose = msg.pose;

	  accumulated_kf_path_.header.stamp = this->now();
	  accumulated_kf_path_.header.frame_id = "map";
	  accumulated_kf_path_.poses.push_back(current_pose_stamped);

	  tb4_kf_noquat_path_publisher_->publish(accumulated_kf_path_);
	}

	bool initialized_ = false;
	rclcpp::Time last_stamp_;

	nav_msgs::msg::Path accumulated_kf_path_;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr kt_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tb4_kf_noquat_path_publisher_;
	message_filters::Subscriber<geometry_msgs::msg::TwistStamped> cmd_vel_sub_;
	message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
	std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::TwistStamped, nav_msgs::msg::Odometry>>> sync;

	Eigen::Matrix3d A_ = Eigen::Matrix3d::Identity(3, 3);
	Eigen::Matrix3d B_ = Eigen::Matrix3d::Identity(3, 3);
//	Eigen::Matrix3d B_ = Eigen::Matrix3d::Identity(3, 3) * 0.05; // message filter syncing cmd_vel msgs elapses approximately 50 ms btw msgs, replaced w dt
	Eigen::Matrix3d C_ = Eigen::Matrix3d::Identity(3, 3);
	Eigen::Vector3d State_ = Eigen::Vector3d(0, 0, 0); // initialize robot state vector at time = 0
	Eigen::Vector3d Control_ = Eigen::Vector3d(0, 0, 0); // initialize control vector at time = 0						   
	Eigen::Matrix3d Sigma_ = Eigen::Matrix3d::Identity(3, 3) * 0.001 ; // initialize starting covariance to small num since tb4's state is known at t = 0

	Eigen::Matrix3d Q_ = Eigen::Matrix3d::Identity(3, 3) * 0.01; // process noise value
	Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity(3, 3) * 0.001; // measurement noise value
};

int main (int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KalmanFilterQuatError>());
  rclcpp::shutdown();
  return 0;
}
