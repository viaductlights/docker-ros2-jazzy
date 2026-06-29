#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <functional>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <math.h>
#include <vector>

#include "message_filters/subscriber.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
//using std::placeholders::_3;

class ExtendedKalmanFilterOdom : public rclcpp::Node{
  public:
	ExtendedKalmanFilterOdom() : Node("ekfo"){
	  rclcpp::QoS qos = rclcpp::QoS(10);

	  cmd_vel_sub_.subscribe(this, "cmd_vel", qos.get_rmw_qos_profile());
	  odom_sub_.subscribe(this, "odom", qos.get_rmw_qos_profile());
	  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_ekfo", 10); // initialize state estimation publisher
	  pose_pub_dr_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_ekf_dr", 10); // initialize dead reckoning pose publisher
//	  tb4_gt_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>("tb4_dynamic_pose", 10, std::bind(&ExtendedKalmanFilterOdom::gtPoseCallback, this, std::placeholders::_1)); // initialize tb4 ground truth pose subscriber from bridged gz sim msg
//	  tb4_gt_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("tb4_gt_path", 10); // initialize tb4 ground truth path publisher
	  tb4_dr_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("tb4_ekf_dr_path", 10); // initialize tb4 dead reckoning path publisher (no ekf)
	  tb4_ekfo_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("tb4_ekfo_path", 10); // initialize tb4 ekf (odom sensor fusion)  path publisher
//  	  this->declare_parameter<int>("tb4_object_index", 1); // param for tb4 ground truth publisher for testing
	  
          // process and measurement noise variables      
          this->declare_parameter<double>("r1", 0.05);
          this->declare_parameter<double>("r2", 0.05);
          this->declare_parameter<double>("r3", 0.05);
          this->declare_parameter<double>("q1", 0.10);
          this->declare_parameter<double>("q2", 0.10);
          this->declare_parameter<double>("q3", 0.10);
          // baseline R: 0.05, Q: 0.10
          // trust measurement R: 0.3, Q: 0.001
          // trust prediction R: 0.001, Q: 0.5
          // non isotropic distrust theta R_.diagonal() << 0.05, 0.05, 0.05; Q_.diagonal() << 0.01, 0.01, 0.5;
          // non isotropic trust theta R_.diagonal() << 0.05, 0.05, 0.05; Q_diagonal() << 0.01, 0.01, 0.001;
          R_.diagonal() << this->get_parameter("r1").as_double(),
                        this->get_parameter("r2").as_double(),
                        this->get_parameter("r3").as_double();
          Q_.diagonal() << this->get_parameter("q1").as_double(),
                        this->get_parameter("q2").as_double(),
                        this->get_parameter("q3").as_double();
	  
	  uint32_t queue_size = 10;
	  sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::TwistStamped, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::TwistStamped, nav_msgs::msg::Odometry>(queue_size), cmd_vel_sub_, odom_sub_); // initialize approximate time message filter for odom and scan msgs
	  
	  sync->setAgePenalty(0.50);
	  sync->registerCallback(std::bind(&ExtendedKalmanFilterOdom::syncCallback, this, _1, _2)); // sync callback for message filter
	  }

  private:
	void syncCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr & cmd_vel, const nav_msgs::msg::Odometry::ConstSharedPtr & odom){
	  //RCLCPP_INFO(this->get_logger(), "odom_time: %d, %u", odom->header.stamp.sec, odom->header.stamp.nanosec); // debugging
	  double vel_l = cmd_vel->twist.linear.x;
	  double vel_a = cmd_vel->twist.angular.z;
	  double odom_x = odom->pose.pose.position.x;
	  double odom_y = odom->pose.pose.position.y;
	  double odom_theta = tf2::getYaw(odom->pose.pose.orientation);

	  Eigen::Vector3d State_bar_;
	  Eigen::Matrix3d Sigma_bar_;
	  Eigen::Vector3d Sensor_data(odom_x, odom_y, odom_theta);
	  Eigen::Vector3d nu_; // innovation
	  
	  Eigen::Matrix3d Kalman_gain_;
	  Eigen::Matrix3d S_;
	  Eigen::Matrix3d Joseph_helper_;
	  
	  // setting time gain dt
	  static rclcpp::Time last_time = cmd_vel->header.stamp;
	  rclcpp::Time current_time = cmd_vel->header.stamp;
	  double dt = (current_time - last_time).seconds();
	  last_time = current_time;
	  if (dt <= 0.0) dt = 0.1;

	  State_bar_ = pose_expected_t(State_, vel_l, vel_a, dt);	// predict tb4 state by applying motion model
	  //RCLCPP_INFO(this->get_logger(), "expectedx: %f, theta: %f", State_bar_(0), State_bar_(2)); // debugging
	  
	  State_dr_ = pose_expected_t(State_dr_, vel_l, vel_a, dt);	// pure dead-reckoning baseline; never touched by any correction loops
	  geometry_msgs::msg::PoseStamped pose_msg; 
          pose_msg.header.stamp = this->now();
          pose_msg.header.frame_id = "map";
          pose_msg.pose.position.x = State_dr_(0);
          pose_msg.pose.position.y = State_dr_(1);
	  tf2::Quaternion q_pure; // quaternion conversion
	  q_pure.setRPY(0, 0, State_dr_(2));
          pose_msg.pose.orientation = tf2::toMsg(q_pure);
          publish_path_dr(pose_msg); // dead reckoning path publisher

	  publish_pose_covariance_dr();	  // dead reckoning pose with covariance publisher


	  Sigma_bar_ = covariance_t(vel_l, State_(2), vel_a, dt);	// incorporate process noise into covariance matrix
	  nu_ = Sensor_data - State_bar_; // compute innovation
	  nu_(2) = atan2(sin(nu_(2)), cos(nu_(2))); // angle wrap
	  S_ = H_ * Sigma_bar_ * H_.transpose() + Q_;
	  Kalman_gain_ = Sigma_bar_ * H_.transpose() * S_.inverse(); // Kalman_gain_
	  State_ = State_bar_ + Kalman_gain_ * nu_; // corrected tb4 state
	  State_(2) = atan2(sin(State_(2)), cos(State_(2)));
	  Joseph_helper_ = Eigen::Matrix3d::Identity() - Kalman_gain_ * H_; // for expressing covariance update in Joseph form
	  Sigma_ = Joseph_helper_ * Sigma_bar_ * Joseph_helper_.transpose() + Kalman_gain_ * Q_ * Kalman_gain_.transpose();
	  Sigma_ = 0.5 * (Sigma_ + Sigma_.transpose()); // symmetrize
//	  Sigma_ *= 100.0; // rviz covariance display debugging
			    
	  publish_pose_covariance(); // publishes to /pose_ekfo

	  geometry_msgs::msg::PoseStamped pose_msg_o; // tb4 state estimation path publisher for visualization
          pose_msg_o.header.stamp = this->now();
          pose_msg_o.header.frame_id = "map";
          pose_msg_o.pose.position.x = State_(0);
          pose_msg_o.pose.position.y = State_(1);
	  tf2::Quaternion q_corrected;
	  q_corrected.setRPY(0, 0, State_(2));
          pose_msg_o.pose.orientation = tf2::toMsg(q_corrected);
          publish_path_t(pose_msg_o);
	}

	// helper functions for ekf algo
	 
	// takes the previous state explicitly so it can be reused for both the corrected state and an independent dead-reckoning-only state (see State_dr_ below)
	Eigen::Vector3d pose_expected_t(const Eigen::Vector3d & prev_state, const double & linear_vel, const double & angular_vel, const double & time_step){
	  Eigen::Vector3d tb4_expected;
	// RCLCPP_INFO(this->get_logger(), "received linear odom: %f, angular odom: %f, state_x: %f, state_theta: %f", linear_vel, angular_vel, prev_state(0), prev_state(2)); // debugging
	  tb4_expected(0) = prev_state(0) + linear_vel * cos(prev_state(2)) * time_step;
	  tb4_expected(1) = prev_state(1) + linear_vel * sin(prev_state(2)) * time_step;
	  tb4_expected(2) = prev_state(2) + angular_vel * time_step;
	  tb4_expected(2) = atan2(sin(tb4_expected(2)), cos(tb4_expected(2))); // wrap heading
	  return tb4_expected;
	}

	Eigen::Matrix3d covariance_t(const double & linear_vel, const double & previous_theta, const double & angular_vel, const double & time_step){
	  Eigen::Matrix3d G = Eigen::Matrix3d::Identity(); // tb4 covariance expected
	  double theta = previous_theta + angular_vel * time_step;
	  G(0,2) = -linear_vel*sin(theta) * time_step;
	  G(1,2) = linear_vel*cos(theta) * time_step;
	  G = G * Sigma_ * G.transpose() + R_;
	  return G;
	}

	void publish_pose_covariance_dr(){
	  geometry_msgs::msg::PoseWithCovarianceStamped msg;
	  
	  msg.header.stamp = this->now();
    	  msg.header.frame_id = "map";
    	  msg.pose.pose.position.x = State_dr_(0);
    	  msg.pose.pose.position.y = State_dr_(1);

    	  tf2::Quaternion q;
    	  q.setRPY(0,0,State_dr_(2));

    	  msg.pose.pose.orientation = tf2::toMsg(q);

    	  std::fill(
        	msg.pose.covariance.begin(),
        	msg.pose.covariance.end(),
        	0.0);

	  pose_pub_dr_->publish(msg);
	}


	void publish_pose_covariance(){
	  geometry_msgs::msg::PoseWithCovarianceStamped msg;
	  
	  msg.header.stamp = this->now();
    	  msg.header.frame_id = "map";
    	  msg.pose.pose.position.x = State_(0);
    	  msg.pose.pose.position.y = State_(1);

    	  tf2::Quaternion q;
    	  q.setRPY(0,0,State_(2));

    	  msg.pose.pose.orientation = tf2::toMsg(q);

    	  std::fill(
        	msg.pose.covariance.begin(),
        	msg.pose.covariance.end(),
        	0.0);

    	  msg.pose.covariance[0]  = Sigma_(0,0);
/*    	  msg.pose.covariance[1]  = Sigma_(0,1);
	  msg.pose.covariance[5]  = Sigma_(0,2);
    	  msg.pose.covariance[6]  = Sigma_(1,0);*/
    	  msg.pose.covariance[7]  = Sigma_(1,1);
/*	  msg.pose.covariance[11] = Sigma_(1,2);
	  msg.pose.covariance[30] = Sigma_(2,0);
	  msg.pose.covariance[31] = Sigma_(2,1);*/
    	  msg.pose.covariance[35] = Sigma_(2,2);
	  
	  pose_pub_->publish(msg);
	}

	// ekf path publisher for visualization purposes. publishes to /tb4_ekfo_path
	void publish_path_t(const geometry_msgs::msg::PoseStamped & msg){

	  geometry_msgs::msg::PoseStamped current_pose_o_stamped;
	  current_pose_o_stamped.header.stamp = msg.header.stamp;
	  current_pose_o_stamped.header.frame_id = "map";
	  current_pose_o_stamped.pose = msg.pose;

	  accumulated_ekfo_path_.header.stamp = this->now();
	  accumulated_ekfo_path_.header.frame_id = "map";
	  accumulated_ekfo_path_.poses.push_back(current_pose_o_stamped);

	  tb4_ekfo_path_publisher_->publish(accumulated_ekfo_path_);
	}

	void publish_path_dr(const geometry_msgs::msg::PoseStamped & msg){ // dead reckoning path publisher

	  geometry_msgs::msg::PoseStamped current_pose_stamped;
	  current_pose_stamped.header.stamp = msg.header.stamp;
	  current_pose_stamped.header.frame_id = "map";
	  current_pose_stamped.pose = msg.pose;

	  accumulated_ekf_dr_path_.header.stamp = this->now();
	  accumulated_ekf_dr_path_.header.frame_id = "map";
	  accumulated_ekf_dr_path_.poses.push_back(current_pose_stamped);

	  tb4_dr_path_publisher_->publish(accumulated_ekf_dr_path_);
	}

	// tb4 ground_truth path publisher for ekf testing purposes. will comment out in favour of ground_truth publisher in test_trajectory node
/*	void gtPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
	  int tb4_index = this->get_parameter("tb4_object_index").as_int();
	  if (tb4_index < 0 || static_cast<size_t>(tb4_index) >= msg->poses.size()){
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "tb4 index %d out of bounds", tb4_index);
                return;
          }
	  
	  const auto& tb4_pose = msg->poses[tb4_index];

          geometry_msgs::msg::PoseStamped current_pose_stamped;
          current_pose_stamped.header.stamp = msg->header.stamp;
          current_pose_stamped.header.frame_id = "map";
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
	}*/

//	nav_msgs::msg::Path accumulated_path_;
	nav_msgs::msg::Path accumulated_ekfo_path_;
	nav_msgs::msg::Path accumulated_ekf_dr_path_;

	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_dr_;
//	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tb4_gt_path_publisher_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tb4_dr_path_publisher_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tb4_ekfo_path_publisher_;
	message_filters::Subscriber<geometry_msgs::msg::TwistStamped> cmd_vel_sub_;
	message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
//	rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr tb4_gt_pose_subscriber_; // comment out once launching w test_trajectories file
	std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::TwistStamped, nav_msgs::msg::Odometry>>> sync;

	Eigen::Matrix3d H_ = Eigen::Matrix3d::Identity(3, 3); // measurement jacobian
	Eigen::Vector3d State_ = Eigen::Vector3d(0, 0, 0); // initialize robot state vector at time = 0
	Eigen::Vector3d State_dr_ = Eigen::Vector3d(0, 0, 0); // pure dead-reckoning baseline (no landmark correction); predicted every cycle, never corrected
	Eigen::Matrix3d Sigma_ = Eigen::Matrix3d::Identity(3, 3) * 0.01; // initialize starting covariance to small number since tb4's state is known at t = 0

	Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity(3, 3); // process noise value
	Eigen::Matrix3d Q_ = Eigen::Matrix3d::Identity(3, 3); // measurement noise value
};

int main (int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExtendedKalmanFilterOdom>());
  rclcpp::shutdown();
  return 0;
}
