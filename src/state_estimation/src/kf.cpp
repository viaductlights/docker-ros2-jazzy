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
/*#include geometry_msgs/msg/pose_with_covariance_stamped.hpp*/
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class KalmanFilter : public rclcpp::Node{
  public:
	KalmanFilter() : Node("kf"){
	  rclcpp::QoS qos = rclcpp::QoS(10);

	  cmd_vel_sub_.subscribe(this, "cmd_vel", qos.get_rmw_qos_profile());
	  odom_sub_.subscribe(this, "odom", qos.get_rmw_qos_profile());
//	  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", qos, std::bind());
//	  odom_sub_ = this->create_subscription<nav_msgs::msg::Odom>("odom", 10, qos, std::bind(&KalmanFilter::odom_callback, this));
//	  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped("pose_kf", 10);
//	  timer1_ = this->create_wall_timer(500ms, std::bind(&KalmanFilter::cmdVelTimerCallback, this));
//	  timer2_ = this->create_wall_timer(550ms, std::bind(&KalmanFilter::odomTimerCallback, this));

	  uint32_t queue_size = 10;
	  sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::TwistStamped, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::TwistStamped, nav_msgs::msg::Odometry>(queue_size), cmd_vel_sub_, odom_sub_);
	  
	  sync->setAgePenalty(0.50);
	  sync->registerCallback(std::bind(&KalmanFilter::syncCallback, this, _1, _2));
	}

  private:
	void syncCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr & cmd_vel, const nav_msgs::msg::Odometry::ConstSharedPtr & odom){
	  RCLCPP_INFO(this->get_logger(), "sync callback with cmd_vel_x: %f and odom_x: %f at cmdvel time: %u and odom time: %u", cmd_vel->twist.linear.x, odom->pose.pose.position.x, cmd_vel->header.stamp.nanosec, odom->header.stamp.nanosec);
	}

/*	void cmdVelTimerCallback(){
//	  geometry_msgs::msg::TwistStamped cmd_vel;
//	  cmd_vel = msg
	  RCLCPP_INFO(this->get_logger(), "cmd_vel callback");
	  auto now = this->get_clock()->now();
	  cmd_vel.header.stamp = now;
	  cmd_vel.header.frame_id = "kf";
	}

	void odomTimerCallback(){
//	  nav_msgs::msg::Odometry odom;
	  RCLCPP_INFO(this->get_logger(), "odom_x callback");
	  auto now = this->get_clock()->now();
	  odom.header.stamp = now;
	  odom.header.frame_id = "kf";
	}*/

//	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStampled>::SharedPtr pose_pub_;
	message_filters::Subscriber<geometry_msgs::msg::TwistStamped> cmd_vel_sub_;
	message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
	std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::TwistStamped, nav_msgs::msg::Odometry>>> sync;
//	rclcpp::TimerBase::SharedPtr timer1_;
//	rclcpp::TimerBase::SharedPtr timer2_;

};

int main (int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KalmanFilter>());
  rclcpp::shutdown();
  return 0;
}

/*  	void pose_expected_t{
	}

	void covariance_t{
	}

	void kalman_gain{
	}

	void pose_updated_t{
	}

	void covariance_update_t{
	}
	
	void pose_update_t{
	}*/


