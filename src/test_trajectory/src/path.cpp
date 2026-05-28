#include <chrono>
#include <memory>
#include <thread>
#include <future>
#include <functional>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using GoalHandleNavPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

class TestPath : public rclcpp::Node{

	public:
		TestPath() : Node("path"), count_(0){
			publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

			action_client_ = rclcpp_action::create_client<NavigateThroughPoses>(
				this, "navigate_through_poses");

			timer_ = this->create_wall_timer(
				500ms, std::bind(&TestPath::publish_initial_pose, this)
			);
		}

	private:
		void publish_initial_pose(){
		  if (count_ >=2){
			timer_->cancel();
			RCLCPP_INFO(this->get_logger(), "initial pose published");
			send_nav_goal();
			return;
	  	  }
		  
	  	  auto message = geometry_msgs::msg::PoseWithCovarianceStamped();

	  	  message.header.frame_id = "map";
	 	  message.header.stamp = this->now();
	  	  //message.pose.pose.position.x = 0.0;
	  	  //message.pose.pose.position.y = 0.0;
	  	  //message.pose.pose.position.z = 0.0;
	  	  //message.pose.pose.orientation.x = 0.0;
	  	  //message.pose.pose.orientation.y = 0.0;
	  	  //message.pose.pose.orientation.z = 0.0;
	  	  message.pose.pose.orientation.w = 1.0;

	  	  publisher_->publish(message);
	  	  RCLCPP_INFO(this->get_logger(), "Initializing pose estimate");
	  	  count_++;
		}

		void send_nav_goal(){
		  if (!action_client_->wait_for_action_server(5s)){
			RCLCPP_ERROR(this->get_logger(), "action client not available");
			rclcpp::shutdown();
			return;
		  }

		  auto make_pose = [this](double x, double y, double qz, double qw){
			geometry_msgs::msg::PoseStamped p;
			p.header.frame_id = "map";
			p.header.stamp = this->now();
			p.pose.position.x = x;
			p.pose.position.y = y;
			p.pose.orientation.z = qz;
			p.pose.orientation.w = qw;
			return p;
		  };

		  auto goal_msg = NavigateThroughPoses::Goal();
		  goal_msg.poses.push_back(make_pose( 4.0, 0.0, 0.0, 1.0)); //point1
		  goal_msg.poses.push_back(make_pose( 7.0, 6.0, 0.0, 1.0)); //point2			
		  goal_msg.poses.push_back(make_pose( 7.0, 2.35, 0.0, 0.0)); //point3							       
		  RCLCPP_INFO(this->get_logger(), "Sending pose1");
		  RCLCPP_INFO(this->get_logger(), "Sending pose1");
		  
		  auto opts = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();

		  opts.goal_response_callback = [this](GoalHandleNavPoses::SharedPtr gh){
			if (!gh)
      				RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
			else
				RCLCPP_INFO(this->get_logger(), "Goal accepted, navigating");			
		  };

		  //opts.feedback_callback = [this](GoalHandleNavPoses::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback){
		  //	RCLCPP_INFO(this->get_logger(), "Goal reached");
		  //};

		  opts.result_callback = [this](const GoalHandleNavPoses::WrappedResult & result){
		  	switch (result.code) {
                    	  case rclcpp_action::ResultCode::SUCCEEDED:
                          RCLCPP_INFO(this->get_logger(),  "Path complete — all waypoints reached"); break;
                    	  case rclcpp_action::ResultCode::ABORTED:
                          RCLCPP_ERROR(this->get_logger(), "Navigation aborted");   break;
                    	  case rclcpp_action::ResultCode::CANCELED:
                          RCLCPP_WARN(this->get_logger(),  "Navigation canceled");  break;
                    	  default:
                          RCLCPP_ERROR(this->get_logger(), "Unknown result code");  break;
                	}
                	rclcpp::shutdown();
            	  };
		  
		  action_client_->async_send_goal(goal_msg, opts);	
		
		}
	
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
		rclcpp_action::Client<NavigateThroughPoses>::SharedPtr action_client_;
		int count_;
		//std::this_thread::sleep_for(std::chrono::milliseconds(500));
};

int main (int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestPath>());
  rclcpp::shutdown();
  return 0;
}

