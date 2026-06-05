
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
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using GoalHandleNavPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

class TestPath : public rclcpp::Node{

	public:
		TestPath() : Node("path"), count_(0){
			init_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10); // initialize tb4 initial pose for localization
			tb4_gt_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>("tb4_dynamic_pose", 10, std::bind(&TestPath::gtPoseCallback, this, std::placeholders::_1)); // initialize tb4 ground truth pose subscriber from bridged gz sim msg
//			tb4_gt_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("tb4_gt_pose", 10); // initialize subscribed tb4 ground truth pose publisher
			tb4_gt_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("tb4_gt_path", 10); // initialize tb4 ground truth path publisher

			this->declare_parameter<int>("tb4_object_index", 1); // parameter for tb4's index within pose array
			accumulated_path_.header.frame_id = "map";

			action_client_ = rclcpp_action::create_client<NavigateThroughPoses>( // initialize test trajectory action client
				this, "navigate_through_poses");

			timer_ = this->create_wall_timer(
				500ms, std::bind(&TestPath::publish_initial_pose, this)
			);
		}

	private:
/*		void publish_path(){

		}*/
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
		  current_pose_stamped.pose.position.x = tb4_pose.position.x + 8.0; //off set for difference in rviz and gz sim's coordinate system
		  current_pose_stamped.pose.position.y = tb4_pose.position.y;
		  current_pose_stamped.pose.position.z = tb4_pose.position.z;
		  current_pose_stamped.pose.orientation.x = tb4_pose.orientation.x;
		  current_pose_stamped.pose.orientation.y = tb4_pose.orientation.y;
		  current_pose_stamped.pose.orientation.z = tb4_pose.orientation.z;
		  current_pose_stamped.pose.orientation.w = tb4_pose.orientation.w;


		  accumulated_path_.header.stamp = this->now();
		  accumulated_path_.header.frame_id = "map";
		  accumulated_path_.poses.push_back(current_pose_stamped);

/*		  if (accumulated_path_.poses.size() > 1000){
			  accumulated_path_.poses.erase(accumulated_path_.poses.begin());
		  }*/

		  tb4_gt_path_publisher_->publish(accumulated_path_);
		}

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

	  	  init_publisher_->publish(message);
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

		nav_msgs::msg::Path accumulated_path_;	

		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_publisher_;
		rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr tb4_gt_pose_subscriber_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tb4_gt_path_publisher_;
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

