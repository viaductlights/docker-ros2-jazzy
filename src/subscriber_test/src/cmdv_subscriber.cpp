#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CmdvSubscriber : public rclcpp::Node{
public:
	CmdvSubscriber() : Node("cmdv_subscriber"){
		auto topic_callback = [this](geometry_msgs::msg::Twist::UniquePtr msg) -> void{
		RCLCPP_INFO(this->get_logger(), 
				"cmd_vel: (%f, %f, %f) (%f, %f, %f)",
				msg->linear.x, msg->linear.y, msg->linear.z,
				msg->angular.x, msg->angular.y, msg->angular.z);
		};
	subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, topic_callback);
	}
private:
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main (int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdvSubscriber>());
  rclcpp::shutdown();
  return 0;
}

