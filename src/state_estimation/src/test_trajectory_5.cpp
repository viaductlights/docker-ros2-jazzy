

#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavToPose   = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// NavigateToPose planner - short trajectory

// ---------------------------------------------------------------------------
// Startup sequence
//   AWAITING_READINESS    → poll every 1 s (wall clock) until all three gates
//                           pass; then transition to PUBLISHING_INIT_POSE.
//   PUBLISHING_INIT_POSE  → publish /initialpose twice at 500 ms intervals;
//                           then transition to WAITING_FOR_NAV2_ACTIVE.
//   WAITING_FOR_NAV2_ACTIVE → one-shot wall timer of `nav_goal_delay_ms` ms.
//                           action_server_is_ready() returns true as soon as
//                           the ROS action interface exists, but bt_navigator
//                           is a lifecycle node that can still be INACTIVE
//                           (costmaps loading, etc.) and will reject goals.
//                           This delay gives Nav2 time to reach ACTIVE.
//                           then transition to NAVIGATING.
//   NAVIGATING            → goal sent, waiting for action result.
//
// Three readiness gates (non-blocking):
//   [AMCL]   init_publisher_->get_subscription_count() > 0
//               AMCL has subscribed to /initialpose
//   [bridge] tb4_gt_pose_subscriber_->get_publisher_count() > 0
//               gz-ros bridge is publishing /tb4_dynamic_pose
//   [nav2]   action_client_->action_server_is_ready()
//               NavigateToPose action server is up
//
// Important: all timers are create_wall_timer() so they fire on real
// wall-clock time and are unaffected by sim time not yet flowing.
// ---------------------------------------------------------------------------

class TestPath : public rclcpp::Node {
public:
    TestPath() : Node("path"), init_pose_count_(0)
    {
        // ── publishers / subscribers / action client ─────────────────────
        init_publisher_ = this->create_publisher<
            geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

        tb4_gt_pose_subscriber_ = this->create_subscription<
            geometry_msgs::msg::PoseArray>(
                "tb4_dynamic_pose", 10,
                std::bind(&TestPath::gtPoseCallback, this, std::placeholders::_1));

        tb4_gt_path_publisher_ = this->create_publisher<
            nav_msgs::msg::Path>("tb4_gt_path", 10);

        this->declare_parameter<int>("tb4_object_index", 1);
        // Delay (ms) between the last /initialpose publish and sending the nav
        // goal.  action_server_is_ready() only confirms the ROS interface is
        // present; bt_navigator is a lifecycle node that may still be INACTIVE
        // (costmaps loading) and will reject goals until it reaches ACTIVE.
        this->declare_parameter<int>("nav_goal_delay_ms", 5000);
        accumulated_path_.header.frame_id = "map";

        action_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        // ── begin readiness polling ──────────────────────────────────────
        // Wall timer: unaffected by use_sim_time, safe before /clock flows
        readiness_timer_ = this->create_wall_timer(
            1s, std::bind(&TestPath::checkReadiness, this));

        RCLCPP_INFO(this->get_logger(),
            "[startup] waiting for AMCL, gz-bridge, and Nav2 action server...");
    }

private:

    // ── gate: poll all required connections ────────────────────────────
    void checkReadiness()
    {
        const bool amcl_ready   = init_publisher_->get_subscription_count() > 0;
        const bool bridge_ready = tb4_gt_pose_subscriber_->get_publisher_count() > 0;
        const bool nav2_ready   = action_client_->action_server_is_ready();

        if (amcl_ready && bridge_ready && nav2_ready) {
            readiness_timer_->cancel();
            RCLCPP_INFO(this->get_logger(),
                "[startup] all systems ready — publishing initial pose");
            // Transition → PUBLISHING_INIT_POSE
            init_timer_ = this->create_wall_timer(
                500ms, std::bind(&TestPath::publishInitialPose, this));
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "[startup] waiting  AMCL[%s]  bridge[%s]  nav2[%s]",
                amcl_ready   ? "OK" : "--",
                bridge_ready ? "OK" : "--",
                nav2_ready   ? "OK" : "--");
        }
    }

    // ── ground-truth path accumulation ─────────────────────────────────
    void gtPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        int tb4_index = this->get_parameter("tb4_object_index").as_int();

        if (tb4_index < 0 || static_cast<size_t>(tb4_index) >= msg->poses.size()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "tb4_object_index %d out of bounds (array size %zu)",
                tb4_index, msg->poses.size());
            return;
        }

        const auto& p = msg->poses[tb4_index];

        geometry_msgs::msg::PoseStamped stamped;
        stamped.header.stamp        = msg->header.stamp;
        stamped.header.frame_id     = "map";              // gt poses are in the map frame
        stamped.pose.position.x     = p.position.x + 8.0; // gz↔RViz origin offset
        stamped.pose.position.y     = p.position.y;
        stamped.pose.position.z     = p.position.z;
        stamped.pose.orientation    = p.orientation;

        accumulated_path_.header.stamp    = this->now();
        accumulated_path_.header.frame_id = "map";
        accumulated_path_.poses.push_back(stamped);

        // Uncomment when path size becomes a concern:
        // if (accumulated_path_.poses.size() > 10000)
        //     accumulated_path_.poses.erase(accumulated_path_.poses.begin());

        tb4_gt_path_publisher_->publish(accumulated_path_);
    }

    // ── initial pose injection ──────────────────────────────────────────
    void publishInitialPose()
    {
        if (init_pose_count_ >= 2) {
            init_timer_->cancel();
            int delay_ms = this->get_parameter("nav_goal_delay_ms").as_int();
            RCLCPP_INFO(this->get_logger(),
                "[startup] initial pose set — waiting %d ms for Nav2 to reach ACTIVE",
                delay_ms);
            // One-shot wall timer: fires once, then sendNavGoal() is called.
            nav_goal_delay_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(delay_ms),
                std::bind(&TestPath::onNavGoalDelayExpired, this));
            return;
        }

        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.frame_id          = "map";
        msg.header.stamp             = this->now();
        msg.pose.pose.orientation.w  = 1.0; // identity at origin
        init_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(),
            "[startup] published initial pose (%d/2)", init_pose_count_ + 1);
        ++init_pose_count_;
    }

    // ── one-shot: fires after nav_goal_delay_ms, then sends the goal ───
    void onNavGoalDelayExpired()
    {
        nav_goal_delay_timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "[startup] delay elapsed — sending navigation goal");
        sendNavGoal();
    }

    // ── send NavigateToPose goals ──────────────────────────────────

    void sendNavGoal()
    {
    	// Secondary guard: action server should already be ready
    	if (!action_client_->wait_for_action_server(5s)) {
        RCLCPP_ERROR(this->get_logger(),
            "NavigateToPose action server unavailable — aborting");
        rclcpp::shutdown();
        return;
    	}

    	auto make_pose = [this](double x, double y, double qx, double qw) {
            geometry_msgs::msg::PoseStamped p;
	    p.header.frame_id    = "map";
            p.header.stamp       = this->now();
            p.pose.position.x    = x;
            p.pose.position.y    = y;
            p.pose.orientation.x = qx;
            p.pose.orientation.w = qw;
            return p;
    	};

    	// Populate the class vector once at the start
    	if (target_poses_.empty()) {
        	target_poses_.push_back(make_pose(0.0, 0.0, 0.0, 1.0));  // waypoint 1
        	target_poses_.push_back(make_pose(3.05, 2.03, 0.91, 0.42));  // waypoint 2
        	target_poses_.push_back(make_pose(7.44, 0.61, 0.92, 0.38));  // waypoint 3
        	target_poses_.push_back(make_pose(12.2, 4.7, 0.73, -0.68)); // waypoint 4
        	target_poses_.push_back(make_pose(18.1, -5.95, 0.105, -1.0));  // waypoint 5
        	target_poses_.push_back(make_pose(-4.86, -3.7, 0.62, -0.78)); // waypoint 6
		current_pose_idx_ = 0;									    
    	}

    	// Safety check in case it gets called again when done
    	if (current_pose_idx_ >= target_poses_.size()) {
        	RCLCPP_INFO(this->get_logger(), "All waypoints already visited.");
        	return;
    	}

    	// Create the NavigateToPose single goal message
    	auto goal = NavigateToPose::Goal();
    	goal.pose = target_poses_[current_pose_idx_];

    	RCLCPP_INFO(this->get_logger(), "Sending waypoint %zu/%zu to NavigateToPose", current_pose_idx_ + 1, target_poses_.size());

	auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

	opts.goal_response_callback =
        	[this](GoalHandleNavToPose::SharedPtr gh) {
            	if (!gh) {
                	RCLCPP_ERROR(this->get_logger(), "Goal rejected by server. Aborting mission.");
                	rclcpp::shutdown();
            	} else {
                	RCLCPP_INFO(this->get_logger(), "Goal accepted — navigating");
            	}
        	};

    	opts.result_callback =
        	[this](const GoalHandleNavToPose::WrappedResult& result) {
            	switch (result.code) {
                	case rclcpp_action::ResultCode::SUCCEEDED:
                    	  RCLCPP_INFO(this->get_logger(), "Waypoint %zu reached successfully!", current_pose_idx_ + 1);
			  current_pose_idx_++;
	                  // If more poses remain, send the next goal recursively
                    	  if (current_pose_idx_ < target_poses_.size()) {
                        	this->sendNavGoal();
                    	  } else {
                        	RCLCPP_INFO(this->get_logger(), "Path complete — all waypoints reached");
                        	rclcpp::shutdown();
                    	  }
                    	  break;

                	case rclcpp_action::ResultCode::ABORTED:
                    	  RCLCPP_ERROR(this->get_logger(), "Navigation aborted at waypoint %zu", current_pose_idx_ + 1);
                     	  rclcpp::shutdown();
                    	  break;
                
			case rclcpp_action::ResultCode::CANCELED:
                    	  RCLCPP_WARN(this->get_logger(), "Navigation canceled");
                    	  rclcpp::shutdown();
                    	  break;
                
			default:
                    	  RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    	  rclcpp::shutdown();
                    	  break;
            }
        };

    action_client_->async_send_goal(goal, opts);
    }

    // ── state ───────────────────────────────────────────────────────────
    int init_pose_count_;
    size_t current_pose_idx_ = 0;
    nav_msgs::msg::Path accumulated_path_;
    std::vector<geometry_msgs::msg::PoseStamped> target_poses_;

    rclcpp::TimerBase::SharedPtr readiness_timer_;      // wall clock — fires even before /clock
    rclcpp::TimerBase::SharedPtr init_timer_;           // wall clock — started after readiness
    rclcpp::TimerBase::SharedPtr nav_goal_delay_timer_; // wall clock — one-shot after init pose

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr              tb4_gt_pose_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                           tb4_gt_path_publisher_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr                      action_client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestPath>());
    rclcpp::shutdown();
    return 0;
}
