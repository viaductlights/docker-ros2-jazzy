#include <chrono>
#include <memory>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using GoalHandleNavPoses   = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

// NavigateThroughPoses palnner - short trajectory

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
//               NavigateThroughPoses action server is up
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

        action_client_ = rclcpp_action::create_client<NavigateThroughPoses>(
            this, "navigate_through_poses");

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

    // ── send NavigateThroughPoses goal ──────────────────────────────────
    void sendNavGoal()
    {
        // Secondary guard: action server should already be ready, but
        // wait_for_action_server gives a clean error if something went wrong.
        if (!action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(),
                "NavigateThroughPoses action server unavailable — aborting");
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

        auto goal = NavigateThroughPoses::Goal();
        goal.poses.push_back(make_pose( 0.0,  0.0,  0.0, 1.0)); // waypoint 1
	goal.poses.push_back(make_pose(4.0, 0.0, 0.0, 1.0));    // waypoint 2
  	goal.poses.push_back(make_pose( 7.0,  6.0,  0.0, 1.0)); // waypoint 3
      	goal.poses.push_back(make_pose( 7.0,  2.35, 0.0, 0.0)); // waypoint 4

        RCLCPP_INFO(this->get_logger(),
            "Sending %zu waypoint(s) to NavigateThroughPoses", goal.poses.size());

        auto opts = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();

        opts.goal_response_callback =
            [this](GoalHandleNavPoses::SharedPtr gh) {
                if (!gh)
                    RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
                else
                    RCLCPP_INFO(this->get_logger(), "Goal accepted — navigating");
            };

        opts.result_callback =
            [this](const GoalHandleNavPoses::WrappedResult& result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO (this->get_logger(), "Path complete — all waypoints reached"); break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "Navigation aborted");  break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN (this->get_logger(), "Navigation canceled"); break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown result code"); break;
                }
                rclcpp::shutdown();
            };

        action_client_->async_send_goal(goal, opts);
    }

    // ── state ───────────────────────────────────────────────────────────
    int init_pose_count_;
    nav_msgs::msg::Path accumulated_path_;

    rclcpp::TimerBase::SharedPtr readiness_timer_;      // wall clock — fires even before /clock
    rclcpp::TimerBase::SharedPtr init_timer_;           // wall clock — started after readiness
    rclcpp::TimerBase::SharedPtr nav_goal_delay_timer_; // wall clock — one-shot after init pose

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr              tb4_gt_pose_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                           tb4_gt_path_publisher_;
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr                      action_client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestPath>());
    rclcpp::shutdown();
    return 0;
}
