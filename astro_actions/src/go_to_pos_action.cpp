#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

#include "astro_action_interfaces/action/move_to_position.hpp"

namespace astro_action_servers
{
  using moveit::planning_interface::MoveGroupInterface;

  class GoToPositionActionServer : public rclcpp::Node
  {
  public:
    using GoToPosition = astro_action_interfaces::action::MoveToPosition;
    using GoalHandleGoToPosition = rclcpp_action::ServerGoalHandle<GoToPosition>;

    explicit GoToPositionActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("go_to_position_action_server", options), move_group_interface(MoveGroupInterface(std::shared_ptr<rclcpp::Node>(this),"uf850"))
    {
      using namespace std::placeholders;
      move_group_interface.setPoseReferenceFrame("link_base");
      move_group_interface.setEndEffectorLink("m_connector");
      auto handle_goal = [this](
                             const rclcpp_action::GoalUUID &uuid,
                             std::shared_ptr<const GoToPosition::Goal> goal)
      {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal request with x %f , y %f, z %f", goal->x, goal->y, goal->z);
        // The Fibonacci action uses int32 for the return of sequences, which means it can only hold
        // 2^31-1 (2147483647) before wrapping negative in two's complement. Based on empirical
        // tests, that means that an order of > 46 will cause wrapping, so we don't allow that here.

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      };

      auto handle_cancel = [this](
                               const std::shared_ptr<GoalHandleGoToPosition> goal_handle)
      {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
      };

      auto handle_accepted = [this](
                                 const std::shared_ptr<GoalHandleGoToPosition> goal_handle)
      {
        // this needs to return quickly to avoid blocking the executor,
        // so we declare a lambda function to be called inside a new thread
        auto execute_in_thread = [this, goal_handle]()
        { return this->execute(goal_handle); };
        std::thread{execute_in_thread}.detach();
      };

      this->action_server_ = rclcpp_action::create_server<GoToPosition>(
          this,
          "go_to_position",
          handle_goal,
          handle_cancel,
          handle_accepted);
    }

  private:
    moveit::planning_interface::MoveGroupInterface move_group_interface;
    rclcpp_action::Server<GoToPosition>::SharedPtr action_server_;

    void execute(const std::shared_ptr<GoalHandleGoToPosition> goal_handle)
    {
      auto result = std::make_shared<GoToPosition::Result>();

      RCLCPP_INFO(this->get_logger(), "Executing goal");
      geometry_msgs::msg::Pose target_pose;
      const auto goal = goal_handle->get_goal();

      std::cout << "End Effector Link: " << move_group_interface.getEndEffectorLink() << std::endl;

      tf2::Quaternion orientation;
      orientation.setRPY(goal->ax, goal->ay, goal->az);

      target_pose.orientation.x = 0;
      target_pose.orientation.y = 0;
      target_pose.orientation.z = 0;
      target_pose.orientation.w = 1;

      target_pose.position.x = goal->x;
      target_pose.position.y = goal->y;
      target_pose.position.z = goal->z;

      // Log the target pose information
      RCLCPP_INFO(this->get_logger(), "Target Pose:");
      RCLCPP_INFO(this->get_logger(), "Orientation: x: %.2f, y: %.2f, z: %.2f, w: %.2f",
                  target_pose.orientation.x,
                  target_pose.orientation.y,
                  target_pose.orientation.z,
                  target_pose.orientation.w);

      RCLCPP_INFO(this->get_logger(), "Position: x: %.2f, y: %.2f, z: %.2f",
                  target_pose.position.x,
                  target_pose.position.y,
                  target_pose.position.z);

      move_group_interface.setPoseTarget(target_pose);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = static_cast<bool>(move_group_interface.plan(plan));

      if (success)
      {
        move_group_interface.execute(plan);
        result->reached_destination = true;
        RCLCPP_INFO(this->get_logger(), "Movement to the pose completed successfully.");
      }
      else
      {
        result->reached_destination = false;
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
      }
      // Check if goal is done
      if (rclcpp::ok())
      {
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      }
    }
  }; // class GoToPositionActionServer

} // namespace astro_action_servers

RCLCPP_COMPONENTS_REGISTER_NODE(astro_action_servers::GoToPositionActionServer)
