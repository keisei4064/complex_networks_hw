#include "rclcpp/rclcpp.hpp"
#include "sushi_bot_task_executor/convergence_checker.hpp"
#include "sushi_bot_task_executor/ik.hpp"
#include <queue>
#include "rclcpp_action/rclcpp_action.hpp"

#include "sushi_bot_interfaces/srv/drop_object.hpp"
#include "sushi_bot_interfaces/srv/pick_object.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

using namespace std::chrono_literals;

constexpr double PI = 3.14159265358979323846;

constexpr double rad2deg(double rad) noexcept
{
  return rad * 180.0 / PI;
}

constexpr double deg2rad(double deg) noexcept
{
  return deg * PI / 180.0;
}

class TaskExecutorNode : public rclcpp::Node
{
public:
  TaskExecutorNode()
      : Node("task_executor_node", "sushi_bot_task_executor")
  {
    pick_client_ = this->create_client<sushi_bot_interfaces::srv::PickObject>(
        "/sushi_bot_gazebo_plugins/pick_and_drop/pick_object");
    drop_client_ = this->create_client<sushi_bot_interfaces::srv::DropObject>(
        "/sushi_bot_gazebo_plugins/pick_and_drop/drop_object");
    follow_joint_trajectory_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(this,
                                                                                                                "/joint_trajectory_controller/follow_joint_trajectory");

    RCLCPP_INFO(this->get_logger(), "Waiting for services to be available...");
    if (!pick_client_->wait_for_service(5s))
    {
      RCLCPP_ERROR(this->get_logger(), "PickObject service not available after waiting.");
      return;
    }
    if (!drop_client_->wait_for_service(5s))
    {
      RCLCPP_ERROR(this->get_logger(), "DropObject service not available after waiting.");
      return;
    }
    if (!follow_joint_trajectory_client_->wait_for_action_server(5s))
    {
      RCLCPP_ERROR(this->get_logger(), "FollowJointTrajectory action server not available after waiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Services are available.");

    RCLCPP_INFO(this->get_logger(), "Task Executor Node started.");

    DoTask();

    rclcpp::shutdown();
  }

private:
  void DoTask()
  {
    RCLCPP_INFO(this->get_logger(), "Executing task...");
    RCLCPP_INFO(this->get_logger(), "Step 1: Go to Neutral Position");
    MoveJoints(NEUTRAL_POSITION);
    rclcpp::sleep_for(100ms);

    RCLCPP_INFO(this->get_logger(), "Step 2: Pick Salmon Sushi 1");
    MoveJoints({0.06283, 1.320, 0.0520, 0.0, 1.760, 0.0});
    rclcpp::sleep_for(100ms);
    MoveJoints({0.06283, 2.0734, 0.0520, 0.0, 1.005, 0.0});
    rclcpp::sleep_for(100ms);
    PickObject("SalmonSushi_2");
    MoveJoints({0.06283, 1.320, 0.0520, 0.0, 1.760, 0.0});
    rclcpp::sleep_for(100ms);
    MoveJoints({1.85, 1.320, 0.116, 0.0, 1.760, 0.0});
    rclcpp::sleep_for(100ms);
    MoveJoints({1.85, 1.633632, 0.1, 0.0, 1.445136, -0.439818});
    DropObject();
    MoveJoints({1.85, 1.320, 0.116, 0.0, 1.760, 0.0});

    MoveJoints(NEUTRAL_POSITION);

    RCLCPP_INFO(this->get_logger(), "Step 3: Pick Salmon Sushi 2");
    MoveJoints({-0.18, 1.320, 0.044000, 0.0, 1.760, -0.188486});
    rclcpp::sleep_for(100ms);
    MoveJoints({-0.18, 2.110000, 0.044000, 0.0, 0.942480, -0.188486});
    rclcpp::sleep_for(100ms);
    PickObject("SalmonSushi_0");
    MoveJoints({-0.18, 1.320, 0.044000, 0.0, 1.760, -0.188486});
    rclcpp::sleep_for(100ms);
    MoveJoints({1.382304, 1.320000, 0.080000, 0.0, 1.760, -0.188486});
    rclcpp::sleep_for(100ms);
    MoveJoints({1.382304, 1.634160, 0.076000, 0.0, 1.508672, -0.816806});
    DropObject();
    MoveJoints({1.382304, 1.320000, 0.080000, 0.0, 1.760, -0.188486});
    rclcpp::sleep_for(100ms);

    MoveJoints(NEUTRAL_POSITION);

    RCLCPP_INFO(this->get_logger(), "Step 4: Pick Salmon Sushi 3");
    MoveJoints({-0.420000, 1.320, 0.052000, 0.0, 1.760, -0.376992});
    rclcpp::sleep_for(100ms);
    MoveJoints({-0.420000, 2.090000, 0.052000, 0.0, 1.005309, -0.376992});
    rclcpp::sleep_for(100ms);
    PickObject("SalmonSushi_1");
    MoveJoints({-0.420000, 1.320, 0.052000, 0.0, 1.760, -0.376992});
    rclcpp::sleep_for(100ms);
    MoveJoints({0.96, 1.320, 0.112000, 0.0, 1.760, -0.376992});
    rclcpp::sleep_for(100ms);
    MoveJoints({0.96, 1.634160, 0.124000, 0.0, 1.508672, -1.256640});
    DropObject();
    MoveJoints({0.96, 1.320, 0.112000, 0.0, 1.760, -0.376992});
    rclcpp::sleep_for(100ms);

    MoveJoints(NEUTRAL_POSITION);
    RCLCPP_INFO(this->get_logger(), "All tasks completed successfully.");
  }

  void PickObject(std::string object_name)
  {
    auto request = std::make_shared<sushi_bot_interfaces::srv::PickObject::Request>();
    request->object_name = object_name;

    auto future = pick_client_->async_send_request(request);
    auto ret = rclcpp::spin_until_future_complete(
        this->get_node_base_interface(), // Node を spin するために渡す
        future,
        std::chrono::seconds(2) // タイムアウト（任意）
    );

    if (ret != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "service call failed or timed out");
      return;
    }

    if (future.get()->success)
    {
      RCLCPP_INFO(this->get_logger(), "Successfully picked object: %s", object_name.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to pick object: %s", object_name.c_str());
    }
  }

  void DropObject()
  {
    auto future = drop_client_->async_send_request(std::make_shared<sushi_bot_interfaces::srv::DropObject::Request>());
    auto ret = rclcpp::spin_until_future_complete(
        this->get_node_base_interface(), // Node を spin するために渡す
        future,
        std::chrono::seconds(2) // タイムアウト（任意）
    );

    if (ret != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "service call failed or timed out");
      return;
    }

    if (future.get()->success)
    {
      RCLCPP_INFO(this->get_logger(), "Successfully dropped object.");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to drop object.");
    }
  }

  void MoveJoints(std::array<double, 6> joint_positions)
  {
    auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    goal_msg.trajectory.points.resize(1);
    goal_msg.trajectory.points[0].positions.resize(6);
    RCLCPP_DEBUG(this->get_logger(), "Setting joint positions for trajectory.");
    std::copy(joint_positions.begin(), joint_positions.end(),
              goal_msg.trajectory.points[0].positions.begin());
    goal_msg.trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(1.0);
    RCLCPP_DEBUG(this->get_logger(), "Joint positions set.");
    goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(3.0);
    goal_msg.goal_tolerance.resize(6);
    for (size_t i = 0; i < 6; ++i)
    {
      goal_msg.goal_tolerance[i].name = goal_msg.trajectory.joint_names[i];
      goal_msg.goal_tolerance[i].position = 0.15; // 0.15 rad
      goal_msg.goal_tolerance[i].velocity = 0.10; // 0.10 rad/s

      if (i == 2)
      {
        goal_msg.goal_tolerance[i].position = 0.015; // 1.5 cm
        goal_msg.goal_tolerance[i].velocity = 0.05;  // 5 cm/s
      }
    }

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();

    send_goal_options.goal_response_callback = [this](rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr goal_handle)
    {
      if (!goal_handle)
      {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the action server.");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by the action server.");
      }
    };
    send_goal_options.feedback_callback = [this](rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
                                                 const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
    {
      std::ostringstream ss;
      ss << "Current joint errors: [";
      for (const auto &pos : feedback->error.positions)
      {
        ss << pos << " ";
      }
      ss << "]";
      RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    };

    send_goal_options.result_callback = [this](rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult result)
    {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
      {
        RCLCPP_INFO(this->get_logger(), "Joint trajectory executed successfully.");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Joint trajectory execution failed.");
      }
    };

    auto future = follow_joint_trajectory_client_->async_send_goal(goal_msg, send_goal_options);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to send goal to FollowJointTrajectory action server.");
      return;
    }

    auto result_future = follow_joint_trajectory_client_->async_get_result(future.get());
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get result from FollowJointTrajectory action server.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Joint trajectory goal sent successfully.");
  }

  rclcpp::Client<sushi_bot_interfaces::srv::PickObject>::SharedPtr pick_client_;
  rclcpp::Client<sushi_bot_interfaces::srv::DropObject>::SharedPtr drop_client_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr follow_joint_trajectory_client_;

  static constexpr std::array<double, 6> NEUTRAL_POSITION = {0.0, deg2rad(60), 0.0, 0.0, deg2rad(90), 0.0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskExecutorNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
