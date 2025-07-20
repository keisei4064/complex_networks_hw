#include "rclcpp/rclcpp.hpp"
#include "sushi_bot_task_executor/convergence_checker.hpp"
#include "sushi_bot_task_executor/ik.hpp"
#include <queue>
#include "rclcpp_action/rclcpp_action.hpp"

#include "sushi_bot_interfaces/srv/drop_object.hpp"
#include "sushi_bot_interfaces/srv/pick_object.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

using namespace std::chrono_literals;

class TaskExecutorNode : public rclcpp::Node
{
public:
  TaskExecutorNode()
      : Node("task_executor_node", "sushi_bot_task_executor")
  {
    update_timer_ = this->create_wall_timer(
        10ms,
        std::bind(&TaskExecutorNode::Update, this));
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
  }

private:
  void Update()
  {
    if (task_queue_.empty())
    {
      RCLCPP_INFO(this->get_logger(), "No tasks in the queue.");
      rclcpp::shutdown();
      return;
    }

    // 先頭のタスク（関数）を実行，完了したらキューから削除
    auto &task = task_queue_.front();
    if (task())
    {
      task_queue_.pop();
    }
  }

  void PickObject(std::string object_name)
  {
    auto request = std::make_shared<sushi_bot_interfaces::srv::PickObject::Request>();
    request->object_name = object_name;

    auto future = pick_client_->async_send_request(request);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    if (future.get() == nullptr)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call PickObject service.");
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

    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    if (future.get() == nullptr)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call DropObject service.");
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
    std::copy(joint_positions.begin(), joint_positions.end(),
              goal_msg.trajectory.points[0].positions.begin());

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
      ss << "Current joint positions: [";
      for (const auto &pos : feedback->actual.positions)
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
    RCLCPP_INFO(this->get_logger(), "Joint trajectory goal sent successfully.");
  }

  std::queue<std::function<bool()>> task_queue_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  rclcpp::Client<sushi_bot_interfaces::srv::PickObject>::SharedPtr pick_client_;
  rclcpp::Client<sushi_bot_interfaces::srv::DropObject>::SharedPtr drop_client_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr follow_joint_trajectory_client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskExecutorNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
