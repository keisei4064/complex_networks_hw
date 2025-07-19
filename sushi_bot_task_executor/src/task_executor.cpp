#include "rclcpp/rclcpp.hpp"
#include "sushi_bot_task_executor/convergence_checker.hpp"
#include "sushi_bot_task_executor/ik.hpp"
#include <queue>

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

  std::queue<std::function<bool()>> task_queue_;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskExecutorNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
