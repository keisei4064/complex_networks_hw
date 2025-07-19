#include <memory>
#include <string>

#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo_ros/node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "ignition/math/Pose3.hh"
#include "sushi_bot_interfaces/srv/drop_object.hpp"
#include "sushi_bot_interfaces/srv/pick_object.hpp"
#include "rclcpp/rclcpp.hpp"

class PickAndDropNode : public rclcpp::Node
{
public:
  PickAndDropNode() : Node("pick_and_drop", "sushi_bot_gazebo_plugins")
  {
    // /sushi_bot_gazebo_plugins/pick_and_drop/pick_object
    pick_service_ = this->create_service<sushi_bot_interfaces::srv::PickObject>(
      "~/pick_object",
      std::bind(&PickAndDropNode::OnPick, this, std::placeholders::_1, std::placeholders::_2));
    // /sushi_bot_gazebo_plugins/pick_and_drop/drop_object
    drop_service_ = this->create_service<sushi_bot_interfaces::srv::DropObject>(
      "~/drop_object",
      std::bind(&PickAndDropNode::OnDrop, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "ROS2 node: 'sushi_bot_gazebo_plugins/pick_and_drop' started.");
  }

  void SetGazeboInterface(
    const gazebo::physics::WorldPtr & world,
    const std::string & robot_model,
    const std::string & robot_link,
    const ignition::math::Pose3d & relative_pose,
    double manipulatable_distance)
  {
    world_ = world;
    robot_model_name_ = robot_model;
    robot_link_name_ = robot_link;
    relative_pose_ = relative_pose;
    manipulatable_distance_ = manipulatable_distance;
    RCLCPP_INFO(this->get_logger(), "Set gazebo interface.");
    RCLCPP_INFO(
      this->get_logger(),
      "robot_model: %s, robot_link: %s",
      robot_model.c_str(),
      robot_link.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "relative_pose: {%f, %f, %f, %f, %f, %f}",
      relative_pose.Pos().X(),
      relative_pose.Pos().Y(),
      relative_pose.Pos().Z(),
      relative_pose.Rot().Roll(),
      relative_pose.Rot().Pitch(),
      relative_pose.Rot().Yaw());
    RCLCPP_INFO(this->get_logger(), "manipulatable_distance: %f", manipulatable_distance);
  }

private:
  void OnPick(
    const std::shared_ptr<sushi_bot_interfaces::srv::PickObject::Request> request,
    std::shared_ptr<sushi_bot_interfaces::srv::PickObject::Response> response)
  {
    // エラー判定 ------------------------------------------------------------------
    if (!world_) {
      response->success = false;
      return;
    }

    if (joint_) {
      response->success = false;
      return;
    }

    auto robot_model = world_->ModelByName(robot_model_name_);
    if (!robot_model) {
      response->success = false;
      return;
    }
    auto object_model = world_->ModelByName(request->object_name);
    if (!robot_model || !object_model) {
      response->success = false;
      return;
    }

    auto robot_link = robot_model->GetLink(robot_link_name_);
    if (!robot_link) {
      response->success = false;
      return;
    }
    auto object_link = object_model->GetLink();
    if (!robot_link || !object_link) {
      response->success = false;
      return;
    }

    // 距離判定
    auto pose_diff = robot_link->WorldPose() - object_link->WorldPose();
    double distance = std::sqrt(pose_diff.X() * pose_diff.X() + pose_diff.Y() * pose_diff.Y());
    if (distance > manipulatable_distance_) {
      response->success = false;
      return;
    }

    // アタッチ処理 ------------------------------------------------------------------
    // オブジェクトをロボットのリンクに対して相対姿勢で配置
    ignition::math::Pose3d rel_pose_rotated = relative_pose_;
    // rel_pose_rotated.Pos() = robot_link->WorldPose().Rot().RotateVector(relative_pose_.Pos());
    // rel_pose_rotated.Rot() = robot_link->WorldPose().Rot() * relative_pose_.Rot();
    // rel_pose_rotated = robot_link->WorldPose() * rel_pose_rotated;
    ignition::math::Pose3d target_pose = robot_link->WorldPose() * rel_pose_rotated;
    object_model->SetWorldPose(target_pose);

    // ジョイント生成
    joint_ = world_->Physics()->CreateJoint("fixed", robot_model);
    joint_->Load(
      robot_link, object_link, robot_link->WorldPose().Inverse() * object_link->WorldPose());
    joint_->Init();
    RCLCPP_INFO(
      this->get_logger(),
      "Picked '%s' to '%s'.",
      object_link->GetName().c_str(),
      robot_link_name_.c_str());
    response->success = true;

    picked_object_model_ = object_model;
  }

  void OnDrop(
    const std::shared_ptr<sushi_bot_interfaces::srv::DropObject::Request> request,
    std::shared_ptr<sushi_bot_interfaces::srv::DropObject::Response> response)
  {
    if (!world_) {
      response->success = false;
      return;
    }

    if (!joint_ || !picked_object_model_) {
      response->success = false;
      return;
    }

    auto robot_model = world_->ModelByName(robot_model_name_);
    auto robot_link = robot_model ? robot_model->GetLink(robot_link_name_) : nullptr;
    if (!robot_link) {
      response->success = false;
      return;
    }

    joint_->Detach();
    joint_.reset();

    // rotated_rel_pose.Pos() = robot_link->WorldPose().Rot().RotateVector(relative_pose.Pos());
    // ignition::math::Pose3d world_pose = robot_link->WorldPose() + rotated_rel_pose;
    // picked_object_model_->SetWorldPose(world_pose);

    // RCLCPP_INFO(
    //   this->get_logger(),
    //   "Droped object '%s' to {%f, %f, %f, %f, %f, %f}.",
    //   picked_object_model_->GetName().c_str(),
    //   relative_pose.Pos().X(),
    //   relative_pose.Pos().Y(),
    //   relative_pose.Pos().Z(),
    //   relative_pose.Rot().Roll(),
    //   relative_pose.Rot().Pitch(),
    //   relative_pose.Rot().Yaw());

    picked_object_model_.reset();
    response->success = true;
  }

  gazebo::physics::WorldPtr world_;
  gazebo::physics::JointPtr joint_;
  gazebo::physics::ModelPtr picked_object_model_;

  std::string robot_model_name_;
  std::string robot_link_name_;
  ignition::math::Pose3d relative_pose_;
  double manipulatable_distance_{};

  rclcpp::Service<sushi_bot_interfaces::srv::PickObject>::SharedPtr pick_service_;
  rclcpp::Service<sushi_bot_interfaces::srv::DropObject>::SharedPtr drop_service_;
};

// Gazebo plugin bridge to ROS2
class PickAndDropPlugin : public gazebo::WorldPlugin
{
public:
  void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override
  {
    // ROS2が初期化されていない場合は初期化する
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }

    node_ = std::make_shared<PickAndDropNode>();

    std::string robot_model = _sdf->Get<std::string>("robot_model");
    std::string robot_link = _sdf->Get<std::string>("robot_link");
    ignition::math::Pose3d relative_pose(0.0, 0, 0.0, 0, 0, 0);
    if (_sdf->HasElement("relative_pose")) {
      relative_pose = _sdf->Get<ignition::math::Pose3d>("relative_pose");
    }
    double manipulatable_distance = _sdf->Get<double>("manipulatable_distance");

    node_->SetGazeboInterface(
      _world, robot_model, robot_link, relative_pose, manipulatable_distance);

    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(node_);
    spin_thread_ = std::thread([this]() { exec_->spin(); });
  }

private:
  std::shared_ptr<PickAndDropNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::thread spin_thread_;
};

GZ_REGISTER_WORLD_PLUGIN(PickAndDropPlugin)
