#include "controlko_controllers/displacement_controller.hpp"

// ユーティリティを置く匿名名前空間
namespace
{

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

using ControllerReferenceMsg =
  controlko_controllers::DisplacementController::ControllerReferenceMsg;

// リファレンスメッセージのリセット
// リアルタイム制御ループから呼び出される
void reset_controller_reference_msg(
  std::shared_ptr<ControllerReferenceMsg> & msg, const std::vector<std::string> & joint_names)
{
  msg->joint_names = joint_names;
  msg->displacements.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->velocities.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

namespace controlko_controllers
{

// コンストラクタ（空）
DisplacementController::DisplacementController() : controller_interface::ControllerInterface() {}

// 初期化
controller_interface::CallbackReturn DisplacementController::on_init()
{
  // 制御モードの初期化
  control_mode_.initRT(control_mode_type::FAST);

  // パラメータリスナーの初期化
  try {
    param_listener_ = std::make_shared<displacement_controller::ParamListener>(get_node());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize ParamListener: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

// unconfigured -> inactiveにする際の処理
controller_interface::CallbackReturn DisplacementController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // joint情報取得
  params_ = param_listener_->get_params();
  if (!params_.state_joints.empty()) {
    state_joints_ = params_.state_joints;
  } else {
    state_joints_ = params_.joints;
  }
  if (params_.joints.size() != state_joints_.size()) {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'joints' (%zu) and 'state_joints' (%zu) parameters has to be the same!",
      params_.joints.size(),
      state_joints_.size());
    return CallbackReturn::FAILURE;
  }

  // トピックのQoS設定
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // 目標(reference)を受信するサブスクライバを作成
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference",
    subscribers_qos,
    std::bind(&DisplacementController::reference_callback, this, std::placeholders::_1));

  // 目標リセット
  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, params_.joints);
  input_ref_.writeFromNonRT(msg);

  // モード切替サービス
  auto set_slow_mode_service_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response) {
      if (request->data) {
        control_mode_.writeFromNonRT(control_mode_type::SLOW);
      } else {
        control_mode_.writeFromNonRT(control_mode_type::FAST);
      }
      response->success = true;
    };
  set_slow_control_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
    "~/set_slow_control_mode",
    set_slow_mode_service_callback,
    rmw_qos_profile_services_hist_keep_all);

  // コントローラ状態(state)のパブリッシャー
  try {
    s_publisher_ =
      get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  } catch (const std::exception & e) {
    fprintf(
      stderr,
      "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  // TODO(anyone): Reserve memory in state publisher depending on the message type
  state_publisher_->lock();
  state_publisher_->msg_.header.frame_id = params_.joints[0];
  state_publisher_->unlock();

  // configure完了
  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

// 目標(reference)を受信するコールバック関数
void DisplacementController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  if (msg->joint_names.size() == params_.joints.size()) {
    // 目標値更新
    input_ref_.writeFromNonRT(msg);
  } else {
    // エラー処理
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received %zu , but expected %zu joints in command. Ignoring message.",
      msg->joint_names.size(),
      params_.joints.size());
  }
}

// interface_configuration_type
// +------------+--------------------------+-------------------------------+
// | モード     | インターフェースの取得   | 一般的な用途                  |
// +------------+--------------------------+-------------------------------+
// | ALL        | すべて取得               | テスト、実験、簡易コントローラ |
// | INDIVIDUAL | 指定したものだけ取得     | 通常の制御（推奨）             |
// | NONE       | 取得しない               | モニタリング系など             |
// +------------+--------------------------+-------------------------------+

// 	使用するコマンドインターフェースの指定
controller_interface::InterfaceConfiguration
DisplacementController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.joints.size());
  for (const auto & joint : params_.joints) {
    command_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
  }

  return command_interfaces_config;
}

// 使用するステートインターフェースの指定
controller_interface::InterfaceConfiguration DisplacementController::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_joints_.size());
  for (const auto & joint : state_joints_) {
    state_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
  }

  return state_interfaces_config;
}

// inactive -> activeにする際の処理
controller_interface::CallbackReturn DisplacementController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // デフォルト目標値を設定します
  reset_controller_reference_msg(*(input_ref_.readFromRT)(), params_.joints);
  return controller_interface::CallbackReturn::SUCCESS;
}

// active -> inactiveにする際の処理
controller_interface::CallbackReturn DisplacementController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

// 制御処理
// リアルタイム制約を満たす必要がある．
controller_interface::return_type DisplacementController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto current_ref = input_ref_.readFromRT();  // リアルタイム側から読む

  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    // 目標値がNaNでない場合のみコマンドを設定
    if (!std::isnan((*current_ref)->displacements[i])) {
      // SLOWモードでは目標値を半分にする
      if (*(control_mode_.readFromRT()) == control_mode_type::SLOW) {
        (*current_ref)->displacements[i] /= 2;
      }

      // コマンドインターフェースに目標値を設定
      command_interfaces_[i].set_value(
        (*current_ref)->displacements[i] + state_interfaces_[i].get_value());

      // 目標値をNaNに設定し，処理済みであることを示す
      (*current_ref)->displacements[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }

  // (オプション) コントローラ状態のパブリッシュ
  if (state_publisher_ && state_publisher_->trylock()) {
    state_publisher_->msg_.header.stamp = time;
    state_publisher_->msg_.set_point = command_interfaces_[CMD_MY_ITFS].get_value();
    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace controlko_controllers

// ここでプラグインをエクスポートする

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  controlko_controllers::DisplacementController,  // エクスポートするクラス
  controller_interface::ControllerInterface)      // その基底
