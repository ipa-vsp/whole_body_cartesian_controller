#include "wb_cartesian_controller/wb_cartesian_controller.hpp"
#include "pinocchio/multibody/joint/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "wb_cartesian_controller/wb_cartesian_controller_parameters.hpp"
#include <cstdio>
#include <exception>
#include <memory>

namespace wb_cartesian_controller
{
WbCartesianController::WbCartesianController() : controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn WbCartesianController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    param_ = param_listener_->get_params();
  }
  catch (const std::exception& e)
  {
    fprintf(stderr, "Execption thrown during init station with the message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  std::string urdf_path = param_.urdf_path;
  RCLCPP_INFO(get_node()->get_logger(), "Loading URDF from %s\n", urdf_path.c_str());

  pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelPlanar(), model_);
  data_ = pinocchio::Data(model_);
  const auto ee_id = model_.getFrameId(param_.end_effector_frame);
  RCLCPP_INFO(get_node()->get_logger(), "End effector frame ID: %ld\n", ee_id);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
WbCartesianController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
WbCartesianController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
WbCartesianController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type WbCartesianController::update(const rclcpp::Time& /*time*/,
                                                                const rclcpp::Duration& /*period*/)
{
  return controller_interface::return_type::OK;
}

controller_interface::return_type
WbCartesianController::on_command_received(const trajectory_msgs::msg::JointTrajectory& /*command*/)
{
  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration WbCartesianController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::NONE;
  return conf;
}

controller_interface::InterfaceConfiguration WbCartesianController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::NONE;
  return conf;
}
}  // namespace wb_cartesian_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(wb_cartesian_controller::WbCartesianController, controller_interface::ControllerInterface)