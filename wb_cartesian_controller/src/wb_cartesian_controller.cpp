#include "wb_cartesian_controller/wb_cartesian_controller.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/joint/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial.hpp"
#include "pinocchio/spatial/fwd.hpp"
#include "wb_cartesian_controller/wb_cartesian_controller_parameters.hpp"
#include <cstdio>
#include <exception>
#include <memory>
#include <rclcpp/logging.hpp>

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

  model_ = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelPlanar(), *model_);
  data_ = std::make_shared<pinocchio::Data>(*model_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
WbCartesianController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  ee_id = model_->getFrameId(param_.end_effector_frame);
  RCLCPP_INFO(get_node()->get_logger(), "End effector name: %s frame ID: %ld\n", param_.end_effector_frame.c_str(),
              ee_id);
  RCLCPP_INFO(get_node()->get_logger(), "Base frame: %s", param_.base_frame.c_str());
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

void WbCartesianController::computeTask(const Eigen::VectorXd& q, const pinocchio::SE3& X_des)
{
  pinocchio::forwardKinematics(*model_, *data_, q);
  pinocchio::updateFramePlacements(*model_, *data_);
  pinocchio::computeJointJacobians(*model_, *data_, q);

  const pinocchio::SE3 iMd = data_->oMf[ee_id].actInv(X_des);
  err = pinocchio::log6(iMd).toVector();
  J_frame.resize(6, model_->nv);
  J_task.resize(6, model_->nv);

  J_frame.setZero();
  pinocchio::getFrameJacobian(*model_, *data_, ee_id, pinocchio::LOCAL, J_frame);
  pinocchio::Jlog6(iMd.inverse(), Jlog);
  J_task.noalias() = -Jlog * J_frame;
}
}  // namespace wb_cartesian_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(wb_cartesian_controller::WbCartesianController, controller_interface::ControllerInterface)