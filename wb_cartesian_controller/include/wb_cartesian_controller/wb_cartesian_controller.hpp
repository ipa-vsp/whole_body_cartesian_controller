#ifndef WB_CARTESIAN_CONTROLLER_WB_CARTESIAN_CONTROLLER_HPP
#define WB_CARTESIAN_CONTROLLER_WB_CARTESIAN_CONTROLLER_HPP

#include <memory>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/joint/joints.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/spatial/fwd.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_server_goal_handle.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include <wb_cartesian_controller/wb_cartesian_controller_parameters.hpp>

using namespace std::chrono_literals;

namespace wb_cartesian_controller
{
class WbCartesianController : public controller_interface::ControllerInterface
{
public:
  WbCartesianController();
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  controller_interface::return_type on_command_received(const trajectory_msgs::msg::JointTrajectory& command);
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:
  realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectory>> rt_command_buffer_;
  std::vector<hardware_interface::LoanedCommandInterface> joint_command_interfaces_;
  std::vector<hardware_interface::LoanedStateInterface> joint_state_interfaces_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> state_interfaces_;
  std::vector<std::string> command_interfaces_;
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_accelerations_;

  // Pinocchio model and data
  std::shared_ptr<pinocchio::Model> model_;
  std::shared_ptr<pinocchio::Data> data_;
  pinocchio::FrameIndex ee_id;
  pinocchio::FrameIndex base_id;
  pinocchio::Data::Matrix6x J_frame;
  Eigen::MatrixXd J_task;
  Eigen::Matrix<double, 6, 6> Jlog;
  Eigen::Matrix<double, 6, 1> err;

  std::shared_ptr<ParamListener> param_listener_;
  Params param_;

  void computeTask(const Eigen::VectorXd& q, const pinocchio::SE3& X_des);
};
}  // namespace wb_cartesian_controller

#endif  // WB_CARTESIAN_CONTROLLER_WB_CARTESIAN_CONTROLLER_HPP