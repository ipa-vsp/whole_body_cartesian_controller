#ifndef WB_CARTESIAN_CONTROLLER_WB_CARTESIAN_CONTROLLER_HPP
#define WB_CARTESIAN_CONTROLLER_WB_CARTESIAN_CONTROLLER_HPP

#include <array>
#include <atomic>
#include <memory>
#include <utility>
#include <vector>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/joint/joints.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/spatial/fwd.hpp"

#include "proxsuite/proxqp/dense/dense.hpp"

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_server_goal_handle.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <wb_cartesian_controller/wb_cartesian_controller_parameters.hpp>

using namespace std::chrono_literals;

namespace wb_cartesian_controller
{
  using OdomType = nav_msgs::msg::Odometry;
  using CmdVelType = geometry_msgs::msg::TwistStamped;

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

  // ---- ros2_control interfaces ---------------------------------------------
  // The loaned handles themselves live in the base class vectors
  // ControllerInterfaceBase::state_interfaces_ / ::command_interfaces_ and
  // arrive in exactly the order listed by *_interface_configuration() below,
  // i.e. joint-major: handle index (i * types.size() + k) is joint i's
  // interface type k. Nothing here may be named state_interfaces_ /
  // command_interfaces_, which would shadow those base members.
  std::vector<std::string> joint_names_;              // param 'joints'
  std::vector<std::string> command_joint_names_;      // 'command_joints', else 'joints'
  std::vector<std::string> state_interface_types_;    // e.g. {position, velocity}
  std::vector<std::string> command_interface_types_;  // e.g. {position}
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_accelerations_;

  // Pinocchio (q, v) index of every claimed joint, resolved in on_configure().
  // The model carries the planar root joint first, so these are NOT identities:
  // they are what maps an interface handle onto a column of q_ / v_.
  std::vector<Eigen::Index> state_joint_q_index_;
  std::vector<Eigen::Index> state_joint_v_index_;
  std::vector<Eigen::Index> command_joint_q_index_;
  std::vector<Eigen::Index> command_joint_v_index_;

  // Layout of state_interfaces_, resolved once in on_configure() so that the
  // real-time path indexes the handles instead of searching for the position
  // type on every tick: handle (i * stride + offset) is joint i's position.
  std::size_t state_interface_stride_{ 0 };
  std::size_t position_state_offset_{ 0 };

  // Slice of q_ owned by the mobile base, resolved in on_activate(). update()
  // copies the odometry-derived configuration into [base_q_index_, +base_nq_).
  Eigen::Index base_q_index_{ 0 };
  Eigen::Index base_nq_{ 0 };

  bool has_speed_scaling_state_{ false };
  bool has_speed_scaling_command_{ false };

  // Pinocchio model and data
  std::shared_ptr<pinocchio::Model> model_;
  std::shared_ptr<pinocchio::Data> data_;
  pinocchio::FrameIndex ee_id;
  pinocchio::FrameIndex base_id;
  pinocchio::Data::Matrix6x J_frame;
  Eigen::MatrixXd J_task;
  Eigen::Matrix<double, 6, 6> Jlog;
  Eigen::Matrix<double, 6, 1> err;

  // Configuration: current, and the one commanded by the QP after integration
  Eigen::VectorXd q_;
  Eigen::VectorXd q_next_;
  pinocchio::SE3 X_des_;

  // ---- QP ------------------------------------------------------------------
  // Decision variable is the generalised velocity v = q̇ ∈ ℝ^nv, ordered as
  // [base (planar root joint) dofs ; arm dofs].
  std::unique_ptr<proxsuite::proxqp::dense::QP<double>> qp_;
  bool qp_initialized_{ false };

  Eigen::Index n_v_{ 0 };     // total dofs           (= model_->nv)
  Eigen::Index n_base_{ 0 };  // dofs of the root joint (3 for a planar base)
  Eigen::Index n_arm_{ 0 };   // remaining actuated dofs
  Eigen::Index n_eq_{ 0 };    // number of equality rows (1 if nonholonomic)

  Eigen::Matrix<double, 6, 1> w_task_;           // diagonal task weights
  Eigen::Matrix<double, 6, 1> Werr_;             // W · err            (cached)
  Eigen::Matrix<double, 6, Eigen::Dynamic> WJ_;  // W · J_task         (cached)
  Eigen::MatrixXd H_;
  Eigen::VectorXd g_;
  Eigen::MatrixXd A_eq_;
  Eigen::VectorXd b_eq_;
  Eigen::VectorXd l_box_, u_box_;  // velocity box bounds, rebuilt every tick
  Eigen::VectorXd v_max_;          // static per-dof velocity limits
  Eigen::VectorXd v_;              // QP solution
  Eigen::VectorXd dq_;             // v_ · dt

  // (v_index, q_index) of the scalar joints that carry finite position limits.
  std::vector<std::pair<Eigen::Index, Eigen::Index>> pos_limited_dofs_;

  double alpha_{ 1.0 };         // task proportional gain
  double lambda_base_{ 0.0 };   // base regularisation weight
  double lambda_arm_{ 0.0 };    // arm regularisation weight
  double lm_damping_{ 0.0 };    // Levenberg-Marquardt damping factor
  double pos_limit_margin_{ 0.0 };
  bool nonholonomic_{ false };

  std::shared_ptr<ParamListener> param_listener_;
  Params param_;

  // Subscriber and publisher
  realtime_tools::RealtimeBuffer<std::array<double, 4>> rt_base_configuration_{
    std::array<double, 4>{ 0.0, 0.0, 1.0, 0.0 }
  };
  // The buffer above is seeded with the identity pose, which is indistinguishable
  // from a real measurement. This says whether /odom has actually been heard.
  std::atomic<bool> base_configuration_received_{ false };
  rclcpp::Subscription<OdomType>::SharedPtr odom_subscriber_;

  realtime_tools::RealtimeThreadSafeBox<CmdVelType> rt_cmd_publisher_;
  CmdVelType cmd_vel_;
  rclcpp::Publisher<CmdVelType>::SharedPtr cmd_vel_publisher_;


  void computeTask(const Eigen::VectorXd& q, const pinocchio::SE3& X_des);
  void computeQP(const Eigen::VectorXd& q, double dt);
  bool resolveJointIndices(const std::vector<std::string>& joint_names, std::vector<Eigen::Index>& q_index,
                           std::vector<Eigen::Index>& v_index);
};
}  // namespace wb_cartesian_controller

#endif  // WB_CARTESIAN_CONTROLLER_WB_CARTESIAN_CONTROLLER_HPP