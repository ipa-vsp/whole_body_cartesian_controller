#include "wb_cartesian_controller/wb_cartesian_controller.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/joint/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial.hpp"
#include "pinocchio/spatial/fwd.hpp"
#include "wb_cartesian_controller/wb_cartesian_controller_parameters.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <memory>
#include <rclcpp/logging.hpp>
#include <utility>

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
  param_ = param_listener_->get_params();

  // ---- Problem dimensions -------------------------------------------------
  // The root joint is the mobile base (a planar joint, see on_init), everything
  // after it is the arm. The QP decision variable is v = q̇ ∈ ℝ^nv.
  n_v_ = model_->nv;
  n_base_ = (model_->joints.size() > 1) ? static_cast<Eigen::Index>(model_->joints[1].nv()) : 0;
  n_arm_ = n_v_ - n_base_;

  nonholonomic_ = param_.nonholonomic_base;
  if (nonholonomic_ && n_base_ != 3)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "nonholonomic_base is set but the root joint has %ld velocity dofs (3 expected for "
                 "a planar base)",
                 static_cast<long>(n_base_));
    return controller_interface::CallbackReturn::ERROR;
  }
  n_eq_ = nonholonomic_ ? 1 : 0;

  // ---- Cost / task weights ------------------------------------------------
  w_task_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(param_.task.weights.data());
  alpha_ = param_.task.gain;
  lm_damping_ = param_.task.lm_damping;
  lambda_base_ = param_.regularization.base;
  lambda_arm_ = param_.regularization.arm;
  pos_limit_margin_ = param_.limits.position_margin;

  // ---- Static velocity box ------------------------------------------------
  v_max_ = Eigen::VectorXd::Constant(n_v_, param_.limits.joint_velocity);
  if (n_base_ == 3)
  {
    v_max_[0] = param_.limits.base_linear_velocity;   // v_x   (body frame)
    v_max_[1] = param_.limits.base_linear_velocity;   // v_y   (body frame)
    v_max_[2] = param_.limits.base_angular_velocity;  // omega
  }
  // Tighten with the URDF velocity limits wherever the model provides one.
  for (Eigen::Index i = n_base_; i < n_v_; ++i)
  {
    const double lim = std::min(std::abs(model_->lowerVelocityLimit[i]), model_->upperVelocityLimit[i]);
    if (std::isfinite(lim) && lim > 0.0)
    {
      v_max_[i] = std::min(v_max_[i], lim);
    }
  }

  // Cache the (v_index, q_index) pairs of the scalar joints that carry finite
  // position limits; only those contribute a braking bound in computeQP().
  // Continuous joints (nq == 2, nv == 1) and the planar root are skipped.
  pos_limited_dofs_.clear();
  for (std::size_t j = 1; j < model_->joints.size(); ++j)
  {
    const auto& joint = model_->joints[j];
    if (joint.nq() != 1 || joint.nv() != 1)
    {
      continue;
    }
    const auto q_idx = static_cast<Eigen::Index>(joint.idx_q());
    const auto v_idx = static_cast<Eigen::Index>(joint.idx_v());
    const double lo = model_->lowerPositionLimit[q_idx];
    const double hi = model_->upperPositionLimit[q_idx];
    if (!std::isfinite(lo) || !std::isfinite(hi) || hi - lo <= 2.0 * pos_limit_margin_)
    {
      continue;
    }
    pos_limited_dofs_.emplace_back(v_idx, q_idx);
  }

  // ---- Pre-allocate every matrix touched by the real-time loop ------------
  J_frame.setZero(6, n_v_);
  J_task.setZero(6, n_v_);
  WJ_.setZero(6, n_v_);
  Werr_.setZero();
  err.setZero();
  H_.setZero(n_v_, n_v_);
  g_.setZero(n_v_);
  A_eq_.setZero(n_eq_, n_v_);
  b_eq_.setZero(n_eq_);
  // Equality (nonholonomic): planar-joint velocities are BODY-frame, so the
  // lateral direction is v[1] and the unicycle constraint is v_lat = 0.
  // Constant, hence built once here rather than on every tick.
  if (nonholonomic_)
  {
    A_eq_(0, 1) = 1.0;
  }
  l_box_.setZero(n_v_);
  u_box_.setZero(n_v_);
  v_.setZero(n_v_);
  dq_.setZero(n_v_);
  q_ = pinocchio::neutral(*model_);
  q_next_ = q_;
  X_des_ = pinocchio::SE3::Identity();

  qp_ = std::make_unique<proxsuite::proxqp::dense::QP<double>>(
      n_v_, n_eq_, /*n_in=*/0, /*box_constraints=*/true, proxsuite::proxqp::HessianType::Dense);
  qp_->settings.verbose = false;
  qp_->settings.compute_timings = false;
  qp_initialized_ = false;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
WbCartesianController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type WbCartesianController::update(const rclcpp::Time& /*time*/,
                                                                const rclcpp::Duration& period)
{
  const double dt = period.seconds();
  if (dt <= 0.0)
  {
    return controller_interface::return_type::OK;
  }

  // TODO(wb_cartesian_controller): fill q_ from the state interfaces and X_des_
  // from the command buffer; both are still placeholders (neutral / identity).
  computeTask(q_, X_des_);
  computeQP(q_, dt);

  // TODO(wb_cartesian_controller): write q_next_ / v_ to the command interfaces.
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

/**
 * Computes the 6D SE(3) Cartesian pose tracking error and its exact analytic task Jacobian.
 *
 * Target pose is X_des ∈ SE(3) (position and orientation) in the world frame (o).
 * Current end-effector pose is oMf[ee_id] ∈ SE(3) = oM_ee(q).
 *
 * Because SE(3) is a curved Lie group manifold and not a vector space,
 * relative displacement is computed via group multiplication:
 *
 *             [ World / Origin Frame (o) ]
 *                    /            \
 *      oMf[ee_id]   /              \  X_des
 *                  ▼                ▼
 *        [ Current EE Frame ] ───▶ [ Desired EE Frame ]
 *                     iMd = (oMf)⁻¹ · X_des = ᵉM_d ∈ SE(3)
 *
 * The 6D error vector in the Lie algebra se(3) ≅ ℝ⁶ is:
 *     err = log₆(iMd) = log₆(ᵉM_d)
 *
 * The exact time-derivative of the error with respect to joint velocities is:
 *     ė = ∂log₆((oM_ee)⁻¹ · X_des)/∂q · q̇ = -Jlog₆(iMd⁻¹) · J_frame · q̇
 *
 * Yielding the exact analytic task Jacobian:
 *     J_task = -Jlog₆(iMd⁻¹) · J_frame
 *
 * @param q      Joint configuration vector (size: nq).
 * @param X_des  Desired end-effector pose in the world frame ∈ SE(3).
 */
void WbCartesianController::computeTask(const Eigen::VectorXd& q, const pinocchio::SE3& X_des)
{
  // 1. Update kinematics and frame placements for current configuration q
  pinocchio::forwardKinematics(*model_, *data_, q);
  pinocchio::updateFramePlacements(*model_, *data_);
  pinocchio::computeJointJacobians(*model_, *data_, q);

  // 2. Relative displacement in local frame: iMd = (oM_ee)⁻¹ · X_des = ᵉM_d ∈ SE(3)
  const pinocchio::SE3 iMd = data_->oMf[ee_id].actInv(X_des);

  // 3. 6D error vector in the Lie algebra: err = log₆(iMd) ∈ se(3) ≅ ℝ⁶
  err = pinocchio::log6(iMd).toVector();

  // 4. Allocate matrices for (6 x nv) task space dimensions
  J_frame.resize(6, model_->nv);
  J_task.resize(6, model_->nv);
  J_frame.setZero();

  // 5. Local frame geometric Jacobian: ν_ee_local = J_frame · q̇
  pinocchio::getFrameJacobian(*model_, *data_, ee_id, pinocchio::LOCAL, J_frame);

  // 6. Exact analytic task Jacobian via the right Jacobian of SE(3):
  //    ∂(log₆(A⁻¹ · B))/∂A = -Jlog₆((A⁻¹ · B)⁻¹) · J_A
  //    J_task = -Jlog₆(iMd⁻¹) · J_frame
  pinocchio::Jlog6(iMd.inverse(), Jlog);
  J_task.noalias() = -Jlog * J_frame;
}

/**
 * Solves one tick of the whole-body differential IK as a QP in the generalised
 * velocity v = q̇ ∈ ℝ^nv, then integrates it into a configuration command.
 *
 *     min_v  ½‖J_task·v + α·err‖²_W  +  ½ vᵀΛv  +  ½ λ_lm‖v‖²
 *     s.t.   A_eq·v = 0                     (unicycle: body lateral velocity = 0)
 *            l_box ≤ v ≤ u_box              (velocity + position/braking limits)
 *
 * Expanding the tracking residual gives the standard ½vᵀHv + gᵀv form with
 *     H = J_taskᵀ·W·J_task + Λ + λ_lm·I,   g = α·J_taskᵀ·W·err
 * so the unconstrained optimum drives ė = J_task·v = -α·err, i.e. the Cartesian
 * error decays exponentially with time constant 1/α.
 *
 * Λ = diag(λ_base·I₃, λ_arm·I_na) is the block regularisation that makes the
 * base move reluctantly relative to the arm, and λ_lm = lm_damping·(errᵀW·err)
 * is Levenberg-Marquardt damping: large far from the target where the Jacobian
 * may be ill-conditioned, vanishing as the task converges.
 *
 * @param q   Current joint configuration (size nq); must be the same q that was
 *            passed to computeTask(), since J_task and err are evaluated there.
 * @param dt  Control period [s], used for the braking bounds and integration.
 */
void WbCartesianController::computeQP(const Eigen::VectorXd& q, double dt)
{
  // ---- Cost ---------------------------------------------------------------
  Werr_ = w_task_.asDiagonal() * err;
  WJ_ = w_task_.asDiagonal() * J_task;
  H_.noalias() = J_task.transpose() * WJ_;
  H_.diagonal().head(n_base_).array() += lambda_base_;  // base moves reluctantly
  H_.diagonal().tail(n_arm_).array() += lambda_arm_;
  H_.diagonal().array() += lm_damping_ * err.dot(Werr_);
  g_.noalias() = alpha_ * J_task.transpose() * Werr_;

  // ---- Box: velocity limits, tightened by the position limits -------------
  // A dof may travel at most (q_limit ∓ margin − q)/dt before leaving its range.
  // The bounds are clamped so that they always bracket zero: if a joint already
  // sits outside its limits the box stays feasible and only motion further out
  // is forbidden.
  l_box_ = -v_max_;
  u_box_ = v_max_;
  for (const auto& [v_idx, q_idx] : pos_limited_dofs_)
  {
    const double lo = (model_->lowerPositionLimit[q_idx] + pos_limit_margin_ - q[q_idx]) / dt;
    const double hi = (model_->upperPositionLimit[q_idx] - pos_limit_margin_ - q[q_idx]) / dt;
    l_box_[v_idx] = std::max(l_box_[v_idx], std::min(lo, 0.0));
    u_box_[v_idx] = std::min(u_box_[v_idx], std::max(hi, 0.0));
  }

  // ---- Solve --------------------------------------------------------------
  // The problem dimensions are fixed, so the workspace is set up once and every
  // later tick only refreshes the data and warm-starts from the last solution.
  // The general inequality block (C, l, u) is empty: the QP is built with
  // n_in = 0 in on_activate(). Collision barrier rows (h = dist(q) − d_margin
  // from coal, gradient via the contact-point Jacobians) would go there, which
  // means sizing n_in to the number of collision pairs at activation.
  if (!qp_initialized_)
  {
    qp_->init(H_, g_, A_eq_, b_eq_, proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt, l_box_, u_box_);
  }
  else
  {
    qp_->update(H_, g_, A_eq_, b_eq_, proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt, l_box_, u_box_);
  }
  qp_->solve();
  if (!qp_initialized_)
  {
    // Only switch to warm starting once a factorisation exists: asking for
    // WARM_START_WITH_PREVIOUS_RESULT before the first solve makes ProxQP skip
    // the setup and dereference an empty LDLT permutation.
    qp_->settings.initial_guess = proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
    qp_initialized_ = true;
  }

  if (qp_->results.info.status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED)
  {
    // Hold still rather than command the partial iterate of a failed solve.
    v_.setZero();
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "QP did not solve (status %d), commanding zero velocity",
                         static_cast<int>(qp_->results.info.status));
  }
  else
  {
    v_ = qp_->results.x;
  }

  // ---- Integrate on the manifold -----------------------------------------
  // q ⊕ (v·dt); the in-place overload keeps the real-time path allocation free.
  dq_ = v_ * dt;
  pinocchio::integrate(*model_, q, dq_, q_next_);
}

}  // namespace wb_cartesian_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(wb_cartesian_controller::WbCartesianController, controller_interface::ControllerInterface)