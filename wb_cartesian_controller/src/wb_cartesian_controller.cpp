#include "wb_cartesian_controller/wb_cartesian_controller.hpp"

namespace wb_cartesian_controller{
    WbCartesianController::WbCartesianController(): controller_interface::ControllerInterface()
    {
    }

    controller_interface::CallbackReturn WbCartesianController::on_init()
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn WbCartesianController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn WbCartesianController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn WbCartesianController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type WbCartesianController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        return controller_interface::return_type::OK;
    }

    controller_interface::return_type WbCartesianController::on_command_received(const trajectory_msgs::msg::JointTrajectory & /*command*/)
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
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  wb_cartesian_controller::WbCartesianController, controller_interface::ControllerInterface)