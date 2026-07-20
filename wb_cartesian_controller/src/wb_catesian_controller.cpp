#include "wb_cartesian_controller/wb_cartesian_controller.hpp"

namespace wb_cartesian_controller{
    WbCartesianController::WbCartesianController(): controller_interface::ControllerInterface()
    {

    }

    controller_interface::CallbackReturn on_init()
    {
        
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  wb_cartesian_controller::WbCartesianController, controller_interface::ControllerInterface)