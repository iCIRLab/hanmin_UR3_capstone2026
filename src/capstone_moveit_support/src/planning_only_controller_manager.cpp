#include <string>
#include <vector>

#include "moveit/controller_manager/controller_manager.h"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace capstone_moveit_support
{

class PlanningOnlyControllerManager
  : public moveit_controller_manager::MoveItControllerManager
{
public:
  void initialize(const rclcpp::Node::SharedPtr & node) override
  {
    logger_ = node->get_logger();
    RCLCPP_INFO(
      logger_,
      "Planning-only controller boundary active; MoveIt execution disabled.");
  }

  moveit_controller_manager::MoveItControllerHandlePtr
  getControllerHandle(const std::string & name) override
  {
    (void)name;
    return nullptr;
  }

  void getControllersList(std::vector<std::string> & names) override
  {
    names.clear();
  }

  void getActiveControllers(std::vector<std::string> & names) override
  {
    names.clear();
  }

  void getControllerJoints(
    const std::string & name,
    std::vector<std::string> & joints) override
  {
    (void)name;
    joints.clear();
  }

  ControllerState getControllerState(const std::string & name) override
  {
    (void)name;
    return ControllerState();
  }

  bool switchControllers(
    const std::vector<std::string> & activate,
    const std::vector<std::string> & deactivate) override
  {
    return activate.empty() && deactivate.empty();
  }

private:
  rclcpp::Logger logger_{rclcpp::get_logger(
      "capstone_moveit_support.planning_only_controller_manager")};
};

}  // namespace capstone_moveit_support

PLUGINLIB_EXPORT_CLASS(
  capstone_moveit_support::PlanningOnlyControllerManager,
  moveit_controller_manager::MoveItControllerManager)
