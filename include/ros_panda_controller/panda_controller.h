// Code adapted from:
// https://github.com/frankaemika/franka_ros/blob/noetic-devel/franka_example_controllers/include/franka_example_controllers/joint_velocity_example_controller.h
//
// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#pragma once

#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros_panda_controller/AgentActions.h>
#include <ros_panda_controller/RobotModel.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>


namespace ros_panda_controller {

class PandaController : public controller_interface::MultiInterfaceController<
                                           franka_hw::FrankaModelInterface,
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaStateInterface> {
public:
    bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
    void update(const ros::Time&, const ros::Duration& period) override;
    void starting(const ros::Time&) override;
    void stopping(const ros::Time&) override;




private:
    ros::Duration elapsed_time_;
    hardware_interface::VelocityJointInterface* velocity_joint_interface_;
    std::vector<hardware_interface::JointHandle> velocity_joint_handles_;

    franka_hw::FrankaModelInterface* model_interface_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

    ros::Subscriber action_sub;
    ros::Publisher state_pub;
    double agent_actions[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    ros_panda_controller::RobotModel get_empty_model_msg();
    void subscriber_callback(const ros_panda_controller::AgentActions& action);
};

}  // namespace franka_example_controllers
