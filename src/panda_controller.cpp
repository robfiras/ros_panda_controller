// Code adapted from:
// https://github.com/frankaemika/franka_ros/blob/noetic-devel/franka_example_controllers/src/joint_velocity_example_controller.cpp
//
// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <ros_panda_controller/panda_controller.h>

#include <cmath>
#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros_panda_controller/AgentActions.h>
#include <ros_panda_controller/RobotModel.h>



namespace ros_panda_controller {


bool PandaController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
    velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
    if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "PandaController: Error getting velocity joint interface from hardware!");
    return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("PandaController: Could not parse joint names");
    }
    if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("PandaController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
    }

    velocity_joint_handles_.resize(7);
    for (size_t i = 0; i < 7; ++i) {
        try {
          velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
          ROS_ERROR_STREAM(
              "PandaController: Exception getting joint handles: " << ex.what());
          return false;
        }
    }

    std::string arm_id = "";
    if (!node_handle.getParam("arm_id", arm_id)) {
      ROS_ERROR("PandaController: Could not read parameter arm_id");
      return false;
    }

    model_interface_ = robot_hardware->get<franka_hw::FrankaModelInterface>();
    if (model_interface_ == nullptr) {
      ROS_ERROR_STREAM("PandaController: Error getting model interface from hardware");
      return false;
    }

    try {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
          model_interface_->getHandle(arm_id + "_model"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "PandaController: Exception getting model handle from interface: " << ex.what());
      return false;
    }

    // create a subscriber, which receives different commands
    action_sub = node_handle.subscribe("/rl_agent_actions", 10, &PandaController::subscriber_callback,this);
    // create a publisher, which publishes the robot state
    state_pub = node_handle.advertise<ros_panda_controller::RobotModel>("/robot_model_publisher", 1);

    return true;
}

void PandaController::subscriber_callback(const ros_panda_controller::AgentActions& action){
    agent_actions[0] = action.actions[0];
    agent_actions[1] = action.actions[1];
    agent_actions[2] = action.actions[2];
    agent_actions[3] = action.actions[3];
    agent_actions[4] = action.actions[4];
    agent_actions[5] = action.actions[5];
    agent_actions[6] = action.actions[6];
    return;
}

ros_panda_controller::RobotModel PandaController::get_empty_model_msg(){
    // until now, only the Jacobian is part of the model msg
    ros_panda_controller::RobotModel model_msg;
    model_msg.zeroJacobian = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    return model_msg;
}

void PandaController::starting(const ros::Time& /* time */) {
    elapsed_time_ = ros::Duration(0.0);
}

void PandaController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
    elapsed_time_ += period;

    try {
        // send the latest actions received from the agent
        uint i = 0;
        for (auto joint_handle : velocity_joint_handles_) {
        joint_handle.setCommand(agent_actions[i]);
        i++;
        }

        // we need to send some model information for redundancy resolution
        // receives base jacobian
       std::array<double, 42> endeffector_zero_jacobian =
           model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
       i = 0;
       // create empty model message, fill it with Jacobian and send it
       ros_panda_controller::RobotModel model_msg = get_empty_model_msg();
       for (const double &elem : endeffector_zero_jacobian){
           model_msg.zeroJacobian[i] = elem;
           i++;
       }
       state_pub.publish(model_msg);

    } catch (const hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM("I have catched this error: " << e.what());
    }

}

void PandaController::stopping(const ros::Time& /*time*/) {
    // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
    // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
    // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}


}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(ros_panda_controller::PandaController,
                       controller_interface::ControllerBase)

