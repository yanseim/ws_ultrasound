// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_example_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

//yxj 0603
#include <fstream>

namespace franka_example_controllers {

class MyCartesianImpedanceTrajController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  double filter_params_{0.005};
  double nullspace_stiffness_{5.0};
  double nullspace_stiffness_target_{20.0};
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_,orientation_d_last_;
  std::mutex position_and_orientation_d_target_mutex_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;

  // yxj 0525
  Eigen::Matrix<double, 6, 7> jacobian_last_,jacobian_analytic_last_;
  Eigen::Matrix<double, 6, 6> M_d;
  Eigen::Matrix<double, 6, 1> error_last_;
  Eigen::Matrix<double, 7, 1> tau_ext_initial_;
  Eigen::Matrix<double, 6, 6> K_d;
  Eigen::Matrix<double, 6, 6> D_d;
  Eigen::Matrix<double, 7, 1> dq_last_;
  Eigen::Matrix<double, 6, 1> F_ext_filtered_;
  Eigen::Matrix<double, 7, 1> orientation;
  Eigen::Vector3d omega_d_last_;
  int yxj_counter=0;
  std::vector<Eigen::Matrix<double, 6, 1>> log_F_ext,log_error,log_F_ext_filtered;
  std::vector<double> log_t;
  std::vector<Eigen::Matrix<double, 7, 1>> log_tau;

  //save data; yxj 0603
  void saveData6(std::vector<Eigen::Matrix<double,6,1>> &Data, std::string filePath);
  void saveData7(std::vector<Eigen::Matrix<double,7,1>> &Data, std::string filePath);
  void saveData_time(std::vector<double> &Data, std::string filePath);
  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>
      dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
  void complianceParamCallback(franka_example_controllers::compliance_paramConfig& config,
                               uint32_t level);

  // Equilibrium pose subscriber
  ros::Subscriber sub_equilibrium_pose_;
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  // yxj0625
  ros::Duration elapsed_time_;
  double radius = 0.05;
};

}  // namespace franka_example_controllers
