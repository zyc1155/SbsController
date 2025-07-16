#pragma once

#include <stdio.h>
#include <mc_control/mc_controller.h>
// #include <mc_tasks/PostureTask.h>
#include "SbsController/ModifiedTasks.h"
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "geometry_msgs/PointStamped.h"

#include "SbsController/api.h"

using Eigen::Matrix3d;
using Eigen::MatrixXd;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector6d;
using Eigen::VectorXd;

const double GRAVITY = 9.8;
const double HEIGHTREF = 0.9;
const double A_LIM = 0.45;

struct SbsController_DLLAPI SbsController : public mc_control::MCController
{
  SbsController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config);

  bool run() override;

  void reset(const mc_control::ControllerResetData &reset_data) override;

  void get_values();
  void set_CtrlPos();
  void state_swiching();
  void set_desiredVel();
  void set_desiredTask();

  double sat_func(double lim, double val);
  Vector3d sat_func(double lim, const Vector3d &val);
  sva::ForceVecd error_func(const sva::ForceVecd &f_m);

protected:
  void createGUI();

private:
  // Falcon_Driver right_falcon, left_falcon;
  void CommandCallback_Left(const geometry_msgs::PointStamped &msg);
  void CommandCallback_Right(const geometry_msgs::PointStamped &msg);

  std::shared_ptr<ros::NodeHandle> nh_;
  // ros::CallbackQueue callbackQueue_;
  ros::CallbackQueue spinner;
  ros::Subscriber right_falcon, left_falcon;

  mc_rtc::Configuration config_;

  std::shared_ptr<mc_tasks::OrientationTask> otTask;
  std::shared_ptr<mc_tasks::EndEffectorTask_NoGUI> efTask_left, efTask_right;
  std::shared_ptr<mc_tasks::lipm_stabilizer::StabilizerTask> lipmTask;

  FILE *fp;
  bool first;
  double leftFootRatio;
  double omega;
  double ttime, timer_mode;
  int ctrl_mode, ctrl_mode2;
  std::chrono::_V2::system_clock::time_point z_start;
  Matrix3d COMShifter_Kp, COMShifter_Kd;
  double copAdmittance_ds, copAdmittance_ss;

  // for new trajectory
  Vector3d direction;

  double limit_vel, limit_acc, limit_jerk;

  double kp_dcm, kd_dcm;
  void cal_motion(const Vector3d &target, const Vector3d &W_p_GW_0, const Vector3d &W_v_GW_0, const Vector3d &W_a_GW_0, const Vector3d &n);
  //////

  Vector3d W_pos_A, W_pos_B;
  Vector3d posRA_, posRB_;
  Vector3d posRA, posRB, posRAp, posRBp, vel_posRA, vel_posRB;
  Vector3d Q_ref, Q_ep, Q_epd, W_Q_W, W_Q_A, W_Q_B, W_Q, A_Q_A, B_Q_B;

  Vector3d W_p_AW, W_p_BW, W_p_GW, W_p_BW_, W_p_GW_p, W_p_AW_;
  Matrix3d R_0_mIMU;
  Matrix3d W_R_A, W_R_B, W_R_H;
  Matrix3d W_R_B_ref, W_R_A_ref;
  Vector3d A_p_BA_ref, B_p_AB_ref, W_p_GW_ref, W_p_GWd;
  sva::PTransformd A_T_BA_d, B_T_AB_d;
  Vector3d W_v_GW, W_v_GWd, W_a_GW, W_a_GWd, W_v_GW_p, W_v_GW_ref, W_a_GW_ref, W_a_GWdp;

  Vector3d A_f_A, B_f_B, A_n_A, B_n_B, W_f_A, W_f_B, W_n_A, W_n_B;

  sva::ForceVecd left, right, world_wrench;
  sva::ForceVecd force_limit, error_limit, admittance;

  // Set 'true' to lift right foot
  bool rightFootLift_ = false;
};
