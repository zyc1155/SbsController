#include "SbsController/SbsController.h"
#include <mc_rtc/ros.h>

SbsController::SbsController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
    : mc_control::MCController(rm, dt)
{
  config_.load(config);

  solver().setContacts({{}});

  dof << 0, 0, 1, 1, 1, 0;
  addContact({robot().name(), "ground", "LeftFoot", "AllGround", 0.7, dof});
  addContact({robot().name(), "ground", "RightFoot", "AllGround", 0.7, dof});

  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  solver().addConstraintSet(selfCollisionConstraint);
  solver().addConstraintSet(*compoundJointConstraint);

  solver().addTask(postureTask);

  std::vector<std::string> activeJoints = {"Root", "RCY", "RCR", "RCP", "RKP", "RAP", "RAR", "LCY", "LCR", "LCP", "LKP", "LAP", "LAR"};
  std::vector<std::string> headJoints = {"HY", "HP"};
  std::vector<std::string> UnactiveJoints;
  UnactiveJoints.reserve(activeJoints.size() + headJoints.size());
  UnactiveJoints.insert(UnactiveJoints.end(), activeJoints.begin(), activeJoints.end());
  UnactiveJoints.insert(UnactiveJoints.end(), headJoints.begin(), headJoints.end());

  postureTask->stiffness(10.0);
  postureTask->weight(1000.0);

  postureTask->selectUnactiveJoints(solver(), UnactiveJoints);

  efTask_left = std::make_shared<mc_tasks::EndEffectorTask_NoGUI>("Lleg_Link5", robots(), 0);
  efTask_right = std::make_shared<mc_tasks::EndEffectorTask_NoGUI>("Rleg_Link5", robots(), 0);
  efTask_left->selectActiveJoints(solver(), activeJoints);
  efTask_right->selectActiveJoints(solver(), activeJoints);
  solver().addTask(efTask_left);
  solver().addTask(efTask_right);

  otTask = std::make_shared<mc_tasks::OrientationTask>("Head_Link1_Plan2", robots(), 0);
  otTask->selectActiveJoints(solver(), headJoints);

  solver().addTask(otTask);

  copAdmittance_ss = {0.005, 0.008};
  copAdmittance_ds = {0.01, 0.01};

  dcmGainP_ds << 2.5, 2.5;
  dcmGainP_ss << 1.5, 2.5;

  dcmGainI_ds << 15, 15;
  dcmGainI_ss = dcmGainI_ds;

  dcmGainD_ds << 0.2, 0.2;
  dcmGainD_ss = dcmGainD_ds;

  auto stabiConf = robot().module().defaultLIPMStabilizerConfiguration();
  stabiConf.comHeight = HEIGHTREF;
  stabiConf.torsoPitch = 0;
  stabiConf.zmpcc.comAdmittance = Vector2d{0.0, 0.0};
  // stabiConf.copAdmittance = copAdmittance_ds;
  // stabiConf.dcmPropGain = 1.5; // 2.0;
  // stabiConf.dcmIntegralGain = 15;
  // stabiConf.dcmDerivGain = 0.2;
  stabiConf.dcmDerivatorTimeConstant = 5;
  stabiConf.dcmIntegratorTimeConstant = 5; // 5.0

  lipmTask = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(
      solver().robots(),
      solver().realRobots(),
      0,
      stabiConf.leftFootSurface,
      stabiConf.rightFootSurface,
      stabiConf.torsoBodyName,
      solver().dt());

  lipmTask->setCtrlMode(ctrl_mode);
  lipmTask->configure(stabiConf);
  lipmTask->setContacts({mc_tasks::lipm_stabilizer::ContactState::Left, mc_tasks::lipm_stabilizer::ContactState::Right});

  solver().addTask(lipmTask);

  // for new trajectory
  omega = sqrt(GRAVITY / HEIGHTREF);

  limit_vel = 0.4;
  limit_acc = 0.45;
  limit_jerk = 8;

  kp_dcm = 30;
  kd_dcm = 2 * sqrt(kp_dcm);
  /////
  coff_falcon << 10.0, 5.0, 5.0;

  datastore().make_call("KinematicAnchorFrame::" + robot().name(),
                        [this](const mc_rbdyn::Robot &robot_)
                        {
                          return sva::interpolate(robot_.surfacePose("LeftFoot"),
                                                  robot_.surfacePose("RightFoot"),
                                                  0.5);
                        });

#ifdef USE_FALCON
  // Skip if ROS is not initialized
  if (!mc_rtc::ROSBridge::get_node_handle())
  {
    mc_rtc::log::error("[TeleopState] ROS is not initialized.");
    return;
  }
  // Setup ROS
  nh_ = std::make_shared<ros::NodeHandle>();
  nh_->setCallbackQueue(&spinner);
  // Use a dedicated queue so as not to call callbacks of other modules
  left_falcon = nh_->subscribe("/ros_falcon_left/falconPos", 10, &SbsController::CommandCallback_Left, this);
  right_falcon = nh_->subscribe("/ros_falcon_right/falconPos", 10, &SbsController::CommandCallback_Right, this);
  left_falcon_button = nh_->subscribe("/ros_falcon_left/falcon_button", 1, &SbsController::ButtonChanged_Left, this);
  right_falcon_button = nh_->subscribe("/ros_falcon_right/falcon_button", 1, &SbsController::ButtonChanged_Right, this);
#endif

  createGUI();

  logger().addLogEntries(
      this,
      "leftfoot_ankelPosition", [this]()
      { return W_p_AaW; },
      "rightfoot_ankelPosition", [this]()
      { return W_p_BaW; },
      "leftfoot_transform", [this]()
      { return W_T_A; },
      "rightfoot_transform", [this]()
      { return W_T_B; },
      "ref_COM", [this]()
      { return W_p_GW_d; },
      "real_COM_p", [this]()
      { return W_p_GW; },
      "real_COM_v", [this]()
      { return W_v_GW; },
      "real_COM_a", [this]()
      { return W_a_GW; },
      "ZMP_total", [this]()
      { return W_Q_W; },
      "ZMP_L", [this]()
      { return A_Q_A; },
      "ZMP_R", [this]()
      { return B_Q_B; },
      "COP_L", [this]()
      { return A_Q_A; },
      "COP_R", [this]()
      { return B_Q_B; },
      "EZMP_desired", [this]()
      { return Q_epd; },
      "EZMP_real", [this]()
      { return Q_ep; });

  mc_rtc::log::success("SbsController init done ");
}

void SbsController::reset(const mc_control::ControllerResetData &reset_data)
{
  mc_control::MCController::reset(reset_data);

  ctrl_mode = 0;
  ctrl_mode2 = 0;
  timer_mode = 0;
  start_time = std::chrono::high_resolution_clock::now();
  passed_time = .0;

  center_button_down_r = false;
  center_button_down_l = false;

  W_T_A = realRobot().surfacePose("LeftFootCenter");
  W_p_AcW = W_T_A.translation();
  W_T_B = realRobot().surfacePose("RightFootCenter");
  W_p_BcW = W_T_B.translation();
  W_p_GW_ref = (W_p_AcW + W_p_BcW) / 2.0;
  W_p_GW_ref(2) += HEIGHTREF;
  W_p_GW = realRobot().com();
  W_p_GW_d = W_p_GW;
  if ((W_p_GW_ref - W_p_GW_d).norm() > 1e-6)
    direction = (W_p_GW_ref - W_p_GW_d).normalized();
  else
    direction << .0, .0, 1;

  W_R_A_ref = Matrix3d::Identity();
  W_R_B_ref = Matrix3d::Identity();

  W_p_AaW = realRobot().frame("Lleg_Link5").position().translation();
  W_p_BaW = realRobot().frame("Rleg_Link5").position().translation();

  W_p_A_ref = W_p_AaW;
  W_p_B_ref = W_p_BaW;

  W_p_A_ref_p = W_p_A_ref;
  W_p_B_ref_p = W_p_B_ref;

  W_p_AB_ref_0 = W_p_AaW - W_p_BaW;
  W_p_BA_ref_0 = W_p_BaW - W_p_AaW;

  W_p_AB_ref = W_p_AB_ref_0;
  W_p_BA_ref = W_p_BA_ref_0;
  W_p_AB_ref_p = W_p_AB_ref;
  W_p_BA_ref_p = W_p_BA_ref;

  W_v_A_ref = Vector3d::Zero();
  W_v_B_ref = Vector3d::Zero();

  efTask_left->dimWeight(Vector3d::Zero());
  efTask_right->dimWeight(Vector3d::Zero());

  lipmTask->reset();
  lipmTask->setCtrlMode(ctrl_mode);
  lipmTask->copAdmittance(copAdmittance_ds);
  lipmTask->dcmGains(dcmGainP_ds, dcmGainI_ds, dcmGainD_ds);

  otTask->reset();
  efTask_left->reset();
  efTask_right->reset();
}

bool SbsController::run()
{

#ifdef USE_FALCON
  spinner.callAvailable(ros::WallDuration());
#endif

  passed_time = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time).count();
  get_values();
  state_swiching();
  set_desiredVel();
  set_desiredTask();

  return mc_control::MCController::run();
}

void SbsController::get_values()
{
  sva::PTransformd ZMP_frame(Matrix3d::Identity(), Vector3d::Zero());
  std::vector<std::string> activeJoints = {"LCY", "LCR", "LCP", "LKP", "LAP", "LAR", "RCY", "RCR", "RCP", "RKP", "RAP", "RAR"};
  std::vector<std::string> sensornames = {"LeftFootForceSensor", "RightFootForceSensor"};

  // left = realRobot().surfaceWrench("LeftFootCenter");
  // right = realRobot().surfaceWrench("RightFootCenter");
  left = robot().forceSensor("LeftFootForceSensor").worldWrench(realRobot());
  right = robot().forceSensor("RightFootForceSensor").worldWrench(realRobot());

  W_f_A = left.force();
  W_n_A = left.moment();

  W_f_B = right.force();
  W_n_B = right.moment();

  world_wrench = realRobot().netWrench(sensornames);

  if (world_wrench.force()[2] > 1.0)
    W_Q_W = realRobot().zmp(world_wrench, ZMP_frame);
  // std::vector<std::vector<double> > zyc;
  // zyc=robot().q();

  left = robot().surfaceWrench("LeftFootCenter");
  right = robot().surfaceWrench("RightFootCenter");

  A_f_A = left.force();
  A_n_A = left.moment();

  B_f_B = right.force();
  B_n_B = right.moment();

  W_p_GW = realRobot().com();
  W_v_GW = realRobot().comVelocity();
  W_a_GW = realRobot().comAcceleration();

  Q_ep = W_p_GW - W_a_GW / (omega * omega);
  Q_ep(2) -= HEIGHTREF;

  W_T_A = realRobot().surfacePose("LeftFootCenter");
  W_p_AcW = W_T_A.translation();
  W_R_A = W_T_A.rotation();

  W_T_B = realRobot().surfacePose("RightFootCenter");
  W_p_BcW = W_T_B.translation();
  W_R_B = W_T_B.rotation();

  W_p_AaW = realRobot().frame("Lleg_Link5").position().translation();
  W_p_BaW = realRobot().frame("Rleg_Link5").position().translation();

  if (A_f_A(2) > 1e-1)
    A_Q_A << -A_n_A(1) / A_f_A(2), A_n_A(0) / A_f_A(2), .0;
  else
    A_Q_A = Vector3d::Zero();

  if (B_f_B(2) > 1e-1)
    B_Q_B << -B_n_B(1) / B_f_B(2), B_n_B(0) / B_f_B(2), .0;
  else
    B_Q_B = Vector3d::Zero();

  if (W_f_A(2) > 1e-1)
    W_Q_A << -W_n_A(1) / W_f_A(2), W_n_A(0) / W_f_A(2), .0;
  else
    W_Q_A = Vector3d::Zero();

  if (W_f_B(2) > 1e-1)
    W_Q_B << -W_n_B(1) / W_f_B(2), W_n_B(0) / W_f_B(2), .0;
  else
    W_Q_B = Vector3d::Zero();

  W_Q = Vector3d::Zero();

  if (W_f_A(2) > 1e-1)
  {
    if (W_f_B(2) > 1e-1)
    {
      W_Q(0) = (W_f_A(2) * W_Q_A(0) + W_f_B(2) * W_Q_B(0)) / (W_f_A(2) + W_f_B(2));
      W_Q(1) = (W_f_A(2) * W_Q_A(1) + W_f_B(2) * W_Q_B(1)) / (W_f_A(2) + W_f_B(2));
    }
    else
    {
      W_Q(0) = (W_f_A(2) * W_Q_A(0)) / (W_f_A(2));
      W_Q(1) = (W_f_A(2) * W_Q_A(1)) / (W_f_A(2));
    }
  }
  else if (W_f_B(2) > 1e-1)
  {
    W_Q(0) = (W_f_B(2) * W_Q_B(0)) / (W_f_B(2));
    W_Q(1) = (W_f_B(2) * W_Q_B(1)) / (W_f_B(2));
  }

  W_R_H = robot().bodyPosW("Body").rotation();

  R_0_mIMU = robot().bodySensor("Accelerometer").orientation().toRotationMatrix();
}

void SbsController::state_swiching()
{
  if (ctrl_mode == 1)
  {
    Vector3d A_p_QA;
    A_p_QA = W_R_A.transpose() * (Q_epd - W_p_AcW);

    if (fabs(A_p_QA(0)) < 0.01 && fabs(A_p_QA(1)) < 0.01) // 0.04
    {
      ctrl_mode = 2;
      timer_mode = 0.0;

      removeContact({robot().name(), "ground", "RightFoot", "AllGround"});
      efTask_right->dimWeight(Vector3d::Constant(1000.0));
      lipmTask->setContacts({mc_tasks::lipm_stabilizer::ContactState::Left});
      lipmTask->setCtrlMode(ctrl_mode);
      lipmTask->copAdmittance(copAdmittance_ss);
      lipmTask->dcmGains(dcmGainP_ss, dcmGainI_ss, dcmGainD_ss);
      W_R_B_ref = W_R_B;
    }
  }
  else if (ctrl_mode == 2)
  {
    timer_mode += timeStep;

    if ((timer_mode > 1.0) && B_f_B(2) > FORCE_THRESHOLD)
    {
      ctrl_mode = 3;
      timer_mode = 0;

      addContact({robot().name(), "ground", "RightFoot", "AllGround", 1.0, dof});
      efTask_right->dimWeight(Vector3d::Zero());
      lipmTask->setContacts({mc_tasks::lipm_stabilizer::ContactState::Left, mc_tasks::lipm_stabilizer::ContactState::Right});
      lipmTask->setCtrlMode(ctrl_mode);
      lipmTask->copAdmittance(copAdmittance_ds);
      lipmTask->dcmGains(dcmGainP_ds, dcmGainI_ds, dcmGainD_ds);
    }
  }
  else if (ctrl_mode == 3)
  {
    if (B_Q_B.norm() < 0.03)
      timer_mode += timeStep;
    else
      timer_mode = 0;

    if ((center_button_down_l || center_button_down_r) || timer_mode > 0.1)
    {
      ctrl_mode = 0;
      timer_mode = 0;
      lipmTask->setCtrlMode(ctrl_mode);
      lipmTask->setContacts({mc_tasks::lipm_stabilizer::ContactState::Left, mc_tasks::lipm_stabilizer::ContactState::Right});
      W_p_GW_ref = (W_p_AcW + W_p_BcW) / 2.0;
      W_p_GW_ref(2) += HEIGHTREF;

      if ((W_p_GW_ref - W_p_GW_d).norm() > 1e-6)
        direction = (W_p_GW_ref - W_p_GW_d).normalized();
    }
  }
  else if (ctrl_mode == 5)
  {
    Vector3d B_p_QB;
    B_p_QB = W_R_B.transpose() * (Q_epd - W_p_BcW);
    if (fabs(B_p_QB(0)) < 0.01 && fabs(B_p_QB(1)) < 0.01)
    {
      ctrl_mode = 6;
      timer_mode = 0.0;

      removeContact({robot().name(), "ground", "LeftFoot", "AllGround"});
      efTask_left->dimWeight(Vector3d::Constant(1000.0));
      lipmTask->setContacts({mc_tasks::lipm_stabilizer::ContactState::Right});
      lipmTask->setCtrlMode(ctrl_mode);
      lipmTask->copAdmittance(copAdmittance_ss);
      lipmTask->dcmGains(dcmGainP_ss, dcmGainI_ss, dcmGainD_ss);
      W_R_A_ref = W_R_A;
    }
  }
  else if (ctrl_mode == 6)
  {
    timer_mode += timeStep;
    if ((timer_mode > 1.0) && A_f_A(2) > FORCE_THRESHOLD)
    {
      ctrl_mode = 7;
      timer_mode = 0;

      addContact({robot().name(), "ground", "LeftFoot", "AllGround", 1.0, dof});
      efTask_left->dimWeight(Vector3d::Zero());
      lipmTask->setContacts({mc_tasks::lipm_stabilizer::ContactState::Left, mc_tasks::lipm_stabilizer::ContactState::Right});
      lipmTask->setCtrlMode(ctrl_mode);
      lipmTask->copAdmittance(copAdmittance_ds);
      lipmTask->dcmGains(dcmGainP_ds, dcmGainI_ds, dcmGainD_ds);
    }
  }
  else if (ctrl_mode == 7)
  {
    if (A_Q_A.norm() < 0.03)
      timer_mode += timeStep;

    if ((center_button_down_l || center_button_down_r) || timer_mode > 0.1)
    {
      ctrl_mode = 0;
      timer_mode = 0;
      lipmTask->setCtrlMode(ctrl_mode);
      lipmTask->setContacts({mc_tasks::lipm_stabilizer::ContactState::Left, mc_tasks::lipm_stabilizer::ContactState::Right});
      W_p_GW_ref = (W_p_AcW + W_p_BcW) / 2.0;
      W_p_GW_ref(2) += HEIGHTREF;

      if ((W_p_GW_ref - W_p_GW_d).norm() > 1e-6)
        direction = (W_p_GW_ref - W_p_GW_d).normalized();
    }
  }
  else if (ctrl_mode == 0 && W_v_B_ref(2) > 0.1 && W_p_B_ref(2) - W_p_BaW(2) > 0)
  {
    if (ctrl_mode2 == 1)
      anchorFrame = realRobot().surfacePose("LeftFoot");

    ctrl_mode2 = 0;

    ctrl_mode = 1;
    lipmTask->setCtrlMode(ctrl_mode);
    W_p_GW_ref = W_p_AcW;
    W_p_GW_ref(2) += HEIGHTREF;

    if ((W_p_GW_ref - W_p_GW_d).norm() > 1e-6)
      direction = (W_p_GW_ref - W_p_GW_d).normalized();
  }
  else if (ctrl_mode == 0 && W_v_A_ref(2) > 0.1 && W_p_A_ref(2) - W_p_AaW(2) > 0)
  {
    if (ctrl_mode2 == 0)
      anchorFrame = realRobot().surfacePose("RightFoot");

    ctrl_mode2 = 1;

    ctrl_mode = 5;
    lipmTask->setCtrlMode(ctrl_mode);
    W_p_GW_ref = W_p_BcW;
    W_p_GW_ref(2) += HEIGHTREF;

    if ((W_p_GW_ref - W_p_GW_d).norm() > 1e-6)
      direction = (W_p_GW_ref - W_p_GW_d).normalized();
  }
}

void SbsController::set_desiredVel()
{
  cal_motion(W_p_GW_ref, W_p_GW_d, W_v_GW_d, W_a_GW_d, direction);

  Q_epd = W_p_GW_d - W_a_GW_d / (omega * omega);
  Q_epd(2) = Q_epd(2) - HEIGHTREF;

#ifdef USE_FALCON
  W_p_A_ref = W_p_BaW + W_p_AB_ref;
  W_p_B_ref = W_p_AaW + W_p_BA_ref;
#else
  W_v_A_ref = (W_p_A_ref - W_p_A_ref_p) / timeStep;
  W_v_B_ref = (W_p_B_ref - W_p_B_ref_p) / timeStep;
  W_p_A_ref_p = W_p_A_ref;
  W_p_B_ref_p = W_p_B_ref;
#endif
}

void SbsController::set_desiredTask()
{

  lipmTask->target(W_p_GW_d, W_v_GW_d, W_a_GW_d, Q_epd);

  if (ctrl_mode == 2)
    efTask_right->set_ef_pose(sva::PTransformd(W_R_B_ref, W_p_B_ref));
  else if (ctrl_mode == 6)
    efTask_left->set_ef_pose(sva::PTransformd(W_R_A_ref, W_p_A_ref));
}

double SbsController::sat_func(double lim, double val)
{
  double res;
  if (val > lim)
    res = lim;
  else if (val < -lim)
    res = -lim;
  else
    res = val;

  return res;
}

Vector3d SbsController::sat_func(double _lim, const Vector3d &val)
{
  double lim = fabs(_lim);
  Vector3d result;

  for (int i = 0; i < 3; i++)
  {
    if (val(i) > lim)
      result(i) = lim;
    else if (val(i) < -lim)
      result(i) = -lim;
    else
      result(i) = val(i);
  }

  return result;
}

void SbsController::createGUI()
{
#ifdef USE_FALCON
  gui()->addElement({"SbsController", "Task"}, mc_rtc::gui::Point3D("Point_Left", mc_rtc::gui::PointConfig(mc_rtc::gui::Color(1, 1, 0), 0.075), [this]()
                                                                    { return W_p_A_ref; }),
                    mc_rtc::gui::Point3D("Point_Right", mc_rtc::gui::PointConfig(mc_rtc::gui::Color(1, 1, 0), 0.075), [this]()
                                         { return W_p_B_ref; }));
#else
  gui()->addElement({"SbsController", "Task"}, mc_rtc::gui::Point3D("Point_Left", [this]()
                                                                    { return W_p_A_ref; }, [this](const Vector3d &pos)
                                                                    { W_p_A_ref = pos; }),
                    mc_rtc::gui::Point3D("Point_Right", [this]()
                                         { return W_p_B_ref; }, [this](const Vector3d &pos)
                                         { W_p_B_ref = pos; }));
#endif
}

void SbsController::cal_motion(const Vector3d &target, const Vector3d &W_p_GW_0, const Vector3d &W_v_GW_0, const Vector3d &W_a_GW_0, const Vector3d &n)
{
  Vector3d alpha_state;
  double alpha_jerk;
  double alpha_traget = (target - W_p_GW_0).dot(n);

  // alpha_state << 0, W_v_GW_0.dot(n), W_a_GW_0.dot(n);
  // alpha_jerk = sat_func(limit_jerk, kp_dcm * (alpha_traget - alpha_state(0)) - (kp_dcm / omega + kd_dcm) * alpha_state(1) - (kd_dcm / omega + 1) * alpha_state(2));
  // // alpha_jerk = sat_func(limit_jerk, kp_dcm * alpha_state(0) + (kp_dcm / omega + kd_dcm) * alpha_state(1) + (kd_dcm / omega - 1) * alpha_state(2));
  // alpha_state(2) = sat_func(limit_acc, alpha_state(2) + alpha_jerk * timeStep);
  // alpha_state(1) = sat_func(limit_vel, alpha_state(1) + alpha_state(2) * timeStep);
  // alpha_state(0) = alpha_state(0) + alpha_state(1) * timeStep;

  alpha_state << 0, W_v_GW_0.dot(n), W_a_GW_0.dot(n);
  alpha_state(2) = sat_func(limit_acc, kp_dcm * alpha_traget - (kp_dcm / omega + omega) * alpha_state(1));
  alpha_state(1) = sat_func(limit_vel, alpha_state(1) + alpha_state(2) * timeStep);
  alpha_state(0) = alpha_state(0) + alpha_state(1) * timeStep;

  W_p_GW_d = W_p_GW_0 + alpha_state(0) * n;
  W_v_GW_d = alpha_state(1) * n;
  W_a_GW_d = alpha_state(2) * n;
}

void SbsController::CommandCallback_Left(const geometry_msgs::PointStamped &msg)
{
  Vector3d posRA(-(msg.point.z - 0.12), -msg.point.x, msg.point.y);

  W_p_AB_ref = W_p_AB_ref_0 + coff_falcon.cwiseProduct(posRA);

  W_v_A_ref = (W_p_AB_ref - W_p_AB_ref_p) / timeStep;
  W_p_AB_ref_p = W_p_AB_ref;
}
void SbsController::CommandCallback_Right(const geometry_msgs::PointStamped &msg)
{
  Vector3d posRB(-(msg.point.z - 0.12), -msg.point.x, msg.point.y);

  W_p_BA_ref = W_p_BA_ref_0 + coff_falcon.cwiseProduct(posRB);

  W_v_B_ref = (W_p_BA_ref - W_p_BA_ref_p) / timeStep;
  W_p_BA_ref_p = W_p_BA_ref;
}

void SbsController::ButtonChanged_Left(const ros_falcon::FourButtonDown &msg)
{
  center_button_down_l = msg.CENTER_BUTTON_DOWN;
}

void SbsController::ButtonChanged_Right(const ros_falcon::FourButtonDown &msg)
{
  center_button_down_r = msg.CENTER_BUTTON_DOWN;
}
