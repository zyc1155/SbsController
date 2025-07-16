#include "SbsController/SbsController.h"
#include <mc_rtc/ros.h>

SbsController::SbsController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
    : mc_control::MCController(rm, dt)
{
  config_.load(config);
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

  solver().setContacts({{}});

  Vector6d dof = Vector6d::Ones();
  dof(0) = 0.0;
  dof(1) = 0.0;
  dof(5) = 0.0;

  //  Vector6d dof =  Vector6d::Zero();
  // dof(2) = 1.0;

  addContact({robot().name(), "ground", "LeftFoot", "AllGround", 0.7, dof});
  addContact({robot().name(), "ground", "RightFoot", "AllGround", 0.7, dof});

  postureTask->stiffness(10.0);
  postureTask->weight(1000.0);

  postureTask->selectUnactiveJoints(solver(), UnactiveJoints);

  efTask_left = std::make_shared<mc_tasks::EndEffectorTask_NoGUI>("Lleg_Link5", robots(), 0);
  efTask_right = std::make_shared<mc_tasks::EndEffectorTask_NoGUI>("Rleg_Link5", robots(), 0);

  efTask_left->selectActiveJoints(solver(), activeJoints);
  efTask_right->selectActiveJoints(solver(), activeJoints);

  otTask = std::make_shared<mc_tasks::OrientationTask>("Head_Link1_Plan2", robots(), 0);
  otTask->selectActiveJoints(solver(), headJoints);

  solver().addTask(otTask);

  copAdmittance_ss = 0.005;
  copAdmittance_ds = 0.01;

  auto stabiConf = robot().module().defaultLIPMStabilizerConfiguration();
  stabiConf.comHeight = HEIGHTREF;
  stabiConf.torsoPitch = 0;
  stabiConf.copAdmittance = Vector2d::Constant(copAdmittance_ds);
  stabiConf.zmpcc.comAdmittance = Vector2d{0.0, 0.0};
  stabiConf.dcmPropGain = 2.0; // 2.0;
  stabiConf.dcmIntegralGain = 15;
  stabiConf.dcmDerivGain = 0.5;
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

  lipmTask->configure(stabiConf);
  lipmTask->setContacts({mc_tasks::lipm_stabilizer::ContactState::Left, mc_tasks::lipm_stabilizer::ContactState::Right});

  solver().addTask(lipmTask);

  ttime = 0;

  ctrl_mode = 0;
  ctrl_mode2 = 0;

  posRA = Vector3d::Zero();
  posRB = Vector3d::Zero();
  W_R_A_ref = Matrix3d::Zero();
  W_R_B_ref = Matrix3d::Zero();
  vel_posRA = Vector3d::Zero();
  vel_posRB = Vector3d::Zero();

  sva::ForceVecd force_limit, error_limit, admittance;
  force_limit = sva::ForceVecd(Vector3d::Constant(1), Vector3d::Constant(10));
  error_limit = sva::ForceVecd(Vector3d::Constant(10.0), Vector3d::Constant(0.01));
  admittance = sva::ForceVecd(Vector3d::Constant(1.0), Vector3d::Constant(0.5));

  // W_pos_A =

  W_v_GW_ref = Vector3d::Zero();
  W_v_GWd = Vector3d::Zero();
  Q_epd = Vector3d::Zero();
  omega = sqrt(GRAVITY / HEIGHTREF);
  COMShifter_Kp = Matrix3d::Zero();
  COMShifter_Kd = Matrix3d::Zero();

  for (int i = 0; i < 3; i++)
  {
    COMShifter_Kp(i, i) = 30.0;

    COMShifter_Kd(i, i) = COMShifter_Kp(i, i) / omega + omega;
  }

  // for new trajectory
  direction << 0, 0, 1;
  limit_vel = 0.4;
  limit_acc = 0.45;
  limit_jerk = 8;

  kp_dcm = 30;
  kd_dcm = 2 * sqrt(kp_dcm);
  /////

  first = true;

  leftFootRatio = 0.5;
  datastore().make_call("KinematicAnchorFrame::" + robot().name(),
                        [this](const mc_rbdyn::Robot &robot)
                        {
                          return sva::interpolate(robot.surfacePose("RightFoot"),
                                                  robot.surfacePose("LeftFoot"),
                                                  leftFootRatio);
                        });

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

  createGUI();

  logger().addLogEntries(
      this,
      "leftFootRatio", [this]()
      { return leftFootRatio; },
      "left_foot_center", [this]()
      { return W_p_AW; },
      "right_foot_center", [this]()
      { return W_p_BW; },
      "left_foot_a", [this]()
      { return W_p_AW_; },
      "right_foot_a", [this]()
      { return W_p_BW_; },
      "ref_COM", [this]()
      { return W_p_GW_ref; },
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
      "EZMP", [this]()
      { return Q_epd; },
      "real_EZMP", [this]()
      { return Q_ep; });

  mc_rtc::log::success("SbsController init done ");
}

bool SbsController::run()
{

  spinner.callAvailable(ros::WallDuration());

  if (first)
  {
    z_start = std::chrono::high_resolution_clock::now();
  }
  ttime = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - z_start).count();
  get_values();
  // set_CtrlPos();
  state_swiching();
  set_desiredVel();
  set_desiredTask();
  // output_data();

  if (first)
    first = false;

  return mc_control::MCController::run();
}

void SbsController::reset(const mc_control::ControllerResetData &reset_data)
{
  mc_control::MCController::reset(reset_data);

  lipmTask->reset();
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

  W_R_A = realRobot().surfacePose("LeftFootCenter").rotation();
  W_p_AW = realRobot().surfacePose("LeftFootCenter").translation();

  W_p_AW_ = realRobot().frame("Lleg_Link5").position().translation();

  W_R_B = realRobot().surfacePose("RightFootCenter").rotation();
  W_p_BW = realRobot().surfacePose("RightFootCenter").translation();

  W_p_BW_ = realRobot().frame("Rleg_Link5").position().translation();

  W_p_GW = realRobot().com();
  // W_v_GW = robot().comVelocity();
  // W_a_GW = robot().comAcceleration();
  // if (first)
  // {
  //   W_pos_A = W_p_AW_;
  //   W_pos_B = W_p_BW_;
  // }

  if (first)
    W_p_GW_p = W_p_GW;

  W_v_GW = (W_p_GW - W_p_GW_p) / timeStep;

  if (first)
    W_v_GW_p = W_v_GW;

  W_a_GW = (W_v_GW - W_v_GW_p) / timeStep;

  W_p_GW_p = W_p_GW;
  W_v_GW_p = W_v_GW;

  // W_v_GW_ = realRobot().comVelocity();

  // W_p_GW_= realRobot().com();
  // W_p_GW_(2) = 0.0;

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

  Q_ep = W_p_GW - W_a_GW / (omega * omega);
  Q_ep(2) = Q_ep(2) - HEIGHTREF;
}

void SbsController::set_CtrlPos()
{

  // posRB_ = right_falcon.Get_Pos();
  // posRA_ = left_falcon.Get_Pos();

  posRB << -(posRB_(2) - 0.12), -posRB_(0), posRB_(1);
  posRA << -(posRA_(2) - 0.12), -posRA_(0), posRA_(1);

  if (first)
  {
    posRAp = posRA;
    posRBp = posRB;
  }

  vel_posRA = (posRA - posRAp) / timeStep;
  vel_posRB = (posRB - posRBp) / timeStep;

  posRAp = posRA;
  posRBp = posRB;
}

void SbsController::state_swiching()
{
  Vector6d dof = Vector6d::Ones();
  dof(0) = 0.0;
  dof(1) = 0.0;
  dof(5) = 0.0;
  if (ctrl_mode == 1)
  {
    Vector3d A_p_QA;
    A_p_QA = W_R_A.transpose() * (Q_epd - W_p_AW);

    if (fabs(A_p_QA(0)) < 0.01 && fabs(A_p_QA(1)) < 0.01) // 0.04
    {
      ctrl_mode = 2;
      timer_mode = 0.0;

      removeContact({robot().name(), "ground", "RightFoot", "AllGround"});
      solver().addTask(efTask_right);
      lipmTask->setContacts({mc_tasks::lipm_stabilizer::ContactState::Left});
      lipmTask->setCtrlMode(ctrl_mode);
      lipmTask->copAdmittance(Vector2d::Constant(copAdmittance_ss));
      W_R_B_ref = W_R_B;
    }
  }
  else if (ctrl_mode == 2)
  {
    timer_mode += timeStep;

    if ((timer_mode > 1.0) && B_f_B(2) > 10.0)
    {
      ctrl_mode = 3;
      timer_mode = 0;

      addContact({robot().name(), "ground", "RightFoot", "AllGround", 1.0, dof});
      solver().removeTask(efTask_right);
      lipmTask->setContacts({mc_tasks::lipm_stabilizer::ContactState::Left, mc_tasks::lipm_stabilizer::ContactState::Right});
      lipmTask->setCtrlMode(ctrl_mode);
      lipmTask->copAdmittance(Vector2d::Constant(copAdmittance_ds));
    }
  }
  else if (ctrl_mode == 3)
  {
    if (B_Q_B.norm() < 0.03)
      timer_mode += timeStep;
    else
      timer_mode = 0;

    if (timer_mode > 0.1)
    {
      ctrl_mode = 0;
      timer_mode = 0;
      lipmTask->setCtrlMode(ctrl_mode);
      lipmTask->setContacts({mc_tasks::lipm_stabilizer::ContactState::Left, mc_tasks::lipm_stabilizer::ContactState::Right});
      Q_ref = (W_p_AW + W_p_BW) / 2.0;
      Q_ref(2) += HEIGHTREF;

      if ((Q_ref - W_p_GW_ref).norm() > 1e-6)
        direction = (Q_ref - W_p_GW_ref).normalized();
    }
  }
  else if (ctrl_mode == 5)
  {
    Vector3d B_p_QB;
    B_p_QB = W_R_B.transpose() * (Q_epd - W_p_BW);
    if (fabs(B_p_QB(0)) < 0.01 && fabs(B_p_QB(1)) < 0.01)
    {
      ctrl_mode = 6;
      timer_mode = 0.0;

      removeContact({robot().name(), "ground", "LeftFoot", "AllGround"});
      solver().addTask(efTask_left);
      lipmTask->setContacts({mc_tasks::lipm_stabilizer::ContactState::Right});
      lipmTask->setCtrlMode(ctrl_mode);
      lipmTask->copAdmittance(Vector2d::Constant(copAdmittance_ss));
      W_R_A_ref = W_R_A;
    }
  }
  else if (ctrl_mode == 6)
  {
    timer_mode += timeStep;
    if ((timer_mode > 1.0) && A_f_A(2) > 10.0)
    {
      ctrl_mode = 7;
      timer_mode = 0;

      addContact({robot().name(), "ground", "LeftFoot", "AllGround", 1.0, dof});
      solver().removeTask(efTask_left);
      lipmTask->setContacts({mc_tasks::lipm_stabilizer::ContactState::Left, mc_tasks::lipm_stabilizer::ContactState::Right});
      lipmTask->setCtrlMode(ctrl_mode);
      lipmTask->copAdmittance(Vector2d::Constant(copAdmittance_ds));
    }
  }
  else if (ctrl_mode == 7)
  {
    if (A_Q_A.norm() < 0.03)
      timer_mode += timeStep;

    if (timer_mode > 0.1)
    {
      ctrl_mode = 0;
      timer_mode = 0;
      lipmTask->setCtrlMode(ctrl_mode);
      lipmTask->setContacts({mc_tasks::lipm_stabilizer::ContactState::Left, mc_tasks::lipm_stabilizer::ContactState::Right});
      Q_ref = (W_p_AW + W_p_BW) / 2.0;
      Q_ref(2) += HEIGHTREF;

      if ((Q_ref - W_p_GW_ref).norm() > 1e-6)
        direction = (Q_ref - W_p_GW_ref).normalized();
    }
  }
  else if (ctrl_mode == 0 && vel_posRB(2) > 0.05 && W_pos_B(2) - W_p_BW_(2) > 0)
  {
    ctrl_mode2 = 0;

    ctrl_mode = 1;
    lipmTask->setCtrlMode(ctrl_mode);
    Q_ref = W_p_AW;
    Q_ref(2) += HEIGHTREF;

    if ((Q_ref - W_p_GW_ref).norm() > 1e-6)
      direction = (Q_ref - W_p_GW_ref).normalized();
  }
  else if (ctrl_mode == 0 && vel_posRA(2) > 0.05 && W_pos_A(2) - W_p_AW_(2) > 0)
  {
    ctrl_mode2 = 1;

    ctrl_mode = 5;
    lipmTask->setCtrlMode(ctrl_mode);
    Q_ref = W_p_BW;
    Q_ref(2) += HEIGHTREF;

    if ((Q_ref - W_p_GW_ref).norm() > 1e-6)
      direction = (Q_ref - W_p_GW_ref).normalized();
  }
}

void SbsController::set_desiredVel()
{
  Vector3d jerk;

  if (first)
  {
    Q_ref = (W_p_AW + W_p_BW) / 2.0;
    Q_ref(2) += HEIGHTREF;
    W_p_GW_ref = W_p_GW;
    if ((Q_ref - W_p_GW_ref).norm() > 1e-6)
      direction = (Q_ref - W_p_GW_ref).normalized();
  }

  cal_motion(Q_ref, W_p_GW_ref, W_v_GW_ref, W_a_GW_ref, direction);

  Q_epd = W_p_GW_ref - W_a_GW_ref / (omega * omega);
  Q_epd(2) = Q_epd(2) - HEIGHTREF;

  A_p_BA_ref(0) = posRB(0) * 10.0;
  A_p_BA_ref(1) = -0.21 + posRB(1) * 5.0;
  A_p_BA_ref(2) = posRB(2) * 5.0;

  B_p_AB_ref(0) = posRA(0) * 10.0;
  B_p_AB_ref(1) = 0.21 + posRA(1) * 5.0;
  B_p_AB_ref(2) = posRA(2) * 5.0;

  if ((ctrl_mode == 0 || ctrl_mode == 1 || ctrl_mode == 5))
  {
    W_pos_A = W_p_BW_ + B_p_AB_ref;
    W_pos_B = W_p_AW_ + A_p_BA_ref;
  }

  if (ctrl_mode2 == 0)
  {
    // W_pos_A = W_p_BW_ + B_p_AB_ref;
    W_pos_B = W_p_AW_ + A_p_BA_ref;
  }
  else if (ctrl_mode2 == 1)
  {
    W_pos_A = W_p_BW_ + B_p_AB_ref;
    // W_pos_B = W_p_BW_ + Vector3d(posRB(0) * 10.0, 0.21 + posRB(1) * 5.0, posRB(2) * 5.0);
  }
}

void SbsController::set_desiredTask()
{

  // tra_gen.Cal_Func(ttime);

  // W_a_GWd(1) = tra_gen.s[2];
  // W_v_GWd(1) = tra_gen.s[1];
  // W_p_GW_ref(1) = tra_gen.s[0];

  leftFootRatio = lipmTask->leftFootRatio();
  lipmTask->target(W_p_GW_ref, W_v_GW_ref, W_a_GW_ref, Q_epd);

  if ((ctrl_mode == 0 || ctrl_mode == 1 || ctrl_mode == 5))
  {
    // otTask->orientation( Matrix3d::Identity());
  }
  else if (ctrl_mode2 == 0)
  {
    efTask_right->set_ef_pose(sva::PTransformd(W_R_B_ref, W_pos_B));

    // otTask->orientation(W_R_H);
  }
  else if (ctrl_mode2 == 1)
  {
    efTask_left->set_ef_pose(sva::PTransformd(W_R_A_ref, W_pos_A));
    // otTask->orientation(W_R_H);
  }
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

sva::ForceVecd SbsController::error_func(const sva::ForceVecd &f_m)
{
  Vector3d measure_, limit_f, limit_e, gain_;
  Vector3d res[2];
  for (int j = 0; j < 2; j++)
  {
    if (j == 0)
    {
      measure_ = f_m.couple();
      limit_f = force_limit.couple();
      limit_e = error_limit.couple();
      gain_ = admittance.couple();
    }
    else
    {
      measure_ = f_m.force();
      limit_f = force_limit.force();
      limit_e = error_limit.force();
      gain_ = admittance.force();
    }

    for (int i = 0; i < 3; i++)
    {
      if (measure_(i) > limit_f(i))
      {
        res[j](i) = gain_(i) * (measure_(i) - limit_f(i));
        if (res[j](i) > limit_e(i))
          res[j](i) = limit_e(i);
      }
      else if (measure_(i) < -limit_f(i))
      {
        res[j](i) = gain_(i) * (measure_(i) + limit_f(i));
        if (res[j](i) < -limit_e(i))
          res[j](i) = -limit_e(i);
      }
    }
  }

  return sva::ForceVecd(res[0], res[1]);
}

void SbsController::createGUI()
{
  gui()->addElement({"SbsController", "Task"}, mc_rtc::gui::Label("Lift right foot", [this]()
                                                                  { return rightFootLift_; }),
                    mc_rtc::gui::Checkbox(
                        "Activated", [this]()
                        { return rightFootLift_; },
                        [this]()
                        { rightFootLift_ = !rightFootLift_; }));
  gui()->addElement({"SbsController", "Task"}, mc_rtc::gui::Point3D("Point_Left", mc_rtc::gui::PointConfig(mc_rtc::gui::Color(1, 1, 0), 0.075), [this]()
                                                                    { return W_pos_A; }),
                    mc_rtc::gui::Point3D("Point_Right", mc_rtc::gui::PointConfig(mc_rtc::gui::Color(1, 1, 0), 0.075), [this]()
                                         { return W_pos_B; }));
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

  W_p_GW_ref = W_p_GW_0 + alpha_state(0) * n;
  W_v_GW_ref = alpha_state(1) * n;
  W_a_GW_ref = alpha_state(2) * n;
}

void SbsController::CommandCallback_Left(const geometry_msgs::PointStamped &msg)
{
  posRA << -(msg.point.z - 0.12), -msg.point.x, msg.point.y;
  if (first)
    posRAp = posRA;

  vel_posRA = (posRA - posRAp) / timeStep;

  posRAp = posRA;
}
void SbsController::CommandCallback_Right(const geometry_msgs::PointStamped &msg)
{
  posRB << -(msg.point.z - 0.12), -msg.point.x, msg.point.y;
  if (first)
    posRBp = posRB;

  vel_posRB = (posRB - posRBp) / timeStep;

  posRBp = posRB;
}
