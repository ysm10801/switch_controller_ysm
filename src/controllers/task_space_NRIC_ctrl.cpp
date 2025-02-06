#include "controllers.h"

namespace switch_controller{

/*********************************
 * task space NRIC controller
 * *******************************/

Eigen::VectorXd TaskSpaceNRICCtrl::computeTau(const Eigen::VectorXd &nominal_q, const Eigen::VectorXd &nominal_dq, const Eigen::Affine3d &EE_pose_d){
  // Nominal Robot M,c,g
  Eigen::MatrixXd nominal_mass_matrix = robot->RequestNominalMassMatrix(nominal_q) + reflected_inertia;
  // Eigen::MatrixXd nominal_mass_matrix = robot->RequestNominalMassMatrix(nominal_plant.q);
  Eigen::MatrixXd nominal_coriolis = robot->RequestNominalCoriolisVector(nominal_q, nominal_dq);
  Eigen::VectorXd nominal_gravity = robot->RequestNominalGravityVector(nominal_q);
  Eigen::Affine3d nominal_EE_pose = robot->RequestNominalEEPose(nominal_q);
  nominal_plant.ee_pose = nominal_EE_pose; // for visualization

  Eigen::VectorXd tau(7); tau.setZero();

  Eigen::Vector3d nominal_EE_trans(nominal_EE_pose.translation());
  Eigen::Vector3d EE_trans_d(EE_pose_d.translation());
  Eigen::Quaterniond nominal_EE_rot(nominal_EE_pose.linear());
  Eigen::Quaterniond EE_rot_d(EE_pose_d.linear());

  //noFlip
  if (EE_rot_d.coeffs().dot(nominal_EE_rot.coeffs()) < 0.0) {
      nominal_EE_rot.coeffs() << -nominal_EE_rot.coeffs();
  }

  Eigen::Quaterniond error_rot_b(nominal_EE_rot.inverse() * EE_rot_d);
  Eigen::AngleAxisd angle_axis(error_rot_b);
  Eigen::Vector3d error_rot_b_rotvec(angle_axis.angle() * angle_axis.axis());
  Eigen::Vector3d error_rot(-nominal_EE_rot.matrix() * error_rot_b_rotvec);

  Eigen::Matrix<double, 6, 1> error_task; error_task.setZero();
  error_task.head<3>() = nominal_EE_trans - EE_trans_d;
  error_task.tail<3>() = error_rot;

  //J_task for input
  Eigen::MatrixXd nominal_J_t = robot->RequestNominalTaskJacobian(nominal_q);
  Eigen::MatrixXd pseudo_inverse_J_t = nominal_J_t.transpose() * 
    (nominal_J_t * nominal_J_t.transpose()).inverse();

  //null-space control
  Eigen::MatrixXd null_proj = Eigen::MatrixXd::Identity(7, 7) - pseudo_inverse_J_t * nominal_J_t;
  Eigen::VectorXd tau_null = - null_proj * (kd_null * nominal_dq);

  Eigen::VectorXd e_task_DN = k2 * error_task;
  // Eigen::VectorXd e_task_DN = k2 * rJf * error_task;
  Eigen::VectorXd edot_task_DN = k1 * nominal_J_t * nominal_dq;

  // Compute control torques
  // Eigen::VectorXd temp = nominal_J_t.transpose() * (-edot_task_DN - e_task_DN);
  Eigen::VectorXd temp = pseudo_inverse_J_t * (-edot_task_DN - e_task_DN);
  tau = nominal_mass_matrix * temp + nominal_coriolis + nominal_gravity + tau_null;

  return tau;
}

//init function
void TaskSpaceNRICCtrl::init(){

  //NRIC Gain Settings
  printf("NRIC Now Initialized\n");

  k1.resize(k1_vec.size(),k1_vec.size()); k1.setZero();
  k2.resize(k2_vec.size(),k2_vec.size()); k2.setZero();
  K.resize(K_vec.size(),K_vec.size()); K.setZero();
  gamma.resize(gamma_vec.size(),gamma_vec.size()); gamma.setZero();
  reflected_inertia.resize(reflected_inertia_vec.size(),reflected_inertia_vec.size()); reflected_inertia.setZero();
  Kp.resize(Kp_vec.size(),Kp_vec.size()); Kp.setZero();            
  Ki.resize(Ki_vec.size(),Ki_vec.size()); Ki.setZero();
  kd_null.resize(kd_null_vec.size(),kd_null_vec.size()); kd_null.setZero();

  k1.diagonal() = Eigen::Map<Eigen::VectorXd>(k1_vec.data(),k1_vec.size());
  k2.diagonal() = Eigen::Map<Eigen::VectorXd>(k2_vec.data(),k2_vec.size());
  K.diagonal() = Eigen::Map<Eigen::VectorXd>(K_vec.data(),K_vec.size());
  gamma.diagonal() = Eigen::Map<Eigen::VectorXd>(gamma_vec.data(),gamma_vec.size());
  reflected_inertia.diagonal() = Eigen::Map<Eigen::VectorXd>(reflected_inertia_vec.data(),reflected_inertia_vec.size());
  Kp.diagonal() = Eigen::Map<Eigen::VectorXd>(Kp_vec.data(),Kp_vec.size());            
  Ki.diagonal() = Eigen::Map<Eigen::VectorXd>(Ki_vec.data(),Ki_vec.size());
  kd_null.diagonal() = Eigen::Map<Eigen::VectorXd>(kd_null_vec.data(),kd_null_vec.size());

  //Real Panda Robot Initialization
  RobotState7 state = robot->ReadState(); //assume that robot state is read.
  Eigen::Affine3d EE_pose(Eigen::Matrix4d::Map(state.EE_pose.data()));
  // Eigen::Vector3d pos(EE_pose.translation());
  // Eigen::Quaterniond orn(EE_pose.linear());
  // EE_pos_d = pos;
  // EE_orn_d = orn;
  EE_pose_d = EE_pose;
  
  //Nominal Robot Initialization
  nominal_plant.q = Eigen::Map<const Eigen::VectorXd>(state.q.data(), state.q.size());
  nominal_plant.dq = Eigen::Map<const Eigen::VectorXd>(state.dq.data(), state.dq.size());
  nominal_plant.ddq = Eigen::VectorXd::Zero(state.dq.size());
  nominal_plant.ee_pose = EE_pose;

  C_nominal_plant.q = Eigen::Map<const Eigen::VectorXd>(state.q.data(), state.q.size());
  C_nominal_plant.dq = Eigen::Map<const Eigen::VectorXd>(state.dq.data(), state.dq.size());
  C_nominal_plant.ddq = Eigen::VectorXd::Zero(state.dq.size());
  C_nominal_plant.ee_pose = EE_pose;

  ddq_opt.resize(7,1); ddq_opt.setZero();
  ddq_mu.resize(7,1); ddq_mu.setZero();
  mu_st.resize(28,1); mu_st.setZero();
  ddq_diff.resize(7,1); ddq_diff.setZero();

  P.resize(7,7); P.setZero(); 
  c.resize(7,1); c.setZero();
  G.resize(28,7); G.setZero();
  h.resize(28,1); h.setZero();
  enr_max.resize(7,1); enr_max.setZero();
  enr_min.resize(7,1); enr_min.setZero();
  ddq_max.resize(7,1); ddq_max.setZero();
  ddq_min.resize(7,1); ddq_min.setZero();
  tauA_max.resize(7,1); tauA_max.setZero();
  tauA_min.resize(7,1); tauA_min.setZero();

  tau_shift.resize(7,1); tau_shift.setZero(); // Updated constraint from node_Callback

  // C-NRIC Initial Condtition
  enr_max << 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.002;
  // enr_max << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 ;         // almost no constraint
  enr_min = -enr_max;

  // tauA_max << 3.0, 3.0, 3.0, 2.0, 2.0, 1.0, 1.0;
  tauA_max << 1.0, 1.0, 1.0, 0.7, 0.7, 0.3, 0.3;
  // tauA_max << 0.5, 0.5, 0.5, 0.3, 0.3, 0.2, 0.2;
  // tauA_max << 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0 ;         // almost no constraint
  tauA_min = -tauA_max;

  ddq_max << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;
  // ddq_max << 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0;   // almost no constraint
  ddq_min = -ddq_max;

  temp_L.resize(7,7); temp_L.setZero();
  temp_gain.resize(7,7); temp_gain.setZero();

  temp_L = K + gamma;
  temp_gain = 1e-6 * Kp + 1e-3 * Eigen::MatrixXd::Identity(7,7);
};

//torque control loop function
RobotTorque7 TaskSpaceNRICCtrl::loop(const RobotState7 &robot_state){
  RobotState7 state = robot->ReadState();
  RobotTorque7 output;

  const int nv = robot_state.q.size();
  // const double dt = robot->getDuration();
  const double dt = 0.001;

  // Real Robot M,c,g
  RobotInertia mass_array = robot->GetMassMatrix(); 
  RobotTorque7 coriolis_array = robot->GetCoriolisVector();
  RobotTorque7 gravity_array = robot->GetGravityVector();
  Jacobian7 jacobian_array = robot->GetTaskJacobian();

  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(state.q.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(state.dq.data());
  Eigen::Affine3d EE_pose(Eigen::Matrix4d::Map(state.EE_pose.data()));
  
  Eigen::VectorXd tauC(nv); tauC.setZero();
  Eigen::VectorXd tauC_unconst(nv); tauC_unconst.setZero();

  Eigen::VectorXd tauA(nv); tauA.setZero();

  // Constrained Nominal Robot M,c,g
  Eigen::MatrixXd C_nominal_mass_matrix = robot->RequestNominalMassMatrix(C_nominal_plant.q) + reflected_inertia;
  Eigen::MatrixXd C_nominal_coriolis = robot->RequestNominalCoriolisVector(C_nominal_plant.q, C_nominal_plant.dq);
  Eigen::VectorXd C_nominal_gravity = robot->RequestNominalGravityVector(C_nominal_plant.q);
  Eigen::Affine3d C_nominal_EE_pose = robot->RequestNominalEEPose(C_nominal_plant.q);
  Eigen::MatrixXd C_nominal_J_t = robot->RequestNominalTaskJacobian(C_nominal_plant.q);
  C_nominal_plant.ee_pose = C_nominal_EE_pose; // for visualization

  // Nominal Robot M,c,g
  Eigen::MatrixXd nominal_mass_matrix = robot->RequestNominalMassMatrix(nominal_plant.q) + reflected_inertia;
  Eigen::MatrixXd nominal_coriolis = robot->RequestNominalCoriolisVector(nominal_plant.q, nominal_plant.dq);
  Eigen::VectorXd nominal_gravity = robot->RequestNominalGravityVector(nominal_plant.q);
  Eigen::Affine3d nominal_EE_pose = robot->RequestNominalEEPose(nominal_plant.q);
  Eigen::MatrixXd nominal_J_t = robot->RequestNominalTaskJacobian(nominal_plant.q);
  nominal_plant.ee_pose = nominal_EE_pose; // for visualization


  tauC = TaskSpaceNRICCtrl::computeTau(C_nominal_plant.q, C_nominal_plant.dq, EE_pose_d);

  tauC_unconst = TaskSpaceNRICCtrl::computeTau(nominal_plant.q, nominal_plant.dq, EE_pose_d);

  // QP for Constraints

  P = C_nominal_mass_matrix;
  c = - (tauC - C_nominal_coriolis - C_nominal_gravity);
  G.block(0, 0, 7, 7) = - Eigen::MatrixXd::Identity(nv, nv);
  G.block(7, 0, 7, 7) = Eigen::MatrixXd::Identity(nv, nv);
  G.block(14, 0, 7, 7) = - Eigen::MatrixXd::Identity(nv, nv);
  G.block(21, 0, 7, 7) = Eigen::MatrixXd::Identity(nv, nv);

  // h.segment(0, 7) = -1e6 * (enr_min + q + dq * dt - C_nominal_plant.q - C_nominal_plant.dq * dt);    // e_nr Constraints
  // h.segment(7, 7) = 1e6 * (enr_max + q + dq * dt - C_nominal_plant.q - C_nominal_plant.dq * dt);     // e_nr Constraints
  // h.segment(0, 7) = - temp_gain.inverse() * (temp_L.inverse()*tauA_min + dq - C_nominal_plant.dq + Kp * (q + dq * dt - C_nominal_plant.q - C_nominal_plant.dq * dt));   //tau_A Constraitns
  // h.segment(7, 7) = temp_gain.inverse() * (temp_L.inverse()*tauA_max + dq - C_nominal_plant.dq + Kp * (q + dq * dt - C_nominal_plant.q - C_nominal_plant.dq * dt));     //tau_A Constraitns
  h.segment(0, 7) = - temp_gain.inverse() * (temp_L.inverse()*(tauA_min + tau_shift) + dq - C_nominal_plant.dq + Kp * (q + dq * dt - C_nominal_plant.q - C_nominal_plant.dq * dt));   //tau_A Constraitns
  h.segment(7, 7) = temp_gain.inverse() * (temp_L.inverse()*(tauA_max + tau_shift) + dq - C_nominal_plant.dq + Kp * (q + dq * dt - C_nominal_plant.q - C_nominal_plant.dq * dt));     //tau_A Constraitns
  h.segment(14, 7) = - ddq_min; 
  h.segment(21, 7) = ddq_max;

  myQP = QP_SETUP_dense(n_, m_, p_, P.data(), NULL, G.data(), c.data(), h.data(), NULL, NULL, COLUMN_MAJOR_ORDERING);

  myQP->options->maxit = 40;
  myQP->options->reltol = 1e-3;
  myQP->options->abstol = 1e-3;

  qp_int ExitCode = QP_SOLVE(myQP);

  exit_code = int(ExitCode); //

  for(int i=0 ; i<28 ; ++i){
    mu_st(i) = myQP->z[i];
  }

  ddq_mu = C_nominal_mass_matrix.inverse()*(tauC - G.transpose() * mu_st - C_nominal_coriolis - C_nominal_gravity);

  for(int i=0 ; i<7 ; ++i){
    ddq_opt(i) = myQP->x[i];
  }

  ddq_diff = ddq_opt - ddq_mu;

  if(ddq_diff.norm() > 5e-3){
    qp_norm_cond = 1;
  }
  else{
    qp_norm_cond = 0;
  }

  // Update nominal plant
  C_nominal_plant.ddq = ddq_opt;      //CNRIC
  // C_nominal_plant.ddq = C_nominal_mass_matrix.inverse()*(tauC - C_nominal_coriolis - C_nominal_gravity);        // NRIC
  C_nominal_plant.dq += C_nominal_plant.ddq * dt;
  C_nominal_plant.q += C_nominal_plant.dq * dt;
  C_nominal_EE_pose = robot->RequestNominalEEPose(C_nominal_plant.q);
  C_nominal_plant.ee_pose = C_nominal_EE_pose; // for visualization

  nominal_plant.ddq = nominal_mass_matrix.inverse()*(tauC_unconst - nominal_coriolis - nominal_gravity);        // NRIC for Observation
  nominal_plant.dq += nominal_plant.ddq * dt;
  nominal_plant.q += nominal_plant.dq * dt;
  nominal_EE_pose = robot->RequestNominalEEPose(nominal_plant.q);
  nominal_plant.ee_pose = nominal_EE_pose; // for visualization


	// if (ExitCode == QP_OPTIMAL)
	// {
  //   if(ddq_diff.norm() > 5e-3){
  //     printf("[WARNING] NORM FROM QP IS NOT ZERO!!! NOW THE NORM IS %.8f\n", ddq_diff.norm());
  //   }
	// 	// printf("Solve Time     : %f ms\n", (myQP->stats->tsolve + myQP->stats->tsetup) * 1000.0);
	// 	// printf("KKT_Solve Time : %f ms\n", myQP->stats->kkt_time * 1000.0);
	// 	// printf("LDL Time       : %f ms\n", myQP->stats->ldl_numeric * 1000.0);
	// 	// printf("Diff	       : %f ms\n", (myQP->stats->kkt_time - myQP->stats->ldl_numeric) * 1000.0);
	// 	// printf("Iterations     : %ld\n", myQP->stats->IterationCount);
	// 	printf("Optimal Solution Found\n");
  // }
	// if (ExitCode == QP_MAXIT)
	// {
	// // 	printf("Solve Time     : %f ms\n", myQP->stats->tsolve * 1000.0);
	// // 	printf("KKT_Solve Time : %f ms\n", myQP->stats->kkt_time * 1000.0);
	// // 	printf("LDL Time       : %f ms\n", myQP->stats->ldl_numeric * 1000.0);
	// // 	printf("Diff	       : %f ms\n", (myQP->stats->kkt_time - myQP->stats->ldl_numeric) * 1000.0);
	// // 	printf("Iterations     : %ld\n", myQP->stats->IterationCount);
	// 	printf("Maximum Iterations reached\n");
	// }

  ////////////////////////////////////////////////

  // NOMINAL-REAL TASK PD FOR ESTIMATION
  
  // Eigen::Vector3d est_nominal_EE_trans(nominal_EE_pose.translation());
  Eigen::Vector3d est_nominal_EE_trans(C_nominal_EE_pose.translation());
  Eigen::Vector3d EE_trans_r(EE_pose.translation());
  // Eigen::Quaterniond est_nominal_EE_rot(nominal_EE_pose.linear());
  Eigen::Quaterniond est_nominal_EE_rot(C_nominal_EE_pose.linear());
  Eigen::Quaterniond EE_rot_r(EE_pose.linear());

  //noFlip
  if (est_nominal_EE_rot.coeffs().dot(EE_rot_r.coeffs()) < 0.0) {
      est_nominal_EE_rot.coeffs() << -est_nominal_EE_rot.coeffs();
  }

  // Eigen::Quaterniond est_error_rot_a(EE_rot_r.inverse() * est_nominal_EE_rot);
  Eigen::Quaterniond est_error_rot_a(est_nominal_EE_rot * EE_rot_r.inverse());
  Eigen::AngleAxisd est_angle_axis(est_error_rot_a);
  Eigen::Vector3d est_error_rot_a_rotvec(est_angle_axis.angle() * est_angle_axis.axis());
  // Eigen::Vector3d est_error_rot(-EE_rot_r.matrix() * est_error_rot_a_rotvec);
  Eigen::Vector3d est_error_rot(est_error_rot_a_rotvec);

  Eigen::Matrix<double, 6, 1> est_error_task; est_error_task.setZero();
  est_error_task.head<3>() = EE_trans_r - est_nominal_EE_trans;
  est_error_task.tail<3>() = est_error_rot;

  double pos_weight = std::min(1.0, pow(est_error_task.head(3).norm() * 100.0, 3));
  double rot_weight = std::min(1.0, pow(est_error_task.tail(3).norm() * 20.0, 3));

  // std::cout << "position weight: " << pos_weight << "    Rotation weight : " << rot_weight << "\n" << std::endl;

  Eigen::MatrixXd weight_mtx = Eigen::MatrixXd::Identity(6,6);
  weight_mtx.diagonal() << pos_weight, pos_weight, pos_weight, rot_weight, rot_weight, rot_weight;

  Eigen::VectorXd e_task_NR = 2.0 * weight_mtx * k2 * est_error_task;
  // Eigen::VectorXd edot_task_NR = 2.0 * weight_mtx * k1 * (jacobian * dq - nominal_J_t * nominal_plant.dq);
  // Compute control torques
  Eigen::VectorXd temp_a = - e_task_NR ;
  // Eigen::VectorXd temp_a = - e_task_NR - edot_task_NR;


//////////////////////////////////des-real/////////////////////////////////////
  // if (EE_rot_d.coeffs().dot(EE_rot_r.coeffs()) < 0.0) {
  //     EE_rot_d.coeffs() << -EE_rot_d.coeffs();
  // }

  // Eigen::Quaterniond est_error_rot_a_(EE_rot_d * EE_rot_r.inverse());
  // Eigen::AngleAxisd est_angle_axis_(est_error_rot_a_);
  // Eigen::Vector3d est_error_rot_a_rotvec_(est_angle_axis_.angle() * est_angle_axis_.axis());
  // Eigen::Vector3d est_error_rot_(est_error_rot_a_rotvec_);

  // Eigen::Matrix<double, 6, 1> est_error_task_; est_error_task_.setZero();
  // est_error_task_.head<3>() = EE_trans_r - EE_trans_d;
  // est_error_task_.tail<3>() = est_error_rot_;

  // double pos_weight_ = std::min(1.0, pow(est_error_task_.head(3).norm() * 100.0, 3));
  // double rot_weight_ = std::min(1.0, pow(est_error_task_.tail(3).norm() * 20.0, 3));

  // Eigen::MatrixXd weight_mtx_ = Eigen::MatrixXd::Identity(6,6);
  // weight_mtx_.diagonal() << pos_weight_, pos_weight_, pos_weight_, rot_weight_, rot_weight_, rot_weight_;

  // Eigen::VectorXd e_task_DR = 2.0 * weight_mtx_ * k2 * est_error_task_;

  // // Compute control torques
  // Eigen::VectorXd temp_a_ = - e_task_DR ;
//////////////////////////////////des-real/////////////////////////////////////


  /////////////////////////////////////////////////

  // NOMINAL-REAL JOINT PD FOR REAL INPUT

  // Difference between robot and nominal states
  Eigen::VectorXd e_RN = q - C_nominal_plant.q;
  Eigen::VectorXd edot_RN = dq - C_nominal_plant.dq;

  // Sliding variable for NRIC
  Eigen::VectorXd s_vector = (edot_RN + Kp * e_RN);

  // Augmented torque
  tauA = -(K + gamma) * s_vector;

  // nominal_J_t = robot->RequestNominalTaskJacobian(nominal_plant.q);
  // Eigen::MatrixXd pseudo_inverse_J_t_trans = (nominal_J_t * nominal_J_t.transpose()).inverse() * nominal_J_t;

  //gravity compensated by real panda
  Eigen::VectorXd tauC_panda(tauC - C_nominal_gravity);

  for (size_t i = 0; i < output.size(); ++i) {
    output[i] = tauA[i] + tauC_panda[i];
    // output[i] = tauC_panda[i];
  }

  // Eigen::VectorXd tauA_t = pseudo_inverse_J_t_trans * tauA;
  Eigen::VectorXd tauC_t = tauA;
  Eigen::VectorXd tauA_t = temp_a;
  /// TODO: TAU_A JOINT : ONLT E_RN  OR  TAU_A(E_RN + E_DOT_RN)
  // Eigen::VectorXd tauA_j = s_vector;
  Eigen::VectorXd tauA_j = e_RN;

  for(int i ; i < 6 ; i++){
    tau_A_t[i] = tauA_t(i);
    tau_C_t[i] = tauC_t(i);
  }
  
  for(int i ; i < 7 ; i++){
    tau_A_j[i] = tauA_j(i);
  }
  
  tau_A_t_lpf.process(tau_A_t, tau_A_t_filtered);
  tau_A_j_lpf.process(tau_A_j, tau_A_j_filtered);
  tau_C_t_lpf.process(tau_C_t, tau_C_t_filtered);

  // output = {0};

  return output;
};

//stop
void TaskSpaceNRICCtrl::stop(){
  RobotState7 state = robot->ReadState(); //assume that robot state is read.
  Eigen::Affine3d EE_pose(Eigen::Matrix4d::Map(state.EE_pose.data()));
  EE_pose_d = EE_pose;
}

}