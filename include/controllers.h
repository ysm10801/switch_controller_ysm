#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include "base.h"
#include "qpSWIFT.h"

namespace switch_controller{

class TaskSpaceImpCtrl : protected Controller{
private:
  double k_trans;
  double k_rot;
  std::array<double, 6> tau_error_task;
  std::array<double, 6> tau_error_task_filtered;

public:
  std::mutex m;
  Eigen::Vector3d EE_pos_d;
  Eigen::Quaterniond EE_orn_d;
  Eigen::MatrixXd task_k_matrix;
  Eigen::MatrixXd task_d_matrix;
  BilinearLPF tau_lpf = BilinearLPF(1000, 50);

  TaskSpaceImpCtrl(Robot7 *robot) 
    : Controller(robot)
    , task_k_matrix(6,6)
    , task_d_matrix(6,6)
    , k_trans(220.0) // original: 150
    , k_rot(20.0) // original: 10
    , tau_error_task{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    , tau_error_task_filtered{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
  {
    SetTaskImpMatrixTrans(k_trans);
    SetTaskImpMatrixRot(k_rot);
  };
  double GetTaskTransK() {return k_trans;};
  double GetTaskRotK() {return k_rot;};
  std::array<double, 6> GetTauTask() {return tau_error_task_filtered;};
  void SetTaskImpMatrixTrans(double k_trans);
  void SetTaskImpMatrixRot(double k_rot);
  //void SetTaskImpMatrix(double k_trans, double k_rot);
  void init();
  RobotTorque7 loop(const RobotState7 &robot_state);
  void stop();
};

class TaskSpaceNRICCtrl : protected Controller{
private:
  std::array<double, 6> k1_vec;
  std::array<double, 6> k2_vec;
  std::array<double, 7> K_vec;
  std::array<double, 7> Kp_vec;
  std::array<double, 7> Ki_vec;
  std::array<double, 7> gamma_vec;
  std::array<double, 7> reflected_inertia_vec;
  std::array<double, 7> kd_null_vec;
  std::array<double, 6> tau_A_t;
  std::array<double, 7> tau_A_j;
  std::array<double, 7> e_nr;
  std::array<double, 7> tau_C;
  std::array<double, 7> tau_diff;
  int exit_code = 0;
  int qp_norm_cond = 0;
  int wpcount = 0;
  int wp_num = 35;

  qp_int n_ = 7;   // # of variable (= # of Panda joints)
  qp_int m_ = 28;  // # of ineq consts
  qp_int p_ = 0;   // # of eq consts

  struct NominalPlant
  {
    Eigen::VectorXd q;
    Eigen::VectorXd dq;
    Eigen::VectorXd ddq;
    Eigen::Affine3d ee_pose;
  };

  NominalPlant nominal_plant;
  NominalPlant C_nominal_plant;

public:
  std::mutex m;
  Eigen::Affine3d EE_pose_d;
  Eigen::MatrixXd k1;
  Eigen::MatrixXd k2;
  Eigen::MatrixXd K;
  Eigen::MatrixXd Kp;
  Eigen::MatrixXd Ki;
  Eigen::MatrixXd gamma;
  Eigen::MatrixXd reflected_inertia;
  Eigen::MatrixXd kd_null;

  QP *myQP;
  Eigen::VectorXd ddq_opt;
  Eigen::VectorXd ddq_mu;
  Eigen::VectorXd mu_st;
  Eigen::VectorXd ddq_diff; 
  
  Eigen::MatrixXd P;
  Eigen::VectorXd c;
  Eigen::MatrixXd G;
  Eigen::VectorXd h;

  Eigen::VectorXd enr_max;
  Eigen::VectorXd enr_min;

  Eigen::VectorXd ddq_max;
  Eigen::VectorXd ddq_min;

  Eigen::VectorXd tauA_max;
  Eigen::VectorXd tauA_min;

  Eigen::VectorXd FT_pred;
  Eigen::VectorXd tau_shift;

  Eigen::MatrixXd temp_L;
  Eigen::MatrixXd temp_gain;

  std::vector<Eigen::Affine3d> waypoints;

  TaskSpaceNRICCtrl(Robot7 *robot) 
    : Controller(robot)
    , k1(6,6)
    , k2(6,6)
    , K(7,7)
    , Kp(7,7)
    , Ki(7,7)
    , gamma(7,7)
    , reflected_inertia(7,7)
    , k1_vec{30.0, 30.0, 30.0, 20.0, 20.0, 20.0} // d-gian
    , k2_vec{220.0, 220.0, 220.0, 100.0, 100.0, 100.0} // p-gian
    // , K_vec{30.0, 30.0, 30.0, 30.0, 20.0, 20.0, 20.0}
    , K_vec{20.0, 20.0, 20.0, 20.0, 13.0, 13.0, 6.0} // done
    // , Kp_vec{20.0, 20.0, 20.0, 20.0, 15.0, 15.0, 10.0}
    , Kp_vec{13.0, 13.0, 13.0, 13.0, 8.0, 8.0, 4.0} // done
    , Ki_vec{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    // , gamma_vec{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}
    , gamma_vec{4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0}  //done
    // , reflected_inertia_vec{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    , reflected_inertia_vec{0.05, 0.05, 0.05, 0.05, 0.2, 0.2, 0.2}
    , kd_null_vec{2.0, 2.0, 2.0, 4.0, 1.0, 1.0, 1.0}
    , nominal_plant{
        Eigen::VectorXd::Zero(7),
        Eigen::VectorXd::Zero(7),
        Eigen::VectorXd::Zero(7),
        Eigen::Affine3d::Identity()
    }
    , C_nominal_plant{
        Eigen::VectorXd::Zero(7),
        Eigen::VectorXd::Zero(7),
        Eigen::VectorXd::Zero(7),
        Eigen::Affine3d::Identity()
    } {};
  std::array<double, 6>  Getk1() {return k1_vec;};
  std::array<double, 6>  Getk2() {return k2_vec;};
  std::array<double, 7>  GetK() {return K_vec;};
  std::array<double, 7>  GetKp() {return Kp_vec;};
  std::array<double, 7>  GetKi() {return Ki_vec;};
  std::array<double, 7>  GetGamma() {return gamma_vec;};
  std::array<double, 7>  GetReflectedInertia() {return reflected_inertia_vec;};
  Eigen::Affine3d GetNominalEEPose() {return nominal_plant.ee_pose;};
  Eigen::VectorXd GetNominalConfig() {return nominal_plant.q;};
  Eigen::VectorXd GetNominalJointVel() {return nominal_plant.dq;};
  int GetQPExitCode() {return exit_code;};
  int GetQPNormCond() {return qp_norm_cond;};
  std::array<double, 6> GetTauATask() {return tau_A_t;};
  std::array<double, 7> GetTauAJoint() {return tau_A_j;};
  std::array<double, 7> GetENR() {return e_nr;};
  std::array<double, 7> GetTauCTask() {return tau_C;};
  std::array<double, 7> GetTauDiffTask() {return tau_diff;};

  Eigen::VectorXd computeTau(const Eigen::VectorXd &nominal_q, const Eigen::VectorXd &nominal_dq, const Eigen::Affine3d &EE_pose_d);
  std::vector<Eigen::Affine3d> InterpolateDesPose(const Eigen::Affine3d &current_pose, const Eigen::Affine3d &desired_pose, int num_waypoints);
  
  
  void init();
  RobotTorque7 loop(const RobotState7 &robot_state);
  void stop();
};

}

#endif //CONTROLLERS_H