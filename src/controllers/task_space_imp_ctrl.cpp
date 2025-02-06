#include "controllers.h"

namespace switch_controller{

/*********************************
 * task space impedance control
 * *******************************/
void TaskSpaceImpCtrl::SetTaskImpMatrixTrans(double k_trans_cmd){
  // Spring damper system with damping ratio=1
  k_trans = k_trans_cmd;
  task_k_matrix.topLeftCorner(3, 3) << k_trans * Eigen::MatrixXd::Identity(3, 3);
  task_d_matrix.topLeftCorner(3, 3) << 2.0 * sqrt(k_trans) *
                                      Eigen::MatrixXd::Identity(3, 3);
}
void TaskSpaceImpCtrl::SetTaskImpMatrixRot(double k_rot_cmd){
  // Spring damper system with damping ratio=1
  k_rot = k_rot_cmd;
  task_k_matrix.bottomRightCorner(3, 3) << k_rot * Eigen::MatrixXd::Identity(3, 3);
  // task_k_matrix.bottomRightCorner(1, 1) << 1.0;
  task_d_matrix.bottomRightCorner(3, 3) << 2.0 * sqrt(k_rot) *
                                          Eigen::MatrixXd::Identity(3, 3);
}

//init function
void TaskSpaceImpCtrl::init(){
    RobotState7 state = robot->ReadState(); //assume that robot state is read.
    Eigen::Affine3d EE_pose(Eigen::Matrix4d::Map(state.EE_pose.data()));
    Eigen::Vector3d pos(EE_pose.translation());
    Eigen::Quaterniond orn(EE_pose.linear());
    EE_pos_d = pos;
    EE_orn_d = orn;
};

//torque control loop function
RobotTorque7 TaskSpaceImpCtrl::loop(const RobotState7 &robot_state){
  RobotState7 state = robot->ReadState();
  RobotTorque7 coriolis_array = robot->GetCoriolisVector();
  Jacobian7 jacobian_array = robot->GetTaskJacobian();
  RobotTorque7 output;

  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(state.q.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(state.dq.data());
  Eigen::Affine3d EE_pose(Eigen::Matrix4d::Map(state.EE_pose.data()));
  Eigen::Matrix<double, 6, 1> error;
  Eigen::Vector3d EE_pos(EE_pose.translation());
  Eigen::Quaterniond EE_orn(EE_pose.linear());

  //error
  error.head(3) << EE_pos - EE_pos_d;
  if (EE_orn_d.coeffs().dot(EE_orn.coeffs()) < 0.0) {
      EE_orn.coeffs() << -EE_orn.coeffs();
  }
  Eigen::Quaterniond error_qtn(EE_orn.inverse() * EE_orn_d);
  // error.tail(3) << error_qtn.x(), error_qtn.y(), error_qtn.z();
  // error.tail(3) << -EE_pose.linear() * error.tail(3);


  Eigen::AngleAxisd angle_axis(error_qtn);
  Eigen::Vector3d error_rot_vec(angle_axis.angle() * angle_axis.axis());
  Eigen::Vector3d error_rot(-EE_pose.linear() * error_rot_vec);
  error.tail(3) << error_rot;

  //Compute control
  Eigen::VectorXd tau_joint(7), tau_task(6), tau_d(7);
  tau_task << -task_k_matrix * error - task_d_matrix * (jacobian * dq);
  tau_joint << jacobian.transpose() * tau_task;
  tau_d << tau_joint + coriolis;

  for(int i=0 ; i < 6 ; i++){
    tau_error_task[i] = tau_task(i);
  }
  
  tau_lpf.process(tau_error_task, tau_error_task_filtered);

  Eigen::VectorXd::Map(&output[0], 7) = tau_d;
  return output;
};

//stop
void TaskSpaceImpCtrl::stop(){
  RobotState7 state = robot->ReadState(); //assume that robot state is read.
  Eigen::Affine3d EE_pose(Eigen::Matrix4d::Map(state.EE_pose.data()));
  EE_pos_d = EE_pose.translation();
  EE_orn_d = EE_pose.linear();
}

}