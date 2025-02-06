#include "switch_controller.h"
#include "robots.h"
#include "key.h"

//ROS messages
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_ros/transform_broadcaster.h>

//dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include "switch_controller/SwitchControllerConfig.h" //The file is in "devel/include"

namespace switch_controller{

class SwitchControllerNode : public SwitchController{
private:
  ros::NodeHandle n;
  ros::Publisher joint_state_pub;
  ros::Publisher ee_pose_pub;
  ros::Publisher nominal_ee_pose_pub;
  ros::Publisher enr_pub;
  ros::Publisher tau_task_pub;
  ros::Publisher tau_A_pub;
  ros::Publisher tau_A_joint_pub;
  ros::Publisher tau_C_pub;
  ros::Publisher tau_diff_pub;
  ros::Publisher QP_checker_pub;

  ros::Subscriber ee_pose_d_sub;
  ros::Subscriber pred_pose_tauA_sub;

  struct PoseData {
    Eigen::Affine3d transform;

    PoseData() : transform(Eigen::Affine3d::Identity()) {}
  };

  struct TauData {
    Eigen::VectorXd tau;

    TauData() : tau(7) {tau.setZero();}
  };
  
  struct VirtualPlant {
    Eigen::VectorXd q;
    Eigen::VectorXd dq;
    Eigen::VectorXd ddq;
    Eigen::Affine3d ee_pose;

    VirtualPlant() : q(7), dq(7), ddq(7), ee_pose(Eigen::Affine3d::Identity()) {
      q.setZero();
      dq.setZero();
      ddq.setZero();
    }
  };

  // std::vector<PoseData> pred_reference;
  // std::vector<PoseData> pred_action;

  VirtualPlant virtual_plant_ref;
  VirtualPlant virtual_plant_act;

  dynamic_reconfigure::Server<switch_controller::SwitchControllerConfig> server;
  dynamic_reconfigure::Server<switch_controller::SwitchControllerConfig>::CallbackType f;
  tf2_ros::TransformBroadcaster br;

public:
  SwitchControllerNode(Robot7 *robot_ptr);
  // Communication
  void PublishJointState();
  void PublishEEPose();
  void PublishNominalEEPose();
  void PublishENR();
  void PublishQPChecker();
  void PublishTauTask();
  void PublishTauATask();
  void PublishTauAJoint();
  void PublishTauCTask();
  void PublishTauDiffTask();
  void Callback_ee_pose_d(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void Callback_tau_const(const std_msgs::Float64MultiArray::ConstPtr& msg);
  // API
  void SetControllerROSWrapper(int ctrl_mode);
  void DynConfigCallback(SwitchControllerConfig &config, uint32_t level);  
};

SwitchControllerNode::SwitchControllerNode(Robot7 *robot_ptr)
  :SwitchController(robot_ptr), virtual_plant_ref(), virtual_plant_act(){
  //advertise topics
  joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_state", 1);
  ee_pose_pub = n.advertise<geometry_msgs::PoseStamped>("ee_pose", 1);
  nominal_ee_pose_pub = n.advertise<geometry_msgs::PoseStamped>("nominal_pose", 1);
  enr_pub = n.advertise<std_msgs::Float64MultiArray>("e_nr", 1);
  QP_checker_pub = n.advertise<std_msgs::Float64MultiArray>("qp_result", 1);
  tau_task_pub = n.advertise<std_msgs::Float64MultiArray>("tau_d", 1);
  tau_A_pub = n.advertise<std_msgs::Float64MultiArray>("tau_A", 1);
  tau_A_joint_pub = n.advertise<std_msgs::Float64MultiArray>("tau_A_joint", 1);
  tau_C_pub = n.advertise<std_msgs::Float64MultiArray>("tau_C", 1);
  tau_diff_pub = n.advertise<std_msgs::Float64MultiArray>("tau_diff", 1);
  ee_pose_d_sub = n.subscribe("ee_pose_d", 1, &SwitchControllerNode::Callback_ee_pose_d, this);
  pred_pose_tauA_sub = n.subscribe("pred_pose_tauA", 1, &SwitchControllerNode::Callback_tau_const, this);
  server.setCallback(boost::bind( &SwitchControllerNode::DynConfigCallback, this, _1, _2));

  // SetControllerROSWrapper(CTRLMODE_TASK_NRIC); //Init
  SetControllerROSWrapper(CTRLMODE_TASK_IMP); //Init
}

void SwitchControllerNode::DynConfigCallback(SwitchControllerConfig &config, uint32_t level){
  //ROS_INFO("Joint Impedance: k-%f, d-%f", config.joint_k_array, config.joint_d);
  //ROS_INFO("Task Impedance: trans_k-%f, rot_k-%f", config.task_trans_k, config.task_rot_k);
  if (config.ctrl_mode != GetCtrlMode()){
    SetControllerROSWrapper(config.ctrl_mode);
  }
  if (config.task_trans_k != ctrl_ts_imp.GetTaskTransK()){
    ROS_INFO("Task Stiffness (trans): %f", ctrl_ts_imp.GetTaskTransK());
    ctrl_ts_imp.SetTaskImpMatrixTrans(ctrl_ts_imp.GetTaskTransK());
  }
  if (config.task_rot_k != ctrl_ts_imp.GetTaskRotK()){
    ROS_INFO("Task Stiffness (rot): %f", ctrl_ts_imp.GetTaskRotK());
    ctrl_ts_imp.SetTaskImpMatrixRot(ctrl_ts_imp.GetTaskRotK());
  }
}

void SwitchControllerNode::Callback_ee_pose_d(const geometry_msgs::PoseStamped::ConstPtr &msg){
  ctrl_ts_imp.EE_pos_d << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  ctrl_ts_imp.EE_orn_d.x() = msg->pose.orientation.x;
  ctrl_ts_imp.EE_orn_d.y() = msg->pose.orientation.y;
  ctrl_ts_imp.EE_orn_d.z() = msg->pose.orientation.z;
  ctrl_ts_imp.EE_orn_d.w() = msg->pose.orientation.w;

  ctrl_ts_nric.EE_pose_d.translation() << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond ee_rot_d(
      msg->pose.orientation.w, 
      msg->pose.orientation.x, 
      msg->pose.orientation.y, 
      msg->pose.orientation.z);

  ctrl_ts_nric.EE_pose_d.linear() = ee_rot_d.toRotationMatrix();
}

void SwitchControllerNode::Callback_tau_const(const std_msgs::Float64MultiArray::ConstPtr &msg){

  RobotState7 state = robot->ReadState();
  const double dt = 0.001;
  std::array<double, 7> reflected_inertia_v = ctrl_ts_nric.GetReflectedInertia();
  int horizon = msg->data.size()/14 - 1;

  std::vector<PoseData> pred_reference(horizon);
  std::vector<PoseData> pred_action(horizon);
  std::vector<TauData> obs_const_nom(2);
  
  for(int t=0 ; t<2 ; ++t){
    for(int k=0 ; k<7 ; ++k){
      obs_const_nom[t].tau(k) = msg->data[7*t + k];
    }
  }

  for(int i=0 ; i<2 * horizon ; i++){
    int idx = i*7 + 14;
    Eigen::Affine3d &target_transform = (i < horizon) ? pred_action[i].transform : pred_reference[i - horizon].transform;
    
    target_transform.translation() << msg->data[idx], msg->data[idx + 1], msg->data[idx + 2];
    
    Eigen::Quaterniond ee_rot_d(
      msg->data[idx + 3],
      msg->data[idx + 4],
      msg->data[idx + 5],
      msg->data[idx + 6]);
    target_transform.linear() = ee_rot_d.toRotationMatrix();
  }

  // Virtual Robot Init
  virtual_plant_ref.q = Eigen::Map<Eigen::VectorXd>(state.q.data(), 7);
  virtual_plant_ref.dq = Eigen::Map<Eigen::VectorXd>(state.dq.data(), 7);

  virtual_plant_act.q = ctrl_ts_nric.GetNominalConfig();
  virtual_plant_act.dq = ctrl_ts_nric.GetNominalJointVel();

  Eigen::MatrixXd reflected_inertia(7,7); reflected_inertia.setZero();
  reflected_inertia.diagonal() = Eigen::Map<Eigen::VectorXd>(reflected_inertia_v.data(), 7);
  Eigen::VectorXd tau_ref(7); tau_ref.setZero();
  Eigen::VectorXd tau_act(7); tau_act.setZero();

  // Virtual Robot configs
  Eigen::MatrixXd virtual_plant_ref_mass_matrix = robot->RequestNominalMassMatrix(virtual_plant_ref.q) + reflected_inertia;
  Eigen::MatrixXd virtual_plant_ref_coriolis = robot->RequestNominalCoriolisVector(virtual_plant_ref.q, virtual_plant_ref.dq);
  Eigen::VectorXd virtual_plant_ref_gravity = robot->RequestNominalGravityVector(virtual_plant_ref.q);
  Eigen::Affine3d virtual_plant_ref_EE_pose = robot->RequestNominalEEPose(virtual_plant_ref.q);
  virtual_plant_ref.ee_pose = virtual_plant_ref_EE_pose;

  Eigen::MatrixXd virtual_plant_act_mass_matrix = robot->RequestNominalMassMatrix(virtual_plant_act.q) + reflected_inertia;
  Eigen::MatrixXd virtual_plant_act_coriolis = robot->RequestNominalCoriolisVector(virtual_plant_act.q, virtual_plant_act.dq);
  Eigen::VectorXd virtual_plant_act_gravity = robot->RequestNominalGravityVector(virtual_plant_act.q);
  Eigen::Affine3d virtual_plant_act_EE_pose = robot->RequestNominalEEPose(virtual_plant_act.q);
  virtual_plant_act.ee_pose = virtual_plant_act_EE_pose;

  Eigen::VectorXd ref_act_error(7); ref_act_error.setZero();
  Eigen::VectorXd ref_cnom_error(7); ref_act_error.setZero();

  ref_cnom_error = (obs_const_nom[0].tau + obs_const_nom[1].tau)/2;

  std::cout << "waypoints number : " << horizon << std::endl;

  // StepstauC
  for(int wp=0 ; wp < horizon ; wp++){
    for(int k=0 ; k<10 ; k++){
      tau_ref = ctrl_ts_nric.computeTau(virtual_plant_ref.q, virtual_plant_ref.dq, pred_reference[wp].transform);
      virtual_plant_ref.ddq = virtual_plant_ref_mass_matrix.inverse()*(tau_ref - virtual_plant_ref_coriolis - virtual_plant_ref_gravity);
      virtual_plant_ref.dq += virtual_plant_ref.ddq * dt;
      virtual_plant_ref.q += virtual_plant_ref.dq * dt;
      virtual_plant_ref.ee_pose = robot->RequestNominalEEPose(virtual_plant_ref.q);

      virtual_plant_ref_mass_matrix = robot->RequestNominalMassMatrix(virtual_plant_ref.q) + reflected_inertia;
      virtual_plant_ref_coriolis = robot->RequestNominalCoriolisVector(virtual_plant_ref.q, virtual_plant_ref.dq);
      virtual_plant_ref_gravity = robot->RequestNominalGravityVector(virtual_plant_ref.q);

      
      tau_act = ctrl_ts_nric.computeTau(virtual_plant_act.q, virtual_plant_act.dq, pred_action[wp].transform);
      virtual_plant_act.ddq = virtual_plant_act_mass_matrix.inverse()*(tau_ref - virtual_plant_act_coriolis - virtual_plant_act_gravity);
      virtual_plant_act.dq += virtual_plant_act.ddq * dt;
      virtual_plant_act.q += virtual_plant_act.dq * dt;
      virtual_plant_act.ee_pose = robot->RequestNominalEEPose(virtual_plant_act.q);

      virtual_plant_act_mass_matrix = robot->RequestNominalMassMatrix(virtual_plant_act.q) + reflected_inertia;
      virtual_plant_act_coriolis = robot->RequestNominalCoriolisVector(virtual_plant_act.q, virtual_plant_act.dq);
      virtual_plant_act_gravity = robot->RequestNominalGravityVector(virtual_plant_act.q);
    }
    ref_act_error += virtual_plant_ref.q - virtual_plant_act.q ;
    // std::cout << "ref_act_error" << std::endl;
    // for(int i=0 ; i<7 ; ++i){
    //   std::cout << ref_act_error(i) <<std::endl;
    // }
    // std::cout << "\n" <<std::endl;
  }
  ref_act_error = ref_act_error / horizon;
  std::cout << "Mean_ref_act_error" << std::endl;
  for(int i=0 ; i<7 ; ++i){
    std::cout << ref_act_error(i) <<std::endl;
  } 
  std::cout << "\n" <<std::endl;

  std::cout << "Mean_ref_cnom_error" << std::endl;
  for(int i=0 ; i<7 ; ++i){
    std::cout << ref_cnom_error(i) <<std::endl;
  } 
  std::cout << "\n" <<std::endl;

  // ctrl_ts_nric.tau_shift = - 3.0 * (ref_cnom_error + ref_act_error);
  // ctrl_ts_nric.tau_shift = - 3.0 * ref_act_error;
  // ctrl_ts_nric.tau_shift.setZero();

  std::cout << "tau_shift" << std::endl;
  for(int i=0 ; i<7 ; ++i){
    std::cout << ctrl_ts_nric.tau_shift(i) << std::endl;
  } 
  std::cout << "\n" <<std::endl;
}

void SwitchControllerNode::PublishJointState(){
  sensor_msgs::JointState msg;
  RobotState7 state = robot->ReadState();

  msg.header.stamp = ros::Time::now();
  for (int i=0;i<7;i++){
    msg.name.push_back(robot->joint_names[i]);
    msg.position.push_back(state.q[i]);
    msg.velocity.push_back(state.dq[i]);
    msg.effort.push_back(state.tau[i]);
  }
  joint_state_pub.publish(msg);
}

void SwitchControllerNode::PublishQPChecker(){
  std_msgs::Float64MultiArray msg;
  msg.data.clear();

  int exitcode = ctrl_ts_nric.GetQPExitCode();
  int qpnorm_cond = ctrl_ts_nric.GetQPNormCond();

  msg.data.push_back(exitcode);
  msg.data.push_back(qpnorm_cond);

  QP_checker_pub.publish(msg);
}

void SwitchControllerNode::PublishNominalEEPose(){
  geometry_msgs::PoseStamped msg;

  msg.header.stamp = ros::Time::now();
  Eigen::Affine3d nominal_ee = ctrl_ts_nric.GetNominalEEPose();
  Eigen::Vector3d nominal_ee_pos(nominal_ee.translation());
  Eigen::Quaterniond nominal_ee_rot(nominal_ee.linear());
  
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "nominal_EE";
  msg.pose.position.x = nominal_ee_pos.x();
  msg.pose.position.y = nominal_ee_pos.y();
  msg.pose.position.z = nominal_ee_pos.z();
  msg.pose.orientation.x = nominal_ee_rot.x();
  msg.pose.orientation.y = nominal_ee_rot.y();
  msg.pose.orientation.z = nominal_ee_rot.z();
  msg.pose.orientation.w = nominal_ee_rot.w();
  
  nominal_ee_pose_pub.publish(msg);
}

void SwitchControllerNode::PublishENR(){
  std_msgs::Float64MultiArray msg;
  msg.data.clear();
  RobotState7 state = robot->ReadState();
  Eigen::VectorXd nom_q = ctrl_ts_nric.GetNominalConfig();

  for (int i=0;i<7;i++){
    msg.data.push_back(nom_q(i) - state.q[i]);
  }
  enr_pub.publish(msg);
}

void SwitchControllerNode::PublishTauTask(){
  std_msgs::Float64MultiArray msg;
  msg.data.clear();

  std::array<double, 6> tau_task {ctrl_ts_imp.GetTauTask()};
  for (size_t i = 0; i < 6; i++){
    msg.data.push_back(tau_task[i]);
  }
  
  tau_task_pub.publish(msg);
}

void SwitchControllerNode::PublishTauATask(){
  std_msgs::Float64MultiArray msg;
  msg.data.clear();

  std::array<double, 6> tau_task {ctrl_ts_nric.GetTauATask()};
  for (size_t i = 0; i < 6; i++){
    msg.data.push_back(tau_task[i]);
  }
  
  tau_A_pub.publish(msg);
}

void SwitchControllerNode::PublishTauAJoint(){
  std_msgs::Float64MultiArray msg;
  msg.data.clear();

  std::array<double, 7> tau_joint {ctrl_ts_nric.GetTauAJoint()};
  for (size_t i = 0; i < 7; i++){
    msg.data.push_back(tau_joint[i]);
  }
  
  tau_A_joint_pub.publish(msg);
}

void SwitchControllerNode::PublishTauCTask(){
  std_msgs::Float64MultiArray msg;
  msg.data.clear();

  std::array<double, 6> tau_task {ctrl_ts_nric.GetTauCTask()};
  for (size_t i = 0; i < 6; i++){
    msg.data.push_back(tau_task[i]);
  }
  
  tau_C_pub.publish(msg);
}

void SwitchControllerNode::PublishTauDiffTask(){
  std_msgs::Float64MultiArray msg;
  msg.data.clear();

  std::array<double, 6> tau_task {ctrl_ts_nric.GetTauDiffTask()};
  for (size_t i = 0; i < 6; i++){
    msg.data.push_back(tau_task[i]);
  }
  
  tau_diff_pub.publish(msg);
}

void SwitchControllerNode::PublishEEPose(){
  //TODO: Publish TF?
  RobotState7 state = robot->ReadState();

  geometry_msgs::PoseStamped msg;
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(state.EE_pose.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "panda_EE";
  msg.pose.position.x = position.x();
  msg.pose.position.y = position.y();
  msg.pose.position.z = position.z();
  msg.pose.orientation.x = orientation.x();
  msg.pose.orientation.y = orientation.y();
  msg.pose.orientation.z = orientation.z();
  msg.pose.orientation.w = orientation.w();
  ee_pose_pub.publish(msg);
}

void SwitchControllerNode::SetControllerROSWrapper(int ctrl_mode){
  switch(ctrl_mode){
    case switch_controller::CTRLMODE_STOP:
      ROS_INFO("Controller Stopped");
    break;
    case switch_controller::CTRLMODE_IDLE:
      ROS_INFO("Idle Mode");
    break;
    case switch_controller::CTRLMODE_TASK_IMP:
      ROS_INFO("Task-space PD Control Mode");
    break;
    case switch_controller::CTRLMODE_TASK_NRIC:
      ROS_INFO("Task-space NRIC Control Mode");
    break;
  }
  SetController(ctrl_mode);
}

}

using namespace switch_controller;

bool keyboard_interface(SwitchControllerNode& robot){
  if(kbhit()){
    int keyPressed = getchar();
    switch(keyPressed){
      case '1':
        std::cout << "---Normal PID Mode---" << std::endl;
      break;
      case '2':
        std::cout << "---Gravity Compensation Mode---" << std::endl;
        robot.SetControllerROSWrapper(CTRLMODE_IDLE);
      break;
      
      case '3':
        std::cout << "---Task Space PD Control Mode---" << std::endl;
        robot.SetControllerROSWrapper(CTRLMODE_TASK_IMP);
      break;
      
      case '4':
        std::cout << "---Task Space NRIC Control Mode---" << std::endl;
        robot.SetControllerROSWrapper(CTRLMODE_TASK_NRIC);
      break;
      
      case 'q':
        return false;
      break;
      default:
      break;
    } 
  }
  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "switch_controller_node");
  
  ROS_INFO("Start switch_controller_node");
  if (argc ==1){
    ROS_INFO("usage : rosrun switch_controller switch_controller_node {ip} {robot}");
    return 0;
  }
  
  std::string ip(argv[1]);
  std::string robot_name(argv[2]);
  std::cout << "Robot ip:" << ip << std::endl;
  
  Panda robot(ip);
  
  SwitchControllerNode n(&robot);
  ros::Rate rate(1000);
  bool isRunning = true;
  init_keyboard();

  while(ros::ok() && isRunning){
    if (robot.CheckNewDataAndUse()){
      n.PublishEEPose();
      n.PublishJointState();
      n.PublishNominalEEPose();
      n.PublishENR();
      n.PublishQPChecker();
      n.PublishTauTask();
      n.PublishTauATask();
      n.PublishTauAJoint();
      n.PublishTauCTask();
      n.PublishTauDiffTask();
    }

    if (n.IsStoppedByError()){
      ROS_INFO("Error: set to idle mode");
      n.SetControllerROSWrapper(CTRLMODE_IDLE);
      n.ClearError();
    }
    isRunning = keyboard_interface(n);

    ros::spinOnce();
    rate.sleep();
  }
  std::cout << "Quit" << std::endl;
  close_keyboard();
  return 0;
}
