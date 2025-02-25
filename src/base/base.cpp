#include "base.h"

namespace switch_controller{

/**
 * @brief Construct a new Robot 7:: Robot 7 object
 * 
 */
Robot7::Robot7(){
  has_new_data = false;
  stop_ctrl = false;
}

/**
 * @brief Destroy the Robot 7:: Robot 7 object
 * 
 */
Robot7::~Robot7(){
  stop_ctrl = true;
}

// RobotState7 Robot7::GetState(){
//   if(has_new_data){
//     has_new_data = false;
//   }
//   return robot_state;
// }

RobotState7 Robot7::ReadState(){
  return robot_state;
}

void Robot7::StopControl(){
  stop_ctrl = true;
}

bool Robot7::CheckNewDataAndUse(){
  if (has_new_data){
    has_new_data = false;
    return true;
  }
  return false;
}
/**********************************************
 * Belows functions should be implemented.
***********************************************/
/*
1. state update functions
2. robot model acquisition functions
3. control loop functions
*/

/**
 * @brief This function inputs the state of the real robot
 * , and updates robot_state(RobotState7) and/or saves a current real robot state temporarily.
 */
void Robot7::UpdateState(){
  if (m.try_lock()){
    has_new_data = true;
    // update robot_states
  }
}

Duration Robot7::getDuration(){
  Duration duration;
  return duration;
}
/**
 * @brief Get the current coriolis vector of the robot
 * 
 * @return RobotTorque7 
 */

RobotInertia Robot7::GetMassMatrix(){
  RobotInertia mass;
  return mass;
}

RobotTorque7 Robot7::GetCoriolisVector(){
  RobotTorque7 coriolis;
  // get current coriolis vector of the robot.
  return coriolis;
}

/**
 * @brief Get the current gravity vector of the robot
 * 
 * @return RobotTorque7 
 */
RobotTorque7 Robot7::GetGravityVector(){
  RobotTorque7 gravity;
  // get current gravity vector of the robot.
  return gravity;
}


Jacobian7 Robot7::GetTaskJacobian(){
  Jacobian7 jacobian;
  return jacobian;
}

Jacobian7 Robot7::GetBodyJacobian(){
  Jacobian7 b_jacobian;
  return b_jacobian;
}

Eigen::VectorXd Robot7::RequestNominalCoriolisVector(const Eigen::VectorXd& q, const Eigen::VectorXd& dq){
  Eigen::VectorXd coriolis;
  return coriolis;
}

Eigen::VectorXd Robot7::RequestNominalGravityVector(const Eigen::VectorXd& q){
  Eigen::VectorXd gravity;
  return gravity;
}

Eigen::MatrixXd Robot7::RequestNominalMassMatrix(const Eigen::VectorXd& q){
  Eigen::MatrixXd mass;
  return mass;
}

Eigen::Affine3d Robot7::RequestNominalEEPose(const Eigen::VectorXd &q){
  Eigen::Affine3d ee_pose;
  return ee_pose;
}

Eigen::MatrixXd Robot7::RequestNominalBodyJacobian(const Eigen::VectorXd &q){
  Eigen::MatrixXd jacobian;
  return jacobian;
}

Eigen::MatrixXd Robot7::RequestNominalTaskJacobian(const Eigen::VectorXd &q){
  Eigen::MatrixXd jacobian;
  return jacobian;
}

BilinearLPF::BilinearLPF(double sampleRate, double cutoffFrequency)
  : sampleRate(sampleRate), cutoffFrequency(cutoffFrequency) {
  for (int i = 0; i < 6; i++) {
    prevInput1.fill(0.0);
    prevOutput1.fill(0.0);
  }
}

void BilinearLPF::process(const std::array<double, 6>& input, std::array<double, 6>& output) {
  for (int i = 0; i < 6; i++) {
    output[i] = (2 - cutoffFrequency / sampleRate) / (2 + cutoffFrequency / sampleRate) * prevOutput1[i]
              + (cutoffFrequency / sampleRate) / (2 + cutoffFrequency / sampleRate) * (input[i] + prevInput1[i]);
  }
  prevInput1 = input;
  prevOutput1 = output;
}

BilinearLPF7::BilinearLPF7(double sampleRate, double cutoffFrequency)
  : sampleRate(sampleRate), cutoffFrequency(cutoffFrequency) {
  for (int i = 0; i < 7; i++) {
    prevInput1.fill(0.0);
    prevOutput1.fill(0.0);
  }
}

void BilinearLPF7::process(const std::array<double, 7>& input, std::array<double, 7>& output) {
  for (int i = 0; i < 7; i++) {
    output[i] = (2 - cutoffFrequency / sampleRate) / (2 + cutoffFrequency / sampleRate) * prevOutput1[i]
              + (cutoffFrequency / sampleRate) / (2 + cutoffFrequency / sampleRate) * (input[i] + prevInput1[i]);
  }
  prevInput1 = input;
  prevOutput1 = output;
}

/**
 * @brief This function has an Idle loop of the switch controller.
 * 
 */
void Robot7::IdleControl(){
  std::cout << "nothing" << std::endl;
  return;
}

/**
 * @brief This function has an Torque control loop of the switch controller.
 * 
 */
void Robot7::TorqueControl( \
  InitFn init_fn, TorqueCtrlLoopFn loop_fn, StopFn stop_fn \
){
  // // Example
  // RobotTorque7 torque;
  // bool loop_trigger;
  // RobotState7 state;

  // while(true){
  //   // ex. loop_trigger = robotAPI.loop_trigger()
  //   if(loop_trigger){
  //     UpdateState(state);
  //     torque = ctrl_loop_fn(state);
  //   }

  //   // The loop code should end if 'stop_ctrl' is true.
  //   if(stop_ctrl){
  //     break;
  //   }
  // }
}



/**
 * @brief Initialize a thread pool
 * 
 */
ThreadPool::ThreadPool()
  : num_threads(2)
  , stop_all(false)
{
  worker_threads_.reserve(num_threads);
  for (size_t i=0; i<num_threads; i++){
    worker_threads_.emplace_back([this](){this->WorkerThread();});
  }
}

/**
 * @brief Destroy the thread pool
 * 
 */
ThreadPool::~ThreadPool(){
  stop_all = true;
  cv_job_q.notify_all();
  for (auto& t : worker_threads_){
    t.join();
  }
}

/**
 * @brief Thread pool loop for changable controller
 * 
 */
void ThreadPool::WorkerThread(){
  while (true){
    std::unique_lock<std::mutex> lock(m_job_q);
    cv_job_q.wait(lock, [this](){ return !this->jobs.empty() || stop_all; });
    if (stop_all && this->jobs.empty()){
      return;
    }
    // pop first job
    std::function<void()> job = std::move(jobs.front());
    jobs.pop();
    lock.unlock();

    // do job
    job();
  }
}

/**
 * @brief Enqueue job(control thread)
 * 
 * @param job Function to execute
 */
void ThreadPool::EnqueueJob(std::function<void()> job) {
  if (stop_all) {
    throw std::runtime_error("Stop All Thread in this pool!");
  }
  {
    std::lock_guard<std::mutex> lock(m_job_q);
    jobs.push(std::move(job));
  }
  cv_job_q.notify_one();
}
}