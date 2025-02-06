#include "controllers.h"
#include "switch_controller.h"
#include <ctime>
#include <iostream>

namespace switch_controller{

void SwitchController::RunCtrl(){
  InitFn init_fn;
  TorqueCtrlLoopFn loop_fn;
  StopFn stop_fn;

  try{
    switch(ctrl_mode){
      case CTRLMODE_STOP:
        //std::cout << "stopped" << std::endl;
        StopCtrl();
      break;
      case CTRLMODE_IDLE:
        //std::cout << "idle" << std::endl;
        robot->IdleControl();
      break;
      case CTRLMODE_TASK_IMP:
        //std::cout << "task imp ctrl" << std::endl;
        //task-space impedance control
        init_fn = boost::bind(&TaskSpaceImpCtrl::init, &ctrl_ts_imp);
        loop_fn = boost::bind(&TaskSpaceImpCtrl::loop, &ctrl_ts_imp, _1);
        stop_fn = boost::bind(&TaskSpaceImpCtrl::stop, &ctrl_ts_imp);
        robot->TorqueControl(init_fn, loop_fn, stop_fn);
      break;
      case CTRLMODE_TASK_NRIC:
        //std::cout << "task nric ctrl" << std::endl;
        //task-space NRIC control
        init_fn = boost::bind(&TaskSpaceNRICCtrl::init, &ctrl_ts_nric);
        loop_fn = boost::bind(&TaskSpaceNRICCtrl::loop, &ctrl_ts_nric, _1);
        stop_fn = boost::bind(&TaskSpaceNRICCtrl::stop, &ctrl_ts_nric);
        robot->TorqueControl(init_fn, loop_fn, stop_fn);
      break;
    }
  } catch(int expn) {
    StopCtrl();
    is_stopped_by_error = true;
  }
}

void SwitchController::StopCtrl(){
  ctrl_mode = CTRLMODE_STOP;
  robot->StopControl();
  std::this_thread::sleep_for(std::chrono::seconds(1));
}

void SwitchController::SetController(int ctrl_mode_cmd){
  if (is_ctrl_running){
    StopCtrl();
  }
  is_ctrl_running = true;
  SetCtrlMode(ctrl_mode_cmd);
  EnqueueJob([this](){ this->RunCtrl(); });  
}

}