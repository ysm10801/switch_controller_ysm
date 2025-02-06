#ifndef SWITCH_CONTROLLER_H
#define SWITCH_CONTROLLER_H

#include "base.h"
#include "controllers.h"

namespace switch_controller{

enum CtrlMode{
  CTRLMODE_STOP,
  CTRLMODE_IDLE,
  CTRLMODE_TASK_IMP,
  CTRLMODE_TASK_NRIC
};

class SwitchController : protected ThreadPool{
private:
  int ctrl_mode;
  bool is_ctrl_running;
  bool is_stopped_by_error;

public:
  Robot7 *robot;
  TaskSpaceImpCtrl ctrl_ts_imp;
  TaskSpaceNRICCtrl ctrl_ts_nric;

  SwitchController(Robot7 *robot_ptr)
    : ctrl_mode(0)
    , is_ctrl_running(false)
    , robot(robot_ptr)
    , ctrl_ts_imp(robot_ptr)
    , ctrl_ts_nric(robot_ptr)
    , is_stopped_by_error(false)
  {};
  ~SwitchController(){ 
    ctrl_mode = CTRLMODE_STOP;
    StopCtrl();
  };

  void SetCtrlMode(int mode) {ctrl_mode = mode;};
  int GetCtrlMode() {return ctrl_mode;};
  bool IsStoppedByError() {return is_stopped_by_error;}
  void ClearError() {is_stopped_by_error = false;}

  void StopCtrl();
  void RunCtrl();
  void SetController(int ctrl_mode);
};

}
#endif