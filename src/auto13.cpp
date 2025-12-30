#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "AutoTools.h"

void auto13()
{
  ChassisControlSTS.ChassisStoppingHold();
  ChassisControlSTS.ChassisLeftControl_pct(0);
  ChassisControlSTS.ChassisRightControl_pct(0);
  wait(100000, msec);
}