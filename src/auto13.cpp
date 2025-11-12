#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "ToolControl.h"


void auto13(){
  ChassisControlSTS.CurveMove(10, 780, 700, 'p');
}