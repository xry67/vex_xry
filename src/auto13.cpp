#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "ToolControl.h"


void auto13(){

  double now_angle = 0;

  ChassisControlSTS.Move(3, 100, now_angle, 'n');
  ChassisControlSTS.Turn(10, 180, 'r');
  now_angle += 180;

}