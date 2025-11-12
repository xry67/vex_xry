#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "ToolControl.h"


void auto13(){

  double now_angle = 0;
  while (true){
    ChassisControlSTS.Move(10, 1000, now_angle, 'n');
    ChassisControlSTS.Turn(10, 180, 'r');
    now_angle += 180;
  }
}