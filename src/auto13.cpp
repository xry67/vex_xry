#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "AutoTools.h"

void auto13()
{

  int line = 50;

  // while(1){
  //   ChassisControlSTS.ChassisLeftControl(2);
  //   ChassisControlSTS.ChassisRightControl(-2);
  //   Brain.Screen.printAt(150, 150, "%f", ChassisControlSTS.GetVelocity_pct_mean());
  //   wait(10, msec);
  // }

  for (double a : {180, 130, 100, 90, 45, 30, 10})
  {
      
    ChassisControlSTS.Turn_PPS = {0.025, 0.5, 5};
    ChassisControlSTS.min_turn_speed = 1;

    ChassisInertial.setRotation(0, deg);
    ChassisControlSTS.Turn(100, a, 'r');
    Controller.rumble(".");
    wait(2000, msec);
    Brain.Screen.printAt(150, line, "%f", ChassisInertial.rotation());
    line += 20;
  }
}