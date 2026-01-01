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

  for (double a : {90})
  {
    ChassisInertial.setRotation(0, deg);
    ChassisControlSTS.Turn(16, a, 'r');
    wait(2000, msec);
    line += 20;
  }
}