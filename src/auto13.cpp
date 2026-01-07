#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "AutoTools.h"

void auto13()
{

  int now_l = 150;
  ChassisControlSTS.Over_count = 4;
  for (int a : {30, 45, 90, 130, 180})
  {
    ChassisInertial.setRotation(0, deg);
    ChassisControlSTS.Turn(16, a, 'l');
    wait(1000, msec);

    Brain.Screen.printAt(200, now_l, "%f", ChassisInertial.rotation());
    now_l += 20;
  }
}