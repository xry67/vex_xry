#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "AutoTools.h"
#include <vector>
#include <deque>
#include <RemoteControl.h>

void auto14()
{
  ChassisInertial.setRotation(-180 - 360, deg);
  intake_and_pushback_new_r(-180 - 360, 2000, 1500);
}