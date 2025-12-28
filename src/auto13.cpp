#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "AutoTools.h"

void auto13()
{
  digitalH.set(1);
  intake_and_pushback_new(0, 500, 1500);
}