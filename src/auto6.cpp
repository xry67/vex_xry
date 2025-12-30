#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "AutoTools.h"

// 右0+4 横向向右摆位
void auto6()
{
  ChassisInertial.setRotation(0, degrees);
  ChassisControlSTS.Over_count = 1;
  // 第一个导入桶
  WaitFrontDistanceArriveThenStopMove(12, 1690, 0, 440);
  digitalH.set(1);
  wait(100, msec); // 需要保留等待时间，否则自转的旋转中心会由于惯性发生改变
  // 转至180
  ChassisControlSTS.AutoPPSTurnTo(10, 90);

  intake_and_pushback_new(90, 450, 1500);

  // 钩子
  Hook(90);
}