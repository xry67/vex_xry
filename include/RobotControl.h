#ifndef ROBOTCONTROL_H  
#define ROBOTCONTROL_H

#include "vex.h"

// 指定电机旋转到指定位置
void MotorSpinFor(vex::motor Motor, double v, double val, bool task);

// 将所有装置恢复到默认状态（马达停转）
void RestoreInitial();

// 等待陀螺仪校准完成
void WaitInertialCalibrating();

// 保证陀螺仪矫正完成
int InertialCalibrating();

// 检测是否飘值
bool CheckInertialCalibrationFailed(double time, double max_error);


#endif


