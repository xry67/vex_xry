#include <iostream>  
#include <chrono>  
#include <algorithm> // 包含 min 和 max 函数
#include <thread>  
#include "vex.h"
#include "robot-config.h"
#include "units.h"
#include "ToolControl.h"
#include "ChassisControl.h"

extern double ToolControl_speed1  = 0;
extern double ToolControl_speed2  = 0;
extern double ToolControl_speed3  = 0;
extern double ToolControl_timeout = 0;
vex::task ToolMotorSpinTask;

int ToolMotorSpinTaskFun(){
  ToolMotor1.spin(forward, ToolControl_speed1, voltageUnits::volt);
  ToolMotor2.spin(forward, ToolControl_speed2, voltageUnits::volt);
  ToolMotor3.spin(forward, ToolControl_speed3, voltageUnits::volt);
  wait(ToolControl_timeout, msec);
  ToolMotor1.spin(forward, 0, voltageUnits::volt);
  ToolMotor2.spin(forward, 0, voltageUnits::volt);
  ToolMotor3.spin(forward, 0, voltageUnits::volt);
  return 0;
}

void ToolMotorSpin(double speed1, double speed2, double speed3, double timeout){
  ToolMotor1.spin(forward, speed1, voltageUnits::volt);
  ToolMotor2.spin(forward, speed2, voltageUnits::volt);
  ToolMotor3.spin(forward, speed3, voltageUnits::volt);
  wait(timeout, msec);
  ToolMotor1.spin(forward, 0, voltageUnits::volt);
  ToolMotor2.spin(forward, 0, voltageUnits::volt);
  ToolMotor3.spin(forward, 0, voltageUnits::volt);
}

// 工作电机旋转设置
void ToolMotorSpin(double speed1, double speed2, double speed3, double timeout, bool task){

  
  if (task){
    ToolControl_speed1  = speed1;
    ToolControl_speed2  = speed2;
    ToolControl_speed3  = speed3;
    ToolControl_timeout = timeout;
    ToolMotorSpinTask = vex::task(ToolMotorSpinTaskFun);  // 电量监控开启
  }else{
    ToolMotorSpin(speed1, speed2, speed3, timeout);
  }
}

// 设置数据
void SetToolMotorSpin(double speed1, double speed2, double speed3, double timeout){
    ToolControl_speed1  = speed1;
    ToolControl_speed2  = speed2;
    ToolControl_speed3  = speed3;
    ToolControl_timeout = timeout;
}

int AutoIntake(){
  // 分球暂停计数器
    int ErrorBall = 0;
    int timeout = ToolControl_timeout;
    double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();// 当前时间
    double now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();// 当前时间

    ChassisControlSTS.ChassisStraight(ChassisInertial.rotation());

    int MoveCount = 0; 
    ChassisControlSTS.SetStopModCoast();
    while (now_time - start_time < timeout / 1000){    
        now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();// 当前时间

        if ((stance == 'r' && OpticalHaveBlue()) || (stance == 'b' && OpticalHaveRed())){
            digitalF.set(1);
            ErrorBall = 20;
            break;
        }
        else if ((stance == 'b' && OpticalHaveBlue()) || (stance == 'r' && OpticalHaveRed())){
            ErrorBall = 0;
        }

        if (ErrorBall > 0){
            digitalF.set(1);
            // ToolMotor1.spin(forward, -1 * ToolControl_speed1, voltageUnits::volt);
        }else{
            digitalF.set(0);
            // ToolMotor1.spin(forward, ToolControl_speed1, voltageUnits::volt);
        }

        if (MoveCount <= 0){
          MoveCount = 500;
        }
        // if (MoveCount > 250){
        //   // ChassisControlSTS.ChassisForward(10);
        //   // ChassisControlSTS.ChassisLeftControl(10);
        //   // ChassisControlSTS.ChassisRightControl(10);
        // }else{
        //   ChassisControlSTS.ChassisForward(0);
        // }

        MoveCount -= 10;

        ToolMotor1.spin(forward,  ToolControl_speed1, voltageUnits::volt);
        ToolMotor2.spin(forward, ToolControl_speed2, voltageUnits::volt);

        wait(10, msec);
    }
    // ChassisControlSTS.SetStopModHold();

    return 0;
}

// 分球的放置
int AutoPush(){

    // 分球暂停计数器
    int ErrorBall = 0;
    int timeout = ToolControl_timeout;
    double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();// 当前时间

    double now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();// 当前时间
    while (now_time - start_time < timeout / 1000){    
        
        now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();// 当前时间

        if ((stance == 'r' && OpticalHaveBlue()) || (stance == 'b' && OpticalHaveRed())){
            ErrorBall = 20;
        }
        else if ((stance == 'b' && OpticalHaveBlue()) || (stance == 'r' && OpticalHaveRed())){
            ErrorBall = 0;
        }

        if (ErrorBall > 0){
            digitalF.set(1);
            // ToolMotor1.spin(forward, -1 * ToolControl_speed1, voltageUnits::volt);
        }else{
            digitalF.set(0);
            // ToolMotor1.spin(forward, ToolControl_speed1, voltageUnits::volt);
        }
        
        ToolMotor1.spin(forward,  ToolControl_speed1, voltageUnits::volt);
        ToolMotor2.spin(forward, ToolControl_speed2, voltageUnits::volt);
        ToolMotor3.spin(forward, ToolControl_speed3, voltageUnits::volt);

        // if (std::abs(ToolMotor1.velocity(dps)) < 200){
        //   ToolMotor1.spin(forward,  -1 * ToolControl_speed1, voltageUnits::volt);
        //   wait(200, msec);
        // }

        // if (std::abs(ToolMotor2.velocity(dps)) < 200){
        //   ToolMotor2.spin(forward,  -1 * ToolControl_speed2, voltageUnits::volt);
        //   wait(200, msec);
        // }

        // if (std::abs(ToolMotor3.velocity(dps)) < 200){
        //   ToolMotor3.spin(forward,  -1 * ToolControl_speed3, voltageUnits::volt);
        //   wait(200, msec);
        // }

          // digitalE.set(0);
          // wait(500, msec);
          // digitalE.set(1);
          // wait(500, msec);
        
        // if (timeout < 500){
        //   digitalE.set(0);
        //   wait(500, msec);
        //   digitalE.set(1);
        //   break;
        // }
        wait(10, msec);
    }
  ToolMotor1.spin(forward,  0, voltageUnits::volt);
  ToolMotor2.spin(forward, 0, voltageUnits::volt);
  ToolMotor3.spin(forward, 0, voltageUnits::volt);
    return 0;
}

