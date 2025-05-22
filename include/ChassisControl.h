#ifndef ChassisControl_H  
#define ChassisControl_H 

#include <iostream>  
#include <chrono>  
#include <thread>  
#include <vector>
#include "vex.h"
#include "PID.h"
#include "robot-config.h"

// 视觉识别物体限制
class AiTarget{
  public: 
  std::vector<int>  Limit_w = {0, aiVision_w};
  std::vector<int>  Limit_h = {0, aiVision_h};
  std::vector<int>  Limit_x = {0, aiVision_w};
  std::vector<int>  Limit_y = {0, aiVision_h};
  aivision::colordesc Color = Red;  // 目标颜色
  std::vector<double>  Limit_proportion = {0, 100};
  double Align = 0; // 对齐位置

  int Target_CenterX = -1;
  int Target_CenterY = -1;
  int Target_Width   = -1;
  int Target_Height = -1;

  AiTarget(aivision::colordesc color) : Color(color){}

  AiTarget(std::vector<int> limit_w, std::vector<int>  limit_h, std::vector<int>  limit_x, std::vector<int>  limit_y, double align, aivision::colordesc color):
  Limit_w(limit_w),
  Limit_h(limit_h),
  Limit_x(limit_x),
  Limit_y(limit_y),
  Align(align),
  Color(color){}

  // 检测是否符合限制要求
  bool IsGood(aivision::object obj){
    if (obj.centerX < Limit_x[0] || obj.centerX > Limit_x[1] ){
      return false;
    }
    if (obj.centerY < Limit_y[0] || obj.centerY > Limit_y[1] ){
      return false;
    }
    if (obj.width < Limit_w[0] || obj.width > Limit_w[1] ){
      return false;
    }
    if (obj.height < Limit_h[0] || obj.height > Limit_h[1] ){
      return false;
    }
    if (obj.width / obj.height < Limit_proportion[0] || obj.width / obj.height > Limit_proportion[1] ){
      return false;
    }
    return true;
  }

  bool SetTarget(){
    aiVision.takeSnapshot(this->Color);  // 当前视觉传感器的数据
    int size = 0;
    for (int i=0; i<aiVision.objectCount; i++){
      if (this->IsGood(aiVision.objects[i])){
        if (aiVision.objects[i].width * aiVision.objects[i].height > size)
          Target_CenterX = aiVision.objects[i].centerX;
          Target_CenterY = aiVision.objects[i].centerY;
          Target_Width   = aiVision.objects[i].width;
          Target_Height   = aiVision.objects[i].height;
      }
    }
    return Target_CenterX != -1;
  }

  // 从序列中得到最合理的一个中心x返回
  int GetTargetCenterX(){
    int num = aiVision_w * aiVision_h * aiVision_w * aiVision_h;
    int ans = -1;
    std::cout << "----" << std::endl;
    for (int i=0; i<aiVision.objectCount; i++){
      int disparity = abs(aiVision.objects[i].centerX - Target_CenterX) * 10 + 
                      abs(aiVision.objects[i].centerY - Target_CenterY) + 
                      abs(aiVision.objects[i].width - Target_Width) +
                      abs(aiVision.objects[i].height - Target_Height);
      std::cout << disparity << std::endl;
      if (disparity < num){
        num = disparity;
        ans = i;
      }
    }
    std::cout << "ans " << ans << std::endl;
    Target_CenterX = aiVision.objects[ans].centerX;
    Target_CenterY = aiVision.objects[ans].centerY;
    Target_Width   = aiVision.objects[ans].width;
    Target_Height   = aiVision.objects[ans].height;

    return aiVision.objects[ans].centerX;
  }

};


// 底盘运动控制算法
class ChassisControl{
  public: 

  // 底盘重新清零
  void Reset();

  // 设置停止方式
  void SetStopMod(std::string str);

  // 底盘停止
  void ChassisStop();

  // 底盘停止（抱死）
  void ChassisStoppingHold();

  // 底盘停止（滑行）
  void ChassisStoppingCoast();
  
  // 底盘直行初始化
  void ChassisStraight(int negative);

  // 底盘前进
  void ChassisForward(double v, double yaw);

  // 底盘左轮控制
  void ChassisLeftControl(double v);

  // 底盘左轮控制
  void ChassisRightControl(double v);

  // 获取当前电机的位置
  double GetPositions_mean();

  // 获取当前电机的速度
  double GetVelocity_mean();

  // 设置电机归位 
  void ChassisRsetPosition();

  // 直线运动函数
  void Move(double v, double val, double yaw, char direction);

  // 直线运动函数(可选多线程)
  void Move(double v, double val, double yaw, char direction, bool task);

  // 直线运动函数（使用距离传感器）
  void MoveToDistance(double v, double val, std::string DistanceChioce);

  // 直线运动函数(使用距离传感器)
  void MoveToDistance(double v, double val, std::string DistanceChioce, bool task);

  // 直线运动函数（使用定位轮）
  void MoveBySmallWheel(double v, double val, double yaw, char direction);

  // 直线运动函数（使用定位轮）
  void MoveBySmallWheel(double v, double val, double yaw, char direction, bool task);

  // 旋转函数
  void Turn(double v, double val, char direction);

  // 旋转函数(可选多线程)
  void Turn(double v, double val, char direction, bool task);

  // 旋转到指定角度
  void TurnTo(double v, double val, bool task);

  // 追随某个目标
  void FollowTarget(double v, double val, AiTarget Target);

  // 追随某个目标(多线程)
  void FollowTarget(double v, double val, AiTarget Target, bool task);

  // 等待底盘运动完成
  void WaitMoveFinish();

  // 获取当前是否在运动
  bool IsMoving();

  // 立即停止当前运动
  void StopMoving();

  // 使用坐标-移动到某一坐标位置
  void MoveTo(double x, double y, double v, bool task);


  private: 
    std::string StopMod = "Hold";    // 设置停止方式

    double  Move_time_out   = 3000;  // 超时保护，默认3000毫秒
    double  Stuck_time_out  = 1000;  // 电机卡死保护，默认1000毫秒
    double  Stuck_threshold = 2;     // 电机卡死认定阈值，单位dps
    bool    Moveing         = true; // 记录是否在运动
    bool    MoveEnd         = true;  // 记录运动是否完成
    int     MoveId          = 0;     // 运动id记录（计数）

    PID     *StraightPid;            // 创建PID控制器实例 
    double  StraightAngle   = 0.0;   // 记录直行的角度
    int     StraightNegative = 1;    // 记录执行方向

    int Target_CenterX       = 0;    // 目标物体的中心x
    int Target_CenterY       = 0;    // 目标物体的中心y
    int Target_Width         = 0;    // 目标物体的宽度
    int Target_Height        = 0;    // 目标物体的高度

};

//  多线程运动友元函数
int MoveTask(void* args);
//  多线程运动（直线定位轮）友元函数
int MoveBySmallWheelTask(void* args);
//  多线程旋转友元函数
int TurnTask(void* args);

// 多线程准备的结构体函数
struct ChassisMoveParams{
    ChassisControl* self;
    double v = 0.0;        // 速度
    double val = 0.0;      // 加速度
    double yaw = 0.0;      // 减速度
    char direction;  // 运动方向
};

extern ChassisControl ChassisControlSTS;  // sts底盘控制

#endif