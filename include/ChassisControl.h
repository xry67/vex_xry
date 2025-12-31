#ifndef ChassisControl_H  
#define ChassisControl_H 

#include <iostream>  
#include <chrono>  
#include <thread>  
#include <vector>
#include "vex.h"
#include "PPS.h"
#include "robot-config.h"

double GetDistance_front();

double GetDistance_back();

double GetDistance_left();

double GetDistance_right();

// 组合运动参数相关
struct ComboMoveParams{
    double v = 0.0;        // 速度
    double val = 0.0;      // 距离
    double yaw = 0.0;      // 角度
    char direction;        // 运动方向
    void (*func)();        // 需要执行的函数

    // 构造函数
    ComboMoveParams(double speed, double distance, double angle, char dir, void (*function)())
        : v(speed), val(distance), yaw(angle), direction(dir), func(function) {}
};

// 视觉识别物体限制
// class AiTarget{
//   public: 
//   std::vector<int>  Limit_w = {0, aiVision_w};
//   std::vector<int>  Limit_h = {0, aiVision_h};
//   std::vector<int>  Limit_x = {0, aiVision_w};
//   std::vector<int>  Limit_y = {0, aiVision_h};
//   aivision::colordesc *Color;  // 目标颜色
//   std::vector<double>  Limit_proportion = {0, 100};
//   double Align = 0; // 对齐位置

//   int Target_CenterX = -1;
//   int Target_CenterY = -1;
//   int Target_Width   = -1;
//   int Target_Height = -1;

//   AiTarget(){}

//   AiTarget(aivision::colordesc color) : Color(&color){}

//   AiTarget(std::vector<int> limit_w, std::vector<int>  limit_h, std::vector<int>  limit_x, std::vector<int>  limit_y, double align, aivision::colordesc color):
//   Limit_w(limit_w),
//   Limit_h(limit_h),
//   Limit_x(limit_x),
//   Limit_y(limit_y),
//   Align(align),
//   Color(&color){}

//   void SetColor(aivision::colordesc color){
//     Color = &color;
//   }

//   // 检测是否符合限制要求
//   bool IsGood(aivision::object obj){
//     if (obj.centerX < Limit_x[0] || obj.centerX > Limit_x[1] ){
//       return false;
//     }
//     if (obj.centerY < Limit_y[0] || obj.centerY > Limit_y[1] ){
//       return false;
//     }
//     if (obj.width < Limit_w[0] || obj.width > Limit_w[1] ){
//       return false;
//     }
//     if (obj.height < Limit_h[0] || obj.height > Limit_h[1] ){
//       return false;
//     }
//     if (obj.width / obj.height < Limit_proportion[0] || obj.width / obj.height > Limit_proportion[1] ){
//       return false;
//     }
//     return true;
//   }

//   bool SetTarget(){
//     aiVision.takeSnapshot(*(this->Color));  // 当前视觉传感器的数据
//     int size = 0;
//     int k = 100;
//     // Brain.Screen.printAt( 100, 140, "count %d" , aiVision.objectCount);
//     for (int i=0; i<aiVision.objectCount; i++){
//       if (this->IsGood(aiVision.objects[i])){
//         // Brain.Screen.printAt( 10, k, "w %d. h %d. s. %d" , Target_Width, Target_Height, aiVision.objects[i].width * aiVision.objects[i].height);
//         k += 20;
//         if (aiVision.objects[i].width * aiVision.objects[i].height > size)
//           {
//             size = aiVision.objects[i].width * aiVision.objects[i].height;
//             Target_CenterX = aiVision.objects[i].centerX;
//             Target_CenterY = aiVision.objects[i].centerY;
//             Target_Width   = aiVision.objects[i].width;
//             Target_Height   = aiVision.objects[i].height;
//             // Brain.Screen.printAt( 10, k, "x %d. w %d. h %d. s. %d" ,Target_CenterX, Target_Width, Target_Height, size);
//             k += 20;
//         }
//       }
//     }
//     // Brain.Screen.printAt( 10, k, "x %d. w %d. h %d. s. %d" ,Target_CenterX, Target_Width, Target_Height, size);
//     return Target_CenterX != -1;
//   }

//   // 从序列中得到最合理的一个中心x返回
//   int GetTargetCenterX(){
//     int num = aiVision_w * aiVision_h * aiVision_w * aiVision_h;
//     int ans = -1;
//     std::cout << "----" << std::endl;
//     // Brain.Screen.printAt( 10, 50, "s. %d" ,aiVision.objectCount);

//     for (int i=0; i<aiVision.objectCount; i++){
//       int disparity = abs(aiVision.objects[i].centerX - Target_CenterX) * 10 + 
//                       abs(aiVision.objects[i].centerY - Target_CenterY) + 
//                       abs(aiVision.objects[i].width - Target_Width) +
//                       abs(aiVision.objects[i].height - Target_Height);
//       std::cout << disparity << std::endl;
//       if (disparity < num){
//         num = disparity;
//         ans = i;
//       }
//     }

//     Target_CenterX = aiVision.objects[ans].centerX;
//     Target_CenterY = aiVision.objects[ans].centerY;
//     Target_Width   = aiVision.objects[ans].width;
//     Target_Height   = aiVision.objects[ans].height;

//     return aiVision.objects[ans].centerX;
//   }

//   // 添加赋值运算符重载
//   AiTarget& operator=(const AiTarget& other) {

//       Limit_w = other.Limit_w;
//       Limit_h = other.Limit_h;
//       Limit_x = other.Limit_x;
//       Limit_y = other.Limit_y;
//       // Color = new aivision::colordesc(10, other.Color->red, other.Color->green, other.Color->blue, other.Color->hangle, other.Color->hdsat);
//       Color = &Red;
//       Align = other.Align;
//       Target_CenterX = other.Target_CenterX;
//       Target_CenterY = other.Target_CenterY;
//       Target_Width = other.Target_Width;
//       Target_Height = other.Target_Height;

//       return *this;
//   }

// };


// 底盘运动控制算法
class ChassisControl{
  public: 

  ChassisControl();
  

  // 重新设置相关数据
  void ResetPPS();

  // 重新设置当前最大速度
  void ResetMaxSpeed(double now_speed);

  // 重新设置当前直行角度
  void ResetStraightAngle(double yaw);

  // 底盘重新清零
  void Reset();

  // 设置停止方式
  void SetStopMod(std::string str);

  // 设置停止模式为hold
  void SetStopModHold();

  // 设置停止模式为Coasst
  void SetStopModCoast();

  // 获得缓冲后的速度
  double GetBufferSpeed(double nowspeed, double output, int negative);

  // 获得缓冲后的速度
  double GetBufferSpeed_turn(double nowspeed, double output, int negative);

  // 底盘停止
  void ChassisStop();

  // 底盘停止（抱死）
  void ChassisStoppingHold();

  // 底盘停止（滑行）
  void ChassisStoppingCoast();
  
  // 底盘直行初始化
  void ChassisStraight(double angle);

  // 底盘前进
  void ChassisForward(double v);

  // 更新底盘直行对准角度
  void ChassisForwardAngle(double angle);

  // 底盘左轮控制
  void ChassisLeftControl(double v);

  // 底盘左轮控制
  void ChassisRightControl(double v);

  // 底盘左轮控制-百分比
  void ChassisLeftControl_pct(double v);

  // 底盘左轮控制-百分比
  void ChassisRightControl_pct(double v);

  // 底盘控制-百分比整体
  void ChassisMoveControl_pct(double v);

  // 获取当前电机的位置
  double GetPositions_mean();

  // 获取当前电机的速度
  double GetVelocity_mean();

  // 获取当前的速度pct
  double GetVelocity_pct_mean();

  // 等待电机编码器到达某个值
  void WaitPositionArrive(double val);

  // 等待定位轮到达
  void WaitSmallWheelPositionArrive(double val);

  // 设置电机归位 
  void ChassisRsetPosition();

  // 运动前
  void MoveStart();

  // 运动后
  void MoveOver();

  // 直线运动函数
  void Move(double v, double val, char direction);

  // 直线运动函数（可选多线程）
  void Move(double v, double val, char direction, bool task);

  // 直线运动函数(可选角度)
  void Move(double v, double val, double yaw, char direction);

  // 直线运动函数(可选角度与多线程 可选多线程)
  void Move(double v, double val, double yaw, char direction, bool task);

  // 曲线移动
  void CurveMove(double v, double r, double TargetPosition, char direction);

// 曲线运动可选多线程
  void CurveMove(double v, double r, double TargetPosition, char direction, bool task);

  // 组合运动
  void ComboMove(std::vector<ComboMoveParams> MoveParams);

  // 直线运动函数（使用距离传感器）
  void MoveToDistance(double v, double val, std::string DistanceChioce);

  // 直线运动函数(使用距离传感器)
  void MoveToDistance(double v, double val, std::string DistanceChioce, bool task);

  // 直线运动函数（使用定位轮）
  void MoveBySmallWheel(double v, double val, double yaw, char direction);

  // 直线运动函数（使用定位轮）
  void MoveBySmallWheel(double v, double val, double yaw, char direction, bool task);

  // 直线运动函数（使用定位轮）
  void MoveBySmallWheel(double v, double val, char direction);

  // 直线运动函数（使用定位轮）
  void MoveBySmallWheel(double v, double val, char direction, bool task);

  // 自动选择pps旋转
  void AutoPPSTurn(double v, double val, char direction);

  // 自动选择pps转至
  void AutoPPSTurnTo(double v, double val);

  // 旋转函数
  void Turn(double v, double val, char direction);

  // 旋转函数(可选多线程)
  void Turn(double v, double val, char direction, bool task);

  // 旋转到指定角度
  void TurnTo(double v, double val, bool task);

  // 追随某个目标
  void FollowTarget(double v, double val);

  // 追随某个目标(多线程)
  void FollowTarget(double v, double val, bool task);

  // 旋转到指定对象
  void TurnToTarget(double v, double val, char direction);

  // 等待底盘运动开始
  void WaitMoveStart();

  // 等待底盘运动完成
  void WaitMoveFinish();

  // 获取当前是否在运动
  bool IsMoving();

  // 立即停止当前运动
  void StopMoving();

  // 使用坐标-移动到某一坐标位置
  void MoveTo(double x, double y, double v, bool task);


  public: 
    std::string StopMod = "Hold";    // 设置停止方式

    int ChassisSpeedConfig = 0;    // 底盘速度

    double  Move_time_out   = 3000;  // 超时保护，默认3000毫秒
    double  Stuck_time_out  = 300;   // 电机卡死保护，默认1000毫秒
    double  Stuck_threshold = 10;    // 电机卡死认定阈值，单位dps
    double  Move_max_speed  = 0.0;   // 直行最大速度
    int     Over_count      = 10;    // 到位计数当前次数后，才会退出
    bool    Moveing         = true;  // 记录是否在运动
    bool    MoveEnd         = true;  // 记录运动是否完成
    bool    Whileing        = false; // 是否进入循环
    int     MoveId          = 0;     // 运动id记录（计数）
    double maxinum_speed_acceleration = 0.4;  // 最大加速度
    double maxinum_speed_acceleration_turn = 5;  // 最大加速度

    // 各类pps

    // 直行相关参数
    PPS     *StraightPid;
    std::vector<double> Move_PPS;
    std::vector<double> MoveBySmallWheel_PPS;
    double  min_move_speed       = 0;  // 直行电机的最小速度
    double  move_allowable_error = 10; // 直行允许误差

    // 旋转相关参数
    std::vector<double> Turn_PPS;
    double  min_turn_speed       = 0;   // 自转的最小速度
    double  turn_allowable_error = 1.5; // 旋转允许误差

    double StraightAngleErrorSum = 0.0;
    double  StraightAngle   = 0.0;   // 记录直行的角度
    double  StraightNegative = 1.05;    // 记录执行方向

    int Target_CenterX       = 0;    // 目标物体的中心x
    int Target_CenterY       = 0;    // 目标物体的中心y
    int Target_Width         = 0;    // 目标物体的宽度
    int Target_Height        = 0;    // 目标物体的高度
    
    public: 
    // AiTarget Target = AiTarget();
};

//  多线程运动友元函数
int MoveTask(void* args);
//  多线程运动（直线定位轮）友元函数
int MoveBySmallWheelTask(void* args);
//  多线程旋转友元函数
int TurnTask(void* args);
//  多线程目标追踪友元函数
int FollowTargetTask(void* args);
//  多线程距离传感器友元函数 
int MoveToDistancetTask(void* args);
// 多线程曲线友元函数
int CurveMoveTask(void* args);

// 多线程准备的结构体函数
struct ChassisMoveParams{
    ChassisControl* self;
    double v = 0.0;        // 速度
    double val = 0.0;      // 距离
    double yaw = 0.0;      // 角度
    char direction;  // 运动方向
};

// 目标追踪结构体函数
struct FollowTargetParams{
    ChassisControl* self;
    double   v;    // 速度
    double   val;  // 加速度
};

// 多线程距离传感器结构体函数
struct MoveToDistanceParams{
    ChassisControl* self;
    double v;
    double val;
    std::string DistanceChioce;
};

// 曲线运动结构体函数
struct CurveMoveParams{
    ChassisControl* self;
    double v;
    double r;
    double TargetPosition;
    char direction;
};

extern ChassisControl ChassisControlSTS;  // sts底盘控制


#endif