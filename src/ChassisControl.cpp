#include <iostream>
#include <chrono>
#include <algorithm> // 包含 min 和 max 函数
#include <thread>
#include <vector>
#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "Positioning.h"
#include "units.h"

double GetDistance_front()
{
  double value1 = Distance_f.objectDistance(mm);
  return value1;
}

double GetDistance_back()
{
  double value1 = Distance_b.objectDistance(mm);
  return value1;
}

double GetDistance_left()
{
  return Distance_l.objectDistance(mm);
}

double GetDistance_right()
{
  return Distance_r.objectDistance(mm);
}

double GetDistance(std::string aspect)
{
  if (aspect == "front")
  {
    return GetDistance_front();
  }

  if (aspect == "back")
  {
    return GetDistance_back();
  }

  if (aspect == "left")
  {
    return GetDistance_left();
  }

  if (aspect == "right")
  {
    return GetDistance_right();
  }
  return 0.0;
}

double GetDistance(char aspect)
{
  if (aspect == 'f')
  {
    return GetDistance_front();
  }

  if (aspect == 'b')
  {
    return GetDistance_back();
  }

  if (aspect == 'l')
  {
    return GetDistance_left();
  }

  if (aspect == 'r')
  {
    return GetDistance_right();
  }
  return 0.0;
}

ChassisControl ChassisControlSTS = ChassisControl(); // sts底盘控制

// 底盘运动控制算法==================================
ChassisControl::ChassisControl()
{
  // pps参数设置相关
  ChassisSpeedConfig = 450;
  ResetPPS();
}

// 重新设置相关数据
void ChassisControl::ResetPPS()
{
  // 直线修正
  std::map<int, std::vector<double>> Straight_PPS_Map = {{343, {7.5, 0.0, 0.1}}, {450, {3, 0.0, 0.1}}, {300, {7.5, 0.0, 0.1}}};

  // 编码器
  std::map<int, std::vector<double>> Move_PPS_Map = {{343, {0.00950, 0.800, 2}}, {450, {0.0082, 0.5, 2}}, {300, {0.02, 0.800, 2}}};
  std::map<int, double> Min_Move_Speed_Map = {{343, 0.75}, {450, 0.75}, {300, 0.95}};

  // 距离传感器
  std::map<int, std::vector<double>> Distance_PPS_Map = {{343, {0.0165, 0.3, 2.5}}, {450, {0.0165, 0.3, 2.5}}, {300, {0.0165, 0.3, 2.5}}};

  // 陀螺仪
  std::map<int, std::vector<double>> Turn_PPS_Map = {{343, {0.075, 0, 3}}, {450, {0.025, 0.5, 5.5}}, {300, {0.042, 0.5, 5}}};
  std::map<int, double> Min_Turn_Speed_Map = {{343, 1.10}, {450, 1}, {300, 1.5}};

  min_move_speed = Min_Move_Speed_Map[ChassisSpeedConfig]; // 直行电机的最小速度
  min_turn_speed = Min_Turn_Speed_Map[ChassisSpeedConfig]; // 自转的最小速度

  StraightPid = new PPS(Straight_PPS_Map[ChassisSpeedConfig][0], Straight_PPS_Map[ChassisSpeedConfig][1], Straight_PPS_Map[ChassisSpeedConfig][2]);
  Move_PPS = Move_PPS_Map[ChassisSpeedConfig];
  Turn_PPS = Turn_PPS_Map[ChassisSpeedConfig];
  Distance_PPS = Distance_PPS_Map[ChassisSpeedConfig];
}

// 重新设置各个PPS
void Reset_Move_PPS(double p1, double p2, double s)
{
}
void Reset_Turn_PPS(double p1, double p2, double s)
{
}

// 重新设置当前最大速度
void ChassisControl::ResetMaxSpeed(double now_speed)
{
  this->speed = now_speed;
  this->Move_max_speed = now_speed;
}

// 重新设置当前直行角度
void ChassisControl::ResetStraightAngle(double yaw)
{
  this->StraightAngle = yaw;
}

// 底盘重新清零
void ChassisControl::Reset()
{
  ChassisInertial.resetRotation(); // 陀螺仪归零
  Moveing = true;                  // 记录是否在运动
  MoveEnd = true;                  // 记录运动是否完成

  // Target.Target_CenterX = -1;
  // Target.Target_CenterY = -1;
  // Target.Target_Width   = -1;
  // Target.Target_Height = -1;
}

// 底盘停止（抱死）
void ChassisControl::ChassisStoppingHold()
{
  LeftMotor1.spin(vex::forward, 0, dps);
  LeftMotor2.spin(vex::forward, 0, dps);
  LeftMotor3.spin(vex::forward, 0, dps);
  RightMotor1.spin(vex::forward, 0, dps);
  RightMotor2.spin(vex::forward, 0, dps);
  RightMotor3.spin(vex::forward, 0, dps);
  LeftMotor1.setStopping(hold);
  LeftMotor2.setStopping(hold);
  LeftMotor3.setStopping(hold);
  RightMotor1.setStopping(hold);
  RightMotor2.setStopping(hold);
  RightMotor3.setStopping(hold);
}

// 底盘停止（滑行）
void ChassisControl::ChassisStoppingCoast()
{
  LeftMotor1.spin(vex::forward, 0, dps);
  LeftMotor2.spin(vex::forward, 0, dps);
  LeftMotor3.spin(vex::forward, 0, dps);
  RightMotor1.spin(vex::forward, 0, dps);
  RightMotor2.spin(vex::forward, 0, dps);
  RightMotor3.spin(vex::forward, 0, dps);
  LeftMotor1.stop(coast);
  RightMotor1.stop(coast);
  LeftMotor2.stop(coast);
  RightMotor2.stop(coast);
  LeftMotor3.stop(coast);
  RightMotor3.stop(coast);
}

// 底盘停止
void ChassisControl::ChassisStop()
{
  if (this->StopMod == "Hold")
  {
    this->ChassisControl::ChassisStoppingHold();
  }
  else
  {
    this->ChassisControl::ChassisStoppingCoast();
  }
}

// 设置停止方式
void ChassisControl::SetStopMod(std::string str)
{
  this->StopMod = str;
}

// 设置停止模式为hold
void ChassisControl::SetStopModHold()
{
  this->SetStopMod("Hold");
}

// 设置停止模式为Coasst
void ChassisControl::SetStopModCoast()
{
  this->SetStopMod("Coast");
}

// 获得缓冲后的速度
double ChassisControl::GetBufferSpeed(double nowspeed, double output, int negative)
{
  if (negative == 1)
  {
    if (output > nowspeed)
    {
      nowspeed = std::min(output, nowspeed + maxinum_speed_acceleration);
    }
    else
    {
      nowspeed = output;
    }
  }
  else
  {
    if (output < nowspeed)
    {
      nowspeed = std::max(output, nowspeed - maxinum_speed_acceleration);
    }
    else
    {
      nowspeed = output;
    }
  }
  // nowspeed = output;
  return nowspeed;
}

// 获得缓冲后的速度
double ChassisControl::GetBufferSpeed_turn(double nowspeed, double output, int negative)
{
  if (negative == 1)
  {
    if (output > nowspeed)
    {
      nowspeed = std::min(output, nowspeed + maxinum_speed_acceleration_turn);
    }
    else
    {
      nowspeed = output;
    }
  }
  else
  {
    if (output < nowspeed)
    {
      nowspeed = std::max(output, nowspeed - maxinum_speed_acceleration_turn);
    }
    else
    {
      nowspeed = output;
    }
  }
  // nowspeed = output;
  return nowspeed;
}

// 底盘直行初始化
void ChassisControl::ChassisStraight(double angle)
{
  // angle = NormsAngle_0_360(angle);
  this->StraightPid->reset(0.1, 0.0, 0);
  StraightAngleErrorSum = 0.0;
  this->StraightAngle = angle;
  this->StraightPid->setSetpoint(angle);
  this->StraightNegative = 1;
}

// 更新底盘直行对准角度
void ChassisControl::ChassisForwardAngle(double angle)
{
  this->StraightPid->reset(0.1, 0.0, 0);
  StraightAngleErrorSum = 0.0;
  this->StraightAngle = angle;
  this->StraightPid->setSetpoint(angle);
  this->StraightNegative = 1;
}

// 底盘前进
void ChassisControl::ChassisForward(double v)
{
  double now_rotation = ChassisInertial.rotation();
  StraightAngleErrorSum += (now_rotation - this->StraightAngle);

  double output = this->StraightPid->calculate(now_rotation, 0.01) * -1; // 补偿
  // Brain.Screen.printAt( 10, 100, "output %f", output);
  // Brain.Screen.printAt( 10, 150, "this->StraightAngle %f", this->StraightPid->getsetpoint());
  if (output < 0)
  {
    output = std::max(output, -1 * StraightCorrectionSpeed);
  }
  else
  {
    output = std::min(output, 1 * StraightCorrectionSpeed);
  }
  // std::cout << "s-> " << now_rotation << " " << output << std::endl;

  LeftMotor1.spin(vex::forward, v - (output * this->StraightNegative), voltageUnits::volt);
  LeftMotor2.spin(vex::forward, v - (output * this->StraightNegative), voltageUnits::volt);
  LeftMotor3.spin(vex::forward, v - (output * this->StraightNegative), voltageUnits::volt);
  RightMotor1.spin(vex::forward, v + (output * this->StraightNegative), voltageUnits::volt);
  RightMotor2.spin(vex::forward, v + (output * this->StraightNegative), voltageUnits::volt);
  RightMotor3.spin(vex::forward, v + (output * this->StraightNegative), voltageUnits::volt);
}

// 底盘左轮控制
void ChassisControl::ChassisLeftControl(double v)
{
  LeftMotor1.spin(vex::forward, v, voltageUnits::volt);
  LeftMotor2.spin(vex::forward, v, voltageUnits::volt);
  LeftMotor3.spin(vex::forward, v, voltageUnits::volt);
}

// 底盘左轮控制
void ChassisControl::ChassisRightControl(double v)
{
  RightMotor1.spin(vex::forward, v, voltageUnits::volt);
  RightMotor2.spin(vex::forward, v, voltageUnits::volt);
  RightMotor3.spin(vex::forward, v, voltageUnits::volt);
}

// 底盘左轮控制-百分比
void ChassisControl::ChassisLeftControl_pct(double v)
{
  v *= 6.25;
  LeftMotor1.spin(vex::forward, v, pct);
  LeftMotor2.spin(vex::forward, v, pct);
  LeftMotor3.spin(vex::forward, v, pct);
}

// 底盘左轮控制-百分比
void ChassisControl::ChassisRightControl_pct(double v)
{
  v *= 6.25;
  RightMotor1.spin(vex::forward, v, pct);
  RightMotor2.spin(vex::forward, v, pct);
  RightMotor3.spin(vex::forward, v, pct);
}

// 等待电机编码器到达某个值
void ChassisControl::WaitPositionArrive(double val)
{

  int now_p;

  while (this->IsMoving())
  {
    now_p = this->GetPositions_mean();

    if (now_p < 0)
    {
      if (-1 * now_p > val)
      {
        break;
      }
    }
    else
    {
      if (now_p > val)
      {
        break;
      }
    }

    wait(10, msec);
  }
}

// 获取当前的位置
double ChassisControl::GetPositions_mean()
{
  double ans = 0.0;

  ans += medianOfThree(LeftMotor1.position(vex::rotationUnits::deg), LeftMotor2.position(vex::rotationUnits::deg), LeftMotor3.position(vex::rotationUnits::deg));
  ans += medianOfThree(RightMotor1.position(vex::rotationUnits::deg), RightMotor2.position(vex::rotationUnits::deg), RightMotor3.position(vex::rotationUnits::deg));

  return ans / 2;
}

// 获取当前的速度
double ChassisControl::GetVelocity_mean()
{
  double ans = 0.0;

  ans += std::abs(medianOfThree(LeftMotor1.velocity(dps), LeftMotor2.velocity(dps), LeftMotor3.velocity(dps)));
  ans += std::abs(medianOfThree(RightMotor1.velocity(dps), RightMotor2.velocity(dps), RightMotor3.velocity(dps)));

  return ans / 2;
}

// 获取当前的速度pct
double ChassisControl::GetVelocity_pct_mean()
{
  double ans = 0.0;

  ans += std::abs(medianOfThree(LeftMotor1.velocity(pct), LeftMotor2.velocity(pct), LeftMotor3.velocity(pct)));
  ans += std::abs(medianOfThree(RightMotor1.velocity(pct), RightMotor2.velocity(pct), RightMotor3.velocity(pct)));

  return ans / 2;
}

// 设置电机归位
void ChassisControl::ChassisRsetPosition()
{
  LeftMotor1.resetPosition();
  RightMotor1.resetPosition();
  LeftMotor2.resetPosition();
  RightMotor2.resetPosition();
  LeftMotor3.resetPosition();
  RightMotor3.resetPosition();
}

// 运动前
void ChassisControl::MoveStart()
{
}

// 运动后
void ChassisControl::MoveOver()
{
  ResetPPS();
  ChassisStop();
  Moveing = false;  // 运动停止了
  MoveEnd = true;   // 运动完成
  Whileing = false; // 循环结束了
}

// ================================================================================
// 获取运动当前值
double ChassisControl::GetNowCurrentPosition()
{
  if (Now_Mod == EncoderWheel)
  {
    return GetPositions_mean();
  }
  else if (Now_Mod == CurveMove_Position)
  {
    return GetPositions_mean();
  }
  else if (Now_Mod == Distance)
  {
    return GetDistance(this->DistanceChioce);
  }
  return 0;
}

// 设置为编码轮直走
// 距离，角度，方向
void ChassisControl::ReSetToEncoderWheel(double val_, double yaw_, char direction)
{
  ChassisRsetPosition();                                              // 电机初始化
  Now_Mod = EncoderWheel;                                             // 更改当前配置
  pps = PPS(this->Move_PPS[0], this->Move_PPS[1], this->Move_PPS[2]); // PPS切换
  this->yaw = yaw_;
  this->val = val_;
  this->ChassisForwardAngle(yaw);

  //  方向指示器
  if (direction == 'u')
  {
    negative = -1;
    this->val *= -1;
  }
  else if (direction == 'n')
  {
    negative = 1;
  }
  pps.setSetpoint(this->val);
}

// 设置为编码轮直走
// 距离，方向
void ChassisControl::ReSetToEncoderWheel(double val_, char direction)
{
  ChassisRsetPosition();                                              // 电机初始化
  Now_Mod = EncoderWheel;                                             // 更改当前配置
  pps = PPS(this->Move_PPS[0], this->Move_PPS[1], this->Move_PPS[2]); // PPS切换
  this->ChassisForwardAngle(ChassisInertial.rotation());
  this->val = val_;

  //  方向指示器
  if (direction == 'u')
  {
    negative = -1;
    this->val *= -1;
  }
  else if (direction == 'n')
  {
    negative = 1;
  }
  pps.setSetpoint(this->val);
}

// 设置距离传感器
void ChassisControl::ReSetToDistance(double val_, char DistanceChioce_)
{
  ChassisRsetPosition(); // 电机初始化
  this->Now_Mod = Distance;
  this->val = val_;
  this->DistanceChioce = DistanceChioce_;
  this->Distance_start_val = GetDistance(DistanceChioce_);

  //  方向指示器
  if (DistanceChioce == 'f')
  {
    negative = -1;
  }
  else if (DistanceChioce == 'b')
  {
    negative = 1;
  }

  pps = PPS(this->Distance_PPS[0], this->Distance_PPS[1], this->Distance_PPS[2]);
  pps.setSetpoint(val - Distance_start_val); // 设置目标位置
}

// 设置曲线运动(位置)
void ChassisControl::ReSetToCurveMove_Position(double r_, double TargetPosition_, char direction)
{
  ChassisRsetPosition(); // 电机初始化
  Now_Mod = CurveMove_Position;
  pps = PPS(this->Move_PPS[0], this->Move_PPS[1], this->Move_PPS[2]); // PPS切换
  //  方向指示器
  if (direction == 'p')
  {
    negative = 1;
    yaw_negative = 1;
  }
  else if (direction == 'q')
  {
    negative = 1;
    yaw_negative = -1;
  }
  else if (direction == 'b')
  {
    negative = -1;
    yaw_negative = 1;
    TargetPosition_ *= -1;
  }
  else if (direction == 'd')
  {
    negative = -1;
    yaw_negative = -1;
    TargetPosition_ *= -1;
  }
  this->r = r_;
  this->val = TargetPosition_;
  pps.setSetpoint(TargetPosition_);
  start_angle = ChassisInertial.rotation(); // 运动开始角度
}

// 设置曲线运动（角度）
void ChassisControl::ReSetToCurveMove_Angle(double r_, double TargetAngle_, char direction)
{
}

// 获得输出
double ChassisControl::GetOutPut()
{
  if (Now_Mod == EncoderWheel)
  {
    return EncoderWheel_OutPut();
  }
  else if (Now_Mod == CurveMove_Position)
  {
    return CurveMove_Position_OutPut();
  }
  else if (Now_Mod == Distance)
  {
    return Distance_OutPut();
  }
  return 0;
}

// 编码轮运动
double ChassisControl::EncoderWheel_OutPut()
{
  double current_position = GetPositions_mean(); // 当前位置
  // 直接返回 pps返回值即可
  return pps.calculate(current_position, 0.01);
}

//  曲线运动 位置
double ChassisControl::CurveMove_Position_OutPut()
{
  double current_position = GetPositions_mean(); // 当前位置
  return pps.calculate(current_position, 0.01);
}

//  曲线运动 角度
double ChassisControl::CurveMove_Angle_OutPut()
{

  return 0;
}

double ChassisControl::Distance_OutPut()
{
  double current_position = GetDistance(this->DistanceChioce) - this->Distance_start_val; // 当前位置
  double output = pps.calculate(current_position, 0.01) * negative;
  return output;
}

void ChassisControl::ChassisBasicMove(double now_speed)
{
  if (Now_Mod == EncoderWheel)
  {
    EncoderWheel_Move(now_speed);
  }
  else if (Now_Mod == CurveMove_Position)
  {
    CurveMove_Position_Move(now_speed);
  }
  else if (Now_Mod == Distance)
  {
    Distance_Move(now_speed);
  }
}

// 编码轮运动
void ChassisControl::EncoderWheel_Move(double now_speed)
{
  ChassisForward(now_speed); //  控制移动
}

//  曲线运动 位置
void ChassisControl::CurveMove_Position_Move(double now_speed)
{
  double PI = 3.1415;
  double current_position = GetPositions_mean();
  double current_angle = ChassisInertial.rotation(); // 当前位置

  // 计算当前理论陀螺仪角度
  double true_angle = start_angle + current_position / (2 * PI * r) * 360 * yaw_negative;

  double angle_compensation = (true_angle - current_angle);

  ChassisLeftControl(now_speed * (1 + 200 / (2 * r) * yaw_negative) + angle_compensation * 0.1);
  ChassisRightControl(now_speed * (1 - 200 / (2 * r) * yaw_negative) - angle_compensation * 0.1);
}

//  曲线运动 位置
void ChassisControl::CurveMove_Angle_Move(double now_speed)
{
}

// 距离传感器运动
void ChassisControl::Distance_Move(double now_speed)
{
  ChassisForward(now_speed); //  控制移动
}

// 基本的运动
void ChassisControl::BasicMove()
{
  ChassisRsetPosition(); // 电机初始化

  double dt = 0.01;
  double stuck_time_count = 0;                                                                                    // 电机卡死时间统计
  double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); // 运动开始时间
  double move_time_out = Move_time_out;
  int negative = 1; // 负方向指示器

  // 加速度测试
  double now_speed = 0;

  this->Move_max_speed = this->speed; // 获取最大速度

  // 直线修正指示器
  this->StraightAngle = this->yaw;
  ChassisControl::ChassisStraight(this->StraightAngle);

  // 多线程阻断等待
  while (MoveEnd == false)
  {
    wait(10, msec);
    Moveing = false;
  }

  Moveing = true;    // 当前在运动
  MoveEnd = false;   // 运动没有完成
  int overcount = 0; // 成功到位计数器
  int nearcount = 0; // 贴近计数
  double now_min_speed = min_move_speed;
  double last_speed = 0;
  bool add_over = false;

  while (true)
  {
    Whileing = true;                                   // 表示正在循环了
    double current_position = GetNowCurrentPosition(); // 当前位置
    // Brain.Screen.printAt( 10, 50, "current_position %f", current_position);
    this->ChassisForwardAngle(this->StraightAngle);

    double output = this->GetOutPut(); // 补偿
    // std::cout << output << " " << val << std::endl;

    if (std::abs(current_position - val) <= move_allowable_error && std::abs(output) <= min_move_speed * 10)
    {
      overcount += 1;
      if (overcount == Over_count)
      {
        // PrintAt(2, " ->到位");
        break;
      }
    }
    else
    {
      overcount = 0;
    }

    // 贴近
    if ((output < 0 && last_speed > 0) || (output > 0 && last_speed < 0))
    {
      nearcount += 1;
      if (nearcount >= 2)
      {
        now_min_speed -= 0.1;
      }
    }

    // 多线程保护
    if (Moveing == false)
    {
      // PrintAt(2, " ->打断");
      break;
    }

    // 电机抱死保护
    if (GetVelocity_mean() <= Stuck_threshold)
    {
      stuck_time_count += dt * 1000.0; // 计算抱死时间
      if (stuck_time_count >= Stuck_time_out)
      {
        // PrintAt(2, " ->抱死");
        break;
      }
      now_min_speed += 0.1;
    }
    else
    {
      // stuck_time_count = 0; // 不抱死则刷新计数
    }

    // 超时保护
    double now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); // 当前时间
    if (now_time - start_time >= move_time_out / 1000)
    {
      // PrintAt(2, " ->超时");
      break;
    }

    // 加速度设置
    if (!add_over)
    {
      now_speed = GetBufferSpeed(now_speed, output, negative);
      if (std::abs(now_speed) < std::abs(last_speed))
      {
        add_over = true;
      }
    }
    else
    {
      now_speed = output;
    }

    last_speed = now_speed;

    std::cout << now_speed << " " << current_position << std::endl;
    now_speed = NumInterval(now_speed, now_min_speed, this->Move_max_speed);
    this->ChassisBasicMove(now_speed);

    wait(dt * 1000.0, msec);
  }
  MoveOver();
}

// ==============================================================================
// 获得当前的pps算法输出（编码轮-直走）
// double get

// 直线运动函数
void ChassisControl::Move(double v, double val, char direction)
{
  this->Move(v, val, ChassisInertial.rotation(), direction);
}

// 直线运动函数（可选多线程）
void ChassisControl::Move(double v, double val, char direction, bool task)
{
  this->Move(v, val, ChassisInertial.rotation(), direction, task);
}

// 直线运动函数
void ChassisControl::Move(double v, double val, double yaw, char direction)
{
  this->ResetMaxSpeed(v);
  this->ReSetToEncoderWheel(val, yaw, direction);
  this->BasicMove();
}

// 直线运动函数(可选多线程)
void ChassisControl::Move(double v, double val, double yaw, char direction, bool task)
{
  if (task == true)
  {
    ChassisMoveParams *Params = new ChassisMoveParams();
    Params->self = this;
    Params->v = v;
    Params->val = val;
    Params->yaw = yaw;
    Params->direction = direction;

    void *p = Params;
    vex::task Task(MoveTask, p);
    this->WaitMoveStart();
  }
  else
  {
    Move(v, val, yaw, direction);
  }
}

// 曲线移动
void ChassisControl::CurveMove(double v, double r, double TargetPosition, char direction)
{
  this->ResetMaxSpeed(v);
  this->ReSetToCurveMove_Position(r, TargetPosition, direction);
  this->BasicMove();
}

// 曲线移动可选多线程
void ChassisControl::CurveMove(double v, double r, double TargetPosition, char direction, bool task)
{
  if (task == true)
  {
    CurveMoveParams *Params = new CurveMoveParams();
    Params->self = this;
    Params->v = v;
    Params->r = r;
    Params->TargetPosition = TargetPosition;
    Params->direction = direction;

    void *p = Params;
    vex::task Task(CurveMoveTask, p);
    this->WaitMoveStart();
  }
  else
  {
    CurveMove(v, r, TargetPosition, direction);
  }
}

// 旋转函数
void ChassisControl::Turn(double v, double val, char direction)
{
  // 最大速度
  // 运动距离
  // 方向
  // ChassisInertial.resetRotation();  // 陀螺仪初始化

  // PrintAt(1, "Turn %f", val);
  // PrintAt(2, "Moving...");
  v = std::abs(v);
  val = NormsAngle_0_360(val);

  double taget_angle = 0.0;
  double start_angle = ChassisInertial.rotation();
  double dt = 0.01;
  double stuck_time_count = 0;                                                                                    // 电机卡死时间统计
  double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); // 运动开始时间
  double move_time_out = Move_time_out + (val / v * 10) * 1000;
  int yaw_negative = 1; // 负偏指示器

  //  方向指示器
  if (direction == 'q' || direction == 'b' || direction == 'l')
  {
    yaw_negative = -1;
    taget_angle = start_angle - val;
  }
  else if (direction == 'p' || direction == 'd' || direction == 'r')
  {
    yaw_negative = 1;
    taget_angle = start_angle + val;
  }

  // Brain.Screen.printAt( 10, 50, "taget_angle %f", taget_angle);

  PPS pps(this->Turn_PPS[0], this->Turn_PPS[1], this->Turn_PPS[2]); // 创建PPS控制器实例

  pps.setSetpoint(taget_angle - start_angle); // 设置目标位置

  // 多线程阻断等待
  while (MoveEnd == false)
  {
    wait(10, msec);
    Moveing = false;
  }

  Moveing = true;    // 当前在运动
  MoveEnd = false;   // 运动没有完成
  int overcount = 0; // 到位计数
  int nearcount = 0; // 贴近计数
  double now_min_speed = min_turn_speed;
  double last_speed = 0;
  bool add_over = false;
  double blockage_num = 0; // 阻塞补偿

  while (true)
  {
    Whileing = true;
    double current_position = ChassisInertial.rotation();                             // 当前位置
    double output = pps.calculate(current_position - start_angle, dt) * yaw_negative; // 补偿

    // 检查目标是否达到
    // std::cout <<"turn" << current_position - start_angle << " " << output << std::endl;
    // std::cout << "overcount" << overcount << std::endl;

    // 到位
    if (std::abs(current_position - taget_angle) <= turn_allowable_error && std::abs(output) < min_turn_speed)
    {
      overcount += 1;
      if (overcount == Over_count)
      {
        // PrintAt(2, " ->到位");
        // std::cout <<"daowei" <<std::endl;
        break;
      }
    }
    else
    {
      overcount = 0;
    }

    // 贴近
    if ((output < 0 && last_speed > 0) || (output > 0 && last_speed < 0))
    {
      // Brain.Screen.printAt(10, 80, "----%d-------", nearcount);
      nearcount += 1;
      if (nearcount >= 2)
      {
        now_min_speed -= 0.45;
        blockage_num = 0;
      }
    }

    // 过慢保护
    if (GetVelocity_pct_mean() <= 3)
    {
      blockage_num += std::abs(3 - GetVelocity_pct_mean()) * 0.2;
    }

    if (nearcount <= 0)
    {
      if (output > 0)
      {
        output += blockage_num;
      }
      else
      {
        output -= blockage_num;
      }
    }

    // 多线程保护
    if (Moveing == false)
    {
      // PrintAt(2, " ->打断");
      break;
    }

    // 算法收敛保护
    // if (std::abs(output) <= Stuck_threshold / 2){
    //   PrintAt(2, " ->收敛");
    //   break;
    // }

    // 电机抱死保护
    if (GetVelocity_mean() <= Stuck_threshold)
    {
      stuck_time_count += dt * 1000.0; // 计算抱死时间
      if (stuck_time_count >= Stuck_time_out)
      {
        // PrintAt(2, " ->抱死");
        break;
      }

      // now_min_speed += 0.01;
    }
    else
    {
      stuck_time_count = 0; // 不抱死则刷新计数
    }

    // 超时保护
    double now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); // 当前时间
    if (now_time - start_time >= move_time_out / 1000)
    {
      // PrintAt(2, " ->超时");
      break;
    }

    output = NumInterval(output, now_min_speed, v);
    std::cout << output << " " << this->GetVelocity_pct_mean() << " " << current_position << "\n"
              << std::endl;

    // ChassisForward(output, yaw_negative);  //  控制移动
    ChassisLeftControl(output * yaw_negative);
    ChassisRightControl(-1 * output * yaw_negative);

    last_speed = output;
    wait(dt * 1000.0, msec);
  }

  MoveOver();
  // PrintAt(3, " %d", (int)ChassisInertial.rotation());
}

// 旋转函数(可选多线程)
void ChassisControl::Turn(double v, double val, char direction, bool task)
{
  if (task == true)
  {
    ChassisMoveParams *Params = new ChassisMoveParams();
    Params->self = this;
    Params->v = v;
    Params->val = val;
    Params->direction = direction;

    void *p = Params;
    vex::task Task(TurnTask, p);
    this->WaitMoveStart();
  }
  else
  {
    Turn(v, val, direction);
  }
}

// 自动选择pps旋转
void ChassisControl::AutoPPSTurn(double v, double val, char direction)
{
  double abs_val = std::abs(val);
  if (abs_val < 70)
  {
    ChassisControlSTS.Turn_PPS = {0.09, 0.5, 2};
    ChassisControlSTS.min_turn_speed = 1.5;
  }
  else if (abs_val < 150)
  {
    ChassisControlSTS.Turn_PPS = {0.0625, 0.5, 2};
    ChassisControlSTS.min_turn_speed = 1.5;
  }
  else
  {
    ChassisControlSTS.Turn_PPS = {0.055, 0.5, 2};
    ChassisControlSTS.min_turn_speed = 1;
  }
  this->Turn(v, val, direction);
}

// 自动选择pps转至
void ChassisControl::AutoPPSTurnTo(double v, double val)
{
  double start_angle = ChassisInertial.rotation();
  double start_angle_norm = NormsAngle(start_angle);

  if (start_angle_norm < 0)
  {
    start_angle_norm = 360 + start_angle_norm;
  }
  if (val < 0)
  {
    val = 360 + val;
  }
  // std::cout << start_angle_norm - val  << std::endl;

  double lval = start_angle_norm - val;
  double rval = 360 - lval;

  if (start_angle_norm > val)
  {
    lval = start_angle_norm - val;
    rval = 360 - lval;
  }
  else
  {
    rval = val - start_angle_norm;
    lval = 360 - rval;
  }

  // 自动参数选择s
  double abs_val = std::abs(std::min(rval, lval));
  if (abs_val < 70)
  {
    ChassisControlSTS.Turn_PPS = {0.5, 0.5, 2};
    ChassisControlSTS.min_turn_speed = 1;
  }
  else if (abs_val < 150)
  {
    ChassisControlSTS.Turn_PPS = {0.050, 0.5, 2};
    ChassisControlSTS.min_turn_speed = 1.5;
  }
  else
  {
    ChassisControlSTS.Turn_PPS = {0.050, 0.5, 2};
    ChassisControlSTS.min_turn_speed = 1;
  }

  if (lval > rval)
  {
    this->Turn(v, rval, 'r', false);
  }
  else
  {
    this->Turn(v, lval, 'l', false);
  }
}

// 旋转到指定角度
void ChassisControl::TurnTo(double v, double val, bool task)
{
  double start_angle = ChassisInertial.rotation();
  double start_angle_norm = NormsAngle(start_angle);

  if (start_angle_norm < 0)
  {
    start_angle_norm = 360 + start_angle_norm;
  }
  if (val < 0)
  {
    val = 360 + val;
  }
  // std::cout << start_angle_norm - val  << std::endl;

  double lval = start_angle_norm - val;
  double rval = 360 - lval;

  if (start_angle_norm > val)
  {
    lval = start_angle_norm - val;
    rval = 360 - lval;
  }
  else
  {
    rval = val - start_angle_norm;
    lval = 360 - rval;
  }

  if (lval > rval)
  {
    this->Turn(v, rval, 'r', task);
  }
  else
  {
    this->Turn(v, lval, 'l', task);
  }
}

// 等待底盘运动开始
void ChassisControl::WaitMoveStart()
{
  while (!this->Whileing)
  {
    wait(10, msec);
  }
}

// 等待底盘运动完成
void ChassisControl::WaitMoveFinish()
{
  while (this->IsMoving())
  {
    wait(10, msec);
  }
}

// 获取当前是否在运动
bool ChassisControl::IsMoving()
{
  return (Moveing == true) || (MoveEnd == false) || (Whileing == true);
}

// 立即停止当前运动(并等待)
void ChassisControl::StopMoving()
{
  // 多线程阻断等待
  while (this->MoveEnd == false)
  {
    wait(10, msec);
    this->Moveing = false;
  }
  this->WaitMoveFinish();
}

// 使用距离传感器移动
void ChassisControl::MoveToDistance(double v, double val, char DistanceChioce)
{
  this->ResetMaxSpeed(v);
  this->ReSetToDistance(val, DistanceChioce);
  this->BasicMove();
}

// 使用距离传感器移动(可选多线程)
void ChassisControl::MoveToDistance(double v, double val, char DistanceChioce, bool task)
{
  if (task == true)
  {
    MoveToDistanceParams *Params = new MoveToDistanceParams();
    Params->self = this;
    Params->v = v;
    Params->val = val;
    Params->DistanceChioce = DistanceChioce;

    void *p = Params;
    vex::task Task(MoveToDistancetTask, p);
  }
  else
  {
    MoveToDistance(v, val, DistanceChioce);
  }
}

// 直线运动函数（使用定位轮）
void ChassisControl::MoveBySmallWheel(double v, double val, double yaw, char direction)
{
  // 最大速度
  // 运动距离
  // 偏转
  // 方向
  // 多线程
  // ChassisRsetPosition();  // 电机初始化

  // PrintAt(1, "MoveBSW %f", val);
  // PrintAt(2, "Moving...");

  Rotation_x.resetPosition(); // 定位轮初始化

  double dt = 0.01;
  double stuck_time_count = 0;                                                                                    // 电机卡死时间统计
  double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); // 运动开始时间
  double move_time_out = Move_time_out + (val / v * 10);
  int negative = 1;     // 负方向指示器
  int yaw_negative = 1; // 负偏指示器

  // 加速度测试
  double now_speed = 0;
  // double maxinum_speed_acceleration = 50;

  //  方向指示器
  if (direction == 'u')
  {
    negative = -1;
    val *= -1;
    yaw_negative = 1;
    yaw = 0;
  }
  else if (direction == 'n')
  {
    negative = 1;
    yaw_negative = 1;
    yaw = 0;
  }
  else if (direction == 'p')
  {
    negative = 1;
    yaw_negative = 1;
  }
  else if (direction == 'q')
  {
    negative = 1;
    yaw_negative = -1;
  }
  else if (direction == 'b')
  {
    val *= -1;
    negative = -1;
    yaw_negative = 1;
  }
  else if (direction == 'd')
  {
    negative = -1;
    yaw_negative = -1;
    val *= -1;
  }

  // 直线修正指示器
  if (abs(yaw) < 0.01)
  {
    ChassisControl::ChassisStraight(ChassisInertial.rotation());
    // ( 10, 50, "start %f", ChassisInertial.rotation());
  }

  PPS pps(0.0085, 0.8, 3); // 创建PPS控制器实例
  pps.setSetpoint(val);    // 设置目标位置

  // 多线程阻断等待
  while (MoveEnd == false)
  {
    wait(10, msec);
    Moveing = false;
  }

  Moveing = true;    // 当前在运动
  MoveEnd = false;   // 运动没有完成
  int nearcount = 0; // 贴近计数
  double now_min_speed = min_move_speed;
  double last_speed = 0;

  int overcount = 0; // 成功到位计数器

  while (true)
  {
    double current_position = Rotation_x.position(deg);  // 当前位置
    double output = pps.calculate(current_position, dt); // 补偿

    // 检查目标是否达到
    if (std::abs(current_position - val) <= 7.5 && std::abs(output) <= Stuck_threshold)
    {
      overcount += 1;
      if (overcount == Over_count)
      {
        // PrintAt(2, " ->到位");
        // std::cout <<"daowei" <<std::endl;
        break;
      }
    }
    else
    {
      overcount = 0;
    }

    // 贴近
    if ((output < 0 && last_speed > 0) || (output > 0 && last_speed < 0))
    {
      // Brain.Screen.printAt(10, 80, "----%d-------", nearcount);
      nearcount += 1;
      if (nearcount >= 2)
      {
        now_min_speed -= 0.1;
      }
    }

    // 多线程保护
    if (Moveing == false)
    {
      // PrintAt(2, " ->打断");
      break;
    }

    // 算法收敛保护
    // if (std::abs(output) <= Stuck_threshold){
    //   PrintAt(2, " ->收敛");
    //   break;
    // }

    // 电机抱死保护
    if (GetVelocity_mean() <= Stuck_threshold)
    {
      stuck_time_count += dt * 1000.0; // 计算抱死时间
      if (stuck_time_count >= Stuck_time_out)
      {
        // PrintAt(2, " ->抱死");
        break;
      }
      now_min_speed += 0.1;
    }
    else
    {
      stuck_time_count = 0; // 不抱死则刷新计数
    }

    // 超时保护
    double now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); // 当前时间
    if (now_time - start_time >= move_time_out / 1000)
    {
      // PrintAt(2, " ->超时");
      break;
    }
    last_speed = output;
    now_speed = GetBufferSpeed(now_speed, output, negative);
    // std::cout << "output----" << output << std::endl;
    now_speed = NumInterval(now_speed, now_min_speed, v);
    ChassisForward(now_speed); //  控制移动

    wait(dt * 1000.0, msec);
  }

  ChassisStop();
  // PrintAt(3, "%f",  Rotation_x.position(deg));
  Moveing = false; // 运动停止了
  MoveEnd = true;  // 运动完成
}

// 直线运动函数（使用定位轮）
void ChassisControl::MoveBySmallWheel(double v, double val, double yaw, char direction, bool task)
{
  if (task == true)
  {
    ChassisMoveParams *Params = new ChassisMoveParams();
    Params->self = this;
    Params->v = v;
    Params->val = val;
    Params->yaw = yaw;
    Params->direction = direction;

    void *p = Params;
    vex::task Task(MoveBySmallWheelTask, p);
    wait(500, msec);
  }
  else
  {
    MoveBySmallWheel(v, val, yaw, direction);
  }
}

// 追随某个目标(多线程)
void ChassisControl::FollowTarget(double v, double val, bool task)
{
  if (task == true)
  {
    FollowTargetParams *Params = new FollowTargetParams();
    Params->self = this;
    Params->v = v;
    Params->val = val;
    void *p = Params;
    vex::task Task(FollowTargetTask, p);
    wait(500, msec);
  }
  else
  {
    FollowTarget(v, val);
  }
}

//  多线程运动（直线）友元函数
int MoveTask(void *args)
{
  ChassisMoveParams *params = (ChassisMoveParams *)args;
  ChassisControl *self = params->self; // 当前类
  double v = params->v;                // 速度
  double val = params->val;            // 路径长度
  char direction = params->direction;  // 运动方向
  double yaw = params->yaw;            // 偏转
  self->Move(v, val, yaw, direction);
  // delete args;
  return 0;
}

//  多线程运动（直线定位轮）友元函数
int MoveBySmallWheelTask(void *args)
{
  ChassisMoveParams *params = (ChassisMoveParams *)args;
  ChassisControl *self = params->self; // 当前类
  double v = params->v;                // 速度
  double val = params->val;            // 路径长度
  char direction = params->direction;  // 运动方向
  double yaw = params->yaw;            // 偏转
  self->MoveBySmallWheel(v, val, yaw, direction);

  // delete args;
  return 0;
}

//  多线程运动（旋转）友元函数
int TurnTask(void *args)
{
  ChassisMoveParams *params = (ChassisMoveParams *)args;
  ChassisControl *self = params->self; // 当前类
  double v = params->v;                // 速度
  double val = params->val;            // 路径长度
  char direction = params->direction;  // 运动方向
  self->Turn(v, val, direction);
  // delete args;
  return 0;
}

//  多线程目标追踪友元函数
// int FollowTargetTask(void* args){
//     FollowTargetParams* params = (FollowTargetParams*)args;
//     ChassisControl* self= params->self;       // 当前类
//     double v            = params->v;          // 速度
//     double val          = params->val;        // 路径长度

//     self->FollowTarget(v, val);

//     // delete args;
//     return 0;
// }

//  多线程距离传感器友元函数
int MoveToDistancetTask(void *args)
{
  MoveToDistanceParams *params = (MoveToDistanceParams *)args;
  ChassisControl *self = params->self;                 // 当前类
  double v = params->v;                                // 速度
  double val = params->val;                            // 路径长度
  char DistanceChioce = params->DistanceChioce; // 前后距离传感器选择

  self->MoveToDistance(v, val, DistanceChioce);

  // delete args;
  return 0;
}

// 多线程曲线友元函数
int CurveMoveTask(void *args)
{
  CurveMoveParams *params = (CurveMoveParams *)args;
  ChassisControl *self = params->self;            // 当前类
  double v = params->v;                           // 速度
  double r = params->r;                           // 半径
  double TargetPosition = params->TargetPosition; // 路径长度
  char direction = params->direction;             // 方向
  self->CurveMove(v, r, TargetPosition, direction);

  // delete args;
  return 0;
}
