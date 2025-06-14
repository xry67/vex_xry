#include <iostream>  
#include <chrono>  
#include <algorithm> // 包含 min 和 max 函数
#include <thread>  
#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "Positioning.h"
#include "units.h"

double GetDistance_front(){
  double value1 = Distance_fl.objectDistance(mm);
  double value2 = Distance_fr.objectDistance(mm);
  return (value1 + value2) / 2.0;
}

double GetDistance_back(){
  double value1 = Distance_bl.objectDistance(mm);
  double value2 = Distance_br.objectDistance(mm);
  return (value1 + value2) / 2.0;
}

double GetDistance_left(){
  return Distance_l.objectDistance(mm);
}

double GetDistance_right(){
  return Distance_r.objectDistance(mm);
}

double GetDistance(std::string aspect){
  if (aspect == "front"){
    return GetDistance_front();
  }

  if (aspect == "back"){
    return GetDistance_back();
  }
  
  if (aspect == "left"){
    return GetDistance_left();
  }
  
  if (aspect == "right"){
    return GetDistance_right();
  }
  return 0.0;
}


ChassisControl ChassisControlSTS = ChassisControl();  // sts底盘控制

// 底盘运动控制算法

  // 底盘重新清零
  void ChassisControl::Reset(){
      ChassisInertial.resetRotation();  // 陀螺仪归零
      Moveing         = true; // 记录是否在运动
      MoveEnd         = true;  // 记录运动是否完成

      Target.Target_CenterX = -1;
      Target.Target_CenterY = -1;
      Target.Target_Width   = -1;
      Target.Target_Height = -1;
  }

  // 底盘停止（抱死）
  void ChassisControl::ChassisStoppingHold(){
      LeftMotor1.spin(vex::forward,  0, dps);
      LeftMotor2.spin(vex::forward,  0, dps);
      LeftMotor3.spin(vex::forward,  0, dps);
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
  void ChassisControl::ChassisStoppingCoast(){
      LeftMotor1.spin(vex::forward,  0, dps);
      LeftMotor2.spin(vex::forward,  0, dps);
      LeftMotor3.spin(vex::forward,  0, dps);
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
  void ChassisControl::ChassisStop(){
    if (this->StopMod == "Hold"){
      this->ChassisControl::ChassisStoppingHold();
    }
    else{
      this->ChassisControl::ChassisStoppingCoast();
    }
  }

  // 设置停止方式
  void ChassisControl::SetStopMod(std::string str){
      this->StopMod = str;
  }

  // 设置停止模式为hold
  void ChassisControl::SetStopModHold(){
    this->SetStopMod("Hold");
  }
  // 设置停止模式为Coasst
  void ChassisControl::SetStopModCoast(){
    this->SetStopMod("Coast");
  }

  // 获得缓冲后的速度
  double ChassisControl::GetBufferSpeed(double nowspeed, double output, int negative){
    if (negative == 1){
      if (output > nowspeed){
        nowspeed = std::min(output, nowspeed + maxinum_speed_acceleration);
      }else{
        nowspeed = output;
      }
    }else{
      if (output < nowspeed){
        nowspeed = std::max(output, nowspeed - maxinum_speed_acceleration);
      }else{
        nowspeed = output;
      }
    }
    return nowspeed;
  }

  // 底盘直行初始化
  void ChassisControl::ChassisStraight(int negative = 1){
      this->StraightPid->reset(0.1, 0.0, 0);
      this->StraightAngle = ChassisInertial.rotation();
      this->StraightPid->setSetpoint(ChassisInertial.rotation());
      this->StraightNegative = negative;
  }

  // 底盘前进
  void ChassisControl::ChassisForward(double v, double yaw = 0){
      if (abs(yaw) < 0.1){
        double now_rotation = ChassisInertial.rotation();

        double output = this->StraightPid->calculate(now_rotation, 0.01) * -1;  // 补偿
        Brain.Screen.printAt( 10, 100, "output %f", output);
        Brain.Screen.printAt( 10, 150, "this->StraightAngle %f", this->StraightPid->getsetpoint());
        if (output < 0){
          output = std::max(output, -1 * StraightCorrectionSpeed);
        }else{
          output = std::min(output, 1 * StraightCorrectionSpeed);
        }
        std::cout << "s-> " << now_rotation << " " << output << std::endl;

        LeftMotor1.spin(vex::forward,  v - (output * this->StraightNegative), voltageUnits::volt);
        LeftMotor2.spin(vex::forward,  v - (output * this->StraightNegative), voltageUnits::volt);
        LeftMotor3.spin(vex::forward,  v - (output * this->StraightNegative), voltageUnits::volt);
        RightMotor1.spin(vex::forward, v + (output * this->StraightNegative), voltageUnits::volt);
        RightMotor2.spin(vex::forward, v + (output * this->StraightNegative), voltageUnits::volt);
        RightMotor3.spin(vex::forward, v + (output * this->StraightNegative), voltageUnits::volt);
        return ;
      }
      
      double left_yaw  = 1.0;
      double right_yaw = 1.0;

      // 左转
      if (yaw < 0){
          left_yaw = 2.0 * yaw + 1;
      }
      // 右转
      if (yaw > 0){
          right_yaw = -2.0 * yaw + 1;
      }


      LeftMotor1.spin(vex::forward, v * left_yaw, voltageUnits::volt);
      LeftMotor2.spin(vex::forward, v * left_yaw, voltageUnits::volt);
      LeftMotor3.spin(vex::forward, v * left_yaw, voltageUnits::volt);
      RightMotor1.spin(vex::forward, v * right_yaw, voltageUnits::volt);
      RightMotor2.spin(vex::forward, v * right_yaw, voltageUnits::volt);
      RightMotor3.spin(vex::forward, v * right_yaw, voltageUnits::volt);
  }

  // 底盘左轮控制
  void ChassisControl::ChassisLeftControl(double v){
    LeftMotor1.spin(vex::forward,  v, voltageUnits::volt);
    LeftMotor2.spin(vex::forward,  v, voltageUnits::volt);
    LeftMotor3.spin(vex::forward,  v, voltageUnits::volt);
  }

  // 底盘左轮控制
  void ChassisControl::ChassisRightControl(double v){
    RightMotor1.spin(vex::forward, v, voltageUnits::volt);
    RightMotor2.spin(vex::forward, v, voltageUnits::volt);
    RightMotor3.spin(vex::forward, v, voltageUnits::volt);
  }

  // 获取当前的位置
  double ChassisControl::GetPositions_mean(){
      double ans = 0.0;

      ans += LeftMotor1.position(vex::rotationUnits::deg);
      ans += LeftMotor2.position(vex::rotationUnits::deg);
      ans += LeftMotor3.position(vex::rotationUnits::deg);
      ans += RightMotor1.position(vex::rotationUnits::deg);
      ans += RightMotor2.position(vex::rotationUnits::deg);
      ans += RightMotor3.position(vex::rotationUnits::deg);
      return ans / 6;
  }

  // 获取当前的速度
  double ChassisControl::GetVelocity_mean(){
      double ans = 0.0;

      ans += std::abs(LeftMotor1.velocity(dps));
      ans += std::abs(LeftMotor2.velocity(dps));
      ans += std::abs(LeftMotor3.velocity(dps));
      ans += std::abs(RightMotor1.velocity(dps));
      ans += std::abs(RightMotor2.velocity(dps));
      ans += std::abs(RightMotor3.velocity(dps));
      return ans / 6;
  }

  // 设置电机归位 
  void ChassisControl::ChassisRsetPosition(){
      LeftMotor1.resetPosition();
      RightMotor1.resetPosition();
      LeftMotor2.resetPosition();
      RightMotor2.resetPosition();
      LeftMotor3.resetPosition();
      RightMotor3.resetPosition();
  }

  // 直线运动函数
  void ChassisControl::Move(double v, double val, double yaw, char direction){
      // 最大速度
      // 运动距离
      // 偏转
      // 方向
      // 多线程
      PrintAt(1, "Move %f", val);
      PrintAt(2, "Moving...");
      ChassisRsetPosition();  // 电机初始化
 
      double dt = 0.01;
      double stuck_time_count = 0;  // 电机卡死时间统计
      double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();  // 运动开始时间
      double move_time_out = Move_time_out + (val / v * 1000);
      int    negative = 1;   // 负方向指示器
      int yaw_negative= 1;   // 负偏指示器

      // 加速度测试
      double now_speed = 0;

      //  方向指示器
      if (direction == 'u'){
          negative = -1;
          yaw_negative = 1;
          yaw = 0;
          val *= -1;
      }
      else if (direction == 'n'){
          negative = 1;
          yaw_negative = 1;
          yaw = 0;
      }
      else if (direction == 'p'){
          negative = 1;
          yaw_negative = 1;
      }
      else if (direction == 'q'){
          negative = 1;
          yaw_negative = -1;
      }
      else if (direction == 'b'){
          negative = -1;
          yaw_negative = 1;
          val *= -1;
      }
      else if (direction == 'd'){
          negative = -1;
          yaw_negative = -1;
          val *= -1;
      }

      // 直线修正指示器
      if (abs(yaw) < 0.01){
        ChassisControl::ChassisStraight();
        // Brain.Screen.printAt( 10, 50, "start %f", ChassisInertial.rotation());
      }


      PID pid(0.00900, 0.0000, 0.000); // 创建PID控制器实例 
      pid.setSetpoint(val); // 设置目标位置

      // 多线程阻断等待
      while (MoveEnd == false){
          wait(10, msec);
          Moveing = false;
      }

      Moveing = true;  // 当前在运动
      MoveEnd = false; // 运动没有完成
      int overcount = 0;  // 成功到位计数器

      while (true) {  
          double current_position = GetPositions_mean();  // 当前位置
          // Brain.Screen.printAt( 10, 50, "current_position %f", current_position);
         
          double output = pid.calculate(current_position, dt);  // 补偿

          if (std::abs(current_position - val) <= 5 && std::abs(output) <= Stuck_threshold) {
            overcount += 1;
            if (overcount == Over_count){
              PrintAt(2, " ->到位");
              std::cout <<"daowei" <<std::endl;
              break;  
            }
          }else{
            overcount = 0;
          } 

          // 多线程保护
          if (Moveing == false){
              PrintAt(2, " ->打断");
              break;
          }

          // 算法收敛保护
          // if (std::abs(output) <= Stuck_threshold){
          //   PrintAt(2, " ->收敛");
          //   break;
          // }

          // 电机抱死保护
          if (GetVelocity_mean() <= Stuck_threshold){
              stuck_time_count += dt * 1000.0;  // 计算抱死时间
              if (stuck_time_count >= Stuck_time_out){
                PrintAt(2, " ->抱死");
                break;
              }
          }
          else{
            stuck_time_count = 0; // 不抱死则刷新计数
          }

          // 超时保护
          double now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();// 当前时间
          if (now_time - start_time >= move_time_out / 1000){
              PrintAt(2, " ->超时");
              break;
          }

          now_speed = GetBufferSpeed(now_speed, output, negative);
          // std::cout << "output----" << output << std::endl;
          now_speed = NumInterval(now_speed, min_move_speed, v);
          ChassisForward(now_speed, yaw * yaw_negative);  //  控制移动

          wait(dt * 1000.0, msec);
      }  

      ChassisStop();
      PrintAt(3, "%f",  GetPositions_mean());
      Moveing = false;  // 运动停止了
      MoveEnd = true; // 运动完成

      std::cout << "ans: " << GetPositions_mean() << std::endl;
  }

  // 直线运动函数(可选多线程)
  void ChassisControl::Move(double v, double val, double yaw, char direction, bool task){
    if (task == true){
        ChassisMoveParams* Params = new ChassisMoveParams();
        Params->self = this;
        Params->v = v;
        Params->val = val;
        Params->yaw = yaw;
        Params->direction = direction;

        void* p = Params;
        vex::task Task(MoveTask, p);
    }
    else{
        Move(v, val, yaw, direction);    
    }
  }

  // 旋转函数
  void ChassisControl::Turn(double v, double val, char direction){
      // 最大速度
      // 运动距离
      // 方向
      // ChassisInertial.resetRotation();  // 陀螺仪初始化

      PrintAt(1, "Turn %f", val);
      PrintAt(2, "Moving...");
  
      double taget_angle = 0.0;
      double start_angle   = ChassisInertial.rotation();
      double dt = 0.01;
      double stuck_time_count = 0;  // 电机卡死时间统计
      double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();  // 运动开始时间
      double move_time_out = Move_time_out + (val / v * 10) * 1000;
      int yaw_negative= 1;   // 负偏指示器

      //  方向指示器
      if (direction == 'q' || direction == 'b' || direction == 'l'){
          yaw_negative = -1;
          taget_angle  = start_angle - val;
      }
      else if (direction == 'p' || direction == 'd' || direction == 'r'){
          yaw_negative = 1;
          taget_angle  = start_angle + val;
      }

      // Brain.Screen.printAt( 10, 50, "taget_angle %f", taget_angle);

      PID pid(0.08, 0.0, 0); // 创建PID控制器实例 
      pid.setSetpoint(taget_angle); // 设置目标位置

      // 多线程阻断等待
      while (MoveEnd == false){
          wait(10, msec);
          Moveing = false;
      }

      Moveing = true;  // 当前在运动
      MoveEnd = false; // 运动没有完成
      int overcount = 0; // 到位计数
      int nearcount = 0; // 贴近计数
      double now_min_speed = min_turn_speed;
      double last_speed = 0;

      while (true) {  
          double current_position = ChassisInertial.rotation();  // 当前位置
          double output = pid.calculate(current_position, dt) * yaw_negative;  // 补偿

          // 检查目标是否达到  
          std::cout <<"turn" << std::abs(current_position - taget_angle) << " " << std::abs(output) << std::endl;

          // 到位
          if (std::abs(current_position - taget_angle) <= 1 && std::abs(output) < Stuck_threshold) {
            overcount += 1;
            if (overcount == Over_count){
              PrintAt(2, " ->到位");
              std::cout <<"daowei" <<std::endl;
              break;  
            }
          }  
          else{
            overcount = 0;
          }

          // 贴近
          if ((output < 0 && last_speed > 0) || (output > 0 && last_speed < 0)){
            Brain.Screen.printAt(10, 80, "----%d-------", nearcount);
            nearcount += 1;
            if (nearcount >= 2){
              now_min_speed -= 0.1;
            }
          }

          // 多线程保护
          if (Moveing == false){
              PrintAt(2, " ->打断");
              break;
          }

          // 算法收敛保护
          // if (std::abs(output) <= Stuck_threshold / 2){
          //   PrintAt(2, " ->收敛");
          //   break;
          // }

          // 电机抱死保护
          if (GetVelocity_mean() <= Stuck_threshold){
              stuck_time_count += dt * 1000.0;  // 计算抱死时间
              if (stuck_time_count >= Stuck_time_out){
                PrintAt(2, " ->抱死");
                break;
              }
              now_min_speed += 0.1;
          }
          else{
            stuck_time_count = 0; // 不抱死则刷新计数
          }

          // 超时保护
          double now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();// 当前时间
          if (now_time - start_time >= move_time_out / 1000){
              PrintAt(2, " ->超时");
              break;
          }

          output = NumInterval(output, now_min_speed, v);
          ChassisForward(output, yaw_negative);  //  控制移动
          last_speed = output;
          wait(dt * 1000.0, msec);
      }  

      ChassisStop();
      PrintAt(3, " %d", (int)ChassisInertial.rotation());
      Moveing = false;  // 运动停止了
      MoveEnd = true; // 运动完成

      
  }

  // 旋转函数(可选多线程)
  void ChassisControl::Turn(double v, double val, char direction, bool task){
      if (task == true){
          ChassisMoveParams* Params = new ChassisMoveParams();
          Params->self = this;
          Params->v = v;
          Params->val = val;
          Params->direction = direction;

          void* p = Params;
          vex::task Task(TurnTask, p);
      }
      else{
          Turn(v, val, direction);    
      }
  }

  // 旋转到指定角度
  void ChassisControl::TurnTo(double v, double val, bool task){
    double start_angle        = ChassisInertial.rotation();
    double start_angle_norm   = NormsAngle(start_angle);

    std::cout << start_angle_norm - val  << std::endl;

    if (val > start_angle_norm){
      this->Turn(v, val - start_angle_norm, 'r', task);
    }
    else{
      this->Turn(v, start_angle_norm - val, 'l', task);
    }

    
  }

  // 等待底盘运动完成
  void ChassisControl::WaitMoveFinish(){
      while(this->IsMoving()){
          wait(10, msec);
      }
  }

  // 获取当前是否在运动
  bool ChassisControl::IsMoving(){
      return (Moveing == true) || (MoveEnd == false);
  }

  // 立即停止当前运动(并等待)
  void ChassisControl::StopMoving(){
      // 多线程阻断等待
      while (this->MoveEnd == false){
          wait(10, msec);
          this->Moveing = false;
      }
      this->WaitMoveFinish();
  }

  // 使用坐标-移动到某一坐标位置
  void ChassisControl::MoveTo(double x, double y, double v, bool task){
    PositioningSTS.TurnOFF();
    double target_angle = PositioningSTS.GetAngle(x - PositioningSTS.x, y - PositioningSTS.y);
    std::cout << "xy:" << x - PositioningSTS.x << " " << y - PositioningSTS.y << std::endl;
    std::cout << "angle:" << target_angle << std::endl;

    this->TurnTo(v, target_angle, task);

    // wait(2000, msec);
    std::cout<< ChassisInertial.rotation(deg) << std::endl;
    PositioningSTS.TurnOn();

    //return ;

    double val = sqrt((x - PositioningSTS.x) * (x - PositioningSTS.x) + (y - PositioningSTS.y) * (y - PositioningSTS.y));
    char direction = 'n';
    double yaw = 0;

    // 最大速度
    // 运动距离
    // 偏转
    // 方向
    // 多线程
    Rotation_y.resetPosition();

    double dt = 0.01;
    double stuck_time_count = 0;  // 电机卡死时间统计
    double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();  // 运动开始时间
    double move_time_out = Move_time_out + (val / v * 1000);
    int    negative = 1;   // 负方向指示器
    int yaw_negative= 1;   // 负偏指示器

    //  方向指示器
    if (direction == 'u'){
        negative = -1;
        yaw_negative = 1;
        yaw = 0;
    }
    else if (direction == 'n'){
        negative = 1;
        yaw_negative = 1;
        yaw = 0;
    }


    PID pid(8, 0.03, 0.1); // 创建PID控制器实例 
    pid.setSetpoint(val); // 设置目标位置

    // 多线程阻断等待
    while (MoveEnd == false){
        wait(10, msec);
        Moveing = false;
    }

    Moveing = true;  // 当前在运动
    MoveEnd = false; // 运动没有完成

    while (true) {  
        double current_position = Rotation_y.position(deg);  // 当前位置
        double output = pid.calculate(current_position, dt);  // 补偿
        // 控制小车运动 
        ChassisForward(std::min(v, output) * negative, yaw * yaw_negative);   //  控制移动

        // 检查目标是否达到  
        if (fabs(current_position - val) <= 1) {
            break;  
        }  

        // 多线程保护
        if (Moveing == false){
            break;
        }

        // 算法收敛保护
        if (output <= Stuck_threshold){
          break;
        }

        // 电机抱死保护
        if (GetVelocity_mean() <= Stuck_threshold){
            stuck_time_count += dt * 1000.0;  // 计算抱死时间
            if (stuck_time_count >= Stuck_time_out){
              std::cout << "stuck_time_count" << std::endl;
              break;
            }
        }
        else{
          stuck_time_count = 0; // 不抱死则刷新计数
        }

        // 超时保护
        double now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();// 当前时间
        if (now_time - start_time >= move_time_out / 1000){
            std::cout << "timeout" << std::endl;
            break;
        }

        wait(dt * 1000.0, msec);
    }  

    ChassisStop();
    Moveing = false;  // 运动停止了
    MoveEnd = true; // 运动完成

    std::cout << "ans: " << GetPositions_mean() << std::endl;

  }

  // 使用距离传感器移动
  void ChassisControl::MoveToDistance(double v, double val, std::string DistanceChioce){
      // 最大速度
      // 运动距离
      // 偏转
      // 方向
      // 多线程
      ChassisRsetPosition();  // 电机初始化
      PrintAt(1, "MoveTD %f", val);
      PrintAt(2, "Moving...");

      double start_val = GetDistance(DistanceChioce);
      double dt = 0.01;
      double stuck_time_count = 0;  // 电机卡死时间统计
      double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();  // 运动开始时间
      double move_time_out = Move_time_out + (std::abs(start_val - val) / v * 1000);
      int    negative = 1;   // 负方向指示器

      // 加速度测试
      double now_speed = 0;

      //  方向指示器
      if (DistanceChioce == "front"){
          negative = -1;
      }
      else if (DistanceChioce == "back"){
          negative = 1;
      }

      PID pid(0.015, 0.0, 0.0); // 创建PID控制器实例。 
      pid.setSetpoint(val); // 设置目标位置

      // 多线程阻断等待
      while (MoveEnd == false){
          wait(10, msec);
          Moveing = false;
      }

      // 直线修正指示器
      ChassisControl::ChassisStraight();


      Moveing = true;  // 当前在运动
      MoveEnd = false; // 运动没有完成
      int overcount = 0;  // 成功到位计数器

      while (true) {  
          double current_position = GetDistance(DistanceChioce);  // 当前位置
          double output = pid.calculate(current_position, dt) * negative;  // 补偿

          // 检查目标是否达到  
          if (std::abs(current_position - val) <= 4 && std::abs(output) <= Stuck_threshold) {
            overcount += 1;
            if (overcount == Over_count){
              PrintAt(2, " ->到位");
              std::cout <<"daowei" <<std::endl;
              break;  
            }
          }else{
            overcount = 0;
          }

          // 多线程保护
          if (Moveing == false){
              PrintAt(2, " ->打断");
              break;
          }

          // // 算法收敛保护
          // if (abs(output) <= Stuck_threshold){
          //   PrintAt(2, " ->收敛");
          //   break;
          // }

          // 电机抱死保护
          if (GetVelocity_mean() <= Stuck_threshold){
              stuck_time_count += dt * 1000.0;  // 计算抱死时间
              if (stuck_time_count >= Stuck_time_out){
                PrintAt(2, " ->抱死");
                break;
              }
          }
          else{
            stuck_time_count = 0; // 不抱死则刷新计数
          }

          // 超时保护
          double now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();// 当前时间
          if (now_time - start_time >= move_time_out / 1000){
              PrintAt(2, " ->超时");
              break;
          }

          now_speed = GetBufferSpeed(now_speed, output, negative);
          // std::cout << "output----" << output << std::endl;
          now_speed = NumInterval(now_speed, min_move_speed, v);
          ChassisForward(now_speed, 0);  //  控制移动

          wait(dt * 1000.0, msec);
      }  

      ChassisStop();
      Moveing = false;  // 运动停止了
      MoveEnd = true; // 运动完成
      PrintAt(3, "%f", GetDistance(DistanceChioce));
  }

  // 使用距离传感器移动(可选多线程)
  void ChassisControl::MoveToDistance(double v, double val, std::string DistanceChioce, bool task){
      if (task == true){
          MoveToDistanceParams* Params = new MoveToDistanceParams();
          Params->self = this;
          Params->v = v;
          Params->val = val;
          Params->DistanceChioce = DistanceChioce;

          void* p = Params;
          vex::task Task(MoveToDistancetTask, p);
          wait(500, msec);
      }
      else{
          MoveToDistance(v, val, DistanceChioce);    
      }
  }
  
  // 直线运动函数（使用定位轮）
  void ChassisControl::MoveBySmallWheel(double v, double val, double yaw, char direction){
      // 最大速度
      // 运动距离
      // 偏转
      // 方向
      // 多线程
      // ChassisRsetPosition();  // 电机初始化

      PrintAt(1, "MoveBSW %f", val);
      PrintAt(2, "Moving...");

      Rotation_x.resetPosition();       // 定位轮初始化

      double dt = 0.01;
      double stuck_time_count = 0;  // 电机卡死时间统计
      double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();  // 运动开始时间
      double move_time_out = Move_time_out + (val / v * 10);
      int    negative = 1;   // 负方向指示器
      int yaw_negative= 1;   // 负偏指示器

      // 加速度测试
      double now_speed = 0;
      // double maxinum_speed_acceleration = 50;

      //  方向指示器
      if (direction == 'u'){
          negative = -1;
          val *= -1;
          yaw_negative = 1;
          yaw = 0;
      }
      else if (direction == 'n'){
          negative = 1;
          yaw_negative = 1;
          yaw = 0;
      }
      else if (direction == 'p'){
          negative = 1;
          yaw_negative = 1;
      }
      else if (direction == 'q'){
          negative = 1;
          yaw_negative = -1;
      }
      else if (direction == 'b'){
          val *= -1;
          negative = -1;
          yaw_negative = 1;
      }
      else if (direction == 'd'){
          negative = -1;
          yaw_negative = -1;
          val *= -1;
      }

      // 直线修正指示器
      if (abs(yaw) < 0.01){
        ChassisControl::ChassisStraight();
        Brain.Screen.printAt( 10, 50, "start %f", ChassisInertial.rotation());
      }


      PID pid(0.010, 0.00, 0.000); // 创建PID控制器实例 
      pid.setSetpoint(val); // 设置目标位置

      // 多线程阻断等待
      while (MoveEnd == false){
          wait(10, msec);
          Moveing = false;
      }

      Moveing = true;  // 当前在运动
      MoveEnd = false; // 运动没有完成
      int nearcount = 0; // 贴近计数
      double now_min_speed = min_move_speed;
      double last_speed = 0;

      int overcount = 0;  // 成功到位计数器

      while (true) {  
          double current_position = Rotation_x.position(deg);  // 当前位置
          double output = pid.calculate(current_position, dt);  // 补偿

          // 检查目标是否达到  
          if (std::abs(current_position - val) <= 10 && std::abs(output) <= Stuck_threshold) {
            overcount += 1;
            if (overcount == Over_count){
              PrintAt(2, " ->到位");
              std::cout <<"daowei" <<std::endl;
              break;  
            }
          }else{
            overcount = 0;
          }

          // 贴近
          if ((output < 0 && last_speed > 0) || (output > 0 && last_speed < 0)){
            Brain.Screen.printAt(10, 80, "----%d-------", nearcount);
            nearcount += 1;
            if (nearcount >= 2){
              now_min_speed -= 0.1;
            }
          }

          // 多线程保护
          if (Moveing == false){
              PrintAt(2, " ->打断");
              break;
          }

          // 算法收敛保护
          // if (std::abs(output) <= Stuck_threshold){
          //   PrintAt(2, " ->收敛");
          //   break;
          // }

          // 电机抱死保护
          if (GetVelocity_mean() <= Stuck_threshold){
              stuck_time_count += dt * 1000.0;  // 计算抱死时间
              if (stuck_time_count >= Stuck_time_out){
                PrintAt(2, " ->抱死");
                break;
              }
              now_min_speed += 0.1;
          }
          else{
            stuck_time_count = 0; // 不抱死则刷新计数
          }

          // 超时保护
          double now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();// 当前时间
          if (now_time - start_time >= move_time_out / 1000){
              PrintAt(2, " ->超时");
              break;
          }
          last_speed = output;
          now_speed = GetBufferSpeed(now_speed, output, negative);
          // std::cout << "output----" << output << std::endl;
          now_speed = NumInterval(now_speed, now_min_speed, v);
          ChassisForward(now_speed, yaw * yaw_negative);  //  控制移动

          wait(dt * 1000.0, msec);
      }  

      ChassisStop();
      PrintAt(3, "%f",  Rotation_x.position(deg));
      Moveing = false;  // 运动停止了
      MoveEnd = true; // 运动完成

  }

  // 直线运动函数（使用定位轮）
  void ChassisControl::MoveBySmallWheel(double v, double val, double yaw, char direction, bool task){
    if (task == true){
        ChassisMoveParams* Params = new ChassisMoveParams();
        Params->self = this;
        Params->v = v;
        Params->val = val;
        Params->yaw = yaw;
        Params->direction = direction;

        void* p = Params;
        vex::task Task(MoveBySmallWheelTask, p);
        wait(500, msec);
    }
    else{
        MoveBySmallWheel(v, val, yaw, direction);    
    }
  }

  // 追随某个目标(多线程)
  void ChassisControl::FollowTarget(double v, double val, bool task){
    if (task == true){
        FollowTargetParams* Params = new FollowTargetParams();
        Params->self = this;
        Params->v = v;
        Params->val = val;
        void* p = Params;
        vex::task Task(FollowTargetTask, p);
        wait(500, msec);
    }
    else{
        FollowTarget(v, val);    
    }
  }

  // 追随某个目标
  void ChassisControl::FollowTarget(double v, double val){
    // 最大速度
      // 运动距离
      // 偏转
      // 方向
      // 多线程
      // ChassisRsetPosition();  // 电机初始化
      PrintAt(1, "FollowT %f", val);
      PrintAt(2, "Moving...");

      Rotation_x.resetPosition();       // 定位轮初始化

      double dt = 0.01;
      double stuck_time_count = 0;  // 电机卡死时间统计
      double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();  // 运动开始时间
      double move_time_out = Move_time_out + (val / v * 1000);
      int    negative = 1;   // 负方向指示器
      int yaw_negative= 1;   // 负偏指示器

      PID pid(0.00900, 0.0, 0.0); // 创建PID控制器实例 
      pid.setSetpoint(val); // 设置目标位置

      PID pid_target(0.05, 0.0, 0.0); // 创建PID控制器实例 
      pid_target.setSetpoint(aiVision_w / 2 + (Target.Align * aiVision_w / 2)); // 设置目标位置


      // 多线程阻断等待
      while (MoveEnd == false){
          wait(10, msec);
          Moveing = false;
      }

      Moveing = true;  // 当前在运动
      MoveEnd = false; // 运动没有完成

      // 设置目标位置
      bool have_target = false;
      have_target = Target.SetTarget();

      // 加速度测试
      double now_speed = 0.0;
      int overcount = 0;  // 成功到位计数器

      while (true) {  
          double current_position = Rotation_x.position(deg);  // 当前位置
          double output = pid.calculate(current_position, dt);  // 前进补偿
          
          // 视觉传感器
          aiVision.takeSnapshot(*(Target.Color));  // 当前视觉传感器的数据
          double output_target = 0.0;
          
          // 获取相机元素
          if (aiVision.objectCount > 0 && have_target){
            int TargetCenterX = Target.GetTargetCenterX();

            output_target = pid_target.calculate(TargetCenterX, dt) * -1;  // 前进补偿
            Brain.Screen.printAt( 10, 10, "target %d, %d", aiVision.largestObject.centerX, 5);
            
            if (output_target > 0){
              output_target = std::min(v / 2, output_target);
            }else{
              output_target = std::max(v / 2 * -1, output_target);
            }
            Brain.Screen.printAt( 10, 90, "output_target %f", output_target);
          }

          // 检查目标是否达到  
          if (std::abs(current_position - val) <= 7.5 && std::abs(output) <= Stuck_threshold) {
            overcount += 1;
            if (overcount == Over_count){
              PrintAt(2, " ->到位");
              std::cout <<"daowei" <<std::endl;
              break;  
            }
          }else{
            overcount = 0;
          }

          // 多线程保护
          if (Moveing == false){
              PrintAt(2, " ->打断");
              break;
          }

          // // 算法收敛保护
          // if (abs(output) <= Stuck_threshold){
          //   PrintAt(2, " ->收敛");
          //   break;
          // }

          // 电机抱死保护
          if (GetVelocity_mean() <= Stuck_threshold){
              stuck_time_count += dt * 1000.0;  // 计算抱死时间
              if (stuck_time_count >= Stuck_time_out){
                PrintAt(2, " ->抱死");
                break;
              }
          }
          else{
            stuck_time_count = 0; // 不抱死则刷新计数
          }

          // 超时保护
          double now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();// 当前时间
          if (now_time - start_time >= move_time_out / 1000){
              PrintAt(2, " ->超时");
              break;
          }

          // if (std::abs(output_target) < 0.5){
          //   output_target = 0.0;
          // }
          
          now_speed = GetBufferSpeed(now_speed, output, negative);
          now_speed = NumInterval(now_speed, min_move_speed, v);
      
          ChassisLeftControl(NumInterval(now_speed + output_target, 0, v * 2));
          ChassisRightControl(NumInterval(now_speed - output_target, 0, v * 2));

          wait(dt * 1000.0, msec);
      }  

      ChassisStop();
      PrintAt(3, "%f", abs(ChassisInertial.rotation()));
      Moveing = false;  // 运动停止了
      MoveEnd = true; // 运动完成

      
  }

  // 旋转到指定对象
  void ChassisControl::TurnToTarget(double v, double val, char direction){
      // 最大速度
      // 运动距离
      // 方向
      // ChassisInertial.resetRotation();  // 陀螺仪初始化

      PrintAt(1, "Turn %f", val);
      PrintAt(2, "Moving...");
  
      double taget_angle = 0.0;
      double start_angle   = ChassisInertial.rotation();
      double dt = 0.01;
      double stuck_time_count = 0;  // 电机卡死时间统计
      double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();  // 运动开始时间
      double move_time_out = Move_time_out + (val / v * 1000) * 1000;
      int yaw_negative= 1;   // 负偏指示器

      //  方向指示器
      if (direction == 'q' || direction == 'b' || direction == 'l'){
          yaw_negative = -1;
          taget_angle  = start_angle - val;
      }
      else if (direction == 'p' || direction == 'd' || direction == 'r'){
          yaw_negative = 1;
          taget_angle  = start_angle + val;
      }


      PID pid(0.1, 0, 0); // 创建PID控制器实例 
      pid.setSetpoint(taget_angle); // 设置目标位置

      // 多线程阻断等待
      while (MoveEnd == false){
          wait(10, msec);
          Moveing = false;
      }

      Moveing = true;  // 当前在运动
      MoveEnd = false; // 运动没有完成

      bool use_taget = false;
      int overcount = 0;
      
      while (true) {  
          double current_position = ChassisInertial.rotation();  // 当前位置
          double output = pid.calculate(current_position, dt) * yaw_negative;  // 补偿

          // 目标角度附近找到了目标物体
          if (abs(current_position - taget_angle) <= 30 and Target.SetTarget()){
            break;
          }
          
          // 检查目标是否达到 
          if (std::abs(current_position - taget_angle) <= 1 && std::abs(output) < Stuck_threshold) {
            overcount += 1;
            if (overcount == Over_count){
              PrintAt(2, " ->到位");
              std::cout <<"daowei" <<std::endl;
              break;  
            }
          }  
          else{
            overcount = 0;
          }

          // 多线程保护
          if (Moveing == false){
              PrintAt(2, " ->打断");
              break;
          }

          // 算法收敛保护
          // if (abs(output) <= Stuck_threshold){
          //   PrintAt(2, " ->收敛");
          //   break;
          // }

          // 电机抱死保护
          if (GetVelocity_mean() <= Stuck_threshold){
              stuck_time_count += dt * 1000.0;  // 计算抱死时间
              if (stuck_time_count >= Stuck_time_out){
                PrintAt(2, " ->抱死");
                break;
              }
          }
          else{
            stuck_time_count = 0; // 不抱死则刷新计数
          }

          // 超时保护
          double now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();// 当前时间
          if (now_time - start_time >= move_time_out / 1000){
              PrintAt(2, " ->超时");
              break;
          }

          output = NumInterval(output, min_turn_speed, v);
          ChassisForward(output, yaw_negative);  //  控制移动


          wait(dt * 1000.0, msec);
      }  
      
      if (Target.SetTarget()){
        PID pid_target(0.04, 0.0, 0.0); // 创建PID控制器实例 
        int tagetX = aiVision_w / 2 + (Target.Align * aiVision_w / 2);
        tagetX = aiVision_w / 2;
        pid_target.setSetpoint(tagetX); // 设置目标位置

        overcount = 0;
        while (true) {  
            aiVision.takeSnapshot(*(Target.Color));  // 当前视觉传感器的数据
            int TargetCenterX = Target.GetTargetCenterX();
            double output= pid_target.calculate(TargetCenterX, dt) * -1;  // 前进补偿

            // 检查目标是否达到  
            if (std::abs(TargetCenterX - tagetX) <= 1 && std::abs(output) < Stuck_threshold) {
              overcount += 1;
              if (overcount == Over_count){
                PrintAt(2, " ->到位");
                std::cout <<"daowei" <<std::endl;
                break;  
              }
            }  

            // 多线程保护
            if (Moveing == false){
                PrintAt(2, " ->打断");
                break;
            }

            // 算法收敛保护
            // if (abs(output) <= Stuck_threshold){
            //   PrintAt(2, " ->收敛");
            //   break;
            // }

            // 电机抱死保护
            if (GetVelocity_mean() <= Stuck_threshold){
                stuck_time_count += dt * 1000.0;  // 计算抱死时间
                if (stuck_time_count >= Stuck_time_out){
                  PrintAt(2, " ->抱死");
                  break;
                }
            }
            else{
              stuck_time_count = 0; // 不抱死则刷新计数
            }

            // 超时保护
            double now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();// 当前时间
            if (now_time - start_time >= move_time_out / 1000){
                PrintAt(2, " ->超时");
                break;
            }

            // 控制小车运动 
            output = NumInterval(output, min_turn_speed, v);
            ChassisLeftControl(output);
            ChassisRightControl(-1 * output);

            wait(dt * 1000.0, msec);
        }  

      }

      ChassisStop();
      PrintAt(3, " %f", ChassisInertial.rotation());
      Moveing = false;  // 运动停止了
      MoveEnd = true; // 运动完成
  }

//  多线程运动（直线）友元函数
int MoveTask(void* args){
    ChassisMoveParams* params = (ChassisMoveParams*)args;
    ChassisControl* self= params->self;       // 当前类
    double v            = params->v;          // 速度
    double val          = params->val;        // 路径长度
    char direction      = params->direction;  // 运动方向
    double yaw          = params->yaw;        // 偏转
    self->Move(v, val, yaw, direction);

    // delete args;
    return 0;
}

//  多线程运动（直线定位轮）友元函数
int MoveBySmallWheelTask(void* args){
    ChassisMoveParams* params = (ChassisMoveParams*)args;
    ChassisControl* self= params->self;       // 当前类
    double v            = params->v;          // 速度
    double val          = params->val;        // 路径长度
    char direction      = params->direction;  // 运动方向
    double yaw          = params->yaw;        // 偏转
    self->MoveBySmallWheel(v, val, yaw, direction);

    // delete args;
    return 0;
}

//  多线程运动（旋转）友元函数
int TurnTask(void* args){
    ChassisMoveParams* params = (ChassisMoveParams*)args;
    ChassisControl* self= params->self;       // 当前类
    double v            = params->v;          // 速度
    double val          = params->val;        // 路径长度
    char direction      = params->direction;  // 运动方向
    self->Turn(v, val, direction);

    // delete args;
    return 0;
}

//  多线程目标追踪友元函数
int FollowTargetTask(void* args){
    FollowTargetParams* params = (FollowTargetParams*)args;
    ChassisControl* self= params->self;       // 当前类
    double v            = params->v;          // 速度
    double val          = params->val;        // 路径长度

    self->FollowTarget(v, val);

    // delete args;
    return 0;
}

//  多线程距离传感器友元函数 
int MoveToDistancetTask(void* args){
    MoveToDistanceParams* params = (MoveToDistanceParams*)args;
    ChassisControl* self= params->self;       // 当前类
    double v            = params->v;          // 速度
    double val          = params->val;        // 路径长度
    std::string DistanceChioce = params->DistanceChioce; // 前后距离传感器选择

    self->MoveToDistance(v, val, DistanceChioce);

    // delete args;
    return 0;
}