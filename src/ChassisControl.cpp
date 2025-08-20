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

  ChassisControl::ChassisControl(){
    // pps参数设置相关
    ChassisSpeedConfig = 450;
    ResetPPS();
  }
  

  // 重新设置相关数据
  void ChassisControl::ResetPPS(){
    // 直线修正
    std::map<int, std::vector<double>> Straight_PPS_Map = {{343, {7.5, 0.0, 0.1}}, {450, {7.5, 0.0, 0.1}}, {300, {7.5, 0.0, 0.1}}};

    // 编码器
    std::map<int, std::vector<double>> Move_PPS_Map = {{343, {0.00950, 0.800, 2}}, {450, {0.00950, 0.800, 2}}, {300, {0.02, 0.800, 2}}};
    std::map<int, double> Min_Move_Speed_Map = {{343, 0.75}, {450, 0.75}, {300, 0.95}};

    // 陀螺仪
    std::map<int, std::vector<double>> Turn_PPS_Map = {{343, {0.075, 0, 3}}, {450, {0.022, 0.5, 5}}, {300, {0.042, 0.5, 5}}};
    std::map<int, double> Min_Turn_Speed_Map = {{343, 1.10}, {450, 1.6}, {300, 1.5}};

    min_move_speed       = Min_Move_Speed_Map[ChassisSpeedConfig];  // 直行电机的最小速度
    min_turn_speed       = Min_Turn_Speed_Map[ChassisSpeedConfig];  // 自转的最小速度

    StraightPid = new PPS(Straight_PPS_Map[ChassisSpeedConfig][0], Straight_PPS_Map[ChassisSpeedConfig][1], Straight_PPS_Map[ChassisSpeedConfig][2]);    
    Move_PPS = Move_PPS_Map[ChassisSpeedConfig];
    Turn_PPS = Turn_PPS_Map[ChassisSpeedConfig];
  }

  // 重新设置当前最大速度
  void ChassisControl::ResetMaxSpeed(double now_speed){
    this->Move_max_speed = now_speed;
  }

    // 重新设置当前直行角度
  void ChassisControl::ResetStraightAngle(double yaw){
    this->StraightAngle = yaw;
  }

  // 底盘重新清零
  void ChassisControl::Reset(){
      ChassisInertial.resetRotation();  // 陀螺仪归零
      Moveing         = true; // 记录是否在运动
      MoveEnd         = true;  // 记录运动是否完成

      // Target.Target_CenterX = -1;
      // Target.Target_CenterY = -1;
      // Target.Target_Width   = -1;
      // Target.Target_Height = -1;
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
    }
    else{
      if (output < nowspeed){
        nowspeed = std::max(output, nowspeed - maxinum_speed_acceleration);
      }else{
        nowspeed = output;
      }
    }
    // nowspeed = output;
    return nowspeed;
  }

  // 底盘直行初始化
  void ChassisControl::ChassisStraight(double angle){
      // angle = NormsAngle_0_360(angle);
      this->StraightPid->reset(0.1, 0.0, 0);
      StraightAngleErrorSum = 0.0;
      this->StraightAngle = angle;
      this->StraightPid->setSetpoint(angle);
      this->StraightNegative = 1;
  }

  // 更新底盘直行对准角度
  void ChassisControl::ChassisForwardAngle(double angle){
      this->StraightPid->reset(0.1, 0.0, 0);
      StraightAngleErrorSum = 0.0;
      this->StraightAngle = angle;
      this->StraightPid->setSetpoint(angle);
      this->StraightNegative = 1;
  }

  // 底盘前进
  void ChassisControl::ChassisForward(double v){
      double now_rotation = ChassisInertial.rotation();
      StraightAngleErrorSum += (now_rotation - this->StraightAngle);

      double output = this->StraightPid->calculate(now_rotation, 0.01) * -1;  // 补偿
      // Brain.Screen.printAt( 10, 100, "output %f", output);
      // Brain.Screen.printAt( 10, 150, "this->StraightAngle %f", this->StraightPid->getsetpoint());
      if (output < 0){
        output = std::max(output, -1 * StraightCorrectionSpeed);
      }else{
        output = std::min(output, 1 * StraightCorrectionSpeed);
      }
      // std::cout << "s-> " << now_rotation << " " << output << std::endl;

      LeftMotor1.spin(vex::forward,  v - (output * this->StraightNegative), voltageUnits::volt);
      LeftMotor2.spin(vex::forward,  v - (output * this->StraightNegative), voltageUnits::volt);
      LeftMotor3.spin(vex::forward,  v - (output * this->StraightNegative), voltageUnits::volt);
      RightMotor1.spin(vex::forward, v + (output * this->StraightNegative), voltageUnits::volt);
      RightMotor2.spin(vex::forward, v + (output * this->StraightNegative), voltageUnits::volt);
      RightMotor3.spin(vex::forward, v + (output * this->StraightNegative), voltageUnits::volt);
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

  // 底盘左轮控制-百分比
  void ChassisControl::ChassisLeftControl_pct(double v){
    v *= 6.25;
    LeftMotor1.spin(vex::forward,  v, pct);
    LeftMotor2.spin(vex::forward,  v, pct);
    LeftMotor3.spin(vex::forward,  v, pct);
  }

  // 底盘左轮控制-百分比
  void ChassisControl::ChassisRightControl_pct(double v){
    v *= 6.25;
    RightMotor1.spin(vex::forward, v, pct);
    RightMotor2.spin(vex::forward, v, pct);
    RightMotor3.spin(vex::forward, v, pct);
  }

  // 等待电机编码器到达某个值
  void ChassisControl::WaitVelocityArrive(double val){

    int now_p;

    while(this->IsMoving()){
      now_p = this->GetPositions_mean();  

      if (now_p < 0){
          if (-1 * now_p > val){
            break;
          }

      }else{
          if (now_p > val){
            break;
          }
      }

        wait(10, msec);

      }
  }

  // 获取当前的位置
  double ChassisControl::GetPositions_mean(){
      double ans = 0.0;

      ans += medianOfThree(LeftMotor1.position(vex::rotationUnits::deg), LeftMotor2.position(vex::rotationUnits::deg), LeftMotor3.position(vex::rotationUnits::deg));
      ans += medianOfThree(RightMotor1.position(vex::rotationUnits::deg), RightMotor2.position(vex::rotationUnits::deg), RightMotor3.position(vex::rotationUnits::deg));

      return ans / 2;
  }

  // 获取当前的速度
  double ChassisControl::GetVelocity_mean(){
      double ans = 0.0;

      ans += std::abs(medianOfThree(LeftMotor1.velocity(dps), LeftMotor2.velocity(dps), LeftMotor3.velocity(dps)));
      ans += std::abs(medianOfThree(RightMotor1.velocity(dps), RightMotor2.velocity(dps), RightMotor3.velocity(dps)));

      return ans / 2;
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

  // 运动前
  void ChassisControl::MoveStart(){

  }

  // 运动后
  void ChassisControl::MoveOver(){
      ResetPPS();
      ChassisStop();
      Moveing = false;  // 运动停止了
      MoveEnd = true; // 运动完成
      Whileing = false; // 循环结束了
  }

  // 直线运动函数
  void ChassisControl::Move(double v, double val, char direction){
    this->Move(v, val, ChassisInertial.rotation(), direction);
  }

  // 直线运动函数（可选多线程）
  void ChassisControl::Move(double v, double val, char direction, bool task){
    this->Move(v, val, ChassisInertial.rotation(), direction, task);
  }

  // 直线运动函数
  void ChassisControl::Move(double v, double val, double yaw, char direction){
      // 最大速度
      // 运动距离
      // 偏转
      // 方向
      // 多线程
      // PrintAt(1, "Move %f", val);
      // PrintAt(2, "Moving...");
      v    = std::abs(v);
      val  = std::abs(val);

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
          val *= -1;
      }
      else if (direction == 'n'){
          negative = 1;
          yaw_negative = 1;
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

      this->Move_max_speed = v;

      // 直线修正指示器
      this->StraightAngle = yaw;
      ChassisControl::ChassisStraight(yaw);

      PPS pps(this->Move_PPS[0], this->Move_PPS[1], this->Move_PPS[2]); // 创建PPS控制器实例

      pps.setSetpoint(val); // 设置目标位置

      // 多线程阻断等待
      while (MoveEnd == false){
          wait(10, msec);
          Moveing = false;
      }

      Moveing = true;  // 当前在运动
      MoveEnd = false; // 运动没有完成
      int overcount = 0;  // 成功到位计数器
      int nearcount = 0; // 贴近计数
      double now_min_speed = min_move_speed;
      double last_speed = 0;
      bool add_over = false;

      while (true) {  
          Whileing = true;  // 表示正在循环了
          double current_position = GetPositions_mean();  // 当前位置
          // Brain.Screen.printAt( 10, 50, "current_position %f", current_position);
          this->ChassisForwardAngle(this->StraightAngle);
         
          double output = pps.calculate(current_position, dt);  // 补偿

          if (std::abs(current_position - val) <= move_allowable_error && std::abs(output) <= min_move_speed * 10) {
            overcount += 1;
            if (overcount == Over_count){
              // PrintAt(2, " ->到位");
              // std::cout <<"daowei" <<std::endl;
              break;  
            }
          }else{
            overcount = 0;
          } 

          // 贴近
          if ((output < 0 && last_speed > 0) || (output > 0 && last_speed < 0)){
            // Brain.Screen.printAt(10, 80, "----%d-------", nearcount);
            nearcount += 1;
            if (nearcount >= 2){
              now_min_speed -= 0.1;
            }
          }

          // 多线程保护
          if (Moveing == false){
              // PrintAt(2, " ->打断");
              break;
          }

          // std::cout <<  ";;;;;;;" << GetVelocity_mean() << std::endl;

          // 电机抱死保护
          if (GetVelocity_mean() <= Stuck_threshold){
              stuck_time_count += dt * 1000.0;  // 计算抱死时间
              if (stuck_time_count >= Stuck_time_out){
                // PrintAt(2, " ->抱死");
                break;
              }
              now_min_speed += 0.1;
          }
          else{
            // stuck_time_count = 0; // 不抱死则刷新计数
          }

          // 超时保护
          double now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();// 当前时间
          if (now_time - start_time >= move_time_out / 1000){
              // PrintAt(2, " ->超时");
              break;
          }
          
          // 加速度设置
          if (! add_over){
            now_speed = GetBufferSpeed(now_speed, output, negative);
            if (std::abs(now_speed) < std::abs(last_speed)){
              add_over = true;
            }

          }else{
            now_speed = output;
          }

          last_speed = now_speed;

          // std::cout << "output----" << output  << " " << current_position << std::endl;
          now_speed = NumInterval(now_speed, now_min_speed, this->Move_max_speed);
          ChassisForward(now_speed);  //  控制移动

          wait(dt * 1000.0, msec);
      }  
      MoveOver();
      // PrintAt(3, "%f",  GetPositions_mean());
      // std::cout << "ans: " << GetPositions_mean() << std::endl;
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
        this->WaitMoveStart();
    }
    else{
        Move(v, val, yaw, direction);    
    }
  }

  // 曲线移动
  void ChassisControl::CurveMove(double v, double r, double TargetPosition, char direction){
      ChassisRsetPosition();  // 电机初始化

      double dt = 0.01;
      double stuck_time_count = 0;  // 电机卡死时间统计
      double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();  // 运动开始时间
      double move_time_out = Move_time_out + (TargetPosition / v * 1000);
      int    negative = 1;   // 负方向指示器
      int yaw_negative= 1;   // 负偏指示器
      double PI = 3.1415;

      // 加速度测试
      double now_speed = 0;

      //  方向指示器
      if (direction == 'p'){
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
          TargetPosition *= -1;
      }
      else if (direction == 'd'){
          negative = -1;
          yaw_negative = -1;
          TargetPosition *= -1;
      }

      PPS pps(this->Move_PPS[0], this->Move_PPS[1], this->Move_PPS[2]); // 创建PPS控制器实例

      pps.setSetpoint(TargetPosition); // 设置目标位置

      // 多线程阻断等待
      while (MoveEnd == false){
          wait(10, msec);
          Moveing = false;
      }

      Moveing = true;  // 当前在运动
      MoveEnd = false; // 运动没有完成
      int overcount = 0;  // 成功到位计数器
      int nearcount = 0; // 贴近计数
      double now_min_speed = min_move_speed;
      double last_speed = 0;
      bool add_over = false;

      double start_angle   = ChassisInertial.rotation();

      while (true){
          Whileing = true;  // 表示正在循环了
          double current_angle = ChassisInertial.rotation();  // 当前位置
          double true_angle    = 0;
          double current_position = GetPositions_mean(); 

          // 计算当前理论陀螺仪角度
          true_angle = start_angle + current_position / (2 * PI * r) * 360 * yaw_negative;
          std::cout << true_angle << std::endl;
          ChassisForwardAngle(true_angle);

          double output = pps.calculate(current_position, dt);  // 补偿

          ChassisForwardAngle(true_angle);

          if (std::abs(current_position - TargetPosition) <= move_allowable_error && std::abs(output) <= min_move_speed * 10) {
            overcount += 1;
            if (overcount == Over_count){
              // PrintAt(2, " ->到位");
              // std::cout <<"daowei" <<std::endl;
              break;  
            }
          }else{
            overcount = 0;
          } 

          // 贴近
          if ((output < 0 && last_speed > 0) || (output > 0 && last_speed < 0)){
            // Brain.Screen.printAt(10, 80, "----%d-------", nearcount);
            nearcount += 1;
            if (nearcount >= 2){
              now_min_speed -= 0.1;
            }
          }

          // 多线程保护
          if (Moveing == false){
              // PrintAt(2, " ->打断");
              break;
          }

          // std::cout <<  ";;;;;;;" << GetVelocity_mean() << std::endl;

          // 电机抱死保护
          if (GetVelocity_mean() <= Stuck_threshold){
              stuck_time_count += dt * 1000.0;  // 计算抱死时间
              if (stuck_time_count >= Stuck_time_out){
                // PrintAt(2, " ->抱死");
                break;
              }
              now_min_speed += 0.1;
          }
          else{
            // stuck_time_count = 0; // 不抱死则刷新计数
          }

          // 超时保护
          double now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();// 当前时间
          if (now_time - start_time >= move_time_out / 1000){
              // PrintAt(2, " ->超时");
              break;
          }
          
          // 加速度设置
          if (! add_over){
            now_speed = GetBufferSpeed(now_speed, output, negative);
            if (std::abs(now_speed) < std::abs(last_speed)){
              add_over = true;
            }

          }else{
            now_speed = output;
          }

          last_speed = now_speed;

          // std::cout << "output----" << output  << " " << current_position << std::endl;
          now_speed = NumInterval(now_speed, now_min_speed, v);
          // ChassisForward(now_speed);  //  控制移动

          double angle_compensation = (true_angle - current_angle);

          ChassisLeftControl(now_speed * (1 + 200 / (2 * r) * yaw_negative) + angle_compensation * 0.1);
          ChassisRightControl(now_speed * (1 - 200 / (2 * r) * yaw_negative) - angle_compensation * 0.1);

          //  ChassisLeftControl(now_speed * (1 + 820 / (2 * r) * yaw_negative));
          // ChassisRightControl(now_speed * (1 - 820 / (2 * r) * yaw_negative));

          wait(dt * 1000.0, msec);
          


      }
      MoveOver();

  }

  // 组合运动
  void ChassisControl::ComboMove(std::vector<ComboMoveParams> MoveParams){
      // PrintAt(1, "CMove ");
      // PrintAt(2, "Moving...");
      ChassisRsetPosition();  // 电机初始化

      double dt = 0.01;
      double stuck_time_count = 0;  // 电机卡死时间统计
      double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();  // 运动开始时间
      double move_time_out = Move_time_out + (MoveParams[0].val / MoveParams[0].v * 1000);
      bool over = false;  // 记录是否异常退出

      for (ComboMoveParams Params: MoveParams){

          int    negative = 1;   // 负方向指示器
          int yaw_negative= 1;   // 负偏指示器
          double v   = Params.v;
          double yaw = Params.yaw;
          double val = Params.val;

          //  方向指示器
          if (Params.direction == 'u'){
              negative = -1;
              yaw_negative = 1;
              yaw = 0;
              val *= -1;
          }
          else if (Params.direction == 'n'){
              negative = 1;
              yaw_negative = 1;
              yaw = 0;
          }
          else if (Params.direction == 'p'){
              negative = 1;
              yaw_negative = 1;
          }
          else if (Params.direction == 'q'){
              negative = 1;
              yaw_negative = -1;
          }
          else if (Params.direction == 'b'){
              negative = -1;
              yaw_negative = 1;
              val *= -1;
          }
          else if (Params.direction == 'd'){
              negative = -1;
              yaw_negative = -1;
              val *= -1;
          }
          // 直线修正指示器
          if (abs(yaw) < 0.01){
            ChassisControl::ChassisStraight(ChassisInertial.rotation());
          }

          // 调用函数
          if (Params.func != nullptr){
            Params.func();
          }

          while(true){
              double current_position = GetPositions_mean();  // 当前位置

              if (negative == 1){
                if (current_position > val) {
                    break;
                } 
              }else{
                if (current_position < val) {
                    break;
                } 
              }

              // 电机抱死保护
              if (GetVelocity_mean() <= Stuck_threshold){
                  stuck_time_count += dt * 1000.0;  // 计算抱死时间
                  if (stuck_time_count >= Stuck_time_out){
                    // PrintAt(2, " ->抱死");
                    over = true;
                    break;
                  }
              }

              // 超时保护/
              double now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();// 当前时间
              if (now_time - start_time >= move_time_out / 1000){
                  // PrintAt(2, " ->超时");
                  over = true;
                  break;
              }
              // Brain.Screen.printAt(100, 50, "====%f", yaw * yaw_negative);
              ChassisForward(v * negative);  //  控制移动
              wait(dt * 1000.0, msec);
          }

          // 异常退出
          if (over){
            break;
          }
      }
      MoveOver();
      // PrintAt(3, "%f",  GetPositions_mean());
      // std::cout << "ans: " << GetPositions_mean() << std::endl;
  }

  // 旋转函数
  void ChassisControl::Turn(double v, double val, char direction){
      // 最大速度
      // 运动距离
      // 方向
      // ChassisInertial.resetRotation();  // 陀螺仪初始化

      // PrintAt(1, "Turn %f", val);
      // PrintAt(2, "Moving...");
      v    = std::abs(v);
      val  = std::abs(val);
  
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

      PPS pps(this->Turn_PPS[0], this->Turn_PPS[1], this->Turn_PPS[2]); // 创建PPS控制器实例

      pps.setSetpoint(taget_angle - start_angle); // 设置目标位置

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
      bool add_over = false;

      while (true) {  
          Whileing = true;
          double current_position = ChassisInertial.rotation();  // 当前位置
          double output = pps.calculate(current_position - start_angle, dt) * yaw_negative;  // 补偿

          // 检查目标是否达到  
          // std::cout <<"turn" << current_position - start_angle << " " << output << std::endl;
          // std::cout << "overcount" << overcount << std::endl;

          // 到位
          if (std::abs(current_position - taget_angle) <= turn_allowable_error && std::abs(output) <  min_turn_speed ) {
            overcount += 1;
            if (overcount == Over_count){
              PrintAt(2, " ->到位");
              // std::cout <<"daowei" <<std::endl;
              break;  
            }
          }  
          else{
            overcount = 0;
          }

          // 贴近
          if ((output < 0 && last_speed > 0) || (output > 0 && last_speed < 0)){
            // Brain.Screen.printAt(10, 80, "----%d-------", nearcount);
            nearcount += 1;
            if (nearcount >= 2){
              now_min_speed -= 0.45;
            }
          }

          // 多线程保护
          if (Moveing == false){
              // PrintAt(2, " ->打断");
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

              // now_min_speed += 0.01;
          }
          else{
            stuck_time_count = 0; // 不抱死则刷新计数
          }

          // 过慢保护
          if (GetVelocity_mean() <= 100){
              now_min_speed += 0.007;
          }

          // 超时保护
          double now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();// 当前时间
          if (now_time - start_time >= move_time_out / 1000){
              // PrintAt(2, " ->超时");
              break;
          }

          output = NumInterval(output, now_min_speed, v);
          
          // std::cout << output << " " << now_min_speed << " " << output * yaw_negative  << " " << -1 * output * yaw_negative << std::endl;
          // ChassisForward(output, yaw_negative);  //  控制移动
          ChassisLeftControl(output * yaw_negative);
          ChassisRightControl(-1 * output * yaw_negative);

          last_speed = output;
          wait(dt * 1000.0, msec);
      }  
      // std::cout << ChassisInertial.rotation() << std::endl;
      MoveOver();
      // PrintAt(3, " %d", (int)ChassisInertial.rotation());
    
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
          this->WaitMoveStart();
      }
      else{
          Turn(v, val, direction);    
      }
  }

  // 旋转到指定角度
  void ChassisControl::TurnTo(double v, double val, bool task){
    double start_angle        = ChassisInertial.rotation();
    double start_angle_norm   = NormsAngle(start_angle);

    if (start_angle_norm < 0){
      start_angle_norm = 360 + start_angle_norm;
    }

    // std::cout << start_angle_norm - val  << std::endl;

    double lval = start_angle_norm - val;
    double rval = 360 - lval;

    if (start_angle_norm > val){
      lval = start_angle_norm - val;
      rval = 360 - lval;
    }else{
      rval = val - start_angle_norm;
      lval = 360 - rval;
    }

    if (lval > rval){
      this->Turn(v, rval, 'r', task);
    }
    else{
      this->Turn(v, lval, 'l', task);
    }
    
  }

  // 等待底盘运动开始
  void ChassisControl::WaitMoveStart(){
      while(! this->Whileing){
          wait(10, msec);
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
      return (Moveing == true) || (MoveEnd == false) || (Whileing == true);
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

  // 使用距离传感器移动
  void ChassisControl::MoveToDistance(double v, double val, std::string DistanceChioce){
      // 最大速度
      // 运动距离
      // 偏转
      // 方向
      // 多线程
      ChassisRsetPosition();  // 电机初始化
      // PrintAt(1, "MoveTD %f", val);
      // PrintAt(2, "Moving...");

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
      double start_position = GetDistance(DistanceChioce);  // 当前位置
      PPS pps(0.0165, 0.3, 2.5); // 创建PPS控制器实例。 
      pps.setSetpoint(val - start_position); // 设置目标位置

      // 多线程阻断等待
      while (MoveEnd == false){
          wait(10, msec);
          Moveing = false;
      }

      // 直线修正指示器
      ChassisControl::ChassisStraight(ChassisInertial.rotation());


      Moveing = true;  // 当前在运动
      MoveEnd = false; // 运动没有完成
      int overcount = 0;  // 成功到位计数器
      int nearcount = 0; // 贴近计数
      double now_min_speed = min_move_speed;
      double last_speed = 0;

      while (true) {  
          double current_position = GetDistance(DistanceChioce) - start_position;  // 当前位置
          double output = pps.calculate(current_position, dt)  * negative;  // 补偿

          // 检查目标是否达到  
          if (std::abs(current_position - val + start_position) <= 4 && std::abs(output) <= Stuck_threshold) {
            overcount += 1;
            if (overcount == Over_count){
              // PrintAt(2, " ->到位");
              break;  
            }
          }else{
            overcount = 0;
          }

          // 贴近
          if ((output < 0 && last_speed > 0) || (output > 0 && last_speed < 0)){
            // Brain.Screen.printAt(10, 80, "----%d-------", nearcount);
            nearcount += 1;
            if (nearcount >= 2){
              now_min_speed -= 0.1;
            }
          }

          // 多线程保护
          if (Moveing == false){
              // PrintAt(2, " ->打断");
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
                // PrintAt(2, " ->抱死");
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
              // PrintAt(2, " ->超时");
              break;
          }

          last_speed = output;
          now_speed = GetBufferSpeed(now_speed, output, negative);
          // std::cout << "output----" << output << std::endl;
          now_speed = NumInterval(now_speed, now_min_speed, v);
          ChassisForward(now_speed);  //  控制移动

          wait(dt * 1000.0, msec);
      }  

      ChassisStop();
      Moveing = false;  // 运动停止了
      MoveEnd = true; // 运动完成
      // PrintAt(3, "%f", GetDistance(DistanceChioce));
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

      // PrintAt(1, "MoveBSW %f", val);
      // PrintAt(2, "Moving...");

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
        ChassisControl::ChassisStraight(ChassisInertial.rotation());
        // ( 10, 50, "start %f", ChassisInertial.rotation());
      }


      PPS pps(0.0085, 0.8, 3); // 创建PPS控制器实例 
      pps.setSetpoint(val); // 设置目标位置

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
          double output = pps.calculate(current_position, dt);  // 补偿

          // 检查目标是否达到  
          if (std::abs(current_position - val) <= 7.5 && std::abs(output) <= Stuck_threshold) {
            overcount += 1;
            if (overcount == Over_count){
              // PrintAt(2, " ->到位");
              // std::cout <<"daowei" <<std::endl;
              break;  
            }
          }else{
            overcount = 0;
          }

          // 贴近
          if ((output < 0 && last_speed > 0) || (output > 0 && last_speed < 0)){
            // Brain.Screen.printAt(10, 80, "----%d-------", nearcount);
            nearcount += 1;
            if (nearcount >= 2){
              now_min_speed -= 0.1;
            }
          }

          // 多线程保护
          if (Moveing == false){
              // PrintAt(2, " ->打断");
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
                // PrintAt(2, " ->抱死");
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
              // PrintAt(2, " ->超时");
              break;
          }
          last_speed = output;
          now_speed = GetBufferSpeed(now_speed, output, negative);
          // std::cout << "output----" << output << std::endl;
          now_speed = NumInterval(now_speed, now_min_speed, v);
          ChassisForward(now_speed);  //  控制移动

          wait(dt * 1000.0, msec);
      }  

      ChassisStop();
      // PrintAt(3, "%f",  Rotation_x.position(deg));
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

  // // 追随某个目标
  // void ChassisControl::FollowTarget(double v, double val){
  //   // 最大速度
  //     // 运动距离
  //     // 偏转
  //     // 方向
  //     // 多线程
  //     // ChassisRsetPosition();  // 电机初始化
  //     // PrintAt(1, "FollowT %f", val);
  //     // PrintAt(2, "Moving...");

  //     Rotation_x.resetPosition();       // 定位轮初始化

  //     double dt = 0.01;
  //     double stuck_time_count = 0;  // 电机卡死时间统计
  //     double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();  // 运动开始时间
  //     double move_time_out = Move_time_out + (val / v * 1000);
  //     int    negative = 1;   // 负方向指示器
  //     int yaw_negative= 1;   // 负偏指示器

  //     PPS pps(0.00900, 0.0, 0.0); // 创建PPS控制器实例 
  //     pps.setSetpoint(val); // 设置目标位置

  //     PPS pps_target(0.03, 0.0, 0.0); // 创建PPS控制器实例 
  //     pps_target.setSetpoint(aiVision_w / 2 + (Target.Align * aiVision_w / 2)); // 设置目标位置


  //     // 多线程阻断等待
  //     while (MoveEnd == false){
  //         wait(10, msec);
  //         Moveing = false;
  //     }

  //     Moveing = true;  // 当前在运动
  //     MoveEnd = false; // 运动没有完成

  //     // 设置目标位置
  //     bool have_target = false;
  //     have_target = Target.SetTarget();

  //     // 加速度测试
  //     double now_speed = 0.0;
  //     int overcount = 0;  // 成功到位计数器

  //     int nearcount = 0; // 贴近计数
  //     double now_min_speed = min_move_speed;
  //     double last_speed = 0;

  //     while (true) {  
  //         double current_position = Rotation_x.position(deg);  // 当前位置
  //         double output = pps.calculate(current_position, dt);  // 前进补偿
          
  //         // 视觉传感器
  //         aiVision.takeSnapshot(*(Target.Color));  // 当前视觉传感器的数据
  //         double output_target = 0.0;
          
  //         // 获取相机元素
  //         if (aiVision.objectCount > 0 && have_target){
  //           int TargetCenterX = Target.GetTargetCenterX();

  //           output_target = pps_target.calculate(TargetCenterX, dt) * -1;  // 前进补偿
  //           // Brain.Screen.printAt( 10, 10, "target %d, %d", aiVision.largestObject.centerX, 5);
            
  //           if (output_target > 0){
  //             output_target = std::min(v / 2, output_target);
  //           }else{
  //             output_target = std::max(v / 2 * -1, output_target);
  //           }
  //           // Brain.Screen.printAt( 10, 90, "output_target %f", output_target);
  //         }

  //         // 检查目标是否达到  
  //         if (std::abs(current_position - val) <= 7.5 && std::abs(output) <= Stuck_threshold) {
  //           overcount += 1;
  //           if (overcount == Over_count){
  //             // PrintAt(2, " ->到位");
  //             // std::cout <<"daowei" <<std::endl;
  //             break;  
  //           }
  //         }else{
  //           overcount = 0;
  //         }

  //         // 贴近
  //         if ((output < 0 && last_speed > 0) || (output > 0 && last_speed < 0)){
  //           // Brain.Screen.printAt(10, 80, "----%d-------", nearcount);
  //           nearcount += 1;
  //           if (nearcount >= 2){
  //             now_min_speed -= 0.1;
  //           }
  //         }

  //         // 多线程保护
  //         if (Moveing == false){
  //             // PrintAt(2, " ->打断");
  //             break;
  //         }

  //         // // 算法收敛保护
  //         // if (abs(output) <= Stuck_threshold){
  //         //   PrintAt(2, " ->收敛");
  //         //   break;
  //         // }

  //         // 电机抱死保护
  //         if (GetVelocity_mean() <= Stuck_threshold){
  //             stuck_time_count += dt * 1000.0;  // 计算抱死时间
  //             if (stuck_time_count >= Stuck_time_out){
  //               // PrintAt(2, " ->抱死");
  //               break;
  //             }
  //             now_min_speed += 0.1;
  //         }
  //         else{
  //           stuck_time_count = 0; // 不抱死则刷新计数
  //         }

  //         // 超时保护
  //         double now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();// 当前时间
  //         if (now_time - start_time >= move_time_out / 1000){
  //             // PrintAt(2, " ->超时");
  //             break;
  //         }

  //         // if (std::abs(output_target) < 0.5){
  //         //   output_target = 0.0;
  //         // }

  //         last_speed = output;
  //         now_speed = GetBufferSpeed(now_speed, output, negative);
  //         now_speed = NumInterval(now_speed, now_min_speed, v);
      
  //         ChassisLeftControl(NumInterval(now_speed + output_target, 0, v * 2));
  //         ChassisRightControl(NumInterval(now_speed - output_target, 0, v * 2));

  //         wait(dt * 1000.0, msec);
  //     }  

  //     ChassisStop();
  //     // PrintAt(3, "%f", abs(ChassisInertial.rotation()));
  //     Moveing = false;  // 运动停止了
  //     MoveEnd = true; // 运动完成

      
  // }

  // // 旋转到指定对象
  // void ChassisControl::TurnToTarget(double v, double val, char direction){
      // 最大速度
      // 运动距离
      // 方向
      // ChassisInertial.resetRotation();  // 陀螺仪初始化

      // PrintAt(1, "Turn %f", val);
      // PrintAt(2, "Moving...");
  
  //     double taget_angle = 0.0;
  //     double start_angle   = ChassisInertial.rotation();
  //     double dt = 0.01;
  //     double stuck_time_count = 0;  // 电机卡死时间统计
  //     double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();  // 运动开始时间
  //     double move_time_out = Move_time_out + (val / v * 1000) * 1000;
  //     int yaw_negative= 1;   // 负偏指示器

  //     //  方向指示器
  //     if (direction == 'q' || direction == 'b' || direction == 'l'){
  //         yaw_negative = -1;
  //         taget_angle  = start_angle - val;
  //     }
  //     else if (direction == 'p' || direction == 'd' || direction == 'r'){
  //         yaw_negative = 1;
  //         taget_angle  = start_angle + val;
  //     }


  //     PPS pps(0.1, 0, 0); // 创建PPS控制器实例 
  //     pps.setSetpoint(taget_angle); // 设置目标位置

  //     // 多线程阻断等待
  //     while (MoveEnd == false){
  //         wait(10, msec);
  //         Moveing = false;
  //     }

  //     Moveing = true;  // 当前在运动
  //     MoveEnd = false; // 运动没有完成

  //     int overcount = 0; // 到位计数
  //     int nearcount = 0; // 贴近计数
  //     double now_min_speed = min_turn_speed;
  //     double last_speed = 0;

  //     bool use_taget = false;
      
  //     while (true) {  
  //         double current_position = ChassisInertial.rotation();  // 当前位置
  //         double output = pps.calculate(current_position, dt) * yaw_negative;  // 补偿

  //         // 目标角度附近找到了目标物体
  //         if (abs(current_position - taget_angle) <= 30 and Target.SetTarget()){
  //           break;
  //         }
          
  //         // 检查目标是否达到 
  //         if (std::abs(current_position - taget_angle) <= 1 && std::abs(output) < Stuck_threshold) {
  //           overcount += 1;
  //           if (overcount == Over_count){
  //             // PrintAt(2, " ->到位");
  //             // std::cout <<"daowei" <<std::endl;
  //             break;  
  //           }
  //         }  
  //         else{
  //           overcount = 0;
  //         }

  //         // 贴近
  //         if ((output < 0 && last_speed > 0) || (output > 0 && last_speed < 0)){
  //           // Brain.Screen.printAt(10, 80, "----%d-------", nearcount);
  //           nearcount += 1;
  //           if (nearcount >= 2){
  //             now_min_speed -= 0.1;
  //           }
  //         }

  //         // 多线程保护
  //         if (Moveing == false){
  //             // PrintAt(2, " ->打断");
  //             break;
  //         }

  //         // 算法收敛保护
  //         // if (abs(output) <= Stuck_threshold){
  //         //   PrintAt(2, " ->收敛");
  //         //   break;
  //         // }

  //         // 电机抱死保护
  //         if (GetVelocity_mean() <= Stuck_threshold){
  //             stuck_time_count += dt * 1000.0;  // 计算抱死时间
  //             if (stuck_time_count >= Stuck_time_out){
  //               // PrintAt(2, " ->抱死");
  //               break;
  //             }
  //             now_min_speed += 0.1;
  //         }
  //         else{
  //           stuck_time_count = 0; // 不抱死则刷新计数
  //         }

  //         // 超时保护
  //         double now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();// 当前时间
  //         if (now_time - start_time >= move_time_out / 1000){
  //             // PrintAt(2, " ->超时");
  //             break;
  //         }

  //         output = NumInterval(output, now_min_speed, v);
  //         ChassisForward(output, yaw_negative);  //  控制移动
  //         last_speed = output;

  //         wait(dt * 1000.0, msec);
  //     }  
      
  //     if (Target.SetTarget()){
  //       PPS pps_target(0.04, 0.0, 0.0); // 创建PPS控制器实例 
  //       int tagetX = aiVision_w / 2 + (Target.Align * aiVision_w / 2);
  //       tagetX = aiVision_w / 2;
  //       pps_target.setSetpoint(tagetX); // 设置目标位置

  //       overcount = 0;
  //       while (true) {  
  //           aiVision.takeSnapshot(*(Target.Color));  // 当前视觉传感器的数据
  //           int TargetCenterX = Target.GetTargetCenterX();
  //           double output= pps_target.calculate(TargetCenterX, dt) * -1;  // 前进补偿

  //           // 检查目标是否达到  
  //           if (std::abs(TargetCenterX - tagetX) <= 1 && std::abs(output) < Stuck_threshold) {
  //             overcount += 1;
  //             if (overcount == Over_count){
  //               // PrintAt(2, " ->到位");
  //               // std::cout <<"daowei" <<std::endl;
  //               break;  
  //             }
  //           }  

  //           // 多线程保护
  //           if (Moveing == false){
  //               // PrintAt(2, " ->打断");
  //               break;
  //           }

  //           // 算法收敛保护
  //           // if (abs(output) <= Stuck_threshold){
  //           //   PrintAt(2, " ->收敛");
  //           //   break;
  //           // }

  //           // 电机抱死保护
  //           if (GetVelocity_mean() <= Stuck_threshold){
  //               stuck_time_count += dt * 1000.0;  // 计算抱死时间
  //               if (stuck_time_count >= Stuck_time_out){
  //                 // PrintAt(2, " ->抱死");
  //                 break;
  //               }
  //           }
  //           else{
  //             stuck_time_count = 0; // 不抱死则刷新计数
  //           }

  //           // 超时保护
  //           double now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();// 当前时间
  //           if (now_time - start_time >= move_time_out / 1000){
  //               // PrintAt(2, " ->超时");
  //               break;
  //           }

  //           // 控制小车运动 
  //           output = NumInterval(output, min_turn_speed, v);
  //           ChassisLeftControl(output);
  //           ChassisRightControl(-1 * output);

  //           wait(dt * 1000.0, msec);
  //       }  

  //     }

  //     ChassisStop();
  //     // PrintAt(3, " %f", ChassisInertial.rotation());
  //     Moveing = false;  // 运动停止了
  //     MoveEnd = true; // 运动完成
  // }

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

