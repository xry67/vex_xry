#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "ToolControl.h"
#include "RemoteControl.h"

// 该文件为帮助文件，记录各个函数的用法

void Help(){

    // 基础控制===========================================================
    
    // ->马达基础控制
    // ToolMotor1：马达的名字，可以在robot-config.cpp中根据端口查找需要的电机名称
    // -16：电机的旋转的功率取值范围 -16~16
    // voltageUnits::volt：表示使用电压作为单位
    ToolMotor1.spin(forward, -16, voltageUnits::volt);

    // ->气动基础控制
    // digitalE：气动的名字，可以根据接线查找需要的气动端口
    digitalE.set(0);  // E端口的气动设置回到初始状态
    digitalE.set(1);  // E端口的气动设置为反转的初始状态

    // ->阻断程序
    // 1000: 等待一千毫秒
    // msec：毫秒
    wait(1000, msec);


    // 基础底盘运动相关====================================================

    // ->到位计数更改
    // 设置到位计数为4次，默认为10次
    // 次数越多，运动到位越慢，精确度越高
    // 次数越少，运动到位越快，精确度越底
    ChassisControlSTS.Over_count = 4;    

    // ->直线运动调用
    // 16：运动的速度，取值范围0~16
    // 600：运动的长度，是电机编码器的编码值。100大约为一拳的距离
    // ‘n’：运动的方向，‘n’向前；‘u’向后
    ChassisControlSTS.Move(16, 600, 'n');  // 默认不开启多线程
    ChassisControlSTS.Move(16, 600, 'n', true);  // 开启多线程

    // ->曲线运动调用
    // 10：运动的速度，取值范围0~16
    // 780：曲线运动的半径
    // 700：曲线运动的弧长，是电机编码器的编码值
    // ‘p'：运动的方向，‘p’右前；‘q’左前；‘d’左后；‘b’右后
    ChassisControlSTS.CurveMove(10, 780, 700, 'p');

    // ->自转运动调用
    // 10：运动的速度，取值范围0~16
    // 45：旋转的角度，取值范围0~360
    // ‘l’：旋转的方向；‘l’：左转；‘r’：右转
    ChassisControlSTS.Turn(10, 45, 'l');   // 默认不开启多线程
    ChassisControlSTS.Turn(10, 45, 'l', true);  // 开启多线程

    // ->转到 运动调用
    // 10：运动的速度，取值范围0~16
    // 180：旋转的角度，取值范围-360~360；该角度是相对初始摆位的角度
    // false: 代表不开启多线程；true：开启多线程
    ChassisControlSTS.TurnTo(10, 180, false); 

    // 进阶底盘运动相关====================================================

    // ->角度校准直线运动调用
    // 16：运动的速度，取值范围0~16
    // 600：运动的长度，是电机编码器的编码值。100大约为一拳的距离
    // 90：运动过程中将自动对齐相对初始摆位的90度（如果不传入此参数，将自动对齐运动开始角度（直线修正））
    // ‘n’：运动的方向，‘n’向前；‘u’向后
    ChassisControlSTS.Move(16, 600, 90, 'n');  // 默认不开启多线程
    ChassisControlSTS.Move(16, 600, 90, 'n', true);  // 开启多线程

    // ->在运动过程中进行其他操作
    ChassisControlSTS.Move(16, 2000, 'n', true);  // 开启多线程
    ChassisControlSTS.WaitVelocityArrive(500);        // 等待运动500之后
    digitalE.set(1);                                  // 设置E到1状态
    ChassisControlSTS.WaitVelocityArrive(1500);       // 等待运动到1500之后
    digitalE.set(0);                                  // 设置E到0状态
    ChassisControlSTS.WaitMoveFinish();               // 等待多线程运动完成

    // ->使直走拥有多段速度
    ChassisControlSTS.Move(10, 2000, 'n', true);  // 发布多线程运动
    ChassisControlSTS.WaitVelocityArrive(500);    // 等待行走到500
    ChassisControlSTS.ResetMaxSpeed(1.5);         // 修改最大速度
    ChassisControlSTS.WaitMoveFinish();           // 等待多线程运动执行完成

    // ->检测是否碰撞
    ChassisControlSTS.Move(6, 2000, 'n', true);  // 下发多线程运动
    wait(100, msec);  // 进行少许等待，防止车辆还未起步，运动就被取消
    // 在运动过程中检测
    while (ChassisControlSTS.IsMoving()){
      // ChassisControlSTS.GetVelocity_mean() 用于获取车辆底盘电机的平均转速
      if (ChassisControlSTS.GetVelocity_mean() < 50){
          ChassisControlSTS.StopMoving();  // 停止当前多线程运动
      } 
      wait(10, msec);
    }
    ChassisControlSTS.WaitMoveFinish();  // 多线程运动保险

    // ->在直走中途更改对齐角度
    ChassisControlSTS.Move(10, 2000, 'n', true);  // 发布多线程运动
    ChassisControlSTS.WaitVelocityArrive(1000);   // 等待运动至1000
    ChassisControlSTS.ResetStraightAngle(45);     // 修改对齐角度为45度
    ChassisControlSTS.WaitMoveFinish();           // 等待多线程运动执行完成

    // ->修改单次PPS参数的运动
    // 这些参数只会影响下一次运动，运动完成后自行恢复默认参数
    ChassisControlSTS.min_move_speed = 1.0;       // 修改最小直走速度
    ChassisControlSTS.min_turn_speed = 2.0;       // 修改旋转最小速度
    ChassisControlSTS.Move_PPS = {0.01, 0.5, 5};  // 修改直走PPS参数
    ChassisControlSTS.Turn_PPS = {0.1, 0.5, 5};   // 修改旋转PPS参数

    ChassisControlSTS.Move(10, 2000, 'n');
    ChassisControlSTS.Turn(10, 90, 'l');
    ChassisControlSTS.TurnTo(10, 90, false);   // TurnTo函数使用Turn同样的参数

    // ->修改运动其他参数
    // 这些修改会影响后续的所有运动，不会自行恢复
    ChassisControlSTS.Stuck_time_out  = 300;             // 电机卡死保护，默认1000毫秒
    ChassisControlSTS.Stuck_threshold = 100;             // 电机卡死认定阈值，单位dps
    ChassisControlSTS.Over_count      = 10;              // 到位计数当前次数后，才会退出
    ChassisControlSTS.move_allowable_error = 10;         // 直行允许误差（电机编码值）
    ChassisControlSTS.turn_allowable_error = 1.5;        // 旋转允许误差（度）
    ChassisControlSTS.maxinum_speed_acceleration = 0.5;  // 最大加速度

    ChassisControlSTS.Move(10, 2000, 'n');  // 后续所有运动都会受到影响


    // ->复刻遥控 读取文件（需要内存卡）
    // FileName：默认的文件储存路径
    // 999999999：运行的时间。文件读完或者时间到了会退出程序，因此这里选无限时间
    AutoRemoteControl(FileName, 999999999);


}