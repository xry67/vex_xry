#include "vex.h"
#include "robot-config.h"
#include "ControllerWindow.h"
#include "initialize.h"
#include "ChassisControl.h"
#include "RemoteControl.h"

using namespace vex;

int main()
{
  std::cout << "-------------------->" << std::endl;

  // while (1)
  // {
  //   if (bumper_elevator.pressing())
  //   {
  //     Brain.Screen.printAt(100, 150, "++++");
  //   }
  //   else
  //   {
  //     Brain.Screen.printAt(100, 150, "----");
  //   }
  //   wait(10, msec);
  // }

  vexcodeInit(); // 机器人初始化

  Competition.autonomous(AutoControl); // 自动程序加入

  // 程序运行
  MainWindow main_window = MainWindow();
  main_window.Run();
}
