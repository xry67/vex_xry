#include "vex.h"
#include "robot-config.h"
#include "ControllerWindow.h"
#include "initialize.h"
#include "ChassisControl.h"

using namespace vex;

int main() {
  std::cout << "-------------------->" << std::endl;
  vexcodeInit();   // 机器人初始化

  ChassisControlSTS.Move(100, 1000, 0, 'n', true);
  wait(500, msec);
  ChassisControlSTS.Move(200, 1000, 0, 'u', true);

  // 程序运行
  MainWindow main_window = MainWindow();
  main_window.Run();

}
