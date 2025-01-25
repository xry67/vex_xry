#include "vex.h"
#include "AutoFun.h"

using namespace vex;

int ControllerScreenWidth = 210;  // 屏幕宽度
int RemoteControlMod      = 0;    // 遥控模式


// A global instance of brain used for printing to the V5 brain screen
std::vector<AutoFun> AutoFunctions = {AutoFun("自动程序1", auto1), AutoFun("自动程序2", auto2)};

brain Brain; 
controller Controller = controller();


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}