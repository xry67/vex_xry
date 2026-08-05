#include "ClawController.h"

ClawController *claw;
int task_ClawController_modMove_void()
{
    claw->modMove();
    return 0;
}
void task_ClawController_modMove(ClawController *claw_)
{
    claw = claw_;
    vex::task T(task_ClawController_modMove_void);
}