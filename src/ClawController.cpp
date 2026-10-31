#include "ClawController.h"

ClawController *claw;
double liftMoveTask = 0;

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


int task_ClawController_liftMove_void()
{
    claw->LiftMove(liftMoveTask);
    return 0;
}

void task_ClawController_liftMove(ClawController *claw_, double liftMove)
{
    claw = claw_;
    liftMoveTask = liftMove;
    vex::task T(task_ClawController_liftMove_void);
}