#include "ClawController.h"

ClawController *claw;

double lift_target = 0;

int task_ClawController_reversalMove_void()
{
    claw->reversalMove();
    return 0;
}
void task_ClawController_reversalMove(ClawController *claw_)
{
    claw = claw_;
    vex::task T(task_ClawController_reversalMove_void);
}

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
    claw->LiftMove_Normalized(lift_target);
    return 0;
}

void task_ClawController_liftMove(ClawController *claw_, double lift_target_){
    claw = claw_;
    lift_target = lift_target_;
    vex::task T(task_ClawController_liftMove_void);
}