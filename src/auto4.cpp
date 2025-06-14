#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto4(){

    for (int i=0; i<=360; i+=30){
        ChassisControlSTS.Turn(7, i, 'l');
    }
}