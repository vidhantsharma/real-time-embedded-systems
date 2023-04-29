#include <stdio.h>
#include <stdlib.h>

#include "controller.h"
#include "servo.h"

/* 
Done: move straight(dirn, speed), rotate abt center(dirn,speed)

To Do: trace trajectory (only on level plane)
integrate X,Y acceleration to get position, find 
initial position 
*/

void turn_ctrlr(float ang_des, float ang_curr){ // Only turning
    /*
    servo_out(int traj, int dirn, int speed );
    traj=1 => straight; dirn=1 => frwd; dirn=-1 => bwd;
    traj=0 => circle; dirn=1 => CCW; dirn=-1 => CW;
    dirn=0 => stop
    speed levels limited to 6
    */ 
    float tolr = 3.0;

    if(abs(ang_des - ang_curr) >= tolr){
        servo_out(0,1,2);
    }
    else{
        printf("stopping");
        servo_out(0,0,2);
    }
}

void move_ctrlr(int dirn){ // Only turning
    servo_out(1,dirn,2);
}

