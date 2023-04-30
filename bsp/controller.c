#include "controller.h"

int FORWARD = 1;
int REVERSE = -1;
int STOP = 0;

// Controller commands
void forward(int v)
{
    servo_out(FORWARD, v, REVERSE, v);
}

void reverse(int v)
{
    servo_out(REVERSE, v, FORWARD, v);
}

void turn_left(int v)
{
    servo_out(FORWARD, v, FORWARD, v);
}

void turn_right(int v)
{
    servo_out(REVERSE, v, REVERSE, v);
}

// For testing motors
void motor_A(void)
{
    servo_out(FORWARD,1,STOP,1);
}

void motor_B(void)
{
    servo_out(STOP,1,FORWARD,1);
}

void stop(){
    servo_out(STOP,1,STOP,1);
}