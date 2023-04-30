#include "servo.h"

// Controller commands
void forward(int v);
void reverse(int v);
void turn_right(int v);
void turn_left(int v);
void stop();

// For testing motors
void motor_A(void);
void motor_B(void);