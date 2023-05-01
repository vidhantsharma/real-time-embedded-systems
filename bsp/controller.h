#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "servo.h"

void turn_ctrlr(int dirn, float ang_des, float ang_curr);
void move_ctrlr(int dirn, float orient, float ang_curr);

void print_float_ctrl(float data, int dec_places);

#endif //_CONTROLLER_H_