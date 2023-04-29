#ifndef SERVO_H
#define SERVO_H
#include <stdint.h>


#define A_PIN 3    // P0.03 (Pin1)
#define B_PIN 4    // P0.04 (Pin2)

void delay_ms(uint32_t ms);
void servo_init(void);
void servo_out(int traj, int dirn, int speed);
/*
traj=1 => straight; dirn=1 => frwd
traj=0 => circle; dirn=1 => CCW
speed levels limited to 6
*/ 



#endif  /* PWM_H */