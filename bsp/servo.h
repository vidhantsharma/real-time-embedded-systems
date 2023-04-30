#ifndef SERVO_H
#define SERVO_H
#include <stdint.h>

#define B_PIN 3    // P0.03 (Pin1)
#define A_PIN 4    // P0.04 (Pin2)

void servo_init(void);
void servo_out(int dirA, int vA, int dirB, int vB);


#endif  /* PWM_H */