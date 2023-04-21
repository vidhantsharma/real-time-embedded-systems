#ifndef RNG_H
#define RNG_H
#include <stdint.h>

void rng_init(void);
int getRngRaw(void);
int getRng(int min, int max);

#endif // RNG_H