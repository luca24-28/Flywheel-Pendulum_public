#ifndef ENCODER
#define ENCODER

#include "pico/stdlib.h"
#include "hardware/i2c.h"

void init_encoder();
float read_encoder();


#endif