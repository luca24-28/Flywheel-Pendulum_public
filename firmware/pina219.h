// function prototypes for the INA219 current sensor

#ifndef INA219_H
#define INA219_H

#include "pico/stdlib.h"
#include "hardware/adc.h"

void init_amp();
float read_amp();

#endif