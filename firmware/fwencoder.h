#ifndef ENCODER_H
#define ENCODER_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "quadrature_encoder.pio.h"

void initfwEncoder();
int getEncoder();
void setEncoderToZero();
void initRo();

#endif