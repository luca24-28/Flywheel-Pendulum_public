#ifndef HBRIDGE
#define HBRIDGE
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "encoder.h"
#include "pina219.h"

// A matrix: 4x4
#define A11  1.0001f
#define A12  0.0020f
#define A13  3.4881e-7f
#define A14 -7.7183e-6f

#define A21  0.1054f
#define A22  1.0001f
#define A23  3.6217e-4f
#define A24 -0.0040f

#define A31 -0.1052f
#define A32 -1.0530e-4f
#define A33  0.9970f
#define A34  0.0331f

#define A41  3.6950e-4f
#define A42  3.5587e-7f
#define A43 -0.0036f
#define A44 -1.2093e-4f


// B matrix: 4x1
#define B1 -3.4309e-5f
#define B2 -0.0356f
#define B3 0.2941f
#define B4 0.3583f


// LQR gain K: 1x4
#define K1 -11.2665f
#define K2 -2.4730f
#define K3 -0.0198f
#define K4 0.0093f


// Kalman gain L: 4x2
#define L11 0.13645518f
#define L12 -3.5379e-6f
#define L13 -1.5695e-7f

#define L21 4.9962701f
#define L22 1.1219e-4f
#define L23 -5.1023e-6f

#define L31 -0.04780974f
#define L32 0.08116933f
#define L33 -0.00330118f

#define L41 1.7396e-4f
#define L42 -2.7077e-4f
#define L43 1.2152e-5f

void init_hbridge();
void set_duty(float duty);
void init_position_control();
void print_gains();

#endif