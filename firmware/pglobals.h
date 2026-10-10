#ifndef GLOBALS
#define GLOBALS
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"

#define DATASIZE 3000

typedef enum {IDLE, SWING, ITEST, HOLD, TRACK} mode_t;
extern volatile mode_t motor_mode;
extern volatile float d;
extern volatile bool flag;
extern volatile float energy;
extern volatile float direction;
extern volatile float theta;
extern volatile float theta_dot;
extern volatile float xpred[4];
extern volatile float xhat[4];
extern volatile float angle_error;
extern volatile float ro;
extern volatile bool ro_new;
extern volatile float ro_error;
extern volatile float current_error;
extern volatile float u1;
extern volatile float u2;
extern volatile float u3;
extern volatile float u4;
extern volatile float u;
extern volatile float theta_now;
extern volatile float theta_dot_raw;
extern volatile float state_obsv[4][DATASIZE];
extern volatile float raw_data[5][DATASIZE];
#endif