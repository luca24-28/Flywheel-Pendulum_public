#include "fwencoder.h"
#include <math.h>
#include "pglobals.h"

PIO pio = pio0;
const uint sm = 0;

// encoder pins must be next to each other
#define ENCODER_FIRST_PIN 2
volatile int32_t encoderOffset = 0;

void initfwEncoder(){
    // init the PIO for the encoder

    pio_add_program(pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio, sm, ENCODER_FIRST_PIN, 0);
}

int getEncoder(){
    // return the encoder counts
    return quadrature_encoder_get_count(pio, sm) - encoderOffset;
}

// set the offset to zero the encoder
void setEncoderToZero(){
    encoderOffset = quadrature_encoder_get_count(pio, sm);
}

#define VEL_N 10
#define ENCODER_CPR 180.0f
#define DT 0.002f

int32_t enc_hist[VEL_N] = {0};
int vel_idx = 0;
volatile int posit_prev = 0;
volatile float ro = 0;
volatile bool ro_new = 0;
bool fwspeed(__unused struct repeating_timer * t){
    int32_t count_now = getEncoder();

    int32_t count_old = enc_hist[vel_idx];

    enc_hist[vel_idx] = count_now;

    vel_idx++;
    if (vel_idx >= VEL_N)
        vel_idx = 0;

    int32_t delta_count = count_now - count_old;

    ro = delta_count
       * (2.0f * M_PI / ENCODER_CPR)
       / (VEL_N * DT);
    ro = -ro;
    return true;
}

struct repeating_timer timer_3;
void initRo(){   
add_repeating_timer_ms(-2, fwspeed, NULL, &timer_3);
}
