#include "pglobals.h"
#include "phbridge.h"
#include "fwencoder.h"

#define IN1 6
#define IN2 7
#define WRAP 3750
#define M .115f
#define G 9.81f
#define LENGTH .0556f
#define J_0 .001f
#define K_ENERGY 150.0f
#define DT 0.002f
#define ALPHA 0.9

#define DUTY_SWING 35.0f
#define PRELOAD_VEL 1.2f
 

void set_duty(float duty){
    if (duty > 100.0f) duty = 100.0f;
    if (duty < -100.0f) duty = -100.0f;

    if (duty<0){duty = 100 + duty;
    pwm_set_gpio_level(IN1, WRAP); pwm_set_gpio_level(IN2, WRAP * (duty/100.0));}
    else {duty = 100 - duty;
    pwm_set_gpio_level(IN2, WRAP); pwm_set_gpio_level(IN1, WRAP * (duty/100.0));}
}

void init_hbridge(){
gpio_set_function(IN1, GPIO_FUNC_PWM); // Set the LED Pin to be PWM
gpio_set_function(IN2, GPIO_FUNC_PWM);
uint slice_num = pwm_gpio_to_slice_num(IN1); // Get PWM slice number
// the clock frequency is 150MHz divided by a float from 1 to 255
float div = 2; // must be between 1-255
pwm_set_clkdiv(slice_num, div); // sets the clock speed
uint16_t wrap = WRAP; // when to rollover, must be less than 65535
// set the PWM frequency and resolution
// this sets the PWM frequency to 150MHz / div / wrap
pwm_set_wrap(slice_num, wrap);
pwm_set_enabled(slice_num, true); // turn on the PWM
pwm_set_gpio_level(IN1, 0);
pwm_set_gpio_level(IN2, 0);
}


// State estimate
volatile float xhat[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// Discrete state-space matrices from MATLAB
const float Ad[4][4] = {
    {A11, A12, A13, A14},
    {A21, A22, A23, A24},
    {A31, A32, A33, A34},
    {A41, A42, A43, A44}
};

const float Bd[4] = {
    B1,
    B2,
    B3,
    B4
};

// Steady-state Kalman gain from MATLAB
const float L[4][3] = {
    {L11, L12, L13},
    {L21, L22, L23},
    {L31, L32, L33},
    {L41, L42, L43}
};

// LQR gain
const float K[4] = {
    K1,
    K2,
    K3,
    K4
};
volatile float state_obsv[4][DATASIZE];
volatile float raw_data[5][DATASIZE];
volatile int k = 0;
volatile float angle_error;
volatile float current_error;
volatile float ro_error;
volatile float xpred[4];
volatile float d = 0;
volatile float u_p;
volatile float read_angle;
volatile float energy;
volatile float direction;
volatile int dir = 0;
volatile float theta = 0;
volatile float theta_prev = 0;
volatile float dtheta;
volatile float theta_dot;
volatile float i;
volatile float u1;
volatile float u2;
volatile float u3;
volatile float u4;
volatile float theta_now;
volatile float theta_dot_raw;
volatile bool flag = 0;
bool position_control(__unused struct repeating_timer * t){
    switch(motor_mode){
    case IDLE:{
        break;
    }
    case TRACK:
    {
        //state prediction
        xpred[0] =
        Ad[0][0] * xhat[0] +
        Ad[0][1] * xhat[1] +
        Ad[0][2] * xhat[2] +
        Ad[0][3] * xhat[3] +
        Bd[0] * (d/100.0f);

         xpred[1] =
        Ad[1][0] * xhat[0] +
        Ad[1][1] * xhat[1] +
        Ad[1][2] * xhat[2] +
        Ad[1][3] * xhat[3] +
        Bd[1] * (d/100.0f);

         xpred[2] =
        Ad[2][0] * xhat[0] +
        Ad[2][1] * xhat[1] +
        Ad[2][2] * xhat[2] +
        Ad[2][3] * xhat[3] +
        Bd[2] * (d/100.0f);

         xpred[3] =
        Ad[3][0] * xhat[0] +
        Ad[3][1] * xhat[1] +
        Ad[3][2] * xhat[2] +
        Ad[3][3] * xhat[3] +
        Bd[3] * (d/100.0f);

//get errors
theta = read_encoder();
i = read_amp();
angle_error   = theta - xpred[0];
ro_error = ro - xpred[2];
current_error = i     - xpred[3];


//state correction
xhat[0] = xpred[0]
        + L[0][0] * angle_error
        + L[0][1] * ro_error
        + L[0][2] * current_error;

xhat[1] = xpred[1]
        + L[1][0] * angle_error
        + L[1][1] * ro_error
        + L[1][2] * current_error;

xhat[2] = xpred[2]
        + L[2][0] * angle_error
        + L[2][1] * ro_error
        + L[2][2] * current_error;

xhat[3] = xpred[3]
        + L[3][0] * angle_error
        + L[3][1] * ro_error
        + L[3][2] * current_error;

//apply LQR gains
u1 = -K[0]*xhat[0];
u2 = -K[1]*xhat[1];
u3 = -K[2]*xhat[2];
u4 = -K[3]*xhat[3];

//calculate duty
d = 100.0f * (u1 + u2 + u3 + u4);

// diagnostic raw pendulum velocity
static float theta_prev = 0.0f;

theta_now = theta;

float dtheta = theta_now - theta_prev;

while (dtheta > M_PI)
    dtheta -= 2.0f*M_PI;

while (dtheta < -M_PI)
    dtheta += 2.0f*M_PI;

theta_dot_raw = dtheta / DT;

theta_prev = theta_now;

 const float DEADZONE = 5.0f;   // percent duty

    if (d > 0.0f) {
        d += DEADZONE;
    }
    else if (d < 0.0f) {
        d -= DEADZONE;
    }
//d = -100.0f * (K[0]*xhat[0] + K[1]*xhat[1] + K[2]*xhat[2] + K[3]*xhat[3]);
// duty saturation
if (d > 100.0f) d = 100.0f;
    if (d < -100.0f) d = -100.0f;

set_duty(d);

if (theta_dot_raw >= 20.0f){
    theta_dot_raw = 0.0f;
}
if (theta_dot_raw <= -20.0f){
    theta_dot_raw = 0.0f;
} 
//data logging
if (k <= DATASIZE - 1){
    raw_data[0][k] = theta;
    raw_data[1][k] = theta_dot_raw;
    raw_data[2][k] = ro;
    raw_data[3][k] = i;
    raw_data[4][k] = d;

    state_obsv[0][k] = xhat[0];
    state_obsv[1][k] = xhat[1];
    state_obsv[2][k] = xhat[2];
    state_obsv[3][k] = xhat[3];
    k++;
}
//logging finished, dump array
if (k >= DATASIZE){
   flag = 1;}

        break;
    }
    case SWING:{

theta = read_encoder();

dtheta = theta - theta_prev;

if (dtheta > M_PI)
    dtheta -= 2.0f * M_PI;
else if (dtheta < -M_PI)
    dtheta += 2.0f * M_PI;

float theta_dot_raw = dtheta / DT;

theta_dot =
    ALPHA * theta_dot +
    (1.0f - ALPHA) * theta_dot_raw; //low pass filter

theta_prev = theta;
energy = (0.5f * J_0 * theta_dot * theta_dot
    + M * G * LENGTH * (cosf(theta) - 1.0f)) - 0.033;

    direction = (theta_dot >= 0.0f) ? 1.0f : -1.0f;

    d =
     K_ENERGY * energy * theta_dot;

    


if (d > 100.0f) d = 100.0f;
    if (d < -100.0f) d = -100.0f;


    set_duty(d);

        float posit = read_encoder();
        if ((posit < 0.1f) && (posit > -0.1f)){ // small angle range
            motor_mode = TRACK; // switch to controller
        }
    break;
    }
}
return true;
}

struct repeating_timer timer_2;
void init_position_control(){   
add_repeating_timer_ms(-2, position_control, NULL, &timer_2);
}

void print_gains(){
    printf("Bd: %e %e %e %e\n",
       B1, B2, B3, B4);

printf("K: %e %e %e %e\n",
       K1, K2, K3, K4);

printf("A:\n");
printf("%e %e %e %e\n", A11, A12, A13, A14);
printf("%e %e %e %e\n", A21, A22, A23, A24);
printf("%e %e %e %e\n", A31, A32, A33, A34);
printf("%e %e %e %e\n", A41, A42, A43, A44);

printf("L:\n");
printf("%e %e % e\n", L11, L12, L13);
printf("%e %e %e\n", L21, L22, L23);
printf("%e %e %e\n", L31, L32, L33);
printf("%e %e %e\n", L41, L42, L43);
}