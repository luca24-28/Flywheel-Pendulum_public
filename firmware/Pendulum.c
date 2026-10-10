
#include "pglobals.h"
#include "phbridge.h"
#include "encoder.h"
#include "pina219.h"
#include "fwencoder.h"
volatile mode_t motor_mode = IDLE;
int main()
{
    stdio_init_all(); //initialize all sensors and interrupts
    init_hbridge();
    initfwEncoder();
    setEncoderToZero();
    initRo();
    init_encoder();
    init_amp();
    set_duty(0.0f);
    init_position_control();

char input; //wait for input from python client function
while (input != 's'){
    scanf(" %c", &input);
}
    set_duty(60);//initial swing kick
    sleep_ms(1000);
    motor_mode = SWING;
    //wait for catch
    while (flag == 0){
        tight_loop_contents();
    }
        for (int j = 0; j < DATASIZE; j++){
        printf("%f %f %f %f %f %f %f %f %f\n", 
            raw_data[0][j], 
            raw_data[1][j], 
            raw_data[2][j], 
            raw_data[3][j], 
            raw_data[4][j],
            state_obsv[0][j],
            state_obsv[1][j],
            state_obsv[2][j],
            state_obsv[3][j]);
        sleep_ms(1);}
         
       while(true){
        tight_loop_contents();
        printf("x %.3f %.3f %.3f %.3f | "    //diagnostic printing loop
       "u %.3f %.3f %.3f %.3f | total %.3f\n",
       xhat[0], xhat[1], xhat[2], xhat[3],
       u1, u2, u3, u4, d);
       sleep_ms(5);
       }
    
    }
