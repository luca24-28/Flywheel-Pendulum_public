// function prototypes for the INA219 current sensor
#include "pina219.h"
#include "pglobals.h"
#include "phbridge.h"

float read_amp() {
    int adc_raw = adc_read();
    float current = -((adc_raw * (3.3f/4095)) - 1.7325f);
    return current;
}



void init_amp(){
adc_init();

    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    adc_select_input(0);

}