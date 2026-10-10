#include "encoder.h"
#include "pglobals.h"
#include <math.h>

#define INA219_ADDR 0x36 // I2C address
#define INA219_REG_CONFIG 0x00 // config register address
#define INA219_REG_CURRENT_HIGH 0x0C // current register
#define INA219_REG_CURRENT_LOW 0x0D // current register

#define SDA_PIN 18
#define SCL_PIN 19
#define I2C_INST i2c1


// private functions
void writeINA219(int reg, int value);
uint16_t readINA219(unsigned char reg);

volatile uint16_t theta_offset = 0;
void init_encoder(){
    // init I2C on GP14 (SDA) and GP15 (SCL) on I2C1
    i2c_init(I2C_INST, 400 * 1000); // baud of 400kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    theta_offset = 4096 - readINA219(INA219_REG_CURRENT_HIGH);

}

float read_encoder(){
    uint16_t  theta_raw = readINA219(INA219_REG_CURRENT_HIGH) + theta_offset;

     if (theta_raw > 4096)
         theta_raw -= 4096; //wrap
    
    // ma = value / 3.0;
    float theta = - ((theta_raw * (2.0f * M_PI / 4096.0f)) - M_PI);
    theta -= 0.025f;
    return theta;
}

// write 2 bytes
void writeINA219(int reg, int value){
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = value>>8;
    buf[2] = value&0xff;
    i2c_write_blocking(I2C_INST, INA219_ADDR, buf, 3, false);
}

// read 1 byte
uint16_t readINA219(unsigned char reg){
    uint8_t buffer[2];
    i2c_write_blocking(I2C_INST, INA219_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_INST, INA219_ADDR, buffer, 2, false);
    uint16_t value = ((buffer[0] & 0x0F) << 8) | buffer[1];
    return value;
}