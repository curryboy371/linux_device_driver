#ifndef MY_I2C_SAMPLE_H
#define MY_I2C_SAMPLE_H

#include <linux/types.h>


struct my_i2c_session {
    uint8_t slave_addr;         
};

#endif // MY_I2C_SAMPLE_H