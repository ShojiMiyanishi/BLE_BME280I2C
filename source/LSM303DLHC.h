
#ifndef __LSM303DLHC_H
#define __LSM303DLHC_H
#include "mbed-drivers/mbed.h"



class LSM303DLHC {
    public:
        /** Create a new interface for an LSM303DLHC
         *
         * @param sda is the pin for the I2C SDA line
         * @param scl is the pin for the I2C SCL line
         */
        LSM303DLHC(PinName sda, PinName scl);

          /** Create a BME280 instance
         *  which is connected to specified I2C pins with specified address
         *
         * @param i2c_obj I2C object (instance)
         */
        LSM303DLHC(I2C &i2c_obj);

        /** read the raw accelerometer and compass values
         *
         * @param ax,ay,az is the accelerometer 3d vector, written by the function
         * @param mx,my,mz is the magnetometer 3d vector, written by the function
         */
        bool read(float *ax, float *ay, float *az, float *mx, float *my, float *mz);
        virtual ~LSM303DLHC();


    private:
        I2C *i2c_p;
        I2C &i2c;
        //_LSM303;
        void init();
         
        float ax, ay, az;
        float mx, my, mz;         
         
        bool write_reg(int addr_i2c,int addr_reg, char v);
        bool read_reg(int addr_i2c,int addr_reg, char *v);
        bool recv(char sad, char sub, char *buf, int length);
};

#endif
