//**********************
// HDC1000.h for mbed
//
// (C)Copyright 2015 All rights reserved by Y.Onodera
// http://einstlab.web.fc2.com
//**********************
#ifndef HDC1000_H_
#define HDC1000_H_

#define HDC1000_ADDR            0x80
#define HDC1000_TEMP            0x00
#define HDC1000_HUM             0x01
#define HDC1000_CONFIG          0x02
#define HDC1000_SERIAL0         0xFB
#define HDC1000_SERIAL1         0xFC
#define HDC1000_SERIAL2         0xFD
#define HDC1000_MAMUFUCTUREER   0xFE
#define HDC1000_DEVICE          0xFF

#include "mbed-drivers/mbed.h"
#include "typedef.h"
/****
 * staticな関数は、インスタンスはクラスで１個。クラスオブジェクトの変数は見えない。
 *　複数のセンサーに対応数のは難しいので、１個だけに対応する。
 */

class HDC1000{
public:
    HDC1000 (PinName sda, PinName scl):
        	    i2c_p(new I2C(sda, scl )),
        	    i2c(*i2c_p)
    {
        init();
    }
    /*
     *
     */
    HDC1000 (I2C &i2c_obj)    :
	    i2c_p(new I2C(i2c_obj )),
	    i2c(*i2c_p)
    {
        init();
    }

    void get();
    float humidity();
    float temperature();
    /*
     *  interval:更新間隔　秒単位
     */
    void init();

private:

protected:
    I2C *i2c_p;
    I2C &i2c;
    char buf[8];

};

#endif /* HDC1000_H_ */


