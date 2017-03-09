//**********************
// HDC1000.cpp for mbed
//
// HDC1000 hdc1000(P0_5,P0_4);
// or
// I2C i2c(P0_5,P0_4);
// HDC1000 hdc1000(i2c);
//
// (C)Copyright 2015 All rights reserved by Y.Onodera
// http://einstlab.web.fc2.com
//**********************
#include "minar/minar.h"
#include "mbed-drivers/mbed.h"
#include "core-util/FunctionPointer.h"
//#include "i2cMemory.h"
#include "HDC1000.h"

void HDC1000::get()
{

    // Trigger
    buf[0] = 0x00;  // Pointer
    i2c.write(HDC1000_ADDR, buf, 1);   // with stop

    // Wait 6.35ms + 6.5ms
    wait_ms(20);

    // get data
    i2c.read( HDC1000_ADDR, buf, 4);

}

float HDC1000::humidity()
{
    WORD_VAL hum;
    float h;
    // get hum
    get();
    hum.Val=0;
    hum.byte.HB=buf[2];
    hum.byte.LB=buf[3];
    h=hum.Val;
    h=h/0x10000*100.0;
    return h;
}
float HDC1000::temperature()
{
    WORD_VAL temp;
    float t;

    // get temp
    get();
    temp.Val=0;
    temp.byte.HB=buf[0];
    temp.byte.LB=buf[1];
    t=temp.Val;
    t=t/0x10000*165.-40;
    return t;

}
void HDC1000::init()
{

    wait_ms(15);

    // Set configuration
    buf[0] = 0x02;  // Pointer
    buf[1] = 0x10;  // High byte
    buf[2] = 0x00;  // Low byte
    i2c.write(HDC1000_ADDR, buf, 3);

}



