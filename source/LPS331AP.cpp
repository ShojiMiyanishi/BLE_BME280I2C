/*
*   LPS331AP pressure sensor Library
*
*    @author: aruaru
*    @date: August 26, 2015
*    @license: MIT license
*     
*   Permission is hereby granted, free of charge, to any person obtaining a copy
*   of this software and associated documentation files (the "Software"), to deal
*   in the Software without restriction, including without limitation the rights
*   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*   copies of the Software, and to permit persons to whom the Software is
*   furnished to do so, subject to the following conditions:
*
*   The above copyright notice and this permission notice shall be included in
*   all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
*   THE SOFTWARE.
*
*/
#include "mbed-drivers/mbed.h"
//using minar::Scheduler;
//using mbed::util::FunctionPointer0;

// Some part of the code is adapted from Adafruit LPS331AP library
#include "LPS331AP.h"
#define LPS331AP_RES_CONF 0x10
#define LPS331AP_CTRL_REG1 0x20
#define LPS331AP_PRESS_POUT_XL_REH    0x28
#define LPS331AP_PRESS_OUT_L  0x29
#define LPS331AP_PRESS_OUT_H    0x2a
#define LPS331AP_TEMP_OUT_L  0x2b
#define LPS331AP_TEMP_OUT_H    0x2c

LPS331AP::LPS331AP(I2C &i2c_obj,int8_t addr)
    :
i2c(i2c_obj),
address(addr)
{
    initialize();
}
void LPS331AP::writeByte(uint8_t regAddress, uint8_t data)
{
    char data_write[2];
    data_write[0]=regAddress;           // I2C sends MSB first. Namely  >>|regAddress|>>|data|
    data_write[1]=data;
    i2c.write(address,data_write,2);  // i2c.write(int address, char* data, int length, bool repeated=false);

}

int LPS331AP::readByte( uint8_t regAddress)
{
    char data_read[1];                   // will store the register data
    char data_write[1];
    data_write[0]=regAddress;
    i2c.write(address,data_write,1);   // repeated = true
    i2c.read(address,data_read,1);     // read the data and stop
    return data_read[0];

}

bool LPS331AP::initialize()
{
    int data;
    data=readByte(LPS331AP_WHOAMI);
    if (data!=0xbb){
        return false;
    }
    writeByte(LPS331AP_CTRL_REG1,0x90);//パワーダウンoff アクティブモード
    writeByte(LPS331AP_RES_CONF,0xfa);//temp avr 16,press avr 16
    offset=42.5;
    return true;

}
float LPS331AP::getPressure()
{
    uint8_t data[3];
    data[0]=readByte(LPS331AP_PRESS_POUT_XL_REH);
    data[1]=readByte(LPS331AP_PRESS_OUT_L);
    data[2]=readByte(LPS331AP_PRESS_OUT_H);
    return ((data[2]<<16)+(data[1]<<8)+data[0])/4096.;
}
float LPS331AP::getTemperature()
{
    uint8_t data[2]={0,0};
    uint16_t data16;
    float temp;
    data[0]=(uint8_t)readByte( LPS331AP_TEMP_OUT_L);
    data[1]=(uint8_t)readByte( LPS331AP_TEMP_OUT_H);
    data16=(data[1]<<8)|data[0];
    if((data16 & 0x8000)==0){
        temp=offset+data16/480;
    }else{
        temp=offset-(((~data16) & 0xffff)+1)/480;
    }
    return temp;

}
