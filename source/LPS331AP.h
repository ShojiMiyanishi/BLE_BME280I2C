/*     
*   LPS331AP pressure sensor Library
*
*    @author: aruaru
*    @date: August 26, 2015
*    @license: MIT license

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

#ifndef LPS331AP_H
#define LPS331AP_H

#include "mbed-drivers/mbed.h"

#define LPS331AP_ADDRESS (0xb8)
#define LPS331AP_WHOAMI 0x0f
class LPS331AP
{
    public:
    
        /** Create a LPS331AP instance
         *  which is connected to specified I2C pins with specified address
         *
         * @param i2c_obj I2C object (instance)
         */
        LPS331AP(I2C &i2c_obj,int8_t addr=LPS331AP_ADDRESS);
        //LPS331AP(PinName sda, PinName scl,int8_t addr=LPS331AP_ADDRESS);
    
        /** Destructor of LPS331AP
         */
        //virtual ~LPS331AP();
        bool initialize();
        float getPressure();
        float getTemperature();
        int readByte( uint8_t regAddress );
        void setOffset(float data)
        {offset=data;};
        void writeByte( uint8_t regAddress, uint8_t data);

    private:
        I2C         &i2c;
        float offset;
        uint8_t address;

};

#endif
