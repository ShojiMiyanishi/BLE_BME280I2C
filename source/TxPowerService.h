/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __BLE_TXPOWER_SERVICE_H__
#define __BLE_TXPOWER_SERVICE_H__

#include "ble/BLE.h"
extern DigitalOut led;
/**
* @class TxPowerService
* @brief BLE TxPower Service. This service displays the txPower level from 0% to 100%, represented as an 8bit number.
* Service:  https://developer.bluetooth.org/gatt/services/Pages/ServiceViewer.aspx?u=org.bluetooth.service.txPower_service.xml
* TxPower Level Char:  https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.txPower_level.xml
*/
class TxPowerService {
public:
    /**
     * @param[in] _ble
     *               BLE object for the underlying controller.
     * @param[in] level
     *               8bit batterly level. Usually used to represent percentage of batterly charge remaining.
     */
    TxPowerService(BLE &_ble, int8_t level = 4) :
        ble(_ble),
        txPowerLevel(level),
        txPowerLevelCharacteristic(GattCharacteristic::UUID_TX_POWER_LEVEL_CHAR, &txPowerLevel,
        		GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE || GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ) {

        GattCharacteristic *charTable[] = {&txPowerLevelCharacteristic};
        GattService         txPowerService(GattService::UUID_TX_POWER_SERVICE, charTable, sizeof(charTable) / sizeof(GattCharacteristic *));

        ble.addService(txPowerService);
        ble.onDataWritten(this, &TxPowerService::onDataWritten);
    }

    /**
     * Note: TX and RX characteristics are to be interpreted from the viewpoint of the GATT client using this service.
     */
    uint16_t getHandle() {
        return txPowerLevelCharacteristic.getValueAttribute().getHandle();
    }

    /**
     * @brief Update the txPower level with a new value. Valid values lie between 0 and 100,
     * anything outside this range will be ignored.
     *
     * @param newLevel
     *              Update to txPower level.
     */
    /*
     * Radio transmit power in dBm (accepted values are
     * -40, -30, -20, -16, -12, -8, -4, 0, and 4 dBm).
    */
    int8_t get(void)
    {
    	return txPowerLevel;
    }
protected:
    /**
     * This callback allows the UART service to receive updates to the
     * txCharacteristic. The application should forward the call to this
     * function from the global onDataWritten() callback handler; if that's
     * not used, this method can be used as a callback directly.
     */
    void onDataWritten(const GattWriteCallbackParams *params) {
        if (params->handle == getHandle()) {
            if(params->len==1){
            	int8_t newLevel=params->data[0];
            	if (newLevel>=4)newLevel=4;
            	else if(newLevel>=0)newLevel=0;
            	else if(newLevel>=-4)newLevel=-4;
            	else if(newLevel>=-8)newLevel=-8;
            	else if(newLevel>=-12)newLevel=-12;
            	else if(newLevel>=-16)newLevel=-16;
            	else if(newLevel>=-20)newLevel=-20;
            	else if(newLevel>=-30)newLevel=-30;
            	else newLevel=-40;
				memcpy(&txPowerLevel, (uint8_t*)&newLevel, 1);
		        ble.gap().setTxPower(txPowerLevel);
		        //Serial& pc = get_stdio_serial();
		    	//pc.printf("TX:%d\n",txPowerLevel);
		        ble.gattServer().write(txPowerLevelCharacteristic.getValueHandle(), (uint8_t*)&txPowerLevel, 1);
		    	led=!led;
            }
        }
    }

protected:
    /**
     * A reference to the underlying BLE instance that this object is attached to.
     * The services and characteristics will be registered in this BLE instance.
     */
    BLE &ble;

    /**
     * The current txPower level represented as an integer from 0% to 100%.
     */
    int8_t    txPowerLevel;
    /**
     * A ReadOnlyGattCharacteristic that allows access to the peer device to the
     * txPowerLevel value through BLE.
     */
    ReadWriteGattCharacteristic<int8_t> txPowerLevelCharacteristic;
};

#endif /* #ifndef __BLE_TXPOWER_SERVICE_H__*/
