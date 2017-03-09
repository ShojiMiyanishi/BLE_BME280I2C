/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
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

#include "mbed-drivers/mbed.h"
//mbed/mbed.h
#include "ble/BLE.h"
#include "ble_gap.h"
#include "ble_conn_params.h"
#include "ble/services/BatteryService.h"
#include "EnvironmentalService.h"
#include "TxPowerService.h"

//#include "HDC1000.h"
//#include "LPS331AP.h"
#include "bme280.h"
//#include "LSM303DLHC.h"
using namespace mbed::util;
static void callbackBlink(void);
//static iBeacon* ibeaconPtr;
DigitalOut led(P0_19,1);
DigitalOut p0_28(P0_28,0);
DigitalOut p0_29(P0_29,0);
static minar::callback_handle_t blinkCallbackHandle = 0;
bool connect=false;
//bool led;
//LSM303DLHC  lsm303(i2c);
//LPS331AP    lps331ap(i2c);
//i2cMemory i2x(i2c,4,1);
//HDC1000     hdc1000(i2c);
//RawSerial uart(P0_9, P0_11);//tx,rx
AnalogIn vcc(p0);
#include "source/nordic_sdk/components/device/nrf51_bitfields.h"
void analogIn_enable(){
    NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
}
void analogIn_disable(){
    NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Disabled;
}

const static char     deviceName[] = "BTPH";

float humidity;
float temperature;
float pressure;
static uint8_t batteryLevel;
static BatteryService* batteryServicePtr;
static int8_t txPowerLevel = 0;
static TxPowerService* txPowerServicePtr;
static EnvironmentalService*  environmentalServicePtr;
static uint16_t advInterval = 999;

static const uint16_t uuid16_list[] = {
		GattService::UUID_BATTERY_SERVICE,	//0x180F
		GattService::UUID_TX_POWER_SERVICE,	//0x1804
		GattService::UUID_ENVIRONMENTAL_SERVICE,//0x181A
};
static const uint8_t uuid[] = {0x45, 0x35, 0x56, 0x80, 0x0F, 0xD8, 0x5F, 0xB5,
							   0x51, 0x48, 0x30, 0x27, 0x06, 0x9B, 0x3F, 0xD9};

const uint8_t writeCharUUID[]      = {
    0x43,0x35,0x56,0x83,0x0F,0xD8,0x5F,0xB5,0x51,0x48,0x30,0x27,0x06,0x9B,0x3F,0xD9
};
static uint8_t writeValue[2] = {10};
ReadWriteArrayGattCharacteristic<uint8_t, sizeof(writeValue)> writeChar(writeCharUUID, writeValue);
/*
 * カスタムサービス：アドバタイスの間隔（単位ms）
 */
GattCharacteristic *characteristics[] = { &writeChar};
GattService        customService(uuid, characteristics, sizeof(characteristics) / sizeof(GattCharacteristic *));


void updateSensorValue() {
	DigitalOut power(P0_8,1);
	I2C i2c(P0_4, P0_5);
	//HDC1000     hdc1000(i2c);
	//LPS331AP    lps331ap(i2c);
	//lps331ap.initialize();
	//hdc1000.init();
	BME280 bme280(i2c);
    {
		analogIn_enable();
		int16_t battery;
		battery=vcc.read_u16();
		batteryLevel=battery*9/256;// 0.1v単位
		//batteryLevel=vcc.read_u16()/10;// 3.6Vに対する百分率
		analogIn_disable();
    }

    batteryServicePtr->updateBatteryLevel(batteryLevel);
    {
    	int count=0;
    	while(!bme280.ready() && count<10000){
    		count++;
    	}
		if(count>=10000){//センサー読み出しエラー
			environmentalServicePtr->updateHumidity(0);
			environmentalServicePtr->updateTemperature(0);
			environmentalServicePtr->updatePressure(0);
		}else{
			environmentalServicePtr->updateHumidity(bme280.humidity());//0.01%単位を100倍してint16_tに変換
			environmentalServicePtr->updateTemperature(bme280.temperature());//0.1°C単位を10倍してint16_tに変換
			environmentalServicePtr->updatePressure(bme280.pressure());//0.1Pa=0.001hPa単位を10倍してint32_tに変換
		}
    }
	power=0;
}
void errorPrint(int id,ble_error_t e){
    //Serial& pc = get_stdio_serial();
	//pc.printf("%d:%02x\n",id,e);
	int temp[2];
	temp[0]=id;temp[1]=e;
	for(int i=0 ; i<2 ; i++){
		for(int j=0 ; j<32 ; j++){
			led=(temp[i] & (1<<j) )? 1 : 0 ;
		}
	}
}
/**** GATTの custom service の設定 ****/
void callbackConnection(const Gap::ConnectionCallbackParams_t *params)
{
	connect=true;
	updateSensorValue();
	blinkCallbackHandle=minar::Scheduler::postCallback(callbackBlink).period(minar::milliseconds(1)).getHandle();
}
void callbackDisconnection(const Gap::DisconnectionCallbackParams_t *params)
{
	connect=false;
	BLE::Instance().gap().setAdvertisingInterval(advInterval); /* ?ms. 最小値 */
    BLE::Instance().gap().startAdvertising();
    minar::Scheduler::cancelCallback(blinkCallbackHandle);
}
/* コネクションがタイムアウトでなくなった時の動作 */
void callbackTimeout(const Gap::TimeoutSource_t source)
{
	connect=false;
	BLE::Instance().gap().setAdvertisingInterval(advInterval); /* ms単位. 最小値 */
    BLE::Instance().gap().startAdvertising();
}
/**
 * This callback allows the LEDService to receive updates to the ledState Characteristic.
 *
 * @param[in] params
 *     Information about the characterisitc being updated.
 */
void callbackOnDataWritten(const GattWriteCallbackParams *params) {
    // check to see what characteristic was written, by handle
    if(params->handle == writeChar.getValueHandle()) {
        if(params->len >= 1) {
            advInterval = params->data[0];
        }
        if(params->len == 2) {
            advInterval |= params->data[1]<<8;
        }
    	if (advInterval<20)advInterval=20;
    	if (advInterval>2000)advInterval=3000;
    }
}
void callback1Hour(void)
{
    {
		analogIn_enable();
		int16_t battery;
		battery=vcc.read_u16();
		batteryLevel=battery*9/256;// 0.1v単位
		//batteryLevel=vcc.read_u16()/10;// 3.6Vに対する百分率
		analogIn_disable();
    }
	if( batteryLevel<=18 ){
	    BLE &ble = BLE::Instance();
	    ble.gap().stopAdvertising();
	}
}
static void callbackBlink(void)
{
	static int counter;
	static int counterSensor;
	counter++;
	counterSensor++;
	led=1;//led off
	if(counter>1000){
		led= 0;
		if(counter>1003){
			counter=0;
		}
	}
	//!led; /* Do blinky on LED1 to indicate system aliveness. */
}
void callbackBlink3000(void)
{
	static int counter;
	static int counterSensor;
	counter++;
	counterSensor++;
	led=1;//led off
	if(counter==2){
		led= 0;
	    BLE::Instance().waitForEvent();
	}
	if(counter==3){
		led= 1;
	    BLE::Instance().waitForEvent();
	}
	if(counter>=4){
		led= 0;
		if(counter>4){
		    BLE::Instance().waitForEvent();
			counter=0;
		}
	}
	//!led; /* Do blinky on LED1 to indicate system aliveness. */

}

/**
 * This function is called when the ble initialization process has failled
 */
void onBleInitError(BLE &ble, ble_error_t error)
{
    /* Initialization error handling should go here */
    //Serial& pc = get_stdio_serial();
	//pc.printf("%02x\n",error);
	led=0;//led on
}

/**
 * Callback triggered when the ble initialization process has finished
 */
void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{
    BLE&        ble   = params->ble;
    ble_error_t error = params->error;

    if (error != BLE_ERROR_NONE) {
        /* In case of error, forward the error handling to onBleInitError */
        onBleInitError(ble, error);
        return;
    }

    /* Ensure that it is the default instance of BLE */
    if(ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {
        return;
    }
    {
        ble_gap_conn_sec_mode_t sec_mode;
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode); // no security is needed

        sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *) deviceName, strlen(deviceName));

    }

    //ble.setAddress(BLEProtocol::AddressType::PUBLIC,)

    ble.gap().onDisconnection(callbackDisconnection);
    ble.gap().onConnection(callbackConnection);
    ble.gap().onTimeout(callbackTimeout);

    batteryServicePtr = new BatteryService(ble, batteryLevel);
    txPowerServicePtr = new TxPowerService(ble, txPowerLevel);
    environmentalServicePtr =  new EnvironmentalService(ble);
    // add our custom service
    writeValue[0]=advInterval&0xff;
    writeValue[1]=advInterval>>8;
    ble.gattServer().onDataWritten(callbackOnDataWritten);
    ble.gattServer().write(writeChar.getValueAttribute().getHandle(), writeValue, sizeof(writeValue));
    ble.addService(customService);



    /* Setup advertising */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED |
    										GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS,
    		(uint8_t *) uuid16_list, sizeof(uuid16_list));
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                    (uint8_t *)uuid, sizeof(uuid));
    ble.gap().accumulateAdvertisingPayload(
    		GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *) deviceName,  sizeof(deviceName));
    ble.gap().accumulateAdvertisingPayload(
    		GapAdvertisingData::SHORTENED_LOCAL_NAME, (uint8_t *) deviceName,  sizeof(deviceName));

    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);

    /*
     * アドバタイジングのパケットを位相センサーが捕まえやすいように、
     *  nRF51822/source/noldic-sdk/nRF5xGap.cppのstartAdvertising中で
     * アドバタイジングにチャンネル37と38をを使用しない設定をしている。
     *    adv_para.channel_mask.ch_37_off=1;//
     *    adv_para.channel_mask.ch_38_off=1;//
    adv_para.channel_mask.ch_37_off=(nRF5xGap::advChannelMask&0x01)==0x1? 0 : 1 ;//1の時、アドバタイジングにチャンネル37を使用しない。
    adv_para.channel_mask.ch_38_off=(nRF5xGap::advChannelMask&0x02)==0x02? 0 : 1 ;//1の時、アドバタイジングにチャンネル38を使用しない。
    adv_para.channel_mask.ch_39_off=(nRF5xGap::advChannelMask&0x04)==0x04? 0 : 1 ;//1の時、アドバタイジングにチャンネル39を使用しない。

     BLE_API/ble/Gap.h追加
    int8_t advChannelMask;
    virtual ble_error_t setAdvChannelMask(int8_t mask){
            advChannelMask=mask&0x07;
            return BLE_ERROR_NONE;
        }

     */
	typedef union {
		uint64_t uint64;
		unsigned char uint8[6];
	} mac_t;
	mac_t mac;
	mac.uint64=0x650238240005ULL;

	BLEProtocol::Address_t address;
	address.type=BLEProtocol::AddressType::PUBLIC;
	BLE::Instance().gap().setAddress(address.type,&mac.uint8[0]);

	ble.gap().setAdvChannelMask(0x00);
    /*
     * Radio transmit power in dBm (accepted values are
     * -40, -30, -20, -16, -12, -8, -4, 0, and 4 dBm).
    */
    ble.gap().setTxPower(txPowerLevel);
    ble.gap().setAdvertisingInterval(advInterval); /* ?ms. 最小値 */
    ble.gap().startAdvertising();
}
void app_start(int, char**)
{
    NRF_POWER->RESET=1;//hw_eset有効
	//HDC1000     hdc1000(i2c);
	//LPS331AP    lps331ap(i2c);
	//lps331ap.initialize();
	//hdc1000.init();
    //Serial& pc = get_stdio_serial();
    //pc.baud(115200);    //wait(2);
    //hdc1000.setInterval(2.000);
    //uart.printf("\n\r********* Starting Main Loop *********\n\r");
    //hdc1000.setInterval(1.0);
    //minar::Scheduler::postCallback(callbackBlink).period(minar::milliseconds(1));
    //minar::Scheduler::postCallback(callbackBlink3000).period(minar::milliseconds(3000));
    minar::Scheduler::postCallback(callback1Hour).period(minar::milliseconds(3600*1000));

    BLE &ble = BLE::Instance();
    ble.init(bleInitComplete);
}
