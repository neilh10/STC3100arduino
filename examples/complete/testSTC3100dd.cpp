/* STC3100lib - example 
Output D:\usr\a\PlatformIO\Projects\STC3100lib

Each Pass delay(2000);
Time  ,  Pass, ChgCounter,  Vbat, Current,  Tempeature, Charge_mAh, charge_raw
4,12, 4.1187V, -114.17,mA, 19.12,C, 14636.15,mAh,
5,16, 4.1187V, -114.56,mA, 19.12,C, 14635.93,mAh,
2021-02-28 15:23:56, 8, 146, 4.1846,V, -10.59,mA, 18.62,C, 14636.15,mAh, FFFF
2021-02-28 15:24:07, 9, 167, 4.1846,V, -9.81,mA, 18.62,C, 14635.93,mAh, FFFE

*/

#include <Arduino.h>
#include "STC3100dd.h"

#define USE_RTCLIB Sodaq_DS3231
#if defined USE_RTCLIB 
#include <Sodaq_DS3231.h>
USE_RTCLIB  rtcExtPhy;
#endif 

#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
    #define SERIAL SerialUSB
#else
    #define SERIAL Serial
    #define SERIAL_PORT_USBVIRTUAL Serial
#endif

STC3100dd  battMon(STC3100_REG_MODE_ADCRES_14BITS,STC3100_R_SERIES_mOhms); 
uint32_t  counter =0;

#if defined USE_RTCLIB 
//This assumes an RTC in UTC, and need to adapt to local time
int8_t _loggerTimeZone = -8;
DateTime StartUp_dt;
#define HOURS_TO_SECS 3600
#define EPOCH_TIME_OFF 946684800 

uint32_t getNowSecs2kTz(void) {
    int64_t currentEpochTime = (int64_t)((uint64_t)rtc.now().getEpoch());
    currentEpochTime += (_loggerTimeZone * HOURS_TO_SECS);
    return (uint32_t)currentEpochTime-EPOCH_TIME_OFF;
}
#endif //defined USE_RTCLIB 

#define USE_POWER
#if defined USE_POWER
const int8_t powerPin = 22;
#define PowerOn()  digitalWrite(powerPin, HIGH)
#define PowerOff()  digitalWrite(powerPin, LOW)
#endif //USE_POWER

#if !defined SERIAL_BAUD
#define SERIAL_BAUD 115200
#endif //SERIAL_BAUD
void setup(void) {


    SERIAL.begin(SERIAL_BAUD);

    // Enter <CR> to start
    //while (!SERIAL.available());

    SERIAL.println("STC3100 Raw Data");

    battMon.init();  //Wire.begin();
    bool fStatus = battMon.start();
    if (fStatus) {
        SERIAL.print("STC3100 sn ");
        for (int snlp=1;snlp<(STC3100_ID_LEN-1);snlp++) {
            SERIAL.print(battMon.serial_number[snlp],HEX);
        }
        SERIAL.print(" Type ");
        SERIAL.println(battMon.serial_number[0],HEX);

    } else {SERIAL.println("STC3100 start failed");}

    #if defined USE_RTCLIB 
    if (rtcExtPhy.begin()) {
        StartUp_dt= getNowSecs2kTz();//2000
        String dateTimeStr;
        StartUp_dt.addToString(dateTimeStr);
        SERIAL.print(F("Current RTC time is: "));
        SERIAL.println(dateTimeStr);
    }
    #endif 

    #if defined USE_POWER
    if (powerPin >= 0) {
        PowerOff();
        pinMode(powerPin, OUTPUT);
        SERIAL.print(F("Using power pin"));
        SERIAL.println(powerPin);
    }
    #endif //USE_POWER
}

//STC3100::batteryReadings_t batValues;
void loop(void) {

    battMon.readValues();
    counter++;

    #if defined USE_RTCLIB 
    DateTime time_dt =getNowSecs2kTz();//2000
    String dateTimeStr;
    time_dt.addToString(dateTimeStr);
    SERIAL.print(dateTimeStr);
    SERIAL.print(", ");   
    #endif //USE_RTCLIB 

    SERIAL.print( counter);
    SERIAL.print(", ");   
    SERIAL.print( battMon.v.counter);
    SERIAL.print(", ");    
    SERIAL.print( battMon.v.voltage_V,4); 
    SERIAL.print(",V, ");

    SERIAL.print(battMon.v.current_mA);
    SERIAL.print(",mA, ");
    SERIAL.print(battMon.v.temperature_C);
    SERIAL.print(",C, ");
    SERIAL.print(battMon.v.charge_mAhr);
    SERIAL.print(",mAh, 0x");
    SERIAL.print(battMon.v.charge_raw,HEX);
    SERIAL.println();

    #if defined USE_POWER
    if (counter &0x01) {
        PowerOn();
    }else {
        PowerOff();
    }
    #endif //USE_POWER

    delay(9940);  //10sec 9940
    //with 10,000 Every 20passes the above takes 1second longer - try 9,950
    // with 9,950 EVery 300=458-150 only takes 9Secs 300*10sec=3,000secs - try 9,940
}
