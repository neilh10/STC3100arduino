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

// #define USE_RTCLIB Sodaq_DS3231
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

#define STC3100_RESISTOR    ((uint16_t) 10)  // 10 mOhms series

STC3100dd battMon(STC3100_REG_MODE_ADCRES_14BITS, STC3100_RESISTOR); 
uint32_t counter = 0;

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

// #define USE_POWER
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
    while (!SERIAL.available());

    SERIAL.println("STC3100 Raw Data");
    Wire.setSDA(PB11);
    Wire.setSCL(PB10);
    battMon.begin();  //Wire.begin();
    bool fStatus = battMon.start();
    if (fStatus) {
        SERIAL.print("STC3100 S/N: ");
        String sn(battMon.getSn());
        SERIAL.print(sn);
        SERIAL.print(" Type: 0x");
        SERIAL.println(battMon.getType());

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

void measureBattery() {

    #if defined USE_RTCLIB 
    DateTime time_dt =getNowSecs2kTz();//2000
    String dateTimeStr;
    time_dt.addToString(dateTimeStr);
    SERIAL.print(dateTimeStr);
    SERIAL.print(", ");   
    #endif //USE_RTCLIB 
    if (battMon.readValues()) {
        //0 is pass
        SERIAL.println(" Failed to get new reading "); 
        return;     
    }
    char log_buf[128] = {};
    snprintf(log_buf, sizeof(log_buf), "#%d, cnt %d, %.3fV, %+.1fmA, %.1fC, %.2fmAh, chrg raw %d",
            counter,
            battMon.v.counter,
            battMon.v.voltage_V,
            battMon.v.current_mA,
            battMon.v.temperature_C,
            battMon.v.charge_mAhr,
            battMon.v.charge_raw);
    SERIAL.println(log_buf);
}

#define MEASUREMENT_INTERVAL_MS  1000
void loop(void) {
    static uint32_t last_meas;
    uint32_t now = millis();
    if (now - last_meas > MEASUREMENT_INTERVAL_MS) {
        last_meas = now;
        counter++;
#if defined USE_POWER
        if (counter % 4 == 0) {
            SERIAL.println("Power ON");
            PowerOn();
        } else if (counter % 4 == 2) {
            SERIAL.println("Power OFF");
            PowerOff();
        }
#endif //USE_POWER
        measureBattery();
    }
}
