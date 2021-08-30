/* STC3100dm.h
 * This is a battery management STC3100 device manager  
 * It manages the STC3100  charge readings for them to be meaningful.  
 * Copyright 2020  Neil Hancock
 * 
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE READ THE "MIT LICENSE" AGREEMENT FILE LOCATED
 * IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE. LICENSE.md
 * 
 */

#ifndef STC3100dm_H
#define STC3100dm_H

#include "STC3100dd.h"

/* From AN3064 3.2 Periodic GasGauge task   ~ tbd functional calls
At first call, use the battery voltage to estimate the battery’s state-of-charge and save it
in a RAM location of the STC3100 as a reference value.

● During subsequent calls, use the saved reference value and the present charge
register content to calculate the present battery capacity:
battery capacity = reference value + charge register content.

● Trigger a "low battery" alarm if the battery’s voltage is low (typically less than 3.1 V).

● Calculate the remaining operating time from the remaining battery capacity and current
consumption when the battery is not charging (negative current)
*/

class STC3100dm : public STC3100dd 
{
    public: 
    /* Only do once at setup, before dmBegin */
    void setBatteryCapacity_mAh(float batteryCapacity_mAh);
    /* Only do once at setup */
    uint8_t  dmBegin();

    void  snapEnergyMarker1();
    void  setHandshake1(){_handshake1=true;}
    float getEnergyUsed1_mAhr();

    float getEnergyAvlbl_mAhr();
    /* Call periodically 1 ~ 15minutes */
    uint8_t periodicTask();

    /* Internal */
    void  setBatteryFullyCharged();
    float getBatteryCharge_mAh() {return _batteryCapacity_mAh;}
    //uint8_t getBatteryCharge_percent(); // target?

    //public: allows some debugging, ideally would be private:
    #define STC3100_DM_DEFAULT_BATTERY_MAH 4400
    //To be realistic, the system always needs a perceived value
    #define STC3100_DM_BATTERY_MIN_MAH 10
    #define STC3100_DM_CHG_ACCUM_RESET 0xffff

    int16_t _batCharge1_raw=0;
    float   _energyUsed_mAhr=0;
    float   _batteryCapacity_mAh=STC3100_DM_DEFAULT_BATTERY_MAH;
    float   _calculatedBatteryCapacityRemaining_mAh=STC3100_DM_DEFAULT_BATTERY_MAH;
    int8_t  _chargeDirCounter; //Counts up for +positive charging
    bool    _handshake1=false;
};

#endif // STC3100dm_H
