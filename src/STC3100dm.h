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
    void  setBatteryCapacity_mAh(float batteryCapacity_mAh){_batteryCapacity_mAh = batteryCapacity_mAh;}
    /* Only do once at setup */
    uint8_t  dmBegin();

    float snapEnergyUsed1_mAhr();
    void  setEnergyMarker1();
    float getEnergyUsed1_mAhr();

    /* Fut: call periodically ~ 15minutes */
    uint8_t periodicTask();

    /* Internal */
    void  setBatteryFullyCharged(){_calculatedBatteryCapacityRemaining_mAh= _batteryCapacity_mAh;}
    float getBatteryCharge_mAh() {return _batteryCapacity_mAh;}
    //uint8_t getBatteryCharge_percent(); // target?

    private:
    #define STC3100_DM_DEFAULT_BATTERY_MAH 2000
    int16_t _batCharge1_raw=0;
    float    _batteryCapacity_mAh=STC3100_DM_DEFAULT_BATTERY_MAH;
    float    _calculatedBatteryCapacityRemaining_mAh=STC3100_DM_DEFAULT_BATTERY_MAH;
};

#endif // STC3100dm_H