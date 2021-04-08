/* STC3100dd.h
 * This is a battery management STC3100 device driver  
 * decodes voltage, current and charge 
 * Copyright 2020  Neil Hancock
 * 
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE READ THE "MIT LICENSE" AGREEMENT FILE LOCATED
 * IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE. LICENSE.md
 * 
 */

#include "STC3100dm.h"

uint8_t  STC3100dm::dmBegin() 
{
    /* Do Battery state initialization, 
    possibly read last state from Stc3100ram
    */
   uint8_t status = readValues(); //Need 1st time through
   snapEnergyMarker1();
    #if defined STC3100DM_DEBUG
    Serial.print(F("STC3100dm dmBegin("));
    Serial.print(status);
    Serial.print((":"));
    Serial.print(v.counter);
    Serial.print(F(") charge_raw="));
    Serial.println((uint16_t)_batCharge1_raw,HEX );
    #endif //STC3100DM_DEBUG
    return status;
}

uint8_t STC3100dm::periodicTask() 
{
    /*  Battery scharge analysis,
        Check if battery fully charged - high voltage and low charge current 
        possibly store state to Stc3100ram
    */
    #if defined STC3100DM_DEBUG
    Serial.print(F("STC3100dm periodic Cnt="));
    Serial.print(chargeDirCounter);
    Serial.print(F(" mA="));
    Serial.println(v.current_mA  );    
    #endif //STC3100DM_DEBUG
 
    #define STC3100_DM_CHARGE_THRESHOLD_UP_COUNT 3
    #define STC3100_DM_CHARGE_THRESHOLD_FULL_V 4.10
    #define STC3100_DM_CURRENT_THRESHOLD_mA 20.0
    if ((v.current_mA < -0.1) || (v.current_mA > STC3100_DM_CURRENT_THRESHOLD_mA )) {
       //Discharging ~ or heavilt charging.
       _chargeDirCounter = 0;

    } else {

        if (++_chargeDirCounter>STC3100_DM_CHARGE_THRESHOLD_UP_COUNT) {
            _chargeDirCounter=STC3100_DM_CHARGE_THRESHOLD_UP_COUNT;
            #if defined STC3100DM_DEBUG
            Serial.print(F("STC3100dm charged? V="));
            Serial.print(v.voltage_V );
            Serial.print(F(" mAhr="));
            Serial.print(_calculatedBatteryCapacityRemaining_mAh);
            Serial.print(F(" charge_raw="));
            Serial.println((uint16_t)_batCharge1_raw,HEX );
            #endif //STC3100DM_DEBUG

            if (v.voltage_V >  STC3100_DM_CHARGE_THRESHOLD_FULL_V)
            {
                setBatteryFullyCharged();
                _chargeDirCounter = 0;
            }
       }
    }
    return 0; //No event
}
 
float STC3100dm::getEnergyUsed1_mAhr() 
{
    if (_handshake1) {
        //FUDGE: only updated _energyUsed_mAhr once on each update
        _handshake1 = false;
        snapEnergyMarker1();
    }
    return _energyUsed_mAhr;
}
 
void STC3100dm::snapEnergyMarker1() 
{
    int16_t energyUsed_raw = v.charge_raw - _batCharge1_raw;
    _batCharge1_raw = v.charge_raw;

    _energyUsed_mAhr = rawToCharge_mAhr(energyUsed_raw);

    _calculatedBatteryCapacityRemaining_mAh += _energyUsed_mAhr;
    #if defined STC3100DM_DEBUG
    Serial.print(F("STC3100dm snapEnerygMarker1 used(mAh)"));
    Serial.print(_energyUsed_mAhr);
    Serial.print(F(" Cap(mAh)"));
    Serial.println(_calculatedBatteryCapacityRemaining_mAh);
    #endif //STC3100DM_DEBUG
}

void  STC3100dm::setBatteryCapacity_mAh(float batteryCapacity_mAh)
{
    //Only done at init, 
    _calculatedBatteryCapacityRemaining_mAh= _batteryCapacity_mAh = batteryCapacity_mAh;
    // if resetCharAcc()/setBatteryFullyCharged() seems to halt the IC
    #if defined STC3100DM_DEBUG
    Serial.print(F("STC3100dm setBatteryCapacity "));
    Serial.println(_batteryCapacity_mAh);
    #endif //STC3100DM_DEBUG
    }

void  STC3100dm::setBatteryFullyCharged()
{
    _calculatedBatteryCapacityRemaining_mAh= _batteryCapacity_mAh;
    resetChargeAcc();
    #if defined STC3100DM_DEBUG
    Serial.print(F("STC3100dm setBatteryFullyCharged "));
    Serial.println(_batteryCapacity_mAh);
    #endif //STC3100DM_DEBUG
}

float STC3100dm::getEnergyAvlbl_mAhr() {
    return _calculatedBatteryCapacityRemaining_mAh;
}
