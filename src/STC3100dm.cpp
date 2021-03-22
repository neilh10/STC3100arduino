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

uint8_t  STC3100dm::dmBegin() {
    /* Do Battery state initialization, 
    possibly read last state from Stc3100ram
    */
   uint8_t status = readValues(); //Need 1st time through
   setEnergyMarker1();
    #if defined STC3100DM_DEBUG
    Serial.print(F("STC3100dm dmBegin("));
    Serial.print(status);
    Serial.print((":"));
    Serial.print(v.counter);
    Serial.print(F(") charge_raw="));
    Serial.println(_batCharge1_raw,HEX );
    #endif //STC3100DM_DEBUG
    return status;
}

uint8_t STC3100dm::periodicTask() {
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
   if ((v.current_mA < -0.1) || (v.current_mA > 20) ) {
       //Discharging ~ or heavilt charging.
       chargeDirCounter = 0;

   }else {
       #define STC3100_DM_CHARGE_THRESHOLD_UP_COUNT 2
            #define STC3100_DM_CHARGE_THRESHOLD_FULL_V 4.2
            #define STC3100_DM_CURRENT_THRESHOLD_mA 5.0
       if (++chargeDirCounter>STC3100_DM_CHARGE_THRESHOLD_UP_COUNT) {
            chargeDirCounter=STC3100_DM_CHARGE_THRESHOLD_UP_COUNT;
            #if defined STC3100DM_DEBUG
            Serial.print(F("STC3100dm charged? V="));
            Serial.print(v.voltage_V );
            Serial.print(F(" mAhr="));
            Serial.print(_calculatedBatteryCapacityRemaining_mAh);
            Serial.print(F(" charge_raw="));
            Serial.println(_batCharge1_raw,HEX );
            #endif //STC3100DM_DEBUG

            if ( (v.voltage_V >  STC3100_DM_CHARGE_THRESHOLD_FULL_V)
            && (v.current_mA < STC3100_DM_CURRENT_THRESHOLD_mA)) {
                setBatteryFullyCharged();
            }
       }
   }
   return 0; //No event
}
 
void STC3100dm::setEnergyMarker1() {
    _batCharge1_raw = v.charge_raw;
}

float STC3100dm::getEnergyUsed1_mAhr() {
    int16_t  energyUsed;
    /* The charge code is a 16-bit binary number using the two’s complement binary format. 
     * Bit 15 is the sign bit and the LSB value is 6.70 µV.h. 
     * To convert the charge data into mA.h, the external resistance Rsense must be taken into account:
     *    charge data (mA.h) = 6.70 * charge_code / Rsense (mΩ).
     *  
     * When the result of an accumulation exceeds the limits of a signed binary number 
     * (below 0x8000 or above 0x7FFF), the carry bit is lost, no overflow is generated 
     * and the accumulator continues to accumulate 
     * See  rawToCharge_mAhr(uint16_t rawReg) for conversion and description of two's complement
     * */
    energyUsed = v.charge_raw - _batCharge1_raw;

    /*Normally would be 
      0xFFFF-0xFFF0 = 0x000F or +15
      0x000F-0x0000 = 0x000F
      0x0000-0xFFFF =   0001 or +1
      0x000F-0xFFF0 =   001F or +31
      
      0xFFFF-0x0001 = 0xFFFE  of -2   
      0xFFFE-0xFFFF = 0xFFFF  or  -1
      0xFFF0-0xFFFF = 0xFFF1  or -15
      0x0000-0x000F = 0xFFF1  or -15
      0x0000-0x0001 = 0xFFFF  or  -1

      Charge         0.03ohms            0.1ohms
      0x8000  32,768    7,318,186mAhr   2,195,456mAhr

      Check for wrap or overflow - below 0x8000 or above 0x7FFF
      though is unlikely in real system.


      */
    #define STC3100_CHARGE_NAX 0x7FFF
    #define STC3100_CHARGE_MIN 0x8000
    #if defined STC3100DM_DEBUG
    Serial.print(F("STC3100dm energyUsed ["));
    Serial.print(v.counter);
    Serial.print(F("] "));
    Serial.print(energyUsed,HEX);
    Serial.print(F("="));
    Serial.print(v.charge_raw,HEX);
    Serial.print(F(" - "));
    Serial.println(_batCharge1_raw,HEX );
    #endif //STC3100DM_DEBUG
    //if (energyUsed > )

    return rawToCharge_mAhr(energyUsed);
}

float STC3100dm::snapEnergyUsed1_mAhr() {
    float energyUsed_mAhr = getEnergyUsed1_mAhr();
    setEnergyMarker1();
    _calculatedBatteryCapacityRemaining_mAh += energyUsed_mAhr;
    return energyUsed_mAhr;
}

void  STC3100dm::setBatteryCapacity_mAh(float batteryCapacity_mAh){
    _batteryCapacity_mAh = batteryCapacity_mAh;
    setBatteryFullyCharged();
    }
void  STC3100dm::setBatteryFullyCharged(){
    _calculatedBatteryCapacityRemaining_mAh= _batteryCapacity_mAh;
    resetChargeAcc();
    }
float STC3100dm::getEnergyAvlbl_mAhr() {
    //float energyUsed_mAhr = getEnergyUsed1_mAhr();
    //setEnergyMarker1();
    return _calculatedBatteryCapacityRemaining_mAh;
}