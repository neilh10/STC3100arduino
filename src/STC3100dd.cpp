/* STC3100dd.h
 * This is a battery management STC3100 device driver  
 * decodes voltage, current and charge 
 * Copyright 2020  Neil Hancock
 * 
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE READ THE LICENSE AGREEMENT FILE LOCATED
 * IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
 * 
 */

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

STC3100dd::STC3100dd(uint8_t sample_rate, uint16_t resistor_value){
    _current_resistor_milliohms = resistor_value;
}

void STC3100dd::init(){
    Wire.begin();
}

bool STC3100dd::start(){
    if(!readSerialNumber(serial_number)){
        return false;
    }
    Serial.print("STC3100dd STC3100_REG_CTRL 0x");
    Serial.println(getReadingWire(STC3100_REG_MODE), HEX);
    // CG_RST=1 ~ ensure charge is cleared.
    writeByteWire(STC3100_REG_CTRL, 0x02);
    // Start gauge -Clock AutoDetect, ADC=14Bits, CG_RUN=1
    writeByteWire(STC3100_REG_MODE, 0x10); //Set to operate

    Serial.print("STC3100dd After initCTL 0x");
    Serial.println(getReadingWire(STC3100_REG_MODE), HEX);

    return true;
}

STC3100dd::fgValues_t STC3100dd::readValues(){

    Wire.beginTransmission(BUS_ADDRESS);
    Wire.write(STC3100_REG_CHARGE_LOW);
    Wire.endTransmission();
    Wire.requestFrom(BUS_ADDRESS, STC3100_REG_LEN );
    
    v.valid = true;
    v.charge_raw = get2BytesBuf();
    v.charge_mAhr = (float) rawToCharge_mAhr(v.charge_raw);
    v.counter = get2BytesBuf();
    v.current_mA = rawToCurrent_mA(get2BytesBuf());
    v.voltage_V = get2BytesBuf() * STC3100_VFACTOR;
    v.temperature_C = rawToTemperature_C(get2BytesBuf());
    return v;
}

float STC3100dd::rawToCharge_mAhr(uint16_t rawReg) {
    return ((float) rawReg) *STC3100_mAHr_FACTOR;
} 

float STC3100dd::rawToCurrent_mA(uint16_t rawReg) {
    signed current = rawReg & 0x3fff; // mask unused bits
    if (current>=0x2000) current -= 0x4000; // convert to signed value
    return ((float)current * STC3100_CURRENT_FACTOR); // result in mA
} 

float STC3100dd::readCurrent_mA(){
    uint16_t current_raw =  getReadingWire(STC3100_REG_CURRENT_LOW);
    return rawToCurrent_mA(current_raw);
}

float STC3100dd::readVoltage_V(){
    float v = ((uint16_t) getReadingWire(STC3100_REG_VOLTAGE_LOW) * STC3100_VFACTOR);
    return v;
}

float STC3100dd::rawToTemperature_C(uint16_t rawReg){ 
    signed temperature = rawReg & 0xfff; // mask unused bits
    if (temperature>=0x0800) temperature -= 0x1000;
    return (temperature  *STC3100_TEMPERATURE_FACTOR);
}

float STC3100dd::readTemperature_C(){
    float t = ((uint16_t)getReadingWire(STC3100_REG_TEMPERATURE_LOW));
    return t;
}

bool STC3100dd::readSerialNumber(uint8_t *serialNum){
    Wire.beginTransmission(BUS_ADDRESS);
    Wire.write(STC3100_REG_ID0);
    Wire.endTransmission();
    Wire.requestFrom(BUS_ADDRESS, 8);
    for(uint16_t i =0; i< STC3100_ID_LEN; i++){
        serialNum[i] = Wire.read();
    }
    return crc8calc(serialNum, 7)==serialNum[7];
}

void STC3100dd::getAllReg(uint8_t *dataReg){
    Wire.beginTransmission(BUS_ADDRESS);
    Wire.write(STC3100_REG_CHARGE_LOW);
    Wire.endTransmission();
    Wire.requestFrom(BUS_ADDRESS, STC3100_REG_LEN);
    for(uint16_t i =0; i< STC3100_REG_LEN; i++){
        dataReg[i] = Wire.read();
    }
    return ;
} 

uint16_t STC3100dd::getReadingWire(uint8_t reg){
    Wire.beginTransmission(BUS_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(BUS_ADDRESS, 2);
    uint8_t low =  Wire.read();         // print the character
    uint8_t high = Wire.read();
    uint16_t value = low;
    value |= high<<8;
    return value;
}

uint16_t STC3100dd::get2BytesBuf() {
    uint8_t low =  Wire.read();         // print the character
    uint8_t high = Wire.read();
    //uint16_t high_byte  = high;
    uint16_t value = low;
    value |= high<<8;
    return value;
}

/* FUT not tested */
void STC3100dd::writeByteWire(uint8_t reg, uint8_t value){
    Wire.beginTransmission(BUS_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t STC3100dd::crc8calc(const void * data, size_t size){
    uint8_t val = 0;
    uint8_t * pos = (uint8_t *) data;
    uint8_t * end = pos + size;
    while (pos < end) {
        val = CRC_LOOKUP[val ^ *pos];
        pos++;
    }
    return val;
}
