/* STC3100dd.cpp
 * This is a battery management STC3100 device driver  
 * decodes voltage, current and charge 
 * Copyright 2020  Neil Hancock
 * 
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE READ THE "MIT LICENSE" AGREEMENT FILE LOCATED
 * IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE. LICENSE.md
 * 
 */

#include "STC3100dd.h"



STC3100dd::STC3100dd(uint8_t adc_resolution, uint16_t resistor_value){
    _i2c           = &Wire;
    _current_resistor_milliohms = resistor_value;
    _adc_resolution = (adc_resolution & STC3100_REG_MODE_ADCRES_MASK);
}

STC3100dd::STC3100dd(TwoWire* theI2C, uint8_t adc_resolution, uint16_t resistor_value){
    _i2c           = theI2C;
    _current_resistor_milliohms = resistor_value;
    _adc_resolution = (adc_resolution & STC3100_REG_MODE_ADCRES_MASK);
}

void STC3100dd::begin(){
    _i2c->begin();
}

bool STC3100dd::start(){
    if(!readSerialNumber(serial_number)){
        #if defined STC3100DD_DEBUG
        Serial.print("STC3100dd device not detected");
        #endif // STC3100DD_DEBUG
        return false;
    }
    _detectedPresent = true;
    #if defined STC3100DD_DEBUG
    Serial.print("STC3100dd STC3100_REG_CTRL 0x");
    Serial.println(getReadingWire(STC3100_REG_MODE), HEX);
    #endif // STC3100DD_DEBUG
    
    // CG_RST=1 ~ ensure charge is cleared.
    writeByteWire(STC3100_REG_CTRL, STC3100_REG_CTRL_RST_MASK|STC3100_REG_CTRL_IO0DATA_MASK);

    updateModeReg();

    #if defined STC3100DD_DEBUG
    Serial.print("STC3100dd After initCTL 0x");
    Serial.println(getReadingWire(STC3100_REG_MODE), HEX);
    #endif // STC3100DD_DEBUG

    return true;
}

String STC3100dd::getSn(void)   {
    String sn;
    if (!_detectedPresent) {
        sn += String(F("None"));
    } else {
        sn.reserve(STC3100_ID_LEN+1);
        for (int snlp=1;snlp<(STC3100_ID_LEN-2);snlp++) {
                sn +=String(serial_number[snlp],HEX);
        }
    }
    return sn;
}

void STC3100dd::setAdcResolution(uint8_t adc_resolution){
    _adc_resolution = (adc_resolution & STC3100_REG_MODE_ADCRES_MASK);
}

uint8_t STC3100dd::updateModeReg() {
    // Start gauge -Clock AutoDetect, ADC, CG_RUN
    uint8_t regWr = (_operate) ? STC3100_REG_MODE_RUN_MASK  : 0;
    regWr |=  (_adc_resolution<<STC3100_REG_MODE_ADCRES_POS);
    writeByteWire(STC3100_REG_MODE, regWr); 
    return regWr;
}

uint8_t STC3100dd::readValues(){
    uint8_t status;

    _i2c->beginTransmission(_i2cAddressHex);
    _i2c->write(STC3100_REG_CHARGE_LOW);
    status = _i2c->endTransmission();
    if (0 == status) {
        _i2c->requestFrom(_i2cAddressHex, STC3100_REG_LEN );
        
        v.valid = true;
        v.charge_raw = get2BytesBuf();
        v.charge_mAhr = (float) rawToCharge_mAhr(v.charge_raw);
        v.counter = get2BytesBuf();
        v.current_mA = rawToCurrent_mA(get2BytesBuf());
        v.voltage_V = get2BytesBuf() * STC3100_VFACTOR;
        v.temperature_C = rawToTemperature_C(get2BytesBuf());
    } else {
        #if defined STC3100DD_DEBUG
        Serial.print("STC3100dd readValues not read");
        #endif // STC3100DD_DEBUG
    }
    return status;
}

float STC3100dd::rawToCharge_mAhr(int16_t rawReg) {
    return ((float) rawReg) *STC3100_mAHr_FACTOR;
} 

float STC3100dd::rawToCurrent_mA(uint16_t rawReg) {
    signed current = rawReg & 0x3fff; // mask unused bits
    if (current>=0x2000) current -= 0x4000; // convert to signed value

    return ((float)current * STC3100_CURRENT_FACTOR); // result in mA
} 

float STC3100dd::getCharge_mAhr(bool pollDevice){
    if (pollDevice) readValues();
    return v.charge_mAhr;
}

float STC3100dd::getCurrent_mA(bool pollDevice){
    if (pollDevice) readValues();
    return v.current_mA;
}

float STC3100dd::getVoltage_V(bool pollDevice){
    if (pollDevice) readValues();
    return v.voltage_V;
}

float STC3100dd::rawToTemperature_C(uint16_t rawReg){ 
    signed temperature = rawReg & 0xfff; // mask unused bits
    if (temperature>=0x0800) temperature -= 0x1000;
    return (temperature  *STC3100_TEMPERATURE_FACTOR);
}

float STC3100dd::getTemperature_C(bool pollDevice){
    if (pollDevice) readValues();
    return v.temperature_C;
}

bool STC3100dd::readSerialNumber(uint8_t *serialNum){
    _i2c->beginTransmission(_i2cAddressHex);
    _i2c->write(STC3100_REG_ID0);
    _i2c->endTransmission();
    _i2c->requestFrom(_i2cAddressHex, 8);
    for(uint16_t i =0; i< STC3100_ID_LEN; i++){
        serialNum[i] = _i2c->read();
    }
    return crc8calc(serialNum, 7)==serialNum[7];
}

void STC3100dd::getAllReg(uint8_t *dataReg){
    _i2c->beginTransmission(_i2cAddressHex);
    _i2c->write(STC3100_REG_CHARGE_LOW);
    _i2c->endTransmission();
    _i2c->requestFrom(_i2cAddressHex, STC3100_REG_LEN);
    for(uint16_t i =0; i< STC3100_REG_LEN; i++){
        dataReg[i] = _i2c->read();
    }
    return ;
} 

uint16_t STC3100dd::getReadingWire(uint8_t reg){
    _i2c->beginTransmission(_i2cAddressHex);
    _i2c->write(reg);
    _i2c->endTransmission();
    _i2c->requestFrom(_i2cAddressHex, 2);
    uint8_t low =  _i2c->read();         // print the character
    uint8_t high = _i2c->read();
    uint16_t value = low;
    value |= high<<8;
    return value;
}

uint16_t STC3100dd::get2BytesBuf() {
    uint8_t low =  _i2c->read();         // print the character
    uint8_t high = _i2c->read();
    uint16_t value = low;
    value |= high<<8;
    return value;
}

void STC3100dd::writeByteWire(uint8_t reg, uint8_t value){
    _i2c->beginTransmission(_i2cAddressHex);
    _i2c->write(reg);
    _i2c->write(value);
    _i2c->endTransmission();
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
