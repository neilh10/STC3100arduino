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
#if defined(ARDUINO_ARCH_AVR)
    _i2c->setWireTimeout(STC310_SETWIRETIMEOUT_MS ,true); //enable recovery from lockup
#endif
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
    uint16_t reg = getReadingWire(STC3100_REG_MODE);
    uint8_t lo = reg & 0xFF;
    uint8_t hi = (reg >> 8) & 0xFF;
    Serial.print("STC3100dd STC3100_REG_MODE 0x");
    Serial.println(lo, HEX);
    Serial.print("STC3100dd STC3100_REG_CTRL 0x");
    Serial.println(hi, HEX);
    #endif // STC3100DD_DEBUG
    
    //Reset Charge Acc to 0
    resetChargeAcc();

    updateModeReg();

    #if defined STC3100DD_DEBUG
    reg = getReadingWire(STC3100_REG_MODE);
    lo = reg & 0xFF;
    hi = (reg >> 8) & 0xFF;
    Serial.println("STC3100dd After initCTL");    
    Serial.print("STC3100dd STC3100_REG_MODE 0x");
    Serial.println(lo, HEX);
    Serial.print("STC3100dd STC3100_REG_CTRL 0x");
    Serial.println(hi, HEX);
    #endif // STC3100DD_DEBUG

    return true;
}

String STC3100dd::getSn(void)   {
    String sn;
    if (!_detectedPresent) {
        sn += String(F("None"));
    } else {
        sn.reserve(2*STC3100_ID_LEN+2); //Number of Ascii characcters for 8*bytes
        #if !defined(ARDUINO_ARCH_STM32)
        for (int snlp=1;snlp<(STC3100_ID_LEN-1);snlp++) {
                //1st byte is ID (STC3100=0x10) last byte is CRC 
                uint8_t temp_num=serial_number[snlp];
                sn +=String((temp_num>>4) & 0x0f,HEX);
                sn +=String((temp_num & 0x0f),HEX);
        }
        #else
        for (int snlp = 1; snlp < STC3100_ID_LEN - 1; snlp++) {
            if (serial_number[snlp] < 0x10) {
                sn += '0';
            }
            sn += String(serial_number[snlp], HEX);
        }
        #endif // ! ARDUINO_ARCH_STM32
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

uint8_t STC3100dd::resetChargeAcc() {
    // CG_RST=1 ~ ensure charge is cleared.
    uint8_t regWr  =  STC3100_REG_CTRL_RST_MASK|STC3100_REG_CTRL_IO0DATA_MASK;
    writeByteWire(STC3100_REG_CTRL, regWr); 
    return regWr;
}

uint8_t STC3100dd::readValuesIc(){
    uint8_t status;

    v.valid = false;
    _i2c->beginTransmission(_i2cAddressHex);
    _i2c->write(STC3100_REG_CHARGE_LOW);
    status = _i2c->endTransmission();
    if (0 == status) {
        uint8_t numRead =_i2c->requestFrom(_i2cAddressHex, STC3100_REG_LEN );
#define TWOWIRE_ERR_TIMEOUT 0x41
#if defined(ARDUINO_ARCH_AVR)
        if (_i2c->getWireTimeoutFlag()) {
            return TWOWIRE_ERR_TIMEOUT;
        }
#elif defined(ARDUINO_ARCH_STM32)
        I2C_HandleTypeDef * hi2c = _i2c->getHandle();
        if (hi2c && HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_NONE) {
            return TWOWIRE_ERR_TIMEOUT;
        }
#endif
        if (STC3100_REG_LEN != numRead) {
            #if defined STC3100DD_DEBUG
            Serial.print(F("STC3100dd readValues unexpected "));
            Serial.print(F("numRead"));
            #endif //STC3100DD_DEBUG
        }
        v.charge_raw = (int16_t) get2BytesBuf();
        v.charge_mAhr = rawToCharge_mAhr(v.charge_raw);
        v.counter = get2BytesBuf();
        v.current_mA = rawToCurrent_mA(get2BytesBuf());
        v.voltage_V = get2BytesBuf() * STC3100_VFACTOR;
        v.temperature_C = rawToTemperature_C(get2BytesBuf());
        v.valid = true;
    } else {
        #if defined STC3100DD_DEBUG
        Serial.print(F("STC3100dd readValues not read. Err"));
        Serial.print(status);
        #endif // STC3100DD_DEBUG
    }
    uint8_t writeError = _i2c->getWriteError();
    if (0!=writeError ) {
            #if defined STC3100DD_DEBUG
            Serial.print(F("STC3100dd readValues getWriteError "));
            Serial.print(F("numRead"));
            #endif //STC3100DD_DEBUG
   }
    return status;
}
uint8_t STC3100dd::readValues(){
    uint8_t status;
    #define TWOWIRE_RETRY_CNT 5
    uint8_t repeatCntr=TWOWIRE_RETRY_CNT;
    do {
        status =  readValuesIc();
#if defined(ARDUINO_ARCH_AVR)
        if (!_i2c->getWireTimeoutFlag() && (0 == status)) {
            break; // out of whileComplete
        }
        _i2c->clearWireTimeoutFlag();
  #elif defined(ARDUINO_ARCH_STM32)
        I2C_HandleTypeDef * hi2c = _i2c->getHandle();
        if (hi2c && HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_NONE && (0 == status)) {
            break;
        }
#endif      
        if (--repeatCntr) {
            #if defined STC3100DD_DEBUG
            Serial.print(F("STC3100dd serious TimeoutsError, tried and failed "));
            Serial.print(TWOWIRE_RETRY_CNT);
            #endif // STC3100DD_DEBUG
        }

    } while (repeatCntr);
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
    #if defined STC3100DD_DEBUG
    if (0xffff == value) {
        Serial.println(F("STC3100dd get2BytesBuf FFFF"));
    }
    #endif //STC3100DD_DEBUG

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
