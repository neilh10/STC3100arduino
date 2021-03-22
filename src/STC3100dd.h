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
#ifndef STC3100dd_H
#define STC3100dd_H
#include <Arduino.h>
#include <Wire.h>

/*Address of the STC3100 register --------------------------------------------*/
#define STC3100_REG_MODE                 0x00 /*Mode Register                 */
#define STC3100_REG_CTRL                 0x01 /*Control and Status Register   */
#define STC3100_REG_CHARGE_LOW           0x02 /*Gas Gauge Charge Data Bits 0-7*/
#define STC3100_REG_CHARGE_HIGH          0x03 /*Gas Gauge Charge Data Bits 8-15*/    
#define STC3100_REG_COUNTER_LOW          0x04 /*Number of Conversion Bits 0-7*/
#define STC3100_REG_COUNTER_HIGH         0x05 /*Number of Conversion Bits 8-15*/
#define STC3100_REG_CURRENT_LOW          0x06 /*Battery Current Value Bits 0-7*/
#define STC3100_REG_CURRENT_HIGH         0x07 /*Battery Current Value Bits 8-15*/
#define STC3100_REG_VOLTAGE_LOW          0x08 /*Battery Voltage Value Bits 0-7*/
#define STC3100_REG_VOLTAGE_HIGH         0x09 /*Battery Voltage Value Bits 8-15*/
#define STC3100_REG_TEMPERATURE_LOW      0x0A /*Temperature Values Bits 0-7) */
#define STC3100_REG_TEMPERATURE_HIGH     0x0B /*Temperature Values Bits 8-15)*/

/* Device ID registers Address 24 to 31 --------------------------------------*/
#define STC3100_REG_ID0                  0x18 /*Part Type ID 10h  */
#define STC3100_REG_ID1                  0x19 /*Unique Part ID Bits 0-7  */
#define STC3100_REG_ID2                  0x1A /*Unique Part ID Bits 8-15  */
#define STC3100_REG_ID3                  0x1B /*Unique Part ID Bits 16-23  */
#define STC3100_REG_ID4                  0x1C /*Unique Part ID Bits 24-31  */
#define STC3100_REG_ID5                  0x1D /*Unique Part ID Bits 32-39  */
#define STC3100_REG_ID6                  0x1E /*Unique Part ID Bits 40-47  */
#define STC3100_REG_ID7                  0x1F /*Device ID CRC  */

// RAM registers go up to 31 address 63 for 32 ram bytes
#define STC3100_REG_RAM0  0x20  

#define STC3100_BUS_ADDRESS 0x70

#define STC3100_ID_LEN 8
#define STC3100_REG_LEN 10

//With 30milliOhms
#define STC3100_R_SERIES_mOhms 30
#define STC3100_mAHr_FACTOR (6.7/_current_resistor_milliohms)
#define STC3100_CURRENT_FACTOR (11.77 / _current_resistor_milliohms)
#define STC3100_VFACTOR 0.00244
#define STC3100_TEMPERATURE_FACTOR 0.125

#define STC3100_REG_MODE_ADC_RES_OFFSET  0x01 /* Mode GG_RES (ADC) bit start   */

#define STC3100_REG_MODE_ADCRES_POS   0x01
#define STC3100_REG_MODE_ADCRES_MASK  0x03 /* ADC bit mask */
#define STC3100_REG_MODE_ADCRES_14BITS  00
#define STC3100_REG_MODE_ADCRES_13BITS  01
#define STC3100_REG_MODE_ADCRES_12BITS  10

#define STC3100_REG_MODE_RUN_MASK     0x10

#define STC3100_REG_CTRL_IO0DATA_MASK  0x01
#define STC3100_REG_CTRL_RST_MASK      0x02

static const uint8_t CRC_LOOKUP[256] = {
  0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
  0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
  0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
  0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
  0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
  0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
  0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
  0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
  0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
  0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
  0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
  0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
  0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
  0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
  0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
  0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
  0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
  0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
  0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
  0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
  0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
  0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
  0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
  0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
  0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
  0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
  0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
  0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
  0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
  0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
  0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
  0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

class STC3100dd
{
  public: 

  typedef struct{
        bool valid;
        int16_t charge_raw;
        float charge_mAhr;
        uint16_t counter;
        float current_mA;
        float voltage_V;
        float temperature_C;
    } fgValues_t;
    
    /* Fastest interface is to read from 'v' values */

    fgValues_t v;


  uint8_t serial_number[STC3100_ID_LEN];

/**
 * @brief Construct a new STC3100::STC3100 object to reference IC
 * 
 * @param resistor_value The value of the current sensing resistor in milliOhms
 * @param sample_rate The value of the sampling rate
 */
  STC3100dd( uint8_t sample_rate=STC3100_REG_MODE_ADCRES_14BITS, uint16_t resistor_batValuesalue=STC3100_R_SERIES_mOhms);
  STC3100dd(TwoWire* theI2C, uint8_t sample_rate=STC3100_REG_MODE_ADCRES_14BITS, uint16_t resistor_batValuesalue=STC3100_R_SERIES_mOhms);
  /**
 * @brief  setI2cAddress- call at beginning if need to change 
 * There only appears to be one IC that is commercially available,
 * but a number of IC addresses are defined in the manual
 * 
 */
  #if defined STC3100_USE_ADDR
  void setI2cAddress(int8_t i2cAddressHex=STC3100_BUS_ADDRESS) {_i2cAddressHex=i2cAddressHex;}
  #endif // STC3100_USE_ADDR
 /**
 * @brief begin - call at beginning, before start()
 * 
 */
  void begin();
  /**
 * @brief start() will check if it can read and confirm the serial number. 
 * If the serial number can be read and confirmed, this function will set the
 * chip to start taking readings.
 * 
 * @return true When serial is read and confirmed else false
 */
  bool start();

  void    setAdcResolution(uint8_t adc_resolution=STC3100_REG_MODE_ADCRES_14BITS);
  uint8_t getAdcResolution() {return _adc_resolution;}

  void    setCurrentResistor(uint8_t current_resistor_milliohms = STC3100_R_SERIES_mOhms) {_current_resistor_milliohms = current_resistor_milliohms;}
  uint8_t getCurrentResistor() {return _current_resistor_milliohms;}

  void    setModeOperateRun() {_operate=true;}
  void    setModeOperateStandby() {_operate=false;}
  uint8_t getModeOperate() {return _operate;}

  uint8_t updateModeReg();
  uint8_t resetChargeAcc(); //Reset the charge acc

  String getSn(void);
  String getType() {return  String(serial_number[0],HEX);}

  uint8_t readValues();

/**
 * @brief Gets the stored charge.
 * 
 * @return float last read Charge in mAh
 */
  float getCharge_mAhr(bool pollDevice=false);

/**
 * @brief Gets the current battery voltage - better to read from v.
 * 
 * @return float Battery voltage in Volts
 */
  float getVoltage_V(bool pollDevice=false);

/**
 * @brief Gets the current reading  better to read from v.
 * 
 * @return float Battery Current in milliAmps
 */
  float getCurrent_mA(bool pollDevice=false);

/**
 * @brief Gets the current temperature as a float in celsius
 * 
 * @return float Temperature in celsius
 */
  float getTemperature_C(bool pollDevice=false);

/**
 * @brief Reads all the data registers
 * 
 * @param dataReg Pointer to array to store readings
 */
  void getAllReg(uint8_t *dataReg);  

  /**
 * @brief Reads serial number from battery IC
 * 
 * @param serialNum Pointer to 8 byte array to store serial number in
 * @return true Serial number is valid else failed CRC
 */
  bool readSerialNumber(uint8_t *serial);  

  protected:

  /**
 * @brief Processes the charge reading from the raw register
 
When using an external 30 milliOhms sense resistor, the 28-bit accumulator results in a capacity of 
approximately +/- 7300 mA.h. 
From reset, with fully charged battery this looks like FFFF=14,636.15mAh, and discharge (-ve mA)counts down.
The voltage drop across the external sense resistor is integrated during a conversion period 
and input to a 12- to 14-bit AD converter. 
The output conversion is accumulated into a 28-bit accumulator. 
The upper 16 bits of the accumulator are read.
The AD converter output is in two’s complement format. When a conversion cycle is
completed, the result is added to the charge accumulator and the number of conversions is
incremented in a 16-bit counter. 
The LSB value is set by the internal gain and internal reference and is 11.77 uV at maximum resolutions. 

The upper 16 bits of the accumulator can be read  giving a resolution of 0.2 mAhr

  * @return float Battery Charge in milliAmpsHrs
  */
  float rawToCharge_mAhr(int16_t rawReg); 

/**
  * @brief Processes the current reading from the raw register
  * 
  The battery current is coded in 2’s complement format, 
  The LSB value is 11.77 uV,irrespective of the current conversion resolution.
  It is a 14-bit binary number using the two’s complement binary format. 
  Bit 13 is the sign bit and bits 0 to 12 are the data bits 
  In 13-bit resolution mode, the 0 bit is always set to zero. 
  In 12-bit resolution, bits 0 and 1 are always set to zero

  Two's complement is common way of represents signed integers.
  In this scheme, if the binary number 010 encodes the signed integer 2,
  then its two's complement, 110, encodes the inverse: −2.
    For three-bit signed integers
    Decimal
    value	Two's-complement representation
        0	    000
        1	    001
        2	    010
        3	    011
        −4	    100
        −3	    101
        −2	    110
        −1	    111  
  Two's complement has no representation for negative zero    
  For two's complement the fundamental arithmetic operations of addition, subtraction, and multiplication 
  are identical to those for unsigned binary numbers 
  (as long as the inputs are represented in the same number of bits as the output, 
  and any overflow beyond those bits is discarded from the result).    
  * 
  * @return float Battery Current in milliAmps
  */
  float rawToCurrent_mA(uint16_t rawReg); 

/**
 * @brief Processes the temperature reading from the raw register
 * 
The temperature value is coded in 2’s complement format.
The LSB value is 0.125° C.
The temperature of 0° C corresponds to code 0.
 * 
 *  @return float Temperature in degrees C
 * */
  float rawToTemperature_C(uint16_t rawReg); 


/**
 * @brief Writes byte to STC3100 IC register
 * 
 * @param reg Location in i2c IC to write value
 * @param value Value to write to IC
 */
  void writeByteWire(uint8_t reg, uint8_t value);


/**
 * @brief crc8calc preforms a cyclica reduncy calculation radix 8 on data
 * 
 * @param data Pointer to data array we want to checksum
 * @param size Size of data we want to checksum
 * @return uint8_t CRC8 value generated from data
 */
  uint8_t  crc8calc(const void * data, size_t size);

/**
 * @brief Get value from the STC3100 chip by reading the 
 *low byte and high byte and returning a single 16 bit value
 * 
 * @param reg Register address to read two bytes from
 * @return uint16_t Value read
 */
  uint16_t getReadingWire(uint8_t reg);

/**
 * @brief Reads two bytes from Wire buffer
 * 
 *  Wire transaction must have been completed.
 * 
 * @return 16bits integer
 */
  uint16_t get2BytesBuf();

  uint16_t _current_resistor_milliohms = STC3100_R_SERIES_mOhms;
  uint8_t _adc_resolution = 0;
  bool _operate = true;
  bool _detectedPresent = false;

  /**
   * @brief An internal reference to the hardware Wire instance.
   */
  TwoWire* _i2c;  // Hardware Wire

  /**
   * @brief The I2C address of the STC3100.
   * 
   * There only appears to be one IC that is commercially available,
   * but a number of IC addresses are defined in the manual
   */
  #if !defined STC3100_USE_ADDR
    const 
  #endif //STC3100_USE_ADDR
   int8_t _i2cAddressHex=STC3100_BUS_ADDRESS;
  
};

#endif //STC3100dd_H
