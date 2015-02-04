/*************************************************** 
  This is a library for the MPR121 I2C 12-chan Capacitive Sensor

  Designed specifically to work with the MPR121 sensor from Adafruit
  ----> https://www.adafruit.com/products/1982

  These sensors use I2C to communicate, 2+ pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "MPR121.h"
#include <stdio.h>

MPR121::MPR121() {

}

boolean MPR121::begin(uint8_t i2caddr, boolean debug) {
    Wire.begin();

     _i2caddr = i2caddr;

    // soft reset
    writeRegister(MPR121_SOFTRESET, 0x63);
    delay(1);
    if( debug ) {
        char buffer[3];
        Serial.print("\t\t");
        for( uint8_t i = 0; i < 16; i++ ) {
            sprintf(buffer,"%02X",i);
            Serial.print(buffer);
            Serial.print("\t");
        }
        for( uint8_t i = 0; i < 8; i++ ) {
            sprintf(buffer,"%02X",i * 16);
            Serial.print(buffer);
            Serial.print("\t\t")
            for( uint16_t j = 0; j < 16; j++ ) {
                uint8_t value = readRegister8(j + 16 * i);
                sprintf(buffer,"%02X",value);
                Serial.print("\t");
                delay(5);
            }
        }
        Serial.println("");
    }
    
    writeRegister(MPR121_ECR, 0x0);

    uint8_t c = readRegister8(MPR121_CONFIG2);

    if( c != 0x24 ) return false;


    setThreshholds(12, 6);
    writeRegister(MPR121_MHDR, 0x01);   //  1 is the maximum value passing through baseline filter (rising)
    writeRegister(MPR121_NHDR, 0x01);   //  noise half delta (rising) = 1
    writeRegister(MPR121_NCLR, 0x0E);   //  # samples greater than MHD to distinguish noise = 0x0E
    writeRegister(MPR121_FDLR, 0x00);   //  filter delay count limit

    writeRegister(MPR121_MHDF, 0x01);   //  maximum half delta (falling)
    writeRegister(MPR121_NHDF, 0x05);   //  noise half delta (falling)
    writeRegister(MPR121_NCLF, 0x01);   //  noise count limit (falling)
    writeRegister(MPR121_FDLF, 0x00);   //  filter delay count limit (falling)

    writeRegister(MPR121_NHDT, 0x00);   //  noise half delta touched
    writeRegister(MPR121_NCLT, 0x00);   //  noise count limit (touched)
    writeRegister(MPR121_FDLT, 0x00);   //  filter delay count limit

    writeRegister(MPR121_DEBOUNCE, 0);  //  debounce for touch/release = 0
    writeRegister(MPR121_CONFIG1, 0x10); // default, 16uA charge current
    writeRegister(MPR121_CONFIG2, 0x20); // 0.5uS encoding, 1ms period

    //  writeRegister(MPR121_AUTOCONFIG0, 0x8F);

    //  writeRegister(MPR121_UPLIMIT, 150);
    //  writeRegister(MPR121_TARGETLIMIT, 100); // should be ~400 (100 shifted)
    //  writeRegister(MPR121_LOWLIMIT, 50);
    // enable all electrodes
    writeRegister(MPR121_ECR, 0x8F);  // start with first 5 bits of baseline tracking

    return true;
}

void MPR121::setThreshholds(uint8_t touch, uint8_t release) {
    for (uint8_t i=0; i<12; i++) {
        writeRegister(MPR121_TOUCHTH_0 + 2*i, touch);
        writeRegister(MPR121_RELEASETH_0 + 2*i, release);
    }
}

uint16_t  MPR121::filteredData(uint8_t t) {
    if (t > 12) return 0;
    return readRegister16(MPR121_FILTDATA_0L + t*2);
}

uint16_t  MPR121::baselineData(uint8_t t) {
    if (t > 12) return 0;
    uint16_t bl = readRegister8(MPR121_BASELINE_0 + t);
    return (bl << 2);
}

uint16_t  MPR121::touched(void) {
    uint16_t t = readRegister16(MPR121_TOUCHSTATUS_L);
    return t & 0x0FFF;
}

/*********************************************************************/


uint8_t MPR121::readRegister8(uint8_t reg) {
    Wire.beginTransmission(_i2caddr);
    Wire.write(reg);
    Wire.endTransmission(false);
    while (Wire.requestFrom(_i2caddr, 1) != 1);
    return ( Wire.read());
}

uint16_t MPR121::readRegister16(uint8_t reg) {
    Wire.beginTransmission(_i2caddr);
    Wire.write(reg);
    Wire.endTransmission(false);
    while (Wire.requestFrom(_i2caddr, 2) != 2);
    uint16_t v = Wire.read();
    v |=  ((uint16_t) Wire.read()) << 8;
    return v;
}

/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
void MPR121::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)(value));
    Wire.endTransmission();
}
