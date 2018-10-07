//==============================================================================
//    E - R A D I O N I C A . C O M,  H.Kolomana 6/A, Djakovo 31400, Croatia
//==============================================================================
// Project   :  SHT21 Arduino Library (V1.0)
// File      :  SHT21.h
// Author    :  e-radionica.com 2017
// Licence   :  Open-source ! 
//==============================================================================
//==============================================================================
// Use with any SHT21 breakout. Check ours: 
// https://e-radionica.com/en/sht21-humidity-and-temperature-sensor.html
// If any questions, 
// just contact techsupport@e-radionica.com
//==============================================================================

#ifndef SHT21_H
#define SHT21_H

#include "driver/i2c.h"



void sht21_init(void);
//==============================================================================
float sht21_getHumidity(void);
//==============================================================================
// calls humidity measurement with hold master mode

//==============================================================================
float sht21_getTemperature(void);
//==============================================================================
// calls temperature measurement with hold master mode

//==============================================================================
void sht21_reset();
//==============================================================================
// performs a soft reset, delays 15ms

//==============================================================================
uint8_t sht21_getSerialNumber(uint8_t return_sn);
//==============================================================================
// returns electronical identification code depending of selected memory
// location

#endif