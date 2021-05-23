//==============================================================================
//    E - R A D I O N I C A . C O M,  H.Kolomana 6/A, Djakovo 31400, Croatia
//==============================================================================
// Project   :  SHT21 Arduino Library (V1.0)
// File      :  SHT21.cpp
// Author    :  e-radionica.com 2017
// Licence   :  Open-source ! 
//==============================================================================
//==============================================================================
// Use with any SHT21 breakout. Check ours: 
// https://e-radionica.com/en/sht21-humidity-and-temperature-sensor.html
// If any questions, 
// just contact techsupport@e-radionica.com
//==============================================================================

#include <sht21.h>
#include "driver/i2c.h"
#include "esp_log.h"

//---------- Defines -----------------------------------------------------------
#define I2C_ADD 0x40	// I2C device address
#define TAG "SHT21"

const uint16_t POLYNOMIAL = 0x131;  // P(x)=x^8+x^5+x^4+1 = 100110001

//==============================================================================
#define TRIGGER_T_MEASUREMENT_HM 0XE3   // command trig. temp meas. hold master
#define TRIGGER_RH_MEASUREMENT_HM 0XE5  // command trig. hum. meas. hold master
#define TRIGGER_T_MEASUREMENT_NHM 0XF3  // command trig. temp meas. no hold master
#define TRIGGER_RH_MEASUREMENT_NHM 0XF5 // command trig. hum. meas. no hold master
#define USER_REGISTER_W 0XE6		    // command writing user register
#define USER_REGISTER_R 0XE7            // command reading user register
#define SOFT_RESET 0XFE                 // command soft reset
//==============================================================================
// HOLD MASTER - SCL line is blocked (controlled by sensor) during measurement
// NO HOLD MASTER - allows other I2C communication tasks while sensor performing
// measurements.

//==============================================================================
uint16_t sht21_readSensor_hm(uint8_t command);
//==============================================================================
// reads SHT21 with hold master operation mode
// input:	temp/hum command
// return:	temp/hum raw data (16bit scaled)


//==============================================================================
float sht21_CalcRH(uint16_t rh);
//==============================================================================
// calculates the relative humidity
// input:  rh:	 humidity raw value (16bit scaled)
// return:		 relative humidity [%RH] (float)

//==============================================================================
float sht21_CalcT(uint16_t t);
//==============================================================================
// calculates the temperature
// input:  t: 	temperature raw value (16bit scaled)
// return:		relative temperature [°C] (float)

//==============================================================================
uint8_t sht21_CRC_Checksum(uint8_t data[], uint8_t no_of_bytes, uint8_t checksum);
//==============================================================================
// CRC-8 checksum for error detection
// input:  data[]       checksum is built based on this data
//         no_of_bytes  checksum is built for n bytes of data
//         checksum     expected checksum
// return:              1 			   = checksum does not match
//                      0              = checksum matches



//==============================================================================
// PUBLIC
//==============================================================================

void sht21_init(void){
	ESP_LOGI(TAG, "Initializing I2C for SHT21");
	i2c_config_t sht21_conf;
    sht21_conf.mode = I2C_MODE_MASTER;
    sht21_conf.sda_io_num = GPIO_NUM_33;
    sht21_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    sht21_conf.scl_io_num = GPIO_NUM_32;
    sht21_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    sht21_conf.master.clk_speed = 100000;
    //ESP_ERROR_CHECK
    (i2c_param_config(I2C_NUM_1, &sht21_conf));
    ESP_LOGD(TAG, "SHT21 i2c_driver_install %d", I2C_NUM_1);
    //ESP_ERROR_CHECK
    (i2c_driver_install(I2C_NUM_1, sht21_conf.mode, 0, 0, 0));
}

float sht21_getHumidity(void) {
	uint16_t result; 	// return variable
	result = sht21_readSensor_hm(TRIGGER_RH_MEASUREMENT_HM);
	return sht21_CalcRH(result);
}

float sht21_getTemperature(void) {
	uint16_t result; 	// return variable
	result = sht21_readSensor_hm(TRIGGER_T_MEASUREMENT_HM);
	return sht21_CalcT(result);
}

void sht21_reset() {
	/*Wire.beginTransmission(I2C_ADD);
	Wire.write(SOFT_RESET);
	Wire.endTransmission();*/

	vTaskDelay(15);	// wait for SHT to reset
}

uint8_t sht21_getSerialNumber(uint8_t return_sn) {

	uint8_t serialNumber[8];

	uint8_t value = 0;
	i2c_cmd_handle_t cmd;
	esp_err_t ret;
	cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);								// Send i2c start on bus
	i2c_master_write_byte(cmd, (I2C_ADD<<1) | I2C_MASTER_WRITE, 0);
	i2c_master_write_byte(cmd, 0xFA, 0);
	i2c_master_write_byte(cmd, 0x0F, 0);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (I2C_ADD<<1) | I2C_MASTER_READ, 0);

	i2c_master_read_byte(cmd, &value, 1); // read SNB_3
	serialNumber[5] = value;
	i2c_master_read_byte(cmd, &value, 1); // CRC SNB_3 not used
	i2c_master_read_byte(cmd, &value, 1); // read SNB_2
	serialNumber[4] = value;
	i2c_master_read_byte(cmd, &value, 1); // CRC SNB_2 not used
	i2c_master_read_byte(cmd, &value, 1); // read SNB_1
	serialNumber[3] = value;
	i2c_master_read_byte(cmd, &value, 1); // CRC SNB_1 not used
	i2c_master_read_byte(cmd, &value, 1); // read SNB_0
	serialNumber[2] = value;
	i2c_master_read_byte(cmd, &value, 1); // CRC SNB_0 not used

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (I2C_ADD<<1) | I2C_MASTER_WRITE, 0);
	i2c_master_write_byte(cmd, 0xFC, 0);
	i2c_master_write_byte(cmd, 0xC9, 0);

	i2c_master_start(cmd);
	i2c_master_read_byte(cmd, &value, 1); /// read SNC_1
	serialNumber[1] = value;
	i2c_master_read_byte(cmd, &value, 1); // read SNC_0
	serialNumber[0] = value;
	i2c_master_read_byte(cmd, &value, 1); // CRC SNC_1/SNC_0 not used
	i2c_master_read_byte(cmd, &value, 1); // read SNA_1	
	serialNumber[7] = value;
	i2c_master_read_byte(cmd, &value, 1); // read SNA_0
	serialNumber[6] = value;
	i2c_master_read_byte(cmd, &value, 1); // CRC SNA_1/SNA_0 not used

	// read memory location 1
	/*Wire.beginTransmission(I2C_ADD);
	Wire.write(0xFA);
	Wire.write(0x0F);
	Wire.endTransmission();

	Wire.requestFrom(I2C_ADD,8);
	while(Wire.available() < 8) {}

	serialNumber[5] = Wire.read();	// read SNB_3
	Wire.read();					// CRC SNB_3 not used
	serialNumber[4] = Wire.read();  // read SNB_2
	Wire.read();					// CRC SNB_2 not used
	serialNumber[3] = Wire.read();	// read SNB_1
	Wire.read();					// CRC SNB_1 not used
	serialNumber[2] = Wire.read();	// read SNB_0
	Wire.read();					// CRC SNB_0 not used

	// read memory location 2
	Wire.beginTransmission(I2C_ADD);
	Wire.write(0xFC);
	Wire.write(0xC9);
	Wire.endTransmission();

	Wire.requestFrom(I2C_ADD,6);
	while(Wire.available() < 6) {}

	serialNumber[1] = Wire.read();	// read SNC_1
	serialNumber[0] = Wire.read();  // read SNC_0
	Wire.read();					// CRC SNC_1/SNC_0 not used
	serialNumber[7] = Wire.read();	// read SNA_1			
	serialNumber[6] = Wire.read();	// read SNA_0
	Wire.read();					// CRC SNA_1/SNA_0 not used
*/
	return serialNumber[return_sn];
}

//==============================================================================
// PRIVATE
//==============================================================================

uint16_t sht21_readSensor_hm(uint8_t command) {
	uint8_t checksum = 0;
	uint8_t data[2];
	uint16_t result;

	i2c_cmd_handle_t cmd;
	esp_err_t ret;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);								// Send i2c start on bus
	i2c_master_write_byte(cmd, (I2C_ADD<<1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, command, 1);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	vTaskDelay(100 / portTICK_PERIOD_MS);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (I2C_ADD<<1) | /*I2C_MASTER_READ*/1, 1);
	i2c_master_read_byte(cmd, data+0, 0); // data MSB
	i2c_master_read_byte(cmd, data+1, 0); // data LSB
	i2c_master_read_byte(cmd, &checksum, 0); // CRC
	i2c_master_stop(cmd);
  	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  	i2c_cmd_link_delete(cmd);

	printf("Received: %d %d, %d", data[0], data[1], checksum);

	result = (data[0] << 8);
	result += data[1];

	if(sht21_CRC_Checksum (data,2,checksum)) {
		sht21_reset();
		return 1;
	}
	
	else return result;
}

float sht21_CalcRH(uint16_t rh) {

	rh &= ~0x0003;	// clean last two bits

  	return (-6.0 + 125.0/65536.0 * (float)rh); // return relative humidity
}

float sht21_CalcT(uint16_t t) {

	t &= ~0x0003;	// clean last two bits
 	
	return (-46.85 + 175.72/65536.0 * (float)t);
}

uint8_t sht21_CRC_Checksum(uint8_t data[], uint8_t no_of_bytes, uint8_t checksum) {
	uint8_t crc = 0;	
  	uint8_t byteCtr;

 	 //calculates 8-Bit checksum with given polynomial
  	for (byteCtr = 0; byteCtr < no_of_bytes; ++byteCtr)
 	 { crc ^= (data[byteCtr]);
 	   for (uint8_t bit = 8; bit > 0; --bit)
 	   { if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
 	     else crc = (crc << 1);
 	   }
 	 }
 	 if (crc != checksum) return 1;
 	 else return 0;
}


