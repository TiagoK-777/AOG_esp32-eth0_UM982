/**************************************************************************/
/*!
  Adapted from adafruit ADS1015/ADS1115 library

    v1.0 - First release
*/
/**************************************************************************/

#include "Arduino.h"
#include "hardware/ads1115.h"

/**************************************************************************/
/*!
    @brief  Instantiates a new ADS1115 class w/appropriate properties
*/
/**************************************************************************/
ADS1115_lite::ADS1115_lite(uint8_t i2cAddress) {
	//Wire.end();
	//Wire.begin();
	_i2cAddress = i2cAddress;
	_gain = ADS1115_REG_CONFIG_PGA_2_048V; /* +/- 6.144V range (limited to VDD +0.3V max!) */
	_mux = ADS1115_REG_CONFIG_MUX_DIFF_0_1; /* to default */
	_rate = ADS1115_REG_CONFIG_DR_128SPS; /* to default */
}

/**************************************************************************/
/*!
    @brief  Sets up the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
bool ADS1115_lite::testConnection() {
  Wire.beginTransmission(_i2cAddress);
  Wire.write(ADS1115_REG_POINTER_CONVERT);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(_i2cAddress, (uint8_t)2) == 2) {
    return 1;
  }
  return 0;
}

/**************************************************************************/
/*!
    @brief  Sets the gain and input voltage range.  Valid numbers:
	 ADS1115_REG_CONFIG_PGA_6_144V   (0x0000)  // +/-6.144V range = Gain 2/3
	 ADS1115_REG_CONFIG_PGA_4_096V   (0x0200)  // +/-4.096V range = Gain 1
	 ADS1115_REG_CONFIG_PGA_2_048V   (0x0400)  // +/-2.048V range = Gain 2 (default)
	 ADS1115_REG_CONFIG_PGA_1_024V   (0x0600)  // +/-1.024V range = Gain 4
	 ADS1115_REG_CONFIG_PGA_0_512V   (0x0800)  // +/-0.512V range = Gain 8
	 ADS1115_REG_CONFIG_PGA_0_256V   (0x0A00)  // +/-0.256V range = Gain 16
*/
/**************************************************************************/
void ADS1115_lite::setGain(uint16_t gain) {
  _gain = gain;
}

/**************************************************************************/
/*!
    @brief  Sets the mux.  Valid numbers:
	ADS1115_REG_CONFIG_MUX_DIFF_0_1 (0x0000)  // Differential P = AIN0, N = AIN1 (default)
	ADS1115_REG_CONFIG_MUX_DIFF_0_3 (0x1000)  // Differential P = AIN0, N = AIN3
	ADS1115_REG_CONFIG_MUX_DIFF_1_3 (0x2000)  // Differential P = AIN1, N = AIN3
	ADS1115_REG_CONFIG_MUX_DIFF_2_3 (0x3000)  // Differential P = AIN2, N = AIN3
	ADS1115_REG_CONFIG_MUX_SINGLE_0 (0x4000)  // Single-ended AIN0
    ADS1115_REG_CONFIG_MUX_SINGLE_1 (0x5000)  // Single-ended AIN1
    ADS1115_REG_CONFIG_MUX_SINGLE_2 (0x6000)  // Single-ended AIN2
    ADS1115_REG_CONFIG_MUX_SINGLE_3 (0x7000)  // Single-ended AIN3
*/
/**************************************************************************/
void ADS1115_lite::setMux(uint16_t mux) {
  _mux = mux;
}
/**************************************************************************/
/*!
    @brief  Sets the mux.  Valid numbers:
	ADS1115_REG_CONFIG_MUX_DIFF_0_1 (0x0000)  // Differential P = AIN0, N = AIN1 (default)
	ADS1115_REG_CONFIG_MUX_DIFF_0_3 (0x1000)  // Differential P = AIN0, N = AIN3
	ADS1115_REG_CONFIG_MUX_DIFF_1_3 (0x2000)  // Differential P = AIN1, N = AIN3
	ADS1115_REG_CONFIG_MUX_DIFF_2_3 (0x3000)  // Differential P = AIN2, N = AIN3
	ADS1115_REG_CONFIG_MUX_SINGLE_0 (0x4000)  // Single-ended AIN0
    ADS1115_REG_CONFIG_MUX_SINGLE_1 (0x5000)  // Single-ended AIN1
    ADS1115_REG_CONFIG_MUX_SINGLE_2 (0x6000)  // Single-ended AIN2
    ADS1115_REG_CONFIG_MUX_SINGLE_3 (0x7000)  // Single-ended AIN3
*/
/**************************************************************************/
void ADS1115_lite::setSampleRate(uint8_t rate) {
  _rate = rate;
}
/**************************************************************************/
/*!
		Trigger Conversion to Read later
*/
/**************************************************************************/
bool ADS1115_lite::triggerConversion() {

  //Bits 0 through 4 deal with the comparator function.  The combined default value for these bits is 0x0003
  uint16_t config = 0x0003; // Disable the comparator (default val)

  // OR in the Data rate or Sample Per Seconds bits 5 through 7
  config |= _rate;

  // OR in the mode bit 8
  config |= ADS1115_REG_CONFIG_MODE_SINGLE; // Single-shot mode (default)

  // OR in the PGA/voltage range bits 9 through 11
  config |= _gain;

  // OR in the mux channel, bits 12 through 14
  config |= _mux;

  // OR in the start conversion bit bit 15
  config |= ADS1115_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  Wire.beginTransmission(_i2cAddress);
  Wire.write(ADS1115_REG_POINTER_CONFIG);
  Wire.write((uint8_t)(config >> 8));
  Wire.write((uint8_t)(config & 0xFF));
  return Wire.endTransmission() == 0;
}

/**************************************************************************/
/*!
    @brief  Waits and returns the  last conversion results
*/
/**************************************************************************/
bool ADS1115_lite::getConversion(int16_t& result) {
  Wire.beginTransmission(_i2cAddress);
  Wire.write(ADS1115_REG_POINTER_CONVERT);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(_i2cAddress, static_cast<uint8_t>(2)) != 2) return false;
  uint8_t high = Wire.read();
  uint8_t low = Wire.read();
  result = static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
  return true;
}
/**************************************************************************/
/*!
    @brief  returns true if the conversion is done
*/
/**************************************************************************/
bool ADS1115_lite::isConversionDone() {

  Wire.beginTransmission(_i2cAddress); //Sets the Address of the ADS1115.
  Wire.write(ADS1115_REG_POINTER_CONFIG); //queue the data to be sent, in this case modify the pointer register so that the following RequestFrom reads the config register
  Wire.endTransmission(); //Set the stop bit

  Wire.requestFrom(_i2cAddress, (uint8_t)2); //Request 2 byte config register
  return ((Wire.read() << 8) | Wire.read()) >> 15 ; //Read 2 bytes.  Return the most signifagant bit
}
