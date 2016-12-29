#include <avr/io.h>
#include <avr/interrupt.h>
#include "Arduino.h"
#include "I2CDevice.h"

I2CDevice::I2CDevice() {}

void I2CDevice::setBitrate(uint8_t bitrate) {
	/* Sets the bitrate */
	TWBR = bitrate;
}

void I2CDevice::beginTransmission(uint8_t address, uint8_t isWrite) {
	uint8_t twst;
	/*
	 * Send START condition
	 */
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	/*
	 * Wait until transmission completed
	 */
	while (!(TWCR & (1<<TWINT)));
	/*
	 * Check value of TWI Status Register. Mask prescaler bits.
	 */
	twst = TWSR & 0xF8;
	if ((twst != TWI_START) && (twst != TWI_REP_START)) return;
	/*
	 * Send device address
	 */
	TWDR = (address<<1) + isWrite;
	TWCR = (1<<TWINT)|(1<<TWEN);
	/*
	 * Wait until transmission completed and ACK/NACK has been received
	 */
	while (!(TWCR & (1<<TWINT)));
	/*
	 * Check value of TWI Status Register. Mask prescaler bits.
	 */
	twst = TWSR & 0xF8;
	if ((twst != TWI_MTX_ADR_ACK) && (twst != TWI_MRX_ADR_ACK)) return;
	return;
}

void I2CDevice::endTransmission(){
	/*
	 * Send stop condition
	 */
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	/*
	 * Wait until stop condition is executed and bus released
	 */
	while (TWCR & (1<<TWINT));
}

uint8_t I2CDevice::write(uint8_t byte) {
	uint8_t twst;
	/*
	** Send data to the previously addressed device
	*/
	TWDR = byte;
	TWCR = (1<<TWINT)|(1<<TWEN);
	/*
	** Wait until transmission completed
	*/
	while (!(TWCR & (1<<TWINT)));
	/*
	** Check value of TWI Status Register. Mask prescaler bits
	*/
	twst = TWSR & 0xF8;
	if (twst != TWI_MTX_DATA_ACK) return FALSE;
	return TRUE;
}

uint8_t I2CDevice::readNextByte() {
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while (!(TWCR & (1<<TWINT)));   
	return TWDR;
}

uint8_t I2CDevice::readLastByte() {
	TWCR = (1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	return TWDR;
}

uint8_t I2CDevice::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
	int i = 0;
	I2CDevice::beginTransmission(devAddr, I2C_WRITE); // set device address and write mode
	I2CDevice::write(regAddr);
	for(; i < length; i++)
		if(!I2CDevice::write(data[i])) return FALSE;
	I2CDevice::endTransmission();
	return TRUE;
}

uint8_t I2CDevice::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data) {
	return I2CDevice::readBytes(devAddr, regAddr, 1, data);
}

uint8_t I2CDevice::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
	int i = 0;
	I2CDevice::beginTransmission(devAddr, I2C_WRITE);
	if(!I2CDevice::write(regAddr)) return FALSE;
	I2CDevice::beginTransmission(devAddr, I2C_READ);
	for(;i<length-1;i++){
	  data[i] = I2CDevice::readNextByte();
	}
	data[i] = I2CDevice::readLastByte();
	I2CDevice::endTransmission();
	return TRUE;
}

uint8_t I2CDevice::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
	uint8_t b;
    I2CDevice::readByte(devAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return I2CDevice::writeBytes(devAddr, regAddr, 1, &b);
}
