#include "i2c.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Arduino.h"

I2CDevice::I2CDevice(uint8_t address, uint32_t bitrate) {
	this -> address = address;
	/* Sets the bitrate */
	TWBR = ((F_CPU/bitrate)-16)/2;
	if(TWBR < 11) status = FALSE;
	else status = TRUE;
}

I2CDevice::~I2CDevice() {
	stop();
}

uint8_t I2CDevice::start(uint8_t type){
	if(!status) return FALSE;
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
	if ((twst != TWI_START) && (twst != TWI_REP_START)) return FALSE;
	/*
	 * Send device address
	 */
	TWDR = (address<<1) + type;
	TWCR = (1<<TWINT)|(1<<TWEN);
	/*
	 * Wait until transmission completed and ACK/NACK has been received
	 */
	while (!(TWCR & (1<<TWINT)));
	/*
	 * Check value of TWI Status Register. Mask prescaler bits.
	 */
	twst = TWSR & 0xF8;
	if ((twst != TWI_MTX_ADR_ACK) && (twst != TWI_MRX_ADR_ACK)) return FALSE;

	return TRUE;
}

void I2CDevice::stop(){
	if(!status) return;
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
	if(!status) return FALSE;
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
	if(!status) return FALSE;
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while (!(TWCR & (1<<TWINT)));   

	return TWDR;
}

uint8_t I2CDevice::readLastByte() {
	if(!status) return FALSE;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	return TWDR;
}

void I2CDevice::writeRegister(char reg, char value) {
	start(I2C_WRITE); // set device address and write mode
	write(reg);
	write(value);
	stop();
}

char I2CDevice::readRegister(char reg){
	start(I2C_WRITE);
	write(reg);

	start(I2C_READ); // set device address and read mode
	char ret = readLastByte();
	stop();
	return ret;
} 

char* I2CDevice::readNRegisters(char regStart, int n) {
	char *data = new char[n];
	for(int i = 0; i < n; i++)
		data[i] = readRegister(regStart + i);
	return data;
}