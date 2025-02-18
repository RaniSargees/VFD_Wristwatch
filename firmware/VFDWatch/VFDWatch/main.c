/*
 * main.c
 *
 * VFD Wristwatch Firmware (INCOMPLETE)
 *
 * Created: 3/6/2021 12:07:43 PM
 *  Author: Rani Sargees
 */ 

#include <xc.h>
#include <util/delay_basic.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>


//pin definitions
#define SDA  PORTB0
#define SCL  PORTB2
#define CLK  PORTB1
#define BTNA PORTB3
#define BTNB PORTB4 

//macros for i2c
#define DELAY_TIME 2
#define SDA_LOW  (DDRB |= (1<<SDA))
#define SDA_TRI  (DDRB &=~(1<<SDA))
#define SDA_READ (PINB &  (1<<SDA))
#define SCL_LOW  (DDRB |= (1<<SCL))
#define SCL_TRI  (DDRB &=~(1<<SCL))
#define SCL_READ (PINB &  (1<<SCL))
#define i2c_delay() _delay_loop_1(DELAY_TIME)

//devices on the I2C bus
#define ADDR_IOEXPANDER 0x20
#define REG_IOEX_PIN 0x02
#define REG_IOEX_DIR 0x06
#define ADDR_RTC 0x6F

//timing params
#define MULTIPLEX_DELAY 2048

//minimal i2c bus master implementation
void i2c_start() { //4 delays
	SDA_TRI;
	i2c_delay();
	SCL_TRI;
	i2c_delay();
	SDA_LOW;
	i2c_delay();
	SCL_LOW;
	i2c_delay();
}

void i2c_stop() {
	SCL_LOW;
	i2c_delay();
	SDA_LOW;
	i2c_delay();
	SCL_TRI;
	i2c_delay();
	SDA_TRI;
	i2c_delay();
}

uint8_t i2c_tx(uint8_t data) { //17 delays
	for (uint8_t i=8;i;--i){
		if (data & 0x80) {
			SDA_TRI;
		} else {
			SDA_LOW;
		}
		data<<=1;
		SCL_TRI;
		i2c_delay();
		SCL_LOW;
		i2c_delay();
	}
	SDA_TRI;
	SCL_TRI;
	i2c_delay();
	uint8_t ack = !SDA_READ;
	SCL_LOW;
	return ack;
}

uint8_t i2c_rx(uint8_t ack) {
	uint8_t data = 0;
	SDA_TRI;
	for (uint8_t i=8;i;--i){
		data <<= 1;
		do {
			SCL_TRI;
		} while (!SCL_READ);
		i2c_delay();
		if (SDA_READ) {
			data |= 1;
		}
		i2c_delay();
		SCL_LOW;
	}
	if (ack) {
		SDA_LOW;
	} else {
		SDA_TRI;
	}
	SCL_TRI;
	i2c_delay();
	SCL_LOW;
	SDA_TRI;
	return data;
}

void i2c_start_write(uint8_t addr, uint8_t reg) { //42 delays
	di();
	i2c_start();
	i2c_tx(addr << 1);
	i2c_tx(reg);
}

uint8_t i2c_read(uint8_t addr, uint8_t reg) {
	i2c_start_write(addr,reg);
	i2c_start();
	i2c_tx((addr << 1) | 1);
	return i2c_rx(1);
}

const uint8_t digits[10][2] = {   //G4 d G3 c G2 g e G1 x x x x b f a G5
		{0b01010100, 0b1110}, //0-abcdef
		{0b00010000, 0b1000}, //1-bc
		{0b01000110, 0b1010}, //2-abdeg
		{0b01010010, 0b1010}, //3-abcdg
		{0b00010010, 0b1100}, //4-bcfg
		{0b01010010, 0b0110}, //5-acdfg
		{0b01010110, 0b0110}, //6-acdefg
		{0b00010000, 0b1010}, //7-abc
		{0b01010110, 0b1110}, //8-abcdefg
		{0b01010010, 0b1110}  //9-abcdfg
	};

const uint8_t grids[5][2] = {
		{0b00000001, 0b11000000},
		{0b00001000, 0b11000000},
		{0b00100000, 0b11000000},
		{0b10000000, 0b11000000},
		{0b00000000, 0b11000001}
	};

volatile uint8_t hour   = 0x00;
volatile uint8_t minute = 0x00;

volatile uint8_t pulseCounter = 0;

volatile uint8_t currPins = 0;
volatile uint8_t prevPins = 0;

void wake();

void btnChange(uint8_t btn) {
	pulseCounter = 0;
}

void fetchTime() {
	minute = i2c_read(ADDR_RTC, 0x01);
	i2c_stop();
	hour = i2c_read(ADDR_RTC, 0x02);
}

ISR(PCINT0_vect) {
	prevPins = currPins;
	currPins = PINB;
	uint8_t btn = 0;
	switch((prevPins^currPins)&PCMSK) {
		case (1 << CLK):;
			fetchTime();
			++pulseCounter;
		break;
		case (1 << BTNA):;
			++btn;
		case (1 << BTNB):;
			btnChange(btn);
		break;
	}
}

void ioEx(uint8_t reg, uint8_t val1, uint8_t val2) { //76 delays
	i2c_start_write(ADDR_IOEXPANDER, reg); //write 0xff to 0x06 and 0x07 (set ports 0,1 to input)
	i2c_tx(val1);
	i2c_tx(val2);
}

void showDigit(uint8_t pos, uint8_t num) {
	i2c_start_write(ADDR_IOEXPANDER, REG_IOEX_PIN);
	for (uint8_t i=0; i<2; ++i) {
		i2c_tx(digits[num][i] | grids[pos][i]);
	}
	ioEx(REG_IOEX_DIR, 0x00, 0x00);
	ioEx(REG_IOEX_DIR, 0xff, 0b00111111); //blank the display without shutting off HV/filament
	_delay_loop_2(MULTIPLEX_DELAY);
}

void sleep() {
	PCMSK = PORTB; // use pin-change for button handling only
	ioEx(REG_IOEX_DIR, 0xff, 0xff); //blank the display
	set_sleep_mode(SLEEP_MODE_PWR_DOWN); //sleep
	sleep_enable();
	ei(); //allow waking with pcint
	sleep_cpu();
	sleep_disable();
	wake();
}

void wake() {
	PCMSK = (1<<CLK) | (1<<BTNA) | (1<<BTNB); // use pin-change for button handling and 1hz clock
	fetchTime();
}

int main(void) {
	//power management
	PRR = 0b00000011; //disable ADC and Timer/Counter0

	//change clock speed
	//CLKPR = 0b10000000;
	//CLKPR = 0b00000000;

	//initialize pins
	PORTB  =  (1<<BTNA) | (1<<BTNB);
	//DDRB  &= ~((1<<SDA) | (1<<SCL) | (1<<CLK) | (1<<BTNA) | (1<<BTNB)); //set up input pins (and tristate i2c)
	currPins = PINB;
	
	//configure RTC interrupt output
	i2c_start_write(ADDR_RTC, 0x07); //write to control register
	i2c_tx(0b01000000); //1hz square wave output
	
	//start RTC timekeeping
	uint8_t secByte = i2c_read(ADDR_RTC, 0x00);
	i2c_start_write(ADDR_RTC, 0x00);
	i2c_tx(0b10000000 | secByte);
	
	//set up interrupts
	GIMSK = 0b00100000; // enable pin-change interrupts
	wake();

	while(1) {
		ei();
		if (pulseCounter > 10 && !((~PINB) & PORTB)) {
			sleep();
		}
		di();
		showDigit(2, 1 + (pulseCounter & 0b1)); //the first digit displayed always blinks, so do the : first

		if (hour & 0b00110000){ //conditionally show hours first digit
			showDigit(0, (hour >> 4) & 0b11);
		}
		showDigit(1, hour & 0b1111);
		showDigit(3, (minute >> 4) & 0b111);
		showDigit(4, minute & 0b1111);
    }
}