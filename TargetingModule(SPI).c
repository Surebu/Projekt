/*
 * TargetingModule_SPI_.c
 *
 * Created: 12/3/2015 9:01:29 AM
 *  Author: hamvi791
 */ 


#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Kommunikation med datorn
volatile uint8_t requestFlag = 0;	//flagga för att signalera om datorn frågat om ett värde
volatile uint8_t dataAddress = 0;	//vilken "address" ligger värdet som datorn efterfrågat på

volatile uint8_t sensor = 0; //Which sensor sends data
volatile uint8_t sensorData = 0; //Data from sensor

const uint8_t RETIEVABLE_SENSOR_DATA = 11;

volatile uint8_t dataValues[13] = {
	1,	//IR-sensor 1	vänster
	3,	//IR-sensor 2	bak
	3,	//IR-sensor 3	fram
	7,	//IR-sensor 4	höger
	0,	//Tejpsensor 1	fram-vänster
	4,	//Tejpsensor 2	bak-vänster
	2,	//Tejpsensor 3	bak-höger
	0,	//Tejpsensor 4	fram-höger
	13,	//Avståndssensor
	37,	//Träffdetektor
	0,	//Tape values
	7,	//Liv
	1	//Kontrolläge
};

const uint8_t IR_SENSOR_LEFT = 0;
const uint8_t IR_SENSOR_BACK = 1;
const uint8_t IR_SENSOR_FRONT = 2;
const uint8_t IR_SENSOR_RIGHT = 3;

const uint8_t TAPE_SENSOR_FRONT_LEFT = 4;
const uint8_t TAPE_SENSOR_BACK_LEFT = 5;
const uint8_t TAPE_SENSOR_BACK_RIGHT = 6;
const uint8_t TAPE_SENSOR_FRONT_RIGHT = 7;

const uint8_t DISTANCE_SENSOR = 8;
const uint8_t HIT_DETECTOR = 9;
const uint8_t TAPE_VALUES =  12;

//----------------------------------BT----------------------------------
//----------------------------------------------------------------------

void btInit(void)
{
	/* Set baud rate */
	
	UBRRH = 0x0;
	UBRRL = 0x08;		//115200 http://wormfood.net/avrbaudcalc.php
	/* Enable receiver and transmitter */
	UCSRB = _BV(RXEN) | _BV(TXEN) | _BV(RXCIE);
	/* Set frame format: 8data, 1stop bit */
	UCSRC = (1 << URSEL) | (0 << UMSEL) | (0 << UPM1) | (0 << UPM0) | (0 << USBS) | (1 << UCSZ1) | (1 << UCSZ0) | (0 << UCPOL)  ;//_BV(UCSZ0) | _BV(UCSZ1);	//?!?!??!??!?!?!
}

/* Send one byte as soon as transmit buffer is empty.*/
void btTransmit(unsigned char data)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSRA & _BV(UDRE) ));
	/* Put data into buffer, sends the data */
	UDR = data;
}

unsigned char btReceive()
{
	/* Wait for data to be received */
	while (!(UCSRA & _BV(RXC)));
	/* Get and return received data from buffer */
	return UDR;
}

ISR(USART_RXC_vect)
{
	dataAddress = UDR;	//vilken sensor vill dator veta om?
	requestFlag = 1;// sätt flagga att skicka saker
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------


void SPI_MasterInit(void)
{
	/* Set MOSI and SCK output, all others input */
	DDRB = (1<<PB5)|(1<<PB7);
	
	//Gyro
	DDRB |= _BV(PB4); //SS-gyro
	PORTB |= _BV(PB4); //Set SS-signal to high
	
	//Sensorenheten
	DDRB |= _BV(PB3); //SS-sensor
	PORTB |= _BV(PB3); //Set SS-signal to high
	DDRD &= ~_BV(PD2); //Set PD2 as input, interrupt signal from sensorenheten
	
	//Styrenheten
	DDRB |= _BV(PB1); //SS-styrenheten
	PORTB |= _BV(PB1); //Set SS-signal to high
	
	/* Enable SPI, Master, set clock rate fck/16 and enable interrupt when transfer completed*/
	SPCR = (1<<SPE)|(1<<MSTR);
	
	
	MCUCR = _BV(ISC01) | _BV(ISC00);	// Trigger INT0 on rising edge
	GICR = _BV(INT0);
	
	DDRD |= _BV(PD6);
}

unsigned char SPI_MasterTransmit(char cData)
{
	
	/* Start transmission */
	SPDR = cData;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)))
	;
	return SPDR;
}


int main(void)
{
	SPI_MasterInit();
	btInit();
	sei();
	btTransmit(0);
    while(1)
    {
		
		if(requestFlag == 1){
			cli();
			uint8_t data = dataValues[dataAddress];
			sei();
			btTransmit(data);		//skicka efterfrågat värde till datorn
			requestFlag = 0;	//nu har vi skickat
		}
/*
		PORTB &= ~_BV(PB3);
		SPI_MasterTransmit(0xC);
		PORTB |= _BV(PB3);
		_delay_us(5);
		PORTB &= ~_BV(PB3);
		dataValues[0xC] = SPI_MasterTransmit(0xAA);
		PORTB |= _BV(PB3);
		_delay_us(5);
*/		
		for(uint8_t i = 0; i < RETIEVABLE_SENSOR_DATA; ++i){
			PORTB &= ~_BV(PB3);
			SPI_MasterTransmit(i);
			PORTB |= _BV(PB3);
			_delay_us(5);
			PORTB &= ~_BV(PB3);
			dataValues[i] = SPI_MasterTransmit(0xAA);
			PORTB |= _BV(PB3);
			_delay_us(5);
		}		
    }
}