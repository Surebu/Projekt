/*
 * M�ls�kningsenhet.c
 *
 * Created: 11/17/2015 9:10:37 AM
 *  Author: hamvi791
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//Commands
const uint8_t STATR = 136;
const char ADCC = 148; //CHAN set to 0 for angular channel and ADEN set to 1 to enable AD conversion
const char ADCR = 128;
const char REFUSAL = 128; //Refusal answer to indicate error at accepting instr.

uint8_t byte = 0; //Keep track of which byte that is recieved
int data = 0; //Data from gyro

//SPI-variables
const uint8_t MOVE_FORWARD_SLOW = 0x1A;
const uint8_t TURN_LEFT_SLOW = 0x16;
const uint8_t STOP = 0x10;
uint8_t command = 0;
uint8_t cnt = 0;
uint8_t dataH = 0;
uint8_t dataL = 0;
uint8_t sensor = 0; //Which sensor sends data
uint8_t sensorData = 0; //Data from sensor

uint8_t byteCount = 0;

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
	sei(); //Enable interrupts
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

void timer_init(){
	TCCR1B |= _BV(WGM12) | _BV(CS10) | _BV(CS12); //Set CTC with prescaling 1024
	TIMSK |= _BV(OCIE1A); //Enable interrupt on compare match, compare register 1A
	OCR1A = 15625; //Roughly 1s, calculate with prescaling of 1024 using the following formula: 1 = (1024*x)/(16*10^6)
}

ISR(TIMER1_COMPA_vect){
	PORTB ^= _BV(PB3);
	
	if(cnt == 0){
		PORTB &= ~_BV(PB1);
		command = STOP;
		SPI_MasterTransmit(STOP);
		PORTB |= _BV(PB1);
		cnt += 1;
	}else if(cnt == 1){
		PORTB &= ~_BV(PB1);
		command = TURN_LEFT_SLOW;
		SPI_MasterTransmit(TURN_LEFT_SLOW);
		PORTB |= _BV(PB1);
		cnt = 0;
	}
}

//Interrupt to receive data from sensorenheten
ISR(INT0_vect){
	
	byteCount += 1;
	
	//Sensor
	if(byteCount == 1){
		PORTB &= ~_BV(PB3);
		sensor = SPI_MasterTransmit(0x0F); //Receive which sensor that wants to send data
		PORTB |= _BV(PB3);
		btTransmit(sensor);
	}
	
	//Data
	else if(byteCount == 2){	
		PORTB &= ~_BV(PB3);
		sensorData = SPI_MasterTransmit(0xF0); //Receive the sensor data
		PORTB |= _BV(PB3);
		byteCount = 0;
		//btTransmit(sensorData);
	}
	
	/*PORTD |= _BV(PD6);
	//Sensor
	PORTB &= ~_BV(PB3);
	sensor = SPI_MasterTransmit(0x0F); //Receive which sensor that wants to send data
	PORTB |= _BV(PB3);
	
	//Data
	PORTB &= ~_BV(PB3);
	sensorData = SPI_MasterTransmit(0x0F); //Receive the sensor data
	PORTB |= _BV(PB3);*/
}

short adcToAngularRate(unsigned short adcValue){
	short vOutAngularRate = (adcValue * 25/12)+400;  // in mV (millivolts)
	return vOutAngularRate;
	
	// from the data sheet, N2 version is 6,67
	//return (vOutAngularRate - 2500)/26.67;
	// E2 is 13,33 and R2 is 26,67 mV/deg
	// change accordingly.
}

void getGyroValue(){
	PORTB &= ~_BV(PB4);
	data += SPI_MasterTransmit(ADCC);
	data += SPI_MasterTransmit(0x00);
	data += SPI_MasterTransmit(0x00);
	PORTB |= _BV(PB4);
	_delay_us(115);

	PORTB &= ~_BV(PB4);
	data += SPI_MasterTransmit(ADCR);
	dataH = SPI_MasterTransmit(0x00); //MSBs
	dataL = SPI_MasterTransmit(0x00); //LSBs
	PORTB |= _BV(PB4);
	
	dataH = dataH & 0x0F; //The 4 highest bits are not adc-values
	dataL = dataL >> 1; //Shift out the lowest bit
	
	//Unsigned makes a difference!!!
	unsigned short adcValue = dataL; //Store the two received bytes to an int
	unsigned short temp = dataH;
	temp = temp << 7;
	adcValue = adcValue + temp;
	
	short angularRate = adcToAngularRate(adcValue);
	
	
	PORTA = angularRate;
	//PORTC = (angularRate >> 8);
}

void btInit(void)
{
	/*16MHz ska ha 115.2k i baud rate och har en felmarginal p� -3.5%*/
	/*ej s�ker p� f�ljande 3 rader*/
	/*f�r att f� �nskad baudrate s� s�tter man F_CPU = 4.7456E56 n�nting fr�ga peter*/

	//DDRA |= _BV(PA2) | _BV(PA3);
	//PORTA &= ~( _BV(PA2) | _BV(PA3) );
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

/*ISR(USART_RXC_vect)
{
	//val = UDR;
	// Echo back received data 
	//btTransmit(val);
}*/

int main(void)
{	
	
	SPI_MasterInit();
	btInit();
	//timer_init();
	
	//DDRA = 0xFF; //Port a output
	//DDRC = 0xC3; //Port c output except for the JTAG-pins
	//DDRB |= _BV(PB4);
	//DDRD &= ~_BV(PD2);
	
	/*PORTB &= ~_BV(PB4);
	data += SPI_MasterTransmit(ADCC);
	data += SPI_MasterTransmit(0x00);
	data += SPI_MasterTransmit(0x00);
	PORTB |= _BV(PB4);
	_delay_us(115);*/
	
    while(1)
    {		
		if(sensorData == 1){
			_delay_ms(2000);	
			PORTB &= ~_BV(PB1);
			SPI_MasterTransmit(2);
			PORTB = _BV(PB1);
		}
		
		/*PORTB &= ~_BV(PB3);
		SPI_MasterTransmit(0xF0);
		PORTB |= _BV(PB3);*/
		
		/*if(command = TURN_LEFT_SLOW){
			getGyroValue();
		}*/
			
		/*PORTB &= ~_BV(PB1);
		SPI_MasterTransmit(MOVE_FORWARD_SLOW);
		PORTB = _BV(PB1);
		_delay_ms(1000);
		PORTB &= ~_BV(PB1);
		SPI_MasterTransmit(STOP);
		PORTB = _BV(PB1);
		_delay_ms(1000);
		PORTB &= ~_BV(PB1);
		SPI_MasterTransmit(TURN_LEFT_SLOW);
		PORTB = _BV(PB1);
		_delay_ms(1000);
		PORTB &= ~_BV(PB1);
		SPI_MasterTransmit(STOP);
		PORTB = _BV(PB1);
		_delay_ms(1000);*/
    }
}