/*
 * Sensorenhet.c
 *
 * Created: 11/13/2015 10:35:06 AM
 *  Author: henpe071
 */ 


#define F_CPU 16000000UL
#define PA1HIGH   ((PINA & (1<<PA1)))
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

volatile uint16_t distance = 0; // in cm
volatile uint8_t IRSTOPFLAG = 0;
uint8_t tapeThreshold = 128;
uint8_t IRsignals[4];
uint8_t TapeValues;

volatile uint8_t byteCount = 0; //Viktigt att den är volatile

//SPI-constants
<<<<<<< HEAD
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
const uint8_t TAPE_VALUES = 10;
=======
const uint8_t TAPE_SENSOR_FRONT_LEFT = 0x04;
const uint8_t TAPE_SENSOR_FRONT_RIGHT = 0x07;
const uint8_t TAPE_SENSOR_BACK_LEFT = 0x05;
const uint8_t TAPE_SENSOR_BACK_RIGHT = 0x06;

const uint8_t IR_SENSOR_FRONT = 0x02;
const uint8_t IR_SENSOR_RIGHT = 0x03;
const uint8_t IR_SENSOR_BACK = 0x01;
const uint8_t IR_SENSOR_LEFT = 0x00;

const uint8_t DISTANCE_SENSOR = 0x08;
const uint8_t HIT_DETECTOR = 0x09;
const uint8_t TAPE_VALUES = 0x0C;
>>>>>>> origin/master

uint8_t hit = 0;

ISR(INT0_vect){
	distance = 0;			// set dist to 0
	TCNT1 = 0;
	while(PA1HIGH)
	{
		if(TCNT1 > 927){	//16mhz clock => 16 pulses/us => (16*58)-1 = distance in cm
			TCNT1 = 0;		//reset timer
			distance++;		//+1cm
		}
	}
<<<<<<< HEAD
	//sendData(DISTANCE_SENSOR, distance);
	//sendData(DISTANCE_SENSOR);
	//sendData(distance); 
=======
>>>>>>> origin/master
}

ISR(TIMER1_COMPA_vect){
	IRSTOPFLAG = 1;
}
void enableInterrupts(){
	GICR = 1<<INT0;					// Enable INT0
	MCUCR = 1<<ISC01 | 1<<ISC00;	// Trigger INT0 on rising edge	
	TIMSK |= _BV(OCIE1A);			//enable timer interrupts
	OCR1A = 7000;					//64*x/16000000 = 28*10^-3 => x = 7000
}
void initPorts(){
	//ALLA PORTAR
	//DDRB |= _BV(PB0) | _BV(PB1) | _BV(PB2) | _BV(PB3);	//muxenable , muxmuxmux
	DDRA |= _BV(PA2);									//PA2 triggersignal dist.
	DDRB = 0xFF;// _BV(PB4) | _BV(PB5) | _BV(PB6) | _BV(PB7); //tests
	
}
void initIRSensors(){
	TCCR0 |= _BV(CS02);	//prescaler 256
	
	TCCR1B = 0;
	TCCR1B |= _BV(CS10) | _BV(CS11) | _BV(WGM12);//CTC, 64 prescaler
	 
	IRSTOPFLAG = 0;
	TCNT0 = 0;	
}

void initDistanceSensor(){
	TCCR1B = 0;	
	TCCR1B |= _BV(CS10);	//start 16bit timer
}

void initADConverter(){
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescalar to 128 - 125KHz sample rate @ 16MHz

	ADMUX |= (1 << REFS0); // Set ADC reference to AVCC
	ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading

	ADMUX |= _BV(MUX0) | _BV(MUX1) | _BV(MUX2); //Set to ADC7 (0111)
	
	ADCSRA |= (0 << ADATE);  // Set ADC to single Mode
	ADCSRA |= (1 << ADEN);  // Enable ADC
	ADCSRA |= (1 << ADSC);  // Start A2D Conversions	
}

void SPI_init(){
	DDRB |= _BV(PB6); //MISO-pin configured as output
	DDRB |= _BV(PB3); //Pin that sends a signal to interrupt the targeting module when a sensor data should be sent
	
	SPCR |= _BV(SPE) | _BV(SPIE); //Set as slave, SPI-enable set and interrupts enabled
}

//Laddar data som ska skickas över SPIn
void sendData(uint8_t data){
	SPDR = data; //Load data to be sent
	PORTB |= _BV(PB3);
	//while(!(SPSR & (1<<SPIF))); //Wait for transfer to be completed, FUNKAR INTE
	PORTB &= ~_BV(PB3);
	while(byteCount < 1);
	byteCount = 0;
	PORTD &= 0x9F;
}

//Räknar hur många bytes vi har skickat över SPIn
ISR(SPI_STC_vect){ //www.avrfreaks.net/forum/spif-flag-spi-interface
	//PORTD |= _BV(PD6);
	byteCount += 1;
	//PORTD &= 0x9F;
	//PORTD |= byteCount << 5;
	//PORTD &= ~_BV(PD6);
}

//Triggersignal för att starta avståndssensorn
void triggerSignal(){
	PORTA &= ~_BV(PA2);
	_delay_us(100);
	PORTA |= _BV(PA2);	//Begin the trigger signal
	_delay_us(20);		//Wait for 20 us
	PORTA &= ~_BV(PA2);	//End the signal
}

//Läser av tejpsensor nr ch
uint8_t adc_read(uint8_t ch)
{
	// select the corresponding channel 0~7
	// ANDing with ’7? will always keep the value
	// of ‘ch’ between 0 and 7
	ch &= 0b00000111;  // AND operation with 7
	ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing
	
	// start single convertion
	// write ’1? to ADSC
	ADCSRA |= (1<<ADSC);
	
	// wait for conversion to complete
	// ADSC becomes ’0? again
	// till then, run loop continuously
	while(ADCSRA & (1<<ADSC));
	
	return (ADCH);
}

void readTapeSensors(){
	for (uint8_t tapeNum = 0; tapeNum<4; tapeNum++)
	{
		if (tapeNum == 0)
		{			
			sendData(TAPE_SENSOR_FRONT_LEFT);
			sendData(adc_read(6));
			adc_read(6);
			ADConvert(tapeNum);	
		}
		else if (tapeNum == 1)
		{
			sendData(TAPE_SENSOR_BACK_LEFT);
			sendData(adc_read(7));
			adc_read(7);
			ADConvert(tapeNum);
		}
		else if (tapeNum == 2)
		{
			sendData(TAPE_SENSOR_BACK_RIGHT);
			sendData(adc_read(5));
			adc_read(5);		//måste användas om vi skickar Tapevalues som sendData
			ADConvert(tapeNum);
		}
		else if (tapeNum == 3)
		{
<<<<<<< HEAD
			//sendData(TAPE_SENSOR_FRONT_RIGHT, adc_read(4));
			sendData(TAPE_SENSOR_FRONT_RIGHT);
			sendData(adc_read(4));
			ADConvert(tapeNum);
		}
	}
	sendData(TAPE_VALUES);
	sendData(TapeValues);
=======
			sendData(TAPE_SENSOR_FRONT_RIGHT);
			sendData(adc_read(4));
			adc_read(4);
			ADConvert(tapeNum);
		}
	}
>>>>>>> origin/master
}

//konverterar sensorvärdet och lagrar det i Tapevalues
void ADConvert(uint8_t tapeNum){
	if (ADCH < tapeThreshold)
	{
		TapeValues |= _BV(tapeNum);
	}
	else{
		TapeValues &= ~_BV(tapeNum);
	}
}

void readIRSensor(uint8_t num){
	initIRSensors();
	IRsignals[num] = 0;
	uint8_t data = 0;
	uint8_t state = 0; //0 = startbit, 1 = data
	uint8_t cnt = 0; //Counts the number of databits that have been sent
    while(!IRSTOPFLAG) // Count up while 1
    {
		if (!(PINA & (1<<PA3))) // if 0
		{
			TCNT0 = 0;
			while (!IRSTOPFLAG) // Count up while 0
			{
				if (PINA & (1<<PA3)) // if 1
				{
					
					if (TCNT0 >= 120 && state == 0)
					{
						//data = 0;
						state = 1;
						cnt = 0;
					}
					else if (TCNT0 >= 60 && state == 1)
					{
						data = data << 1;
						data++;
						cnt++;
					}
					else if (TCNT0 >= 30 && state == 1)
					{
						data = data << 1;
						cnt++;
					}
					if (cnt >= 3)
					{
						state = 0;
						cnt = 0;
						IRsignals[num] = data & 0x07;
					}
					TCNT0 = 0;
					break;
				}
			}
		}
	}
}

void readIRSensors(){
	PORTB &= ~_BV(PB2);	//enable mux x
    for (uint8_t num = 0; num<4; num++)//Loop over the sensors
    {
		PORTB &= 0xFC;
	    PORTB |= num; //set portb to 000000xx, where xx is num
		readIRSensor(num);
    }
<<<<<<< HEAD
	for (uint8_t num = 0; num<4; num++)//Loop to send all IR-sensor values
	{
		switch(num){
			case 0:
				//sendData(IR_SENSOR_LEFT, IRsignals[num]);
				sendData(IR_SENSOR_LEFT);
				sendData(IRsignals[num]);
				break;
			case 1:
				//sendData(IR_SENSOR_BACK, IRsignals[num]);
				sendData(IR_SENSOR_BACK);
				sendData(IRsignals[num]);
				break;
			case 2:
				//sendData(IR_SENSOR_FRONT, IRsignals[num]);
				sendData(IR_SENSOR_FRONT);
				sendData(IRsignals[num]);
				break;
			case 3:
				//sendData(IR_SENSOR_RIGHT, IRsignals[num]);
				sendData(IR_SENSOR_RIGHT);
				sendData(IRsignals[num]);
				break;
			default:
				sendData(IR_SENSOR_RIGHT);
				sendData(IRsignals[num]);
		}
	}
=======
>>>>>>> origin/master
	
	PORTB |= _BV(PB2);	//disable mux x
}


void outputValues(){
	//PORTB &= 0x0F;		//Clear the port except the select bits to the MUX
	
	//PORTB |= (TapeValues << 4); //00011100, tape works
	
	//PORTB |= (IRsignals[2] << 2) & 0x1C; //00011100
	//PORTB |= (IRsignals[2] << 5) & 0xE0; //11100000, IR-sensor 0 funkar inte
	
	//Distance sensor
	//PORTB |= (distance << 3); //visar 1 cm för mycket
	
	for (uint8_t num = 0; num<4; num++)//Loop to send all IR-sensor values
	{
		switch(num){
			case 0:
			sendData(IR_SENSOR_LEFT);
			sendData(IRsignals[num]);
			break;
			case 1:
			sendData(IR_SENSOR_BACK);
			sendData(IRsignals[num]);
			break;
			case 2:
			sendData(IR_SENSOR_FRONT);
			sendData(IRsignals[num]);
			break;
			case 3:
			sendData(IR_SENSOR_RIGHT);
			sendData(IRsignals[num]);
			break;
			default:
			sendData(0xFF);
			sendData(0xFF);
		}
	}
	
	sendData(DISTANCE_SENSOR);
	sendData(distance);
	
	sendData(TAPE_VALUES);
	sendData(TapeValues);
}

void readLaserDetector(){
	if(hit == 1){
		sendData(HIT_DETECTOR);
		sendData(hit);
	}
}

int main(void)
{
	enableInterrupts();
	initADConverter();
	initPorts();
	SPI_init();
	sei();
		
    while(1)
    {
		//readLaserDetector();
		readIRSensors();
		readTapeSensors();
		initDistanceSensor();
		triggerSignal();
		//_delay_ms(20);
		_delay_ms(10);
		outputValues();
		//_delay_ms(10);
		//check avstandssensor
    }
}