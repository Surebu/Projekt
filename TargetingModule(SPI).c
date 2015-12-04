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

const uint8_t RETRIEVABLE_SENSOR_DATA = 10;

//Robot commands
const uint8_t MOVE_FORWARD_SLOW = 0x1A;
const uint8_t MOVE_FORWARD_FAST = 0x1F;
const uint8_t MOVE_BACK = 0x15;
const uint8_t TURN_RIGHT_SLOW = 0x19;
const uint8_t TURN_LEFT_SLOW = 0x16;
const uint8_t STOP = 0x10;

volatile uint8_t dataValues[14] = {
	1,	//IR-sensor 1	vänster			0
	3,	//IR-sensor 2	bak	§			1
	3,	//IR-sensor 3	fram			2
	7,	//IR-sensor 4	höger			3
	0,	//Tejpsensor 1	fram-vänster	4
	4,	//Tejpsensor 2	bak-vänster		5
	2,	//Tejpsensor 3	bak-höger		6
	0,	//Tejpsensor 4	fram-höger		7
	13,	//Avståndssensor				8
	37,	//Träffdetektor					9
	0,	//Tape values					A
	7,	//Liv							B
	1,	//Kontrolläge					C
	0   //latest_move					D
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
const uint8_t TAPE_VALUES =  10;

uint8_t tapeThreshold = 120;

volatile uint8_t backFlag = 0;
volatile uint8_t turnFlag = 0;
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

//---------------------------------------Commands----------------------
//---------------------------------------------------------------------
//Use this function to move the robot using one of the pre-defined commands
void moveRobot(uint8_t move){
	PORTB &= ~_BV(PB1);
	SPI_MasterTransmit(move);
	PORTB |= _BV(PB1);
	
	dataValues[13] = move;
}

//--------------------------------Timer--------------------------------------
//---------------------------------------------------------------------------

void timer_init(){
	TCCR1B |= _BV(WGM12) | _BV(CS10) | _BV(CS12); //Set CTC with prescaling 1024
	TIMSK |= _BV(OCIE1A); //Enable interrupt on compare match, compare register 1A
	OCR1A = 15625; //Roughly 1s, calculated with prescaling of 1024 using the following formula: 1 = (1024*x)/(16*10^6)
}

ISR(TIMER1_COMPA_vect){
	
	if(backFlag == 1){
		backFlag = 0;
		turnFlag = 1;
	}else if(turnFlag == 1){		
		turnFlag = 0;
		TCCR1B = 0;
	}
	
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------

//---------------------------------------------------------------------
//---------------------------------------------------------------------
//konverterar sensorvärdet och lagrar det i Tapevalues
void ADConvert(){
	for(uint8_t i = 4; i < 8; ++i){
		if (dataValues[i] > tapeThreshold)
		{
			dataValues[TAPE_VALUES] |= _BV(i-4);
		}
		else{
			dataValues[TAPE_VALUES] &= ~_BV(i-4);
		}
	}
}

//Gets all the sensor values from sensorenheten via the SPI-bus
void getSensorValues(){
	for(uint8_t i = 0; i < RETRIEVABLE_SENSOR_DATA; ++i){
		
		PORTB &= ~_BV(PB3);
		SPI_MasterTransmit(i);
		PORTB |= _BV(PB3);
		_delay_us(3);
		
		PORTB &= ~_BV(PB3);
		dataValues[i] = SPI_MasterTransmit(0xAA);
		PORTB |= _BV(PB3);
		_delay_us(3);
	}
	
	ADConvert();
}

//Responds to the request flag sent by the computer and sends back the relevant data
void BT_SensorValues(){
	if(requestFlag == 1){
		cli();
		uint8_t data = dataValues[dataAddress];
		sei();
		btTransmit(data);		//skicka efterfrågat värde till datorn
		requestFlag = 0;	//nu har vi skickat
	}
}

int main(void)
{
	SPI_MasterInit();
	btInit();
	sei();
	btTransmit(0);
	
	DDRD |= _BV(PD5);
	DDRD |= _BV(PD4);
	
	uint8_t frontTapeValues = 0;
	uint8_t backTapeValues = 0;
	uint8_t frontLeftTape = 0;
	uint8_t frontRightTape = 0;
	uint8_t leftOrRight = 0;
	
    while(1)
    {
	
		BT_SensorValues();
		getSensorValues();
				
		//Start of AI program that should keep the robot within the boundaries of the tape track
		
		//Mutexlock, clisei-senpai!!!!!
		cli();
		frontTapeValues = dataValues[TAPE_VALUES] & 0x09;
		sei();
		
		frontLeftTape = frontTapeValues & 0x01;
		frontRightTape = frontTapeValues & 0x08;
		/*if(backFlag == 1){
			moveRobot(STOP);
		}*/
		if(frontLeftTape != 0){
			leftOrRight = 0;
			backFlag = 1;
			moveRobot(MOVE_BACK);
			timer_init();
		}
		else if(frontRightTape != 0){
			leftOrRight = 1;
			backFlag = 1;
			moveRobot(MOVE_BACK);
			timer_init();
		}
		else if(turnFlag == 1){
			if(leftOrRight == 0){
				moveRobot(TURN_LEFT_SLOW);
			}else if(leftOrRight == 1){
				moveRobot(TURN_RIGHT_SLOW);
			}		
		}
		else if ((frontTapeValues == 0x00) && (backFlag == 0) && (turnFlag == 0)){
			moveRobot(MOVE_FORWARD_SLOW);
		}
    }
}