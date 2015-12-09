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

//Tapethresholds
const uint8_t tapeThreshold = 120;

volatile uint8_t tapeThresholdFL = 0;
volatile uint8_t tapeThresholdFR = 0;
volatile uint8_t tapeThresholdBL = 0;
volatile uint8_t tapeThresholdBR = 0;

const uint8_t thresholdOffset = 10; 

volatile uint8_t calibrating = 0;

//Robot commands
const uint8_t MOVE_FORWARD_SLOW = 0x1A;
const uint8_t MOVE_FORWARD_FAST = 0x1F;
const uint8_t MOVE_BACK = 0x15;
const uint8_t TURN_RIGHT = 0x1D;
const uint8_t TURN_LEFT = 0x17;
const uint8_t STOP = 0x10;
const uint8_t IR_OFF = 0x31; //=========D
const uint8_t IR_ON = 0x32; //=========D
const uint8_t ACTIVATE_HIT = 0x33; //=========D

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

volatile uint8_t backFlag = 0;
volatile uint8_t turnFlag = 0;
volatile uint8_t hitFlag = 0; // =========D
volatile uint8_t diodCount = 7; // =========D
volatile uint16_t timer0_5sec = 0; // =========D
volatile uint8_t deadFlag = 0;   // =========D

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

//---------------------------------------SPI---------------------------
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
//---------------------------------------------------------------------
//---------------------------------------------------------------------

//---------------------------------------Commands----------------------
//---------------------------------------------------------------------
//Use this function to move the robot using one of the pre-defined commands
void moveRobot(uint8_t move){
	
	if(calibrating >= 2){
		PORTB &= ~_BV(PB1);
		SPI_MasterTransmit(move);
		PORTB |= _BV(PB1);
	}
	dataValues[13] = move;
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------

//--------------------------------Timer--------------------------------------
//---------------------------------------------------------------------------

void timer1_init(){												
	TCCR1B |= _BV(WGM12) | _BV(CS10) | _BV(CS12); //Set CTC with prescaling 1024
	TIMSK |= _BV(OCIE1A); //Enable interrupt on compare match, compare register 1A
	OCR1A = 15625; //Roughly 1s, calculated with prescaling of 1024 using the following formula: 1 = (1024*x)/(16*10^6)
}

ISR(TIMER1_COMPA_vect){
	
}

void timer0_init(){  //=========D
	TCCR0 |= _BV(WGM01) | _BV(CS00) | _BV(CS02); //Set CTC with prescaling 1024
	TIMSK |= _BV(OCIE0); //Enable interrupt on compare match, compare register 0
	OCR0 = 255; //Roughly 16 ms, calculated with prescaling of 1024 using the following formula: 0,016 = (1024*x)/(16*10^6)
}

ISR(TIMER0_COMPA_vect){ //=============D
	timer0_5sec++;
	if(timer0_5sec == 307){
		timer0_5sec = 0;
		moveRobot(IR_ON);
		moveRobot(ACTIVATE_HIT);
	}
}


//---------------------------------------------------------------------
//---------------------------------------------------------------------

//--------------------------------Tape threshold calibrating------------
//----------------------------------------------------------------------
void interruptINT1_init(){
	
	MCUCR = _BV(ISC11) | _BV(ISC10);	// Trigger INT1 on rising edge
	GICR = _BV(INT1);
}

//Interrupt that sets the value for each of the tapesensor's thresholds
ISR(INT1_vect){
	if(calibrating == 0){
		tapeThresholdBL = (dataValues[TAPE_SENSOR_BACK_LEFT] - thresholdOffset);
		tapeThresholdBR = (dataValues[TAPE_SENSOR_BACK_RIGHT] - thresholdOffset);
		tapeThresholdFL = (dataValues[TAPE_SENSOR_FRONT_LEFT] - thresholdOffset);
		tapeThresholdFR = (dataValues[TAPE_SENSOR_FRONT_RIGHT] - thresholdOffset);
		
		dataValues[1] = tapeThresholdBL;
		dataValues[2] = tapeThresholdBR;
		dataValues[0] = tapeThresholdFL;
		dataValues[3] = tapeThresholdFR;
	}
	
	calibrating += 1;
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------


//konverterar sensorvärdet och lagrar det i Tapevalues
void ADConvert(){
	/*for(uint8_t i = 4; i < 8; ++i){
		if (dataValues[i] > tapeThreshold)
		{
			dataValues[TAPE_VALUES] |= _BV(i-4);
		}
		else{
			dataValues[TAPE_VALUES] &= ~_BV(i-4);
		}
	}*/
	
	if(dataValues[TAPE_SENSOR_FRONT_LEFT] > tapeThresholdFL){
		dataValues[TAPE_VALUES] |= _BV(0);
	}else{
		dataValues[TAPE_VALUES] &= ~_BV(0);
	}
	if(dataValues[TAPE_SENSOR_BACK_LEFT] > tapeThresholdBL){
		dataValues[TAPE_VALUES] |= _BV(1);
	}else{
		dataValues[TAPE_VALUES] &= ~_BV(1);
	}
	if(dataValues[TAPE_SENSOR_BACK_RIGHT] > tapeThresholdBR){
		dataValues[TAPE_VALUES] |= _BV(2);
	}else{
		dataValues[TAPE_VALUES] &= ~_BV(2);
	}
	if(dataValues[TAPE_SENSOR_FRONT_RIGHT] > tapeThresholdFR){
		dataValues[TAPE_VALUES] |= _BV(3);
	}else{
		dataValues[TAPE_VALUES] &= ~_BV(3);
	}
	
}

//Gets all the sensor values from sensorenheten via the SPI-bus
void getSensorValues(){
	for(uint8_t i = 4; i < RETRIEVABLE_SENSOR_DATA; ++i){
		
		PORTB &= ~_BV(PB3);
		SPI_MasterTransmit(i);
		PORTB |= _BV(PB3);
		_delay_us(5);
		
		
		PORTB &= ~_BV(PB3);	
		dataValues[i] = SPI_MasterTransmit(0xAA);
		PORTB |= _BV(PB3);
		_delay_us(5);
		
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

//Loops until the tape thresholds have been calibrated
void calibrateLoop(){
	
	while(calibrating == 1);
}

void dead(){
	moveRobot(MOVE_FORWARD_FAST);
	if(frontLeftTape != 0 || frontRightTape != 0){
													// ===================D Timer'
		moveRobot(STOP);
	}
}

int main(void)
{
	SPI_MasterInit();
	btInit();
	interruptINT1_init();
		
	DDRD |= _BV(PD5);
	DDRD |= _BV(PD4);	
	uint8_t frontTapeValues = 0;
	uint8_t backTapeValues = 0;
	uint8_t frontLeftTape = 0;
	uint8_t frontRightTape = 0;
	uint8_t leftOrRight = 0;
	uint8_t hit = 0;		// ===============D
	
	//getSensorValues();
	sei();
	//calibrateLoop();
	btTransmit(0);
	
    while(1)
    {
	
		BT_SensorValues();
		getSensorValues();
				
		//Start of AI program that should keep the robot within the boundaries of the tape track
		
		//Mutexlock, clisei-senpai!!!!!
		cli();
		frontTapeValues = dataValues[TAPE_VALUES] & 0x09;
		hit = dataValues[HIT_DETECTOR]; //============D
		sei();
		
		frontLeftTape = frontTapeValues & 0x01;
		frontRightTape = frontTapeValues & 0x08;
		/*if(backFlag == 1){
			moveRobot(STOP);
		}*/
		
		if(!deadFlag){
			if(hit && !hitFlag){				// =====================D
				hitFlag = 1;
				diodCount >> 1;
				moveRobot(IR_OFF);
				moveRobot(0x40 | diodCount);
				if(!diodCount){
					deadFlag = 1;
				}
				else {
					timer0_init();
				}
			}
		
			if(frontLeftTape != 0){
				leftOrRight = 0;
				backFlag = 1;
				moveRobot(MOVE_BACK);
				timer1_init();
			}
			else if(frontRightTape != 0){
				leftOrRight = 1;
				backFlag = 1;
				moveRobot(MOVE_BACK);
				timer1_init();
			}
			else if(turnFlag == 1){
				if(leftOrRight == 0){
					moveRobot(TURN_LEFT);
				}else if(leftOrRight == 1){
					moveRobot(TURN_RIGHT);
				}		
			}
			else if ((frontTapeValues == 0x00) && (backFlag == 0) && (turnFlag == 0)){
				moveRobot(MOVE_FORWARD_FAST);
			}
		}
		else{								// =========D
			dead();
		}
	}
	
}

