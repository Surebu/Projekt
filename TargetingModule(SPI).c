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

//------------------------Tapethresholds and calibrating----------------------------
const uint8_t tapeThreshold = 120;

volatile uint8_t tapeThresholdFL = 0;
volatile uint8_t tapeThresholdFR = 0;
volatile uint8_t tapeThresholdBL = 0;
volatile uint8_t tapeThresholdBR = 0;

const uint8_t thresholdOffset = 20; //Offset to give the robot some added sensitivity in detecting tape

volatile uint8_t calibrating = 0; //1 if the robot is calibrating the tape thresholds
//-----------------------------------------------------------------------------

//-------------------------Robot commands-------------------------------------------
const uint8_t MOVE_FORWARD_SLOW = 0x1A;
const uint8_t MOVE_FORWARD_FAST = 0x1F;
const uint8_t MOVE_BACK = 0x15;
const uint8_t TURN_RIGHT = 0x17;
const uint8_t TURN_LEFT = 0x1D;
const uint8_t STOP = 0x10;
//------------------------------------------------------------------------------------

//-------------------Sensor data and index constants----------------------------------
volatile uint8_t dataValues[14] = {
	1,	//IR-sensor 1	vänster			0
	3,	//IR-sensor 2	bak				1
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

const uint8_t RETRIEVABLE_SENSOR_DATA = 10; //Number of sensors giving data

volatile uint8_t sensor = 0; //Which sensor sends data
volatile uint8_t sensorData = 0; //Data from sensor

//Index constants to different sensor values in dataValues
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
//----------------------------------------------------------------------------------------

//--------Sensor-control "booleans" to determine behavior the of robot---------------------
uint8_t distanceValue = 0;
uint8_t IRLeft = 0;
uint8_t frontTapeValues = 0;
uint8_t frontLeftTape = 0;
uint8_t frontRightTape = 0;
uint8_t leftOrRight = 0; //If 0 turn left, 1 turn right
//-------------------------------------------------------------------------------------------

//------------------------------Course-correction vairables--------------------------------
//volatile uint8_t backFlag = 0;
//volatile uint8_t turnFlag = 0;
volatile uint8_t correctingCourse = 0; //1 if the robot is correcting its course
volatile uint8_t correctCourseStep = 0; //Which step the robot is on in its course-correcting
//-------------------------------------------------------------------------------------------

//-------------------------------------Gyro---------------------------
//---------------------------------------------------------------------

/*void initGyro(){
	DDRB |= _BV(PB4);
	PORTB &= ~_BV(PB4);
	data += SPI_MasterTransmit(ADCC);
	data += SPI_MasterTransmit(0x00);
	data += SPI_MasterTransmit(0x00);
	PORTB |= _BV(PB4);
	_delay_us(115);
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
}*/
//---------------------------------------------------------------------
//---------------------------------------------------------------------

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

void timer_init(){
	TCCR1B |= _BV(WGM12) | _BV(CS10) | _BV(CS12); //Set CTC with prescaling 1024
	TIMSK |= _BV(OCIE1A); //Enable interrupt on compare match, compare register 1A
	OCR1A = 15625; //Roughly 1s, calculated with prescaling of 1024 using the following formula: 1 = (1024*x)/(16*10^6), 5*(16*10^6)/1024 = x 
}

//Interrupt that increments and resets the variables determining the behavior of the course-correction of the robot
ISR(TIMER1_COMPA_vect){
	
	/*if(backFlag == 1){
		backFlag = 0;
		turnFlag = 1;
	}else if(turnFlag == 1){		
		turnFlag = 0;
		TCCR1B = 0;
	}*/
	correctCourseStep += 1;
	if(correctCourseStep == 2){
		correctCourseStep = 0;
		correctingCourse = 0;
		TCCR1B = 0;
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

//Interrupt that sets the value for each of the tapesensor's thresholds minus a offset to give added sensitivity
ISR(INT1_vect){
	if(calibrating == 0){
		tapeThresholdBL = (dataValues[TAPE_SENSOR_BACK_LEFT] - thresholdOffset);
		tapeThresholdBR = (dataValues[TAPE_SENSOR_BACK_RIGHT] - thresholdOffset);
		tapeThresholdFL = (dataValues[TAPE_SENSOR_FRONT_LEFT] - thresholdOffset);
		tapeThresholdFR = (dataValues[TAPE_SENSOR_FRONT_RIGHT] - thresholdOffset);
		
		/*dataValues[1] = tapeThresholdBL;
		dataValues[2] = tapeThresholdBR;
		dataValues[0] = tapeThresholdFL;
		dataValues[3] = tapeThresholdFR;*/
	}
	
	calibrating += 1;
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------

//-----------------------------AI functions----------------------------
//---------------------------------------------------------------------
//konverterar sensorvärdet och lagrar det i Tapevalues
void ADConvert(){
	
	//Front left
	if(dataValues[TAPE_SENSOR_FRONT_LEFT] > tapeThresholdFL){
		dataValues[TAPE_VALUES] |= _BV(0);
	}else{
		dataValues[TAPE_VALUES] &= ~_BV(0);
	}
	
	//Back left
	if(dataValues[TAPE_SENSOR_BACK_LEFT] > tapeThresholdBL){
		dataValues[TAPE_VALUES] |= _BV(1);
	}else{
		dataValues[TAPE_VALUES] &= ~_BV(1);
	}
	
	//Back right
	if(dataValues[TAPE_SENSOR_BACK_RIGHT] > tapeThresholdBR){
		dataValues[TAPE_VALUES] |= _BV(2);
	}else{
		dataValues[TAPE_VALUES] &= ~_BV(2);
	}
	
	//Front right
	if(dataValues[TAPE_SENSOR_FRONT_RIGHT] > tapeThresholdFR){
		dataValues[TAPE_VALUES] |= _BV(3);
	}else{
		dataValues[TAPE_VALUES] &= ~_BV(3);
	}
	
}

//Gets all the sensor values from sensorenheten via the SPI-bus
void getSensorValues(){
	for(uint8_t i = 0; i < RETRIEVABLE_SENSOR_DATA; ++i){
		
		PORTB &= ~_BV(PB3);
		SPI_MasterTransmit(i);
		PORTB |= _BV(PB3);
		_delay_us(10);
		
		
		PORTB &= ~_BV(PB3);	
		dataValues[i] = SPI_MasterTransmit(0xAA);
		PORTB |= _BV(PB3);
		_delay_us(10);
		
	}
	ADConvert();
}

//Sets the variables being used in the AI program that needs the sensor values
void setVariables(){
	//Mutexlock, clisei-senpai!!!!!
	cli();
	frontTapeValues = dataValues[TAPE_VALUES] & 0x09;
	sei();
	
	cli();
	distanceValue = dataValues[DISTANCE_SENSOR];
	sei();
	
	cli();
	IRLeft = dataValues[IR_SENSOR_LEFT];
	sei();
	
	frontLeftTape = frontTapeValues & 0x01;
	frontRightTape = frontTapeValues & 0x08;
}

//Inits the timer and tells the AI that is should correct the course of the robot
void correctCourse(){
	correctingCourse = 1;
	timer_init();
	//backFlag = 1; 
	//moveRobot(MOVE_BACK);	
}

void turn(){
	correctingCourse = 1;
	correctCourseStep = 1;
	timer_init();
}

//Executes the steps that correct the course of the robot by backing for one second and then turning for a second
void executeCorrectionSteps(){
	
	//First step: Move back 
	if(correctCourseStep == 0){
		moveRobot(MOVE_BACK);
	}
	
	//Second step: Turn 
	else if(correctCourseStep == 1){
		if(leftOrRight == 0){
			moveRobot(TURN_LEFT);
		}else if(leftOrRight == 1){
			moveRobot(TURN_RIGHT);
		}
	}
}

//Controls the different sensor values and calls functions accordingly 
void idle(){
	if(frontLeftTape != 0){
		leftOrRight = 1;
		correctCourse();
	}
	else if(frontRightTape != 0){
		leftOrRight = 0;
		correctCourse();
	}
	
	else if (distanceValue <= 20)
	{
		leftOrRight = 1;
		correctCourse();
	}
	
	else if(IRLeft != 4 && IRLeft < 8){
		leftOrRight = 0;
		turn();
	}
	else if ((frontTapeValues == 0x00)){
		moveRobot(MOVE_FORWARD_FAST);
	}
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------

int main(void)
{
	SPI_MasterInit();
	btInit();
	interruptINT1_init();
	sei();
	btTransmit(0);
	
    while(1)
    {
		BT_SensorValues();
		getSensorValues();
		setVariables();	
		
		//Start of AI program that should keep the robot within the boundaries of the tape track	
		if(correctingCourse == 1){
			executeCorrectionSteps();
		}
		else{
			idle();
		}
    }
}