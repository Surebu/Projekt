
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
volatile uint8_t tapeFL = 0;
volatile uint8_t tapeFR = 0;
volatile uint8_t tapeThresholdFL = 0;
volatile uint8_t tapeThresholdFR = 0;
volatile uint8_t floorFL = 0;
volatile uint8_t floorFR = 0;
//const uint8_t thresholdOffset = 20; //Offset to give the robot some added sensitivity in detecting tape
const uint16_t TIMER_1A_SECOND = 15625;	//Roughly 1s, calculated with prescaling of 1024 using the following formula: 1 = (1024*x)/(16*10^6), 5*(16*10^6)/1024 = x 

volatile uint8_t calibrating = 0; //1 if the robot is calibrating the tape thresholds
//-----------------------------------------------------------------------------

//-------------------------Robot commands-------------------------------------------
const uint8_t MOVE_FORWARD_SLOW = 0x1A;
const uint8_t MOVE_FORWARD_FAST = 0x1F;
const uint8_t MOVE_BACK = 0x15;
const uint8_t TURN_RIGHT = 0x17;
const uint8_t TURN_LEFT = 0x1D;
const uint8_t STOP = 0x10;
const uint8_t ACTIVATE_LASER = 0x21;
const uint8_t DEACTIVATE_LASER = 0x22;
const uint8_t ACTIVATE_HIT = 0x33;
const uint8_t LED = 0x40;
const uint8_t IR_OFF = 0x31;
const uint8_t IR_ON = 0x32;
//------------------------------------------------------------------------------------

//-------------------Sensor data and index constants----------------------------------
volatile uint8_t dataValues[14] = {
	8,	//IR-sensor 1	vänster			0
	8,	//IR-sensor 2	bak				1
	8,	//IR-sensor 3	fram			2
	8,	//IR-sensor 4	höger			3
	0,	//Tejpsensor 1	fram-vänster	4
	0,	//Tejpsensor 2	bak-vänster		5
	0,	//Tejpsensor 3	bak-höger		6
	0,	//Tejpsensor 4	fram-höger		7
	100,	//Avståndssensor				8
	0,	//Träffdetektor					9
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
const uint8_t LIFE =  11; 
//----------------------------------------------------------------------------------------

//--------Sensor-control "booleans" to determine behavior the of robot---------------------
uint8_t distanceValue = 0;

uint8_t IRLeft = 8;
uint8_t IRRight = 8;
uint8_t IRFront = 8;
uint8_t IRBack = 8;

//uint8_t frontTapeValues = 0;
uint8_t backTapeValues = 0;
uint8_t frontLeftTape = 0;
uint8_t frontRightTape = 0;
uint8_t backLeftTape = 0;
uint8_t backRightTape = 0;
uint8_t leftOrRight = 0; //If 0 turn left, 1 turn right
uint8_t hit = 0;

volatile uint8_t forward = 1;
volatile uint8_t turning = 0;
volatile uint8_t backing = 0;
volatile uint8_t backnTurn = 0; 
volatile uint8_t IRFound = 0;
volatile uint8_t sprayPray = 0;
volatile uint8_t sprayFlag = 0;
volatile uint8_t hitFlag = 1;
volatile uint8_t TapeFlag = 0;
volatile uint8_t counting = 0;
volatile uint8_t laserCd = 0; //1 if laser is on cooldown, 0 if it isn't

volatile uint8_t activateHitFlag = 0;
volatile uint16_t timer0_5sec = 0;
volatile uint16_t timer2_3sec = 0;

volatile uint8_t lifeCount = 0x07;

uint16_t timerValue = 0;
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
	
	if(calibrating >= 3){
		PORTB &= ~_BV(PB1);
		SPI_MasterTransmit(move);
		PORTB |= _BV(PB1);
	}
	dataValues[13] = move;
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------

//--------------------------------Timers-------------------------------------
//---------------------------------------------------------------------------

void timer0_init(){
	TCNT0 = 0;
	TCCR0 |= _BV(WGM01) | _BV(CS00) | _BV(CS02); //Set CTC with prescaling 1024
	TIMSK |= _BV(OCIE0); //Enable interrupt on compare match, compare register 0
	OCR0 = 250; //Roughly 16 ms, calculated with prescaling of 1024 using the following formula: 0,016 = (1024*x)/(16*10^6)
	
}

ISR(TIMER0_COMP_vect){
	timer0_5sec++;
	if(timer0_5sec >= 300){
		timer0_5sec = 0;
		activateHitFlag = 1;
		//hitFlag = 0;
		TCCR0 = 0;
		TIMSK &= ~_BV(OCIE0);
	}
}

//Laser cooldown
void timer2_init(){
	TCNT2 = 0;
	TCCR2 |= _BV(WGM21) | _BV(CS20) | _BV(CS21) | _BV(CS22); //Set CTC with prescaling 1024
	TIMSK |= _BV(OCIE2); //Enable interrupt on compare match, compare register 0
	OCR2 = 250; //Roughly 16 ms, calculated with prescaling of 1024 using the following formula: 0,016 = (1024*x)/(16*10^6)
	
}

ISR(TIMER2_COMP_vect){
	timer2_3sec++;
	if(timer2_3sec >= 188){
		timer2_3sec = 0;
		laserCd = 0;
		TCCR2 = 0;
		TIMSK &= ~_BV(OCIE2);
	}
}

void timer_init(){
	TCNT1 = 0;
	TCCR1B |= _BV(WGM12) | _BV(CS10) | _BV(CS12); //Set CTC with prescaling 1024
	TIMSK |= _BV(OCIE1A); //Enable interrupt on compare match, compare register 1A
	OCR1A = timerValue; //Roughly 1s, calculated with prescaling of 1024 using the following formula: 1 = (1024*x)/(16*10^6), 5*(16*10^6)/1024 = x 
}

//Interrupt that increments and resets the variables determining the behavior of the course-correction of the robot
ISR(TIMER1_COMPA_vect){

	if(!lifeCount && counting == 1){
		counting += 1;
	}
	if (backnTurn){
		OCR1A = TIMER_1A_SECOND*0.75;
		backnTurn = 0;
		turning = 1;
		backing = 0;
	}
	else if(sprayPray && !sprayFlag){
		leftOrRight = !leftOrRight;
		OCR1A = 2*timerValue;
		sprayFlag = 1;
	}
	/*else if(sprayPray && sprayFlag){
		timer2_init();
		laserCd = 1;
		forward = 1;
		turning = 0;
		backing = 0;
		IRFound = 0;
		sprayPray = 0;
		sprayFlag = 0;
	}*/
	else{
		if(sprayPray && sprayFlag){
			laserCd = 1;
			timer2_init();
		}
		forward = 1;
		turning = 0;
		backing = 0;
		IRFound = 0;
		sprayPray = 0;
		sprayFlag = 0;
		TCCR1B = 0;
		TIMSK &= ~_BV(OCIE1A);
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
		tapeFL = (dataValues[TAPE_SENSOR_FRONT_LEFT]);
		tapeFR = (dataValues[TAPE_SENSOR_FRONT_RIGHT]);
		
		/*dataValues[0] = tapeThresholdFL;
		dataValues[3] = tapeThresholdFR;*/
	}
	else if(calibrating == 1){
		floorFL = (dataValues[TAPE_SENSOR_FRONT_LEFT]);
		floorFR = (dataValues[TAPE_SENSOR_FRONT_RIGHT]);
		
		tapeThresholdFL = 102;//((tapeFL) + (floorFL))/2;
		tapeThresholdFR = 109;//((tapeFR) + (floorFR))/2;
		dataValues[TAPE_SENSOR_BACK_LEFT] = tapeThresholdFL;
		dataValues[TAPE_SENSOR_BACK_RIGHT] = tapeThresholdFR;
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
	if(dataValues[TAPE_SENSOR_BACK_LEFT] > tapeThresholdFL){
		dataValues[TAPE_VALUES] |= _BV(1);
	}else{
		dataValues[TAPE_VALUES] &= ~_BV(1);
	}
	
	//Back right
	if(dataValues[TAPE_SENSOR_BACK_RIGHT] > tapeThresholdFR){
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
	uint8_t temp = 0;
	for(uint8_t i = 0; i < RETRIEVABLE_SENSOR_DATA; ++i){
		if(i != 1 && i != 5 && i != 6){
		
			PORTB &= ~_BV(PB3);
			SPI_MasterTransmit(i);
			PORTB |= _BV(PB3);
			_delay_us(10);
			
			
			PORTB &= ~_BV(PB3);
			temp = SPI_MasterTransmit(0xAA);
			PORTB |= _BV(PB3);
			
			cli();
			dataValues[i] = temp;
			sei();
			
			_delay_us(10);
		}	
	}
	ADConvert();
}

//Sets the variables being used in the AI program that needs the sensor values
void setVariables(){
	//Mutexlock, clisei-senpai!!!!!
	//cli();
	IRBack |= forward;
	IRBack |= turning << 1;
	IRBack |= backing << 2;
	IRBack |= backnTurn << 3;
	IRBack |= IRFound << 4;
	IRBack |= sprayPray << 5;
	IRBack |= sprayFlag << 6;
	IRBack |= hitFlag << 7;
	
	backRightTape |= TapeFlag;
	backRightTape |= counting << 1;
	backRightTape |= laserCd << 2;
	backRightTape |= activateHitFlag << 3;
	
	dataValues[TAPE_SENSOR_BACK_RIGHT] = backRightTape;
	dataValues[IR_SENSOR_BACK] = IRBack;
	
	frontLeftTape = dataValues[TAPE_VALUES] & 0x01;
	frontRightTape = dataValues[TAPE_VALUES] & 0x08;
	backLeftTape = dataValues[TAPE_VALUES] & 0x02;
	//backRightTape = dataValues[TAPE_VALUES] & 0x04;

	distanceValue = dataValues[DISTANCE_SENSOR];

	IRLeft = dataValues[IR_SENSOR_LEFT];
	IRRight = dataValues[IR_SENSOR_RIGHT];
	IRFront = dataValues[IR_SENSOR_FRONT];
	//IRBack = dataValues[IR_SENSOR_BACK];
	
	hit = dataValues[HIT_DETECTOR];
	dataValues[LIFE] = lifeCount;
	//sei();
	
	/*frontLeftTape = frontTapeValues & 0x01;
	frontRightTape = frontTapeValues & 0x08;*/
}

//Controls the different sensor values and calls functions accordingly 
void idle(){
	moveRobot(DEACTIVATE_LASER);
	
	if (distanceValue <= 20)
	{
		leftOrRight = 1;
		backing = 1;
		backnTurn = 1;
		timerValue = TIMER_1A_SECOND/2;
		timer_init();
	}
	
	else if(frontLeftTape != 0){
		leftOrRight = 1;
		backing = 1;
		backnTurn = 1;
		timerValue = TIMER_1A_SECOND/2;
		timer_init();
	}
	else if(frontRightTape != 0){
		leftOrRight = 0;
		backing = 1;
		backnTurn = 1;
		timerValue = TIMER_1A_SECOND/2;
		timer_init();
	}
	
	
	else if (IRFront != 2 && IRFront < 8){
		if(!laserCd){
			moveRobot(ACTIVATE_LASER);
			sprayPray = 1;
			turning = 1;
			timerValue = TIMER_1A_SECOND/4;
			timer_init();
		}
		else{
			forward = 1;
		}
	}
	
	else if (!IRFound){
		
		if(IRLeft != 2 && IRLeft < 8){
			//moveRobot(ACTIVATE_LASER);
			leftOrRight = 0;
			turning = 1;
			IRFound = 1;
			timerValue = TIMER_1A_SECOND*0.8;
			timer_init();
			//turn(TIMER_1A_SECOND);
		}
	
		else if(IRRight != 2 && IRRight < 8){
			//moveRobot(ACTIVATE_LASER);
			leftOrRight = 1;
			turning = 1;
			IRFound = 1;
			timerValue = TIMER_1A_SECOND*0.8;
			timer_init();
			//turn(TIMER_1A_SECOND);
		}
	}
	
	/*else if ((frontTapeValues == 0x00)){
		moveRobot(MOVE_FORWARD_FAST);
	}*/
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------

void sensorControlDead(){
	
	if (distanceValue <= 20)
	{
		TapeFlag = 0;
		counting = 0;
		leftOrRight = 1;
		backing = 1;
		backnTurn = 1;
		timerValue = TIMER_1A_SECOND/2;
		timer_init();
	}
	else if(frontLeftTape != 0){		
		TapeFlag |= _BV(0);
	}
	else if(frontRightTape != 0){
		TapeFlag |= _BV(1);
	}
}

int main(void)
{
	SPI_MasterInit();
	btInit();
	interruptINT1_init();
	sei();
	btTransmit(0);

    while(1)
    {
		getSensorValues();
		setVariables();	
		BT_SensorValues();
		
		//Start of AI program that should keep the robot within the boundaries of the tape track
		//moveRobot(LED | lifeCount);
		
		if(!lifeCount){
			sensorControlDead();
			if(TapeFlag >= 0x03 && counting == 0){
				counting = 1;
				timerValue = TIMER_1A_SECOND;
				timer_init();
			}else if(counting == 2){
				while(1){
					moveRobot(STOP);
				}
			}
			
			
		}else{
				
			if(hit == 0){
				activateHitFlag = 0;
				hitFlag = 1;
			}
			
			if (hit == 1 && activateHitFlag)
			{
				moveRobot(IR_ON);
				moveRobot(ACTIVATE_HIT);
			}
			
			else if(hit == 1 && hitFlag == 1){
				hitFlag = 0;
				lifeCount = lifeCount >> 1;
				timer0_init();
			}
			
			else if (hit == 1 && !activateHitFlag){
				moveRobot(IR_OFF);
			}
			
			
			if(!sprayPray || !backing){
				idle();
			}
		}
			
		if(turning){
			if(leftOrRight == 0){
				moveRobot(TURN_LEFT);
			}
			else if(leftOrRight == 1){
				moveRobot(TURN_RIGHT);
			}
		}
		else if(backing){
			moveRobot(MOVE_BACK);
		}
		else if (forward){
			moveRobot(MOVE_FORWARD_FAST);
			//forward = 0;
		}
		
	}
}
