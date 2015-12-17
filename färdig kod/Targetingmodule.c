
/*
 * TargetingModule_SPI_.c
 *
 * Created: 12/3/2015 9:01:29 AM
 *  Author: hamvi791
 */ 


#define F_CPU 16000000UL
#define PA1HIGH   ((PINA & (1<<PA1)))

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
	8,	//Flags1						1
	8,	//IR-sensor 2	fram			2
	8,	//IR-sensor 3	höger			3
	0,	//Tejpsensor 1	fram-vänster	4
	0,	//Flags3						5
	0,	//Flags2						6
	0,	//Tejpsensor 2	fram-höger		7
	100,//Avståndssensor				8
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
const uint8_t FLAGS1 = 1;
const uint8_t IR_SENSOR_FRONT = 2;
const uint8_t IR_SENSOR_RIGHT = 3;

const uint8_t TAPE_SENSOR_FRONT_LEFT = 4;
const uint8_t FLAGS3 = 5;
const uint8_t FLAGS2 = 6;
const uint8_t TAPE_SENSOR_FRONT_RIGHT = 7;

const uint8_t DISTANCE_SENSOR = 8;
const uint8_t HIT_DETECTOR = 9;
const uint8_t TAPE_VALUES =  10;
const uint8_t LIFE =  11; 
//----------------------------------------------------------------------------------------

//--------Sensor-control "booleans" to determine behavior the of robot---------------------
uint8_t distanceValue = 0;						//Innehåller värdet som avståndsensorn har registrerat

uint8_t IRLeft = 8;								//Innehåller värdet som IR-sensor på den vänstra sidan har registrerat
uint8_t IRRight = 8;							//Innehåller värdet som IR-sensor på den högra sidan har registrerat
uint8_t IRFront = 8;							//Innehåller värdet som IR-sensor på framsidan har registrerat
uint8_t flags1 = 8;

uint8_t frontLeftTape = 0;						//1 när den vänstra främre tape sensorn har registrerat tejp
uint8_t frontRightTape = 0;						//1 när den högre främre tape sensorn har registrerat tejp
uint8_t flags3 = 0;
uint8_t flags2 = 0;
uint8_t leftOrRight = 0;						//If 0 turn left, 1 turn right
uint8_t hit = 0;								//1 när roboten har registrerat träff från träffsensorn

volatile uint8_t forward = 1;					//1 när roboten ska åka framåt
volatile uint8_t turning = 0;					//1 när roboten ska svänga åt något håll
volatile uint8_t backing = 0;					//1 när roboten ska backa
volatile uint8_t backnTurn = 0;					//1 när roboten ska utföra en så kallad backnTurn, då roboten ska backa och sedan svänga
volatile uint8_t IRFound = 0;					//Flagga som är 1 då en IR-signal har hittats på antigen vänster- eller högersida
volatile uint8_t sprayPray = 0;					//Flagga som är 1 då roboten ska utföra en skjutning av lasern
volatile uint8_t sprayState = 0;				//0 om roboten är i den första svängen i sin sprayPray, 1 om den är i den andra
volatile uint8_t hitFlag = 1;					//Flagga som är 1 när vi kan gå in i if-satsen som hanterar hitdetektion
volatile uint8_t TapeFlag = 0;					//När roboten är död används denna för att hålla koll på om roboten har passerat tejp	
volatile uint8_t deathState = 0;				//Används för att avgöra hur roboten ska bete sig när den är död
volatile uint8_t laserCd = 0;					//1 if laser is on cooldown, 0 if it isn't

volatile uint8_t activateHitFlag = 0;			//1 om vi ska aktivera IR-sändaren och träffdetektionen, alltå sluta vara osynliga 
volatile uint16_t timer0_5sec = 0;				//Används för att räkna upp till 5 sekunder i timer 0
volatile uint16_t timer2_3sec = 0;				//Används för att räkna upp till 3 sekunder i timer 2

volatile uint8_t lifeCount = 0x07;				//Värdet för att representera hur lysdioderna ska lysa, men också hur mycket liv roboten har

uint16_t timer1A_Value = 0;						//Värdet som compare register OC1A kommer att tilldelas då timer1A_init kallas
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

//----------------------------------BT----------------------------------
//----------------------------------------------------------------------
void btInit(void){
	/* Set baud rate */
	UBRRH = 0x0;
	UBRRL = 0x08;		//115200 http://wormfood.net/avrbaudcalc.php
	/* Enable receiver and transmitter */
	UCSRB = _BV(RXEN) | _BV(TXEN) | _BV(RXCIE);
	/* Set frame format: 8data, 1stop bit */
	UCSRC = (1 << URSEL) | (0 << UMSEL) | (0 << UPM1) | (0 << UPM0) | (0 << USBS) | (1 << UCSZ1) | (1 << UCSZ0) | (0 << UCPOL)  ;
}

/* Send one byte as soon as transmit buffer is empty.*/
void btTransmit(unsigned char data){
	/* Wait for empty transmit buffer */
	while ( !( UCSRA & _BV(UDRE) ));
	/* Put data into buffer, sends the data */
	UDR = data;
}

unsigned char btReceive(){
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

//Avbrott som hanterar datorns förfrågningar efter data från BT
ISR(USART_RXC_vect){
	dataAddress = UDR;	//vilken sensor vill datorn veta om?
	requestFlag = 1;// sätt flagga att skicka saker
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------

//---------------------------------------SPI---------------------------
//---------------------------------------------------------------------

//Initierar värdena på kontroll- och datariktningsregister så att denna enhet får önskade SPI-inställningar
void SPI_MasterInit(void){
	/* Set MOSI and SCK output, all others input */
	DDRB = (1<<PB5)|(1<<PB7);

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

//Startar en SPI transmission, alltså skickar data och returnerar slavens skickade data
unsigned char SPI_MasterTransmit(char cData){
	/* Start transmission */
	SPDR = cData;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));
	return SPDR;
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------

//---------------------------------------Commands----------------------
//---------------------------------------------------------------------

//Use this function to command the robot using one of the pre-defined commands
void commandRobot(uint8_t move){	
	if(calibrating >= 3){ //Skickar inte förrän kalibreringen är färdig
		PORTB &= ~_BV(PB1); //Slave select
		SPI_MasterTransmit(move);
		PORTB |= _BV(PB1);
	}
	dataValues[13] = move; //För debug på PC
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------

//--------------------------------Timers-------------------------------------
//---------------------------------------------------------------------------
//Initierar osynlighetstimern
void timer0_init(){
	TCNT0 = 0;
	TCCR0 |= _BV(WGM01) | _BV(CS00) | _BV(CS02); //Set CTC with prescaling 1024
	TIMSK |= _BV(OCIE0); //Enable interrupt on compare match, compare register 0
	OCR0 = 250; //Roughly 16 ms, calculated with prescaling of 1024 using the following formula: 0,016 = (1024*x)/(16*10^6)	
}

//Avbrott som används för att räkna upp till 5 sekunder för att sedan signalera att roboten ska sluta vara osynlig
ISR(TIMER0_COMP_vect){
	timer0_5sec++; //timern räknar upp 300 gånger, ca 5 sek, 
	if(timer0_5sec >= 300){
		timer0_5sec = 0;
		activateHitFlag = 1; //sätt flagga för att sluta vara osynlig
		TCNT0 = 0;
		TCCR0 = 0;
		TIMSK &= ~_BV(OCIE0); //Stäng av timern
	}
}

//Laser cooldown
void timer2_init(){
	TCNT2 = 0;
	TCCR2 |= _BV(WGM21) | _BV(CS20) | _BV(CS21) | _BV(CS22); //Set CTC with prescaling 1024
	TIMSK |= _BV(OCIE2); //Enable interrupt on compare match, compare register 0
	OCR2 = 250; //Roughly 16 ms, calculated with prescaling of 1024 using the following formula: 0,016 = (1024*x)/(16*10^6)
	
}

//Avbrott som används för att räkna upp till 3 sekunder, där den då signalerar att roboten kan skjuta lasern igen
ISR(TIMER2_COMP_vect){
	timer2_3sec++;
	if(timer2_3sec >= 188){
		timer2_3sec = 0;
		laserCd = 0;
		TCCR2 = 0;
		TIMSK &= ~_BV(OCIE2);
	}
}

//Timer för att veta hur länge vi ska svänga eller backa
void timer1A_init(){
	TCNT1 = 0;
	TCCR1B |= _BV(WGM12) | _BV(CS10) | _BV(CS12); //Set CTC with prescaling 1024
	TIMSK |= _BV(OCIE1A); //Enable interrupt on compare match, compare register 1A
	OCR1A = timer1A_Value; //Roughly 1s, calculated with prescaling of 1024 using the following formula: 1 = (1024*x)/(16*10^6), 5*(16*10^6)/1024 = x 
}

//Avbrott som hanterar flaggor efter en specifik rörelse roboten utfört. Till exempel  
//nollställa flaggor så att roboten återgår till normal läget, då roboten åker framåt, 
//efter en rörelse är färdigt.
ISR(TIMER1_COMPA_vect){

	if(!lifeCount && deathState == 1){			//När roboten har 0 liv kör 1 sek för att sedan dö
		deathState += 1;
	}
	if (backnTurn){								//Om vi ska svänga efter vi har backat
		if (leftOrRight){
			OCR1A = TIMER_1A_SECOND*0.5;	
		}else{
			OCR1A = TIMER_1A_SECOND*0.6;
		}
		
		turning = 1;							//sätt turningflaggan så att vi svänger
		backnTurn = 0;							//Nollställ så att vi inte går in här igen
		backing = 0;							// Nollställ för att vi inte ska fortsätta backa
	}
	else if(sprayPray && !sprayState){			//Om vi gör våran skjutmanöver och inte har gjort båda svängarna 
		leftOrRight = !leftOrRight;				//Byt vilket håll vi svänger åt
		OCR1A = 2*timer1A_Value;					//Räkna upp timern dubbelt så länge
		sprayState = 1;							//ser till av vi gör vår andra sväng
	}
	else{
		if(sprayPray && sprayState){				//Om vi är färdiga med vår skjutmanöver
			laserCd = 1;						//Sätt lasern på cooldown
			timer2_init();						//Starta tiemrn för laser cooldown
		}
		forward = 1;							//Säg att vi ska köra framåt
		turning = 0;							//Nollställ alla andra flaggor
		backing = 0;
		IRFound = 0;
		sprayPray = 0;
		sprayState = 0;
		TCCR1B = 0; 
		TIMSK &= ~_BV(OCIE1A);					//Stäng av timern
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

//Mäter ett värde för golvet och ett för tejpen och sen tar medelvärdet och sätter det som gränsvärde för tejpsensorerna
ISR(INT1_vect){
	if(calibrating == 0){ //Första knapptryckningen
		tapeFL = (dataValues[TAPE_SENSOR_FRONT_LEFT]);
		tapeFR = (dataValues[TAPE_SENSOR_FRONT_RIGHT]);
	}
	else if(calibrating == 1){ //Andra knapptryckningen
		floorFL = (dataValues[TAPE_SENSOR_FRONT_LEFT]);
		floorFR = (dataValues[TAPE_SENSOR_FRONT_RIGHT]);
		
		tapeThresholdFL = ((tapeFL) + (floorFL))/2;
		tapeThresholdFR = ((tapeFR) + (floorFR))/2;
	}
	calibrating += 1;
	//Tredje knapptryckningen startar roboten
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------

//-----------------------------AI functions----------------------------
//---------------------------------------------------------------------
//kontrollerar om tejpsensorvärdet är högre än gränsvärdet och lagrar det i Tapevalues
void tapeThresholdCheck(){
	
	//Front left tape sensor
	if(dataValues[TAPE_SENSOR_FRONT_LEFT] > tapeThresholdFL){
		dataValues[TAPE_VALUES] |= _BV(0);
	}else{
		dataValues[TAPE_VALUES] &= ~_BV(0);
	}
	
	//Front right tape sensor
	if(dataValues[TAPE_SENSOR_FRONT_RIGHT] > tapeThresholdFR){
		dataValues[TAPE_VALUES] |= _BV(3);
	}else{
		dataValues[TAPE_VALUES] &= ~_BV(3);
	}
}

//Gets all the sensor values from sensorenheten via the SPI-bus
void getSensorValues(){
	uint8_t temp = 0;
	for(uint8_t i = 0; i < RETRIEVABLE_SENSOR_DATA; ++i){ //Itererar över alla sensorer som skickar data
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
	tapeThresholdCheck();
}

//Sets the variables being used in the AI program that needs the sensor values
void setVariables(){
	//Sätter flaggor för felsökning
	flags1 |= forward;
	flags1 |= turning << 1;
	flags1 |= backing << 2;
	flags1 |= backnTurn << 3;
	flags1 |= IRFound << 4;
	flags1 |= sprayPray << 5;
	flags1 |= sprayState << 6;
	flags1 |= hitFlag << 7;
	
	flags2 |= TapeFlag;
	flags2 |= deathState << 1;
	flags2 |= laserCd << 2;
	flags2 |= activateHitFlag << 3;
	
	dataValues[FLAGS2] = flags2;
	dataValues[FLAGS1] = flags1;
	
	frontLeftTape = dataValues[TAPE_VALUES] & 0x01;
	frontRightTape = dataValues[TAPE_VALUES] & 0x08;

	distanceValue = dataValues[DISTANCE_SENSOR];

	IRLeft = dataValues[IR_SENSOR_LEFT];
	IRRight = dataValues[IR_SENSOR_RIGHT];
	IRFront = dataValues[IR_SENSOR_FRONT];
	
	hit = dataValues[HIT_DETECTOR];
	dataValues[LIFE] = lifeCount;
}

//Controls the different sensor values and sets flags accordingly 
void sensorControl(){
	commandRobot(DEACTIVATE_LASER);
	
	if (distanceValue <= 20){ //Kontrollerar så att vi inte kör på något
		if (PA1HIGH){ //Om vi är i testläge
			leftOrRight = 0; //Sätt åt vilket håll vi svänger
		}else{
			leftOrRight = 1;
		}
		backing = 1;	//Sätt att vi ska backa
		backnTurn = 1;	//Sätt att vi ska backa och svänga
		timer1A_Value = TIMER_1A_SECOND/2; //Sätt hur länge timern ska köras så att vi svänger/backar lagom länge
		timer1A_init();	//Starta timern
	}
	
	else if(frontLeftTape != 0){	//Om vi ser tejp på den vänstra tejpsensorn
		leftOrRight = 1;
		backing = 1;
		backnTurn = 1;
		timer1A_Value = TIMER_1A_SECOND/2;
		timer1A_init();
	}
	else if(frontRightTape != 0){	//Om vi ser tejp på den högra tejpsensorn
		leftOrRight = 0;
		backing = 1;
		backnTurn = 1;
		timer1A_Value = TIMER_1A_SECOND/2;
		timer1A_init();
	}
	
	else if (IRFront != 2 && IRFront < 8){	//Om vi ser en godkänd IR-signal som inte är våran egen framför
		if(!laserCd){	//Om vi inte vänta på att få skjuta
			commandRobot(ACTIVATE_LASER);	//SKJut!!
			sprayPray = 1;	//Sätt att vi gör vår skjutmanöver
			turning = 1;
			timer1A_Value = TIMER_1A_SECOND/4;
			timer1A_init();
		}
		else{ //Om vi inte får skjuta, kör framåt
			forward = 1;
		}
	}
	
	else if (!IRFound && !backing && !turning){		//Om vi inte utför någon annan manöver
		if(IRLeft != 2 && IRLeft < 8){	//Om vi ser en godkänd IR-signal och det inte är vår egen på vänster sida
			leftOrRight = 0;
			turning = 1;
			IRFound = 1;	//Sätt att vi har uppfattat en IR-signal på sidan
			timer1A_Value = TIMER_1A_SECOND*0.8;
			timer1A_init();
		}
	
		else if(IRRight != 2 && IRRight < 8){	//Om vi ser en godkänd IR-signal och det inte är vår egen på vänster sida
			leftOrRight = 1;
			turning = 1;
			IRFound = 1;
			timer1A_Value = TIMER_1A_SECOND*0.8;
			timer1A_init();
		}
	}
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------

void sensorControlDead(){ //Denna 
	
	if (distanceValue <= 20)
	{
		TapeFlag = 0;
		deathState = 0;
		leftOrRight = 1;
		backing = 1;
		backnTurn = 1;
		timer1A_Value = TIMER_1A_SECOND/2;
		timer1A_init();
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
		commandRobot(LED | lifeCount); // Tänder livdioder för hur många liv vi har
		
		if(!lifeCount){
			sensorControlDead();
			if(TapeFlag >= 0x03 && deathState == 0){
				deathState = 1;
				timer1A_Value = TIMER_1A_SECOND;
				timer1A_init();
			}else if(deathState == 2){
				while(1){
					commandRobot(STOP);
				}
			}
			
		}else{
			if(hit == 0 && activateHitFlag == 1){
				activateHitFlag = 0;
				hitFlag = 1;
			}
			
			if (hit == 1 && activateHitFlag == 1)
			{
				commandRobot(IR_ON);
				commandRobot(ACTIVATE_HIT);
			}
			
			else if(hit == 1 && hitFlag == 1){
				hitFlag = 0;
				if (lifeCount == 7){
					lifeCount = 3;
				}
				else if (lifeCount == 3){
					lifeCount = 1;
				}
				else if (lifeCount == 1){
					lifeCount = 0;
				}
				timer0_init();
			}
			
			else if (hit == 1 && !activateHitFlag){
				commandRobot(IR_OFF);
			}
			
			
			if(!sprayPray && !backing){
				sensorControl();
			}
		}
		
		if(backing){
			commandRobot(MOVE_BACK);
		}
		else if(turning){
			if(leftOrRight == 0){
				commandRobot(TURN_LEFT);
			}
			else if(leftOrRight == 1){
				commandRobot(TURN_RIGHT);
			}
		}
		else if (forward){
			if (PA1HIGH){
				commandRobot(MOVE_FORWARD_SLOW);
			}
			else{
				commandRobot(MOVE_FORWARD_FAST);
			}
		}
	}
}
