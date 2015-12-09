/*
 * Sensorenhet.c
 *
 * Created: 11/13/2015 10:35:06 AM
 *  Author: henpe071
 */ 


#define F_CPU 16000000UL
#define PA1HIGH   ((PINA & (1<<PA1)))
#define ADCONVERTERFREE !(ADCSRA & (1<<ADSC))
#define IRHIGH (PINA & (1<<PA3))
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

volatile uint16_t distance = 0; // in cm
volatile uint16_t newDistance = 0; // in cm
volatile uint8_t distanceSensorWait = 0;
volatile uint8_t IRSTOPFLAG = 1;
volatile uint8_t IRSENSORWAIT = 0;
uint8_t IRsignals[4];
uint8_t analogTapeValues[4];
uint8_t TapeValues;

//IR grejs
volatile uint8_t IRdata = 0;
volatile uint8_t IRcnt = 0;
volatile uint8_t IRstate = 0;
volatile uint8_t IRnum = 0;
volatile uint8_t IRLoopFlag = 0;

//SPI-constants
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

uint8_t hit = 0;

//AD omvandling
volatile uint8_t requestedTapeSens = 0;
const uint8_t TAPE_NUMS[4] = {6,7,5,4};


ISR(INT0_vect){	//initierar räkning av avstånd med hjälp av timers		
	newDistance = 0;		// set dist to 0
	TCCR2 |= _BV(WGM21) | _BV(CS21) | _BV(CS20);	//CTC, 32prescaler
	OCR2 = 29;				// http://eleccelerator.com/avr-timer-calculator/ 0.0000058s
}

ISR(TIMER2_COMP_vect){
	if(PA1HIGH && !distanceSensorWait && (newDistance < 255)) newDistance++;	//Så länge echo är hög och vi inte väntar öka avstånd, alla värden över 255 ignoreras
	else if(!distanceSensorWait) {	//om echo inte hög och vi inte väntar, måste vi starta väntetid(10ms)
		distance = newDistance;	
		distanceSensorWait = 1;
		TCCR2 = 0;
		TCCR2 = _BV(WGM21) | _BV(CS22) | _BV(CS21) | _BV(CS20); //CTC, 1024prescaler
		OCR2 = 157;		//10ms  http://eleccelerator.com/avr-timer-calculator/	
	}
	else if(distanceSensorWait){	//när vi väntat färdigt kan vi använda avståndssensorn igen
		distanceSensorWait = 0;
		TCCR2 = 0;
	}
}
/*
ISR(TIMER1_COMPA_vect){
	IRSTOPFLAG = 1;
}
*/
void enableInterrupts(){
	GICR = 1<<INT0;					// Enable INT0
	MCUCR = 1<<ISC01 | 1<<ISC00;	// Trigger INT0 on rising edge	
	TIMSK |= _BV(OCIE1A) | _BV(OCIE2);//enable timer interrupts
}
void initPorts(){
	DDRA |= _BV(PA2);									//PA2 triggersignal dist.
	DDRB = 0xFF;	//test
}



void initADConverter(){
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescalar to 128 - 125KHz sample rate @ 16MHz

	ADMUX |= (1 << REFS0); // Set ADC reference to AVCC
	ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading

	ADMUX |= _BV(MUX0) | _BV(MUX1) | _BV(MUX2); //Set to ADC7 (0111)
	
	ADCSRA |= (0 << ADATE);  // Set ADC to single Mode
	ADCSRA |= (1 << ADEN);  // Enable ADC
	ADCSRA |= (1 << ADSC);  // Start A2D Conversions
	ADCSRA |= (1 << ADIE);	// AD interrupt	
}

void SPI_init(){
	DDRB |= _BV(PB6); //MISO-pin configured as output
	DDRB |= _BV(PB3); //Pin that sends a signal to interrupt the targeting module when a sensor data should be sent
	
	SPCR |= _BV(SPE) | _BV(SPIE); //Set as slave, SPI-enable set and interrupts enabled
}

//Triggersignal för att starta avståndssensorn
void triggerSignal(){
	PORTA &= ~_BV(PA2);
	_delay_us(100);
	PORTA |= _BV(PA2);	//Begin the trigger signal
	_delay_us(20);		//Wait for 20 us
	PORTA &= ~_BV(PA2);	//End the signal
}

//Startar adomvandling av tejpsensor nr ch
void adc_read(uint8_t ch)
{
	// select the corresponding channel 0~7
	// ANDing with ’7? will always keep the value
	// of ‘ch’ between 0 and 7
	ch &= 0b00000111;  // AND operation with 7
	ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing
	
	// start single convertion
	// write ’1? to ADSC
	ADCSRA |= (1<<ADSC);
}

ISR(ADC_vect){//när adomvandling färdig
	analogTapeValues[requestedTapeSens] = ADCH;	//spara värdet
	if(requestedTapeSens < 3) requestedTapeSens++;	//och säg att vi vill kolla nästa sensor eller börja om
	else requestedTapeSens = 0; 
}
//--------------------------------------------
//		IR-sensor
//--------------------------------------------

void initNextIRSensors(){

	TCCR0 |= _BV(CS02);	//prescaler 256
	
	TCCR1B = 0;	//init timer1
	TCCR1B |= _BV(CS10) | _BV(CS11) | _BV(WGM12);//CTC, 64 prescaler
	
	IRSTOPFLAG = 0;
	PORTB &= ~_BV(PB2);	//enable mux x
	
	if(IRnum >= 3) IRnum = 0;
	else IRnum++;
	
	PORTB &= 0xFC;
	PORTB |= IRnum; //set portb to 000000xx, where xx is IRnum
	
		
	TCNT0 = 0;
	GICR |= _BV(INT1); //enable INT1
}

ISR(TIMER1_COMPA_vect){
	if(!IRSENSORWAIT){
		IRSTOPFLAG = 1;
		IRSENSORWAIT = 1;
		OCR1A = 25000;		//vänta i 0.1s innan vi läser igen
	}
	else if(IRSENSORWAIT){
		OCR1A = 4000;
		IRSENSORWAIT = 0;		
	}
}

/*ISR(INT1_vect){
	if(!IRSTOPFLAG){
		if(!IRHIGH){
			TCNT0 = 0;
			IRLoopFlag = 1;			
		}
		else if(IRHIGH && IRLoopFlag){
			if (TCNT0 >= 120 && IRstate == 0)
			{
				//data = 0;
				IRstate = 1;
				IRcnt = 0;
			}
			else if (TCNT0 >= 60 && IRstate == 1)
			{
				IRdata = IRdata << 1;
				IRdata++;
				IRcnt++;
			}
			else if (TCNT0 >= 30 && IRstate == 1)
			{
				IRdata = IRdata << 1;
				IRcnt++;
			}
			if (IRcnt >= 3)
			{
				IRstate = 0;
				IRcnt = 0;
				IRsignals[IRnum] = IRdata & 0x07;
			}
			TCNT0 = 0;
			IRLoopFlag = 0;		//föredetta break
		}
	}
}*/

void initIRSensors(){
	TCCR0 |= _BV(CS02);	//prescaler 256

	OCR1A = 4000;					//64*x/16000000 = 28*10^-3 => x = 7000	
	TCCR1B = 0;
	TCCR1B |= _BV(CS10) | _BV(CS11) | _BV(WGM12);//CTC, 64 prescaler
	
	IRSTOPFLAG = 0;
	TCNT0 = 0;
}

void readIRSensor(uint8_t num){
	initIRSensors();
	IRsignals[num] = 8;
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
	PORTB &= 0xFC;
	PORTB |= IRnum; //set portb to 000000xx, where xx is num
	readIRSensor(IRnum);
	PORTB |= _BV(PB2);	//disable mux x
}

void readLaserDetector(){

}

//Räknar hur många bytes vi har skickat över SPIn
ISR(SPI_STC_vect){ //www.avrfreaks.net/forum/spif-flag-spi-interface
	if (SPDR == IR_SENSOR_LEFT){
		SPDR = IRsignals[0];
	}
	
	else if (SPDR == IR_SENSOR_BACK){
		SPDR = IRsignals[1];
	}
	
	else if (SPDR == IR_SENSOR_FRONT){
		SPDR = IRsignals[2];
	}
	
	else if (SPDR == IR_SENSOR_RIGHT){
		SPDR = IRsignals[3];
	}
	
	else if (SPDR == TAPE_SENSOR_FRONT_LEFT){
		SPDR = analogTapeValues[0];
	}
	
	else if (SPDR == TAPE_SENSOR_BACK_LEFT){
		SPDR = analogTapeValues[1];
	}
	
	else if (SPDR == TAPE_SENSOR_BACK_RIGHT){
		SPDR = analogTapeValues[2];
	}
	
	else if (SPDR == TAPE_SENSOR_FRONT_RIGHT){
		SPDR = analogTapeValues[3];
	}
	else if (SPDR == DISTANCE_SENSOR){
		SPDR = distance;
	}
	else if (SPDR == TAPE_VALUES){
		SPDR = TapeValues;
	}
	else if (SPDR == HIT_DETECTOR){
		SPDR = hit;
	}
}

char SPI_SlaveReceive(void)
{
	/* Wait for reception complete */
	while(!(SPSR & (1<<SPIF)));
	/* Return data register */
	return SPDR;
}

int main(void)
{
	enableInterrupts();
	initADConverter();
	initPorts();
	SPI_init();
	sei();
	DDRD |= _BV(PD4) | _BV(PD5);	
    while(1)
    {
		if(ADCONVERTERFREE){
			if(requestedTapeSens == 0){
				PORTD |= _BV(PD5);
				PORTD &= ~_BV(PD5);
			}
			 adc_read(TAPE_NUMS[requestedTapeSens]);	//om vi inte adomvandlar något, starta nästa
		}
		//readLaserDetector();
		if(!IRSENSORWAIT){
			readIRSensors();
			IRnum++;
			if(IRnum >= 4) IRnum = 0;
			
		}
		if(!(distanceSensorWait || PA1HIGH)) triggerSignal();	//Om vi inte väntar eller echo är hög så kan vi göra en ny trigger-signal
		/*if(IRSTOPFLAG){	//om vi väntar på innan vi kan använda avståndssensorn igen kan vi passa på att kolla ir-signaler
			initNextIRSensors();
		}*/
    }
}