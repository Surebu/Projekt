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
//uint8_t TapeValues[4];
uint8_t TapeValues;

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
}

ISR(TIMER1_COMPA_vect){
	IRSTOPFLAG = 1;
}
void enableInterrupts(){
	GICR = 1<<INT0;					// Enable INT0
	MCUCR = 1<<ISC01 | 1<<ISC00;	// Trigger INT0 on rising edge	
	TIMSK |= _BV(OCIE1A);			//enable timer interrupts
	OCR1A = 7000;					//64*x/16000000 = 28*10^-3 => x = 7000
	sei();
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

void triggerSignal(){
	PORTA &= ~_BV(PA2);
	_delay_us(100);
	PORTA |= _BV(PA2);
	_delay_us(20);
	PORTA &= ~_BV(PA2);
}

void readTapeSensor(){
	for (uint8_t tapeNum = 0; tapeNum>4; tapeNum++)
	{
		if (tapeNum == 2)
		{
			ADMUX |= _BV(MUX2) | _BV(MUX1) | ~_BV(MUX0) ; //Set to ADC6 (0110)
			ADCSRA |= (1 << ADSC);  // Start A2D Conversions	
			_delay_us(200);
			ADConvert(tapeNum);
		}
		else if (tapeNum == 3)
		{
			ADMUX |= _BV(MUX2) | _BV(MUX1) | _BV(MUX0); //Set to ADC7 (0111)
			ADCSRA |= (1 << ADSC);  // Start A2D Conversions
			_delay_us(200);
			ADConvert(tapeNum);
		}
	}	
}

void ADConvert(uint8_t tapeNum){
	/*if (ADCH < tapeThreshold)
	{
		TapeValues |= _BV(tapeNum);
		//TapeValues[num] = 1;
	}
	else{
		TapeValues &= ~_BV(tapeNum);
		//TapeValues[num] = 0;
	}*/
	PORTB = ADCH;
}

void readIRSensor(uint8_t num){//FUNKARINTEFÖRHENKEÄRENSNOPDANKWEEDSHITPOJKESOMGILLARANUSPAJ
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
					
					if (TCNT0 >= 120 && state == 0) //40k, 160
					{
						//data = 0;
						state = 1;
						cnt = 0;
					}
					else if (TCNT0 >= 60 && state == 1) //20k, 80
					{
						data = data << 1;
						data++;
						cnt++;
					}
					else if (TCNT0 >= 30 && state == 1)//10k, 50
					{
						data = data << 1;
						cnt++;
					}
					//PORTB = data;
					if (cnt >= 3)
					{
						state = 0;
						cnt = 0;
						IRsignals[num] = data;
						//data = 0;
					}
					TCNT0 = 0;
					break;
				}
			}
		}
	}
}

void readSensors(uint8_t PBx){
	PORTB &= ~_BV(PBx);	//enable mux x
    for (uint8_t num = 0; num<4; num++)//Loop over the sensors
    {
	    //num = (num & 3)//maska till 000000xx
		PORTB &= 0x0C;
	    PORTB |= num; //set portb = yyyyyyxx
		//_delay_ms(40);
		readIRSensor(num);
		//_delay_ms(10);
		
		//_delay_ms(10);
    }
	PORTB |= _BV(PBx);	//disable mux x
}

void outputValues(){
	PORTB &= 0x0F;
	//values &= TapeValues[0];
	//TapeValues = 0x0f;
	//TapeValues = TapeValues << 4;
	//PORTB = (IRsignals[3]<<4) | (PORTB & 0x0f);//TapeValues;
	
	PORTB |= (TapeValues << 4);
	
	/*for (uint8_t num = 0; num<4; num++)
	{
		PORTB |= IRsignals[num] << 5;
	}*/
}

int main(void)
{
	enableInterrupts();
	initADConverter();
	initPorts();
	//DDRD |= _BV(PD0) | _BV(PD1);
	//PB0 && PB1 = styrsignaler 0=>0, 1=>1
	//PB2 = 0 => enable IR
	
    while(1)
    {
		//readSensors(PB2); //Ska fixa IRsensor
		readTapeSensor();
		//initDistanceSensor();
		//triggerSignal();
		//_delay_ms(20);
		//_delay_ms(10);
		outputValues();
		_delay_ms(10);
		//check avstandssensor
    }
}