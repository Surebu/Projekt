

/*
 * Styrenhetens hela program!
 *
 * 
 *  
 */ 

#define F_CPU 16000000UL //OBS 16000000UL OM den externa klockan används

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

const uint8_t STOP = 0;
const uint8_t SLOW = 64;
const uint8_t FAST = 128;

//Variables for compare-interrupts in 8-bit timer 2, values derived from time(s)/(1/(clk/prescaling))
const uint8_t STARTBIT = 37; //2.4 ms
const uint8_t LOGICONE = 19; //1.2 ms
const uint8_t LOGICZERO = 9; //0.6 ms
volatile uint8_t IRState = 0; //0 = "startbit", 1 = "pause"...........

//Intstruction byte for commands from målsökningsenhet
uint8_t command = 0;
uint8_t cmdH = 0; //The highest bits of the command from målsökning
uint8_t cmdL = 0; //The lowest bits....

ISR(TIMER2_COMP_vect){
	
	PORTC |= _BV(PC0);
	//Code = 001
	switch(IRState){
		case 0://start bit
			OCR2 = STARTBIT; //Set time to next interrupt
			TCCR0 |= (1<<WGM01)|(1<<COM00)|(1<<WGM01)|(1<<CS00); //Register settings for an alternating signal of 38 KHz
			break;
		case 1://pause
			TCCR0 = 0; //Normal port function
			PORTB &= ~_BV(PB3); //Force output zero
			OCR2 = LOGICZERO; //Set time to next interrupt
			break;
		case 2://0
			OCR2 = LOGICZERO;
			TCCR0 |= (1<<WGM01)|(1<<COM00)|(1<<WGM01)|(1<<CS00);
			break;
		case 3://pause
			TCCR0 = 0; 
			PORTB &= ~_BV(PB3); 
			OCR2 = LOGICZERO;
			break;
		case 4://0
			OCR2 = LOGICZERO;
			TCCR0 |= (1<<WGM01)|(1<<COM00)|(1<<WGM01)|(1<<CS00);
			break;
		case 5://pause
			TCCR0 = 0;
			PORTB &= ~_BV(PB3); 
			OCR2 = LOGICZERO;
			break;		
		case 6://1		
			OCR2 = LOGICONE;
			TCCR0 |= (1<<WGM01)|(1<<COM00)|(1<<WGM01)|(1<<CS00);
			break;
		case 7://pause
			TCCR0 = 0; 
			PORTB &= ~_BV(PB3); 
			OCR2 = LOGICZERO;
			break;
	}
	
	if(IRState >= 7) IRState = 0;
	else IRState++;
		
	//Restart counters
	TCNT0 = 0;
	TCNT2 = 0;
}

void timer_init(){
	
	//Timer for generating 38Khz alternating signal
	//---------------------
	//set CTC mode with prescaler value of 0 and toggle OC0 on compare match when counting up
	TCCR0 |= (1<<WGM01)|(1<<COM00)|(1<<WGM01)|(1<<CS00);
	
	//This will give us the frequency of 38Khz by (16*10^6)/2*(38*10^3) or clk/2*desired_frequency
	OCR0 = 210;
	//----------------------
	
	//Timer for counting ms
	//-----------------------
	//set normal mode with prescaler value of 1024, with 16MHz clk the 8-bit register can count up to 16 ms
	TCCR2 |= _BV(CS20) | _BV(CS21) | _BV(CS22);
	//------------------------
		
	//Motor Timer
	//--------------
	//Set Fast PWM mode with prescaler value of 64 and clear OC1A and OC1B on compare match
	TCCR1A |= _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
	TCCR1B |= _BV(WGM12) | _BV(CS10) | _BV(CS11);
	//---------------
	
	//Timer interrupt
	//---------------
	TIMSK |= _BV(OCIE2); //Enable interrupts in timer 2 on compare match
	OCR2 = STARTBIT;  //Init value for compare register 2
}

void DDR_init(){
	DDRD |= ( _BV(PD2) | _BV(PD3) | _BV(PD4) | _BV(PD5));
	DDRB |= _BV(PB3); //Utsignal för IR-sändare
	DDRA |= _BV(PA3); //Aktiveringssignal
	PORTA &= ~_BV(PA3);
	DDRA |= _BV(PA0) | _BV(PA1) | _BV(PA2); //Utsignaler för dioder
	DDRD |= _BV(PD6); //Laser pinne
}

//----------------------------------Motors----------------------------------------
//--------------------------------------------------------------------------------
/*
* Sätter hastighet och riktning på motorpar.
* Tar in pekare till OCR för motorparet, vilken pinne som är motorparets direction och ett argument att sätta motorparet till(se DS).
*/
void setMotorPair(volatile uint8_t* OCRx, int PDx, uint8_t arg ){
	if((OCRx == &OCR1A && arg == 1) || (OCRx == &OCR1B &&  arg  != 1)){ // Om motorpar 1 och backa eller motorpar 2 och inte backa 
		PORTD &= ~_BV(PDx);		//sätt dir till 0
	}
	else PORTD |= _BV(PDx);		//annars sätt dir till 1
	switch(arg){
		case 0:
			*OCRx = STOP; 
			break;
		case 1:
			*OCRx = SLOW;
			break;
		case 2:
			*OCRx = SLOW;
			break;
		case 3:
			*OCRx = FAST;
			break;
		default:
			*OCRx = STOP;
			break;
	}	
}

void setMotors(uint8_t arg){
	uint8_t arg1 = (arg &~ 0xF3) >> 2; //maska till 0000xx00 och lsr två steg (000000xx)
	uint8_t arg2 = arg &~ 0xFC; //maska till 000000xx
	setMotorPair(&OCR1A, PD2, arg1);
	setMotorPair(&OCR1B, PD3, arg2);
}
//----------------------------------------------------------------------
//----------------------------------------------------------------------


//---------------------------------Diods---------------------------------
//-----------------------------------------------------------------------
void init_diodPorts(){
	PORTA |= _BV(PA0) | _BV(PA1) | _BV(PA2); //Alla dioder lyser från början
}

void decrement_diods(){
	PORTA >> 1; 
}
//------------------------------------------------------------------------
//------------------------------------------------------------------------

//-----------------------------------SPI---------------------------------
//-----------------------------------------------------------------------

void SPI_init(){
	DDRB |= _BV(PB6); //MISO-pin configured as output
	
	SPCR |= _BV(SPE) | _BV(SPIE); //SPI-enable set and interrupts enabled
}

//Interrupt routine for SPI-transmission
ISR(SPI_STC_vect){ //www.avrfreaks.net/forum/spif-flag-spi-interface
	command = SPDR;
	cmdH = command >> 4;
	cmdL = command & 0x0F;	
}
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------

//--------------------------------Lasorz---------------------------------
//-----------------------------------------------------------------------
void shootLaser(){
	PORTD |= _BV(PD6);
	_delay_ms(500);
	PORTD &= ~_BV(PD6);	
	_delay_ms(500);
}

void activateLaserDetector(){
	PORTA |= _BV(PA3);
	PORTA &= ~_BV(PA3);
}
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------

int main(void)
{
	timer_init();
	DDR_init();
	SPI_init();
	sei(); //Enable interrupts
	//init_diodPorts();
	
	OCR1A = 0; //left
	OCR1B = 0; //right
	
	PORTD |= _BV(PD2); //right, 0 backward and 1 forward
	PORTD |=  _BV(PD3); //left, 1 backward and 0 forward
	
	while(1)
	{
		shootLaser();
		if(cmdH = 1){
			setMotors(cmdL);
		}else if(cmdH = 2){
			activateLaserDetector();
		}
	}
}