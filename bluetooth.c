/*
 * bluetooth.c
 *
 * Created: 11/17/2015 2:04:54 PM
 *  Author: teoti001
 */ 


#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


void btInit(void)
{
	/*16MHz ska ha 115.2k i baud rate och har en felmarginal på -3.5%*/
	/*ej säker på följande 3 rader*/
	/*för att få önskad baudrate så sätter man F_CPU = 4.7456E56 nånting fråga peter*/

	DDRA |= _BV(PA2) | _BV(PA3);
	PORTA &= ~( _BV(PA2) | _BV(PA3) );
	/* Set baud rate */
	
	UBRRH = 0x00;
	UBRRL = 0x08;		//115200 http://wormfood.net/avrbaudcalc.php
	/* Enable receiver and transmitter */
	UCSRB = _BV(RXEN) | _BV(TXEN) | _BV(RXCIE);
	/* Set frame format: 8data, 1stop bit */
	UCSRC = _BV(UCSZ0) | _BV(UCSZ1);	//?!?!??!??!?!?!
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
	unsigned char readData = UDR;
	return readData;
}

ISR(USART_RXC_vect)
{
	uint8_t bt_data = UDR;
	/* Echo back received data */
	btTransmit(bt_data);
}

int main(void)
{
	btInit();
	sei();
	
	char val = 'A'; //0x41;
	PORTB = ~val; // set PORTB
	
	while(1)
	{
		btTransmit(val);
		_delay_ms(250);
	}
};

