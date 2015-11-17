#include <avr/io.h>
#include <bluetooth.h>

void btInit(void)
{
	/*16MHz ska ha 115.2k i baud rate och har en felmarginal på -3.5%*/
	/*ej säker på följande 3 rader*/
	unsigned int baud=7;
	DDRA |= (1<<PORTA2)|(1<<PORTA3);
	PORTA &= ~((1<<PORTA2)|(1<<PORTA3));
	/* Set baud rate */
	UBRR0H = (unsigned char)(baud>>8);
	UBRR0L = (unsigned char)baud;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = (0<<USBS0)|(3<<UCSZ00);
}

/* Send one byte as soon as transmit buffer is empty.*/
void btTransmit(unsigned char data)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

unsigned char btReceive()
{
	/* Wait for data to be received */
	while (!(UCSR0A & (1<<RXC0)));
	/* Get and return received data from buffer */
	unsigned char readData = UDR0;
	return readData;
}

ISR(USART0_RX_vect)
{
	bt_data = UDR0;
	/* Echo back received data */
	btTransmit(bt_data);
}

int main(void)
{
	btInit();
	sei();
	
	value = 'A'; //0x41;
	PORTB = ~value; // set PORTB
	
	while(1)
	{
		btTransmit(value);
		_delay_ms(250);
	}
};