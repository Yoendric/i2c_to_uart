/*
 * I2C_UART_convertion.c
 *
 * Created: 7/28/2020 7:39:37 PM
 * Author : yoendric2
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <string.h>
#define TRUE          1
#define FALSE         0

#define LED_ON PORTB |= (1<<PORTB5)
#define LED_OFF PORTB &= ~(1<<PORTB5)
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#define UART_TIMECOUNT 100000L //CPU Cycles 
#define UART_BUFFER_SIZE 25
#define TWI_BUFFER_SIZE 8
#define TWI_ADR_BITS  1   // Bit position for LSB of the slave address bits in the init byte.
#define TWI_GEN_BIT   0   // Bit position for LSB of the general call bit in the init byte.
unsigned char messageBuf[UART_BUFFER_SIZE];
unsigned char twiBuf[TWI_BUFFER_SIZE];
uint8_t twi_reg_addr = 0;
uint8_t twi_write_complete = FALSE;
int data_count=0;
volatile uint8_t  Address = 20;

void TWI_Init(uint8_t addr) {
	/* Clear interrupts */
	/* set TWI/I2C address */
	TWAR0 = ((addr<<TWI_ADR_BITS) | ((TRUE<<TWI_GEN_BIT)));
	TWCR0 = (1<<TWEN)|(0<<TWIE)|(0<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);
	
	_delay_ms(10);
	sei();
	/* Enable and clear TWI interrupt, enable address matching */
	TWCR0 = (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWEN)| (0<<TWSTA)| (0<<TWSTO)|(0<<TWWC);
}

void TWI_stop() {
	/* clear ACK and enable bits */
	TWCR0 &= ~((1<<TWEA) | (1<<TWEN));
}

void USART_Init( unsigned int ubrr)
{	
	UCSR0A = 0x00;
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)| (1 << RXCIE0); ;
	UCSR0C =  (3<<UCSZ00);
	sei();
}

void USART_Transmit( unsigned char data)
{
	uint16_t timeout=0;
	while(!(UCSR0A & (1 << UDRE0)) && ++timeout<UART_TIMECOUNT);
	UDR0 = data;
}

void command_USART(unsigned char command[])
{
	uint8_t i=0;
	while (i < TWI_BUFFER_SIZE)
	{
		USART_Transmit(command[i]);
		i++;
	}
}

void click_LED(){
	LED_ON;
	_delay_ms(100);
	LED_OFF;
	_delay_ms(100);
}

void init_GPIO()
{
	DDRB |= 1<<DDRB5;
	DDRB &= ~(1 << DDRB0);
	DDRD &= ~((1 << DDRD5) & (1 << DDRD6) & (1 << DDRD7));
}

void get_Address()
{
	if((PIND&(1<<PIND5))==0){
		Address +=1;
		}if((PIND&(1<<PIND6))==0){
		Address +=2;
		}if((PIND&(1<<PIND7))==0){
		Address +=4;
		}if((PINB&(1<<PINB0))==0){
		Address +=8;
	}
}

int main(void)
{
    /* Replace with your application code */
	init_GPIO();
	DDRB |= (1<<DDB5);
	get_Address();
	USART_Init(MYUBRR);
	TWI_Init(Address);
    while (1) 
    {
		if (twi_write_complete == FALSE){
			//click_LED();
			asm("nop");
		} else{
			twi_write_complete = FALSE;
			command_USART(twiBuf);
			memset(twiBuf, 0, sizeof(twiBuf));
		}
    }
}

ISR(USART0_RX_vect){
	messageBuf[data_count] = UDR0;
	data_count ++;
	if (data_count == UART_BUFFER_SIZE){
		data_count = 0;
	}
}

ISR(TWI0_vect)
{
	switch (TWSR0)
	{
		case TW_ST_SLA_ACK:            // Own SLA+R has been received; ACK has been returned
             			twi_reg_addr = 0;
             			TWDR0 = messageBuf[twi_reg_addr];
             			messageBuf[twi_reg_addr]=0;
             			twi_reg_addr++;
             			TWCR0 = (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
             			break;
		case TW_ST_DATA_ACK:           // Data byte in TWDR0 has been transmitted; ACK has been received
		     			TWDR0 = messageBuf[twi_reg_addr];
		     			messageBuf[twi_reg_addr]=0;
		     			twi_reg_addr++;
		     			if (twi_reg_addr < UART_BUFFER_SIZE) {
			     					TWCR0 = (1<<TWEN)|                                 // TWI Interface enabled
			     					(1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interrupt and clear the flag to send byte
			     					(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           //
			     					(0<<TWWC);
			     			} else {
			     					TWCR0 = (1<<TWEN)|                                 // TWI Interface enabled
			     					(1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interrupt and clear the flag to send byte
			     					(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           //
			     					(0<<TWWC);
		     			}
		     			break;
		case TW_ST_DATA_NACK:          // Data byte in TWDR0 has been transmitted; NACK has been received.
		          TWCR0 = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins
		          (1<<TWIE)|(1<<TWINT)|                      // Keep interrupt enabled and clear the flag
		          (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Answer on next address match
		          (0<<TWWC);                                 //
		          break;
		case TW_SR_GCALL_ACK:            // General call address has been received; ACK has been returned
		case TW_SR_SLA_ACK:            // Own SLA+W has been received ACK has been returned
		          twi_reg_addr = 0;                             // Set buffer pointer to first data location
		          TWCR0 = (1<<TWEN)|                                 // TWI Interface enabled
		          (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interrupt and clear the flag to send byte
		          (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Expect ACK on this transmission
		          (0<<TWWC);		
		          break;
		case TW_SR_DATA_ACK:       // Previously addressed with own SLA+W; data has been received; ACK has been returned
		case TW_SR_GCALL_DATA_ACK:       // Previously addressed with general call; data has been received; ACK has been returned
                  if (twi_reg_addr >= TWI_BUFFER_SIZE) {
				         twi_reg_addr = 0x00;   
				  }else{
					     twiBuf[twi_reg_addr] = TWDR0;
					     twi_reg_addr++;
				  }
		          TWCR0 = (1<<TWEN)|                                 // TWI Interface enabled
		          (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interrupt and clear the flag to send byte
		          (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Send ACK after next reception
		          (0<<TWWC);                                 //
		          break;
		case TW_SR_STOP:       // A STOP condition or repeated START condition has been received while still addressed as Slave
		          TWCR0 = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins
		          (1<<TWIE)|(1<<TWINT)|                      // Enable interrupt and clear the flag
		          (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Wait for new address match
		          (0<<TWWC);
				  twi_reg_addr = 0;
				  twi_write_complete = TRUE;                                 //
           		  break;
		case TW_SR_DATA_NACK:      // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
		case TW_SR_GCALL_DATA_NACK:      // Previously addressed with general call; data has been received; NOT ACK has been returned
		case TW_ST_LAST_DATA: // Last data byte in TWDR0 has been transmitted (TWEA = “0”); ACK has been received
		case TW_BUS_ERROR:         // Bus error due to an illegal START or STOP condition
		          TWCR0 =   (1<<TWSTO)|(1<<TWINT);   //Recover from TWI_BUS_ERROR, this will release the SDA and SCL pins thus enabling other devices to use the bus
		          break;
		default:
		          TWCR0 = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins
		          (1<<TWIE)|(1<<TWINT)|                      // Keep interrupt enabled and clear the flag
		          (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Acknowledge on any new requests.
		          (0<<TWWC);                                 //
	}
}

