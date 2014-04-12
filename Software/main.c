/*
 * DRF4463_base.c
 *
 * Base station, example code for testing DRF4463 module
 * and.elektroda.net for elektroda.pl
 * Created: 29.09.2013
 * Based on dorji (www.dorji.com) example code and output of WDS3
 * Wireless Development Suite (WDS) Modules from www.silabs.com
 *
 */

#include "DRF4463_base.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "DRF4463.h"
#define FIELD_LENGTH 64

//receiving data from USART
ISR(USART_RXC_vect){
	volatile static unsigned char buf;
	buf=UDR;	//get data from USART
}

//RS232 data sending
void USART_Send(char *data){
	unsigned char n;
	for(n=0;(n<255 && data[n]!=0) ;n++){
		UDR=data[n]; //send command
		while ( !( UCSRA & (1<<UDRE)) );
	}
}



int main(void){
global_err=0; //err clear
//USART init
UCSRB=0x98; //enable RX and TX, int for receive data complete
UCSRC=0x06;  //asynchronous, parity generation off, one stop bit, 8bit data
UBRRL=0x19; // 19.2kbit/s on 8Mhz
UBRRH=0x00; // 19.2kbit/s on 8Mhz
//SPI init
SPCR=0x51; //MSB first, master, clock polarity low, data sampling rising edge, 500KHz clock
SPSR=0x00;
//i/o init
//PORTB config MOSI SCK SS SDN as output , MISO as input
DDRB=0x2F;

PORTB=PORTB | 0x01; //status LED ON

sei(); //int enable

//buffers
char txt_buf[128]; //char buffer
unsigned char rec_buf[FIELD_LENGTH]; //receive buffer
unsigned char tx_buf[FIELD_LENGTH]; //send buffer
unsigned char tmp;

_delay_ms(1000);
//Radio init
USART_Send("Setting up DRF4463\n\r");
DRF4463_init(); //init DRF4463
USART_Send("Setting completed\n\r");
//PART INFO
USART_Send("Part info\n\r");
tmp=1;
SPI_command(&tmp,1);
SPI_response(rec_buf,8);
sprintf(txt_buf,"Chip rev:%X Part:%X%X Build:%X\n\r",rec_buf[0],rec_buf[1],rec_buf[2],rec_buf[3]);
USART_Send(txt_buf);
sprintf(txt_buf,"CID:%X%X Customer:%X Rom:%X \n\r",rec_buf[4],rec_buf[5],rec_buf[6],rec_buf[7]);
USART_Send(txt_buf);
//FUNC_INFO
USART_Send("Func info:\n\r");
tmp=0x10;
SPI_command(&tmp,1);
SPI_response(rec_buf,5);
sprintf(txt_buf,"ExtRev:%X BranchRev:%X IntRev:%X Patch:%X%X \n\r",rec_buf[0],rec_buf[1],rec_buf[2],rec_buf[3],rec_buf[4]);
USART_Send(txt_buf);
SPI_FIFO_read(rec_buf,FIELD_LENGTH);
//end of radio init

PORTB=PORTB & 0xFE; //status LED OFF

USART_Send("Output packet number,mobile station RSSI,base station RSSI, $GPGGA GPS data\n\r");

unsigned char a;
unsigned char RX_RSSI;
unsigned long int n=0;
//main loop
while(1){

	//preparing tx buffer
	for(a=0;a<FIELD_LENGTH;a++){ // 0-3 header (0,1,2,3), 4-59 dumy data (4,5,...,59);
		tx_buf[a]=a;
	}


	//USART_Send("Sending query\n\r");

	SPI_FIFO_write(tx_buf,FIELD_LENGTH);

	//USART_Send("Waiting\n\r");

	//waiting for data receive or timeout
	n=0;
	while((PIND&0x04)!=0){ //waiting for nIRQ low pulse (is better to use MCU IRQ and timers)

		if(n>=200000){
			//USART_Send("Time out\n\r");
			break;
		}
		n++;
		_delay_us(10);
	}


	RX_RSSI=SPI_fregister(0x50); //RSSI level
	//sprintf(txt_buf,"Rec RSSI:%i\n\r",RX_RSSI);
	//USART_Send(txt_buf);

	//clr_interrupt
	SPI_command(RF_CLR_INT_data,sizeof(RF_CLR_INT_data));
	//get status
	SPI_command(RF_INT_STATUS_data,1);
	SPI_response(rec_buf,4);

	//getting data from FIFO
	if((rec_buf[3] &0x08) == 0 && n<200000){  //check crc error and timeout
		PORTB=PORTB | 0x01; //status LED ON

		SPI_FIFO_read(rec_buf,FIELD_LENGTH);

		sprintf(txt_buf,"%i,%i,%i,",rec_buf[4],rec_buf[5],RX_RSSI); //mobile station packer number, mobile station RSSI, base station RSSI
		USART_Send(txt_buf);

		USART_Send(rec_buf+6); //print GPS data, omit header data
		USART_Send("\n\r"); //new line

		PORTB=PORTB & 0xFE; //status LED OFF
	}


	//error checking
	if(global_err!=0){
		USART_Send("\n\r");
		if((global_err&0x01)!=0)
			USART_Send("SPI err\n\r");
		if((global_err&0x02)!=0)
			USART_Send("DRF4436 CTS err\n\r");
		if((global_err&0x04)!=0)
			USART_Send("FIFO TX err\n\r");

		USART_Send("Reinit DRF4463\n\r");
		global_err=0;
		DRF4463_init();
	}


	_delay_ms(5000); //(is better to use timers and MCU IRQ)
	_delay_ms(5000);


	}
}