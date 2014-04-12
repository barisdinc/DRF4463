/*
 * DRF4463.c
 *
 * Example code for testing DRF4463 module
 * and.elektroda.net for elektroda.pl
 * Created: 29.09.2013
 * Based on Dorji (www.dorji.com) example code and output of WDS3
 * (Wireless Development Suite (WDS) Modules www.silabs.com)
 *
 */

#include "DRF4463_base.h"
#include "DRF4463.h"
#include <avr/io.h>
#include <plib/delays.h>
#include <avr/interrupt.h>


//Data from WDS
const unsigned char RF_POWER_UP_data[] = 			   	{ RF_POWER_UP};		// parameter of power on
const unsigned char RF_GPIO_PIN_CFG_data[] = 			   	{ RF_GPIO_PIN_CFG};    // GPIO setting
const unsigned char RF_GLOBAL_XO_TUNE_1_data[] = 		   	{ RF_GLOBAL_XO_TUNE_1};  // crystal setting
const unsigned char RF_GLOBAL_CONFIG_1_data[] = 		   	{ RF_GLOBAL_CONFIG_1};   //fifo and working mode setting
const unsigned char RF_INT_CTL_ENABLE_2_data[] = 		   	{ RF_INT_CTL_ENABLE_2};  // interrupt setting
const unsigned char RF_FRR_CTL_A_MODE_4_data[] = 		   	{ RF_FRR_CTL_A_MODE_4};   // fast read register setting
const unsigned char RF_PREAMBLE_TX_LENGTH_9_data[] = 		{ RF_PREAMBLE_TX_LENGTH_9};       //preamble setting
const unsigned char RF_SYNC_CONFIG_5_data[] = 		 	   	{ RF_SYNC_CONFIG_5};     // sync word setting
const unsigned char RF_PKT_CRC_CONFIG_1_data[] = 		   	{ RF_PKT_CRC_CONFIG_1};   // CRC setting
const unsigned char RF_PKT_CONFIG1_1_data[] = 			   	{ RF_PKT_CONFIG1_1};      // package setting
const unsigned char RF_PKT_LEN_3_data[] = 			   		{ RF_PKT_LEN_3};   // package length setting
const unsigned char RF_PKT_FIELD_1_LENGTH_12_8_12_data[] =	{ RF_PKT_FIELD_1_LENGTH_12_8_12};   // field 1--3
const unsigned char RF_PKT_FIELD_4_LENGTH_12_8_8_data[] = 	{ RF_PKT_FIELD_4_LENGTH_12_8_8};    // field 4--5
const unsigned char RF_MODEM_MOD_TYPE_12_data[] = 			   	{ RF_MODEM_MOD_TYPE_12};
const unsigned char RF_MODEM_FREQ_DEV_0_1_data[] = 		   	{ RF_MODEM_FREQ_DEV_0_1};    //deviation
const unsigned char RF_MODEM_TX_RAMP_DELAY_8_data[] = 			   	{ RF_MODEM_TX_RAMP_DELAY_8};
const unsigned char RF_MODEM_BCR_OSR_1_9_data[] = 			   	{ RF_MODEM_BCR_OSR_1_9};
const unsigned char RF_MODEM_AFC_GEAR_7_data[] = 			   	{ RF_MODEM_AFC_GEAR_7};
const unsigned char RF_MODEM_AGC_CONTROL_1_data[] = 		{ RF_MODEM_AGC_CONTROL_1};	     // AGC
const unsigned char RF_MODEM_AGC_WINDOW_SIZE_9_data[] = 			   	{ RF_MODEM_AGC_WINDOW_SIZE_9};
const unsigned char RF_MODEM_OOK_CNT1_11_data[] = 			   	{ RF_MODEM_OOK_CNT1_11};
const unsigned char RF_MODEM_RSSI_COMP_1_data[] = 			   	{ RF_MODEM_RSSI_COMP_1};
const unsigned char RF_MODEM_CLKGEN_BAND_1_data[] = 			   	{ RF_MODEM_CLKGEN_BAND_1};
const unsigned char RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_data[] = 			   	{ RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12};
const unsigned char RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_data[] = 			   	{ RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12};
const unsigned char RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_data[] = 			   	{ RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12};
const unsigned char RF_PA_MODE_4_data[] = 			   	{ RF_PA_MODE_4};
const unsigned char RF_SYNTH_PFDCP_CPFF_7_data[] = 			   	{ RF_SYNTH_PFDCP_CPFF_7};
const unsigned char RF_MATCH_VALUE_1_12_data[] =            { RF_MATCH_VALUE_1_12};		    // header
const unsigned char RF_FREQ_CONTROL_INTE_8_data[] = 			   	{ RF_FREQ_CONTROL_INTE_8};
//Commands
const unsigned char RF_FIFO_RESET_data[] = 			   	{ 0x15, 0x03}; //FIFO_INFO, reset tx rx fifo
const unsigned char RF_INT_TX_data[] =	{0x11, 0x01, 0x02, 0x00, 0x01, 0x20}; //enable interrupt on packet send
const unsigned char RF_CLR_INT_data[] =	{0x20, 0x00, 0x00, 0x00}; //clr interrupt PH, modem, chip
const unsigned char RF_TX_FIFO_data[] =	{0x31, 0x00, 0x30, 0x00, 0x00}; //send data from FIFO
const unsigned char RF_INT_RX_data[] =	{0x11, 0x01, 0x03, 0x00, 0x01, 0x18, 0x00}; //enable interrupt on packet received
const unsigned char RF_RX_MODE_data[] =	{0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08}; //enable receive mode
const unsigned char RF_INT_STATUS_data = 0x20;



//send and receive data byte via SPI
unsigned char SPI_byte(unsigned char data){
	unsigned int err=0;
	SPDR=data;
	while(!(SPSR & (1<<SPIF))){ //waiting for send
		err++;
		if(err==65535){
			global_err=global_err|1; //err SPI not responding
			break;
		}
		_delay_us(10);
	}
	return SPDR;
}

unsigned char SPI_fregister(unsigned char r){
unsigned char tmp;
	PORTB=PORTB | 0x04; //set SS H
	_delay_us(50);
	PORTB=PORTB & 0xFB; //set SS L
	_delay_us(50);

	SPI_byte(r);
	tmp=SPI_byte(0xFF);
	PORTB=PORTB | 0x04; //set SS H
	return tmp;
}

//wait for CTS from DRF4463 (chip is ready)
void SPI_cts(){
unsigned int err=0;
	do{
		PORTB=PORTB | 0x04; //set SS H
		_delay_us(10);
		PORTB=PORTB & 0xFB; //set SS L
		_delay_us(10);
		SPI_byte(0x44); //send CTS
		err++;
		if(err==65535){
			global_err=global_err|2; //err CTS not present
			break;
		}
	}while(SPI_byte(0xFF)!=0xFF); //wait for CTS

}

//send command string to DRF4463 via SPI
void SPI_command(const unsigned char *data, unsigned char n){
unsigned char t;
	SPI_cts();
	PORTB=PORTB | 0x04; //set SS H
	_delay_us(5);
	PORTB=PORTB & 0xFB; //set SS L
	_delay_us(5);
	for(t=0;t<n;t++)	//sending data
		SPI_byte(data[t]);
	PORTB=PORTB | 0x04; //set SS H
}

//receive command response from DRF4463 via SPI
void SPI_response(unsigned char *data,unsigned char n){
unsigned char t;
	SPI_cts();
	for(t=0;t<n;t++)
		data[t]=SPI_byte(0xFF);
	PORTB=PORTB | 0x04; //set SS H
}

//write data to DRF4463 FIFO and send it
void SPI_FIFO_write(unsigned char *data, unsigned char n){
unsigned int err=0;
unsigned char t;
	SPI_command(RF_FIFO_RESET_data,sizeof(RF_FIFO_RESET_data)); // reset tx ,rx fifo
	//write fifo
	SPI_cts();
	PORTB=PORTB | 0x04; //set SS H
	_delay_us(5);
	PORTB=PORTB & 0xFB; //set SS L
	_delay_us(5);
	SPI_byte(0x66); //FIFO write
	for(t=0;t<n;t++){ //sending data to FIFO
		SPI_byte(data[t]);
	}
	PORTB=PORTB | 0x04; //set SS H
	_delay_us(5);
	//enable_tx_interrupt
	SPI_command(RF_INT_TX_data,sizeof(RF_INT_TX_data));
	//clr_interrupt
	SPI_command(RF_CLR_INT_data,sizeof(RF_CLR_INT_data));
	//transmit data
	SPI_command(RF_TX_FIFO_data,sizeof(RF_TX_FIFO_data));
	while((PIND&0x04)!=0){ //waiting for nIRQ low pulse
		err++;
		if(err==65535){
			global_err=global_err|4; //err TX FIFO
			break;
		}
		_delay_us(20);
	}
	//back to rx mode
	//reset fifo
	SPI_command(RF_FIFO_RESET_data,sizeof(RF_FIFO_RESET_data)); // reset tx ,rx fifo
	//enable_rx_interrupt
	SPI_command(RF_INT_RX_data,sizeof(RF_INT_RX_data));
	//clr_interrupt
	SPI_command(RF_CLR_INT_data,sizeof(RF_CLR_INT_data));
	//rx mode
	SPI_command(RF_RX_MODE_data,sizeof(RF_RX_MODE_data));
}

void SPI_FIFO_read(unsigned char *data, unsigned char n){
	unsigned char t;
	//clr_interrupt
	SPI_command(RF_CLR_INT_data,sizeof(RF_CLR_INT_data));
	//reading fifo
	SPI_cts();
	PORTB=PORTB | 0x04; //set SS H
	_delay_us(5);
	PORTB=PORTB & 0xFB; //set SS L
	_delay_us(5);
	SPI_byte(0x77); //FIFO read
	for(t=0;t<n;t++)
		data[t]=SPI_byte(0xFF);
	PORTB=PORTB | 0x04; //set SS H
	_delay_us(5);
	//reset fifo
	SPI_command(RF_FIFO_RESET_data,sizeof(RF_FIFO_RESET_data)); // reset tx ,rx fifo
	//enable_rx_interrupt
	SPI_command(RF_INT_RX_data,sizeof(RF_INT_RX_data));
	//clr_interrupt
	SPI_command(RF_CLR_INT_data,sizeof(RF_CLR_INT_data));
	//rx mode
	SPI_command(RF_RX_MODE_data,sizeof(RF_RX_MODE_data));
}


void DRF4463_init(){

	//SDN = 1
	PORTB=PORTB | 0x02; //set SDN H
	_delay_ms(2);	// RF module reset
	//SDN = 0
	PORTB=PORTB & 0xFD; //set SDN L
	_delay_ms(10);	// delay 10ms
	PORTB=PORTB | 0x04; //set SS H
	_delay_us(5);
	PORTB=PORTB & 0xFB; //set SS L
	_delay_us(5);
	SPI_command(RF_POWER_UP_data,sizeof(RF_POWER_UP_data));
	_delay_ms(20);	// delay 20ms ,RF module start working
	SPI_command(RF_GPIO_PIN_CFG_data,sizeof(RF_GPIO_PIN_CFG_data));
	SPI_command(RF_GLOBAL_XO_TUNE_1_data,sizeof(RF_GLOBAL_XO_TUNE_1_data));
	SPI_command(RF_GLOBAL_CONFIG_1_data,sizeof(RF_GLOBAL_CONFIG_1_data));
	SPI_command(RF_INT_CTL_ENABLE_2_data,sizeof(RF_INT_CTL_ENABLE_2_data));
	SPI_command(RF_FRR_CTL_A_MODE_4_data,sizeof(RF_FRR_CTL_A_MODE_4_data));
	SPI_command(RF_PREAMBLE_TX_LENGTH_9_data,sizeof(RF_PREAMBLE_TX_LENGTH_9_data));
	SPI_command(RF_SYNC_CONFIG_5_data,sizeof(RF_SYNC_CONFIG_5_data));
	SPI_command(RF_PKT_CRC_CONFIG_1_data,sizeof(RF_PKT_CRC_CONFIG_1_data));
	SPI_command(RF_PKT_CONFIG1_1_data,sizeof(RF_PKT_CONFIG1_1_data));
	SPI_command(RF_PKT_LEN_3_data,sizeof(RF_PKT_LEN_3_data));
	SPI_command(RF_PKT_FIELD_1_LENGTH_12_8_12_data,sizeof(RF_PKT_FIELD_1_LENGTH_12_8_12_data));
	SPI_command(RF_PKT_FIELD_4_LENGTH_12_8_8_data,sizeof(RF_PKT_FIELD_4_LENGTH_12_8_8_data));
	SPI_command(RF_MODEM_MOD_TYPE_12_data,sizeof(RF_MODEM_MOD_TYPE_12_data));
	SPI_command(RF_MODEM_FREQ_DEV_0_1_data,sizeof(RF_MODEM_FREQ_DEV_0_1_data));
	SPI_command(RF_MODEM_TX_RAMP_DELAY_8_data,sizeof(RF_MODEM_TX_RAMP_DELAY_8_data));
	SPI_command(RF_MODEM_BCR_OSR_1_9_data,sizeof(RF_MODEM_BCR_OSR_1_9_data));
	SPI_command(RF_MODEM_AFC_GEAR_7_data,sizeof(RF_MODEM_AFC_GEAR_7_data));
	SPI_command(RF_MODEM_AGC_CONTROL_1_data,sizeof(RF_MODEM_AGC_CONTROL_1_data));
	SPI_command(RF_MODEM_AGC_WINDOW_SIZE_9_data,sizeof(RF_MODEM_AGC_WINDOW_SIZE_9_data));
	SPI_command(RF_MODEM_OOK_CNT1_11_data,sizeof(RF_MODEM_OOK_CNT1_11_data));
	SPI_command(RF_MODEM_RSSI_COMP_1_data,sizeof(RF_MODEM_RSSI_COMP_1_data));
	SPI_command(RF_MODEM_CLKGEN_BAND_1_data,sizeof(RF_MODEM_CLKGEN_BAND_1_data));
	SPI_command(RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_data,sizeof(RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_data));
	SPI_command(RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_data,sizeof(RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_data));
	SPI_command(RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_data,sizeof(RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_data));
	SPI_command(RF_PA_MODE_4_data,sizeof(RF_PA_MODE_4_data));
	SPI_command(RF_SYNTH_PFDCP_CPFF_7_data,sizeof(RF_SYNTH_PFDCP_CPFF_7_data));
	SPI_command(RF_MATCH_VALUE_1_12_data,sizeof(RF_MATCH_VALUE_1_12_data));
	SPI_command(RF_FREQ_CONTROL_INTE_8_data,sizeof(RF_FREQ_CONTROL_INTE_8_data));

}
