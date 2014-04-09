// 	This Demo program suit for £º   DRF4463F which used the silabs RF chip SI4463,
// 	The parameter is : +/-10PPM£¬¡¡425MHz, 1.2K bps,  3K Deviation,  enable AFC , CRC and PH + FiFo mode.
// 	Test data is : 0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x6d,
//      Header£º "swwx"
//  chksum:  0x6d = (0x41 +0x42 +0x43 +0x44 +0x45 +0x46 +0x47 + 0x48 +0x49)
// 	MCU : RENSAS high performance 8 bit MCU :R5f211B4, internal 8MHz
//  DORJI APPLIED TECHNOLOGIES WWW.DORJI.COM  2013-09-10


#include <pic18f24k22.h>
#include "radio_config_Si4463.h"
#include <xc.h>
#include "config.h"

typedef 	unsigned char	U8;
typedef 	unsigned int	U16;
typedef 	unsigned long	U32;
typedef 	char			S8;
typedef 	int			S16;
typedef 	long			S32;
#define  	SDN	        	PORTBbits.RB0    // si4463 power on/down control
#define  	nIRQ       		PORTBbits.RB1	// Si4463 interrupt control
#define  	nSEL			PORTBbits.RB2	// si4463 SPI control
#define  	SCK			PORTBbits.RB3	// si4463 SPI control
#define  	SDI        		PORTBbits.RB4    // si4463 SPI control
#define  	SDO	        	PORTBbits.RB5	// si4463 SPI control

#define  	LED_GREEN		PORTAbits.RA4  //LED indicator for data received
#define  	LED_BLUE			PORTAbits.RA5  //LED indicator for data transmitted

#define 	master_mode   	0	// master mode, Tx data every 1 second
#define 	slave_mode    	1       // slave mode, reply ACK signal(same as the data received) when received the correct data

//BARIS #define 	EI();		asm("FSET I");
//BARIS #define 	DI();		asm("FCLR I");


// Below is the define of si4463 command

//#define NOP 				0x00
#define PART_INFO                       0x01 // 9
#define FUNC_INFO                       0x10 // 7
#define SET_PROPERTY                    0x11
#define GET_PROPERTY                    0x12
#define GPIO_PIN_CFG                    0x13    // 8
#define GET_ADC_READING                 0x14
#define FIFO_INFO                       0x15    // 3
#define PACKET_INFO                     0x16    // 3
#define IRCAL                           0x17
#define PROTOCOL_CFG                    0x18
#define GET_INT_STATUS                  0x20    // 9
#define GET_PH_STATUS                   0x21    // 3
#define GET_MODEM_STATUS                0x22    // 9
#define GET_CHIP_STATUS                 0x23     // 4
#define START_TX                        0x31
#define START_RX                        0x32
#define REQUEST_DEVICE_STAT             0x33   // 3
#define CHANGE_STATE                    0x34
#define READ_CMD_BUFF                   0x44
#define FRR_A_READ                      0x50   // 4
#define FRR_B_READ                      0x51
#define FRR_C_READ                      0x53
#define FRR_D_READ                      0x57
#define WRITE_TX_FIFO                   0x66
#define READ_RX_FIFO                    0x77
#define START_MFSK                      0x35
#define RX_HOP                          0x36

#define payload_length  				14    //payload length = 4 (header) + 10 (payload)

#define freq_channel		0
#define step_500K_step1		0x88    // register setting for frequency 500KHz step
#define step_500K_step0		0x89	// register setting for frequency 500KHz step

typedef struct
{
	unsigned char reach_1ms				: 1;
	unsigned char reach_1s				: 1;
	unsigned char is_tx					: 1;
	unsigned char rf_reach_timeout		: 1;
}FlagType;


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
const unsigned char RF_MODEM_FREQ_DEV_0_1_data[] = 		   	{ RF_MODEM_FREQ_DEV_0_1};    //deviation
const unsigned char RF_MODEM_AGC_CONTROL_1_data[] = 		{ RF_MODEM_AGC_CONTROL_1};	     // AGC
const unsigned char RF_MATCH_VALUE_1_12_data[] =            { RF_MATCH_VALUE_1_12};		    // header

//  below parameter is got from the WDS of Silabs
unsigned char RF_MODEM_MOD_TYPE_12_data[16]={0x11, 0x20, 0x0C, 0x00,0x03, 0x00, 0x07, 0x00, 0x12, 0xC0, 0x04, 0x2D, 0xC6, 0xC0, 0x00, 0x00};
unsigned char RF_MODEM_TX_RAMP_DELAY_8_data[12]={0x11, 0x20, 0x08, 0x18,0x01, 0x80, 0x08, 0x03, 0x80, 0x00, 0x70, 0x20};
unsigned char RF_MODEM_BCR_OSR_1_9_data[13]={0x11, 0x20, 0x09, 0x22,0x03, 0x0D, 0x00, 0xA7, 0xC6, 0x00, 0x54, 0x02, 0xC2};
unsigned char RF_MODEM_AFC_GEAR_7_data[13]={0x11, 0x20, 0x07, 0x2C,0x04, 0x36, 0x80, 0x03, 0x30, 0xAF, 0x80};
unsigned char RF_MODEM_AGC_WINDOW_SIZE_9_data[13]={0x11, 0x20, 0x09, 0x38,0x11, 0xAB, 0xAB, 0x00, 0x1A, 0x14, 0x00, 0x00, 0x2B};
unsigned char RF_MODEM_OOK_CNT1_11_data[15]={0x11, 0x20, 0x0B, 0x42,0xA4, 0x02, 0xD6, 0x83, 0x00, 0xAD, 0x01, 0x80, 0xFF, 0x0C, 0x00};
unsigned char RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_data[16]={0x11, 0x21, 0x0C, 0x00,0xFF, 0xC4, 0x30, 0x7F, 0xF5, 0xB5, 0xB8, 0xDE, 0x05, 0x17, 0x16, 0x0C};
unsigned char RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_data[16]={0x11, 0x21, 0x0C, 0x0C,0x03, 0x00, 0x15, 0xFF, 0x00, 0x00, 0xFF, 0xC4, 0x30, 0x7F, 0xF5, 0xB5};
unsigned char RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_data[16]={0x11, 0x21, 0x0C, 0x18,0xB8, 0xDE, 0x05, 0x17, 0x16, 0x0C, 0x03, 0x00, 0x15, 0xFF, 0x00, 0x00};
unsigned char RF_SYNTH_PFDCP_CPFF_7_data[11]={0x11, 0x23, 0x07, 0x00,0x2C, 0x0E, 0x0B, 0x04, 0x0C, 0x73, 0x03};

// Below data is fixed for each transmission
const unsigned char tx_ph_data[14] = {'s','w','w','x',0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x6d};  //The last data is the checksum. 0x69 = 0x41 + 0x42+...0x49

FlagType	Flag;

U16	count_1hz, rf_timeout;
U8 spi_read_buf[20];   //
U8 rx_buf[25];
U8 mode;

void spi_read(U8 data_length, U8 api_command );
void tx_data(void);
void SI4463_init(void);
void sysclk_cfg(void);
void port_init(void);
void timerx_init(void);
void delay_1ms(unsigned int delay_time);
unsigned char spi_byte(unsigned char data);
U8 check_cts(void);
void spi_write(unsigned char tx_length, unsigned char *p);

void spi_write_fifo(void);
void spi_read_fifo(void);
void sdn_reset(void);
void clr_interrupt(void);
void fifo_reset(void);
void enable_tx_interrupt(void);
void enable_rx_interrupt(void);
void tx_start(void);
void rx_start(void);
void rx_init(void);



void rf_standby(void);
void rf_init_freq(void);
void nop_10();
void delay_10ms(void);
void delay_x10ms(unsigned int dx10ms);

void main()
{

	//sysclk_cfg();
	nop_10();
	port_init();	
	nop_10();

	delay_x10ms(200);	// power on delay
	LED_GREEN = 0;
	LED_BLUE = 1;
	unsigned char  i, j, chksum;

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

      	mode = master_mode;   // working mode setting

	//while(1)
	//{
	 	asm("nop");

	 	sdn_reset();
 		SI4463_init();  // RF module initial

                
	 	

                

	//}

                while (1)
                {
                    delay_x10ms(20);	// power on delay
                    LED_GREEN = 0;
                    LED_BLUE = 1;
                    delay_x10ms(20);	// power on delay
                    LED_GREEN = 1;
                    LED_BLUE = 0;



                    if(mode == master_mode)
	 		tx_data();
                    if(mode == slave_mode)
	 		rx_init();



                }


}




void main2()
{
	unsigned char  i, j, chksum;

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
//BARIS	DI();

	sysclk_cfg();	// MCU system initial
	nop_10();
	port_init();	// MCU IO port initial
	nop_10();

	delay_x10ms(200);	// power on delay
//	LED_GREEN = 0;
	//LED_BLUE = 0;

	timerx_init();  // 1ms timer
//BARIS	EI();
	asm("nop");

 	mode = master_mode;   // working mode setting
 	//mode = slave_mode;


	while(1)
	{
	 	asm("nop");
	 	sdn_reset();
 		SI4463_init();  // RF module initial

	 	if(mode == master_mode)
	 		tx_data();
	 	if(mode == slave_mode)
	 		rx_init();



		if((mode == master_mode)||(mode == slave_mode))
		{
			count_1hz = 0;
			Flag.reach_1s = 0;

			while(1)
			{
				if(Flag.reach_1s)
				{
					Flag.reach_1s = 0;
					if(mode == master_mode)
						tx_data();		// when working in master mode ,Tx data per second and waiting for the ACK signal
				}

				if(!Flag.is_tx)
				{
					if(!nIRQ)
					{
						clr_interrupt();   // clear interrupt, read the int factor

						if((spi_read_buf[4] &0x08) == 0)  // if crc error check
						{
							spi_read_fifo();	// read out the FiFo data
							fifo_reset();

							chksum = 0;
							for(i=4;i<payload_length-1;i++)		// Cal the checksum
		        				chksum += rx_buf[i];

		     				if(( chksum == rx_buf[payload_length-1] )&&( rx_buf[4] == 0x41 ))  // check if the data correct
		     				{
//		     					LED_GREEN ^= 1;					// data got and correct

		     					if(mode == slave_mode)
		     					{
									delay_1ms(100);
		     						tx_data();	// if working in slave mode , reply the ack signal
		     					}
		        			}
		        			else
		        				rx_init();     // data wrong, restart Rx
		        		}
		        		else
		        		{
							rx_init();   // crc error, restart rx
		        		}
		        	}
				}
			}
		}
	}
}

void SI4463_init(void)
{
	U8 app_command_buf[20],i;

	app_command_buf[0] = 0x13;     // gpio setting
	app_command_buf[1]  = 0x14; // 0x21;    // gpio 0 ,Rx data
	app_command_buf[2]  = 0x02;    // gpio1, output 0  high level when power on
	app_command_buf[3]  = 0x21;  //0x20;   //  gpio2, h = tx mode
	app_command_buf[4]  = 0x20; // 0x14;  //   gpio3
	app_command_buf[5]  = 0x27;   // nIRQ
	app_command_buf[6]  = 0x0b;  // sdo

        spi_write(7, app_command_buf);


        
        
 	app_command_buf[0] = 0x11;         //  frequency adjust
	app_command_buf[1]  = 0x00;    // 0x0000
	app_command_buf[2]  = 0x01;    // total 1 parameter
	app_command_buf[3]  = 0x00;   // 0x0000
	app_command_buf[4]  = 98;  // freq  adjustment
	spi_write(5, app_command_buf);


  	app_command_buf[0] = 0x11;     // rf global setting
	app_command_buf[1]  = 0x00;    // 0x0003
	app_command_buf[2]  = 0x01;     // total 1 parameter
	app_command_buf[3]  = 0x03;   // 0x0003
	app_command_buf[4]  = 0x40;  // tx = rx = 64 byte,  PH£¬high performance mode
	spi_write(5, app_command_buf);

    // *****************************************************************************
//    spi_write(0x08, RF_FRR_CTL_A_MODE_4_data);    // disable all fast response register
        //ustteki spiwrite yerine bunu koyuyorum

        app_command_buf[0]  = 0x11;     // disable all fast response register
	app_command_buf[1]  = 0x02;
	app_command_buf[2]  = 0x04;     // total 4 parameter
	app_command_buf[3]  = 0x00;   // 0x0200
	app_command_buf[4]  = 0x00;  // tx = rx = 64 byte,  PH£¬high performance mode
	app_command_buf[5]  = 0x00;  // tx = rx = 64 byte,  PH£¬high performance mode
	app_command_buf[6]  = 0x00;  // tx = rx = 64 byte,  PH£¬high performance mode
	app_command_buf[7]  = 0x00;  // tx = rx = 64 byte,  PH£¬high performance mode
	spi_write(8, app_command_buf);



 	app_command_buf[0] = 0x11;  	// preamble setting
	app_command_buf[1]  = 0x10;    // 0x1000
	app_command_buf[2]  = 0x09;    // total 9 parameter
	app_command_buf[3]  = 0x00;   // 0x1000

	app_command_buf[4]  = 0x08;   //  8 byte preamble

	app_command_buf[5]  = 0x14;   //20 bit preamble detectin
	app_command_buf[6]  = 0x00;   // standard preamble
	app_command_buf[7]  = 0x0f;   //
	app_command_buf[8]  = 0x31;  // preamble, no machest, lsb, unit = byte for preamble length
	app_command_buf[9]  = 0x0;  	//  abnormal preamble
	app_command_buf[10]  = 0x00;   //  abnormal preamble
	app_command_buf[11]  = 0x00;  //  abnormal preamble
	app_command_buf[12]  = 0x00;  //  abnormal preamble
	spi_write(13, app_command_buf);  //


    app_command_buf[0] = 0x11;  	// sync word setting
	app_command_buf[1]  = 0x11;    // command = 0x1100
	app_command_buf[2]  = 0x05;     // total 9 parameter
	app_command_buf[3]  = 0x00;   // command = 0x1100
	app_command_buf[4]  = 0x01;   //  sync word define in length field , not 4fsk, not manchest code,2 byte, no error accepted

	app_command_buf[5]  = 0x2d;   //sync byte 3
	app_command_buf[6]  = 0xd4;   // sync byte 2

	app_command_buf[7]  = 0x00;   // sync byte 1, no use
	app_command_buf[8]  = 0x00;  // sync byte 0, no use
    spi_write(9, app_command_buf);  //


    app_command_buf[0] = 0x11;     // crc setting
	app_command_buf[1]  = 0x12;    // command = 0x1200
	app_command_buf[2]  = 0x01;    // total 1 parameter
	app_command_buf[3]  = 0x00;   // command = 0x1200
	app_command_buf[4]  = 0x81; //   crc seed , enable, itu-c, enabe.
    spi_write(5, app_command_buf);


    app_command_buf[0] = 0x11;  	// packet   gernale configuration
	app_command_buf[1]  = 0x12;    // command = 0x1206
	app_command_buf[2]  = 0x01;    // total 1 parameter
	app_command_buf[3]  = 0x06;   // command = 0x1206
	app_command_buf[4]  = 0x02;   //  tx = rx = 120d--1220 (tx packet£¬ph enable, not 4fsk, RX off after received data,CRC no invertor£¬CRC MSB£¬ data MSB
    spi_write(5, app_command_buf);


    app_command_buf[0] = 0x11;  	// packet length
	app_command_buf[1]  = 0x12;    // command = 0x1208
	app_command_buf[2]  = 0x03;    // total 3 parameter
	app_command_buf[3]  = 0x08;   // command = 0x1208
	app_command_buf[4]  = 0x00;   //  Length Field = LSB,  length = 1byte£¬length not in FiFo, fixed length
	app_command_buf[5]  = 0x00;   //
	app_command_buf[6]  = 0x00;   //
    spi_write(7, app_command_buf);



	app_command_buf[0] = 0x11;  	// field 1-- 4 setting
	app_command_buf[1]  = 0x12;    // 0x120d
	app_command_buf[2]  = 0x0c;    // total 12 parameter
	app_command_buf[3]  = 0x0d;   // 0x120d
	app_command_buf[4]  = 0x00;   //  length of field 1 , bit 8--11
	app_command_buf[5]  = payload_length;   //  lengh of field 1, bit 0--7
	app_command_buf[6]  = 0x04;   // field 1 not 4fsk, not manchest, no whiting,not PN9,
	app_command_buf[7]  = 0xaa;   // field 1 crc enble, check enbale,
	app_command_buf[8]  = 0x00;  //  length of field2 bit8--bit11
	app_command_buf[9]  = 0x00;  	//  length of field2 bit0--bit7
	app_command_buf[10]  = 0x00;   // field 2  not 4FSK£¬manchest, whiting, PN9
	app_command_buf[11]  = 0x00;  //  field 2 CRC
	app_command_buf[12]  = 0x00;  //  length of field3 bit8--bit11
	app_command_buf[13]  = 0x00;  	//  length of field2 bit0--bit7
	app_command_buf[14]  = 0x00;   //  field 3 not 4FSK£¬manchest, whiting, PN9
	app_command_buf[15]  = 0x00;  //  field 3 CRC
	spi_write(16, app_command_buf);  //


    app_command_buf[0] = 0x11;
	app_command_buf[1]  = 0x12;    // 0x1219
	app_command_buf[2]  = 0x08;   // total 12 parameter
	app_command_buf[3]  = 0x19;   // 0x1219
	app_command_buf[4]  = 0x00;   //  length of field4 bit8--bit11
	app_command_buf[5]  = 0x00;   //  length of field5 bit0--bit7
	app_command_buf[6]  = 0x00;   //   field 4 not 4FSK£¬manchest, whiting, PN9
	app_command_buf[7]  = 0x00;   // field 4 CRC
	app_command_buf[8]  = 0x00;  //  length of field5 bit8--bit11
	app_command_buf[9]  = 0x00;  	//  length of field5 bit0--bit7
	app_command_buf[10]  = 0x00;   //  field 5 not 4FSK£¬manchest, whiting, PN9
	app_command_buf[11]  = 0x00;   // field 5 CRC
    spi_write(12, app_command_buf);  //

    // ********************************************************************************
    spi_write(0x10, RF_MODEM_MOD_TYPE_12_data);   // //  2FSK ,  module source = PH fifo, disable manchest,



    app_command_buf[0] = 0x11;
	app_command_buf[1]  = 0x20;    // 0x200c
	app_command_buf[2]  = 0x01;    // total 1 parameter
	app_command_buf[3]  = 0x0c;   // 0x200c
	app_command_buf[4]  = 0x5e; // frequency deviation bit0-- 7
    spi_write(5, app_command_buf);

    // ********************************************************************************
    spi_write(0x0C, RF_MODEM_TX_RAMP_DELAY_8_data);    	// fixed, from WDS
    spi_write(0x0D, RF_MODEM_BCR_OSR_1_9_data);		// fixed, from WDS
    spi_write(0x0B, RF_MODEM_AFC_GEAR_7_data);		// fixed, from WDS
//BARIS    spi_write(0x05, RF_MODEM_AGC_CONTROL_1_data);	// fixed, from WDS
    spi_write(0x0D, RF_MODEM_AGC_WINDOW_SIZE_9_data);	// fixed, from WDS
    spi_write(0x0F, RF_MODEM_OOK_CNT1_11_data);		// fixed, from WDS

	// spi_write(0x05, RF_MODEM_RSSI_COMP_1_data);	**************************************************
	app_command_buf[0] = 0x11;
	app_command_buf[1] = 0x20;    // 0x204e
	app_command_buf[2] = 0x01;   // total 1 parameter
	app_command_buf[3] = 0x4e;   // 0x204e
	app_command_buf[4]  = 0x40;  //  rssi, read data tolerance, deviation with real value
    spi_write(5, app_command_buf);

    // ********************************************************************************
    spi_write(0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_data);  // fixed, from WDS
    spi_write(0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_data);   // fixed, from WDS
    spi_write(0x10, RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_data);   // fixed, from WDS

	// RF_PA **************************************************************************
	app_command_buf[0] = 0x11;
	app_command_buf[1]  = 0x22;    // 0x2200
	app_command_buf[2]  = 0x04;    // total 4 parameter
	app_command_buf[3]  = 0x00;   // 0x2200
	app_command_buf[4]  = 0x08;  //0x10;   //   PA mode  = default , use Class E mode£¬not Switch Current mode  ????????
	app_command_buf[5]  = 127;   //  set the max. output power
	app_command_buf[6]  =0x00; //???????? 0x00;   // CLK duty = 50%£¬ Other = Default
	app_command_buf[7]  = 0x3d;  // ???????? 0x5d;   // PA ramp time = default
    spi_write(8, app_command_buf);

    // ********************************************************************************
	spi_write(0x0B, RF_SYNTH_PFDCP_CPFF_7_data);      // fixed, from WDS

   	// header match ******************************************************************
   	app_command_buf[0] = 0x11;
	app_command_buf[1]  = 0x30;    // 0x3000
	app_command_buf[2]  = 0x0c;    // total 12 parameter
	app_command_buf[3]  = 0x00;   // 0x3000
	app_command_buf[4]  = 's';    //  match value of 1st byte
	app_command_buf[5]  = 0xff;   //  match 1 mask
	app_command_buf[6]  = 0x40;   // eable match 1, offset after sync word
	app_command_buf[7]  = 'w';    //  match value of 2nd byte
	app_command_buf[8]  = 0xff;   //  match 2 mask
	app_command_buf[9]  = 0x01;   // enable match 2,offset after sync word
	app_command_buf[10] = 'w';   //  match value of 3rd byte
	app_command_buf[11]  =0xff;;    //  match 3 mask
	app_command_buf[12]  =0x02;;    // enable match 3,  enable match 3,offset after sync word
	app_command_buf[13]  = 'x';    //  match value of 4th byte
	app_command_buf[14]  = 0xff;   //   match 4 mask
	app_command_buf[15]  = 0x03;   //  enable match 4,  enable match 4,offset after sync word
        spi_write(16, app_command_buf);

	rf_init_freq();
}

void fifo_reset(void)
{
	U8 p[2];

	p[0] = FIFO_INFO;
	p[1] = 0x03;   // reset tx ,rx fifo
	spi_write(2,p);
}

void clr_interrupt(void)
{
	U8 p[4];

	p[0] = GET_INT_STATUS;
	p[1] = 0;   // clr  PH pending
	p[2] = 0;   // clr modem_pending
	p[3] = 0;   // clr chip pending
	spi_write(4,p);
	spi_read(9,GET_INT_STATUS);
}

void enable_rx_interrupt(void)
{
	U8 p[7];

	p[0] = 0x11;
	p[1] = 0x01;  // 0x0100
	p[2] = 0x03;  //TOTAL 3 parameters
	p[3] = 0x00;   // 0100
	p[4] = 0x03;   // ph, modem int
	p[5] = 0x18; // 0x10;   // Pack received int
	p[6] = 0x00;   //preamble int, sync int setting
	spi_write(0x07, p);  // enable  packet receive interrupt
}

void enable_tx_interrupt(void)
{
	U8 p[6];

	p[0] = 0x11;
	p[1] = 0x01;  // 0x0100
	p[2] = 0x02;   //
	p[3] = 0x00;  // 0x0100
	p[4] = 0x01;   // Ph  int
	p[5] = 0x20; //  enable  packet sent interrupt
	spi_write(0x06, p);  // enable  packet receive interrupt
}

void rf_standby(void)
{
	U8 p[2];

	p[0] = CHANGE_STATE ; 	// CHANGE_STATE command
	p[1] = 0x01 ; 			// sleep mode
	spi_write(2, p);
}

void tx_start(void)
{
	U8 p[5];

	p[0] = START_TX ; // start command
	p[1] = freq_channel ; // channel 0
	p[2] = 0x30; // return to ready mode after tx
	p[3] = 0;
	p[4] = 0; //payload_length;
	spi_write(5, p);
}

void rx_start(void)
{
	U8 p[8];

	p[0] = START_RX ; // start command
	p[1] = freq_channel ; // channel 0
	p[2] = 0x00; // enter rx mode immediately
	p[3] = 0;
	p[4] = 0;// payload_length;
	p[5] = 0;  // unchanged after preamble timeout
	//p[6] = 0x03;  // ready after valid packet received
	//p[7] = 0x0;  // unchanged after invalid packet received
	p[6] = 0x08;
	p[7] = 0x08;
	spi_write(8, p);
}



void rx_init(void)
{
	Flag.is_tx = 0;
	fifo_reset();  // clr fifo
	enable_rx_interrupt();
	clr_interrupt();  // clr int Factor
	rx_start();    // enter rx mode, return to ready mode after received data
}

void tx_data(void)
{
	unsigned char i;

	Flag.is_tx = 1;
//	LED_BLUE ^= 1;
	fifo_reset();  // clear buffer
	spi_write_fifo(); // fill the fifo
	enable_tx_interrupt();
	clr_interrupt();  //  clr int Factor
	tx_start();    // enter tx mode ,start Tx
	rf_timeout = 0;
	Flag.rf_reach_timeout = 0;

	while(nIRQ)		// wait for int
	{
		//wdtr = 0;  //watchdog time reset register  //baris
		//wdtr = 0xff;
		if(Flag.rf_reach_timeout)
		{
			sdn_reset();
 			SI4463_init();  // RF module initial
			break;		//
		}
	}

  	rx_init();		//Tx finished, back to Rx mode
}

unsigned char spi_byte(unsigned char data)
{
	unsigned char i;

	for (i = 0; i < 8; i++)		// SPI communication
	{
		if (data & 0x80)
			SDI = 1;
		else
			SDI = 0;

		data <<= 1;
                delay_1ms(1);//baris
		SCK = 1;

		if (SDO)
			data |= 0x01;
		else
			data &= 0xfe;
                delay_1ms(1);//baris
		SCK = 0;
	}
	return (data);
}

void spi_write(unsigned char tx_length, unsigned char *p)
{
	unsigned char i,j;

	i = 0;
	while(i!=0xff)
		i = check_cts();

	SCK = 0;
	nSEL = 0;

	for (i = 0; i < tx_length; i++)
	{
		j = *(p+i);
		spi_byte(j);
	}

	nSEL = 1;
}

U8 check_cts(void)
{
	U8 i;

	nSEL = 1;
	SCK = 0;
	nSEL = 0;
	spi_byte(0x44); //44
	i = spi_byte(0);//0
	nSEL = 1;
        i = 0xff; //burayi gecmek icin koydum  BARIS
	return (i);
}

void spi_read(U8 data_length, U8 api_command )
{
	U8 i;

	U8 p[1];
	p[0] = api_command;
	i = 0;
	while(i!=0xff)
		i = check_cts();		// check cts

	spi_write(1, p);    //

	i = 0;
	while(i!=0xff)
		i = check_cts();	//check RTS if ready for reading data

	nSEL = 1;
	SCK = 0;
	nSEL = 0;
	spi_byte(0x44);
	for (i = 0; i< data_length; i++)	// data reading
		spi_read_buf[i] = spi_byte(0xff);
	nSEL = 1;
}


void spi_write_fifo(void)
{
	U8 i;

	i = 0;
	while(i!=0xff)
		i = check_cts();		//
	nSEL = 1;
	SCK = 0;
	nSEL = 0;
	spi_byte(WRITE_TX_FIFO);

	for (i = 0; i< payload_length; i++)
	{
		spi_byte(tx_ph_data[i]);
	}

	nSEL = 1;
}

void spi_read_fifo(void)
{
	U8 i;

	i = 0;
	while(i!=0xff)
		i = check_cts();		//

	nSEL = 1;
	SCK = 0;
	nSEL = 0;
	spi_byte(READ_RX_FIFO);
	for (i = 0; i< payload_length; i++)
		rx_buf[i] = spi_byte(0xff);   //
	nSEL = 1;
}

void sdn_reset(void)
{
	U8 i;

 	SDN = 1;
 	delay_1ms(2);	// RF module reset
 	SDN = 0;
 	delay_1ms(10);	// delay 10ms

 	nSEL = 1;
	SCK = 0;
	nSEL = 0;
	for (i = 0; i< 7; i++)
		spi_byte(RF_POWER_UP_data[i]);   // power up command
	nSEL = 1;

 	delay_1ms(20);	// delay 20ms ,RF module start working
}

void rf_init_freq(void)
{
	U8 app_command_buf[20];

	app_command_buf[0] = 0x11;
	app_command_buf[1]  = 0x20;    // 0x2051
	app_command_buf[2]  = 0x01;    //
	app_command_buf[3]  = 0x51;   // 0x2051
	app_command_buf[4]  = 0x0a;
    spi_write(5, app_command_buf);

	app_command_buf[0] = 0x11;
	app_command_buf[1]  = 0x40;    // 0x4000
	app_command_buf[2]  = 0x08;    //
	app_command_buf[3]  = 0x00;   // 0x4000
	app_command_buf[4]  = 0x38;   //  default value
	app_command_buf[5]  = 0x0e;   //  defaul value
	app_command_buf[6]  = 0x66;   //  default value
	app_command_buf[7]  = 0x66;   // frac ,from WDS
	app_command_buf[8]  = step_500K_step1;   // channel step1  from wds
	app_command_buf[9]  = step_500K_step0;   // channel step0  from wds
	app_command_buf[10] = 0x20;
	app_command_buf[11]  = 0xfe;
    spi_write(12, app_command_buf);
}
//system clock setup
void sysclk_cfg(void)
{
/* BARIS

    prc0 = 1;
	prc1 = 1;
	prc3 = 1;

	cm02 = 0;
	cm05 = 1;
	cm06 = 0;

	cm13 = 0;
	cm15 = 1;
	cm16 = 0;
	cm17 = 0;
	cm10 = 0;

	hra00 = 1;

	ocd0 = 0;
	ocd1 = 0;
	ocd2 = 1;
	ocd3 = 0;
	hra01 = 1;
	cm14 = 1;
	prc0 = 0;

 */

 }
//IO function set
void port_init(void)
{

    TRISA = 0;

    TRISB0 = 0; //SDN
    TRISB1 = 1; //NIRQ
    TRISB2 = 0; //NSEL
    TRISB3 = 0; //SCLK
    TRISB4 = 0; //SDI
    TRISB5 = 1; //SDO


    /* BARIS
	pd1 = 0b01111000;
	p1 = 0b00001000;
	pd3 = 0b10111000;
	p3 = 0b00101000;
	pd4 = 0b00100000;
	p4 = 0;
	pur0 = 0x04;
	pur1 = 0x00;
	drr = 0x00;
     * */
}

void delay_1ms(unsigned int delay_time)
{
	unsigned int i;
	while(delay_time !=0)
	{
		for (i =380; i!=0; i--)   // 340 for 7.3728MHz
		{
			asm("NOP");
			asm("NOP");
		}
		delay_time--;
	}
}

void delay_10ms(void)
{
	int i;

	for(i = 0; i<2472; i++)
	{
		;
	}
	// add watchdog

        //watchdog iptal
        //baris
	//wdtr = 0;
	//wdtr = 0xff;
	// add watchdog
}

//-------------------------------------------
void delay_x10ms(unsigned int dx10ms)
{
	unsigned int j;

	for(j = 0; j<dx10ms; j++)
		delay_10ms();
}

void nop_10()
{
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
}

void timerx_init(void)
{

//TIMER i kapattim
//	txmr = 0x00;
//	tcss = 0x01;			// f8
//	prex = 0xff;
//	tx = 129;				// 1/8/(255+1)(129+1) * 8M=30Hz
//	txic = 0x04;
//	txs = 1;
}

//BARIS #pragma INTERRUPT INT_TimerX
void INT_TimerX(void)
{
	rf_timeout++;
	if(rf_timeout == 20)
	{
		rf_timeout=0;
		Flag.rf_reach_timeout = 1;	// timer for tx timeout
	}

	count_1hz++;
    if(count_1hz==60)				//
    {
       	count_1hz=0;
       	Flag.reach_1s = 1;			//timer for 1s
       	//LED_BLUE  ^= 1;
    }
}

