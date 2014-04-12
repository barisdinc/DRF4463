/* 
 * File:   DRF4463.h
 * Author: baris
 *
 * Created on April 7, 2014, 10:03 PM
 */


#ifndef DRF4463_H_
#define DRF4463_H_

#include "radio_config_Si4463.h"
extern unsigned char global_err;

const unsigned char RF_CLR_INT_data[4];
const unsigned char RF_INT_STATUS_data;

unsigned char SPI_fregister(unsigned char);
void SPI_cts(void);
void SPI_command(const unsigned char *,unsigned char);
void SPI_response(unsigned char *,unsigned char);
void SPI_FIFO_write(unsigned char *,unsigned char);
void SPI_FIFO_read(unsigned char *,unsigned char);
void DRF4463_init(void);

#endif /* DRF4463_H_ */