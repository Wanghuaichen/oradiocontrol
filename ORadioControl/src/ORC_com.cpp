/*
 * Author Josef Glatthaar <josef.glatthaar@googlemail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#define OUT_B_SPI_SS PORTB2
#define OUT_B_SPI_MOSI PORTB3
#define INP_B_SPI_MISO PORTB4
#define OUT_B_SPI_SCK PORTB5
#define INP_D_CC2500_GDO0 PORTD2
#define INP_D_CC2500_GDO2 PORTD3


#include "ORC_tx.h"

void SPI_MasterInit(void)
{
  /* Set MOSI and SCK and SS output, all others input */
  CS_C2500_OFF
//  SPSR = 0;
  /* Enable SPI, Master, set clock rate fck/4 */
  SPCR = (1<<SPE) | (1<<MSTR);
}

bool spi_wait;

void SPI_MasterTransmit_void(uint8_t cData)
{
  CS_C2500_ACTIV
  WAIT_C2500_READY                                 // warten bis bereit
  spi_wait = true;
//  if(spi_wait)
//    WAIT_SPI_READY                                  /* Wait for transmission complete */
  /* Start transmission */
  do
    SPDR = cData;
  while(SPSR & (1 << WCOL));

}

uint8_t SPI_MasterTransmit(uint8_t cData)
{
  CS_C2500_ACTIV
  WAIT_C2500_READY                                // warten bis bereit
//  if(spi_wait)
//    WAIT_SPI_READY                                  /* Wait for transmission complete */
  /* Start transmission */
  do
    SPDR = cData;
  while(SPSR & (1 << WCOL));
  spi_wait = false;
  WAIT_SPI_READY                                  /* Wait for transmission complete */
  return(SPDR);
}

void cc2500_Off(void)
{
  if(spi_wait)
  {
    spi_wait = false;
    WAIT_SPI_READY                                  /* Wait for transmission complete */
  }
  CS_C2500_OFF
}

void cc2500_Init(void)
{
  prog_uint8_t *init = cc2500InitValue;

  SPI_MasterTransmit_void(CC2500_SRES);
  cc2500_Off();                         // SS wegnehmen
  _delay_us(40);                    // warten 40us

  SPI_MasterTransmit_void(CC2500_IOCFG2 | CC2500_WRITE_BURST);
//  for(i = 0;i < sizeof(cc2500InitValue);++i)
  do
  {
    SPI_MasterTransmit_void(pgm_read_byte(init++));
  }
  while(init < (cc2500InitValue + sizeof(cc2500InitValue)));
  cc2500_Off();                    // SS wegnehmen
  _delay_us(40);                    // warten 40us wegen SS
  SPI_MasterWriteReg(CC2500_TEST2, 0x81);
  SPI_MasterWriteReg(CC2500_TEST1, 0x35);
  cc2500setPatableMax();
}
