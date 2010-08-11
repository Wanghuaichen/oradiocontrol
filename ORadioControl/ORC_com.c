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


#include "ORC.h"

void SPI_MasterInit(void)
{
  /* Set MOSI and SCK and SS output, all others input */
  CS_C2500_OFF
//  SPSR = 0;
  /* Enable SPI, Master, set clock rate fck/4 */
  SPCR = (1<<SPE) | (1<<MSTR);
}

void cc2500_Reset(void)
{
  SPI_MasterTransmit(CC2500_SRES);
  while(!(PINB & (1 << INP_B_SPI_MISO))) NOP();       // warten bis high
  while(PINB & (1 << INP_B_SPI_MISO)) NOP();          // warten bis low
}

void cc2500_Init(uint8_t power)
{
  prog_uint8_t *init = cc2500InitValue;

  SET_BIT(PORTB, OUT_B_SPI_SCK);
  RES_BIT(PORTB, OUT_B_SPI_MOSI);
  RES_BIT(PORTB, OUT_B_SPI_SS);
  _delay_us(40);                    // warten 40us
  SPI_MasterInit();
  _delay_us(40);                    // warten 40us
  CS_C2500_ACTIV
//  while(PINB & (1 << INP_B_SPI_MISO));          // warten bis low
  cc2500_Reset();
  cc2500_Off();                         // SS wegnehmen

  SPI_MasterTransmit(CC2500_IOCFG2 | CC2500_WRITE_BURST);
//  for(i = 0;i < sizeof(cc2500InitValue);++i)
  do
  {
    SPI_MasterTransmit(pgm_read_byte(init++));
  }
  while(init < (cc2500InitValue + sizeof(cc2500InitValue)));
  cc2500_Off();                    // SS wegnehmen wegen Burst
  SPI_MasterWriteReg(CC2500_TEST2, 0x81);
  SPI_MasterWriteReg(CC2500_TEST1, 0x35);
  cc2500setPatableMax(power);
}
