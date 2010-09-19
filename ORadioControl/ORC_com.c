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
  cc2500_Off();
//  SPSR = 0;
  /* Enable SPI, Master, set clock rate fck/4 */
  SPCR = (1<<SPE) | (1<<MSTR); // | (1<<SPR0);       //fck/16
}

void cc2500_Reset(void)
{
  cc2500CommandStrobe(CC2500_SRES);
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
  cc2500_Reset();
  cc2500_Off();                         // SS wegnehmen

  SPI_MasterTransmit(CC2500_IOCFG2 | CC2500_WRITE_BURST);
  do
  {
    SPI_MasterTransmit(pgm_read_byte(init++));
  }
  while(init < (cc2500InitValue + sizeof(cc2500InitValue) - 3));
  cc2500_Off();                    // SS wegnehmen wegen Burst
  SPI_MasterTransmit(CC2500_TEST2 | CC2500_WRITE_BURST);
  do
  {
    SPI_MasterTransmit(pgm_read_byte(init++));
  }
  while(init < (cc2500InitValue + sizeof(cc2500InitValue)));
  cc2500_Off();                    // SS wegnehmen wegen Burst
  cc2500setPatableMax(power);
}

bool checkcc2500(void)
{
  prog_uint8_t *init = cc2500InitValue;
  bool f = true;

  _delay_ms(100);
  SPI_MasterTransmit(CC2500_IOCFG2 | CC2500_READ_BURST);
  do
  {
    if(SPI_MasterTransmit(0) != pgm_read_byte(init++))
      f = false;
  }
  while(init < (cc2500InitValue + sizeof(cc2500InitValue) - 3));
  cc2500_Off();                    // SS wegnehmen wegen Burst
  SPI_MasterTransmit(CC2500_TEST2 | CC2500_READ_BURST);
  do
  {
    if(SPI_MasterTransmit(0) != pgm_read_byte(init++))
      f = false;
  }
  while(init < (cc2500InitValue + sizeof(cc2500InitValue)));
  cc2500_Off();                    // SS wegnehmen wegen Burst
  return f;
}
