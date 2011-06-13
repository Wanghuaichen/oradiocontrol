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
#include "SMARTRF_SETTING.h"

prog_uint8_t APM cc2500InitValue[] =
{
  SMARTRF_SETTING_IOCFG2,       //  9 IOCFG2     (Default) CHIP_RDY,
  SMARTRF_SETTING_IOCFG1,       //    IOCFG1     (Default) 3-state,
  SMARTRF_SETTING_IOCFG0,       //  6 IOCFG0     Asserts when sync word has been sent / received,
  SMARTRF_SETTING_FIFOTHR,      //    FIFOTHR7   (Default) Tx:33 Rx:32,
  SMARTRF_SETTING_SYNC1,        //    SYNC1      (Default) Sync word, high byte
  SMARTRF_SETTING_SYNC0,        //    SYNC0      (Default) Sync word, low byte
  SMARTRF_SETTING_PKTLEN,       //    PKTLEN     Indicates the packet length when fixed length packets are enabled.,
  SMARTRF_SETTING_PKTCTRL1,     //    PKTCTRL1   (Default) jetzt autoflush
  SMARTRF_SETTING_PKTCTRL0,     //  5 PKTCTRL0
  SMARTRF_SETTING_ADDR,         //    ADDR       (Default) DEVICE_ADDR[7:0], wird nicht benutzt
  SMARTRF_SETTING_CHANNR,       //    CHANNR     CHAN[7:0],
  SMARTRF_SETTING_FSCTRL1,      //  6 FSCTRL1    Frequency synthesizer control, The desired IF frequency to employ in RX
  SMARTRF_SETTING_FSCTRL0,      //    FSCTRL0    (Default) Frequency synthesizer control,
  SMARTRF_SETTING_FREQ2,        // 5c FREQ2      Frequency control word, high byte
  SMARTRF_SETTING_FREQ1,        // 80 FREQ1      Frequency control word, middle byte
  SMARTRF_SETTING_FREQ0,        //  0 FREQ0      Frequency control word, low byte (26MHz); unterster Kanal 2,405 GHz (2400-2483.5 MHz ISM/SRD band)
  SMARTRF_SETTING_MDMCFG4,      // 5b MDMCFG4    Modem configuration, Datarate DRATE_E -> 0xb
  SMARTRF_SETTING_MDMCFG3,      // f8 MDMCFG3    DRATE_M 0xf8 ergibt bei 26MHz -> 99975 Baud also 100kBaud
  SMARTRF_SETTING_MDMCFG2,      //  3 MDMCFG2    2-FSK, 30/32 sync word bits detected
  SMARTRF_SETTING_MDMCFG1,      // 23 MDMCFG1    4 preamble bytes, 2 bit exponent of channel spacing (3)
  SMARTRF_SETTING_MDMCFG0,      //    MDMCFG0    (Default)channel spacing, 399902 Hz ergibt max  2,5069 GHz es sollten nur 205 Kanäle benutzt werden
  SMARTRF_SETTING_DEVIATN,      // 50 DEVIATN    Modem deviation setting,
  SMARTRF_SETTING_MCSM2,        //    MCSM2      (Default)Main Radio Control State Machine configuration,
  SMARTRF_SETTING_MCSM1,        //  0 MCSM1      (Default)Main Radio Control State Machine configuration,
  SMARTRF_SETTING_MCSM0,        // 18 MCSM0      Main Radio Control State Machine configuration,
  SMARTRF_SETTING_FOCCFG,       // 16 FOCCFG     Frequency Offset Compensation configuration,
  SMARTRF_SETTING_BSCFG,        //    BSCFG      (Default) Bit Synchronization configuration,
  SMARTRF_SETTING_AGCCTRL2,     // 67 AGCCTRL2   AGCCTRL2 – AGC control, (01) The highest gain setting can not be used   xxxx
  SMARTRF_SETTING_AGCCTRL1,     // fb AGCCTRL1   AGCCTRL1 – AGC control, (01) 6 dB increase in RSSI value; 8 (1000) Absolute carrier sense threshold disabled
  SMARTRF_SETTING_AGCCTRL0,     // dc AGCCTRL0   (Default) AGCCTRL0 – AGC control,
  SMARTRF_SETTING_WOREVT1,      //    (Default) High byte Event0 timeout,
  SMARTRF_SETTING_WOREVT0,      //    (Default) Low byte Event0 timeout,
  SMARTRF_SETTING_WORCTRL,      //    (Default) Wake On Radio control,
  SMARTRF_SETTING_FREND1,       //    FREND1     (Default) Front end RX configuration,
  SMARTRF_SETTING_FREND0,       //    FREND0     (Default) Front end TX configuration,
  SMARTRF_SETTING_FSCAL3,       //    FSCAL3     Frequency synthesizer calibration
  SMARTRF_SETTING_FSCAL2,       //    FSCAL2     (Default) Frequency synthesizer calibration,
  SMARTRF_SETTING_FSCAL1,       //  0 FSCAL1     Frequency synthesizer calibration,
  SMARTRF_SETTING_FSCAL0,       // 11 FSCAL0     Frequency synthesizer calibration,
  SMARTRF_SETTING_RCCTRL1,      //    (Default) RC oscillator configuration,
  SMARTRF_SETTING_RCCTRL0,      //    (Default) RC oscillator configuration,
  SMARTRF_SETTING_FSTEST,       //              0x29
  SMARTRF_SETTING_TEST2,        //    TEST2  2c
  SMARTRF_SETTING_TEST1,        //    TEST1  2d
  SMARTRF_SETTING_TEST0         //    TEST0  2e
};

void cc2500CommandStrobe(uint8_t str)
{
  SPI_MasterTransmit(str);
}

uint8_t cc2500GetState(void)
{
  return(SPI_MasterTransmit(CC2500_SNOP) & CC2500_STATUS_STATE_BM);
}

uint8_t cc2500ReadStatusReg(uint8_t reg)
{
  SPI_MasterTransmit(reg | CC2500_READ_BURST);
  return(SPI_MasterTransmit(CC2500_SNOP));
}

uint8_t cc2500WriteReg(uint8_t reg, uint8_t c)
{
  SPI_MasterTransmit(reg & ~CC2500_READ_SINGLE);
  return(SPI_MasterTransmit(c));
}

void cc2500WriteRegCheckIdle(uint8_t reg, uint8_t c)
{
  if((cc2500WriteReg(reg, c) & CC2500_STATUS_STATE_BM) != CC2500_STATE_IDLE)
      cc2500Idle();
}

uint8_t cc2500ReadReg(uint8_t reg)
{
  SPI_MasterTransmit(reg | CC2500_READ_SINGLE);
  return(SPI_MasterTransmit(CC2500_SNOP));
}

uint8_t get_RxCount(void)                   // Anzahl Bytes im FIFO
{
  // Bei kleinen Werten auch über lesenden NOP möglich
//  uint8_t temp = SPI_MasterTransmit(CC2500_SNOP | CC2500_READ_SINGLE) & CC2500_STATUS_FIFO_BYTES_AVAILABLE_BM;
  uint8_t temp = cc2500ReadStatusReg(CC2500_RXBYTES);
//  cc2500_Off();                      // Burstzugriff rücksetzen
  return(temp);
}

void cc2500FlushReceiveData(void)
{
  cc2500CommandStrobe(CC2500_SFRX);           // Flush the RX FIFO buffer
}

uint8_t get_Data(void)
{
  SPI_MasterTransmit(CC2500_READ_SINGLE | CC2500_RXFIFO);
  return(SPI_MasterTransmit(CC2500_SNOP));
}

bool cc2500ReadFIFOBlock(uint8_t *p, uint8_t n)
{
  if(n != (SPI_MasterTransmit(CC2500_RXFIFO | CC2500_READ_BURST) & CC2500_STATUS_FIFO_BYTES_AVAILABLE_BM))
  {
    cc2500_Off();                      // Burstzugriff rücksetzen
    return(false);
  }
  while(n--)
    *p++ = SPI_MasterTransmit(CC2500_SNOP);
  cc2500_Off();                      // Burstzugriff rücksetzen
  return(true);
}

uint8_t cc2500WriteFIFOBlock(uint8_t *p, uint8_t n)
{
  uint8_t ret = 0;
  SPI_MasterTransmit(CC2500_WRITE_BURST | CC2500_TXFIFO);
  while(n--)
    ret = SPI_MasterTransmit(*p++);
  cc2500_Off();                      // Burstzugriff rücksetzen
  return(ret);
}

void cc2500WriteSingle(uint8_t *p, uint8_t n)
{
  while(n--)
  {
    SPI_MasterTransmit(CC2500_TXFIFO);
    SPI_MasterTransmit(*p++);
  }
}

void cc2500ReadSingle(uint8_t *p, uint8_t n)
{
  while(n--)
    *p++ =cc2500ReadReg(CC2500_RXFIFO);
}

void cc2500setPatableMax(uint8_t power)
{
  uint8_t i;
  SPI_MasterTransmit(CC2500_PATABLE | CC2500_WRITE_BURST);
  for(i = 0; i < 8; ++i)                 //// 8!!
    SPI_MasterTransmit(power);
  cc2500_Off();
}

void cc2500Idle(void)
{
  if((SPI_MasterTransmit(CC2500_SIDLE) & CC2500_STATUS_STATE_BM) != CC2500_STATE_IDLE)
    while(cc2500GetState() != CC2500_STATE_IDLE);   // Status lesen
}

uint8_t cc2500IdleGetRXB(void)
{
  uint8_t temp;
  if(((temp = SPI_MasterTransmit(CC2500_SIDLE | CC2500_READ_SINGLE)) & CC2500_STATUS_STATE_BM) != CC2500_STATE_IDLE)
    while(cc2500GetState() != CC2500_STATE_IDLE);   // Status lesen
  return(temp & CC2500_STATUS_FIFO_BYTES_AVAILABLE_BM);
}

void cc2500StartCal(void)
{
  cc2500CommandStrobe(CC2500_SCAL);
}

void calibrateFast(void)	                   //Disable charge pump calibration stage when 0
{

  cc2500WriteRegCheckIdle(CC2500_FSCAL3, cc2500ReadReg(CC2500_FSCAL3) & ~0x20);
}

void calibrateSlow(void)
{
  cc2500WriteRegCheckIdle(CC2500_FSCAL3, (cc2500ReadReg(CC2500_FSCAL3) & ~0x20) | 0x20);
}

void gotoIdle(void)
{
  cc2500WriteRegCheckIdle(CC2500_MCSM1, pgm_read_byte(&cc2500InitValue[CC2500_MCSM1]) & ~0xc);
}

void stayInRx(void)
{
  cc2500WriteRegCheckIdle(CC2500_MCSM1, pgm_read_byte(&cc2500InitValue[CC2500_MCSM1]) | 0xc);
}
