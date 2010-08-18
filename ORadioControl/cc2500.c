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


prog_uint8_t APM cc2500InitValue[] =
{
  0x29,     //  9 IOCFG2     (Default) CHIP_RDY,
  0x2e,     //    IOCFG1     (Default) 3-state,
  0x6,      //  6 IOCFG0     Asserts when sync word has been sent / received,
            //               and de-asserts at the end of the packet. In RX,
            //               the pin will de-assert when the optional address,
            //               check fails or the RX FIFO overflows.,
            //               In TX the pin will de-assert if the TX FIFO underflows.,
  0x7,      //    FIFOTHR7   (Default) Tx:33 Rx:32,
  0xd3,     //    SYNC1      (Default) Sync word, high byte
  0x91,     //    SYNC0      (Default) Sync word, low byte
  0x2,      //    PKTLEN     Indicates the packet length when fixed length packets are enabled.,
            //               If variable length packets are used, this value indicates the
            //               maximum length packets allowed. (61 Bytes),
  0xC,      //    PKTCTRL1   (Default) jetzt autoflush
  0x44,     //  5 PKTCTRL0
  0x0,      //    ADDR       (Default) DEVICE_ADDR[7:0], wird nicht benutzt
  0x0,      //    CHANNR     CHAN[7:0],
  0x9,      //  6 FSCTRL1    Frequency synthesizer control, The desired IF frequency to employ in RX
  0x0,      //    FSCTRL0    (Default) Frequency synthesizer control,
  0x5c,     // 5c FREQ2      Frequency control word, high byte
  0x80,     // 80 FREQ1      Frequency control word, middle byte
  0x0,      //  0 FREQ0      Frequency control word, low byte (26MHz); unterster Kanal 2,405 GHz (2400-2483.5 MHz ISM/SRD band)
  0x5b,     // 5b MDMCFG4    Modem configuration, Datarate DRATE_E -> 0xb
  0xf8,     // f8 MDMCFG3    DRATE_M 0xf8 ergibt bei 26MHz -> 99975 Baud also 100kBaud
  0x3,      //  3 MDMCFG2    2-FSK, 30/32 sync word bits detected
  0x23,     // 23 MDMCFG1    4 preamble bytes, 2 bit exponent of channel spacing (3)
  0xf8,     //    MDMCFG0    (Default)channel spacing, 399902 Hz ergibt max  2,5069 GHz es sollten nur 205 Kanäle benutzt werden
  0x50,     // 50 DEVIATN    Modem deviation setting,
  0x7,      //    MCSM2      (Default)Main Radio Control State Machine configuration,
  0x0,      //  0 MCSM1      (Default)Main Radio Control State Machine configuration,
  0x18,     // 18 MCSM0      Main Radio Control State Machine configuration,
            //               Automatically calibrate When going from IDLE to RX or TX (or FSTXON)
            //               Programs the number of times the six-bit ripple counter mustexpire after XOSC has stabilized before CHP_RDYn goes low
            //               PO_TIMEOUT Approx. 149 – 155 μs  2 (10) 64
  0x16,     // 16 FOCCFG     (Default)Frequency Offset Compensation configuration,
  0x6c,     //    BSCFG      (Default) Bit Synchronization configuration,
  0x03,     // 67 AGCCTRL2   AGCCTRL2 – AGC control, (01) The highest gain setting can not be used
  0x58,     // fb AGCCTRL1   AGCCTRL1 – AGC control, (01) 6 dB increase in RSSI value; 8 (1000) Absolute carrier sense threshold disabled
  0x91,     // dc AGCCTRL0   (Default) AGCCTRL0 – AGC control,
  0x87,     //    WOREVT1    (Default) High byte Event0 timeout,
  0x6b,     //    WOREVT0    (Default) Low byte Event0 timeout,
  0xf8,     //    WORCTRL    (Default) Wake On Radio control,
  0x56,     //    FREND1     (Default) Front end RX configuration,
  0x10,     //    FREND0     (Default) Front end TX configuration,
  0x89,     //    FSCAL3     Frequency synthesizer calibration, keine Chargepump cal
  0x0a,     //    FSCAL2     (Default) Frequency synthesizer calibration,
  0x0,      //  0 FSCAL1     Frequency synthesizer calibration,
  0x11,     // 11 FSCAL0     Frequency synthesizer calibration,
  0x41,     //    RCCTRL1    (Default) RC oscillator configuration,
  0x0,      //    RCCTRL0    (Default) RC oscillator configuration,
  0x81,     //    TEST2
  0x35      //    TEST1
};

void cc2500CommandStrobe(uint8_t str)
{
  SPI_MasterTransmit(str);
}

uint8_t cc2500ReadStatus(uint8_t reg)
{
  SPI_MasterTransmit(reg | CC2500_READ_BURST);
  return(SPI_MasterTransmit(CC2500_SNOP));
}

void cc2500WriteReg(uint8_t reg, uint8_t c)
{
  SPI_MasterTransmit(reg & ~CC2500_READ_SINGLE);
  SPI_MasterTransmit(c);
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
  uint8_t temp = cc2500ReadStatus(CC2500_RXBYTES);
//  cc2500_Off();                      // Burstzugriff rücksetzen
  return(temp);
}

void cc2500FlushReceiveData(void)
{
  cc2500CommandStrobe(CC2500_SFRX);           // Flush the RX FIFO buffer
}

void setFrequencyOffset(void)
{
  int8_t freqoff, fsctrl;
  int16_t freq;

  freqoff = cc2500ReadStatus(CC2500_FREQEST);
//  cc2500_Off();
  if(freqoff)
  {
    fsctrl = cc2500ReadReg(CC2500_FSCTRL0);
    freq = freqoff + fsctrl;
    if(freq > 0x7f)
      freq = 0x7f;
    else if(freq < -0x80)
      freq = -0x80;
    cc2500WriteReg(CC2500_FSCTRL0, (int8_t)freq);
  }
}

uint8_t get_Data(void)
{
  SPI_MasterTransmit(CC2500_READ_SINGLE | CC2500_RXFIFO);
  return(SPI_MasterTransmit(CC2500_SNOP));
}

void cc2500ReadFIFOBlock(uint8_t *p, uint8_t n)
{
  cc2500ReadStatus(CC2500_RXFIFO);              // sieht unglücklich aus
  while(n--)
    *p++ = SPI_MasterTransmit(CC2500_SNOP);
  cc2500_Off();                      // Burstzugriff rücksetzen
}

void cc2500WriteFIFOBlock(uint8_t *p, uint8_t n)
{
  SPI_MasterTransmit(CC2500_WRITE_BURST | CC2500_TXFIFO);
  while(n--)
    SPI_MasterTransmit(*p++);
  cc2500_Off();                      // Burstzugriff rücksetzen
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
    while((SPI_MasterTransmit(CC2500_SNOP) & CC2500_STATUS_STATE_BM) != CC2500_STATE_IDLE);   // Status lesen
}

void cc2500StartCal(void)
{
  cc2500CommandStrobe(CC2500_SCAL);
}

void calibrateOff(void)
{
  cc2500WriteReg(CC2500_FSCAL3, pgm_read_byte(&cc2500InitValue[CC2500_FSCAL3]));
}

void calibrateOn(void)
{
  cc2500WriteReg(CC2500_FSCAL3, pgm_read_byte(&cc2500InitValue[CC2500_FSCAL3]) | 0x20);
}

