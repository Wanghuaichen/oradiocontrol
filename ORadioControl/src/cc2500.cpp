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
  0x29,     //  IOCFG2     (Default) CHIP_RDY,
  0x2e,     //  IOCFG1     (Default) 3-state,
  0x6,      //  IOCFG0     Asserts when sync word has been sent / received,
            //             and de-asserts at the end of the packet. In RX,
            //             the pin will de-assert when the optional address,
            //             check fails or the RX FIFO overflows.,
            //             In TX the pin will de-assert if the TX FIFO underflows.,
  0x7,      //  FIFOTHR7   Default Tx:33 Rx:32,
  0xd3,     //  SYNC1      Sync word, high byte
  0x91,     //  SYNC0      Sync word, low byte
  0x2,      //  PKTLEN     Indicates the packet length when fixed length packets are enabled.,
            //             If variable length packets are used, this value indicates the
            //             maximum length packets allowed. (61 Bytes),
  0xC,      //  PKTCTRL1   (Default) jetzt autoflush
  0x44,     //  PKTCTRL0
  0x0,      //  ADDR       (Default) DEVICE_ADDR[7:0], wird nicht benutzt
  0x0,      //  CHANNR     CHAN[7:0],
  0x9,      //  FSCTRL1    Frequency synthesizer control,
  0x0,      //  FSCTRL0    (Default) Frequency synthesizer control,
  0x5c,     //  FREQ2      Frequency control word, high byte
  0x80,     //  FREQ1      Frequency control word, middle byte
  0x0,      //  FREQ0      Frequency control word, low byte (26MHz); unterster Kanal 2,405 GHz (2400-2483.5 MHz ISM/SRD band)
  0x5b,     //  MDMCFG4    Modem configuration, Datarate DRATE_E -> 0xb
  0xf8,     //  MDMCFG3    DRATE_M 0xf8 ergibt bei 26MHz -> 99975 Baud also 100kBaud
  0x3,      //  MDMCFG2    2-FSK, 30/32 sync word bits detected
  0x23,     //  MDMCFG1    4 preamble bytes, 2 bit exponent of channel spacing (3)
  0xf8,     //  MDMCFG0    (Default)channel spacing, 399902 Hz ergibt max  2,5069 GHz es sollten nur 205 Kanäle benutzt werden
  0x50,     //  DEVIATN    Modem deviation setting,
  0x7,      //  MCSM2      Main Radio Control State Machine configuration,
  0x30,     //  MCSM1      Main Radio Control State Machine configuration,
  0x18,     //  MCSM0      Main Radio Control State Machine configuration,
            //             Automatically calibrate when going to RX or TX, or back to IDLE When going from IDLE to RX or TX (or FSTXON)
            //             Programs the number of times the six-bit ripple counter mustexpire after XOSC has stabilized before CHP_RDYn goes low
            //             Approx. 149 – 155 μs  2 (10) 64
  0x16,     //  FOCCFG     Frequency Offset Compensation configuration,
  0x6c,     //  BSCFG      (Default) Bit Synchronization configuration,
  0x43,     //  AGCCTRL2   AGCCTRL2 – AGC control, (01) The highest gain setting can not be used
  0x58,     //  AGCCTRL1   AGCCTRL1 – AGC control, (01) 6 dB increase in RSSI value; 8 (1000) Absolute carrier sense threshold disabled
  0x91,     //  AGCCTRL0   (Default) AGCCTRL0 – AGC control,
  0x87,     //  WOREVT1    (Default) High byte Event0 timeout,
  0x6b,     //  WOREVT0    (Default) Low byte Event0 timeout,
  0xf8,     //  WORCTRL    (Default) Wake On Radio control,
  0x56,     //  FREND1     (Default) Front end RX configuration,
  0x10,     //  FREND0     (Default) Front end TX configuration,
  0xa9,     //  FSCAL3     (Default) Frequency synthesizer calibration,
  0x0a,     //  FSCAL2     (Default) Frequency synthesizer calibration,
  0x0,      //  FSCAL1     Frequency synthesizer calibration,
  0x11,     //  FSCAL0     Frequency synthesizer calibration,
  0x41,     //  RCCTRL1    (Default) RC oscillator configuration,
  0x0,      //  RCCTRL0    (Default) RC oscillator configuration,
};


void SPI_MasterWriteReg(uint8_t reg, int8_t c)
{
  SPI_MasterTransmit(reg);
  SPI_MasterTransmit(c);
}

int8_t SPI_MasterReadReg(uint8_t reg)
{
  SPI_MasterTransmit(reg);
  return(SPI_MasterTransmit(CC2500_SNOP));
}


uint8_t get_RxCount(void)                   // Anzahl Bytes im FIFO
{
  SPI_MasterTransmit(CC2500_RXBYTES | CC2500_READ_BURST);
  return(SPI_MasterTransmit(0));
}

void cc2500FlushData(void)
{
  SPI_MasterTransmit(CC2500_SFRX);           // Flush the RX FIFO buffer
}

void cc2500_RxOn(void)
{
//  SPI_MasterWriteReg(CC2500_FSCTRL0, 0xf2);  // Korrektur schreiben
  cc2500FlushData();                          // Flush the RX FIFO buffer
  SPI_MasterTransmit(CC2500_SRX);            // Enable RX
  RES_BIT(EIFR, INTF0);
  SET_BIT(EIMSK, INT0);                       // INT0 ein
}

void setFrequencyOffset(void)
{
  int8_t freqoff, fsctrl;
  int16_t freq;

  freqoff = SPI_MasterReadReg(CC2500_FREQEST | CC2500_READ_BURST);
  if(freqoff)
  {
    fsctrl = SPI_MasterReadReg(CC2500_FSCTRL0 | CC2500_READ_SINGLE);
    freq = freqoff + fsctrl;
    if(freq > 0x7f)
      freq = 0x7f;
    else if(freq < -0x80)
      freq = -0x80;
    SPI_MasterWriteReg(CC2500_FSCTRL0, (int8_t)freq);
  }
}

uint8_t get_Data(void)
{
  SPI_MasterTransmit(0xbf);
  return(SPI_MasterTransmit(CC2500_SNOP));
}

void cc2500ReadBlock(int8_t *p, uint8_t n)
{
  while(n--)
    *p++ = SPI_MasterTransmit(CC2500_SNOP);
}

void cc2500WriteBlock(int8_t *p, uint8_t n)
{
  while(n--)
    SPI_MasterTransmit(*p++);
}

void cc2500WriteSingle(int8_t *p, uint8_t n)
{
  while(n--)
  {
    SPI_MasterTransmit(CC2500_TXFIFO);
    SPI_MasterTransmit(*p++);
  }
}

