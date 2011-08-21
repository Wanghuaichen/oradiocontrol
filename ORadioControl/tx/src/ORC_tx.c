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
 *
 *
 * Compiler: avr-gcc (GCC) 4.3.4
 *
 * Version 0.1 :First connect
 * Version 0.5 :range check (over 1,5 km)
 * Version 1.0 :first fly
 * Version 1.1 :no stack
 *
 * Sender läuft im festen Raster von 8 * 2,144 ms für die Datenübertragung zum Empfänger
 * und 1 * 3 ms für die Telemetrie vom Empfänger zum Sender = 20,15 ms Zykluszeit bei PPM
 * Bei UART- Input kann Zykluszeit auch 10,05 ms betragen. Weniger macht wahrscheinlich kein Sinn.
 * Die zu übertragenden Känale sind in Gruppen zusammengefasst. In einem einzelnen Telegramm
 * mit 2,144 ms werden jeweils 4 Kanäle (eine Gruppe) übertragen. Insgesamt sind 16 Kanäle vorgesehen.
 * Gruppe 1 Kanal 1-4, Gruppe 2 Kanal 5-8, Gruppe 3 Kanal 9-12, Gruppe 4 Kanal 13-16
 * Die Gruppen können in beliebiger Reihenfolge übertragen werden.
 * Bei PPM- Input wird automatisch auf die erste Gruppe gewechselt, sobald die ersten 4 Kanäle
 * digitalisiert sind. Dies wird auch im Datenstrom kodiert um auf Empfangsseite die Ausgabe zu
 * triggern.
 * Dann folgt die 2 Gruppe nach 2,144 ms. Hier werden aber zum Teil alte Daten übertragen. Durch
 * abwarten von 10 ms im Empfänger für die 2. Gruppe sind diese dann aber aktuell für die Ausgabe.
 * Nach max. 8 ms seit der ersten Übertragung der Gruppe 1, ist die Gruppe 2 aktuell.
 * Gruppe 3 und 4 werden nur übertragen, wenn sie auch in den PPM oder UART- Daten vorhanden sind.
 * Der Master für das Timing ist immer das Sendemodul, weil ansonsten der Empfänger nicht
 * synchronisieren kann.
 * Der Sender springt bei jeder Übertragung auf eine neue Frequenz, bei Telemetrie nicht
 * Die Übertragung vor der Telemetrie und die Telemetrie selber sind auf der gleichen Frequenz,
 * damit es vom Empfänger her gesehen keine Lücke in der Kanalbelegung gibt.
 * Sendetelegramm: 6 Byte Daten + Syncword 4 Byte + preamble 4 Byte + CRC 2 Byte = 16 Byte
 * 240µs blanking time = min 67,2 kBaud
 * mit FEC (6 Byte Daten + Syncword 4 Byte + CRC 2 Byte) * 2 = 24 Byte + 24 preamble = 48 Byte
 * 160µs für RSSI
 * preample auf 24
 * für FEC = 1,936 ms

Probleme:
bugs:
todo:
done:
Bitfelder entfernt, weil EEPROM- Speicher nicht kritisch, Flash ist wichtiger
Kein Stack mehr erforderlich

 */

//PIN 1   PD3   INT1/OC2B/PCINT19                                       GDO2          I
//PIN 2   PD4   XCK/T0/PCINT20                                          EEPROM        I
//PIN 3   GND                                                           -
//PIN 4   VCC                                                           -
//PIN 5   GND                                                           -
//PIN 6   VCC                                                           -
//PIN 7   PB6   OSC                                                     8 MHz
//PIN 8   PB7   OSC                                                     8 MHz
//PIN 9   PD5   T1/OC0B/PCINT21                                         CRX  LOW      O
//PIN 10  PD6   AIN0/OC0A/PCINT22
//PIN 11  PD7   AIN1/OC2B/PCINT23
//PIN 12  PB0   ICP1/CLKO/PCINT0                                        PPM LOWAKTIV  I
//PIN 13  PB1   OC1A/PCINT1                                             CTX  HIGH     O
//PIN 14  PB2   SS/OC1B/PCINT2 (SPI Bus Master Slave select)            CC2500        O
//PIN 15  PB3   MOSI/OC2/PCINT3 (SPI Bus Master Output/Slave Input)     CC2500        O
//PIN 16  PB4   MISO/PCINT4 (SPI Bus Master Input/Slave Output)         CC2500        I
//PIN 17  PB5   SCK/PCINT5 (SPI Bus Master clock Input)                 CC2500        O
//PIN 18  AVCC
//PIN 19  ADC6                                                          R
//PIN 20  AREF
//PIN 21  GND
//PIN 22  ADC7
//PIN 23  PC0   ADC0/PCINT8                                             LED2          O
//PIN 24  PC1   ADC1/PCINT9                                             LED1          O
//PIN 25  PC2   ADC2/PCINT10                                            TASTER        I
//PIN 26  PC3   ADC3/PCINT11
//PIN 27  PC4   SDA/ADC4/PCINT12                                        EEPROM        I
//PIN 28  PC5   SCL/ADC5/PCINT13                                        EEPROM        I
//PIN 29  PC6   RESET/PCINT14                                           RESET         I
//PIN 30  PD0   RXD/PCINT16                                             PINLEISTE     I
//PIN 31  PD1   TXD/PCINT17                                             PINLEISTE     O
//PIN 32  PD2   INT0/PCINT18                                            GDO0          I

//#define OUT_B_SPI_SS PORTB2
//#define OUT_B_SPI_MOSI PORTB3
//#define INP_B_SPI_MISO PINB4
//#define OUT_B_SPI_SCK PORTB5
#define INP_B_PPM PINB0
#define OUT_B_CTX PORTB1

#define OUT_C_LEDRED PORTC0
#define OUT_C_LEDGREEN PORTC1
#define INP_C_KEY PINC2

//#define INP_D_CC2500_GDO0 PIND2
//#define INP_D_CC2500_GDO2 PIND3
#define OUT_D_CRX PORTD5
#define INP_D_RXD PORTD0

#define LEDGREEN_OFF RES_BIT(PORTC, OUT_C_LEDGREEN)
#define LEDGREEN_ON SET_BIT(PORTC, OUT_C_LEDGREEN)
#define LEDRED_OFF RES_BIT(PORTC, OUT_C_LEDRED)
#define LEDRED_ON SET_BIT(PORTC, OUT_C_LEDRED)

#define FOSC 8000000 // Clock Speed
#define BAUD 250000
#define BRATE FOSC/16/BAUD-1

//#define DEBUG

#include "ORC_tx.h"
#include "../../SMARTRF_SETTING.h"
#include <avr/boot.h>

/* EEMEM */ EEData eeprom;
volatile State state;
volatile OutputData output;
volatile uint16_t Timer33ms;
bool InterruptAct;
bool Heartbeat;
uint16_t Telem_error;
TelemetrieReceive TelemetrieMes;
MessageData sendMes;

uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
void txState(void);
void setNextChanRx(void);
void setRx(void);
void setNext(void);
void readWriteMemory(void);
void flashProgram(void);
bool checkKey(void);

void get_mcusr(void) \
  __attribute__((naked)) \
  __attribute__((section(".init3")));
void get_mcusr(void)
{
  wdt_reset();
  mcusr_mirror = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

ISR(TIMER2_COMPA_vect)                 
{                                                // Wenn außerhalb Interrupt auch spi läuft krachts
  InterruptAct = true;
  txState();
  cc2500_Off();
  Heartbeat = true;
}

//uint8_t tes;
ISR(INT0_vect, ISR_NOBLOCK)                // Wird ausgelöst wenn Senden fertig
{
  RES_BIT(EIMSK, INT0);                    // INT0 aus
  RES_BIT(TIMSK2, OCIE2A);                 // Timerinterrupt aus
  if(InterruptAct)                         // Timerinterrupt ist noch aktiv
  {
    SET_BIT(state.ledError, L_NOT_RX);     // Ursache kann Capture sein
    return;
  }

//  if(tes < TCNT2)
//    tes = TCNT2;
  //  if(TCNT2 > CYCLETIME + 10)
  //    OCR2A = TCNT2 + 2;
//  RES_BIT(PORTB, OUT_B_CTX);
//  SET_BIT(PORTD, OUT_D_CRX);
  setNext();
  if(TIFR2 & (1 << OCF2A))
    SET_BIT(state.ledError, L_NOT_RX);     // Ursache kann Capture sein
  cli();
  SET_BIT(TIMSK2, OCIE2A);                 // Timerinterrupt ein
}

ISR(TIMER1_CAPT_vect, ISR_NOBLOCK)                  //8MHz capture
{                                               // Timer läuft immer im Kreis
  static uint16_t capture_alt;
  static uint8_t chanPtr;
  uint8_t chanPtrtemp = chanPtr;

  uint16_t capture = ICR1;
  if(TIFR1 & (1 << OCF1A))              // Neuer PPM- Frame (>3ms Pause)
  {
    state.maxChan = chanPtrtemp;
    chanPtrtemp = 0;
  }
  else if(chanPtrtemp < 16)
  {                                                     // Auf 16 Kanäle begrenzen
    output.chan_1us[chanPtrtemp] = (capture - capture_alt) / 8 - 1500u;
    if(chanPtrtemp == 3)
      output.ppmSync = 4;
    ++chanPtrtemp;
  }	
  cli();
  OCR1A = TCNT1 + (FOSC*100 / 33333); 			// 3ms
  sei();
//  SET_BIT(TIFR1, OCF1A);
  TIFR1 = (1 << OCF1A);
  capture_alt = capture;
  chanPtr = chanPtrtemp;
}

// 32 Takte für Senden bei 250000 Baud -> Interrupt lohnt nicht, weil mehr als 32 Takte lang
//ISR(USART_UDRE_vect)
//{
//  RES_BIT(UCSR0B, UDRIE0);                // USART- Interrupt aus
//  sei();
//  UDR0 = uartTxBuf[uartRead++];
//  if(uartRead < sizeof(uartTxBuf))        // Noch Daten da
//    SET_BIT(UCSR0B, UDRIE0);              // USART- Interrupt ein
//}

ISR(USART_RX_vect)
{
  static uint8_t chantemp, msbdata, i;
  static enum uart rxstate;

  RES_BIT(UCSR0B, RXCIE0);                      // Interrupt aus
  sei();
  uint8_t rxbuf = UDR0;
  OCR0A = TCNT0 + 10;                           // Timeout 1,28 ms
  if(TIFR0 & (1 << OCF0A))
  {
    rxstate = WaitToken;
//    SET_BIT(TIFR0, OCF0A);
    TIFR0 = (1 << OCF0A);
  }
  switch(rxstate)
  {
  case WaitToken:
    if(rxbuf == 'C')
      rxstate = ReadChan;
    else if(rxbuf == 'T')
      rxstate = ReadCommandMSB;
    else if((rxbuf == 'F') && checkKey())
    {
      wdt_disable();                // macht auch cli()
      flashProgram();
    }
    break;
  case ReadChan:
    UDR0 = 'A';
    if(rxbuf & ~0x7)
    {
      i = 2;
      rxstate = ReadDummy;
    }
    else
    {
      if(rxbuf < chantemp)
        state.maxChan = chantemp + 1;           // Bedingt, dass die Kanäle aufsteigend reinkommen
      chantemp = rxbuf;
      rxstate = ReadMSB;
    }
    break;
  case ReadMSB:
    msbdata = rxbuf;
    rxstate = ReadLSB;
    break;
  case ReadLSB:
    UDR0 = 'A';
    output.chan_1us[chantemp] = (msbdata << 8) + rxbuf;
    if(chantemp == 3)
      output.ppmSync = 4;
    rxstate = WaitToken;
    break;
  case ReadCommandMSB:
    if(state.newCommand)        // Altes Kommando ist noch nicht weg
    {
      i = 4;
      rxstate = ReadDummy;
    }
    else
    {
      sendMes.Uart.b0 = rxbuf;
      i = 0;
      rxstate = ReadCommand;
    }
    break;
  case ReadCommand:
    if(!(i & 1))
      UDR0 = 'A';
    sendMes.Uart.b[i++] = rxbuf;
    if(i > 4)
    {
      if((sendMes.MemoryWord.type == 5) && (sendMes.MemoryWord.des == 0))
        readWriteMemory();
      else
        state.newCommand = true;
      rxstate = WaitToken;
    }
    break;
  case ReadDummy:
    --i;
    if(!i)
      rxstate = WaitToken;
    break;
  }
  cli();
  SET_BIT(UCSR0B, RXCIE0);                      // Interrupt ein
}

void readWriteMemory(void)
{
  if(sendMes.MemoryWord.tar == 0)               // eeprom
  {
    if(sendMes.MemoryWord.wr)                   // schreiben
    {
      if(sendMes.MemoryWord.size)
        eeprom_write_word(sendMes.MemoryWord.adr, sendMes.MemoryWord.data);
      else
        eeprom_write_byte(sendMes.MemoryByte.adr, sendMes.MemoryByte.data);
    }
  }
//    else
//      ;
//  }
//  else if(sendMes.MemoryWord.tar == 1)          // ram
//    ;
//  else if(sendMes.MemoryWord.tar == 2)          // flash
//    ;
}

uint8_t SPI_MasterTransmit(uint8_t cData)
{
  uint8_t i = 0;
  if(PORTB & (1 << OUT_B_SPI_SS))
  {
    RES_BIT(PORTB, OUT_B_SPI_SS);
    NOP();
    while(PINB & (1<<OUT_B_SPI_SS))
      if(++i > 0xfe)
      {
        SET_BIT(state.ledError, L_SPI_ERROR);
        break;
      }
    NOP();
    i = 0;
    while(PINB & (1 << INP_B_SPI_MISO))
      if(++i > 0xfe)
      {
        SET_BIT(state.ledError, L_SPI_ERROR);
        break;
      }
  }
  i = 0;
  while(PIND & (1<<INP_D_CC2500_GDO2))
    if(++i > 0xfe)
    {
      SET_BIT(state.ledError, L_SPI_ERROR);
      break;
    }
  if(SPSR & (1<<SPIF))
    i = SPDR;
  do
    SPDR = cData;
  while(SPSR & (1 << WCOL));
  while(!(SPSR & (1<<SPIF))) NOP();                            /* Wait for transmission complete */
  return(SPDR);
}

void cc2500_Off(void)
{
  SET_BIT(PORTB, OUT_B_SPI_SS);
  do
  {
    NOP();
  }
  while(!(PINB & (1<<OUT_B_SPI_SS)));
}

void setBindMode(void)
{
  cc2500WriteReg(CC2500_SYNC0, (unsigned char)BINDMODEID);
  cc2500WriteReg(CC2500_SYNC1, (unsigned char)(BINDMODEID >> 8));
  cc2500WriteReg(CC2500_PKTLEN, sizeof(eeprom.bind));
  state.bindmode = true;
  cc2500setPatableMax(0x7f);             // Sendeleistung runter
}

void setNextChan(void)                // Kanal schreiben
{
  uint16_t tempChan = state.actTxChan;
  if(state.bindmode)
    tempChan += BINDMODESTEP * 2 + 1;
  else
  tempChan += eeprom.bind.step * 2 + 1;
  while(tempChan > MAXHOPPCHAN)
    tempChan -= (MAXHOPPCHAN + 1);
  state.actTxChan = tempChan;
  cc2500WriteRegCheckIdle(CC2500_CHANNR, tempChan);
}

void setRx(void)                      // nach RX
{
  RES_BIT(PORTB, OUT_B_CTX);
  SET_BIT(PORTD, OUT_D_CRX);		      // Auf Empfang umschalten
  RES_BIT(EIMSK, INT0);                    // INT0 aus
  if(PIND & (1<<INP_D_CC2500_GDO0))
    SET_BIT(state.ledError, L_NOT_READY);
  cc2500CommandStrobe(CC2500_SFTX);       // Flush TX
  cc2500CommandStrobe(CC2500_SRX);
}

void setNextChanRx(void)                      // Kanal schreiben und nach RX
{
  RES_BIT(PORTB, OUT_B_CTX);
  SET_BIT(PORTD, OUT_D_CRX);		      // Auf Empfang umschalten
  RES_BIT(EIMSK, INT0);                    // INT0 aus
  if(PIND & (1<<INP_D_CC2500_GDO0))
    SET_BIT(state.ledError, L_NOT_READY);
  setNextChan();

  if(!eeprom.ccaOff && !state.ccaCount)
    cc2500WriteReg(CC2500_MCSM1, (SMARTRF_SETTING_MCSM1 | 0x10) & ~0x20);  // ein

  cc2500CommandStrobe(CC2500_SFTX);       // Flush TX
  cc2500CommandStrobe(CC2500_SRX);
}

//void setNextChanRx(void)                      // Kanal schreiben und nach Rx
//{
//  setNextChan();
//  cc2500WriteReg(CC2500_PKTLEN, sizeof(Telemetrie));
//  SPI_MasterTransmit_void(CC2500_SRX);
//  RES_BIT(PORTB, OUT_B_CTX);
//  SET_BIT(PORTD, OUT_D_CRX);
//}

//void setPaketsizeSend(void)
//{
//  cc2500WriteRegCheckIdle(CC2500_PKTLEN, sizeof(MessageData));
//}
//
//void setPaketsizeReceive(void)
//{
//  cc2500WriteRegCheckIdle(CC2500_PKTLEN, sizeof(Telemetrie));
//}

bool checkKey(void)
{
  return(!(PINC & (1 << INP_C_KEY)));
}

void set_led(void)
{
  static uint8_t timer_alt;
  static uint8_t led_count;
  uint8_t ledErrorTemp;

  int8_t diff = ((uint8_t)Timer33ms) - timer_alt;
  if(diff > (1000 / 4 / 33))
  {
    timer_alt = (uint8_t)Timer33ms;

    if(!(led_count & 0xf))            // unteres Nibble 0
    {
      ledErrorTemp = state.ledError;
      while(ledErrorTemp)
      {
        led_count += 0x10;
        led_count &= 0x7f;
        if(ledErrorTemp & (1 << (led_count >> 4)))
        {
          led_count |= (led_count >> 3) | 1;      // Blinkzähler setzen
          LEDRED_ON;
          break;
        }
      }
    }
    else
    {
      --led_count;
      if(led_count & 1)
        LEDRED_ON;
      else
        LEDRED_OFF;
      if(!(led_count & 0xf))
        timer_alt += (1000 / 4 / 33 * 3);                    // Pause
    }
  }
}

bool checkId(void)
{
  return (eeprom.bind.id && (eeprom.bind.id != 0xffff)
      && ((eeprom.bind.id >> 8) != (eeprom.bind.id & 0xff))
      && ((eeprom.bind.id >> 8) != 0xaa)
      && ((eeprom.bind.id >> 8) != 0x55)
      && (eeprom.bind.step > 4) && (eeprom.bind.step < MAXHOPPCHAN / 2 - 4));
}

uint16_t calcCheckSum(uint8_t *p, uint16_t size)
{
  uint16_t sum = 0;

  while(size--)
    sum += *p++;
  return(sum);
}

void calcNewId(void)
{
  do
  {
    cli();
    eeprom.bind.id += TCNT0 + TCNT1 + TCNT2 + BINDMODEID;
    sei();
    eeprom.bind.step += eeprom.bind.id;
    eeprom.bind.step &= 0x3f;
  }
  while(!checkId());
  eeprom.checksum = calcCheckSum((uint8_t *)&eeprom, sizeof(eeprom) - sizeof(eeprom.checksum));
  eeprom_write_block(&eeprom, 0, sizeof(eeprom));
  SET_BIT(state.ledError, L_EEPROM_ERR);
}

void setNext(void)
{
  if(!state.txCount)
  {
    calibrateSlow();              // wechselt auf idle
//    gotoIdle();                   // Wenn Daten empfangen auf Idle wechseln
//    cc2500WriteRegCheckIdle(CC2500_PKTLEN, sizeof(Telemetrie));  Ist gleich wie MessageData
    setRx();                     // Frequenz nicht wechseln
  }
  else
    setNextChanRx();
}

void tx_error(void)       // Achtung bei CCA
{
//  if(eeprom.ccaOff)
    SET_BIT(state.ledError,L_NOT_TX);
//  else
//  {
//    ++state.ccaSum;
//    if(state.ccaCount < 5)
//      ++state.ccaCount;                   // hier CCA abschalten wenn zuviel
//    if(state.ccaCount >= 5)
//    {
//      cc2500WriteRegCheckIdle(CC2500_MCSM1, SMARTRF_SETTING_MCSM1 & ~0x30);  // aus
//    }
//  }
  setNext();                            // Empfang ein
}

void tx_ok(void)
{
  if(state.ccaCount > 0)
    --state.ccaCount;
}

void TxSendcommand(MessageData *mes)
{
  if((cc2500WriteFIFOBlock((uint8_t *)&mes, sizeof(MessageData)) & CC2500_STATUS_STATE_BM) < CC2500_STATE_TX)
    tx_error();
  else
    tx_ok();
}

void TxSendData(bool lastFlag)
{
  static MessageCommand mes;

  mes.type = 0x7;               // Kommando
  mes.rts = lastFlag;

  if(state.SetFaileSafe)
  {
    mes.command = 1;
    TxSendcommand((MessageData *)&mes);                 // Failsafe- Position auf aktuelle Position setzen
    state.SetFaileSafe = false;
  }
  else
  {
    if(state.newCommand)                                // Kommando vom UART
    {
      sendMes.Uart.rts = lastFlag;
      TxSendcommand(&sendMes);
      state.newCommand = false;
    }
    else
    {
      mes.command = 0;                                  // Dummy
      TxSendcommand((MessageData *)&mes);
    }
  }
}

/*uint8_t searchChan(void)
{
  uint8_t chanNewTmp = output.chanNew, x;
  static uint8_t i;

  if(chanNewTmp)
  {
    uint8_t y = 1 << i;
    while(1)
    {
      ++i;
      i &= 7;
      y = y << 1;                      // die 1 durchschieben
      if(y == 0) y = 1;
      if((x = chanNewTmp & y))             // 8 Bit Arithmetik erzwingen
      {
        cli();
        output.chanNew &= ~y;               // nicht atomic!
        sei();
        return(i + 1);
      }
    }
  }
  else
    return(0);
}*/

void copyTx(bool lastFlag, uint8_t type)
{
  static MessageChan mes;
  uint8_t i;
  uint16_t high;
  high = 0;
  uint8_t ofs = (type & 0x3) * 4;
  mes.rts = lastFlag;
  mes.type = type;
  for(i = 0;i < 4;++i)
  {
    uint16_t temp;
    do
      temp = output.chan_1us[ofs + i];
    while(temp != output.chan_1us[ofs + i]);      // Testen ob Interrupt es verändert hat
    uint8_t temp_low = temp & 0xff;
    mes.chan_1uslow[i] = temp_low;
    uint8_t temp_high = (temp >> 8) & 0x7;      // auf 3 Bits beschränken
    high <<= 3;
    high |= temp_high;
  }
  mes.chan_1ushigh = high;
  if((cc2500WriteFIFOBlock((uint8_t *)&mes, sizeof(MessageChan)) & CC2500_STATUS_STATE_BM) < CC2500_STATE_TX)
    tx_error();
  else
    tx_ok();
}

void TxSend(bool lastFlag)
{
  static uint8_t groupBackup;

  uint8_t syncTemp = output.ppmSync;
  if(syncTemp == 4)
  {                                                   // untere Gruppe senden
//    UDR0 = 'B';                                       // Puls für Sender zum Synchronisieren
    copyTx(lastFlag, 4);
    --syncTemp;
  }
  else if(syncTemp != 0)
  {                                                    // obere Gruppe senden
    copyTx(lastFlag, 4 - syncTemp);
    if((syncTemp == 3) && (state.maxChan <= 8))
      syncTemp = 0;                             // nur untere 8 Kanäle ausgeben
    else
      --syncTemp;
  }
  else if(state.SetFaileSafe || state.newCommand)
  {
    TxSendData(lastFlag);
  }
  else 
  {
    copyTx(lastFlag, groupBackup);
    ++groupBackup;
    if(state.maxChan <= 8)
      groupBackup &= 0x1;
    else
      groupBackup &= 0x3;
  }
  output.ppmSync = syncTemp;                    // Interrupt macht nichts, so lange nicht zu oft
}

void TxReceive(uint8_t anz)
{
  if((anz != sizeof(TelemetrieMes))
      || !cc2500ReadFIFOBlock((uint8_t *)&TelemetrieMes, sizeof(TelemetrieMes)))
  {
    while(anz-- > 0)
      cc2500ReadReg(CC2500_RXFIFO);         // Flush geht nicht weil rx
    if(Telem_error < 0xffff)
      ++Telem_error;
    TelemetrieMes.data.StatusTx.type = 0;
    TelemetrieMes.data.StatusTx.source = 0;
    TelemetrieMes.data.StatusTx.Timer33ms = Timer33ms;
    TelemetrieMes.data.StatusTx.Telem_error = Telem_error;
    TelemetrieMes.crcOk = true;
//    TelemetrieMes.date.statusTx.
  }
}
/*
uint8_t b2hex(uint8_t bin)
{
  bin = bin % 0x10;
  if(bin > 9)
    bin += 'A' - 10 - '0';
  return(bin + '0');
}
*/

void UartTxReady(void)
{
  while(!(UCSR0A & (1 << UDRE0)));              // warten bis Sendepuffer leer
}

void sendTelemetrie2UART(void)
{
  if(TelemetrieMes.crcOk)
  {
    UartTxReady();
    UDR0 = 'T';
    uint8_t i;
    uint8_t *p;
    for(i = 0, p = &TelemetrieMes.data.Unspec.dataB6;i < 6;++i)
    {
      UartTxReady();
      UDR0 = *p++;
    }
    TelemetrieMes.crcOk = false;
  }
}

void chkFailSafe(void)
{
  if(checkKey())
  {
    state.SetFaileSafe = true;
    SET_BIT(state.ledError, L_SET_FAILSAVE);
  }
  else
    RES_BIT(state.ledError, L_SET_FAILSAVE);
}

bool cc2500_EnableTx(void)
{
  // heiße Sache, CPU hat 0,64ms Zeit um die Daten zu schreiben
  // aber das Timing ist konstanter!
  RES_BIT(PORTD, OUT_D_CRX);
  if((SPI_MasterTransmit(CC2500_STX) & CC2500_STATUS_STATE_BM) != CC2500_STATE_RX)
  {
    sei();
    SET_BIT(state.ledError, L_NOT_RX);
    return(false);
  }
  else
  {
    if(eeprom.ccaOff || (cc2500GetState() != CC2500_STATE_RX))
    {
      SET_BIT(PORTB, OUT_B_CTX);
      sei();
      EIFR = 1 << INTF0;                          // Interruptflag löschen
      SET_BIT(EIMSK, INT0);                           // Interrupt ein
      LEDGREEN_ON;
      return(true);
    }
    else
    {
      sei();
      ++state.ccaSum;
      if(state.ccaCount < 5)
        ++state.ccaCount;                   // hier CCA abschalten wenn zuviel
      if(state.ccaCount >= 5)
      {
        cc2500WriteRegCheckIdle(CC2500_MCSM1, SMARTRF_SETTING_MCSM1 & ~0x30);  // aus
      }
      LEDGREEN_OFF;
      return(false);
    }
  }
}

//void cc2500_TxNormOn(void)
//{
//  // heiße Sache, CPU hat 0,64ms Zeit um die Daten zu schreiben
//  // aber das Timing konstanter!
//  cc2500_EnableTx();
//  sei();
//  if(state.txCount >= 7)
//  {
//    TxSend(true);
//    state.txCount = 0;
//  }
//  else
//  {
//    TxSend(false);
//    ++state.txCount;
//  }
//}
#ifdef DEBUG
uint8_t testsperre;
uint8_t testlaenge;
uint8_t sperre;
#endif

void txState(void)
{
  static enum transmitter txstate;
  uint8_t anz;
  bool res;

  switch(txstate)
  {
  case Start:
    sei();
    if(checkKey())               // Taste prüfen
      txstate = TxBindCheck;
    else
    {
      OCR2A  = TELETIME;              // 3,003 ms
      cc2500WriteRegCheckIdle(CC2500_PKTLEN, sizeof(MessageData));
      calibrateSlow();
      gotoIdle();                   // Wenn Daten empfangen auf Idle wechseln
      setNextChanRx();            // Frequenz einstellen
      txstate = RxCalc;           // Erstmal auf Empfang damit auch Kalibriert wird
    }
    InterruptAct = false;
    break;
  case TxOn:                      // Daten senden
#ifdef DEBUG
    if((testsperre != state.txCount) && !sperre)
#endif
     res = cc2500_EnableTx();
#ifdef DEBUG
    else
    {
      if(!sperre)
        sperre = testlaenge;
      else
        --sperre;
      res = false;
    }
#endif
    if(state.txCount >= 7)
    {
      state.txCount = 0;
      if(res)
        TxSend(true);                // Empfang wird im Interrupt aktiviert
      else
        setNext();
      OCR2A  = (TELETIME08 + CHANTIME + 1);
      txstate = RxCalc;
      UDR0 = 'C';                  // Aufforderung Kanäle schicken
    }
    else
    {
      ++state.txCount;
      if(res)
        TxSend(false);
      else
        setNext();
      OCR2A  = CHANTIME;
      if(!state.newCommand)
        UDR0 = 'Q';
    }
    InterruptAct = false;
    break;
  case RxCalc:
    sei();
    OCR2A  = TELETIME02;
    anz = cc2500IdleGetRXB();
    calibrateFast();
    stayInRx();                         // Auf Empfang bleiben auch wenn Daten kommen (für CCA)
//    cc2500WriteRegCheckIdle(CC2500_PKTLEN, sizeof(MessageData));
    setNextChanRx();
    TxReceive(anz);                     // Empfangsdaten auswerten
    txstate = TxOn;                     // Ein Durchlauf = 8*0,4c + 8*0,6c + 3 = 20,15 ms
    InterruptAct = false;
    break;
  case TxBindCheck:                     // Warten bis Taste losgelassen
    sei();
    if(!checkKey())                     // Wenn Taste losgelassen
    {
      if(Timer33ms > (5000 / 33))       // sehr langer Tastendruck (5s)
        calcNewId();
      setBindMode();
      calibrateSlow();
      setNextChanRx();
      state.txCount = 1;                // damit Sender immer läuft
      SET_BIT(state.ledError, L_BIND_ON);
      OCR2A  = CHANTIME2;
      txstate = TxOnBind;               // CHANTIME zum Senden
    }
    InterruptAct = false;
    break;
  case TxOnBind:                        // Daten ins Senderegister
    if(cc2500_EnableTx())                  // Sender aktivieren, Interrupt ein
    {
      if((cc2500WriteFIFOBlock((uint8_t *)&eeprom.bind, sizeof(eeprom.bind)) & CC2500_STATUS_STATE_BM) < CC2500_STATE_TX)
        tx_error();
      else
        tx_ok();
    }
    else
      setNext();
    InterruptAct = false;
//    state.InterruptAlive = true;
    break;
  }
}

void USART_Init( unsigned int ubrr)
{
  UBRR0 = ubrr;
  UCSR0A = 0;                                   //1 << U2X0;
  UCSR0C = (3 << UCSZ00);                     /* Set frame format: 8data, 1stop bit */
  UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);    /* Enable receiver and transmitter */
  SET_BIT(PORTD, INP_D_RXD);                    // Pullup ein
}

int __attribute__((naked)) main(void)
{
  cli();
  CLKPR = 0;
  PRR = 0;        // Powerreduction für ADC?

  PORTB = (1<<OUT_B_SPI_SS);
  DDRB = (1<<OUT_B_SPI_MOSI) | (1<<OUT_B_SPI_SCK) | (1<<OUT_B_SPI_SS) | (1 << OUT_B_CTX);
  PORTB = (1<<OUT_B_SPI_SS)
      | ~((1<<OUT_B_SPI_MOSI) | (1<<OUT_B_SPI_SCK) | (1<<OUT_B_SPI_SS) | (1 << OUT_B_CTX));

  DDRC = (1 << OUT_C_LEDRED) | (1 << OUT_C_LEDGREEN);
  PORTC = ~((1 << OUT_C_LEDRED) | (1 << OUT_C_LEDGREEN));

  DDRD = (1 << OUT_D_CRX);
  PORTD = ~(1 << OUT_D_CRX);

  LEDRED_ON;
// Timer0 32,768ms für clock
  TCCR0B = (5 << CS00);                 // clk/1024  -> 0,128 ms
  TCNT0 = 0;
  TIFR0 = 0xff;
  TIMSK0 = 0;

  wdt_enable(WDTO_500MS);

  EICRA = (1 << ISC01);                       // int0 bei fallender Flanke
  EIMSK = 0;                               // Achtung cc2500 gibt hier Takt aus per default

  eeprom_read_block(&eeprom, 0, sizeof(eeprom));
  if(!checkId() || (eeprom.checksum !=
            calcCheckSum((uint8_t *)&eeprom, sizeof(eeprom) - sizeof(eeprom.checksum))))
    calcNewId();
  cc2500_Init(eeprom.power);
  if(!checkcc2500())
    SET_BIT(state.ledError, L_INIT_ERROR);
  cc2500WriteReg(CC2500_SYNC0,(unsigned char)eeprom.bind.id);
  cc2500WriteReg(CC2500_SYNC1,(unsigned char)(eeprom.bind.id >> 8));
  if(eeprom.ccaOff)
    cc2500WriteReg(CC2500_MCSM1, SMARTRF_SETTING_MCSM1 & ~0x30);
  else
    cc2500WriteReg(CC2500_MCSM1, (SMARTRF_SETTING_MCSM1 | 0x10) & ~0x20);
  set_sleep_mode(SLEEP_MODE_IDLE);
  USART_Init(BRATE);

  wdt_enable(WDTO_30MS);

// Timer1 8MHz   PPM Capture
  TCCR1A = 0;
  TCCR1B = (1 << ICNC1) | (1 << CS10);      // 8MHz, Input Capture Noise Canceler, falling edge
  TCNT1 = 0;
  TIFR1 = 0xff;
  TIMSK1 = (1 << ICIE1);

// Timer2  für Statemachine
  TCCR2A = (2 << WGM20);                        //  CTC mode
  TCCR2B = (6 << CS20);                         // clk/256
  OCR2A  = CHANTIME;
  TCNT2 = 0;
  TIFR2  = 0xff;
  TIMSK2 = (1 << OCIE2A);

  LEDRED_OFF;
  LEDGREEN_ON;

#ifdef DEBUG
  testsperre = 0xff;
#endif

  sei();
  while(1){
    if(TIFR0 & (1 << TOV0))
    {
      ++Timer33ms;
//      SET_BIT(TIFR0, TOV0);
      TIFR0 = (1 << TOV0);
      set_led();
      chkFailSafe();
    }
    sendTelemetrie2UART();           // Hier ist am meisten Zeit

//    if(state.InterruptAlive)
    {
//      state.InterruptAlive = false;
      if(Heartbeat)
      {
        wdt_reset();
        Heartbeat = false;
      }
    }
    sleep_mode();                   //    warten bis Timer (Interrupt)
  }
}

