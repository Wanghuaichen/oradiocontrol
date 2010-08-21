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

Probleme:
bugs:
todo:
done:

FAQ:
Was passiert wenn kein PPM- Frame kommt?  Sender sendet nur Telemetrie

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
#define OUT_D_CRX  PORTD5

#define LEDGREEN_OFF RES_BIT(PORTC, PORTC1)
#define LEDGREEN_ON SET_BIT(PORTC, PORTC1)
#define LEDRED_OFF RES_BIT(PORTC, PORTC0)
#define LEDRED_ON SET_BIT(PORTC, PORTC0)

#define FOSC 8000000 // Clock Speed
#define BAUD 19200
#define MYUBRR FOSC/16/BAUD-1

#include "ORC_tx.h"


/* EEMEM */ EEData eeprom;
volatile State state;
volatile OutputData output;
volatile uint16_t Timer1ms;
volatile uint16_t Timer33ms;
TelemetrieReceive TelemetrieMes;
uint8_t uartTxBuf[11], uartRead;
volatile bool TxInterrupt;

uint8_t mcusr_mirror __attribute__ ((section (".noinit")));

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

ISR(TIMER2_COMPA_vect, ISR_NOBLOCK)              // Timer 1ms
{
//  RES_BIT(TIMSK2, OCIE2A);
//  cli();
  ++Timer1ms;
//  sei();
}

ISR(INT0_vect, ISR_NOBLOCK)
{
  TxInterrupt = true;
  RES_BIT(EIMSK, INT0);                    // INT0 aus, dient nur zum wecken
}

ISR(TIMER1_CAPT_vect, ISR_NOBLOCK)                  //8MHz capture
{
  static uint16_t timer_alt, capture_alt;
  static uint8_t chanPtr;
  uint16_t timer1msTemp;
  uint8_t chanPtrtemp = chanPtr;

  do
  {
    timer1msTemp = Timer1ms;
  }
  while(timer1msTemp != Timer1ms);
  uint16_t capture = ICR1;
  if(timer1msTemp - timer_alt > 2)   // Neuer PPM- Frame
  {
//    TCNT2 = 0;                        // Timer 2 initialisieren
    chanPtrtemp = 0;
    state.maxChan = (chanPtrtemp - 1) & 7;
  }
  else if(chanPtrtemp < 8)               // Auf 8 Kanäle begrenzen
  {
    output.chan_1us[chanPtrtemp] = (capture - capture_alt) / 8 - 1500u;
    output.chanNew |= (1 << chanPtrtemp);   // rechenintensiv!
    ++chanPtrtemp;
  }
  capture_alt = capture;
  timer_alt = timer1msTemp;
  chanPtr = chanPtrtemp;
}

ISR(USART_UDRE_vect, ISR_NOBLOCK)
{
  UDR0 = uartTxBuf[uartRead++];
  if(!uartTxBuf[uartRead])
    RES_BIT(UCSR0B, UDRIE0);                // USART- Interrupt aus
}

ISR(USART_RX_vect, ISR_NOBLOCK)
{
  static uint8_t chantemp, msbdata;
  static enum uart rxstate;

  uint8_t rxbuf = UDR0;

  switch(rxstate)
  {
  case WaitToken:
    if(rxbuf == 'C')
      rxstate = ReadChan;
    break;
  case ReadChan:
    if(rxbuf & ~0x7)
      rxstate = WaitToken;
    else
    {
      chantemp = rxbuf;
      rxstate = ReadMSB;
    }
    break;
  case ReadMSB:
    msbdata = rxbuf;
    rxstate = ReadLSB;
    break;
  case ReadLSB:
//    uint16_t temp;
//    high(temp) = msbdata;
//    low(temp) = rxbuf;
//    output.chan_1us[chantemp] = temp;
    output.chan_1us[chantemp] = (msbdata << 8) + rxbuf;
    output.chanNew |= (1 << chantemp);
    rxstate = WaitToken;
  }
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
  eeprom.bind.step = BINDMODESTEP;
  cc2500setPatableMax(0);             // Sendeleistung runter
}

void setNextChan(void)                // Kanal schreiben
{
  uint16_t tempChan = state.actChan + eeprom.bind.step * 2 + 1;
  if(tempChan > MAXHOPPCHAN)
    tempChan -= (MAXHOPPCHAN + 1);
  state.actChan = tempChan;
  cc2500WriteRegCheckIdle(CC2500_CHANNR, tempChan);
}

void setNextChanRx(void)                      // Kanal schreiben und nach RX
{
  RES_BIT(PORTB, OUT_B_CTX);
  SET_BIT(PORTD, OUT_D_CRX);
  if(PIND & (1<<INP_D_CC2500_GDO0))
    SET_BIT(state.ledError, L_TX_NOT_READY);
  setNextChan();
//  cc2500WriteReg(CC2500_PKTLEN, sizeof(MessageData));
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

void setPaketsizeSend(void)
{
  cc2500WriteRegCheckIdle(CC2500_PKTLEN, sizeof(MessageData));
}

void setPaketsizeReceive(void)
{
  cc2500WriteRegCheckIdle(CC2500_PKTLEN, sizeof(Telemetrie));
}

bool checkKey(void)
{
  return(!(PINC & (1 << INP_C_KEY)));
}

void set_led(void)
{
  static uint8_t timer_alt;
  static uint8_t led_count;

  int8_t diff = ((uint8_t)Timer33ms) - timer_alt;
  if(diff > 7)
  {
    timer_alt = (uint8_t)Timer33ms;;

    if(!(led_count & 0xf))            // unteres Nibble 0
    {
      while(state.ledError)
      {
        led_count += 0x10;
        led_count &= 0x7f;
        if(state.ledError & (1 << (led_count >> 4)))
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
        timer_alt += 20;                    // Pause
    }
  }
}

bool checkId(void)
{
  return (eeprom.bind.id && (eeprom.bind.id != 0xffff) &&
         (eeprom.bind.step > 4) && (eeprom.bind.step < MAXHOPPCHAN / 2 - 4));
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
    eeprom.bind.id += TCNT0 + TCNT1 + TCNT2;
    eeprom.bind.step += eeprom.bind.id;
    eeprom.bind.step &= 0x3f;
//    if(eeprom.step > 204)
//      eeprom.step -= (204 + 1);
  }
  while(!checkId());
  eeprom.checksum = calcCheckSum((uint8_t *)&eeprom, sizeof(eeprom) - sizeof(eeprom.checksum));
  eeprom_write_block(&eeprom, 0, sizeof(eeprom));
  SET_BIT(state.ledError, L_EEPROM_ERR);
}

void TxSendcommand(uint16_t command, bool lastFlag)
{
  MessageCommand mes;

  mes.mode = 1;
  mes.rts = lastFlag;
  mes.command = command;
  cc2500WriteFIFOBlock((uint8_t *)&mes, sizeof(mes));
}

void TxSendData(bool lastFlag)
{
  if(state.SetFaileSafe)
  {
    TxSendcommand(1, lastFlag);           // Failsafe- Position setzen
    state.SetFaileSafe = false;
  }
  else
    TxSendcommand(0, lastFlag);
}

uint8_t searchChan(void)
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
}

void TxSendChan(void)
{
  uint8_t tempPtr;           // = output.chanPtr;
  MessageChan mes;

  // Kanal suchen der gesendet werden soll
  if((tempPtr = searchChan()))    // Wird im Interrupt geändert
  {
    --tempPtr;
    mes.mode = 0;
    mes.channel = tempPtr;
    mes.chan_1us = output.chan_1us[tempPtr];
    cc2500WriteFIFOBlock((uint8_t *)&mes, sizeof(mes));
  }
  else
  {                       // Es ist nichts da, dann Daten senden
    TxSendData(false);
  }
}

void TxReceive()
{
  uint8_t x;
  if((x = get_RxCount()) == sizeof(TelemetrieMes))
    cc2500ReadFIFOBlock((uint8_t *)&TelemetrieMes, sizeof(TelemetrieMes));
  else
    while(x-- > 0)
      cc2500ReadReg(CC2500_RXFIFO);         // Flush geht nicht weil rx
}

uint8_t b2hex(uint8_t bin)
{
  bin = bin % 0x10;
  if(bin > 9)
    bin += 'A' - 10 - '0';
  return(bin + '0');
}

void sendTelemetrie2UART(void)
{
  uint16_t d1, d2;

//  if(UCSR0B & (1 << UDRIE0))                // USART- Interrupt ist ein
    return;

  while(!(UCSR0A & (1 << UDRE0)));          // Warten bis Buffer leer
  if(TelemetrieMes.crcOk)
  {
    UDR0 = 'T';
    d1 = TelemetrieMes.data.sensor;
    d2 = TelemetrieMes.data.data;
    TelemetrieMes.crcOk = false;
  }
  else
  {
    UDR0 = 'S';
    d1 = Timer1ms;
    d2 = Timer33ms;
  }
  while(!(UCSR0A & (1 << UDRE0)));          // Warten bis Buffer leer
  uint8_t temp = (d1 >> 8);
  UDR0 = b2hex(temp / 0x10);
  uartTxBuf[0] = b2hex(temp);
  uartTxBuf[1] = b2hex(((uint8_t )d1) / 0x10);
  uartTxBuf[2] = b2hex(d1);
  uartTxBuf[3] = ' ';
  temp = (d2 / 0x100);
  uartTxBuf[4] = b2hex(temp / 0x10);
  uartTxBuf[5] = b2hex(temp);
  uartTxBuf[6] = b2hex(((uint8_t )d2) / 0x10);
  uartTxBuf[7] = b2hex(d2);
  uartTxBuf[8] = '\r';
  uartTxBuf[9] = '\n';
  uartTxBuf[10] = 0;
  uartRead = 0;
  SET_BIT(UCSR0B, UDRIE0);                // USART- Interrupt ein
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

void cc2500_TxOn(void)
{
  RES_BIT(PORTD, OUT_D_CRX);
  SET_BIT(PORTB, OUT_B_CTX);
  if((SPI_MasterTransmit(CC2500_STX) & CC2500_STATUS_STATE_BM) != CC2500_STATE_RX)
    SET_BIT(state.ledError, L_TX_NOT_RX);       // Enable TX
  SET_BIT(EIFR, INTF0);                          // Interruptflag löschen
  SET_BIT(EIMSK, INT0);                           // Interrupt ein
}

void cc2500_TxNormOn(void)
{
  // heiße Sache, CPU hat 0,64ms Zeit um die Daten zu schreiben
  // aber das Timing ist schön konstant!
  cc2500_TxOn();
  if(state.txCount >= 7)
  {
    TxSendData(true);
    state.txCount = 0;
  }
  else
  {
    TxSendChan();
    ++state.txCount;
  }
}

void cc2500_TxBindOn(void)
{
  cc2500_TxOn();
  cc2500WriteFIFOBlock((uint8_t *)&eeprom.bind, sizeof(eeprom.bind));
}

void txState(void)
{
  static enum transmitter txstate;

  switch(txstate)
  {
  case Start:
    if(checkKey())               // Taste prüfen
      txstate = TxBindCheck;
    else
    {
      setPaketsizeSend();
      txstate = TxReady;
    }
    cc2500StartCal();
    break;
  case TxReady:                  // Frequenz einstellen
    setNextChanRx();
    chkFailSafe();
    txstate = TxOn;
    break;
  case TxOn:                      // Daten senden
    cc2500_TxNormOn();            // Sender aktivieren, Daten senden
    sendTelemetrie2UART();        // Hier ist am meisten Zeit
    if(!state.txCount)
      txstate = RxOn;
    else
      txstate = TxReady;
    break;
  case RxOn:                      // Empfänger einstellen, Kalibrieren
    calibrateOn();
    setPaketsizeReceive();
    setNextChanRx();
    txstate = RxWait2;
    break;
  case RxWait2:                  // Jetzt wird empfangen
    txstate = RxWait3;
    break;
  case RxWait3:                   // Immer noch empfangen
    txstate = RxCalc;
    break;
  case RxCalc:
//    cc2500Idle();
    calibrateOff();
    setPaketsizeSend();
    setNextChanRx();
    TxReceive();                  // Empfangsdaten auswerten
    txstate = TxOn;
    break;
  case TxBindCheck:
    if(!checkKey())               // Wenn Taste losgelassen
    {
      if(Timer33ms > (5000 / 33))
        calcNewId();
      setBindMode();
      SET_BIT(state.ledError, L_BIND_ON);
      txstate = TxNextChanBind;
    }
    break;
  case TxNextChanBind:
    setNextChanRx();
    txstate = TxOnBind;
    break;
  case TxOnBind:                  // Daten ins Senderegister
    cc2500_TxBindOn();                          // Sender aktivieren
    txstate = TxWaitBind;
    break;
  case TxWaitBind:                  // 3ms Slot weil Telegramm länger als 2 Byte
    txstate = TxNextChanBind;
    break;
  }
}

void USART_Init( unsigned int ubrr)
{
  UBRR0H = (unsigned char)(ubrr>>8);          /* Set baud rate */
  UBRR0L = (unsigned char)ubrr;
  UCSR0A = 0;
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);           /* Enable receiver and transmitter */
  UCSR0C = (3<<UCSZ00);                     /* Set frame format: 8data, 1stop bit */
  UCSR0B |= (1 << RXCIE0);
}

int __attribute__((naked)) main(void)
{
  cli();
  CLKPR = 0;
  PRR = 0;        // Powerreduction für ADC?

  PORTB = (1<<OUT_B_SPI_SS) | (1 << INP_B_SPI_MISO);
  DDRB = (1<<OUT_B_SPI_MOSI) | (1<<OUT_B_SPI_SCK) | (1<<OUT_B_SPI_SS) | (1 << OUT_B_CTX);

  PORTC = (1 << INP_C_KEY) | (1 << PORTC3);
  DDRC = (1 << OUT_C_LEDRED) | (1 << OUT_C_LEDGREEN);

  PORTD = (1 << PORTD6) | (1 << PORTD7);
  DDRD = (1 << OUT_D_CRX);

  LEDRED_ON;
// Timer0 32,768ms für clock
  TCCR0B = (5 << CS00);
  TCNT0 = 0;
  TIFR0 = 0xff;
  TIMSK0 = 0;

// Timer1 8MHz   PPM Capture
  TCCR1A = 0;
  TCCR1B = (1 << ICNC1) | (1 << CS10);      // 8MHz
  TCNT1 = 0;
//  TIFR1 = 0xff;
  TIMSK1 = (1 << ICIE1);


// Timer2 1ms für Timer
  TCCR2A = (2 << WGM20);                        //  CTC mode
  TCCR2B = (4 << CS20);                         // clk/32  (3 << CS20)
  OCR2A  = CYCLETIME;       // ergibt 1ms (1000Hz)
  TCNT2 = 0;
//  TIFR2  = 0xff;
  TIMSK2 = (1 << OCIE2A);

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
  set_sleep_mode(SLEEP_MODE_IDLE);
  USART_Init(MYUBRR);
//  wdt_enable(WDTO_30MS);
  LEDRED_OFF;
  LEDGREEN_ON;
  sei();
  while(1){
    static uint16_t old1ms;
    do
      old1ms = Timer1ms;
    while(old1ms != Timer1ms);
    cc2500_Off();
    while(Timer1ms==old1ms)
    {
      sei();
      TxInterrupt = false;
      sleep_mode();                   //    warten bis Timer (Interrupt)
      if(TxInterrupt)               // eventuell auf Empfang
      {
        RES_BIT(PORTB, OUT_B_CTX);
        SET_BIT(PORTD, OUT_D_CRX);
      }
    }
    txState();
    if(TIFR0 & (1 << TOV0))
    {
      ++Timer33ms;
      SET_BIT(TIFR0, TOV0);
      set_led();
    }
    wdt_reset();
  }
}
