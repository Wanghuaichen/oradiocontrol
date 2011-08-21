/*
 * Author	Josef Glatthaar <josef.glatthaar@googlemail.com>
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
 * Compiler: avr-gcc (GCC) 4.3.4
 *
 * Version 0.1 :First connect
 * Version 0.5 :range check (over 1,5 km)
 * Version 1.0 :first fly
 * Version 1.1 :optimized interrupts, reduce overflows
 *

bugs:
todo:
tmain berechnen (min, max)
Achtung: Beim Einschalten FailSafe wegen Servos,
         bei Regler unbedingt richtigen Failsafe- Mode auswählen)

Sender am Boden ist der Master für das Timing

Mischbetrieb PPM- Ausgabe und Einzelausgabe (schwierig)

done:
gleichzeitige Ausgabe einer Gruppe (Kanal 1-4 oder Kanal 5-8) mit einer Auflösung von 1µs.
Die Gruppen sollten bei PPM- Input um 10 ms und bei UART- Input um 3 ms versetzt sein um minimales
Delay zu erreichen. Die Latenz kann bei PPM um 2 ms springen, weil das Sendemodul und nicht PPM der
Master für das Timing ist. In der Telemetrielücke kann die Latenz auch auf 5 ms ansteigen.
Bei UART- Input soll sich der Sender am Timing des Sendemoduls orientieren.
Bitfelder entfernt, weil EEPROM- Speicher nicht kritisch, Flash ist wichtiger

Kein Stackpointer mehr nötig (sollte wegen cli vermieden werden)
*/

//PIN 1   PD3   INT1/OC2B/PCINT19                       CC2500 GDO2
//PIN 2   PD4   XCK/T0/PCINT20                          LED
//PIN 3   GND                                           --
//PIN 4   VCC                                           --
//PIN 5   GND                                           --
//PIN 6   VCC                                           --
//PIN 7   PB6   OSC                                     8 MHz
//PIN 8   PB7   OSC                                     8 MHz
//PIN 9   PD5   T1/OC0B/PCINT21                         Taster
//PIN 10  PD6   AIN0/OC0A/PCINT22                       Antenne 1
//PIN 11  PD7   AIN1/OC2B/PCINT23                       Antenne 2
//PIN 12  PB0   ICP1/CLKO/PCINT0                        AMP high
//PIN 13  PB1   OC1A/PCINT1                             nc
//PIN 14  PB2   SS/OC1B/PCINT2 (SPI Bus Master Slave select)
//PIN 15  PB3   MOSI/OC2/PCINT3 (SPI Bus Master Output/Slave Input)
//PIN 16  PB4   MISO/PCINT4 (SPI Bus Master Input/Slave Output)
//PIN 17  PB5   SCK/PCINT5 (SPI Bus Master clock Input)
//PIN 18  AVCC                                          --
//PIN 19  ADC6                                          -
//PIN 20  AREF                                          --
//PIN 21  GND                                           --
//PIN 22  ADC7                                          -
//PIN 23  PC0   ADC0/PCINT8                             Servo 1
//PIN 24  PC1   ADC1/PCINT9                             Servo 2
//PIN 25  PC2   ADC2/PCINT10                            Servo 3
//PIN 26  PC3   ADC3/PCINT11                            Servo 4
//PIN 27  PC4   SDA/ADC4/PCINT12                        Servo 5
//PIN 28  PC5   SCL/ADC5/PCINT13                        Servo 6
//PIN 29  PC6   RESET/PCINT14                           Reset
//PIN 30  PD0   RXD/PCINT16                             Servo 7
//PIN 31  PD1   TXD/PCINT17                             Servo 8
//PIN 32  PD2   INT0/PCINT18                            CC2500 GDO0

#define OUT_B_PRE PORTB0
//#define OUT_B_SPI_SS PORTB2
//#define OUT_B_SPI_MOSI PORTB3
//#define INP_B_SPI_MISO PINB4
//#define OUT_B_SPI_SCK PORTB5
#define OUT_D_LED PORTD4
#define OUT_D_ANT1 PORTD6
#define OUT_D_ANT2 PORTD7

#define OUT_C_CHANNEL1 PORTC0
#define OUT_C_CHANNEL2 PORTC1
#define OUT_C_CHANNEL3 PORTC2
#define OUT_C_CHANNEL4 PORTC3
#define OUT_C_CHANNEL5 PORTC4
#define OUT_C_CHANNEL6 PORTC5
#define OUT_D_CHANNEL7 PORTD0             // RXD
#define OUT_D_CHANNEL8 PORTD1             // TXD

//#define INP_D_CC2500_GDO0 PIND2
//#define INP_D_CC2500_GDO2 PIND3
#define INP_D_KEY PIND5

#define LED_OFF RES_BIT(PORTD, OUT_D_LED)
#define LED_ON SET_BIT(PORTD, OUT_D_LED)

#define INTERRUPTOFFSET (1500u)                 // Mittelstellung Servos

#include "ORC_rx.h"

/* EEMEM */ EEData eeprom;
State state;
OutputData output;
//volatile uint16_t Timer1ms;
volatile uint16_t Timer25ms;
volatile bool ReceiverInterrupt;
//uint8_t heartbeat;
Telemetrie telemetrieBuf[8];
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
uint8_t resetCounter[3] __attribute__ ((section (".noinit")));
         // 0 -> External Reset, 1 -> Brown-out Reset, 2 -> Watchdog System Reset

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

ISR(INT0_vect, ISR_NAKED)          // Heisse Konstruktion!
{
  asm volatile("push    r24");    // immer prüfen ob nur R24 verwendet wird
  state.RxTimer = TCNT2;
  sei();                          // Verlust zusätzlich 6 Clocks
  ReceiverInterrupt = true;
  asm volatile(
  "pop    r24""\n\t"
  "reti":::"r24");
}

/**********************************/

void warte(uint8_t t)           // 17
{
  asm volatile(
  "subi    r24, 0x02""\n\t"
  "subi    r24, 0x01""\n\t"
  "breq    .+10""\n\t"
  "nop""\n\t"
  "nop""\n\t"
  "nop""\n\t"
  "nop""\n\t"
  "rjmp    .-14""\n\t"
  "nop":::"r24"
  );
  return;                               // 4
}

FASTPPM ppm;

ISR(TIMER1_COMPA_vect)                  // pulse generation, resolution 1µs, Interrupt ist aus!
{
  asm volatile(
  "lds     r25, 0x0088""\n\t"           // OCR1AL
  "subi    r25, -0x47""\n\t"            // + 0x47
  "lds     r24, 0x0084""\n\t"           // TCNT1L
  "sub     r24, r25""\n\t"
  "brmi    .-8""\n\t"
  "lds     r25, 0x0084""\n\t"           // TCNT1L
  "lds     r24, 0x0088""\n\t"           // OCR1AL
  "subi    r25, 0x47 + 2""\n\t"         // normal +4
  "sub     r25, r24""\n\t"              // Differenz durch Sprungtabelle ausgleichen
  "mov     r24, r25""\n\t"
  "andi    r24, 0xf8""\n\t"
  "brne    L_%= + 6""\n\t"              // Über- oder Unterlauf
//  "andi    r25, 0x07""\n\t"
  "ldi     r30, lo8(pm(L_%=))""\n\t"
  "ldi     r31, hi8(pm(L_%=))""\n\t"
  "add     r30, r25""\n\t"
  "adc     r31, r1""\n\t"
  "ijmp""\n\t"
  "L_%=: nop""\n\t"
  "nop""\n\t"
  "nop""\n\t"
  "nop""\n\t"
  "nop""\n\t"
  "nop""\n\t"
  "nop":::"r24","r25","r30","r31");

  uint8_t c = TCNT1L; //- OCR1AL;          // 0x55

  uint8_t *p = &ppm.toggleC[output.idx];
  uint8_t c1,d1,c2,d2,c3,d3,c4,d4;
  c1 = *p;
  c2 = *(p + 1);                        // Hier wird auch eventuell Müll gelesen
  c3 = *(p + 2);                        // macht aber nichts
  c4 = *(p + 3);                        // Wichtig ist der immer gleiche Ablauf
  d1 = *(p + 4);
  d2 = *(p + 5);
  d3 = *(p + 6);
  d4 = *(p + 7);

  while(1)
  {
    PIND = d1;
    PINC = c1;
    uint8_t dif = *(p + 8);
    if(dif != 1)
    {
      if(!dif)
        break;
      if(dif == 2)
      {
        NOP();
        NOP();
        NOP();
      }
      else
        warte(dif);
    }
    NOP();
    PIND = d2;
    PINC = c2;
    dif = *(p + 9);
    if(dif != 1)
    {
      if(!dif)
        break;
      if(dif == 2)
      {
        NOP();
        NOP();
        NOP();
      }
      else
        warte(dif);
    }
    NOP();
    PIND = d3;
    PINC = c3;
    dif = *(p + 10);
    if(dif != 1)
    {
      if(!dif)
        break;
      if(dif == 2)
      {
        NOP();
        NOP();
        NOP();
      }
      else
        warte(dif);
    }
    NOP();
    PIND = d4;
    PINC = c4;
    break;
  }
  bool receiverIntTemp = EIFR & (1 << INTF0);
  TIMSK1 &= ~(1<<OCIE1A);         // PPM Interrupt aus
  sei();                          // Interrupt ein, jetzt kommen alle anderen Interrupts zum Zug
  c -= OCR1AL;
  uint8_t i = output.idx;
  while(*(p + 8) && (i < 3))                    // Ende suchen
  {
    ++p;
    ++i;
  }
  ++i;
  if((ppm.raw[i] == 0) || (i > 3))              // prüfen ob noch was kommt
  {
//    TIMSK1 &= ~(1<<OCIE1A);           // Interrupt aus
    TCCR1B = 0;                         // Timer aus
  }
  else
  {
    OCR1A = ppm.raw[i];
    output.idx = i;
    TIMSK1 |= (1<<OCIE1A);           // Interrupt ein
  }

  if(output.latenzMin > c)
    output.latenzMin = c;
  if(output.latenzMax < c)
    output.latenzMax = c;

  if(receiverIntTemp)                           // Empfängerinterrupt ist während cli() gekommen
    --state.RxTimer;                            // Capturewert etwas korrigieren

//  static uint8_t tes;
//  if(TCNT1L > tes)
//    tes = TCNT1L;
}

void setupPulses(bool highGroup)
{
  uint8_t i, y;

  if(TIMSK1 & (1<<OCIE1A))                // läuft noch 
    return;

  uint8_t Cflag;
  uint8_t Dflag;

  if(!highGroup)
  {
    output.pulsesOffset = 0;
    if(!(output.portCflg & ((1 << OUT_C_CHANNEL1) | (1 << OUT_C_CHANNEL2)
        | (1 << OUT_C_CHANNEL3) | (1 << OUT_C_CHANNEL4))))
      return;
    Cflag = (output.portCflg & ((1 << OUT_C_CHANNEL1) | (1 << OUT_C_CHANNEL2)
        | (1 << OUT_C_CHANNEL3) | (1 << OUT_C_CHANNEL4)))
        | ~((1 << OUT_C_CHANNEL1) | (1 << OUT_C_CHANNEL2) | (1 << OUT_C_CHANNEL3)   // Alle Nichtausgänge auf C setzen
        | (1 << OUT_C_CHANNEL4) | (1 << OUT_C_CHANNEL5) | (1 << OUT_C_CHANNEL6));
    Dflag = (uint8_t)~((1 << OUT_D_LED) | (1 << OUT_D_ANT1) | (1 << OUT_D_ANT2)   // Alle Nichtausgänge (Eingänge) auf D setzen
        | (1 << OUT_D_CHANNEL7) | (1 << OUT_D_CHANNEL8));
    ppm.toggleC[0] = (1 << OUT_C_CHANNEL1) & Cflag;
    ppm.toggleC[1] = (1 << OUT_C_CHANNEL2) & Cflag;
    ppm.toggleC[2] = (1 << OUT_C_CHANNEL3) & Cflag;
    ppm.toggleC[3] = (1 << OUT_C_CHANNEL4) & Cflag;
    ppm.toggleD[0] = 0;
    ppm.toggleD[1] = 0;
    output.pulsesTimer[0] = 0;
    output.pulsesTimer[1] = 0;
    output.pulsesTimer[2] = 0;
    output.pulsesTimer[3] = 0;
    output.portCflg &= ~((1 << OUT_C_CHANNEL1) | (1 << OUT_C_CHANNEL2)
            | (1 << OUT_C_CHANNEL3) | (1 << OUT_C_CHANNEL4));
  }
  else
  {
    if(!(output.portCflg & ((1 << OUT_C_CHANNEL5) | (1 << OUT_C_CHANNEL6)))
        && !(output.portDflg & ((1 << OUT_D_CHANNEL7) | (1 << OUT_D_CHANNEL8))))
      return;
    Cflag = (output.portCflg & ((1 << OUT_C_CHANNEL5) | (1 << OUT_C_CHANNEL6)))
        | ~((1 << OUT_C_CHANNEL1) | (1 << OUT_C_CHANNEL2) | (1 << OUT_C_CHANNEL3)
        | (1 << OUT_C_CHANNEL4) | (1 << OUT_C_CHANNEL5) | (1 << OUT_C_CHANNEL6));
    Dflag = (output.portDflg & ((1 << OUT_D_CHANNEL7) | (1 << OUT_D_CHANNEL8)))
        | ~((1 << OUT_D_LED) | (1 << OUT_D_ANT1) | (1 << OUT_D_ANT2)
        | (1 << OUT_D_CHANNEL7) | (1 << OUT_D_CHANNEL8));
    ppm.toggleC[0] = (1 << OUT_C_CHANNEL5) & Cflag;
    ppm.toggleC[1] = (1 << OUT_C_CHANNEL6) & Cflag;
    ppm.toggleC[2] = 0;
    ppm.toggleC[3] = 0;
    ppm.toggleD[0] = (1 << OUT_D_CHANNEL7) & Dflag;
    ppm.toggleD[1] = (1 << OUT_D_CHANNEL8) & Dflag;
    output.pulsesTimer[4] = 0;
    output.pulsesTimer[5] = 0;
    output.pulsesTimer[6] = 0;
    output.pulsesTimer[7] = 0;
    output.portCflg &= ~((1 << OUT_C_CHANNEL5) | (1 << OUT_C_CHANNEL6));
    output.portDflg = 0;
  }
  ppm.toggleD[2] = 0;
  ppm.toggleD[3] = 0;

  for(i = 0;i < 4;++i)
  {
    uint16_t temp;
    if(highGroup)
      temp = output.chan_1us[i + 4];
    else
      temp = output.chan_1us[i];
    ppm.raw[i] = temp * 8 + (INTERRUPTOFFSET * 8 - 116);   // Mit Pulslänge füllen
  }
  for(i = 0;i < 3;++i)            // qsort
  {
    for(y = i + 1;y < 4;++y)
    {
      if((ppm.raw[i] > ppm.raw[y]) || ((ppm.raw[i] == 0) && (ppm.raw[y] != 0)))
      {
        uint16_t temp = ppm.raw[i];
        ppm.raw[i] = ppm.raw[y];
        ppm.raw[y] = temp;
        uint8_t ctemp = ppm.toggleC[i];
        ppm.toggleC[i] = ppm.toggleC[y];
        ppm.toggleC[y] = ctemp;
        ctemp = ppm.toggleD[i];
        ppm.toggleD[i] = ppm.toggleD[y];
        ppm.toggleD[y] = ctemp;
      }
    }
  }
  for(i = 0;i < 3;++i)            // Gleiche zusammenfassen
  {
    uint16_t delaytemp = (ppm.raw[i + 1] - ppm.raw[i]) / 8;
    if(delaytemp == 0)
    {
      ppm.toggleC[i + 1] |= ppm.toggleC[i];
      ppm.toggleD[i + 1] |= ppm.toggleD[i];
      ppm.raw[i] = 0;             // ungültig markieren
    }
    if(delaytemp > state.minAb)     // kürzeste Zeit zwischen 2 Interrupts
      ppm.nextdelay[i] = 0;
    else
      ppm.nextdelay[i] = (uint8_t)delaytemp;
  }
  ppm.nextdelay[3] = 0;
  for(i = 0;i < 3;++i)            // Rest dahinter, herschieben
  {
    if(ppm.raw[i] == 0)           // Platz ist leer
    {
      for(y = i + 1;y < 4;++y)
      {
        if(ppm.raw[y] == 0)
          continue;
        ppm.raw[i] = ppm.raw[y];
        ppm.toggleC[i] = ppm.toggleC[y];
        ppm.toggleD[i] = ppm.toggleD[y];
        ppm.nextdelay[i] = ppm.nextdelay[y];
        ppm.raw[y] = 0;
        ppm.nextdelay[y] = 0;
        break;
      }
    }
  }
  output.idx = 0;
  output.port = false;
  TCCR1B = 0;      // Timer aus
  TCNT1 = 0;
  OCR1A = ppm.raw[0];
  //SET_BIT(TIFR1, OCF1A);
  TIFR1 = (1 << OCF1A);
  SET_BIT(TIMSK1, OCIE1A);          // Timer Interrupt Mask Register
  TCCR1B = (1 << CS10);      // Timer an 8MHz

//  NOP();
  PORTD |= Dflag;
  PORTC = Cflag;
//  NOP();                // ***
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

//void setNextChan(void)                // Kanal schreiben
//{
//  uint16_t tempChan = state.actChan + eeprom.bind.step * 2 + 1;
//  while(tempChan > MAXHOPPCHAN)
//    tempChan -= (MAXHOPPCHAN + 1);
//  state.actChan = tempChan;
//  cc2500WriteReg(CC2500_CHANNR, tempChan);
//}

//void setNextChanCheckIdleFail(void)                // Kanal schreiben
//{
//  uint16_t tempChan = state.actChan + eeprom.bind.step * 2 + 1;
//  tempChan += eeprom.bind.step * 2 + 1;
//  tempChan += eeprom.bind.step * 2 + 1;
//  while(tempChan > MAXHOPPCHAN)
//    tempChan -= (MAXHOPPCHAN + 1);
//  state.actChan = tempChan;
//  cc2500WriteRegCheckIdle(CC2500_CHANNR, tempChan);
//}

void setRx(void)
{
  SET_BIT(PORTB, OUT_B_PRE);
  cc2500CommandStrobe(CC2500_SRX);       // Emfänger ein
//  SET_BIT(EIFR, INTF0);
//  SET_BIT(EIMSK, INT0);                    // INT0 ein
//  Timercapt = 0;
}

void setNextChanGoRx(uint8_t c)                // Kanal schreiben und auf Empfang
{
  if(PIND & (1<<INP_D_CC2500_GDO0))
    SET_BIT(state.ledError, L_TX_NOT_READY);
  uint16_t tempChan = state.actChan;
  while(c--)
    tempChan += eeprom.bind.step * 2 + 1;
  while(tempChan > MAXHOPPCHAN)
    tempChan -= (MAXHOPPCHAN + 1);
  state.actChan = tempChan;
  cc2500WriteRegCheckIdle(CC2500_CHANNR, tempChan);
  setRx();
}

/*
void setPaketsizeReceive(void)
{
  cc2500WriteReg(CC2500_PKTLEN, sizeof(MessageData));
}

void setPaketsizeSend(void)
{
  cc2500WriteReg(CC2500_PKTLEN, sizeof(Telemetrie));
}*/

void setAnt(bool ant)
{
  if(ant)
  {
    RES_BIT(PORTD, OUT_D_ANT2);
    SET_BIT(PORTD, OUT_D_ANT1);
    state.actAnt = false;
  }
  else
  {
    RES_BIT(PORTD, OUT_D_ANT1);
    SET_BIT(PORTD, OUT_D_ANT2);
    state.actAnt = true;
  }
}

bool checkchanFree(void)            // False: Kanal frei
{
//  if(SPI_MasterReadReg(CC2500_PKTSTATUS | CC2500_READ_BURST) & 0x10)
  int8_t rssi = cc2500ReadStatusReg(CC2500_RSSI);
//  cc2500_Off();
  return(rssi < 0);         // Offset mit ca. 70 entspricht also -70dbm
}

void setFreeChanRx(void)            // Nächsten Kanal einstellen um zu sehen, ob er belegt ist
{
  uint16_t tempChan = state.actChan + 3;
  if(tempChan > MAXHOPPCHAN)
    tempChan -= (MAXHOPPCHAN + 1);
//  cc2500Idle();
  cc2500WriteReg(CC2500_CHANNR, tempChan);
  setRx();
//  SET_BIT(PORTB, OUT_B_PRE);
//  cc2500CommandStrobe(CC2500_SRX);
  state.actChan = tempChan;
}

void setFrequencyOffset(int8_t freqoff, bool lim)
{
  int16_t freq;

//  freqoff = cc2500ReadStatusReg(CC2500_FREQEST);
//  cc2500_Off();
  if(freqoff)
  {
    if(lim)
    {
      if(freqoff > 1)
        freqoff = 1;
      else if(freqoff < -1)
        freqoff = -1;
    }
//    fsctrl = cc2500ReadReg(CC2500_FSCTRL0);
//    freq = freqoff + fsctrl;
    freq = freqoff + state.freqOffset;
    if(freq > 0x7f)
      freq = 0x7f;
    else if(freq < -0x80)
      freq = -0x80;
    state.freqOffset = freq;
    cc2500WriteReg(CC2500_FSCTRL0, (int8_t)freq);
  }
}

prog_int8_t APM freq[] = {0, 40, -40, 120, -120, 80, -80};

void setNewRxFrequ(void)
{
  setAnt(state.actAnt);               // Antenne wechseln
  if(++state.actFreqIdx >= 7)
    state.actFreqIdx = 0;
  state.freqOffset = pgm_read_byte(&freq[state.actFreqIdx]);
  cc2500WriteReg(CC2500_FSCTRL0, pgm_read_byte(&freq[state.actFreqIdx]));
}

void setReceiveError(void)
{
  setAnt(state.actAnt);               // Antenne wechseln
  if(state.errorCount < 0xff)
    ++state.errorCount;
//  if(state.errorSum < 0xffff)
  ++state.errorSum;                 // max über 100 Tage Totalausfall
  if(state.led)
    LED_OFF;
}

bool readBindData(void)
{
  MessageBind mes;

  if(cc2500ReadFIFOBlock((uint8_t *)&mes, sizeof(mes)) && (mes.crcOk))
  {
    eeprom.bind.id = mes.data.id;
    eeprom.bind.step = mes.data.step;
    uint8_t i;
    for(i = 0;i < MAXCHAN;++i)
      if(eeprom.pulsesDelay[i] == 0xff)
        eeprom.pulsesDelay[i] = 15;                  // Default mindestens 15 ms Delay
    if(eeprom.outputOffset == 0xff)
    {
      eeprom.outputOffset = 10;                 // Default 10 ms Versatz
      eeprom_write_block(&eeprom, 0 ,sizeof(eeprom));
    }
    else
      eeprom_write_block(&eeprom.bind, 0 ,sizeof(eeprom.bind));
    state.bindmode = false;
    while(1);                   //       reset durch Watchdog
  }
  else
    return(false);
}

void getFailSafe(void)          // FailSafe- Werte beim Einschalten als Default übernehmen
{
  uint8_t i;
  for(i = 0;i < MAXCHAN;++i)
  {
    if(!eeprom.failSafe[i].failSafeMode)        // Nur Kanäle mit Hold oder On
    {
      uint16_t t = eeprom.failSafe[i].failSafePos;
      if(t & 0x400)                 // Auf zulässige Werte begrenzen
        t |= 0xf800;
      else
        t &= 0x3ff;
      output.chan_1us[i] = t;
      switch(i)
      {
      case 0:
        output.portCflg |= (1 << OUT_C_CHANNEL1);
        break;
      case 1:
        output.portCflg |= (1 << OUT_C_CHANNEL2);
        break;
      case 2:
        output.portCflg |= (1 << OUT_C_CHANNEL3);
        break;
      case 3:
        output.portCflg |= (1 << OUT_C_CHANNEL4);
        break;
      case 4:
        output.portCflg |= (1 << OUT_C_CHANNEL5);
        break;
      case 5:
        output.portCflg |= (1 << OUT_C_CHANNEL6);
        break;
      case 6:
        output.portDflg |= (1 << OUT_D_CHANNEL7);
        break;
      case 7:
        output.portDflg |= (1 << OUT_D_CHANNEL8);
        break;
      }
    }
  }
}

void setFailSafe(void)
{
  uint8_t i;
  for(i = 0;i < MAXCHAN;++i)
    eeprom.failSafe[i].failSafePos = output.chan_1us[i];        // Alle Positionen übernehmen
  eeprom_write_block(&eeprom.failSafe,(uint8_t *)((int)&eeprom.failSafe - (int)&eeprom) ,sizeof(eeprom.failSafe));
  SET_BIT(state.ledError, L_SET_FAILSAVE);
}

void tstFailSafe(void)                  // Alle 20 ms
{
  uint8_t i;

  for(i = 0;i < 2;++i)
  {
    if(state.groupTimer[i] < 0xff)
      ++state.groupTimer[i];            // Zählt Zeit seit letzter Datenaktualisierung (in 20ms Schritten)
    if(state.groupTimer[i] == 2)        // 2 = 40ms
      if(state.frameLost < 0xffff)      // Wenn nach 40ms keine neue Daten -> Frame verloren
        ++state.frameLost;              // Zählt aber nur einmal, wenn nichts mehr empfangen wird
  }

  for(i = 0;i < MAXCHAN;++i)
  {
    if((!eeprom.failSafe[i].failSafeMode)                       // On oder Hold
      && (state.groupTimer[i / 4] > eeprom.failSafe[i].failSafeDelay + 1)  // Zeit abgelaufen
      && (eeprom.failSafe[i].failSafeDelay != 0xff))      // kein Hold
    {
      uint16_t t = eeprom.failSafe[i].failSafePos;
      if(t & 0x400)                 // Auf zulässige Werte begrenzen
        t |= 0xf800;
      else
        t &= 0x3ff;
      output.chan_1us[i] = t;
    }
  }
}

volatile uint16_t xy;

void copyChan(Message *mes, uint8_t x)
{
  uint8_t i;
  uint16_t high = mes->data.channel.chan_1ushigh;

  i = 3;
  do
  {
    uint8_t f = (uint8_t)high & 7;
    uint16_t val = (f << 8) + mes->data.channel.chan_1uslow[i];
    if(val & 0x400)
      val |= 0xf800;              // erweitern auf 16 Bit
    output.chan_1us[i + x] = val;
    high >>= 3;
  }
  while(i-- > 0);
}

bool readData(void)
{
  static Message mes;

  if(cc2500ReadFIFOBlock((uint8_t *)&mes, sizeof(mes)) && (mes.crcOk))          // CRC ok
  {
    if(mes.data.command.rts)
    {
      if(state.RxCount != 7)
      {
        state.syncError = true;
        state.RxCount = 7;
      }
    }
    else if(state.RxCount >= 7)
      state.RxCount = 6;

    uint8_t type = mes.data.channel.type;
    if(type <= 4)                               // Kanaldaten
    {
      if(!eeprom.chanOff == (type & 2))              // Kanäle 8 - 16 verwenden
      {
        if(!(type & 1))
        {
          copyChan(&mes, 0);
          state.groupTimer[0] = 0;
          uint8_t flgtmp = output.portCflg;
          if(output.pulsesTimer[0] > eeprom.pulsesDelay[0])
            flgtmp |= (1 << OUT_C_CHANNEL1);
          if(output.pulsesTimer[1] > eeprom.pulsesDelay[1])
            flgtmp |= (1 << OUT_C_CHANNEL2);
          if(output.pulsesTimer[2] > eeprom.pulsesDelay[2])
            flgtmp |= (1 << OUT_C_CHANNEL3);
          if(output.pulsesTimer[3] > eeprom.pulsesDelay[3])
            flgtmp |= (1 << OUT_C_CHANNEL4);
          output.portCflg = flgtmp;

//          output.portCflg |= (1 << OUT_C_CHANNEL1) | (1 << OUT_C_CHANNEL2)
//              | (1 << OUT_C_CHANNEL3) | (1 << OUT_C_CHANNEL4);
        }
        else if(type & 1)
        {
          copyChan(&mes, 4);
          state.groupTimer[1] = 0;
          uint8_t flgtmp = output.portCflg;
          if(output.pulsesTimer[4] > eeprom.pulsesDelay[4])
            flgtmp |= (1 << OUT_C_CHANNEL5);
          if(output.pulsesTimer[5] > eeprom.pulsesDelay[5])
            flgtmp |= (1 << OUT_C_CHANNEL6);
          output.portCflg = flgtmp;
          flgtmp = output.portDflg;
          if(output.pulsesTimer[6] > eeprom.pulsesDelay[6])
            flgtmp |= (1 << OUT_D_CHANNEL7);
          if(output.pulsesTimer[7] > eeprom.pulsesDelay[7])
            flgtmp |= (1 << OUT_D_CHANNEL8);
          output.portDflg = flgtmp;


//          output.portCflg |= (1 << OUT_C_CHANNEL5) | (1 << OUT_C_CHANNEL6);
//          output.portDflg = (1 << OUT_D_CHANNEL7) | (1 << OUT_D_CHANNEL8);

        }
      }
      if((type == 4) && (output.pulsesOffset > eeprom.outputOffset + 2)) // obere Gruppe erst ausgeben
        setupPulses(false);
    }
    else if(type == 5)  // eeprom
    {
      if((mes.data.MemoryWord.tar == 0)
          && ((int)mes.data.MemoryWord.adr >= sizeof(BindData))
          && (mes.data.MemoryWord.des == 1))            // Adresse auf 1
        if(mes.data.MemoryWord.wr)                      // schreiben
        {
          if(mes.data.MemoryWord.size)
            eeprom_write_word(mes.data.MemoryWord.adr, mes.data.MemoryWord.data);
          else
            eeprom_write_byte(mes.data.MemoryByte.adr, mes.data.MemoryByte.data);
          eeprom_read_block(&eeprom, 0, sizeof(eeprom));    // Daten wieder auslesen
        }
    }
    else if(type == 7)
    {
      if(mes.data.command.command == 1)     // FailSafe Position setzen
        setFailSafe();
    }
    if(/*(mes.rssi > 50) ||*/ (mes.lqi < 5))   // Empfang schlecht
      setAnt(state.actAnt);               // Antenne wechseln
//    if(state.okSum < 0xffffffff)
    ++state.okSum;                        // Hält bald 100 Tage
    state.errorCount = 0;
    if(state.led)
      LED_ON;
    return(true);
  }
  else
    return(false);
}

uint8_t read, write;
void writeTelemetrie(uint16_t sensor, uint16_t value)
{
  telemetrieBuf[write].Sensor.sensor = sensor;
  telemetrieBuf[write].Sensor.data = value;
  telemetrieBuf[write].Sensor.type = 1;
  telemetrieBuf[write].Sensor.source = 1;
  ++write;
  write &= 0x7;
}

void sendTelemetrie(void)  // wird etwas später gesendet, + 0,5ms
{
  static Telemetrie mes;

//  setNextChanCheckIdle();
  RES_BIT(EIMSK, INT0);                    // INT0 aus

  cc2500CommandStrobe(CC2500_SFTX);             // Sendepuffer leeren
  if(read != write)
  {
    mes.Unspec.dataB1 = telemetrieBuf[read].Unspec.dataB1;
    mes.Unspec.dataB2 = telemetrieBuf[read].Unspec.dataB2;
    mes.Unspec.dataB3 = telemetrieBuf[read].Unspec.dataB3;
    mes.Unspec.dataB4 = telemetrieBuf[read].Unspec.dataB4;
    mes.Unspec.dataB5 = telemetrieBuf[read].Unspec.dataB5;
    mes.Unspec.dataB6 = telemetrieBuf[read].Unspec.dataB6;
    ++read;
    read &= 0x7;
  }
  else                                        // Status senden
  {
    mes.StatusRx.type = 1;
    mes.StatusRx.scanCount = state.scanCount;
    if(state.errorSum >> 16)
      mes.StatusRx.errorSum = 0xffff;
    else
      mes.StatusRx.errorSum = state.errorSum;
    mes.StatusRx.frameLost = state.frameLost;
  }
  if(eeprom.txDisable)
  {
    cc2500CommandStrobe(CC2500_SCAL);
  }
  else
  {
    cc2500WriteFIFOBlock((uint8_t *)&mes, sizeof(mes));
    RES_BIT(PORTB, OUT_B_PRE);
    cc2500CommandStrobe(CC2500_STX);            // Enable TX
  }
}

bool processData(uint8_t data_len)                // Nachsehen ob was da
{
  if(state.bindmode)
  {
    if(data_len == sizeof(MessageBind))
      return(readBindData());
    else
    {
//      setReceiveError();
      //  cc2500FlushReceiveData();           // geht nicht Empfänger läuft schon wieder!?
      while(data_len-- > 0)
        cc2500ReadReg(CC2500_RXFIFO);
      return(false);
    }
  }
  else if(data_len == sizeof(Message))
    return(readData());
  else
  {
//    setReceiveError();
    while(data_len-- > 0)
      cc2500ReadReg(CC2500_RXFIFO);
    return(false);
  }
}

bool checkId(void)
{
  return (eeprom.bind.id && (eeprom.bind.id != 0xffff) &&
         (eeprom.bind.step > 4) && (eeprom.bind.step < MAXHOPPCHAN / 2 - 4));
}

bool checkKey(void)
{
  return (!(PIND & (1 << INP_D_KEY)));
}

void set_led(void)
{
  static uint8_t timer_alt;
  static uint8_t led_count;

  int8_t diff = ((uint8_t)Timer25ms) - timer_alt;
  if(diff > (1000 / 4 / 25))
  {
    timer_alt = (uint8_t)Timer25ms;

    if(!(led_count & 0xf))            // unteres Nibble 0 (Blinkzähler)
    {
      uint8_t ledtemp;
      if((ledtemp = state.ledError))
      {
        while(1)
        {
          led_count += 0x10;
          led_count &= 0x7f;
          if(ledtemp & (1 << (led_count >> 4)))
          {
            led_count |= (led_count >> 3) | 1;      // Blinkzähler setzen
            LED_ON;
            break;
          }
        }
      }
    }
    else
    {
      --led_count;
      if(led_count & 1)
        LED_ON;
      else
        LED_OFF;
      if(!(led_count & 0xf))
        timer_alt += (1000 / 4 / 25 * 3);                    // Pause
    }
    if(!(led_count & 0xf) && !state.ledError)
      state.led = true;
  }
  if(state.led && state.ledError)
  {
    state.led = false;
    LED_OFF;
  }
}

void setTimer(uint8_t value)
{
  OCR2A = value;
//  SET_BIT(TIFR2, OCF2A);
  TIFR2 = (1 << OCF2A);
}

void adjTimer(void)
{
  uint8_t temp = -state.RxTimer;
  TCNT2 += temp;            // Timer initialisieren

  int8_t te = state.RxTimer;
  if(te > state.max)
    state.max = te;
  if(te < state.min)
    state.min = te;
  if(checkKey())
  {
    state.min = 0x7f;
    state.max = -0x80;
  }

  state.RxTimer = 0;
//  if(state.RxTimer > CHANTIME)           // negativ
//  {
//    if(state.pll > -128)
//      --state.pll;
//    else
//      NOP();
//  }
//  else
//    if(state.pll < 127)
//      ++state.pll;
//    else
//      NOP();
}

void setBindMode(void)
{
  cc2500WriteReg(CC2500_SYNC0, (unsigned char)BINDMODEID);
  cc2500WriteReg(CC2500_SYNC1, (unsigned char)(BINDMODEID >> 8));
  cc2500WriteReg(CC2500_PKTLEN, sizeof(BindData));
  cc2500WriteReg(CC2500_AGCCTRL2, 0xfb);              // Empfindlichkeit reduzieren

  SET_BIT(state.ledError, L_BIND_ON);
  eeprom.bind.step = BINDMODESTEP;
}

void rxState(void)
{
  static enum receiver rxstate;
  static uint16_t counter;
  uint8_t cc2500status, frequ, data_len;

  switch(rxstate)
  {
  case Start:
    if(checkKey() || !checkId())
    {
      setBindMode();
      state.bindmode = true;
    }
    else
    {
      cc2500WriteReg(CC2500_PKTLEN, sizeof(MessageData));
      state.bindmode = false;
    }
    calibrateSlow();                   // einmal Kalibrieren beim Wechsel auf RX
    setFreeChanRx();                 // Kanal einstellen und Empfang ein
    counter = 0;
    state.errorCount = 1;
    rxstate = checkRSSI;
    setTimer(CHANTIME);
    break;
  case checkRSSI:                              // Kanal frei?
    if((counter > 100) || checkchanFree()      // lesen von RSSI und Status kann gleich sein
        || (PIND & (1 << INP_D_CC2500_GDO0))
        || (cc2500GetState() != CC2500_STATE_RX)
        || ReceiverInterrupt)
    {
      counter = 0;
      rxstate = waitForData;
      setTimer(CHANTIME2);
    }
    else
    {
      cc2500Idle();
      setFreeChanRx();                 // Neuer Kanal einstellen und Empfang ein
      ++counter;
//      SET_BIT(TIFR2, OCF2A);
      TIFR2 = (1 << OCF2A);
    }
    break;
  case waitForData:                 // Eine Sekunde auf Empfang warten
    if(PIND & (1 << INP_D_CC2500_GDO0))        // Einsprung über Timerinterrupt und Empfang läuft gerade
    {
//      SET_BIT(TIFR2, OCF2A);
      TIFR2 = (1 << OCF2A);
      return;
    }
    cc2500status = SPI_MasterTransmit(CC2500_SNOP | CC2500_READ_SINGLE);
    if((cc2500status & CC2500_STATUS_STATE_BM) != CC2500_STATE_RX)  // nicht mehr auf Empfang
    {
      if(ReceiverInterrupt)
      {
        if((data_len = cc2500status & CC2500_STATUS_FIFO_BYTES_AVAILABLE_BM))
        {                           // irgendwelche Daten da (Prüfsumme und Länge war ok!)
          calibrateFast();
          setFrequencyOffset(cc2500ReadStatusReg(CC2500_FREQEST), false);
          setNextChanGoRx(1);    // Schnell wieder auf Empfang
          if(processData(data_len))            // Empfangsregister auswerten, muss als erstes kommen, wegen Auswertung Telegramm
          {
            adjTimer();
            if(state.RxCount < 7)
              setTimer(CHANTIME + CHANTIME02 + 1);
            else
              setTimer(CHANTIME + TELETIME + 1 + CHANTIME02 + 1);           // Beim ersten Mal nicht senden!!
            state.RxCount = 0;
//            if(state.RxTimer == (uint8_t)-CHANTIME02)           // Timersync hat nicht funktioniert
//              setNextChanGoRx(1);    // Schnell wieder auf Empfang
//            else
//            {
            state.syncError = false;
            rxstate = Main;
            counter = 0;
//            SET_BIT(TIFR2, OCF2A);
            TIFR2 = (1 << OCF2A);
//            }
          }
        }
        else
        {
          setNextChanGoRx(2);
//          setRx();
        }
        ReceiverInterrupt = false;
      }
      else                      // Timerinterrupt
      {                         // eventuell nachschauen ob Syncword empfangen
        cc2500Idle();
        cc2500CommandStrobe(CC2500_SFRX);
        setRx();
//        SET_BIT(TIFR2, OCF2A);
        TIFR2 = (1 << OCF2A);
      }
    }
    else                                // Noch auf Empfang
    {
      if(counter > MAXHOPPCHAN)
      {
        cc2500Idle();
        setNewRxFrequ();                // Frequenz verstellen
        setFreeChanRx();                 // Kanal einstellen und Empfang ein
        counter = 0;
        rxstate = checkRSSI;
        setTimer(CHANTIME);
      }
      else
        ++counter;
//      SET_BIT(TIFR2, OCF2A);
      TIFR2 = (1 << OCF2A);
    }
    break;
  case Main:                                    // Frequenz einstellen und Kalibrieren
    if((state.errorCount > 20) && !ReceiverInterrupt)     // 20 Telegramme nicht empfangen
    {
      cc2500Idle();
      state.actFreqIdx = 7;
      setNewRxFrequ();                // Frequenz auf 0 verstellen
      setNextChanGoRx(3);                 // Kanal einstellen und Empfang ein
      counter = 0;
      state.errorCount = 1;
      rxstate = waitForData;
//      state.synch = false;
      setTimer(CHANTIME2);
      if(state.scanCount < 0xff)
        ++state.scanCount;
    }
    else
    {
      if(ReceiverInterrupt)
      {
        ReceiverInterrupt = false;
        cc2500status = SPI_MasterTransmit(CC2500_FREQEST | CC2500_READ_BURST);
        frequ = SPI_MasterTransmit(CC2500_SNOP);

        if((cc2500status & CC2500_STATUS_STATE_BM) != CC2500_STATE_RX)  // nicht mehr auf Empfang
        {
          if((cc2500status & CC2500_STATUS_STATE_BM) != CC2500_STATE_IDLE)  // Auch nicht auf Idle
            cc2500Idle();                                       // dann jetzt auf Idle
          if((data_len = cc2500status & CC2500_STATUS_FIFO_BYTES_AVAILABLE_BM) // Daten da und es war Idle
              && ((cc2500status & CC2500_STATUS_STATE_BM) == CC2500_STATE_IDLE))
          {
            if(state.errorCount > 3)
              setFrequencyOffset(frequ, false);      // Obwohl noch nicht klar ist ob Daten gut sind!
            else
              setFrequencyOffset(frequ, true);      // Obwohl noch nicht klar ist ob Daten gut sind!
            if(state.RxCount < 7)                   // es kommt noch was
              setNextChanGoRx(1);                      // wechselt auf Rx
            if(processData(data_len))                // state.RxCount wird hier geändert
            {
              adjTimer();                           // Timer initialisieren
              if(state.RxCount >= 7)                // als nächstes kommt Telemetrie
              {
                calibrateSlow();
                rxstate = TxOn;
                state.RxCount = 0;
                setTimer(TELETIME02);   // in 0,6 ms bezogen auf Empfangsinterrupt Timer auslösen
              }                                         // Es kommt dann nur Timerinterrupt weil Idle
              else
              {
                ++state.RxCount;
                setTimer(CHANTIME + CHANTIME02 + 1);
              }		
//              SET_BIT(TIFR2, OCF2A);
              TIFR2 = (1 << OCF2A);
            }
//            else                              // Daten waren Schrott     erst im Timerinterupt
//              setReceiveError();              // Antenne wechseln
              // hier ++state.RxCount ??
          }
          else
          {  // keine Daten oder kein Idle beim Interrupt
            cc2500CommandStrobe(CC2500_SFRX);           // Empfänger leeren
            setRx();                                    // und nochmal versuchen
          }
        }
      }
      else                              // Timerinterrupt
      {                                 // eventuell nachschauen ob Syncword empfangen
//        SET_BIT(TIFR2, OCF2A);
        TIFR2 = (1 << OCF2A);
        cc2500Idle();
        setReceiveError();              // Antenne wechseln
        if(state.RxCount >= 7)
        {
          setTimer(TELETIME - CHANTIME02 + 1);
          calibrateSlow();
          sendTelemetrie();             // Da wir schon später dran sind sofort senden
          rxstate = RxOn;
        }
        else
        {
          ++state.RxCount;
          setNextChanGoRx(1);
	  setTimer(CHANTIME);
        }
      }
    }
    break;
  case TxOn:                      // Daten reinschreiben Sender einschalten und senden
    setTimer(TELETIME08);
    sendTelemetrie();
    rxstate = RxOn;
    ReceiverInterrupt = false;
    break;
  case RxOn:                    // Senden fertig, Empfänger ein
    state.RxCount = 0;
    setTimer(CHANTIME + CHANTIME02 + 1);    // Verschiebung wieder herstellen
//    TimerInterrupt = false;    Macht setTimer
    calibrateFast();
    if(state.syncError)
    {
      setRx();
      state.syncError = false;
    }
    else
      setNextChanGoRx(1);

//    SET_BIT(EIFR, INTF0);
    EIFR = 1 << INTF0;

    SET_BIT(EIMSK, INT0);                    // INT0 ein

    rxstate = Main;
    ReceiverInterrupt = false;
    break;
  }
}

void checkPulses(void)
{
  uint8_t i;

  for(i = 0;i < MAXCHAN;++i)
  {
    ++output.pulsesTimer[i];
    if(output.pulsesTimer[i] > 25)           // Spätestens nach 25 ms Servos ansteuern
    {
      if(!eeprom.failSafe[i].failSafeMode)
      {
        switch(i)
        {
        case 0:
          output.portCflg |= (1 << OUT_C_CHANNEL1);
          break;
        case 1:
          output.portCflg |= (1 << OUT_C_CHANNEL2);
          break;
        case 2:
          output.portCflg |= (1 << OUT_C_CHANNEL3);
          break;
        case 3:
          output.portCflg |= (1 << OUT_C_CHANNEL4);
          break;
        case 4:
          output.portCflg |= (1 << OUT_C_CHANNEL5);
          break;
        case 5:
          output.portCflg |= (1 << OUT_C_CHANNEL6);
          break;
        case 6:
          output.portDflg |= (1 << OUT_D_CHANNEL7);
          break;
        case 7:
          output.portDflg |= (1 << OUT_D_CHANNEL8);
          break;
        }
      }
    }
  }
}

int __attribute__((naked)) main(void)
{
  cli();
  CLKPR = 0;
  PRR = 0;        // Powerreduction für ADC?

  uint8_t i;
  for(i = 0;i < 3;++i)
  {
    if(mcusr_mirror & 1)                // Power on Reset
      resetCounter[i] = 0;              // Alle löschen
    else if(mcusr_mirror & (2 << i))
      ++resetCounter[i];      // 0 -> External Reset, 1 -> Brown-out Reset, 2 -> Watchdog System Reset
  }

  PORTB = (1<<OUT_B_SPI_SS) | (1<<OUT_B_PRE);
  DDRB =  (1<<OUT_B_SPI_MOSI) | (1<<OUT_B_SPI_SCK) | (1<<OUT_B_SPI_SS) | (1<<OUT_B_PRE);
  PORTB = (1<<OUT_B_SPI_SS) | (1<<OUT_B_PRE)
      | ~((1<<OUT_B_SPI_MOSI) | (1<<OUT_B_SPI_SCK) | (1<<OUT_B_SPI_SS) | (1<<OUT_B_PRE));


  DDRC = (1 << OUT_C_CHANNEL1) | (1 << OUT_C_CHANNEL2) | (1 << OUT_C_CHANNEL3) |
         (1 << OUT_C_CHANNEL4) | (1 << OUT_C_CHANNEL5) | (1 << OUT_C_CHANNEL6);
  PORTC = ~((1 << OUT_C_CHANNEL1) | (1 << OUT_C_CHANNEL2) | (1 << OUT_C_CHANNEL3) |
         (1 << OUT_C_CHANNEL4) | (1 << OUT_C_CHANNEL5) | (1 << OUT_C_CHANNEL6));

  DDRD = (1 << OUT_D_LED) | (1 << OUT_D_ANT1) | (1 << OUT_D_ANT2) |
         (1 << OUT_D_CHANNEL7) | (1 << OUT_D_CHANNEL8);
  PORTD = ~((1 << OUT_D_LED) | (1 << OUT_D_ANT1) | (1 << OUT_D_ANT2) |
         (1 << OUT_D_CHANNEL7) | (1 << OUT_D_CHANNEL8)) | (1 << OUT_D_ANT2);

  LED_ON;

// Timer0 25 ms für LED und Failsafe
  TCCR0A = 0;                               //(2 << WGM00);
  TCCR0B = (5 << CS00);                     // clk/1024
  OCR0A = (F_CPU * 10 / 1024 / 400 - 1);
//  TIFR0 = 0xff;
  TIMSK0 = 0;

// Timer1 8MHz   Servoausgänge
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1B = (INTERRUPTOFFSET + 0x3ff) * 8;            // Wenn PPM- Signal ~2,5ms überschreitet ist es ganz schlecht

//  TCCR1B = (1 << WGM12) | (1 << CS10);     // CTC OCR1A, 8MHz
//  TCNT1 = 0;
//  OCR1A = 500 * 8;        // in 500us beginnen


// Timer2 1ms für Timeout
//  TCCR2A = 0;
  TCCR2A = (2 << WGM20);                        //  CTC mode
//  TCCR2B = (3 << CS20);                         // clk/32
  TCCR2B = (6 << CS20);                         // clk/256

  OCR2A  = CHANTIME;
  TCNT2 = 0;
//  TIFR2  = 0xff;
  TIMSK2 = 0;

  wdt_enable(WDTO_500MS);
  EICRA = 1 << ISC01;           // int0 bei fallender Flanke
  EIMSK = 0;

//  EIFR = 0xff;
  state.minAb = 0x17;

  eeprom_read_block(&eeprom, 0, sizeof(eeprom));
  cc2500_Init(0xff);
  if(!checkcc2500())
    SET_BIT(state.ledError, L_INIT_ERROR);
  cc2500WriteReg(CC2500_SYNC0,(unsigned char)eeprom.bind.id);
  cc2500WriteReg(CC2500_SYNC1,(unsigned char)(eeprom.bind.id >> 8));
  set_sleep_mode(SLEEP_MODE_IDLE);
  wdt_enable(WDTO_30MS);
  LED_OFF;
  output.latenzMin = 0xff;

  EIFR = 1 << INTF0;
  SET_BIT(EIMSK, INT0);                    // INT0 ein

  TIFR2 = (1 << OCF2A);
//  SET_BIT(TIFR2, OCF2A);
  ReceiverInterrupt = false;
  sei();
  getFailSafe();                // FailSafe- Werte als Default, damit die Servos bei Einschalten nicht weglaufen
  setupPulses(false);
  while(1){
    cc2500_Off();
    sei();
    //nur bei Timer 1ms Interrupt oder Int0, nicht bei PPM- Interrupt
    if((TIFR2 & (1 << OCF2A)) || ReceiverInterrupt)
    {
      rxState();
//      if(state.errorCount)
//        state.led = false;
    }

    static uint8_t FailSafeTimer;
    if(TIFR0 & (1 << OCF0B)) // 1,024 ms
    {
//      SET_BIT(TIFR0,OCF0B);
      TIFR0 = (1 << OCF0B);
      OCR0B += (F_CPU * 10 / 1024 / 9765);
      ++FailSafeTimer;
      if(FailSafeTimer > 20)
      {
        FailSafeTimer = 0;
        tstFailSafe();
      }
      checkPulses();
      if(++output.pulsesOffset > 25)
        setupPulses(false);
      if(output.pulsesOffset == eeprom.outputOffset)         // Versatz zwischen den Gruppen
        setupPulses(true);

      if((TIFR1 & (1 << OCF1B)) && (state.ppmOverflow < 0xff))
      {
        ++state.ppmOverflow;
//        SET_BIT(TIFR1, OCF1B);
        TIFR1 = (1 << OCF1B);
        ++state.minAb;
      }
    }
    if(TIFR0 & (1 << OCF0A))
    {
      OCR0A += (F_CPU * 10 / 1024 / 400);
      TIFR0 = (1 << OCF0A);
//      SET_BIT(TIFR0,OCF0A);
      ++Timer25ms;
      if(!Timer25ms)
        RES_BIT(state.ledError, L_SET_FAILSAVE);
      set_led();
      if(checkKey())
      {
        state.errorSum = 0;
        state.frameLost = 0;
        state.scanCount = 0;
        state.ppmOverflow = 0;
        state.okSum = 0;
      }
    }
    wdt_reset();
//    if(!TimerInterrupt && !ReceiverInterrupt)
//      sleep_mode();                   //    warten bis was empfangen (Interrupt)
  }
}

