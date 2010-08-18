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
 *

bugs:
todo:
tmain berechnen (min, max)
Achtung: Beim Einschalten kein FailSafe wegen Regler (besser richtigen Mode auswählen)

Sender am Boden ist der Master für das Timing

(Option) Solange kein Telegramm ausfällt, reicht es die geänderten zu Übertragen.
(Option) Immer wieder auch nicht veränderte übertragen. (bei Reset im Empfänger)
         Bei Telemetrie könnte Sender Reset erkennen

Mischbetrieb PPM- Ausgabe und Einzelausgabe schwierig

Synchrones Ausgeben schwierig bei gleichen oder ähnlichen Timerwerten.
  dort leichte Schachtelung nötig

10ms Frame ist nicht zu schaffen (1ms pro Kanal + 2ms Rückkanal)

done:
Ausgabe der Einzelkanäle kann sofort erfolgen wenn Daten empfangen
Bei Empfang Interrupt ein, beim Senden aus
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
//PIN 23  PC0   ADC0/PCINT8                             Servo 8
//PIN 24  PC1   ADC1/PCINT9                             Servo 7
//PIN 25  PC2   ADC2/PCINT10                            Servo 6
//PIN 26  PC3   ADC3/PCINT11                            Servo 5
//PIN 27  PC4   SDA/ADC4/PCINT12                        Servo 4
//PIN 28  PC5   SCL/ADC5/PCINT13                        Servo 3
//PIN 29  PC6   RESET/PCINT14                           Reset
//PIN 30  PD0   RXD/PCINT16                             Servo 2
//PIN 31  PD1   TXD/PCINT17                             Servo 1
//PIN 32  PD2   INT0/PCINT18                            CC2500 GDO0


#define OUT_B_PRE PORTB0
//#define OUT_B_SPI_SS PORTB2
//#define OUT_B_SPI_MOSI PORTB3
//#define INP_B_SPI_MISO PINB4
//#define OUT_B_SPI_SCK PORTB5
#define OUT_D_LED PORTD4
#define OUT_D_ANT1 PORTD6
#define OUT_D_ANT2 PORTD7

#define OUT_D_CHANNEL1 PORTD1             // TXD
#define OUT_D_CHANNEL2 PORTD0             // RXD
#define OUT_C_CHANNEL3 PORTC5
#define OUT_C_CHANNEL4 PORTC4
#define OUT_C_CHANNEL5 PORTC3
#define OUT_C_CHANNEL6 PORTC2
#define OUT_C_CHANNEL7 PORTC1
#define OUT_C_CHANNEL8 PORTC0

//#define INP_D_CC2500_GDO0 PIND2
//#define INP_D_CC2500_GDO2 PIND3
#define INP_D_KEY PIND5

#define LED_OFF RES_BIT(PORTD, OUT_D_LED)
#define LED_ON SET_BIT(PORTD, OUT_D_LED)

#include "ORC_rx.h"

/* EEMEM */ EEData eeprom;
State state;
OutputData output;
//volatile uint16_t Timer1ms;
volatile uint16_t Timer25ms;
bool RxInterrupt;
//uint8_t heartbeat;
PPM pulses[16];
Telemetrie telemetrieBuf[8];
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
uint8_t resetCounter[3] __attribute__ ((section (".noinit")));

void Channel_1_On(void){  SET_BIT(PORTD, OUT_D_CHANNEL1);}
void Channel_2_On(void){  SET_BIT(PORTD, OUT_D_CHANNEL2);}
void Channel_3_On(void){  SET_BIT(PORTC, OUT_C_CHANNEL3);}
void Channel_4_On(void){  SET_BIT(PORTC, OUT_C_CHANNEL4);}
void Channel_5_On(void){  SET_BIT(PORTC, OUT_C_CHANNEL5);}
void Channel_6_On(void){  SET_BIT(PORTC, OUT_C_CHANNEL6);}
void Channel_7_On(void){  SET_BIT(PORTC, OUT_C_CHANNEL7);}
void Channel_8_On(void){  SET_BIT(PORTC, OUT_C_CHANNEL8);}

//void
//_delay_loop_1(uint8_t __count)
//{
//  __asm__ volatile (
//    "1: dec %0" "\n\t"
//    "brne 1b"
//    : "=r" (__count)
//    : "0" (__count)
//  );
//}

//extern void Channel_1_Off(void);
//
//void clearOut(uint8_t i)
//{
//  __asm__ __volatile__ (
//      "ldi     R31,hi8(%0)\n\t"
//      "ldi     R30,lo8(%0)\n\t"
//      "add     %1,%1\n\t"
//      "add     r30,%1\n\t"
//      "ijmp\n\t"
//      :
//      : "i" _SFR_MEM_ADDR((Channel_1_Off)),
//        "r" (i)
//      );
//}

void Channel_1_Off(void){ RES_BIT(PORTD, OUT_D_CHANNEL1);}
void Channel_2_Off(void){ RES_BIT(PORTD, OUT_D_CHANNEL2);}
void Channel_3_Off(void){ RES_BIT(PORTC, OUT_C_CHANNEL3);}
void Channel_4_Off(void){ RES_BIT(PORTC, OUT_C_CHANNEL4);}
void Channel_5_Off(void){ RES_BIT(PORTC, OUT_C_CHANNEL5);}
void Channel_6_Off(void){ RES_BIT(PORTC, OUT_C_CHANNEL6);}
void Channel_7_Off(void){ RES_BIT(PORTC, OUT_C_CHANNEL7);}
void Channel_8_Off(void){ RES_BIT(PORTC, OUT_C_CHANNEL8);}
//
//void set_Channel_1(void){
//  RES_BIT(PORTC, OUT_C_CHANNEL8);
//  SET_BIT(PORTD, OUT_D_CHANNEL1);
//}
//
//void set_Channel_1_Q(void){
//  RES_BIT(PORTC, OUT_C_CHANNEL8);
//  SET_BIT(PORTD, OUT_D_CHANNEL1);
//  SET_BIT(PORTC, OUT_C_CHANNEL8);
//}
//
//void set_Channel_2(void){
//  RES_BIT(PORTD, OUT_D_CHANNEL1);
//  SET_BIT(PORTD, OUT_D_CHANNEL2);
//}
//
//void set_Channel_2_Q(void){
//  RES_BIT(PORTD, OUT_D_CHANNEL1);
//  SET_BIT(PORTD, OUT_D_CHANNEL2);
//  SET_BIT(PORTC, OUT_C_CHANNEL8);
//}
//
//void set_Channel_3(void){
//  RES_BIT(PORTD, OUT_D_CHANNEL2);
//  SET_BIT(PORTC, OUT_C_CHANNEL3);
//}
//
//void set_Channel_3_Q(void){
//  RES_BIT(PORTD, OUT_D_CHANNEL2);
//  SET_BIT(PORTC, OUT_C_CHANNEL3);
//  SET_BIT(PORTC, OUT_C_CHANNEL8);
//}
//
//void set_Channel_4(void){
//  RES_BIT(PORTC, OUT_C_CHANNEL3);
//  SET_BIT(PORTC, OUT_C_CHANNEL4);
//}
//
//void set_Channel_4_Q(void){
//  RES_BIT(PORTC, OUT_C_CHANNEL3);
//  SET_BIT(PORTC, OUT_C_CHANNEL4);
//  SET_BIT(PORTC, OUT_C_CHANNEL8);
//}
//
//void set_Channel_5(void){
//  RES_BIT(PORTC, OUT_C_CHANNEL4);
//  SET_BIT(PORTC, OUT_C_CHANNEL5);
//}
//
//void set_Channel_5_Q(void){
//  RES_BIT(PORTC, OUT_C_CHANNEL4);
//  SET_BIT(PORTC, OUT_C_CHANNEL5);
//  SET_BIT(PORTC, OUT_C_CHANNEL8);
//}
//
//void set_Channel_6(void){
//  RES_BIT(PORTC, OUT_C_CHANNEL5);
//  SET_BIT(PORTC, OUT_C_CHANNEL6);
//}
//
//void set_Channel_6_Q(void){
//  RES_BIT(PORTC, OUT_C_CHANNEL5);
//  SET_BIT(PORTC, OUT_C_CHANNEL6);
//  SET_BIT(PORTC, OUT_C_CHANNEL8);
//}
//
//void set_Channel_7(void){
//  RES_BIT(PORTC, OUT_C_CHANNEL6);
//  SET_BIT(PORTC, OUT_C_CHANNEL7);
//}
//
//void set_Channel_7_Q(void){
//  RES_BIT(PORTC, OUT_C_CHANNEL6);
//  SET_BIT(PORTC, OUT_C_CHANNEL7);
//  SET_BIT(PORTC, OUT_C_CHANNEL8);
//}
//
//void set_Channel_8(void){
//  RES_BIT(PORTC, OUT_C_CHANNEL7);
//  SET_BIT(PORTC, OUT_C_CHANNEL8);
//}
//
//void set_Channel_Off(void){
//  RES_BIT(PORTC, OUT_C_CHANNEL8);
//}
//
//void set_Channel_8_Q(void){
//  RES_BIT(PORTC, OUT_C_CHANNEL7);
//  RES_BIT(PORTC, OUT_C_CHANNEL8);
//  SET_BIT(PORTC, OUT_C_CHANNEL8);
//}
//
//FuncP_PROGMEM APM set_ChannelOut[] = {
//  set_Channel_1,
//  set_Channel_2,
//  set_Channel_3,
//  set_Channel_4,
//  set_Channel_5,
//  set_Channel_6,
//  set_Channel_7,
//  set_Channel_8,
//  set_Channel_Off
//};
//
//FuncP_PROGMEM APM set_ChannelOutQ[] = {
//  set_Channel_1_Q,
//  set_Channel_2_Q,
//  set_Channel_3_Q,
//  set_Channel_4_Q,
//  set_Channel_5_Q,
//  set_Channel_6_Q,
//  set_Channel_7_Q,
//  set_Channel_8_Q,
//  set_Channel_Off
//};

FuncP_PROGMEM APM set_Channel_On[] = {
  Channel_1_On,
  Channel_2_On,
  Channel_3_On,
  Channel_4_On,
  Channel_5_On,
  Channel_6_On,
  Channel_7_On,
  Channel_8_On
};

FuncP_PROGMEM APM set_Channel_Off[] = {
  Channel_1_Off,
  Channel_2_Off,
  Channel_3_Off,
  Channel_4_Off,
  Channel_5_Off,
  Channel_6_Off,
  Channel_7_Off,
  Channel_8_Off
};

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

ISR(TIMER2_COMPA_vect, ISR_NOBLOCK)
{
//  ++Timer1ms;
  RxInterrupt = true;
}

ISR(INT0_vect, ISR_NOBLOCK)
{
  RxInterrupt = true;
//  RES_BIT(EIMSK, INT0);                    // INT0 aus, dient nur zum wecken
}

/**********************************/
FASTPPM ppm[8];             // Dieses Sortieren

void test(void)                       // alle 20 ms
{
  uint8_t i, y;
  for(i = 0;i < 8;++i)
  {
    switch(i)
    {
      case 0:
        ppm[i].maskPortC = ~0;
        ppm[i].maskPortD = ~(1 << OUT_D_CHANNEL1);
        break;
      case 1:
        ppm[i].maskPortC = ~0;
        ppm[i].maskPortD = ~(1 << OUT_D_CHANNEL2);
        break;
      case 2:
        ppm[i].maskPortC = ~(1 << OUT_C_CHANNEL3);
        ppm[i].maskPortD = ~0;
        break;
      case 3:
        ppm[i].maskPortC = ~(1 << OUT_C_CHANNEL4);
        ppm[i].maskPortD = ~0;
        break;
      case 4:
        ppm[i].maskPortC = ~(1 << OUT_C_CHANNEL5);
        ppm[i].maskPortD = ~0;
        break;
      case 5:
        ppm[i].maskPortC = ~(1 << OUT_C_CHANNEL6);
        ppm[i].maskPortD = ~0;
        break;
      case 6:
        ppm[i].maskPortC = ~(1 << OUT_C_CHANNEL7);
        ppm[i].maskPortD = ~0;
        break;
      case 7:
        ppm[i].maskPortC = ~(1 << OUT_C_CHANNEL8);
        ppm[i].maskPortD = ~0;
        break;
    }
    ppm[i].raw = output.chan_1us[i];
  }
  for(i = 0;i < 7;++i)            // qsort
  {
    for(y = i + 1;y < 8;++y)
    {
      if((ppm[i].raw > ppm[y].raw) || ((ppm[i].raw == 0) && (ppm[y].raw != 0)))
      {
        uint16_t temp = ppm[i].raw;
        ppm[i].raw = ppm[y].raw;
        ppm[y].raw = temp;
        uint8_t ctemp = ppm[i].maskPortC;
        ppm[i].maskPortC = ppm[y].maskPortC;
        ppm[y].maskPortC = ctemp;
        ctemp = ppm[i].maskPortD;
        ppm[i].maskPortD = ppm[y].maskPortD;
        ppm[y].maskPortD = ctemp;
      }
    }
  }
  for(i = 0;i < 7;++i)            // Gleiche zusammenfassen
  {
    if(ppm[i].raw == ppm[i + 1].raw)
    {
      ppm[i + 1].maskPortC &= ppm[i].maskPortC;
      ppm[i + 1].maskPortD &= ppm[i].maskPortD;
      ppm[i].raw = 0;
    }
    ppm[i].nextdelay = ppm[i + 1].raw - ppm[i].raw;
    if(ppm[i].nextdelay > 10)
      ppm[i].nextdelay = 0;
  }
  ppm[7].nextdelay = 0;
  for(i = 0;i < 7;++i)            // Rest dahinter, herschieben
  {
    if(ppm[i].raw == 0)
    {
      for(y = i + 1;y < 8;++y)
      {
        if(ppm[y].raw == 0)
          continue;
        ppm[i].raw = ppm[y].raw;
        ppm[i].maskPortC = ppm[y].maskPortC;
        ppm[i].maskPortD = ppm[y].maskPortD;
        ppm[y].raw = 0;
        ppm[y].nextdelay = 0;
        break;
      }
    }
  }
  output.p = ppm;
  TCCR1B = (4 << WGM10);      // Timer aus
  TCNT1 = 0;
  OCR1A = (ppm[0].raw  + 1500u) * 8;
  SET_BIT(TIFR1, OCF1A);
  SET_BIT(TIMSK1, OCIE1A);          // Timer Interrupt Mask Register
  TCCR1B = (4 << WGM10) | (1 << CS10);      // Timer an
  PORTD |= (1 << OUT_D_CHANNEL1) | (1 << OUT_D_CHANNEL2);
  NOP();
  PORTC |= (1 << OUT_C_CHANNEL3) | (1 << OUT_C_CHANNEL4) | (1 << OUT_C_CHANNEL5)
         | (1 << OUT_C_CHANNEL6) | (1 << OUT_C_CHANNEL7) | (1 << OUT_C_CHANNEL8);
}

uint8_t uuu[3*8];

void inter(void)
{
  uint8_t *pp = uuu;
  uint8_t z;

  for(z = 0;z < 8;++z)
  {
    *pp++ &= PORTD;
    *pp++ &= PORTC;
  }
  z = 8;
  pp = uuu;

  while(1)
  {
    do
    {
      PORTD = *pp++;
      PORTC = *pp++;
    }
    while(!(*(pp++ - 1) & 0xc0));     // sieht heiß aus
    uint8_t waittemp = *(pp - 1);
    if(waittemp > 20)
      break;
    if(waittemp == 1)
      continue;
    do
    {
      --waittemp;
      NOP();
      NOP();
    }
    while(waittemp);
  }
  // 01 1µs warten
  // 10 2µs warten
  // 11 3-20µs warten

/* Abstand 1µs mit Tabelle kein Problem
 * Abstand 2µs mit Tabelle (32 Einträge)
 * Abstand 3µs schwierig weil wenig Zeit für neue Tabelle
 *
 * bei 20µs könnte Abbruch Interrupt reichen
 */

  FASTPPM *p = output.p;

  while(1)
  {
    PORTD &= p->maskPortD;
    PORTC &= p->maskPortC;
    uint8_t dif = p->nextdelay;
    p++;
    if(dif == 1)
      continue;
    if(!dif)
      break;
    while(dif > 1)
    {
      NOP();
      NOP();
      NOP();
      NOP();
      --dif;
    }
  }
  if(p->raw == 0)
    TIMSK1 &= ~(1<<OCIE1A);           // Interrupt aus
  else
    OCR1A = (p->raw  + 1500u) * 8;
  output.p = p;
}

//ISR(TIMER1_COMPA_vect)                  //8MHz pulse generation
//{
//  register uint8_t i, channel;
//
//  channel = output[].chanPtr;                    // Kanal als lokaler Registerwert
//  (*set_ChannelOutQ[channel])();
//  while(delaykanal++)
//  {
//  delayµs(delay)
//  (*set_ChannelOutQ[channel])();
//  }
//  neuer Wert einstellen
//  sei();
//
//  OCR1A  = output.pulses2MHz[channel++];
//
//  if(output.pulses2MHz[channel] == 0)
//  {
//    TIMSK1 &= ~(1<<OCIE1A);                   // Interrupt aus
//    OCR1A = 5000u * 8u;                       // 5ms Pause
//    set_sleep_mode(SLEEP_MODE_STANDBY);       // Wenn Timer1 fertig
//  }
//  output.chanPtr = channel;
////  heartbeat |= HEART_TIMER2Mhz;
//}

ISR(TIMER1_COMPA_vect)                  //8MHz pulse generation
{
//  TIMSK1 &= ~(1<<OCIE1A);                   // Interrupt aus

//  while(TCNT1L < 25);
  uint8_t act = output.act;
  (*pulses[act].function)();

  sei();

  uint8_t c = TCNT1L;
  if(output.latenzMin > c)
    output.latenzMin = c;
  if(output.latenzMax < c)
    output.latenzMax = c;

//  uint8_t nextFree = output.nextFree;
//  uint8_t temp_chan;
//  if((temp_chan = searchChan()))
//  {
//    --temp_chan;
//    pulses[nextFree].function = set_Channel_On[temp_chan];
//    pulses[nextFree].timer = 200 * 8;
//    ++nextFree;
//    nextFree &= 0xf;
//    pulses[nextFree].function = set_Channel_Off[temp_chan];
//    pulses[nextFree].timer = (output.chan_1us[temp_chan] + 1500u) * 8 ;
//    ++nextFree;
//    nextFree &= 0xf;
//    output.chanFlag &= ~(1 << temp_chan);
//  }
  ++act;
  act &= 0xf;
  if(act == output.nextFree)
    TIMSK1 &= ~(1<<OCIE1A);                   // Interrupt aus
  else
    OCR1A = pulses[act].timer;
  output.act = act;
}

void setupPulses(uint8_t chan)              // Pulse in Warteschlange einfügen
{
  if(output.timer[chan] < 20)                   // Ausgänge mit minimal 20ms Abstand
    return;
  uint8_t nextFree = output.nextFree;
  pulses[nextFree].function = (void (*)())pgm_read_word(&set_Channel_On[chan]);
  pulses[nextFree].timer = 200;             // ca. 200 Befehle Abstand für Interrupts (25us)
  ++nextFree;
  nextFree &= 0xf;
  pulses[nextFree].function = (void (*)())pgm_read_word(&set_Channel_Off[chan]);
  pulses[nextFree].timer = (output.chan_1us[chan] + 1500u) * 8 ;
  ++nextFree;
  nextFree &= 0xf;
  output.chanFlag &= ~(1 << chan);
  output.nextFree = nextFree;       // gefahrlos da atomic

  if(!(TIMSK1 & (1<<OCIE1A)))        // Ist doch schon aus
  {
    TCNT1 = 0;
    OCR1A = pulses[output.act].timer;
    SET_BIT(TIFR1, OCF1A);
    SET_BIT(TIMSK1, OCIE1A);          // Timer Interrupt Mask Register
  }
  output.timer[chan] = 0;
}

//void setupPulsesPPM_quad(void)
//{
//  if(!(TIFR1 & (1<<OCF1A)))
//  {
//    return;               // Noch gar nicht fertig
//  }
//
//  OCR1A = 500 * 8;        // in 500us beginnen
//  TCNT1 = 0;
//
////  sei();
//  uint8_t j=0;
//  output.pulses2MHz[j++]=(uint16_t)300 * 8;
//  for(uint8_t i=0;i<8;i++)
//  {
//    output.pulses2MHz[j++] = (output.chan_1us[i] - 300 + 1500u) * 8;
//    output.pulses2MHz[j++]=(uint16_t)300 * 8;
//  }
//  output.pulses2MHz[j]=0;
//  output.chanPtr = 0;
//  TIFR1 &= ~(1<<OCF1A);
//  TIMSK1 |= (1<<OCIE1A);      // Timer Interrupt Mask Register
////  set_sleep_mode(SLEEP_MODE_IDLE);
//}
//
//void setupPulsesPPM()
//{
//  if(!(TIFR1 & (1<<OCF1A)))
//  {
//    return;               // Noch gar nicht fertig
//  }
//
//  OCR1A = 500 * 8;        // in 500us beginnen (je nach Kanal)
//  TCNT1 = 0;
//
////  sei();
//  uint8_t j=0;
//  for(uint8_t i = 0;i < 8;i++)
//    output.pulses2MHz[j++] = (output.chan_1us[i] + 1500u) * 8;
//  output.pulses2MHz[j] = 0;
//  output.chanPtr = 0;
//  TIFR1 &= ~(1<<OCF1A);
//  TIMSK1 |= (1<<OCIE1A);          // Timer Interrupt Mask Register
////  set_sleep_mode(SLEEP_MODE_IDLE);
//}

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

void setNextChan(void)                // Kanal schreiben
{
  uint16_t tempChan = state.actChan + eeprom.bind.step * 2 + 1;
  while(tempChan > MAXHOPPCHAN)
    tempChan -= (MAXHOPPCHAN + 1);
  state.actChan = tempChan;
  cc2500WriteReg(CC2500_CHANNR, tempChan);
}

void setNextChanRx(void)                // Kanal schreiben
{
  if(PIND & (1<<INP_D_CC2500_GDO0))
    SET_BIT(state.ledError, L_TX_NOT_READY);
  setNextChan();
  SET_BIT(PORTB, OUT_B_PRE);
  cc2500CommandStrobe(CC2500_SRX);       // Kalibrieren
  SET_BIT(EIFR, INTF0);
  SET_BIT(EIMSK, INT0);                    // INT0 ein
}

void setPaketsizeReceive(void)
{
  cc2500WriteReg(CC2500_PKTLEN, sizeof(MessageData));
}

void setPaketsizeSend(void)
{
  cc2500WriteReg(CC2500_PKTLEN, sizeof(Telemetrie));
}

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
  int8_t rssi = cc2500ReadStatus(CC2500_RSSI);
//  cc2500_Off();
  return(rssi > 0);
}

void setFreeChanRx(void)            // Nächsten Kanal einstellen um zu sehen, ob er belegt ist
{
  uint16_t tempChan = state.actChan + 1;
  if(tempChan > MAXHOPPCHAN)
    tempChan -= (MAXHOPPCHAN + 1);
  cc2500Idle();
  cc2500WriteReg(CC2500_CHANNR, tempChan);
  SET_BIT(PORTB, OUT_B_PRE);
  cc2500CommandStrobe(CC2500_SRX);
  state.actChan = tempChan;
}

prog_int8_t APM freq[] = {0, -120, +120, +40, -40, -80, 80};

void setNewRxFrequ(void)
{
  setAnt(state.actAnt);               // Antenne wechseln
  if(++state.actFreqIdx >= 7)
    state.actFreqIdx = 0;
  cc2500WriteReg(CC2500_FSCTRL0, pgm_read_byte(&freq[state.actFreqIdx]));
}

void setReceiveError(void)
{
  setAnt(state.actAnt);               // Antenne wechseln
  if(state.errorCount < 0xff)
    ++state.errorCount;
  if(state.errorSum < 0xffff)
    ++state.errorSum;                 // max über 3h Totalausfall
//  cc2500FlushReceiveData();           // geht nicht Empfänger läuft schon wieder!?
  uint8_t i;
  uint8_t a = get_RxCount();
  for(i = 0;i < a;++i)
    cc2500ReadReg(CC2500_RXFIFO);
}

void clearCount(void)
{
  TCNT2 = 0;                          // Timer kommt eher später
}

void readBindData(void)
{
  MessageBind mes;
  static uint8_t counter;

  cc2500ReadFIFOBlock((uint8_t *)&mes, sizeof(mes));
  setFrequencyOffset();                 // macht nicht viel Sinn, hat ja funktioniert
  if(mes.crcOk)
  {
    clearCount();
    if(counter)
    {
      if((eeprom.bind.id == mes.data.id) && (eeprom.bind.step == mes.data.step))
        ++counter;
      else
        counter = 0;
      if(counter > 100)
      {
        eeprom_write_block(&eeprom.bind, 0 ,sizeof(eeprom.bind));
        state.bindmode = false;
        while(1);                   //       reset
      }
    }
    else
    {
      eeprom.bind.id = mes.data.id;
      eeprom.bind.step = mes.data.step;
      ++counter;
    }
  }
  else
    setReceiveError();
}

void getFailSafe(void)
{
  uint8_t i;
  for(i = 0;i < MAXCHAN;++i)
    output.chan_1us[i] = eeprom.failSafe[i].failSafePos;
}

void setFailSafe(void)
{
  uint8_t i;
  for(i = 0;i < MAXCHAN;++i)
    eeprom.failSafe[i].failSafePos = output.chan_1us[i];
  eeprom_write_block(&eeprom.failSafe,(uint8_t *)((int)&eeprom.failSafe - (int)&eeprom) ,sizeof(eeprom.failSafe));
  SET_BIT(state.ledError, L_SET_FAILSAVE);
}

void tstFailSafe(void)
{
  uint8_t i;
  for(i = 0;i < MAXCHAN;++i)
  {
    if(eeprom.failSafe[i].failSafeMode)
    {
      if((eeprom.failSafe[i].failSafeDelay != 0xff) && (output.timeOut[i] > eeprom.failSafe[i].failSafeDelay + 1))
        output.chan_1us[i] = eeprom.failSafe[i].failSafePos;
      if(output.timeOut[i] > 1)
        setupPulses(i);
      if(output.timeOut[i] < 0xff)
        ++output.timeOut[i];
    }
  }
}

void setOutputTimer(void)
{
  uint8_t i;
  for(i = 0;i < MAXCHAN;++i)
    if(output.timer[i] < 0xff)
      ++output.timer[i];
}

void readData(void)
{
//  uint8_t data, lqi, chan, id;
//  int8_t rssi;
  Message mes;
  static uint8_t chanOld;

  cc2500ReadFIFOBlock((uint8_t *)&mes, sizeof(mes));
  if(mes.crcOk)          // CRC ok
  {
    clearCount();
    if(!mes.data.command.mode)
    {
      uint8_t chan = mes.data.channel.channel;
      if(!eeprom.chanOff)
        chan -= MAXCHAN;                  // Kanäle 8 - 16 verwenden
      if(chan < MAXCHAN)
      {
        if(output.chanMax < chan)
          output.chanMax = chan;

        if(chanOld > chan)                        // Ein Durchlauf fertig
          output.chanFlag = 0;
        chanOld = chan;

        output.chan_1us[chan] = mes.data.channel.chan_1us;
        output.timeOut[chan] = 0;
        output.chanFlag |= (1 << chan);
//        if(output.chanMax > chan)
//          setupPulsesPPM_quad();                  // Wenn alles gut war, Ausgänge aktivieren
        setupPulses(chan);
        //???
        //output.chanMax = chan;                // Einmal muss es gehen, wegen init
      }
    }
    else
    {
      if(mes.data.command.rts)
        state.RxCount = 8;
      if(mes.data.command.command == 1)     // FailSafe Position setzen
        setFailSafe();
      else
        RES_BIT(state.ledError, L_SET_FAILSAVE);
    // Achtung auch bei Ausfall des letzten Telegramms PPM erzeugen
        // Achtung je nach Kanal Latenz ausgleichen!!!
    }
    setFrequencyOffset();
    if((mes.rssi > 50) || (mes.lqi < 5))   // Empfang schlecht
      setAnt(state.actAnt);               // Antenne wechseln
    if(state.okSum < 0xffff)
      ++state.okSum;
    state.errorCount = 0;
  }
  else
    setReceiveError();
}

uint8_t read, write;
void writeTelemetrie(uint16_t sensor, uint16_t value)
{
  telemetrieBuf[write].sensor = sensor;
  telemetrieBuf[write].data = value;
  ++write;
  write &= 0x7;
}

void sendTelemetrie(void)  // Da Timer hinterherhinkt wird etwas später gesendet
{
  Telemetrie mes;
  static uint8_t count;

  setNextChan();
  if(!eeprom.txEnable)
    return;
  cc2500CommandStrobe(CC2500_SFTX);      // ??
  if(read != write)
  {
    mes.sensor = telemetrieBuf[read].sensor;
    mes.data = telemetrieBuf[read].data;
    ++read;
    read &= 0x7;
  }
  else                                        // Status senden
  {
    mes.sensor = 0;
    mes.data = state.scanCount | count++ * 0x100;
  }
  cc2500WriteFIFOBlock((uint8_t *)&mes, sizeof(mes));
  RES_BIT(EIMSK, INT0);                    // INT0 aus
  RES_BIT(PORTB, OUT_B_PRE);
  cc2500CommandStrobe(CC2500_STX);            // Enable TX
}

void processData(void)                // Nachsehen ob was da
{
  uint8_t data_lengh;

  if((data_lengh = get_RxCount()))
  {
//    TIFR2 &= ~(1 << OCF2A);
    if(state.bindmode)
    {
      if(data_lengh == sizeof(MessageBind))
        readBindData();
      else
        setReceiveError();
    }
    else if(data_lengh == sizeof(Message))
      readData();
    else
      setReceiveError();
  }
  else
    setReceiveError();
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
  static uint16_t timer_alt;
  static uint8_t led_count;
  uint16_t timerTemp;

  timerTemp = Timer25ms;
  int16_t diff = timerTemp - timer_alt;
  if(diff > 5)
  {
    timer_alt = timerTemp;

    if(!(led_count & 0xf))            // unteres Nibble 0
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
      else
        LED_ON;
    }
    else
    {
      --led_count;
      if(led_count & 1)
        LED_ON;
      else
        LED_OFF;
      if(!(led_count & 0xf))
        timer_alt += 25;                    // Pause
    }
  }
}

void setBindMode(void)
{
  cc2500WriteReg(CC2500_SYNC0, (unsigned char)BINDMODEID);
  cc2500WriteReg(CC2500_SYNC1, (unsigned char)(BINDMODEID >> 8));
  cc2500WriteReg(CC2500_PKTLEN, sizeof(BindData));
  SET_BIT(state.ledError, L_BIND_ON);
}

void rxState(void)
{
  static enum receiver rxstate;
  static uint16_t counter;

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
      setPaketsizeReceive();
      state.bindmode = false;
    }
    calibrateOn();
    setFreeChanRx();                 // Kanal einstellen und Empfang ein
    counter = 0;
    state.errorCount = 1;
    rxstate = RxWaitStart;
    setOutputTimer();
    break;
  case RxWaitStart:                  // auf Empfänger warten
    rxstate = checkRSSI;
    setOutputTimer();
    break;
  case checkRSSI:                   // Kanal frei?
    if((counter > 100) || checkchanFree() ||
        (PIND & (1 << INP_D_CC2500_GDO0)) || get_RxCount())
    {
      counter = 0;
      rxstate = waitForData;
    }
    else
    {
      setFreeChanRx();                 // Neuer Kanal einstellen und Empfang ein
      ++counter;
      rxstate = RxWaitStart;
    }
    setOutputTimer();
    break;
  case waitForData:                 // Eine Sekunde auf Empfang warten
    if(PIND & (1 << INP_D_CC2500_GDO0))        // Einsprung über Timerinterrupt und Empfang läuft gerade
      return;
    if(get_RxCount())                           // irgendwelche Daten da
    {
      calibrateOff();
      setNextChanRx();                  // Beim Binden wird hier Müll eingestellt
      processData();                  // Empfangsregister auswerten, muss als erstes kommen, wegen Auswertung Telegramm
      if(state.RxCount < 8)
      {
        rxstate = RxWait;             // Beim ersten Mal nicht senden!!
        state.RxCount = 0;
      }
      counter = 0;
    }
    else
    {
      if(counter % 128 == 127)
        setNewRxFrequ();                // Frequenz verstellen
      if(counter > 1000)                  // 1 Sekunde warten
      {                                 // Neu initialisieren ??
        setFreeChanRx();                 // Kanal einstellen und Empfang ein
        counter = 0;
        rxstate = RxWaitStart;
      }
      else
        ++counter;
    }
    setOutputTimer();
    break;
  case Main:                                    // Frequenz einstellen und Kalibrieren
    if(PIND & (1 << INP_D_CC2500_GDO0))        // Einsprung über Timerinterrupt und Empfang läuft gerade
      return;
    if(state.errorCount > 200)       // 200 Telegramme nicht empfangen
    {
      setFreeChanRx();                 // Kanal einstellen und Empfang ein
      counter = 0;
      state.errorCount = 1;
      rxstate = RxWaitStart;
      if(state.scanCount < 0xff)
        ++state.scanCount;
    }
    else
    {
      if(state.RxCount < 8)
      {
        setNextChanRx();
        rxstate = RxWait;
      }
      processData();                // state.RxCount wird hier geändert
      if(state.RxCount >= 8)
      {
        setPaketsizeSend();
        calibrateOn();
        setNextChan();
        rxstate = TxOn;
        state.RxCount = 0;
        OCR2A  = CYCLETIME / 2;               // ergibt 0,5 ms
      }
    }
    setOutputTimer();
    break;
  case RxWait:                      // Daten empfangen
    rxstate = Main;
    ++state.RxCount;
    setOutputTimer();
    break;
  case TxOn:                      // Daten reinschreiben Sender einschalten und senden
    sendTelemetrie();
    rxstate = TxWait;
    break;
  case TxWait:                  // Jetzt wird weitergesendet
    OCR2A  = CYCLETIME;               // ergibt 1ms (1000 Hz)
    rxstate = TxWait2;
    setOutputTimer();
    break;
  case TxWait2:                  // Jetzt wird weitergesendet
    rxstate = RxOn;
    setOutputTimer();
    break;
  case RxOn:                    // Senden fertig, Empfänger ein
    setPaketsizeReceive();
    calibrateOff();
    setNextChanRx();
    rxstate = RxWait;
    setOutputTimer();
    break;
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
    if(mcusr_mirror & 1)     // Power on Reset
      resetCounter[i] = 0;
    else if(mcusr_mirror & (2 << i))
      ++resetCounter[i];
  }

  PORTB = (1<<OUT_B_SPI_SS) | (1<<INP_B_SPI_MISO) | (1<<OUT_B_PRE) | (1 << PORTB1);
  DDRB =  (1<<OUT_B_SPI_MOSI) | (1<<OUT_B_SPI_SCK) | (1<<OUT_B_SPI_SS) | (1<<OUT_B_PRE);

  PORTC = 0x0;
  DDRC = (1 << OUT_C_CHANNEL3) | (1 << OUT_C_CHANNEL4) | (1 << OUT_C_CHANNEL5) |
         (1 << OUT_C_CHANNEL6) | (1 << OUT_C_CHANNEL7) | (1 << OUT_C_CHANNEL8);

  PORTD = 1 << INP_D_KEY;
  DDRD = (1 << OUT_D_LED) | (1 << OUT_D_ANT1) | (1 << OUT_D_ANT2) |
         (1 << OUT_D_CHANNEL1) | (1 << OUT_D_CHANNEL2);

  LED_ON;

// Timer0 25 ms für LED und Failsafe
  TCCR0A = (2 << WGM00);
  TCCR0B = (5 << CS00);                     // clk/1024
  OCR0A = (F_CPU * 10 / 1024 / 400 - 1);
//  TIFR0 = 0xff;
  TIMSK0 = 0;

// Timer1 8MHz   Servoausgänge
//  TCCR1A = 0;
  TCCR1B = (4 << WGM10) | (1 << CS10);      // CTC OCR1A, 8MHz
  TCNT1 = 0;
  OCR1A = 500 * 8;        // in 500us beginnen


// Timer2 1ms für Timeout
//  TCCR2A = 0;
  TCCR2A = (2 << WGM20);                        //  CTC mode
//  TCCR2B = (3 << CS20);                         // clk/32
  TCCR2B = (4 << CS20);                         // clk/32  (3 << CS20)

  OCR2A  = CYCLETIME;                           // ergibt 1ms (1000 Hz)
  TCNT2 = 0;
//  TIFR2  = 0xff;
  TIMSK2 = 1 << OCIE2A;

//  wdt_enable(WDTO_500MS);
  EICRA = 1 << ISC01;           // int0 bei fallender Flanke
  EIMSK = 0;
//  EIFR = 0xff;

  eeprom_read_block(&eeprom, 0, sizeof(eeprom));
  cc2500_Init(0xff);
  if(!checkcc2500())
    SET_BIT(state.ledError, L_INIT_ERROR);
  cc2500WriteReg(CC2500_SYNC0,(unsigned char)eeprom.bind.id);
  cc2500WriteReg(CC2500_SYNC1,(unsigned char)(eeprom.bind.id >> 8));
  set_sleep_mode(SLEEP_MODE_IDLE);
  getFailSafe();
//  wdt_enable(WDTO_30MS);
  LED_OFF;
  sei();
  while(1){
    cc2500_Off();
    sei();
    sleep_mode();                   //    warten bis was empfangen (Interrupt)
    //nur bei Timer 1ms Interrupt oder Int0, nicht bei PPM- Interrupt
    if(RxInterrupt)
    {
      rxState();
      RxInterrupt = false;
    }
    if(TIFR0 & (1 << OCF0A))
    {
      ++Timer25ms;
      SET_BIT(TIFR0,OCF0A);
      set_led();
      tstFailSafe();
    }
    wdt_reset();
//  test();
//  inter();
  }
}

