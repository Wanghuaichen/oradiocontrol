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

Probleme:
bugs:
todo:

FAQ:
Was passiert wenn kein PPM- Frame kommt?
Sender bleibt stehen

done:

 */

//PB0
//PB1 -
//PB2 SS (SPI Bus Master Slave select)
//PB3 MOSI (SPI Bus Master Output/Slave Input)
//PB4 MISO (SPI Bus Master Input/Slave Output)
//PB5 SCK (SPI Bus Master clock Input)
//PB6 OSC
//PB7 OSC

//PC0 Servo 8
//PC1 Servo 7
//PC2 Servo 6
//PC3 Servo 5
//PC4 Servo 4
//PC5 Servo 3
//PC6 RESET


//PD0 Servo 2
//PD1 Servo 1
//PD2 GDO0
//PD3 GDO2
//PD4 LED
//PD5 Taster
//PD6 Ant
//PD7

#define OUT_B_SPI_SS PORTB2
#define OUT_B_SPI_MISO PORTB4
#define OUT_B_SPI_SCK PORTB5
#define OUT_D_LED PORTD4
#define OUT_D_ANT1 PORTD6
#define OUT_D_ANT2 PORTD7               // Antenne 2????

#define OUT_D_CANAL1 PORTD1
#define OUT_D_CANAL2 PORTD2
#define OUT_C_CANAL3 PORTC5
#define OUT_C_CANAL4 PORTC4
#define OUT_C_CANAL5 PORTC3
#define OUT_C_CANAL6 PORTC2
#define OUT_C_CANAL7 PORTC1
#define OUT_C_CANAL8 PORTC0

#define INP_D_CC2500_GDO0 PORTD2
#define INP_D_CC2500_GDO2 PORTD3
#define INP_D_KEY PORTD5

#define LED_OFF SET_BIT(PORTD, OUT_D_LED)
#define LED_ON RES_BIT(PORTD, OUT_D_LED)

#define FOSC 8000000 // Clock Speed
#define BAUD 19200
#define MYUBRR FOSC/16/BAUD-1
#define FAILSAFETIMER 200             // 2 Sekunden

#include "OCR_tx.h"


/* EEMEM */ EEData eeprom;
State state;
OutputData output;
volatile uint16_t Timer1ms;
volatile uint16_t Timer33ms;

ISR(TIMER2_COMPA_vect, ISR_NOBLOCK)              // Timer 1ms
{
//  RES_BIT(TIMSK2, OCIE2A);
  cli();
  ++Timer1ms;
  sei();
}

ISR(INT0_vect, ISR_NOBLOCK)
{
  RES_BIT(EIMSK, INT0);                    // INT0 aus, dient nur zum wecken
}

ISR(TIMER1_CAPT_vect, ISR_NOBLOCK)                  //8MHz capture
{
  static uint16_t timer_alt, capture_alt;
  static uint8_t chanPtr;
  uint16_t timer1msTemp;

  do
  {
    timer1msTemp = Timer1ms;
  }
  while(timer1msTemp != Timer1ms);
  uint16_t capture = ICR1;
  if(timer1msTemp - timer_alt > 2)   // Neuer PPM- Frame
  {
    TCNT2 = 0;                        // Timer 2 initialisieren
    chanPtr = 0;
    state.NewFrame = true;
  }
  else if(chanPtr < 8)
    output.chan_1us[chanPtr++] = (capture - capture_alt) / 8;
  capture_alt = capture;
  timer_alt = timer1msTemp;
}

void SPI_MasterInit(void)
{
  /* Set MOSI and SCK and SS output, all others input */
  PORTB |=  (1<<OUT_B_SPI_SS);
  /* Enable SPI, Master, set clock rate fck/2 */
  SPCR = (1<<SPE)|(1<<MSTR);
  SPSR = (1<<SPI2X);      // Double SPI Speed Bit
}

uint8_t SPI_MasterTransmit(uint8_t cData)
{
  PORTB &= ~(1<<OUT_B_SPI_SS);
  while(PIND & (1<<INP_D_CC2500_GDO2))                   // warten bis bereit
    ;
  /* Wait for transmission complete */
  /* Start transmission */
  SPDR = cData;
  while(!(SPSR & (1<<SPIF)))
    ;
  return(SPDR);
}

void cc2500_Init(void)
{
//  register uint8_t i;
//  register unsigned char counter asm("r3");
  prog_uint8_t *init = cc2500InitValue;

  SPI_MasterTransmit(CC2500_SRES);
  PORTB |= (1<<OUT_B_SPI_SS);       // SS wegnehmen
  _delay_us(40);                    // warten 40us
  PORTB &= ~(1<<OUT_B_SPI_SS);      // SS setzen
  while(PIND & (1<<INP_D_CC2500_GDO2))          //CHIP_RDY, warten bis bereit
      ;

  SPI_MasterTransmit(CC2500_WRITE_BURST);
//  for(i = 0;i < sizeof(cc2500InitValue);++i)
  do
  {
    SPI_MasterTransmit(pgm_read_byte(init++));
  }
  while(init < (cc2500InitValue + sizeof(cc2500InitValue)));
  PORTB |= (1<<OUT_B_SPI_SS);       // SS wegnehmen
  _delay_us(40);                    // warten 40us wegen SS
  SPI_MasterWriteReg(CC2500_SYNC0,(unsigned char)eeprom.id);
  SPI_MasterWriteReg(CC2500_SYNC1,(unsigned char)(eeprom.id >> 8));
}

void setBindMode(void)
{
  SPI_MasterWriteReg(CC2500_SYNC0, (unsigned char)BINDMODEID);
  SPI_MasterWriteReg(CC2500_SYNC1, (unsigned char)(BINDMODEID >> 8));
  SPI_MasterWriteReg(CC2500_PKTLEN, sizeof(eeprom));
}

void setNewChan(void)                // Kanal schreiben und nach FSTXON
{
  uint16_t tempChan = state.actChan + eeprom.step * 2 + 1;
  if(tempChan > 205)
    tempChan -= (205 + 1);
  state.actChan = tempChan;
  SPI_MasterWriteReg(CC2500_CHANNR, tempChan);
  SPI_MasterTransmit(CC2500_SFSTXON);
}

bool check_key(void)
{
  return(!(PIND & (1 << INP_D_KEY)));
}

void set_led(void)
{
  static uint16_t timer_alt;
  static uint8_t led_count;
  uint16_t timerTemp;

  timer1msTemp = Timer33ms;
  if(timerTemp - timer_alt > 10)
  {
    if(led_count & 1)
      LED_ON;
    else
      LED_OFF;
    timer_alt = timerTemp;

    if((led_count & 0xf) == 0)
    {
      timer_alt += 40;                    // Pause
      if(state.error == 0)
        return;
      do
      {
        if(state.error & (1 << (led_count >> 4)))
          led_count |= (led_count >> 3) | 1;
        led_count += 0x10;
        led_count &= 0x7f;
      }
      while(!(led_count & 0xf));
    }
    else
      --led_count;
  }
}

void calcNewId(void)
{
  do
  {
    eeprom.id += TCNT0 + TCNT1 + TCNT2;
    eeprom.step += eeprom.id;
    eeprom.step &= 0x7f;
//    if(eeprom.step > 204)
//      eeprom.step -= (204 + 1);
  }
  while((eeprom.id == 0) || (eeprom.step == 0));
  eeprom_write_block(&eeprom, 0, sizeof(eeprom));
}

void cc2500_TxNormOn(void)
{
  uint8_t tempPtr = output.chanPtr;
  if(state.FaileSafeTimer == FAILSAFETIMER)
  {
    SPI_MasterWriteReg(CC2500_TXFIFO, ((output.chan_1us[tempPtr] >> 8) & 0x3)
        & (tempPtr << 2) & 0x40);
    SPI_MasterWriteReg(CC2500_TXFIFO, (unsigned char)output.chan_1us[tempPtr]);
  }
  else if(state.FaileSafeTimer == FAILSAFETIMER - 1)
  {
    SPI_MasterWriteReg(CC2500_TXFIFO, ((output.chan_1us[tempPtr] >> 8) & 0x3)
        & (tempPtr << 2) & 0x80);
    SPI_MasterWriteReg(CC2500_TXFIFO, (unsigned char)output.chan_1us[tempPtr]);
  }
  else
  {
    if(tempPtr == 7)
      SPI_MasterWriteReg(CC2500_TXFIFO, ((output.chan_1us[tempPtr] >> 8) & 0x3)
          & (tempPtr << 2) & 0xc0);
    else
      SPI_MasterWriteReg(CC2500_TXFIFO, ((output.chan_1us[tempPtr] >> 8) & 0x3) & (tempPtr << 2));
    SPI_MasterWriteReg(CC2500_TXFIFO, (unsigned char)output.chan_1us[tempPtr]);
  }
  SPI_MasterTransmit(CC2500_STX);            // Enable TX;
  if(!output.chanPtr && state.FaileSafeTimer)
    --state.FaileSafeTimer;
  ++tempPtr;
  if(tempPtr > 7)
    tempPtr = 0;
  output.chanPtr = tempPtr;
}

void cc2500_TxBindOn(void)
{
  int8_t *p = (int8_t *)&eeprom;
  for(uint8_t i = 0;i < sizeof(eeprom);++i)
    SPI_MasterWriteReg(CC2500_TXFIFO, *p++);
  SPI_MasterTransmit(CC2500_STX);            // Enable TX
}

void txStateBind(void)
{
  static enum transmitter txstate;

  switch(txstate)
  {
  case WaitPPM:
  case Wait:
    txstate = TXready;
    break;
  case TXready:
    setNewChan();
    txstate = TXon;
    break;
  case TXon:
    cc2500_TxBindOn();                          // Sender aktivieren
    txstate = Wait;
    break;
  }
}

void txStateNorm(void)
{
  static enum transmitter txstate;

  switch(txstate)
  {
  case WaitPPM:
    if(state.NewFrame)
    {
      state.NewFrame = false;
      txstate = Wait;
    }
    break;
  case Wait:
    txstate = TXready;
    break;
  case TXready:
    setNewChan();
    txstate = TXon;
    break;
  case TXon:
    cc2500_TxNormOn();                          // Sender aktivieren
    if(output.chanPtr == 0)
      txstate = WaitPPM;
    else
      txstate = TXready;
    break;
  }
}

void chkFailSafe(void)
{
  if(check_key() && !state.FaileSafeTimer)
    state.FaileSafeTimer = FAILSAFETIMER;       /* 2 Sekunden */
}

void USART_Init( unsigned int ubrr)
{
  UBRR0H = (unsigned char)(ubrr>>8);          /* Set baud rate */
  UBRR0L = (unsigned char)ubrr;
  UCSR0A = 0;
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);           /* Enable receiver and transmitter */
  UCSR0C = (3<<UCSZ00);                     /* Set frame format: 8data, 1stop bit */
}

int main(void)
{
  CLKPR = 0;
  PORTB = 0x04;   DDRB = (1<<OUT_B_SPI_MISO) | (1<<OUT_B_SPI_SCK) | (1<<OUT_B_SPI_SS);    //prüfen
  PORTC = 0x0;    DDRC = 0x3f;
  PORTD = 0x50;   DDRD = 0x53;

// Timer0 32,768ms für clock
  TCCR0B = (5 << CS00);
  TIFR0 = 0;
  TIMSK0 = 0;

// Timer1 8MHz   PPM Capture
  TCCR1A = (0 << WGM10);
  TCCR1B = (1 << ICNC1) | (1 << WGM12) | (1 << CS10);      // CTC OCR1A, 8MHz
  TCNT1 = 0;
  TIFR1 = 0;
  TIMSK1 = (1 << ICIE1);


// Timer2 1ms für Timer
  TCCR2A = 0;
  TCCR2B = (1 << WGM21) | (3 << CS20);      //  CTC mode, clk/32
  OCR2A  = (F_CPU * 10 / 32 / 10000);       // ergibt 1ms (1000Hz)
  TIFR2  = (1 << OCF2A);
  TIMSK2 = (1 << OCIE2A);

//  wdt_enable(WDTO_500MS);
  EICRA = (1 << ISC01);                       // int0 bei fallender Flanke
  EIMSK = 0;
  EIFR = 0;

  eeprom_read_block(&eeprom, 0, sizeof(eeprom));
  if((eeprom.id == 0) || (eeprom.step == 0))
    calcNewId();
  cc2500_Init();
  if(check_key())        // Taste prüfen
  {
    state.bindmode = true;
    setBindMode();
  }
  set_sleep_mode(SLEEP_MODE_IDLE);
  wdt_enable(WDTO_30MS);
  USART_Init(MYUBRR);
  while(1){
    uint16_t old1ms;
    do
      old1ms = Timer1ms;
    while(old1ms != Timer1ms);
    while(Timer1ms==old1ms)
      if(state.bindmode)
      {
        if(!check_key())
          state.keyMode = true;
        if(Timer33ms > (5000 / 33) && !state.keyMode)  // Wenn länger als 5s gedrückt
          state.newIdMode = true;
        if(state.keyMode && state.newIdMode)
        {
          calcNewId();
          state.newIdMode = false;
        }
      }
      else
        sleep_mode();                   //    warten bis Timer (Interrupt)
    if(state.bindmode)
      txStateBind();
    else
      txStateNorm();
    if(TIFR0 & (1 << TOV0))
    {
      ++Timer33ms;
      TIFR0 &= ~(1 << TOV0);
    }
    set_led();
    chkFailSafe();
    wdt_reset();
  }
}
