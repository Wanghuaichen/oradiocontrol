/*
 * Author Josef Glatthaar <josef.glatthaar@googlemail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

bugs:
todo:

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

#include "OCR_tx.h"


/* EEMEM */ EEData eeprom;
uint8_t errorStatus;
State state;
OutputData output;
ChannelData channel[3];
uint16_t Timer;
//uint8_t heartbeat;



ISR(TIMER2_COMPA_vect, ISR_NOBLOCK)              // Timeout, 1. andere Antenne nehmen 2. anderen Kanal nehmen
{
  RES_BIT(TIMSK2, OCIE2A);
  state.timeOut = true;
}

ISR(INT0_vect, ISR_NOBLOCK)
{
  RES_BIT(EIMSK, INT0);                    // INT0 aus, dient nur zum wecken
}

ISR(TIMER1_COMPA_vect)                  //8MHz pulse generation
{
  register uint8_t i, cannal;

  cannal = output.chanPtr;                    // canal als lokaler Registerwert
  i = 0;
  while((TCNT1L < 10) && (++i < 50))  // Timer zu schnell auslesen funktioniert nicht, deshalb i
    ;
  if(state.ngmode)
    if(cannal & 1)
      (*set_CanalOutQ[cannal / 2])();
    else
      PORTC &= ~(1<<OUT_C_CANAL8);
  else
  (*set_CanalOut[cannal])();

  sei();

  OCR1A  = output.pulses2MHz[cannal++];

  if(output.pulses2MHz[cannal] == 0)
  {
    TIMSK1 &= ~(1<<OCIE1A);                   // Interrupt aus
    OCR1A = 5000u * 8u;                       // 5ms Pause
    set_sleep_mode(SLEEP_MODE_STANDBY);       // Wenn Timer1 fertig
  }
  output.chanPtr = cannal;
//  heartbeat |= HEART_TIMER2Mhz;
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
}

void cc2500_RxChanOn(uint8_t chan)
{
  state.actChanIdx = chan;
  SPI_MasterWriteReg(CC2500_CHANNR, eeprom.bind.corona.chan[chan]);
  cc2500_RxOn();
}

prog_int8_t APM freq[] = {0, -120, +120, +40, -40, -80, 80};

void setNewRxPara(void)
{
  // Antenne umschalten
  // die 2 anderen Kanäle probieren Achtung Frametime kann sich ändern!!
  // auf dem nächsten Kanal alle Frequenzen probieren


  setAnt(state.actAnt);               // Antenne wechseln
  if(state.scan)
  {
    if(++state.actFreqIdx == 6)
    {
      state.actFreqIdx = 0;
      if(++state.actChanIdx == 3)
      {
        state.actChanIdx = 0;
      }
    }
    SPI_MasterWriteReg(CC2500_FSCTRL0, pgm_read_byte(&freq[state.actFreqIdx]));
  }
  else
  {
    if(++state.actChanIdx == 3)
      state.actChanIdx = 0;
    if(state.actChanIdx == state.lastRxOkChanIdx)
    {
      state.scan = true;
      state.actFreqIdx = 0;
      SPI_MasterWriteReg(CC2500_FSCTRL0, pgm_read_byte(&freq[state.actFreqIdx]));
    }
  }
  cc2500_RxChanOn(state.actChanIdx);
}

void readBindData(void)
{
  uint8_t lqi;

  SPI_MasterTransmit(CC2500_READ_BURST | CC2500_RXFIFO);
  cc2500ReadBlock((int8_t *)&eeprom.bind, sizeof(eeprom.bind));
  SPI_MasterTransmit(CC2500_SNOP);      // rssi interessiert hier niemand
  lqi = SPI_MasterTransmit(CC2500_SNOP);
  setFrequencyOffset();                 // macht nicht viel Sinn, hat ja funktioniert
  if((lqi & 0x80) && state.bindmode)
  {
    eeprom_write_block(&eeprom.bind, 0 ,sizeof(eeprom.bind));
    state.bindmode = false;
  }
}

void setTimeout(void)
{
  TCNT2 = 0;
  TIFR2 = (1<<OCF2A);
  TIMSK2 = (1<<OCIE2A);
  state.timeOut = false;
}

void setFailSafe(void)
{
  for(uint8_t i = 0;i < 8;++i)
  {
    output.chan_1us[i] = eeprom.failSafe.failSafePos[i];
  }
}

void readChannalData(void)
{
  uint8_t i, data, lqi;
  uint32_t id = 0;

  lqi = 0;
  SPI_MasterTransmit(CC2500_READ_BURST | CC2500_RXFIFO);
  for(i = 0;i < 0x12;++i)
  {
    data = SPI_MasterTransmit(CC2500_SNOP);
    switch(i)
    {
      case 0:
      case 1:
      case 2:
      case 3:
      case 4:
      case 5:
      case 6:
      case 7:
        output.chan_1us[i] = data;
        break;
      case 8:
      case 9:
      case 10:
      case 11:
        output.chan_1us[(i - 8) * 2] += ((data & 0xf) << 8);             // unteres Nibble
        output.chan_1us[((i - 8) * 2) + 1] += ((data & 0xf0) << 4);      // oberes Nibble
       break;
      case 12:        // ID
        id = (uint32_t)data;
        break;
      case 13:        // ID
        id += (uint32_t)data << 8;
        break;
      case 14:        // ID
        id += (uint32_t)data << 16;
        break;
      case 15:        // ID
        id += (uint32_t)data << 24;
        break;
      case 16:        // RSSI value
        channel[state.actChanIdx].rssi = data;
        break;
      case 17:        // CRC ok; LQI
        lqi = data;
        break;
     }
  }
  channel[state.actChanIdx].lqi = lqi & 0x7f;
//  if((eeprom_read_dword(&((EEData *)0)->bind.corona.id) == id) && (lqi & 0x80))
  if((eeprom.bind.corona.id == id) && (lqi & 0x80))
  {
    setFrequencyOffset();
    setupPulsesPPM();                    // Wenn alles gut war, Ausgänge aktivieren
                        // Achtung je nach Kanal Latenz ausgleichen!!!
    setTimeout();                           // Timeout reseten
    state.scan = false;
    state.lastRxOkChanIdx = state.actChanIdx;
    if((channel[state.actChanIdx].rssi > 80) || (channel[state.actChanIdx].lqi < 5))
    {
      state.lowLqiMode = true;
    }
    else
    {
      state.lowLqiMode = false;
    }
  }
}

void processData(void)                //
{
  uint8_t data_lengh;

  if((data_lengh = get_RxCount()))
  {
    switch(get_Data())
    {
      case 0x7:         // Binden
        --data_lengh;   // Status gelesen
        if((data_lengh == 0x9) && (state.bindmode))
          readBindData();
        break;
      case 0x10:        // Normale Daten
        --data_lengh;   // Status gelesen
        if(data_lengh == 0x12)
          readChannalData();
        break;
      default:
        if(errorStatus < 255)
          ++errorStatus;
        break;
    }
    cc2500FlushData();
  }
  else        // CRC war falsch oder timeout
    if(state.timeOut)
    {
      setNewRxPara();
      setupPulsesPPM();                    // Ausgänge aktivieren
    }
}

bool check_key(void)
{
  if(PIND & (1 << INP_D_KEY))
    return false;
  return true;
}

void set_led(void)
{
  static uint8_t timer_alt, led_count;

  if((timer_alt + 10) < (uint8_t)Timer)
  {
    if(led_count & 1)
      LED_ON;
    else
      LED_OFF;
    timer_alt = uint8_t(Timer);

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

// Timer1 8MHz   Servoausgänge
  TCCR1A = (0 << WGM10);
  TCCR1B = (1 << WGM12) | (1 << CS10);      // CTC OCR1A, 8MHz
  TCNT1 = 0;
  OCR1A = 500 * 8;        // in 500us beginnen


// Timer2 30ms für Timeout
  TCCR2A = 0;
  TCCR2B = (1 << WGM21) | (7 << CS20);      //  CTC mode, clk/1024
  OCR2A  = (F_CPU * 10 / 1024 / 333);       // ergibt 30ms (33,3Hz)
  TIFR2  = 1 << OCF2A;
  TIMSK2 = 1 << OCIE2A;

//  wdt_enable(WDTO_500MS);
  EICRA = 1 << ISC01;           // int0 bei fallender Flanke
  EIMSK = 0;
  EIFR = 0;

  eeprom_read_block(&eeprom, 0, sizeof(eeprom));
  cc2500_Init();
  if(check_key())        // Taste prüfen
    state.bindmode = true;
  /* Beim Binden auf 0 oder 0xB8 suchen */
  if(state.bindmode)
  {
    SPI_MasterWriteReg(CC2500_CHANNR, 0);
    cc2500_RxOn();
  }
  else
    cc2500_RxChanOn(0);
  set_sleep_mode(SLEEP_MODE_IDLE);
  setFailSafe();
//  wdt_enable(WDTO_30MS);
  while(1){
    sleep_mode();                   //    warten bis was empfangen (Interrupt)
    processData();                  //    daten lesen
    cc2500_RxOn();
    if(TIFR0 & (1 << TOV0))
    {
      ++Timer;            //32,768ms
      RES_BIT(TIFR0, TOV0);
    }
    set_led();
//    if(heartbeat == 0x3)
//    {
//      wdt_reset();
//      heartbeat = 0;
//    }
  }
}
