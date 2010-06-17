/*
 * Author	Josef Glatthaar <josef.glatthaar@googlemail.com>
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
    woher weis er wenn Bytes da sind? weil ein Signal über GDO0
    stromverbrauch
done:
    // Antenne wechseln alle 5 Telegramme
    // dann auf der Antenne bleiben, die besser war für die nächsten 5 Telegramme
    // Kanal wechseln wenn RSSI zu hoch oder LQI zu klein
     *   Nachteil: Latenz
    // RSSI kann auch periodisch gelesen werden.

Anderer Mode:
  erster Kanal einstellen
  Telegramm empfangen
  zweiter Kanal einstellen
  Antenne wechseln
  Telegramm empfangen
  dritter Kanal einstellen
  Antenne wechseln
  Telegramm empfangen
  erstes gutes Telegramm nehmen

  Vorteil: beide Antennen und alle Kanäle
  Nachteil: Stromverbrauch

noch ein Mode:
  nur bei schlechtem Empfang auf Spezialmode wechseln
  bis Empfang wieder gut
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

#define SET_BIT(port,bit)  (port |=  (1<<bit))
#define RES_BIT(port,bit)  (port &= (uint8_t)~(1<<bit))

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

#include "ORC.h"
typedef PROGMEM void (*FuncP_PROGMEM)(void);

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
  0x3d,     //  PKTLEN     Indicates the packet length when fixed length packets are enabled.,
            //             If variable length packets are used, this value indicates the
            //             maximum length packets allowed. (61 Bytes),
  0xC,      //  PKTCTRL1   (Default) jetzt autoflush
  0x5,      //  PKTCTRL0   Whitening off,
            //             CRC calculation in TX and CRC check in RX enabled,
            //             Variable length packets, packet length configured by the first byte
            //             after sync word,
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


/* EEMEM */ EEData eeprom;
uint8_t errorStatus;
State state;
OutputData output;
ChannelData channel[3];
uint16_t Timer;
//uint8_t heartbeat;

void set_Canal_1(void){
  RES_BIT(PORTC, OUT_C_CANAL8);
  SET_BIT(PORTD, OUT_D_CANAL1);
}

void set_Canal_1_Q(void){
  RES_BIT(PORTC, OUT_C_CANAL8);
  SET_BIT(PORTD, OUT_D_CANAL1);
  SET_BIT(PORTC, OUT_C_CANAL8);
}

void set_Canal_2(void){
  RES_BIT(PORTD, OUT_D_CANAL1);
  SET_BIT(PORTD, OUT_D_CANAL2);
}

void set_Canal_2_Q(void){
  RES_BIT(PORTD, OUT_D_CANAL1);
  SET_BIT(PORTD, OUT_D_CANAL2);
  SET_BIT(PORTC, OUT_C_CANAL8);
}

void set_Canal_3(void){
  RES_BIT(PORTD, OUT_D_CANAL2);
  SET_BIT(PORTC, OUT_C_CANAL3);
}

void set_Canal_3_Q(void){
  RES_BIT(PORTD, OUT_D_CANAL2);
  SET_BIT(PORTC, OUT_C_CANAL3);
  SET_BIT(PORTC, OUT_C_CANAL8);
}

void set_Canal_4(void){
  RES_BIT(PORTC, OUT_C_CANAL3);
  SET_BIT(PORTC, OUT_C_CANAL4);
}

void set_Canal_4_Q(void){
  RES_BIT(PORTC, OUT_C_CANAL3);
  SET_BIT(PORTC, OUT_C_CANAL4);
  SET_BIT(PORTC, OUT_C_CANAL8);
}

void set_Canal_5(void){
  RES_BIT(PORTC, OUT_C_CANAL4);
  SET_BIT(PORTC, OUT_C_CANAL5);
}

void set_Canal_5_Q(void){
  RES_BIT(PORTC, OUT_C_CANAL4);
  SET_BIT(PORTC, OUT_C_CANAL5);
  SET_BIT(PORTC, OUT_C_CANAL8);
}

void set_Canal_6(void){
  RES_BIT(PORTC, OUT_C_CANAL5);
  SET_BIT(PORTC, OUT_C_CANAL6);
}

void set_Canal_6_Q(void){
  RES_BIT(PORTC, OUT_C_CANAL5);
  SET_BIT(PORTC, OUT_C_CANAL6);
  SET_BIT(PORTC, OUT_C_CANAL8);
}

void set_Canal_7(void){
  RES_BIT(PORTC, OUT_C_CANAL6);
  SET_BIT(PORTC, OUT_C_CANAL7);
}

void set_Canal_7_Q(void){
  RES_BIT(PORTC, OUT_C_CANAL6);
  SET_BIT(PORTC, OUT_C_CANAL7);
  SET_BIT(PORTC, OUT_C_CANAL8);
}

void set_Canal_8(void){
  RES_BIT(PORTC, OUT_C_CANAL7);
  SET_BIT(PORTC, OUT_C_CANAL8);
}

void set_Canal_8_Q(void){
  RES_BIT(PORTC, OUT_C_CANAL7);
  RES_BIT(PORTC, OUT_C_CANAL8);
  SET_BIT(PORTC, OUT_C_CANAL8);
}

FuncP_PROGMEM APM set_CanalOut[] = {
  set_Canal_1,
  set_Canal_2,
  set_Canal_3,
  set_Canal_4,
  set_Canal_5,
  set_Canal_6,
  set_Canal_7,
  set_Canal_8
};

FuncP_PROGMEM APM set_CanalOutQ[] = {
  set_Canal_1_Q,
  set_Canal_2_Q,
  set_Canal_3_Q,
  set_Canal_4_Q,
  set_Canal_5_Q,
  set_Canal_6_Q,
  set_Canal_7_Q,
  set_Canal_8_Q,
};


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
  while(PORTD & (1<<INP_D_CC2500_GDO2))                   // warten bis bereit
    ;
  /* Wait for transmission complete */
  /* Start transmission */
  SPDR = cData;
  while(!(SPSR & (1<<SPIF)))
    ;
  return(SPDR);
}

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

void cc2500_Init(void)
{
//  register uint8_t i;
//  register unsigned char counter asm("r3");
  prog_uint8_t *init = cc2500InitValue;

  SPI_MasterTransmit(CC2500_SRES);
  PORTB |= (1<<OUT_B_SPI_SS);       // SS wegnehmen
  _delay_us(40);                    // warten 40us
  PORTB &= ~(1<<OUT_B_SPI_SS);      // SS setzen
  while(PORTD & (1<<INP_D_CC2500_GDO2))          //CHIP_RDY, warten bis bereit
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

void setupPulsesPPM_quad()
{
  OCR1A = 500 * 8;        // in 500us beginnen
  TCNT1 = 0;

//  sei();
  uint8_t j=0;
  output.pulses2MHz[j++]=(uint16_t)300 * 8;
  for(uint8_t i=0;i<8;i++)
  {
    int32_t v = output.chan_1us[i];
                                      // 512 -> 2ms, 0 -> 1,5ms, -512 -> 1ms
//    v = v*78125l/10000l + 1500*8;    // 512 -> 0,5ms -> 4000 -> 7,8125
    v = output.chan_1us[i] * 8 - 300 * 8;
    output.pulses2MHz[j++]=(uint16_t)v;
    output.pulses2MHz[j++]=(uint16_t)300 * 8;
//    pulses2MHz[j++]=300*8;
  }
  output.pulses2MHz[j]=0;
  output.chanPtr = 0;
  TIFR1 &= ~(1<<OCF1A);
  TIMSK1 |= (1<<OCIE1A);
  set_sleep_mode(SLEEP_MODE_IDLE);
}

void setupPulsesPPM()
{
  OCR1A = 500 * 8;        // in 500us beginnen (je nach Kanal)
  TCNT1 = 0;

//  sei();
  uint8_t j=0;
  for(uint8_t i = 0;i < 8;i++)
    output.pulses2MHz[j++] = output.chan_1us[i] * 8;
  output.pulses2MHz[j] = 0;
  output.chanPtr = 0;
  TIFR1 &= ~(1<<OCF1A);
  TIMSK1 |= (1<<OCIE1A);
  set_sleep_mode(SLEEP_MODE_IDLE);
}

uint8_t get_RxCount(void)                   // Anzahl Bytes im FIFO
{
  SPI_MasterTransmit(0xfb);
  return(SPI_MasterTransmit(0));
}

inline void cc2500FlushData(void)
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

void cc2500_RxChanOn(uint8_t chan)
{
  state.actChanIdx = chan;
  SPI_MasterWriteReg(CC2500_CHANNR, eeprom.bind.corona.chan[chan]);
  cc2500_RxOn();
}

uint8_t get_Data()
{
  SPI_MasterTransmit(0xbf);
  return(SPI_MasterTransmit(CC2500_SNOP));
}

void cc2500ReadBlock(int8_t *p, uint8_t n)
{
  while(n--)
    *p++ = SPI_MasterTransmit(CC2500_SNOP);
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

void setAnt(bool ant)
{
  if(ant)
  {
    SET_BIT(PORTD, OUT_D_ANT1);
    RES_BIT(PORTD, OUT_D_ANT2);
    state.actAnt = false;
  }
  else
  {
    SET_BIT(PORTD, OUT_D_ANT2);
    RES_BIT(PORTD, OUT_D_ANT1);
    state.actAnt = true;
  }
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
    }
}

bool check_key(void)
{
  if(PORTD & (1 << INP_D_KEY))
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

