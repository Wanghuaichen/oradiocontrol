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

bugs:
todo:
tmain berechnen (min, max)
Ausgabe der Einzelkanäle kann sofort erfolgen wenn Daten empfangen
Achtung: Beim Einschalten kein FailSafe wegen Regler

Sender am Boden ist der Master für das Timing

(Option) Solange kein Telegramm ausfällt, reicht es die geänderten zu Übertragen.
(Option) Immer wieder auch nicht veränderte übertragen.

Mischbetrieb PPM- Ausgabe und Einzelausgabe schwierig

Tabelle: Timerwert, Kanal, ein, aus
  Minimalabstand nötig, wegen Interrupt
Synchrones Ausgeben schwierig bei gleichen oder ähnlichen Timerwerten.
  dort leichte Schachtelung nötig

pulses2MHz[channel]
routine   1 aus 2 an oder nur 1 aus
10ms Frame ist nicht zu schaffen (1ms pro Kanal + 2ms Rückkanal)

Kanal suchen
Frquenz verstellen

empfangen bind
empfangen normal
senden telemetrie


done:
Nachsehen ob neuer Kanal kleiner als letzter, wenn ja PPM generieren.
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
//PIN 11  PD7   AIN1/OC2B/PCINT23
//PIN 12  PB0   ICP1/CLKO/PCINT0
//PIN 13  PB1   OC1A/PCINT1
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


#define OUT_B_SPI_SS PORTB2
#define OUT_B_SPI_MOSI PORTB3
#define OUT_B_SPI_MISO PORTB4
#define OUT_B_SPI_SCK PORTB5
#define OUT_D_LED PORTD4
#define OUT_D_ANT1 PORTD6
#define OUT_D_ANT2 PORTD7               // Antenne 2????

#define OUT_D_CANAL1 PORTD1             // TXD
#define OUT_D_CANAL2 PORTD0             // RXD
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

#include "ORC_rx.h"


/* EEMEM */ EEData eeprom;
//uint8_t errorStatus;
State state;
OutputData output;
volatile uint16_t Timer1ms;
volatile uint16_t Timer33ms;
//volatile uint8_t IntTimer;
bool RxInterrupt;
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

void set_Canal_Off(void){
  RES_BIT(PORTC, OUT_C_CANAL8);
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
  set_Canal_8,
  set_Canal_Off
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
  set_Canal_Off
};


ISR(TIMER2_COMPA_vect, ISR_NOBLOCK)              // Timeout, 1. andere Antenne nehmen 2. anderen Kanal nehmen
{
//  RES_BIT(TIMSK2, OCIE2A);
//  cli();
  ++Timer1ms;
//  sei();
  RxInterrupt = true;
}

ISR(INT0_vect, ISR_NOBLOCK)
{
  RxInterrupt = true;
//  RES_BIT(EIMSK, INT0);                    // INT0 aus, dient nur zum wecken
}

ISR(TIMER1_COMPA_vect)                  //8MHz pulse generation
{
  register uint8_t i, channel;

  channel = output.chanPtr;                    // canal als lokaler Registerwert
  i = 0;
  while((TCNT1L < 10) && (++i < 50))  // Timer zu schnell auslesen funktioniert nicht, deshalb i
    ;
  if(state.ngmode)
    if(channel & 1)
      (*set_CanalOutQ[channel / 2])();
    else
      PORTC &= ~(1<<OUT_C_CANAL8);
  else
  (*set_CanalOut[channel])();

  sei();

  OCR1A  = output.pulses2MHz[channel++];

  if(output.pulses2MHz[channel] == 0)
  {
    TIMSK1 &= ~(1<<OCIE1A);                   // Interrupt aus
    OCR1A = 5000u * 8u;                       // 5ms Pause
    set_sleep_mode(SLEEP_MODE_STANDBY);       // Wenn Timer1 fertig
  }
  output.chanPtr = channel;
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
  while(PINB & (1<<OUT_B_SPI_SS));
  while(PIND & (1<<INP_D_CC2500_GDO2))                   // warten bis bereit
    ;                                                   // Ist eigentlich unnötig
  /* Wait for transmission complete */
  /* Start transmission */
  SPDR = cData;
  while(!(SPSR & (1<<SPIF)))
    ;
  return(SPDR);
}

void cc2500BurstOff(void)
{
  PORTB |= (1<<OUT_B_SPI_SS);       // SS wegnehmen
  while(!(PINB & (1<<OUT_B_SPI_SS)));
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

  SPI_MasterTransmit(CC2500_IOCFG2 | CC2500_WRITE_BURST);
//  for(i = 0;i < sizeof(cc2500InitValue);++i)
  do
  {
    SPI_MasterTransmit(pgm_read_byte(init++));
  }
  while(init < (cc2500InitValue + sizeof(cc2500InitValue)));
  cc2500BurstOff();       // SS wegnehmen
  _delay_us(40);                    // warten 40us wegen SS
  SPI_MasterWriteReg(CC2500_SYNC0,(unsigned char)eeprom.bind.id);
  SPI_MasterWriteReg(CC2500_SYNC1,(unsigned char)(eeprom.bind.id >> 8));
}

void setupPulsesPPM_quad()
{
  if(!(TIFR1 & (1<<OCF1A)))
  {
    return;               // Noch gar nicht fertig
  }

  OCR1A = 500 * 8;        // in 500us beginnen
  TCNT1 = 0;

//  sei();
  uint8_t j=0;
  output.pulses2MHz[j++]=(uint16_t)300 * 8;
  for(uint8_t i=0;i<8;i++)
  {
    output.pulses2MHz[j++] = output.chan_1us[i] * 8 - 300 * 8 + 1500u;
    output.pulses2MHz[j++]=(uint16_t)300 * 8;
  }
  output.pulses2MHz[j]=0;
  output.chanPtr = 0;
  TIFR1 &= ~(1<<OCF1A);
  TIMSK1 |= (1<<OCIE1A);      // Timer Interrupt Mask Register
//  set_sleep_mode(SLEEP_MODE_IDLE);
}

void setupPulsesPPM()
{
  if(!(TIFR1 & (1<<OCF1A)))
  {
    return;               // Noch gar nicht fertig
  }

  OCR1A = 500 * 8;        // in 500us beginnen (je nach Kanal)
  TCNT1 = 0;

//  sei();
  uint8_t j=0;
  for(uint8_t i = 0;i < 8;i++)
    output.pulses2MHz[j++] = output.chan_1us[i] * 8 + 1500u;
  output.pulses2MHz[j] = 0;
  output.chanPtr = 0;
  TIFR1 &= ~(1<<OCF1A);
  TIMSK1 |= (1<<OCIE1A);          // Timer Interrupt Mask Register
//  set_sleep_mode(SLEEP_MODE_IDLE);
}

void setNextChan(void)                // Kanal schreiben
{
  uint16_t tempChan = state.actChan + eeprom.bind.step * 2 + 1;
  if(tempChan > 205)
    tempChan -= (205 + 1);
  state.actChan = tempChan;
  SPI_MasterWriteReg(CC2500_CHANNR, tempChan);
  SPI_MasterTransmit(CC2500_SFSTXON);       // Kalibrieren
}

//void setTxOn(void)                // Kanal schreiben und nach FSTXON
//{
//  setNextChan();
//  SPI_MasterWriteReg(CC2500_PKTLEN, sizeof(Telemetrie));
//}

void setRxOn(void)                // Kanal schreiben und nach Rx
{
//  setNextChan();
  SPI_MasterWriteReg(CC2500_PKTLEN, sizeof(MessageData));
  SPI_MasterTransmit(CC2500_SRX);
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
  int8_t rssi = SPI_MasterReadReg(CC2500_RSSI | CC2500_READ_BURST);
  return(rssi > 0);
}

void setFreeChan(void)
{
  uint16_t tempChan = state.actChan + 1;
  if(tempChan > 205)
    tempChan -= (205 + 1);
  state.actChan = tempChan;
  SPI_MasterTransmit(CC2500_SIDLE);
  SPI_MasterWriteReg(CC2500_CHANNR, tempChan);
  SPI_MasterTransmit(CC2500_SRX);
}

prog_int8_t APM freq[] = {0, -120, +120, +40, -40, -80, 80};

void setNewRxFrequ(void)
{
  setAnt(state.actAnt);               // Antenne wechseln
  if(++state.actFreqIdx >= 7)
    state.actFreqIdx = 0;
  SPI_MasterWriteReg(CC2500_FSCTRL0, pgm_read_byte(&freq[state.actFreqIdx]));
}

void setReceiveError(void)
{
  setAnt(state.actAnt);               // Antenne wechseln
  ++state.errorcount;
}

void readBindData(void)
{
  MessageBind mes;
  static uint8_t counter;

  SPI_MasterTransmit(CC2500_READ_BURST | CC2500_RXFIFO);
  cc2500ReadBlock((int8_t *)&mes, sizeof(mes));
  cc2500BurstOff();
  setFrequencyOffset();                 // macht nicht viel Sinn, hat ja funktioniert
  if(mes.crcOk)
  {
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
  for(uint8_t i = 0;i < 8;++i)
    output.chan_1us[i] = eeprom.failSafe.failSafePos[i];
}

void setFailSafe(void)
{
  for(uint8_t i = 0;i < 8;++i)
    eeprom.failSafe.failSafePos[i] = output.chan_1us[i];
  eeprom_write_block(&eeprom.failSafe, 0 ,sizeof(eeprom.failSafe));
}

void readData(void)
{
//  uint8_t data, lqi, chan, id;
//  int8_t rssi;
  Message mes;

  SPI_MasterTransmit(CC2500_READ_BURST | CC2500_RXFIFO);
  cc2500ReadBlock((int8_t *)&mes, sizeof(mes));
  cc2500BurstOff();         // Burstzugriff rücksetzen
  if(mes.crcOk)          // CRC ok
  {
    if(!mes.data.command.mode)
    {
      uint8_t chan = mes.data.channel.channel;
      if(chan < 8)
      {
        output.chan_1us[chan] = mes.data.channel.chan_1us;
        output.timeOut[chan] = 0;
        output.chanFlag &= ~(1 << chan);
        if(output.chanMax < chan)
          output.chanMax = chan;
        if(output.chanMax < chan)
          setupPulsesPPM();                    // Wenn alles gut war, Ausgänge aktivieren
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
    // Achtung auch bei Ausfall des letzten Telegramms PPM erzeugen
        // Achtung je nach Kanal Latenz ausgleichen!!!
    }
    setFrequencyOffset();
    if((mes.rssi > 50) || (mes.lqi < 5))   // Empfang schlecht
      setAnt(state.actAnt);               // Antenne wechseln
    state.errorcount = 0;
  }
  else
    setReceiveError();
}

void sendTelemetrie(void)
{
  Telemetrie mes;

  SPI_MasterTransmit(CC2500_STX);            // Enable TX
  SPI_MasterWriteReg(CC2500_PKTLEN, sizeof(Telemetrie));
  mes.sensor = 0;
  mes.data = state.errorcount;
  cc2500WriteSingle((int8_t *)&mes, sizeof(mes));
}

void processData(void)                // Nachsehen ob was da
{
  uint8_t data_lengh;

  if((data_lengh = get_RxCount()))
  {
    TCNT2 = 0;                          // Timer kommt eher später
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
    cc2500FlushData();
  }
  else
    setReceiveError();
}

bool checkKey(void)
{
  if(PIND & (1 << INP_D_KEY))
    return false;
  return true;
}

void set_led(void)
{
  static uint8_t  led_count;
  static uint16_t timer_alt;
  uint16_t timerTemp;

  timerTemp = Timer33ms;

  if((timer_alt + 10) < timerTemp)
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

void setBindMode(void)
{
  SPI_MasterWriteReg(CC2500_SYNC0, (unsigned char)BINDMODEID);
  SPI_MasterWriteReg(CC2500_SYNC1, (unsigned char)(BINDMODEID >> 8));
  SPI_MasterWriteReg(CC2500_PKTLEN, sizeof(BindData));
}

void rxState(void)
{
  static enum receiver rxstate;
  static uint16_t counter;

  switch(rxstate)
  {
  case Start:
    if(checkKey())
    {
      setBindMode();
      state.bindmode = true;
    }
    else
      state.bindmode = false;
    setFreeChan();                 // Kanal einstellen und Empfang ein
    counter = 0;
    rxstate = RxWaitStart;
    break;
  case RxWaitStart:                  // auf Empfänger warten
    rxstate = checkRSSI;
    break;
  case checkRSSI:                   // Kanal frei?
    if((counter > 100) || checkchanFree() ||
        !(PIND & (1 << INP_D_CC2500_GDO0)) || get_RxCount())
    {
      counter = 0;
      rxstate = waitForData;
    }
    else
    {
      setFreeChan();                 // Neuer Kanal einstellen und Empfang ein
      ++counter;
      rxstate = RxWaitStart;
    }
    break;
  case waitForData:                 // Eine Sekunde auf Empfang warten
    if(!(PIND & (1 << INP_D_CC2500_GDO0)))        // Einsprung über Timerinterrupt und Empfang läuft gerade
      return;
    if(get_RxCount())                           // daten da
    {
      setNextChan();
      processData();                  // Empfangsregister auswerten, muss als erstes kommen, wegen Auswertung Telegramm
      if(state.RxCount >= 8)
      {
        rxstate = TxOn;
        state.RxCount = 0;
      }
      else
      {
        setRxOn();
        rxstate = RxWait;
      }
      counter = 0;
    }
    else
    {
      if(counter % 128 == 127)
        setNewRxFrequ();                // Frequenz verstellen
      if(counter > 1000)                  // 1 Sekunde warten
      {                                 // Neu initialisieren ??
        setFreeChan();                 // Kanal einstellen und Empfang ein
        counter = 0;
        rxstate = RxWaitStart;
      }
      else
        ++counter;
    }
    break;
  case Main:                                    // Frequenz einstellen und Kalibrieren
    if(!(PIND & (1 << INP_D_CC2500_GDO0)))        // Einsprung über Timerinterrupt und Empfang läuft gerade
      return;
    setNextChan();
    processData();                  // Empfangsregister auswerten, muss als erstes kommen, wegen Auswertung Telegramm
    if(state.errorcount > 200)       // 200 Telegramme nicht empfangen
    {
      setFreeChan();                 // Kanal einstellen und Empfang ein
      counter = 0;
      rxstate = RxWaitStart;
    }
    if(state.RxCount >= 8)
    {
      rxstate = TxOn;
      state.RxCount = 0;
    }
    else
    {
      setRxOn();
      rxstate = RxWait;
    }
    break;
  case RxWait:                      // Daten empfangen
    rxstate = Main;
    ++state.RxCount;
    break;
  case TxOn:                      // Daten reinschreiben Sender einschalten und senden
    sendTelemetrie();
    rxstate = TxWait;
    break;
  case TxWait:                  // Jetzt wird weitergesendet
    rxstate = RxOn;
    break;
  case RxOn:                    // Senden fertig, Empfänger ein
    setNextChan();
    setRxOn();
    rxstate = RxWait;
    break;
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


// Timer2 2ms für Timeout
  TCCR2A = 0;
  TCCR2B = (1 << WGM21) | (3 << CS20);      //  CTC mode, clk/32
  OCR2A  = (F_CPU * 10 / 32 / 10000);       // ergibt 1ms (1000 Hz)
  TIFR2  = 0;
  TIMSK2 = 1 << OCIE2A;

//  wdt_enable(WDTO_500MS);
  EICRA = 1 << ISC01;           // int0 bei fallender Flanke
  EIMSK = 0;
  EIFR = 0;

  eeprom_read_block(&eeprom, 0, sizeof(eeprom));
  cc2500_Init();
  set_sleep_mode(SLEEP_MODE_IDLE);
  getFailSafe();
  wdt_enable(WDTO_30MS);
  while(1){
    sleep_mode();                   //    warten bis was empfangen (Interrupt)
    //nur bei Timer 1ms Interrupt oder Int0, nicht bei PPM- Interrupt
    if(RxInterrupt)
    {
      rxState();
      RxInterrupt = false;
    }
    if(TIFR0 & (1 << TOV0))
    {
      ++Timer33ms;
      TIFR0 &= ~(1 << TOV0);
      set_led();
    }
    wdt_reset();
  }
}

