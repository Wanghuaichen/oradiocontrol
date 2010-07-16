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
Achtung: Beim Einschalten kein FailSafe wegen Regler


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

#include "OCR_rx.h"


/* EEMEM */ EEData eeprom;
uint8_t errorStatus;
State state;
OutputData output;
ChannelData channel[3];
volatile uint16_t Timer1ms;
volatile uint16_t Timer33ms;
volatile uint8_t IntTimer;
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
  cli();
  ++Timer2ms;
  sei();
  if(IntTimer)
    --IntTimer;
  if(!IntTimer)
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
    output.pulses2MHz[j++] = output.chan_1us[i] * 8 - 300 * 8;
    output.pulses2MHz[j++]=(uint16_t)300 * 8;
  }
  output.pulses2MHz[j]=0;
  output.chanPtr = 0;
  TIFR1 &= ~(1<<OCF1A);
  TIMSK1 |= (1<<OCIE1A);
  set_sleep_mode(SLEEP_MODE_IDLE);
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
    output.pulses2MHz[j++] = output.chan_1us[i] * 8;
  output.pulses2MHz[j] = 0;
  output.chanPtr = 0;
  TIFR1 &= ~(1<<OCF1A);
  TIMSK1 |= (1<<OCIE1A);
  set_sleep_mode(SLEEP_MODE_IDLE);
}

void setNextChan(void)                // Kanal schreiben
{
  uint16_t tempChan = state.actChan + eeprom.bind.step * 2 + 1;
  if(tempChan > 205)
    tempChan -= (205 + 1);
  state.actChan = tempChan;
  SPI_MasterWriteReg(CC2500_CHANNR, tempChan);
}

void setNextChanRx(void)                // Kanal schreiben und nach RX
{
  setNextChan();
  SPI_MasterTransmit(CC2500_SRX);
}

void setNextChanTx(void)                // Kanal schreiben und nach TX
{
  setNextChan();
  SPI_MasterTransmit(CC2500_SFSTXON);
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

void setFreeChan(void)
{
  uint8_t i = 0;
  int8_t rssi;
  uint16_t timer;

  do
  {
    uint16_t tempChan = state.actChan + 1;
    if(tempChan > 205)
      tempChan -= (205 + 1);
    state.actChan = tempChan;
    SPI_MasterTransmit(CC2500_SIDLE);
    SPI_MasterWriteReg(CC2500_CHANNR, tempChan);
    SPI_MasterTransmit(CC2500_SRX);
    timer = Timer1ms;
    do
    {
      if((PIND & (1 << INP_D_CC2500_GDO0)))
        return;                             // Telegramm kommt gerade -> dann zurück und auf Interrupt warten
    }
    while((timer + 4) >= Timer1ms);   // Warten bis RX und RSSI gemessen
    rssi = SPI_MasterReadReg(CC2500_RSSI | CC2500_READ_BURST); //  testen ob frei nein dann anderen
  }
  while((++i < 10) && (rssi > 0));        // && kanal nicht frei)
}

prog_int8_t APM freq[] = {0, -120, +120, +40, -40, -80, 80};

void setNewRxPara(void)
{
  // Antenne umschalten
  // freien Kanal suchen
  // 1 Sekunde zeit lassen
  // Synthesiser verstellen und kontrollieren ob Kanal noch frei
  // neuer Kanal suchen

  if(state.RxTimeOut33++ == 0)
  {
    setAnt(state.actAnt);               // Antenne wechseln
    if(++state.actFreqIdx >= 7)
    {
      state.actFreqIdx = 0;
      SPI_MasterWriteReg(CC2500_FSCTRL0, pgm_read_byte(&freq[state.actFreqIdx]));
      setFreeChan();                  // Achtung läuft länger als 2ms
    }
    else
      SPI_MasterWriteReg(CC2500_FSCTRL0, pgm_read_byte(&freq[state.actFreqIdx]));
  }
  else if (state.RxTimeOut33 > 30)
    state.RxTimeOut33 = 0;

}

void readBindData(void)
{
  uint8_t lqi;

  SPI_MasterTransmit(CC2500_READ_BURST | CC2500_RXFIFO);
  cc2500ReadBlock((int8_t *)&eeprom.bind, sizeof(eeprom.bind));
  SPI_MasterTransmit(CC2500_SNOP);      // rssi interessiert hier niemand
  lqi = SPI_MasterTransmit(CC2500_SNOP);
  cc2500BurstOff();
  setFrequencyOffset();                 // macht nicht viel Sinn, hat ja funktioniert
  if((lqi & 0x80) && state.bindmode)
  {
    eeprom_write_block(&eeprom.bind, 0 ,sizeof(eeprom.bind));
    state.bindmode = false;           // Neu initialisieren?
  }
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
  if(!mes.data.command.mode)
  {
    if(mes.crcOk)          // CRC ok
    {
      uint8_t chan = mes.data.channel.channel;
      if(chan < 8)
      {
        output.chan_1us[chan] = mes.data.channel.chan_1us;
        output.timeOut[chan] = 0;
        //???
        //output.chanMax = chan;                // Einmal muss es gehen, wegen init
        //setupPulsesPPM();                    // Wenn alles gut war, Ausgänge aktivieren
      }
    }
  }
  else
  {
    if(mes.data.command.rts)
    {
      IntTimer = 2;                   // Senden
      state.RxCount = 7;
    }
    if(mes.data.command.command == 1)     // FailSafe Position setzen
      setFailSafe();
    // Achtung auch bei Ausfall des letzten Telegramms PPM erzeugen
        // Achtung je nach Kanal Latenz ausgleichen!!!
  }
  setFrequencyOffset();
  if((mes.rssi > 50) || (mes.lqi < 5))   // Empfang schlecht
    setAnt(state.actAnt);               // Antenne wechseln
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
      if(data_lengh == sizeof(BindData))
        readBindData();
    }
    else if(data_lengh == 2)
      readData();
    cc2500FlushData();
    setNextChanRx();
    state.RxTimeOut = 0;
  }
  else
    if(state.RxTimeOut > 100)           // 100 mal testen
    {
      if(state.RxTimeOut33++ == 0)
        setNewRxPara();                   // Achtung nur alle 1 sec
                                          // Ausfall mehrere kompletter Frames
      else if (state.RxTimeOut33 > 102)
        state.RxTimeOut33 = 0;
    }
    else      // Ausfall eines Telegramms
    {         // Falls es das letzte war?
      ++state.RxTimeOut;
      setNextChanRx();
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
  if(check_key())        // Taste prüfen
  {
    setBindMode();
    state.bindmode = true;
  }
  set_sleep_mode(SLEEP_MODE_IDLE);
  getFailSafe();
//  wdt_enable(WDTO_30MS);
  while(1){
    sleep_mode();                   //    warten bis was empfangen (Interrupt)
    //nur bei Timer 1ms Interrupt oder Int0, nicht bei PPM- Interrupt
    if(RxInterrupt)
    {
      if(!(PIND & (1 << INP_D_CC2500_GDO0)))
      {
        processData();                  //    daten lesen
        if(state.RxCount == 7)
        {
          /* Sender ein */
        }
        else
        {
          setNextChanRx();
          IntTimer = 2;
        ++state.RxCount;
        if(state.RxCount >= 9)


      }
      RxInterrupt = false;
    }
    if(TIFR0 & (1 << TOV0))
    {
      ++Timer33ms;
      TIFR0 &= ~(1 << TOV0);
      set_led();
    }
//    if(heartbeat == 0x3)
//    {
//      wdt_reset();
//      heartbeat = 0;
//    }
  }
}

