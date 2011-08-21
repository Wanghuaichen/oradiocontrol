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

#include "../../ORC.h"

typedef struct t_FailSafeData
{
  uint8_t   failSafeMode;//:5;        // 0 failSafePos nach Zeit, <>0 = off (für Regler)
  uint16_t  failSafePos;//:11;         // Position bei FailSafe
  uint8_t   failSafeDelay;       // Zeit bis failsafepos anspricht in 20ms Steps (255 = hold)
}__attribute__((packed)) FailSafeData;

typedef struct t_EEData
{
  BindData      bind;
  FailSafeData  failSafe[MAXCHAN];
  uint8_t       outputOffset;        // Offset in ms um das die Ausgangsgruppen verschoben sind
  uint8_t       pulsesDelay[MAXCHAN];   // Mindestabstand der Servoimpulse in ms
  bool          txDisable;           //:1;
  bool          chanOff;            //:1;
}__attribute__((packed)) EEData;

typedef struct t_State
{
  bool      bindmode;
//  bool      ngmode:1;             // new generation mode
  bool      led;
//  bool      synch:1;
//  bool      lowLqiMode:1;
//  bool      lastRxOkChanIdx:2;
//  uint8_t   RxTimeOut33;
//  uint8_t   RxTimeOut;
  volatile uint8_t   RxTimer;
  uint8_t   RxCount;              // 0 - 7
  uint8_t   actChan;              // Eingestellter Kanal
  bool      actAnt;             // Eingestellte Antenne
  uint8_t   actFreqIdx;
  int8_t    freqOffset;
  uint8_t    groupTimer[2];
  bool      syncError;
  int8_t    pll;
//  uint8_t   dummy:4;
  uint8_t   ppmOverflow;
  uint8_t   ledError;             // Bitkodierte Fehler für LED
  uint8_t   errorCount;           // Fehlerzähler für Empfänger
  uint8_t   scanCount;            // Anzahl kompletter Empfangsverluste (Neusynchronisierung)
  uint32_t  errorSum;             // Anzahl verlorener Datenpakete
  uint32_t  okSum;                // Anzahl korrekter Datenpakete
  uint16_t  frameLost;            // Anzahl verlorener Frames
  int8_t    min;
  int8_t    max;
  uint8_t   minAb;
}__attribute__((packed)) State;

typedef struct fastppm
{
  uint8_t toggleC[4];
  uint8_t toggleD[4];
  uint8_t nextdelay[4];
  uint16_t raw[4];
}__attribute__((packed)) FASTPPM;

typedef struct t_OutputData
{
//  uint8_t   chanFlag;
  int16_t   chan_1us[MAXCHAN];    // Werte aller 8 Kanäle
//  uint8_t   timeOut[MAXCHAN];     // Timer für FailSafe
//  uint8_t   timer[MAXCHAN];       // ms seit letzter Ausgabe
  uint8_t   portCflg;
  uint8_t   portDflg;
  uint8_t   pulsesOffset;
  uint8_t   pulsesTimer[MAXCHAN];
//  uint16_t  pulses2MHz[18];
//  uint8_t   act;                  // Ringpuffer aktuelle Ausgabe
  uint8_t   idx;
  bool      port;
//  uint8_t   nextFree;             // Ringpuffer PPM
//  uint8_t   chanPtr:4;          // Kanalnummer aktuelle Ausgabe an die Servos
//  uint8_t   chanMax;              // Anzahl der übertragenen Kanäle
//  uint8_t   FailSafeFlg;        // Kanalwert rausschicken
  uint8_t   latenzMin;
  uint8_t   latenzMax;
}__attribute__((packed)) OutputData;

typedef struct t_MessageBind
{
  BindData data;
  uint8_t  rssi;
  uint8_t  lqi:7;
  uint8_t  crcOk:1;
}__attribute__((packed)) MessageBind;

typedef struct t_Message
{
  MessageData data;
  uint8_t     rssi;
  uint8_t     lqi:7;
  uint8_t     crcOk:1;
}__attribute__((packed)) Message;

typedef struct t_ChannelData
{
  int8_t rssi;
  uint8_t crcOk:1;
  uint8_t lqi:7;
}__attribute__((packed)) ChannelData;

//typedef struct t_ppm
//{
//  uint16_t timer;
//  uint8_t  idx;
//}__attribute__((packed)) PPM;

//typedef struct chan
//{
//  uint8_t chan;
//  uint8_t delayIndex;
//}__attribute__((packed)) CHANSORT;

enum receiver
{
  Start,
//  RxWaitStart,
  checkRSSI,
//  waitForBind,
//  searchFreeChan,             // Freien Kanal suchen
  waitForData,                // Warten bis Daten empfangen
//  RxFill,
  Main,                       // Empfänger einstellen, Kalibrieren
//  RxWait,                    // 1ms warten
  TxOn,                        // Daten schreiben und Sender aktivieren
//  TxWait,                      // Telemetrie senden
//  TxWait2,                      // Telemetrie senden
  RxOn
//  RxWaitBind                  // 1ms warten
};

#define L_SET_FAILSAVE 0
#define L_BIND_ON 1
#define L_xxxx 2
#define L_SPI_ERROR 3
#define L_xx 4
#define L_xxx 5
#define L_TX_NOT_READY 6
#define L_INIT_ERROR 7
