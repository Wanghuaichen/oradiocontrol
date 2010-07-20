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

#include "ORC.h"

typedef struct t_BindData
{
  uint16_t      id;     //sync
  uint8_t       step;
}__attribute__((packed)) BindData;

typedef struct t_FailSafeData
{
  uint8_t   failSafeMode[2];        // 0 failSafePos, 1 hold, 2 = off
  uint8_t   failSafeDelay[8];       // Zeit bis failsafepos anspricht
  uint16_t  failSafePos[8];         // Position bei FailSafe
}__attribute__((packed)) FailSafeData;

typedef struct t_EEData
{
  BindData      bind;
  FailSafeData  failSafe;
}__attribute__((packed)) EEData;

typedef struct t_State
{
  bool      bindmode:1;
  bool      ngmode:1;         // new generation mode
//  bool      scan:1;
//  bool      lowLqiMode:1;
//  bool      lastRxOkChanIdx:2;
//  uint8_t   RxTimeOut33;
//  uint8_t   RxTimeOut;
  uint8_t   RxCount;        // 0 - 7
  uint8_t   actChan;        // Eingestellter Kanal
  uint8_t   actAnt:1;         // Eingestellte Antenne
  uint8_t   actFreqIdx:3;
  uint8_t   dummy:4;
  uint8_t   error;              // Bitkodierte Fehler für LED
  uint16_t  errorcount;         // Fehlerzähler für Empfänger
}__attribute__((packed)) State;

typedef struct t_OutputData
{
  uint8_t   chanFlag;
  uint16_t  chan_1us[8];       // Werte aller 8 Kanäle
  uint8_t   timeOut[8];          // Timer für FailSafe
  uint16_t  pulses2MHz[18];
  uint8_t   chanPtr:4;         // Kanalnummer aktuelle Ausgabe an die Servos
  uint8_t   chanMax:4;         // Anzahl der übertragenen Kanäle
}__attribute__((packed)) OutputData;

typedef struct t_MessageBind
{
  BindData  data;
  uint8_t rssi;
  uint8_t crcOk:1;
  uint8_t lqi:7;
}__attribute__((packed)) MessageBind;

typedef struct t_Message
{
  MessageData  data;
  uint8_t rssi;
  uint8_t crcOk:1;
  uint8_t lqi:7;
}__attribute__((packed)) Message;

typedef struct t_ChannelData
{
  int8_t rssi;
  uint8_t crcOk:1;
  uint8_t lqi:7;
}__attribute__((packed)) ChannelData;

enum receiver
{
  Start,
  RxWaitStart,
  checkRSSI,
//  waitForBind,
//  searchFreeChan,             // Freien Kanal suchen
  waitForData,                // Warten bis Daten empfangen
  Main,                       // Empfänger einstellen, Kalibrieren
  RxWait,                    // 1ms warten
  TxOn,                        // Daten schreiben und Sender aktivieren
  TxWait,                      // Telemetrie senden
  RxOn
//  RxWaitBind                  // 1ms warten
};
