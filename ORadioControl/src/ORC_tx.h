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

typedef struct t_EEData
{
  uint16_t      id;     //sync
  uint8_t       step;
}__attribute__((packed)) EEData;

typedef struct t_State
{
//  bool      bindmode:1;
//  bool      newIdMode:1;
//  bool      keyMode:1;
  bool      SetFaileSafe:1;
//  bool      NewFrame:1;
  uint8_t   actChan;        // Eingestellter Kanal
  uint8_t   maxChan;
  uint8_t   txCount;
  uint8_t   error;
  uint16_t  errorcount;
}__attribute__((packed)) State;

//typedef struct t_txMessage
//{
//  uint8_t   mode:1;
//  MessageData data;
//}__attribute__((packed)) txMessage;

typedef struct t_OutputData
{
  uint16_t  chan_1us[8];       // Werte aller 8 Kanäle
  uint8_t   chanPtr;           // Kanalnummer aktuelle Ausgabe
  uint8_t   chanNew;
}__attribute__((packed)) OutputData;

typedef struct t_TelemetrieReceive
{
  Telemetrie data;
  uint8_t rssi;
  uint8_t crcOk:1;
  uint8_t lqi:7;
}__attribute__((packed)) TelemetrieReceive;

enum transmitter
{
  Start,
  TxReady,                      // auf FSTXON wechseln
  TxOn,                        // Daten schreiben und Sender aktivieren
  RxOn,                       // Empfänger einstellen, Kalibrieren
  RxWait2,                    // 1ms warten
  RxWait3,
  RxCalc,
  TxWait,                      // Bind- Mode
  TxBindCheck,
  TxNextChanBind,
  TxOnBind,
  TxWaitBind
};