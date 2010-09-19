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

typedef struct t_EEData
{
//  uint8_t       flyMode:1;
  BindData      bind;
  uint8_t       power;
  uint16_t      checksum;
}__attribute__((packed)) EEData;

typedef struct t_State
{
  bool      bindmode:1;
//  bool      newIdMode:1;
//  bool      keyMode:1;
  bool      SetFaileSafe:1;
  bool      interrupt:1;
//  bool      NewFrame:1;
  uint8_t   actChan;        // Eingestellter Kanal
//  uint8_t   maxChan;
  uint8_t   txCount;
  uint8_t   ledError;
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
//  uint8_t   chanPtr;
  uint8_t   chanNew;
}__attribute__((packed)) OutputData;

typedef struct t_TelemetrieReceive
{
  Telemetrie data;
  uint8_t rssi;
  uint8_t lqi:7;
  uint8_t crcOk:1;
}__attribute__((packed)) TelemetrieReceive;

enum transmitter
{
  Start,
  TxReady,                    // auf FSTXON wechseln
//  TxAmpOn,
  TxOn,                       // Daten schreiben und Sender aktivieren
  RxOn,                       // Empfänger einstellen, Kalibrieren
//  RxWait2,                    // 1ms warten
//  RxWait3,
  RxCalc,
  TxBindCheck,
  TxNextChanBind,
  TxOnBind,
  TxWaitBind
};

enum uart
{
  WaitToken,
  ReadChan,
  ReadMSB,
  ReadLSB,
  ReadTelemetrieMSB,
  ReadTelemetrieLSB
};

#define L_SET_FAILSAVE 0
#define L_BIND_ON 1
#define L_EEPROM_ERR 2
#define L_SPI_ERROR 3
#define L_NOT_TX 4
#define L_TX_NOT_RX 5
#define L_TX_NOT_READY 6
#define L_INIT_ERROR 7
