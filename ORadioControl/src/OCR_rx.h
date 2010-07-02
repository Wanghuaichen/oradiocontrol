/*
 * OCR_rx.h
 *
 *  Created on: 02.07.2010
 *      Author: josef
 */

#include "ORC.h"

typedef struct t_BindDataCor
{
  uint32_t  id;              // eindeutige ID
  uint8_t   chan[3];          // Funkkanäle
}__attribute__((packed)) BindDataCor;

typedef struct t_BindDataNg
{
//  uint32_t  id;              // eindeutige ID
//  uint8_t   chan[64];         // Funkkanäle die zum Jumpen benutzt werden
  uint8_t   step;             // Sprungweite
}__attribute__((packed)) BindDataNg;

typedef union t_BindData
{
  BindDataCor corona;
  BindDataNg  ng;
}__attribute__((packed)) BindData;

typedef struct t_FailSafeData
{
  uint8_t   failSafeMode[8];      // 0 failSafePos, 1 hold, 2 = off
  uint8_t   failSafeDelay[8];     // Zeit bis failsafepos anspricht
  uint16_t  failSafePos[8];
}__attribute__((packed)) FailSafeData;

typedef struct t_EEData
{
  uint8_t       typ;
  BindData      bind;
  FailSafeData  failSafe;
}__attribute__((packed)) EEData;

typedef struct t_State
{
  bool      bindmode:1;
  bool      ngmode:1;         // new generation mode
  bool      timeOut:1;
  bool      scan:1;
  bool      lowLqiMode:1;
  bool      lastRxOkChanIdx:2;
  uint8_t   actChanIdx:2;        // Eingestellter Kanal
  uint8_t   actAnt:1;         // Eingestellte Antenne
  int8_t    actFreqIdx:3;
  uint8_t   dummy:3;
  uint8_t   error;
  uint16_t  errorcount;
}__attribute__((packed)) State;

typedef struct t_OutputData
{
  uint16_t  chan_1us[8];       // Werte aller 8 Kanäle
  uint16_t  pulses2MHz[18];
  uint8_t   chanPtr;         // Kanalnummer aktuelle Ausgabe
}__attribute__((packed)) OutputData;

typedef struct t_ChannelData
{
  uint8_t rssi;
  uint8_t lqi;                // Eigentlich nur 7 Bit
}__attribute__((packed)) ChannelData;
