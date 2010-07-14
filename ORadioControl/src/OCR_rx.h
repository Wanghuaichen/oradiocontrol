/*
 * OCR_rx.h
 *
 *  Created on: 02.07.2010
 *      Author: josef
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
  uint8_t   RxTimeOut33;
  uint8_t   RxTimeOut;
//  uint8_t   RxTimer;        // 0 - 255
  uint8_t   actChan;        // Eingestellter Kanal
  uint8_t   actAnt:1;         // Eingestellte Antenne
  int8_t    actFreqIdx:3;
  uint8_t   dummy:3;
  uint8_t   error;
  uint16_t  errorcount;
}__attribute__((packed)) State;

typedef struct t_OutputData
{
  uint16_t  chan_1us[8];       // Werte aller 8 Kanäle
  uint8_t   timeOut[8];          // Timer für FailSafe
  uint16_t  pulses2MHz[18];
  uint8_t   chanPtr:4;         // Kanalnummer aktuelle Ausgabe an die Servos
  uint8_t   chanMax:4;         // Anzahl der übertragenen Kanäle
}__attribute__((packed)) OutputData;

typedef struct t_ChannelData
{
  uint8_t rssi;
  uint8_t lqi;                // Eigentlich nur 7 Bit
}__attribute__((packed)) ChannelData;

enum receiver
{
  WaitPPM,                    // Warten auf Framestart
  Wait,                       // 1ms warten
  TXready,                    // auf FSTXON wechseln
  TXon                        // Daten schreiben und Sender aktivieren
};

