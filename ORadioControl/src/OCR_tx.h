/*
 * OCR_rx.h
 *
 *  Created on: 02.07.2010
 *      Author: josef
 */

#include "ORC.h"

typedef struct t_EEData
{
  uint16_t      id;     //sync
  uint8_t       step;
}__attribute__((packed)) EEData;

typedef struct t_State
{
  bool      bindmode:1;
  bool      newIdMode:1;
  bool      keyMode:1;
  bool      NewFrame:1;
  uint8_t   actChan;        // Eingestellter Kanal
  uint8_t   error;
  uint8_t   FaileSafeTimer;
  uint16_t  errorcount;
}__attribute__((packed)) State;

typedef struct t_OutputData
{
  uint16_t  chan_1us[8];       // Werte aller 8 Kanäle
  uint8_t   chanPtr;         // Kanalnummer aktuelle Ausgabe
}__attribute__((packed)) OutputData;

enum transmitter
{
  WaitPPM,                    // Warten auf Framestart
  Wait,                       // 1ms warten
  TXready,                    // auf FSTXON wechseln
  TXon                        // Daten schreiben und Sender aktivieren
};
