/*
 * Author	Thomas Husterer <thus1@t-online.de>
 * Author Josef Glatthaar <josef.glatthaar@googlemail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/
#define VERS 1

#include <inttypes.h>
#include <string.h>
///opt/cross/avr/include/avr/pgmspace.h
  #include <stddef.h>
  #include <avr/io.h>
  #define assert(x)
  //disable whole pgmspace functionality for all avr-gcc because
  //avr-gcc > 4.2.1 does not work anyway
  //http://www.mail-archive.com/gcc-bugs@gcc.gnu.org/msg239240.html
  //http://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734
  //
  //Workarounds:
  //
  //PSTR is fixed below
  //all prog_xx definitions must use APM explicitely

  //#define __ATTR_PROGMEM__   
  #include <avr/pgmspace.h>
  #ifdef __cplusplus
    #define APM __attribute__(( section(".progmem.data") ))
    #undef PSTR
    #define PSTR(s) (__extension__({static prog_char APM __c[] = (s);&__c[0];}))
  #endif

  #include <avr/eeprom.h>
  #include <avr/sleep.h>
  #include <avr/interrupt.h>
  #define F_CPU 8000000UL  // 8 MHz
  #include <util/delay.h>
  #define pgm_read_adr(address_short) pgm_read_word(address_short)
  #include <avr/wdt.h>

#include "cc2500.h"

/// liefert Betrag des Arguments
template<class t> inline t abs(t a){ return a>0?a:-a; }
/// liefert das Minimum der Argumente
template<class t> inline t min(t a, t b){ return a<b?a:b; }
/// liefert das Maximum der Argumente
template<class t> inline t max(t a, t b){ return a>b?a:b; }

typedef struct t_BindDataCor
{
  uint8_t   chan[3];          // Funkkanäle
  uint32_t  id;              // eindeutige ID
}__attribute__((packed)) BindDataCor;

typedef struct t_BindDataNg
{
  uint8_t   chan[64];         // Funkkanäle die zum Jumpen benutzt werden
  uint8_t   step;             // Sprungweite
  uint32_t  id;              // eindeutige ID
}__attribute__((packed)) BindDataNg;

typedef union t_BindData
{
  BindDataCor corona;
  BindDataNg  ng;
}__attribute__((packed)) BindData;

typedef struct t_FailSafeData
{
  uint8_t   failSafeMode[8];
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
/*eof*/
