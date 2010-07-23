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

typedef PROGMEM void (*FuncP_PROGMEM)(void);

extern void SPI_MasterInit(void);
extern uint8_t SPI_MasterTransmit(uint8_t);
extern void SPI_MasterWriteReg(uint8_t, int8_t);
extern int8_t SPI_MasterReadReg(uint8_t);
extern void cc2500_Init(void);
extern uint8_t get_RxCount(void);
extern void cc2500FlushData(void);
extern void cc2500_RxOn(void);
extern void cc2500ReadBlock(int8_t *, uint8_t);
extern void cc2500WriteBlock(int8_t *, uint8_t);
extern void cc2500WriteSingle(int8_t *, uint8_t);
extern void setFrequencyOffset(void);
extern uint8_t get_Data(void);

extern prog_uint8_t APM cc2500InitValue[41];

#define SET_BIT(port,bit)  (port |=  (1<<bit))
#define RES_BIT(port,bit)  (port &= (uint8_t)~(1<<bit))

#define BINDMODEID 0x1009
#define K_DUMMY 0x0000
#define K_SETFAILESAFEPOS 0x0001
#define K_MODEFAILSAFE
#define MAXHOPPCHAN 204

typedef struct t_MessageChan
{
  uint8_t   mode:1;
  uint8_t   channel:4;         // Kanalnummer aktuelle Ausgabe
  uint16_t  chan_1us:11;       // Wert
}__attribute__((packed)) MessageChan;

typedef struct t_MessageFailSafe
{
  uint8_t   mode:1;
  uint8_t   rts:1;
  uint8_t   typ:4;
  uint8_t   channel:4;         // Kanalnummer
  uint8_t   FailMode:2;
  uint8_t   FailTime:4;
}__attribute__((packed)) MessageFailSafe;

typedef struct t_MessageCommand
{
  uint8_t   mode:1;
  uint8_t   rts:1;
  uint16_t  command:14;
}__attribute__((packed)) MessageCommand;

typedef union t_MessageData
{
  MessageCommand command;
  MessageChan channel;
  MessageFailSafe failSafe;
}__attribute__((packed)) MessageData;

typedef struct t_Telemetrie
{
  uint16_t  sensor;
  uint16_t  data;
}__attribute__((packed)) Telemetrie;

/*eof*/
