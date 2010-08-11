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
#else
  typedef uint8_t bool;
  #define APM
  #define true 1
  #define false 0
#endif

#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#define F_CPU 8000000UL  // 8 MHz
#include <util/delay.h>
#define pgm_read_adr(address_short) pgm_read_word(address_short)
#include <avr/wdt.h>

#include "cc2500.h"

///// liefert Betrag des Arguments
//template<class t> inline t abs(t a){ return a>0?a:-a; }
///// liefert das Minimum der Argumente
//template<class t> inline t min(t a, t b){ return a<b?a:b; }
///// liefert das Maximum der Argumente
//template<class t> inline t max(t a, t b){ return a>b?a:b; }

typedef PROGMEM APM void (*FuncP_PROGMEM)(void);

#define low(Data) (*((unsigned char *)(&Data)))
#define high(Data) (*((unsigned char *)((&Data)+1)))


extern void SPI_MasterInit(void);
extern uint8_t SPI_MasterTransmit(uint8_t);
extern void SPI_MasterTransmit_void(uint8_t);
extern void SPI_MasterWriteReg(uint8_t, int8_t);
extern int8_t SPI_MasterReadReg(uint8_t);
extern void cc2500_Init(uint8_t);
extern uint8_t get_RxCount(void);
extern void cc2500FlushData(void);
extern void cc2500_RxOn(void);
extern void cc2500ReadBlock(int8_t *, uint8_t);
extern void cc2500ReadSingle(int8_t *, uint8_t);
extern void cc2500WriteBlock(int8_t *, uint8_t);
extern void cc2500WriteSingle(int8_t *, uint8_t);
extern void setFrequencyOffset(void);
extern uint8_t get_Data(void);
extern void cc2500setPatableMax(uint8_t);
extern void cc2500_Off(void);
extern void cc2500Idle(void);

extern prog_uint8_t cc2500InitValue[41];

#define NOP() { __asm__ __volatile__ ("nop"); }
#define SET_BIT(port,bit)  (port |=  (1<<bit))
#define RES_BIT(port,bit)  (port &= (uint8_t)~(1<<bit))
#define WAIT_SPI_READY {while(!(SPSR & (1<<SPIF)));}
#define WAIT_C2500_READY {while(PIND & (1<<INP_D_CC2500_GDO2)) NOP();}
#define CS_C2500_ACTIV { RES_BIT(PORTB, OUT_B_SPI_SS); NOP(); while(PINB & (1<<OUT_B_SPI_SS)) NOP(); while(PINB & (1 << INP_B_SPI_MISO))NOP();}
#define CS_C2500_OFF { SET_BIT(PORTB, OUT_B_SPI_SS); NOP(); while(!(PINB & (1<<OUT_B_SPI_SS))) NOP();}

#define OUT_B_SPI_SS PORTB2
#define OUT_B_SPI_MOSI PORTB3
#define INP_B_SPI_MISO PIN4
#define OUT_B_SPI_SCK PORTB5
#define INP_D_CC2500_GDO0 PIND2
#define INP_D_CC2500_GDO2 PIND3

#define BINDMODEID 0x1009
#define K_DUMMY 0x0000
#define K_SETFAILESAFEPOS 0x0001
#define K_MODEFAILSAFE
#define MAXHOPPCHAN 195               // muss ungerade sein
#define MAXCHAN 8

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
