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
extern void cc2500CommandStrobe(uint8_t);
extern uint8_t cc2500ReadStatusReg(uint8_t);
extern uint8_t cc2500GetState(void);
extern uint8_t cc2500WriteReg(uint8_t, uint8_t);
extern void cc2500WriteRegCheckIdle(uint8_t, uint8_t);
extern uint8_t cc2500ReadReg(uint8_t);
extern void cc2500_Init(uint8_t);
extern uint8_t get_RxCount(void);
extern void cc2500FlushReceiveData(void);
extern void cc2500ReadFIFOBlock(uint8_t *, uint8_t);
extern void cc2500ReadSingle(uint8_t *, uint8_t);
extern uint8_t cc2500WriteFIFOBlock(uint8_t *, uint8_t);
extern void cc2500WriteSingle(uint8_t *, uint8_t);
extern void setFrequencyOffset(int8_t, bool);
extern uint8_t get_Data(void);
extern void cc2500setPatableMax(uint8_t);
extern void cc2500_Off(void);
extern void cc2500Idle(void);
extern bool checkcc2500(void);
extern void cc2500StartCal(void);
extern void calibrateOff(void);
extern void calibrateOn(void);
extern void stayInRx(void);
extern void gotoIdle(void);
extern uint8_t cc2500IdleGetRXB(void);

extern prog_uint8_t cc2500InitValue[45];

#define NOP() { __asm__ __volatile__ ("nop"); }
#define SET_BIT(port,bit)  (port |=  (1<<bit))
#define RES_BIT(port,bit)  (port &= (uint8_t)~(1<<bit))

#define OUT_B_SPI_SS PORTB2
#define OUT_B_SPI_MOSI PORTB3
#define INP_B_SPI_MISO PIN4
#define OUT_B_SPI_SCK PORTB5
#define INP_D_CC2500_GDO0 PIND2
#define INP_D_CC2500_GDO2 PIND3

#define BINDMODEID 0x1009
#define BINDMODESTEP 64
#define K_DUMMY 0x0000
#define K_SETFAILESAFEPOS 0x0001
#define K_MODEFAILSAFE
#define MAXHOPPCHAN 195               // muss ungerade sein
#define MAXCHAN 8

#define CHANTIME (F_CPU * 10 / 256 / 4730 - 1)
#if (CHANTIME > 255)
 #error Zykluszeit zu gross!!
#endif
#define CHANTIME04 ((CHANTIME + 1) * 10 / 25 - 1)
#if (CHANTIME04 > 255)
 #error Zykluszeit zu gross!!
#endif
#define CHANTIME06 ((CHANTIME + 1) - (CHANTIME04 + 1) - 1)
#if (CHANTIME06 > 255)
 #error Zykluszeit zu gross!!
#endif
#define CHANTIME15 ((CHANTIME + 1) * 15 / 10 - 1)
#if (CHANTIME15 > 255)
 #error CHANTIME15 Zykluszeit zu gross!!
#endif
#define CHANTIME2 (CHANTIME * 2 + 1)
#if (CHANTIME2 > 255)
 #error Zykluszeit zu gross!!
#endif
#define CHANTIME3 ((CHANTIME + 1) * 3 - 1)
#if (CHANTIME3 > 255)
 #error Zykluszeit zu gross!!
#endif
#define TELETIME (F_CPU * 10 / 256 / 3333 - 1)
#if (TELETIME > 255)
 #error Zykluszeit zu gross!!
#endif
#define TELETIME02 ((TELETIME + 1) * 10 / 50 - 1)
#if (TELETIME02 > 255)
 #error Zykluszeit zu gross!!
#endif
#define TELETIME08 ((TELETIME + 1) - (TELETIME02 + 1) - 1)
#if (TELETIME08 > 255)
 #error Zykluszeit zu gross!!
#endif

typedef struct t_BindData
{
  uint16_t      id;     //sync
  uint8_t       step;
}__attribute__((packed)) BindData;


typedef struct t_MessageChan
{
  uint16_t   chan1_1us:11;       // Wert
  uint16_t   chan2_1us:11;       // Wert
  uint16_t   chan3_1us:11;       // Wert
  uint16_t   chan4_1us:11;       // Wert
  uint8_t    type:3;
  uint8_t    rts:1;
}__attribute__((packed)) MessageChan;    // 44 + 4 = 48 Bit = 6 Byte

typedef struct t_MessageFailSafe
{
  uint8_t   FailTime1:4;
  uint8_t   FailMode1:2;
  uint8_t   FailTime2:4;
  uint8_t   FailMode2:2;
  uint8_t   FailTime3:4;
  uint8_t   FailMode3:2;
  uint8_t   FailTime4:4;
  uint8_t   FailMode4:2;         // 3 Byte
  uint8_t   dummy1;
  uint8_t   seq;
  uint8_t   dummy2:4;
  uint8_t   type:3;
  uint8_t   rts:1;
}__attribute__((packed)) MessageFailSafe;

typedef struct t_MessageCommand
{
  uint32_t  command;
  uint8_t   seq;
  uint8_t   dummy2:4;
  uint8_t   type:3;
  uint8_t   rts:1;
}__attribute__((packed)) MessageCommand;

typedef union t_MessageData
{
  MessageCommand  command;
  MessageChan     channel;
  MessageFailSafe failSafe;
}__attribute__((packed)) MessageData;

typedef struct t_Telemetrie
{
  uint32_t  data;
  uint8_t   sensor;
  uint8_t   seq;
}__attribute__((packed)) Telemetrie;

/*eof*/
/*
0000
1xxx    Sendeaufforderung
x000    Kanal 1 - 4
x001    Kanal 5 - 8
x010    Kanal 9 - 12
x011    Kanal 13 - 16
x100    ??
x101    ??
x110    ??
x111    Telemetrie

2,125ms für Telegramm nach oben
2,125ms für Telegramm nach oben
2,125ms für Telegramm nach oben
2,125ms für Telegramm nach oben
2,125ms für Telegramm nach oben
2,125ms für Telegramm nach oben
2,125ms für Telegramm nach oben
2,125ms für Telegramm nach oben
3ms für Telemetrie nach unten */
