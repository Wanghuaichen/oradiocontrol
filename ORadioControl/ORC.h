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
#define high(Data) (*((unsigned char *)(((unsigned char *)&Data)+1)))


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
extern bool cc2500ReadFIFOBlock(uint8_t *, uint8_t);
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
extern void calibrateFast(void);
extern void calibrateSlow(void);
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
#define MAXHOPPCHAN 195               // muss ungerade sein, wird 2x pro Sekunde durchlaufen
#define MAXCHAN 8

#define CHANTIME (F_CPU * 10 / 256 / 4660 - 1)        // (66) 2,144 ms
#if (CHANTIME > 255)
 #error Zykluszeit zu gross!!
#endif
#define CHANTIME02 ((CHANTIME + 1) * 10 / 50 - 1)      // 0,4 ms
#if (CHANTIME02 > 255)
 #error Zykluszeit zu gross!!
#endif
#define CHANTIME04 ((CHANTIME + 1) * 10 / 25 - 1)      // 0,8576 ms
#if (CHANTIME04 > 255)
 #error Zykluszeit zu gross!!
#endif
#define CHANTIME06 ((CHANTIME + 1) - (CHANTIME04 + 1) - 1)  // 1,286 ms
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
#define TELETIME (F_CPU * 10 / 256 / 3333 - 1)        // 3,003 ms
#if (TELETIME > 255)
 #error Zykluszeit zu gross!!
#endif
#define TELETIME02 ((TELETIME + 1) * 10 / 50 - 1)     // 0,6 ms
#if (TELETIME02 > 255)
 #error Zykluszeit zu gross!!
#endif
#define TELETIME08 ((TELETIME + 1) - (TELETIME02 + 1) - 1)    // 2,4 ms
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
  uint8_t    chan_1uslow[4];
  uint16_t   chan_1ushigh:12;
  uint8_t    type:3;			 // 0 = Gruppe 1-4, 1 = Gruppe 5-8, 2 = Gruppe 9-12, 3 = Gruppe 13-16, 
                                         // 4 = PPM-sync (Gruppe 1-4), 5 = Speicher, 6 = frei, 7 = Kommando
  uint8_t    rts:1;			 // Anforderung Telemetrie vom Empfänger
}__attribute__((packed)) MessageChan;    // 44 + 4 = 48 Bit = 6 Byte

//typedef struct t_MessageFailSafe
//{
//  uint8_t   FailTime1:4;
//  uint8_t   FailMode1:2;
//  uint8_t   FailTime2:4;
//  uint8_t   FailMode2:2;
//  uint8_t   FailTime3:4;
//  uint8_t   FailMode3:2;
//  uint8_t   FailTime4:4;
//  uint8_t   FailMode4:2;         // 3 Byte
//  uint16_t  command;
//  uint8_t   dummy2:4;
//  uint8_t   type:3;                     // 7
//  uint8_t   rts:1;
//}__attribute__((packed)) MessageFailSafe;

typedef struct t_MessageCommand
{
  uint8_t   data;
  uint16_t  subcommand;
  uint16_t  command;
  uint8_t   des:4;
  uint8_t   type:3;                     // 7
  uint8_t   rts:1;
}__attribute__((packed)) MessageCommand;

typedef struct t_MessageUART
{
  uint8_t   b[5];
  uint8_t   b0:7;
  uint8_t   rts:1;
}__attribute__((packed)) MessageUART;

typedef struct t_MessageMemoryByte
{
  uint8_t   checksum;
  uint8_t   data;               // Wert
  uint8_t   *adr;                // Adresse
  uint8_t   des;                // Zielgerät
  uint8_t   wr:1;
  uint8_t   size:1;            //
  uint8_t   tar:2;               // 0 eeprom, 1 ram, 2 flash
  uint8_t   type:3;             // 5
  uint8_t   rts:1;
}__attribute__((packed)) MessageMemoryByte;

typedef struct t_MessageMemoryWord
{
  uint16_t  data;               // Wert
  uint16_t  *adr;                // Adresse
  uint8_t   des;                // Zielgerät
  uint8_t   wr:1;
  uint8_t   size:1;            //
  uint8_t   tar:2;               // 0 eeprom, 1 ram, 2 flash
  uint8_t   type:3;             // 5
  uint8_t   rts:1;
}__attribute__((packed)) MessageMemoryWord;

typedef union t_MessageData
{
  MessageCommand    command;
  MessageChan       channel;
//  MessageFailSafe   failSafe;
  MessageUART       Uart;
  MessageMemoryWord MemoryWord;
  MessageMemoryByte MemoryByte;
}__attribute__((packed)) MessageData;

typedef struct t_TelemetrieMemoryByte
{
  uint8_t   checksum;
  uint8_t   data;               // Wert
  uint16_t  adr;                // Adresse
  uint8_t   src;
  uint8_t   dummy2:1;
  uint8_t   size:1;            //
  uint8_t   tar:2;               // 0 eeprom, 1 ram, 2 flash
  uint8_t   type:4;               // 5
}__attribute__((packed)) TelemetrieMemoryByte;

typedef struct t_TelemetrieMemoryWord
{
  uint16_t  data;               // Wert
  uint16_t  adr;                // Adresse
  uint8_t   src;
  uint8_t   dummy2:1;
  uint8_t   size:1;            //
  uint8_t   tar:2;               // 0 eeprom, 1 ram, 2 flash
  uint8_t   type:4;               // 5
}__attribute__((packed)) TelemetrieMemoryWord;

typedef struct t_TelemetrieSensor
{
  uint16_t  data;               // Wert
  uint16_t  sensor;              // Adresse
  uint8_t   dummy5;
  uint8_t   source:4;
  uint8_t   type:4;               // 1
}__attribute__((packed)) TelemetrieSensor;

typedef struct t_TelemetrieStatusRx
{
  uint16_t  frameLost;
  uint16_t  errorSum;
  uint8_t   scanCount;
  uint8_t   source:4;           // 0 -> Sender, 1 -> erster Empfänger, 2 -> zweiter Empfänger
  uint8_t   type:4;             // 0 -> Status, 1 > Sensor, 5 -> read Byte / Word
}__attribute__((packed)) TelemetrieStatusRx;

typedef struct t_TelemetrieStatusTx
{
  uint16_t  Timer33ms;
  uint16_t  Telem_error;
  uint8_t   dummy1;
  uint8_t   source:4;
  uint8_t   type:4;
}__attribute__((packed)) TelemetrieStatusTx;

typedef struct t_TelemetrieUnspec
{
  uint8_t  dataB6;
  uint8_t  dataB5;
  uint8_t  dataB4;
  uint8_t  dataB3;
  uint8_t  dataB2;
  uint8_t  dataB1;
}__attribute__((packed)) TelemetrieUnspec;

typedef union t_Telemetrie
{
  TelemetrieMemoryByte MemoryByte;
  TelemetrieMemoryWord MemoryWord;
  TelemetrieSensor     Sensor;
  TelemetrieUnspec     Unspec;
  TelemetrieStatusRx   StatusRx;
  TelemetrieStatusTx   StatusTx;
}__attribute__((packed)) Telemetrie;
/*eof*/
