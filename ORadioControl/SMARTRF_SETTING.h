/* Phase transition time = 0 */
/* Base frequency = 2401.999939 */
/* Carrier frequency = 2481.063965 */
/* Channel number = 195 */
/* Carrier frequency = 2481.063965 */
/* Modulated = true */
/* Modulation format = MSK */
/* Manchester enable = false */
/* Sync word qualifier mode = 30/32 sync word bits detected */
/* Preamble count = 4 */
/* Channel spacing = 405.456543 */
/* Carrier frequency = 2481.063965 */
/* Data rate = 149.963 */
/* RX filter BW = 325.000000 */
/* Data format = Normal mode */
/* Length config = Fixed packet length mode. Length configured in PKTLEN register */
/* CRC enable = true */
/* Packet length = 3 */
/* Device address = 0 */
/* Address config = No address check */
/* CRC autoflush = true */
/*  = false */
/* TX power = 1 */
/***************************************************************
 *  SmartRF Studio(tm) Export
 *
 *  Radio register settings specifed with C-code
 *  compatible #define statements.
 *
 ***************************************************************/

#ifndef SMARTRF_CC2500_H
#define SMARTRF_CC2500_H

#define SMARTRF_RADIO_CC2500

#define SMARTRF_SETTING_FSCTRL1    0x0A
#define SMARTRF_SETTING_IOCFG0     0x06
#define SMARTRF_SETTING_FSCTRL0    0x00
#define SMARTRF_SETTING_FREQ2      0x5C
#define SMARTRF_SETTING_FREQ1      0x62
#define SMARTRF_SETTING_FREQ0      0x76
#define SMARTRF_SETTING_MDMCFG4    0x5C
#define SMARTRF_SETTING_MDMCFG3    0x7A
#define SMARTRF_SETTING_MDMCFG2    0x73
#define SMARTRF_SETTING_MDMCFG1    0x23
#define SMARTRF_SETTING_MDMCFG0    0xFF
#define SMARTRF_SETTING_CHANNR     0xC3
#define SMARTRF_SETTING_DEVIATN    0x00
#define SMARTRF_SETTING_FREND1     0xB6
#define SMARTRF_SETTING_FREND0     0x10
#define SMARTRF_SETTING_MCSM0      0x18
#define SMARTRF_SETTING_FOCCFG     0x1D
#define SMARTRF_SETTING_BSCFG      0x1C
#define SMARTRF_SETTING_AGCCTRL2   0xC7
#define SMARTRF_SETTING_AGCCTRL1   0x00
#define SMARTRF_SETTING_AGCCTRL0   0xB0
#define SMARTRF_SETTING_FSCAL3     0xEA
#define SMARTRF_SETTING_FSCAL2     0x0A
#define SMARTRF_SETTING_FSCAL1     0x00
#define SMARTRF_SETTING_FSCAL0     0x11
#define SMARTRF_SETTING_FSTEST     0x59
#define SMARTRF_SETTING_TEST2      0x88
#define SMARTRF_SETTING_TEST1      0x31
#define SMARTRF_SETTING_TEST0      0x0B
#define SMARTRF_SETTING_FIFOTHR    0x07
#define SMARTRF_SETTING_IOCFG2     0x29
#define SMARTRF_SETTING_PKTCTRL1   0x0C
#define SMARTRF_SETTING_PKTCTRL0   0x44
#define SMARTRF_SETTING_ADDR       0x00
#define SMARTRF_SETTING_PKTLEN     0x03

#define SMARTRF_SETTING_IOCFG2     0x29
#define SMARTRF_SETTING_IOCFG0     0x06
//#define SMARTRF_SETTING_FSCTRL1    0x09
//#define SMARTRF_SETTING_FSCTRL0    0x00
//#define SMARTRF_SETTING_FREQ2      0x5c
//#define SMARTRF_SETTING_FREQ1      0x80
//#define SMARTRF_SETTING_FREQ0      0x00
//#define SMARTRF_SETTING_MDMCFG4    0x5b
//#define SMARTRF_SETTING_MDMCFG3    0xF8
//#define SMARTRF_SETTING_MDMCFG2    0x01       //
//#define SMARTRF_SETTING_MDMCFG1    0x03       //
//#define SMARTRF_SETTING_MDMCFG0    0xF8
//#define SMARTRF_SETTING_CHANNR     0x00
//#define SMARTRF_SETTING_DEVIATN    0x50
//#define SMARTRF_SETTING_FREND1     0x56
#define SMARTRF_SETTING_FREND0     0x10
#define SMARTRF_SETTING_MCSM0      0x18
//#define SMARTRF_SETTING_FOCCFG     0x16
//#define SMARTRF_SETTING_BSCFG      0x6C
//#define SMARTRF_SETTING_AGCCTRL2   0x43
//#define SMARTRF_SETTING_AGCCTRL1   0x58
//#define SMARTRF_SETTING_AGCCTRL0   0x91
//#define SMARTRF_SETTING_FSCAL3     0xA9
//#define SMARTRF_SETTING_FSCAL2     0x0A
//#define SMARTRF_SETTING_FSCAL1     0x00
//#define SMARTRF_SETTING_FSCAL0     0x11
//#define SMARTRF_SETTING_FSTEST     0x59         // o
//#define SMARTRF_SETTING_TEST2      0x88         // a
//#define SMARTRF_SETTING_TEST1      0x31         // a
//#define SMARTRF_SETTING_TEST0      0x0B         // o
//#define SMARTRF_SETTING_FIFOTHR    0x07
//#define SMARTRF_SETTING_PKTCTRL1   0x0c         //
//#define SMARTRF_SETTING_PKTCTRL0   0x44         //
//#define SMARTRF_SETTING_ADDR       0x00         //
//#define SMARTRF_SETTING_PKTLEN     0x03         // 3 Bytes für Binding

#define SMARTRF_SETTING_IOCFG1    0x2e
#define SMARTRF_SETTING_SYNC1     0xd3
#define SMARTRF_SETTING_SYNC0     0x91
#define SMARTRF_SETTING_MCSM2     0x07
#define SMARTRF_SETTING_MCSM1     0x00
#define SMARTRF_SETTING_WOREVT1   0x87
#define SMARTRF_SETTING_WOREVT0   0x6b
#define SMARTRF_SETTING_WORCTRL   0xf8
#define SMARTRF_SETTING_RCCTRL1   0x41
#define SMARTRF_SETTING_RCCTRL0   0x0

#endif
