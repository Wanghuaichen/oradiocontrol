/*
 * Copyright (C) 2003 by egnite Software GmbH. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY EGNITE SOFTWARE GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL EGNITE
 * SOFTWARE GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/*
 * $Log$
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/boot.h>

#define FOSC 8000000 // Clock Speed
#define BAUD 19200
#define MYUBRR (FOSC/16/BAUD-1)

#define OUT_B_SPI_SS PORTB2
#define OUT_B_SPI_MOSI PORTB3
#define INP_B_SPI_MISO PORTB4
#define OUT_B_SPI_SCK PORTB5
#define INP_B_PPM PINB0
#define OUT_B_CTX PORTB1

#define OUT_C_LED1 PORTC0
#define OUT_C_LED2 PORTC1
#define INP_C_KEY PINC2

#define INP_D_CC2500_GDO0 PIND2
#define INP_D_CC2500_GDO2 PIND3
#define OUT_D_CRX  PORTD5


/*
 * Maximum number of consecutive protocol errors until we give up.
 */
#define MAX_PROTO_ERRORS    10

/*
 * We support the original XMODEM protocol only. If you change this,
 * you need to change a lot more.
 */
#define XMODEM_PACKET_SIZE  128

/*
 * Some ASCII code definitions for readability.
 */
#define ASC_TMO  0x00           /* Timeout special */
#define ASC_SOH  0x01           /* Ctrl-A */
#define ASC_EOT  0x04           /* Ctrl-D */
#define ASC_ACK  0x06           /* Ctrl-F */
#define ASC_NAK  0x15           /* Ctrl-U */
#define ASC_CAN  0x18           /* Ctrl-X */
#define ASC_ERR  0xFF           /* Error special */

/*
 * Indicates timeout if zero.
 */
volatile unsigned short intime;

/*
 * Function prototypes.
 */
static void FlashPage(unsigned long address, unsigned char *data);
static void SendOctet(char ch);
static unsigned char RecvPacket(unsigned char pn, unsigned char *cp);

/*
 * Boot loader entry point.
 *
 * Remember that we are running without any initialization or library 
 * code. All CPU registers and global variables are uninitialized.
 *
 * No code is allowed above this point.
 *
 * Compatibility remarks:
 *
 * - I tried to keep the code compatible. However, when using it for
 *   other CPUs than the ATmega128, the it may not even compile.
 *
 * - This code has been tested on an ATmega128 running at 14.7456 MHz.
 *   For lower clock speeds you may have to reduce the baudrate.
 *
 * - It is assumed, that either one or two XMODEM packets fit into a
 *   single program memory page.
 *
 * - Two routines, SendOctet() and RecvOctet(), are used to read from
 *   or write to the ATmega USART0. You may need to change these
 *   routines on different chips.
 * 
 */
unsigned char buff[XMODEM_PACKET_SIZE];
int main(void)
{
    unsigned char ch;   /* Result of last packet receive or response. */
    unsigned char ec;   /* Error counter. */
    unsigned short pn;  /* Expected packet number. */
    unsigned long addr; /* Program memory byte address. */

    /*
     * GCC depends on register r1 set to 0.
     */
    asm volatile ("clr r1");

    /*
     * No interrupts used.
     */
    cli();

    CLKPR = 0;

    PORTB = 0x0;
    DDRB = (1<<OUT_B_SPI_MOSI) | (1<<OUT_B_SPI_SCK) |
           (1<<OUT_B_SPI_SS) | (1 << OUT_B_CTX);

    PORTC = (1 << OUT_C_LED1);
    DDRC = (1 << OUT_C_LED1) | (1 << OUT_C_LED2);

    PORTD = 0x0;
    DDRD = (1 << OUT_D_CRX);


    /*
     * Initialize UART baudrate and enable transmitter and receiver.
     */
    UBRR0H = (unsigned char)(MYUBRR>>8);          /* Set baud rate */
    UBRR0L = (unsigned char)MYUBRR;
    UCSR0A = 0;
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);           /* Enable receiver and transmitter */
    UCSR0C = (3<<UCSZ00);                     /* Set frame format: 8data, 1stop bit */


    /*
     * Wait for the first packet, cancel or end-of-transmission to arrive. 
     * Continuously send NAKs to initiate the transfer.
     */
    pn = 1; /* XMODEM starts with packet number one. */
    for (;;) {
        ch = RecvPacket(pn, buff);
        if (ch != ASC_TMO && ch != ASC_ERR) {
            break;
        }
        SendOctet(ASC_NAK);
    }

    /*
     * We received a packet or a request to stop the transfer. 
     */
    ec = 0;
    addr = 0;
    for (;;) {

        /*
         * Process a packet.
         */
        if (ch == ASC_SOH)
        {
          if(addr + XMODEM_PACKET_SIZE >= 0x0b00)
            break;
          FlashPage(addr, buff);
          addr += SPM_PAGESIZE;
          FlashPage(addr, buff + SPM_PAGESIZE);
          addr += SPM_PAGESIZE;
          ec = 0; /* Clear error counter. */
          pn++;   /* Increment packet number. */
          ch = ASC_ACK;  /* Respond with ACK. */
        } 
        
        /*
         * Process end-of-transmission or cancel.
         */
        else if (ch == ASC_EOT || ch == ASC_CAN) {
            FlashPage(addr, buff);
            SendOctet(ASC_ACK);
            break;
        } 
        
        /*
         * Anything else is treated as an error.
         */
        else {
            if (ec++ > MAX_PROTO_ERRORS) {
                SendOctet(ASC_CAN);
                break;
            }
            ch = ASC_NAK;
        }

        /*
         * Send response and receive next packet.
         */
        SendOctet(ch);
        ch = RecvPacket(pn, buff);
    }
#if !defined (__AVR_ATmega48__)
    boot_rww_enable ();
#endif
    /*
     * Will jump into the loaded code.
     */
//    __asm__ __volatile__("rjmp 0x0000");

    /*
     * Never return to stop GCC to include a reference to the exit code.
     * Actually we will never reach this point, but the compiler doesn't 
     * understand the assembly statement above.
     */
    return 0;
}

/*
 * Write the contents of a buffer to a specified program memory address.
 *
 * \param addr Program memory byte address to start writing.
 * \param data Points to a buffer which contains the data to be written.
 *
 * \todo Would be fine to verify the result and return a result to the
 *       caller.
 */
static void FlashPage(unsigned long addr, unsigned char *data)
{
    unsigned long i;

    /*
     * Erase page.
     */
    boot_spm_busy_wait();
    boot_page_erase(addr);
    boot_spm_busy_wait();


    /*
     * Fill page buffer.
     */
    for (i = addr; i < addr + SPM_PAGESIZE; i += 2)
    {
        boot_page_fill(i, *data + (*(data + 1) << 8));
        data += 2;
    }

    /*
     * Write page.
     */
    boot_page_write(addr);
    boot_spm_busy_wait();

}

/*
 * Send a byte to the RS232 interface.
 *
 * \param ch Byte to send.
 */
static void SendOctet(char ch)
{
    /* This may differ on your device. */
    while(!(UCSR0A & (1 << UDRE0)));    // while ((inb(UCSR0A) & _BV(UDRE)) == 0);
    UDR0 = ch;
}

/*
 * Receive a byte to the RS232 interface.
 *
 * \return Byte received or ASC_TMO on timeout.
 */
static unsigned char RecvOctet(void)
{
    intime = 1;
    /* This may differ on your device. */
    while (!(UCSR0A & (1 << RXC0))) {
        if (++intime == 0)
            return ASC_TMO;
    }
    return(UDR0);
}

/*
 * Receive a protocol packet.
 *
 * \param pn Expected packet number.
 * \param cp Pointer to packet buffer.
 *
 * \return - ASC_SOH if a packet has been received.
 *         - ASC_CAN if remote cancels transmission.
 *         - ASC_EOT if transmission finished.
 *         - ASC_ERR on packet format errors.
 *         - ASC_TMO on timeouts.
 *
 * \todo I think that this will fail if the remote missed a previously
 *       sent ACK.
 */
static unsigned char RecvPacket(unsigned char pn, unsigned char *cp)
{
    unsigned char rc; /* Function result. */
    unsigned char ch; /* Calculated checksum. */
    unsigned char i;

    /*
     * Wait for the first character. Ignore anything except SOH, EOT, CAN
     * or timeout.
     */
    for (;;) {
        if ((rc = RecvOctet()) == ASC_SOH) {
            break;
        }

        /*
         * Return if transmission stopped or timed out.
         */
        if (!intime || rc == ASC_EOT || rc == ASC_CAN) {
            return rc;
        }
    }

    /*
     * We got the start of the header (SOH). Next byte will be the packet
     * number, followed by the inverted packet number.
     */
    ch = RecvOctet();
    if (RecvOctet() + ch != 0xFF || ch != pn)
        rc = ASC_ERR;

    /*
     * Even if the packet number had been invalid, continue receiving. This
     * avoids too many NAK responses by the caller.
     */
    ch = 0;
    for (i = 0; i < XMODEM_PACKET_SIZE; i++) {
        ch += (*cp++ = RecvOctet());
        if (!intime) {
            return ASC_TMO;
        }
    }

    /*
     * Finally receive the checksum.
     */
    if (ch != RecvOctet() || !intime)
        rc = ASC_ERR;

    return rc;
}
