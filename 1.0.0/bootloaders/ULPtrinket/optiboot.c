/* VUSBtinyBoot by me@frank-zhao.com
 *  
 * VUSBtinyBoot is a bootloader that emulates a USBtinyISP (from Adafruit Industries)
 *  
 * Trinket Pro (from Adafruit Industries) will use VUSBtinyBoot
 *
 * This code is heavily derived from USBaspLoader, but also from USBtiny, with USBtinyISP's settings

 * This particular file is mostly a copy of optiboot (April 11, 2013 version) http://optiboot.googlecode.com
 * so that Trinket Pro supports both USB and UART bootloading

   Copyright (c) 2013 Adafruit Industries
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the name of the authors nor the names of its contributors
     may be used to endorse or promote products derived from this software
     without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
 */

#define FROM_OPTIBOOT_C
#include "optiboot.h"
#include <util/delay.h>
#include <avr/wdt.h>

static uint8_t  buff[SPM_PAGESIZE * 2];
static uint16_t address = 0;
static uint8_t  length;

char optibootPoll()
{
  unsigned char ch;
  if (UART_SRA & _BV(RXC0))
  {
    ch = getch();

    if(ch == STK_GET_PARAMETER) {
      unsigned char which = getch();
      if (verifySpace()) return 2;
      if (which == 0x82) {
        /*
         * Send optiboot version as "minor SW version"
         */
        putch(OPTIBOOT_MINVER);
      } else if (which == 0x81) {
          putch(OPTIBOOT_MAJVER);
      } else {
        /*
         * GET PARAMETER returns a generic 0x03 reply for
         * other parameters - enough to keep Avrdude happy
         */
        putch(0x03);
      }
    }
    else if(ch == STK_SET_DEVICE) {
      // SET DEVICE is ignored
      if (getNch(20)) return 2;
    }
    else if(ch == STK_SET_DEVICE_EXT) {
      // SET DEVICE EXT is ignored
      if (getNch(5)) return 2;
    }
    else if(ch == STK_LOAD_ADDRESS) {
      // LOAD ADDRESS
      uint16_t newAddress;
      newAddress = getch();
      newAddress = (newAddress & 0xff) | (getch() << 8);
#ifdef RAMPZ
      // Transfer top bit to RAMPZ
      RAMPZ = (newAddress & 0x8000) ? 1 : 0;
#endif
      newAddress += newAddress; // Convert from word address to byte address
      address = newAddress;
      if (verifySpace()) return 2;
    }
    else if(ch == STK_UNIVERSAL) {
      // UNIVERSAL command is ignored
      if (getNch(4)) return 2;
      putch(0x00);
    }
    /* Write memory, length is big endian and is in bytes */
    else if(ch == STK_PROG_PAGE) {
      // PROGRAM PAGE - we support flash programming only, not EEPROM
      uint8_t *bufPtr;
      uint16_t addrPtr;

      getch();                  /* getlen() */
      length = getch();
      getch();

      // If we are in RWW section, immediately start page erase
      if (address < NRWWSTART) __boot_page_erase_short((uint16_t)(void*)address);

      // While that is going on, read in page contents
      bufPtr = buff;
      do *bufPtr++ = getch();
      while (--length);

      // If we are in NRWW section, page erase has to be delayed until now.
      // Todo: Take RAMPZ into account (not doing so just means that we will
      //  treat the top of both "pages" of flash as NRWW, for a slight speed
      //  decrease, so fixing this is not urgent.)
      if (address >= NRWWSTART) __boot_page_erase_short((uint16_t)(void*)address);

      // Read command terminator, start reply
      if (verifySpace()) return 2;

      // If only a partial page is to be programmed, the erase might not be complete.
      // So check that here
      boot_spm_busy_wait();

      // Copy buffer into programming buffer
      bufPtr = buff;
      addrPtr = (uint16_t)(void*)address;
      ch = SPM_PAGESIZE / 2;
      do {
        uint16_t a;
        a = *bufPtr++;
        a |= (*bufPtr++) << 8;
        __boot_page_fill_short((uint16_t)(void*)addrPtr,a);
        addrPtr += 2;
      } while (--ch);

      // Write from programming buffer
      __boot_page_write_short((uint16_t)(void*)address);
      boot_spm_busy_wait();

#if defined(RWWSRE)
      // Reenable read access to flash
      boot_rww_enable();
#endif

    }
    /* Read memory block mode, length is big endian.  */
    else if(ch == STK_READ_PAGE) {
      // READ PAGE - we only read flash
      getch();                  /* getlen() */
      length = getch();
      getch();

      if (verifySpace()) return 2;
      do {
#if defined(RAMPZ)
        // Since RAMPZ should already be set, we need to use EPLM directly.
        // Also, we can use the autoincrement version of lpm to update "address"
        //      do putch(pgm_read_byte_near(address++));
        //      while (--length);
        // read a Flash and increment the address (may increment RAMPZ)
        __asm__ ("elpm %0,Z+\n" : "=r" (ch), "=z" (address): "1" (address));
#else
        // read a Flash byte and increment the address
        __asm__ ("lpm %0,Z+\n" : "=r" (ch), "=z" (address): "1" (address));
#endif
        putch(ch);
      } while (--length);
    }

    /* Get device signature bytes  */
    else if(ch == STK_READ_SIGN) {
      // READ SIGN - return what Avrdude wants to hear
      if (verifySpace()) return 2;
      putch(SIGNATURE_0);
      putch(SIGNATURE_1);
      putch(SIGNATURE_2);
    }
    else if (ch == STK_LEAVE_PROGMODE) { /* 'Q' */
      // Adaboot no-wait mod
      wdt_enable(WDTO_30MS);
      if (verifySpace()) return 2;
      putch(STK_OK);
      //app_start(); // jump to user app
      return 2;  // never reached!
    }
    else {
      // This covers the response to commands like STK_ENTER_PROGMODE
      if (verifySpace()) return 2;
    }
    putch(STK_OK);
    return 1;
  }
  return 0;
}

void putch(char ch) {
  while (!(UART_SRA & _BV(UDRE0)));
  UART_UDR = ch;
}

uint8_t getch(void) {
  uint8_t ch;

#ifdef LED_DATA_FLASH
#if defined(__AVR_ATmega8__) || defined (__AVR_ATmega32__)
  LED_PORT ^= _BV(LED);
#else
  LED_PIN |= _BV(LED);
#endif
#endif

  int32_t timeout = OPTIBOOT_UART_TIMEOUT;
  while(!(UART_SRA & _BV(RXC0))) {
    timeout-=10;
    if (timeout <= 0) {
      app_start(); // jump to user app
    }
    _delay_us(10);
  }
  
  ch = UART_UDR;

#ifdef LED_DATA_FLASH
#if defined(__AVR_ATmega8__) || defined (__AVR_ATmega32__)
  LED_PORT ^= _BV(LED);
#else
  LED_PIN |= _BV(LED);
#endif
#endif

  return ch;
}

// return 0 on success
char getNch(uint8_t count) {
  do getch(); while (--count);
  if (verifySpace()) return 2;
  return 0;
}

// return 0 on success
char verifySpace() {
  if (getch() != CRC_EOP) {
    return 1;
  }
  putch(STK_INSYNC);
  return 0;
}

void optiboot_init(void)
{
  #if defined(__AVR_ATmega8__) || defined (__AVR_ATmega32__)
    UCSRA = _BV(U2X); //Double speed mode USART
    UCSRB = _BV(RXEN) | _BV(TXEN);  // enable Rx & Tx
    UCSRC = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0);  // config USART; 8N1
    UBRRL = (uint8_t)( (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 );
  #else
    UART_SRA = _BV(U2X0); //Double speed mode USART0
    UART_SRB = _BV(RXEN0) | _BV(TXEN0);
    UART_SRC = _BV(UCSZ00) | _BV(UCSZ01);
    UART_SRL = (uint8_t)( (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 );
    // set RX pin to have a pullup?
    DDRD |= _BV(PD0);
  #endif
}
