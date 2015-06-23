#define FUNC_READ 1
#define FUNC_WRITE 1
/**********************************************************/
/* Optiboot bootloader for Arduino                        */
/*                                                        */
/* http://optiboot.googlecode.com                         */
/*                                                        */
/* Arduino-maintained version : See README.TXT            */
/* http://code.google.com/p/arduino/                      */
/*  It is the intent that changes not relevant to the     */
/*  Arduino production envionment get moved from the      */
/*  optiboot project to the arduino project in "lumps."   */
/*                                                        */
/* Heavily optimised bootloader that is faster and        */
/* smaller than the Arduino standard bootloader           */
/*                                                        */
/* Enhancements:                                          */
/*   Fits in 512 bytes, saving 1.5K of code space         */
/*   Higher baud rate speeds up programming               */
/*   Written almost entirely in C                         */
/*   Customisable timeout with accurate timeconstant      */
/*   Optional virtual UART. No hardware UART required.    */
/*   Optional virtual boot partition for devices without. */
/*                                                        */
/* What you lose:                                         */
/*   Implements a skeleton STK500 protocol which is       */
/*     missing several features including EEPROM          */
/*     programming and non-page-aligned writes            */
/*   High baud rate breaks compatibility with standard    */
/*     Arduino flash settings                             */
/*                                                        */
/* Fully supported:                                       */
/*   ATmega168 based devices  (Diecimila etc)             */
/*   ATmega328P based devices (Duemilanove etc)           */
/*                                                        */
/* Beta test (believed working.)                          */
/*   ATmega8 based devices (Arduino legacy)               */
/*   ATmega328 non-picopower devices                      */
/*   ATmega644P based devices (Sanguino)                  */
/*   ATmega1284P based devices                            */
/*   ATmega1280 based devices (Arduino Mega)              */
/*                                                        */
/* Alpha test                                             */
/*   ATmega32                                             */
/*                                                        */
/* Work in progress:                                      */
/*   ATtiny84 based devices (Luminet)                     */
/*                                                        */
/* Does not support:                                      */
/*   USB based devices (eg. Teensy, Leonardo)             */
/*                                                        */
/* Assumptions:                                           */
/*   The code makes several assumptions that reduce the   */
/*   code size. They are all true after a hardware reset, */
/*   but may not be true if the bootloader is called by   */
/*   other means or on other hardware.                    */
/*     No interrupts can occur                            */
/*     UART and Timer 1 are set to their reset state      */
/*     SP points to RAMEND                                */
/*                                                        */
/* Code builds on code, libraries and optimisations from: */
/*   stk500boot.c          by Jason P. Kyle               */
/*   Arduino bootloader    http://arduino.cc              */
/*   Spiff's 1K bootloader http://spiffie.org/know/arduino_1k_bootloader/bootloader.shtml */
/*   avr-libc project      http://nongnu.org/avr-libc     */
/*   Adaboot               http://www.ladyada.net/library/arduino/bootloader.html */
/*   AVR305                Atmel Application Note         */
/*                                                        */
/* Copyright 2013-2015 by Bill Westfield.                 */
/* Copyright 2010 by Peter Knight.                        */
/*                                                        */
/* This program is free software; you can redistribute it */
/* and/or modify it under the terms of the GNU General    */
/* Public License as published by the Free Software       */
/* Foundation; either version 2 of the License, or        */
/* (at your option) any later version.                    */
/*                                                        */
/* This program is distributed in the hope that it will   */
/* be useful, but WITHOUT ANY WARRANTY; without even the  */
/* implied warranty of MERCHANTABILITY or FITNESS FOR A   */
/* PARTICULAR PURPOSE.  See the GNU General Public        */
/* License for more details.                              */
/*                                                        */
/* You should have received a copy of the GNU General     */
/* Public License along with this program; if not, write  */
/* to the Free Software Foundation, Inc.,                 */
/* 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA */
/*                                                        */
/* Licence can be viewed at                               */
/* http://www.fsf.org/licenses/gpl.txt                    */
/*                                                        */
/**********************************************************/

/**********************************************************/
/*                                                        */
/* Optional defines:                                      */
/*                                                        */
/**********************************************************/
/*                                                        */
/* WS2812:                                                */
/* WS2812B LED Data In avr pin (D6 on ULPNode)            */
/*                                                        */
/* WAKE_SWITCH:                                           */
/* Wake up switch avr pin (D4 on ULPNode)                 */
/*                                                        */
/* PWR_SENSOR:                                            */
/* power sensor avr pin (D9 on ULPNode)                   */
/*                                                        */
/* PWR_RF:                                                */
/* power RF Radio module avr pin (D7 on ULPNode)          */
/*                                                        */
/* PWR_BOOST:                                             */
/* DC/DC booster enable avr pin Arduino (A2 on ULPNode)   */
/*                                                        */
/* BIGBOOT:                                               */
/* Build a 1k bootloader, not 512 bytes. This turns on    */
/* extra functionality.                                   */
/* BIGBOOT:                                               */
/*                                                        */
/* Build a 1k bootloader, not 512 bytes. This turns on    */
/* extra functionality.                                   */
/*                                                        */
/* BAUD_RATE:                                             */
/* Set bootloader baud rate (250000Kbps on ULPNode)       */
/*                                                        */
/* SOFT_UART:                                             */
/* Use AVR305 soft-UART instead of hardware UART.         */
/*                                                        */
/* LED_START_FLASHES:                                     */
/* Number of LED flashes on bootup.                       */
/*                                                        */
/* LED_DATA_FLASH:                                        */
/* Flash LED when transferring data. For boards without   */
/* TX or RX LEDs, or for people who like blinky lights.   */
/* Flash RGB LED RED/GREEN on wrtite/read flash 					*/
/* LED_DATA_FLASH value is RGB Brightness, set at least 16*/
/* to flash RGB, WS2812 and LED_START_FLASHES must be set */
/*                                                        */
/* SUPPORT_EEPROM:                                        */
/* Support reading and writing from EEPROM. This is not   */
/* used by Arduino, so off by default.                    */
/*                                                        */
/* TIMEOUT_MS:                                            */
/* Bootloader timeout period, in milliseconds.            */
/* 500,1000,2000,4000,8000 supported.                     */
/*                                                        */
/* UART:                                                  */
/* UART number (0..n) for devices with more than          */
/* one hardware uart (644P, 1284P, etc)                   */
/*                                                        */
/**********************************************************/

/**********************************************************/
/* Version Numbers!                                       */
/*                                                        */
/* Arduino Optiboot now includes this Version number in   */
/* the source and object code.                            */
/*                                                        */
/* Version 3 was released as zip from the optiboot        */
/*  repository and was distributed with Arduino 0022.     */
/* Version 4 starts with the arduino repository commit    */
/*  that brought the arduino repository up-to-date with   */
/*  the optiboot source tree changes since v3.            */
/* Version 5 was created at the time of the new Makefile  */
/*  structure (Mar, 2013), even though no binaries changed*/
/* It would be good if versions implemented outside the   */
/*  official repository used an out-of-seqeunce version   */
/*  number (like 104.6 if based on based on 4.5) to       */
/*  prevent collisions.                                   */
/*                                                        */
/**********************************************************/

/**********************************************************/
/* Edit History:					                                */
/*							                                          */
/* Jun2015						                                    */
/* 6.2 CHH   : Added frequency Compiled for in func table */
/*              added function to set core freq from user */
/*              user code function now in a table         */
/*              blink RGB Red/Green on flash write/read   */
/* Sep2014						                                    */
/* 6.2 CHH   : Modified for ULPNode                       */
/*              added driver for WS2812                   */
/*              added Low Power function for ULPNode      */
/*              see http://hallard.me/ulp-bootloader      */
/* Aug 2014						                                    */
/* 6.2 WestfW: make size of length variables dependent    */
/*              on the SPM_PAGESIZE.  This saves space    */
/*              on the chips where it's most important.   */
/* 6.1 WestfW: Fix OPTIBOOT_CUSTOMVER (send it!)	        */
/*             Make no-wait mod less picky about	        */
/*               skipping the bootloader.		  						*/
/*             Remove some dead code			  							*/
/* Jun 2014						  																	*/
/* 6.0 WestfW: Modularize memory read/write functions	  	*/
/*             Remove serial/flash overlap		  					*/
/*              (and all references to NRWWSTART/etc)	  	*/
/*             Correctly handle pagesize > 255bytes       */
/*             Add EEPROM support in BIGBOOT (1284)       */
/*             EEPROM write on small chips now causes err */
/*             Split Makefile into smaller pieces         */
/*             Add Wicked devices Wildfire		  					*/
/*	       Move UART=n conditionals into pin_defs.h   		*/
/*	       Remove LUDICOUS_SPEED option		  							*/
/*	       Replace inline assembler for .version      		*/
/*              and add OPTIBOOT_CUSTOMVER for user code  */
/*             Fix LED value for Bobuino (Makefile)       */
/*             Make all functions explicitly inline or    */
/*              noinline, so we fit when using gcc4.8     */
/*             Change optimization options for gcc4.8	  	*/
/*             Make ENV=arduino work in 1.5.x trees.	  	*/
/* May 2014                                               */
/* 5.0 WestfW: Add support for 1Mbps UART                 */
/* Mar 2013                                               */
/* 5.0 WestfW: Major Makefile restructuring.              */
/*             See Makefile and pin_defs.h                */
/*             (no binary changes)                        */
/*                                                        */
/* 4.6 WestfW/Pito: Add ATmega32 support                  */
/* 4.6 WestfW/radoni: Don't set LED_PIN as an output if   */
/*                    not used. (LED_START_FLASHES = 0)   */
/* Jan 2013						  																	*/
/* 4.6 WestfW/dkinzer: use autoincrement lpm for read     */
/* 4.6 WestfW/dkinzer: pass reset cause to app in R2      */
/* Mar 2012                                               */
/* 4.5 WestfW: add infrastructure for non-zero UARTS.     */
/* 4.5 WestfW: fix SIGNATURE_2 for m644 (bad in avr-libc) */
/* Jan 2012:                                              */
/* 4.5 WestfW: fix NRWW value for m1284.                  */
/* 4.4 WestfW: use attribute OS_main instead of naked for */
/*             main().  This allows optimizations that we */
/*             count on, which are prohibited in naked    */
/*             functions due to PR42240.  (keeps us less  */
/*             than 512 bytes when compiler is gcc4.5     */
/*             (code from 4.3.2 remains the same.)        */
/* 4.4 WestfW and Maniacbug:  Add m1284 support.  This    */
/*             does not change the 328 binary, so the     */
/*             version number didn't change either. (?)   */
/* June 2011:                                             */
/* 4.4 WestfW: remove automatic soft_uart detect (didn't  */
/*             know what it was doing or why.)  Added a   */
/*             check of the calculated BRG value instead. */
/*             Version stays 4.4; existing binaries are   */
/*             not changed.                               */
/* 4.4 WestfW: add initialization of address to keep      */
/*             the compiler happy.  Change SC'ed targets. */
/*             Return the SW version via READ PARAM       */
/* 4.3 WestfW: catch framing errors in getch(), so that   */
/*             AVRISP works without HW kludges.           */
/*  http://code.google.com/p/arduino/issues/detail?id=368n*/
/* 4.2 WestfW: reduce code size, fix timeouts, change     */
/*             verifySpace to use WDT instead of appstart */
/* 4.1 WestfW: put version number in binary.		  				*/
/**********************************************************/

#define OPTIBOOT_MAJVER 6
#define OPTIBOOT_MINVER 2

/*
 * OPTIBOOT_CUSTOMVER should be defined (by the makefile) for custom edits
 * of optiboot.  That way you don't wind up with very different code that
 * matches the version number of a "released" optiboot.
 */

#if !defined(OPTIBOOT_CUSTOMVER)
#define OPTIBOOT_CUSTOMVER 0
#endif

unsigned const int __attribute__((section(".version"))) 
optiboot_version = 256*(OPTIBOOT_MAJVER + OPTIBOOT_CUSTOMVER) + OPTIBOOT_MINVER;

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/power.h> 
#include <avr/wdt.h>
#include <avr/sleep.h> 
#include <avr/interrupt.h> 
#include <util/delay.h>

/*
 * Note that we use our own version of "boot.h"
 * <avr/boot.h> uses sts instructions, but this version uses out instructions
 * This saves cycles and program memory.  Sorry for the name overlap.
 */
#include "boot.h"


// We don't use <avr/wdt.h> as those routines have interrupt overhead we don't need.

/*
 * pin_defs.h
 * This contains most of the rather ugly defines that implement our
 * ability to use UART=n and LED=D3, and some avr family bit name differences.
 */
#include "pin_defs.h"

/*
 * ULPNode.h
 * This contains the defines that implement our
 * ability to use ULPNode hardware such as : 
 * PWR_BOOST=D6 PWR_SENSOR=B1 PWR_RF=D7 WAKE_SWITCH=D4
*/ 
#include "ULPNode.h"
#define FREQ_MHZ ((uint8_t) (F_CPU/1000000) )


// address where we put our own bootloader reset flags 
// and other indication 
// We put all stuff in the last bytes of EEPROM
// comment the ones you don't want
#define EEP_FLAGS_MCU_RESET      (uint8_t *) E2END-0
#define EEP_FLAGS_BOOTLOADER     (uint8_t *) E2END-1
//#define EEP_FLAGS_FLASH_READ     (uint8_t *) E2END-2
#define EEP_FLAGS_FLASH_WRITE    (uint8_t *) E2END-3
//#define EEP_FLAGS_STK_LEAVE_PROG (uint8_t *) E2END-4

/*
 * stk500.h contains the constant definitions for the stk500v1 comm protocol
 */
#include "stk500.h"

#ifndef LED_START_FLASHES
#define LED_START_FLASHES 0
#endif

/* set the UART baud rate defaults */
#ifndef BAUD_RATE
#if F_CPU >= 8000000L
#define BAUD_RATE   115200L // Highest rate Avrdude win32 will support
#elsif F_CPU >= 1000000L
#define BAUD_RATE   9600L   // 19200 also supported, but with significant error
#elsif F_CPU >= 128000L
#define BAUD_RATE   4800L   // Good for 128kHz internal RC
#else
#define BAUD_RATE 1200L     // Good even at 32768Hz
#endif
#endif

#ifndef UART
#define UART 0
#endif

#define BAUD_SETTING (( (F_CPU + BAUD_RATE * 4L) / ((BAUD_RATE * 8L))) - 1 )
#define BAUD_ACTUAL (F_CPU/(8 * ((BAUD_SETTING)+1)))
#define BAUD_ERROR (( 100*(BAUD_RATE - BAUD_ACTUAL) ) / BAUD_RATE)

#if BAUD_ERROR >= 5
#error BAUD_RATE error greater than 5%
#elif BAUD_ERROR <= -5
#error BAUD_RATE error greater than -5%
#elif BAUD_ERROR >= 2
#warning BAUD_RATE error greater than 2%
#elif BAUD_ERROR <= -2
#warning BAUD_RATE error greater than -2%
#endif

#if (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 > 250
#error Unachievable baud rate (too slow) BAUD_RATE 
#endif // baud rate slow check
#if (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 < 3
#if BAUD_ERROR != 0 // permit high bitrates (ie 1Mbps@16MHz) if error is zero
#error Unachievable baud rate (too fast) BAUD_RATE 
#endif
#endif // baud rate fastn check

/* Watchdog settings */
#define WATCHDOG_OFF    (0)
#define WATCHDOG_16MS   (_BV(WDE))
#define WATCHDOG_32MS   (_BV(WDP0) | _BV(WDE))
#define WATCHDOG_64MS   (_BV(WDP1) | _BV(WDE))
#define WATCHDOG_125MS  (_BV(WDP1) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_250MS  (_BV(WDP2) | _BV(WDE))
#define WATCHDOG_500MS  (_BV(WDP2) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_1S     (_BV(WDP2) | _BV(WDP1) | _BV(WDE))
#define WATCHDOG_2S     (_BV(WDP2) | _BV(WDP1) | _BV(WDP0) | _BV(WDE))
#ifndef __AVR_ATmega8__
#define WATCHDOG_4S     (_BV(WDP3) | _BV(WDE))
#define WATCHDOG_8S     (_BV(WDP3) | _BV(WDP0) | _BV(WDE))
#endif


/*
 * We can never load flash with more than 1 page at a time, so we can save
 * some code space on parts with smaller pagesize by using a smaller int.
 */
#if SPM_PAGESIZE > 255
typedef uint16_t pagelen_t ;
#define GETLENGTH(len) len = getch()<<8; len |= getch()
#else
typedef uint8_t pagelen_t;
#define GETLENGTH(len) (void) getch() /* skip high byte */; len = getch()
#endif


/* Function Prototypes
 * The main() function is in init9, which removes the interrupt vector table
 * we don't need. It is also 'OS_main', which means the compiler does not
 * generate any entry or exit code itself (but unlike 'naked', it doesn't
 * supress some compile-time options we want.)
 */

int main(void) __attribute__ ((OS_main)) __attribute__ ((section (".init9")));

void __attribute__((noinline)) putch(char);
uint8_t __attribute__((noinline)) getch(void);
void __attribute__((noinline)) verifySpace();
void __attribute__((noinline)) watchdogConfig(uint8_t x);

// Sketch call-able function 
void __bl_showLED(uint8_t *data, uint16_t datlen);
void __bl_boosterMode(uint8_t enable);
void __bl_setCoreFrequency(uint8_t freq);

// Structure of the LED array
struct cRGB { uint8_t g; uint8_t r; uint8_t b; };

static inline void getNch(uint8_t);
static inline void watchdogReset();
static inline void writebuffer(int8_t memtype, uint8_t *mybuff,
			       uint16_t address, pagelen_t len);
static inline void read_mem(uint8_t memtype,
			    uint16_t address, pagelen_t len);

#ifdef SOFT_UART
void uartDelay() __attribute__ ((naked));
#endif
void appStart(uint8_t rstFlags) __attribute__ ((naked));

/*
 * RAMSTART should be self-explanatory.  It's bigger on parts with a
 * lot of peripheral registers.  Let 0x100 be the default
 * Note that RAMSTART (for optiboot) need not be exactly at the start of RAM.
 */
#if !defined(RAMSTART)  // newer versions of gcc avr-libc define RAMSTART
#define RAMSTART 0x100
#if defined (__AVR_ATmega644P__)
// correct for a bug in avr-libc
#undef SIGNATURE_2
#define SIGNATURE_2 0x0A
#elif defined(__AVR_ATmega1280__)
#undef RAMSTART
#define RAMSTART (0x200)
#endif
#endif

/* C zero initialises all global variables. However, that requires */
/* These definitions are NOT zero initialised, but that doesn't matter */
/* This allows us to drop the zero init code, saving us memory */
#define buff    ((uint8_t*)(RAMSTART))
#ifdef VIRTUAL_BOOT_PARTITION
#define rstVect0_sav (*(uint8_t*)(RAMSTART+SPM_PAGESIZE*2+4))
#define rstVect1_sav (*(uint8_t*)(RAMSTART+SPM_PAGESIZE*2+5))
#define wdtVect0_sav (*(uint8_t*)(RAMSTART+SPM_PAGESIZE*2+6))
#define wdtVect1_sav (*(uint8_t*)(RAMSTART+SPM_PAGESIZE*2+7))
#if FLASHEND > 8192
// AVRs with more than 8k of flash have 4-byte vectors, and use jmp.
#define rstVect0 2
#define rstVect1 3
#define wdtVect0 (WDT_vect_num*4+2)
#define wdtVect1 (WDT_vect_num*4+3)
#define appstart_vec (WDT_vect_num*2)
#else
// AVRs with up to 8k of flash have 2-byte vectors, and use rjmp.
#define rstVect0 0
#define rstVect1 1
#define wdtVect0 (WDT_vect_num*2)
#define wdtVect1 (WDT_vect_num*2+1)
#define appstart_vec (WDT_vect_num)
#endif
#else
#define appstart_vec (0)
#endif // VIRTUAL_BOOT_PARTITION
/* main program starts here */
int main(void) {
  uint8_t ch;

  /*
   * Making these local and in registers prevents the need for initializing
   * them, and also saves space because code no longer stores to memory.
   * (initializing address keeps the compiler happy, but isn't really
   *  necessary, and uses 4 bytes of flash.)
   */
  register uint16_t address = 0;
  register pagelen_t  length;

  // Used to drive RGB Led (or use this as var in case no WS2812)
  struct cRGB led;
  
  // After the zero init loop, this is the first code to run.
  //
  // This code makes the following assumptions:
  //  No interrupts will execute
  //  SP points to RAMEND
  //  r1 contains zero
  //
  // If not, uncomment the following instructions:
  // cli();
  asm volatile ("clr __zero_reg__");
#if defined(__AVR_ATmega8__) || defined (__AVR_ATmega32__)
  SP=RAMEND;  // This is done by hardware reset
#endif

  /*
   * modified Adaboot no-wait mod.
   * Pass the reset reason to app.  Also, it appears that an Uno poweron
   * can leave multiple reset flags set; we only want the bootloader to
   * run on an 'external reset only' status
   */
  ch = MCUSR;
  MCUSR = 0;

  // Clear LED values;
  led.r=led.g=led.b=0;
  
  #ifdef ULPNODE
    // don't forget we divided the clock by 8 with fuses
    // so here, we're at 2Mhz with 16MHz crystal

    // Remember that at reset state, all pins are configured as
    // input, so we don't need to set them as input when we need 
    // one configured as input
     
    // 1st thing turn off harvesting devices  

    // Disable powering Radio Module
    #ifdef PWR_RF
      // Enable Pull Up on input before setting it output high 
      // see section 14.2.3 of datasheet
      //PWR_RF_PORT |= _BV(PWR_RF); 
      
      PWR_RF_DDR  |= _BV(PWR_RF); // Set pin as output 
      PWR_RF_PORT |= _BV(PWR_RF); // Set pin to 1 (disable powering sensors)
    #endif

    // Disable powering the sensors and RGB LED
    #ifdef PWR_SENSOR
      // Enable Pull Up on input before setting it output high 
      // see section 14.2.3 of datasheet
      //PWR_SENSOR_PORT |= _BV(PWR_SENSOR); 

      PWR_SENSOR_DDR  |= _BV(PWR_SENSOR); // Set pin as output 
      PWR_SENSOR_PORT |= _BV(PWR_SENSOR); // Set pin to 1 (disable powering module)
    #endif
    
    #ifdef WAKE_SWITCH
      //WAKE_SWITCH_DDR   &= ~_BV(WAKE_SWITCH); // Set switch pin as input 
      WAKE_SWITCH_PORT  |= _BV(WAKE_SWITCH);  // Activate pull up 
    #endif

    // Enable the booster 
    __bl_boosterMode(1);
    
    // As AtMega was setup with fuses CKDIV8 to be 1.8V compatible
    // So we're running at 1Mhz or 2 MHz (8Mhz or 16Mhz resonator) 
    // But the bootloader was compiled for CPU running at 8MHz (3V3 power)
    // so for correct serial speed we set the correct CPU speed here
      
    // Enable ULPNode speed (we should have all power from USB at least 3.3V)
    // so Serial can go up to max defined at bootloader compilation time
    
    // We're running at 4MHz but we have 16MHz crystal
    // divide per 4 so compiled code for 4MHz be happy (and working!)
    #ifdef ULPNODE
      // Set ULPNode cruise speed (4MHz)
      __bl_setCoreFrequency(4);
      //clock_prescale_set(clock_div_4);
    #else
      // Set speed compiled for
      __bl_setCoreFrequency(FREQ_MHZ);
      //clock_prescale_set(clock_div_1);
    #endif

    // ULPNode have 1Mo pull down resistor, on RDX (PD0) so if
    // reading RXD pin is 1 this mean we're connected to a device, 
    // probably FTDI, so we will force entering bootloader mode 
    // because this is surely what we want to do, or at least using serial
    
    //DDRD &= ~_BV(PIND0); // Set RX pin as input (not needed already at reset)
    
    // Serial port RX is bit 0, if FTDI is connected, we will enter bootloader
    if ( PIND & _BV(PIND0) )
    {
      // prepare led green value (avoid 255, we don't need all luminosity,
      // remember we want low power so avoid high consumption)
      led.g=32; 
    }
      
    #ifdef WAKE_SWITCH
      // Switch entry is low ? switch pressed we will enter bootloader
      if ( !(WAKE_SWITCH_PIN & _BV(WAKE_SWITCH)) )
      {
        // prepare led BLUE value (avoid 255, we don't need all luminosity,
        // remember we want low power so avoid high consumption)
        led.b=32; 
      }
    #endif
    
    // no FTDI or wake switch pressed ? direct boot, no bootloader
    if ((!(ch & _BV(EXTRF))) || (led.g==0 && led.b==0)) 
      appStart(ch);
  #else
    if (!(ch & _BV(EXTRF))) 
      appStart(ch);
  #endif  

  // From now, sure we want to use the bootloader so prepare our job
  // after upload or time out, reset will be triggered by watchdog

  // Set Default ULPNode speed 
  __bl_setCoreFrequency(FREQ_MHZ);
  // Set the clock speed
  //clock_prescale_set(clock_div_2);
  
  // powering the sensors and RGB LED if wa want to blink it
  #if defined (PWR_SENSOR) && defined (WS2812) && (LED_START_FLASHES > 0)
    PWR_SENSOR_PORT &= ~_BV(PWR_SENSOR); // Set pin to 0 (enable powering module)
  #endif

  // Indicate we've entered into bootloading sequence and save
  // we need this because we go out from bootloader by watchdog
  // so it's difficult to really know in application what really
  // caused us to start/restart
  #ifdef EEP_FLAGS_BOOTLOADER
  eeprom_update_byte(EEP_FLAGS_BOOTLOADER, 0x01);
  #endif

#if LED_START_FLASHES > 0
  // Set up Timer 1 for timeout counter
  TCCR1B = _BV(CS12) | _BV(CS10); // div 1024
  
  #if defined (LED) || defined(LED_DATA_FLASH)
    // Set LED pin as output 
    LED_DDR |= _BV(LED);
  #endif
  
#endif

#ifndef SOFT_UART
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
#endif
#endif

  // Set up watchdog to trigger after 1S
  watchdogConfig(WATCHDOG_1S);

#ifdef SOFT_UART
  /* Set TX pin as output */
  UART_DDR |= _BV(UART_TX_BIT);
#endif

#if LED_START_FLASHES > 0
  // First Flash color will indicate us what we detected in first 
  // step (wake switch and/or FTDI connector)
  // green => FTDI Connected
  // blue  => Wake up switch pressed
  // cyan  => FTDI Connected And Wake up switch pressed (green+blue)
  
  #ifdef WS2812
  struct cRGB led_off;
  uint8_t * pled;
  uint8_t count = LED_START_FLASHES * 2 ;
  
  // Initialize led off values;
  led_off.r=led_off.g=led_off.b=0;
  #endif 

  do 
  {
    TCNT1 = -(F_CPU/(1024*16));
    TIFR1 = _BV(TOV1);
    while(!(TIFR1 & _BV(TOV1)));
    #ifdef LED
    #if defined(__AVR_ATmega8__)  || defined (__AVR_ATmega32__)
        LED_PORT ^= _BV(LED);
    #else
        LED_PIN |= _BV(LED);
    #endif
    #endif
    watchdogReset();
    #ifdef WS2812
    if (count%2 == 0 )
      pled = (uint8_t *) &led; 
    else
      pled = (uint8_t *) &led_off;

    // transmit values to 1 LED (3 values, R,G,B)
    __bl_showLED(pled,3);
    //_delay_us(50);
    #endif
  } 
  while (--count);
#endif

  /* Forever loop: exits by causing WDT reset */
  for (;;) 
  {
    /* get character from UART */
    ch = getch();
    
    if(ch == STK_GET_PARAMETER) {
      unsigned char which = getch();
      verifySpace();
      /*
       * Send optiboot version as "SW version"
       * Note that the references to memory are optimized away.
       */
      if (which == 0x82) {
	  putch(optiboot_version & 0xFF);
      } else if (which == 0x81) {
	  putch(optiboot_version >> 8);
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
      getNch(20);
    }
    else if(ch == STK_SET_DEVICE_EXT) {
      // SET DEVICE EXT is ignored
      getNch(5);
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
      verifySpace();
    }
    else if(ch == STK_UNIVERSAL) {
      // UNIVERSAL command is ignored
      getNch(4);
      putch(0x00);
    }
    /* Write memory, length is big endian and is in bytes */
    else if(ch == STK_PROG_PAGE) {
      // PROGRAM PAGE - we support flash programming only, not EEPROM
      
      // Writing to flash, red blink
      #if defined(WS2812) && defined(LED_DATA_FLASH)
        led.r = (uint8_t) LED_DATA_FLASH;
        led.g = led.b = 0;
        pled = (uint8_t *) &led; 

        // transmit values to 1 LED (3 values, R,G,B)
        __bl_showLED(pled,3);
      #endif
        
        
      uint8_t desttype;
      uint8_t *bufPtr;
      pagelen_t savelength;

      GETLENGTH(length);
      savelength = length;
      desttype = getch();

      // read a page worth of contents
      bufPtr = buff;
      do *bufPtr++ = getch();
      while (--length);

      // Read command terminator, start reply
      verifySpace();
      
      #ifdef VIRTUAL_BOOT_PARTITION
      #if FLASHEND > 8192
        /*
         * AVR with 4-byte ISR Vectors and "jmp"
         */
              if (address == 0) {
          // This is the reset vector page. We need to live-patch the
          // code so the bootloader runs first.
          //
          // Save jmp targets (for "Verify")
          rstVect0_sav = buff[rstVect0];
          rstVect1_sav = buff[rstVect1];
          wdtVect0_sav = buff[wdtVect0];
          wdtVect1_sav = buff[wdtVect1];

                // Move RESET jmp target to WDT vector
                buff[wdtVect0] = rstVect0_sav;
                buff[wdtVect1] = rstVect1_sav;

                // Add jump to bootloader at RESET vector
                buff[rstVect0] = ((uint16_t)main) & 0xFF;
                buff[rstVect1] = ((uint16_t)main) >> 8;
              }
      #else
        /*
         * AVR with 2-byte ISR Vectors and rjmp
         */
              if ((uint16_t)(void*)address == rstVect0) {
                // This is the reset vector page. We need to live-patch
                // the code so the bootloader runs first.


                //
                // Move RESET vector to WDT vector
          // Save jmp targets (for "Verify")
          rstVect0_sav = buff[rstVect0];
          rstVect1_sav = buff[rstVect1];
          wdtVect0_sav = buff[wdtVect0];
          wdtVect1_sav = buff[wdtVect1];

          // Instruction is a relative jump (rjmp), so recalculate.
          uint16_t vect=rstVect0_sav+(rstVect1_sav<<8);
                vect -= WDT_vect_num;
                // Move RESET jmp target to WDT vector
                buff[wdtVect0] = vect & 0xff;
                buff[wdtVect1] = vect >> 8;
                // Add rjump to bootloader at RESET vector
                buff[0] = (((uint16_t)main) & 0xFFF) & 0xFF; // rjmp 0x1d00 instruction
          buff[1] =  ((((uint16_t)main) & 0xFFF) >> 8) | 0xC0;
              }
      #endif // FLASHEND
      #endif // VBP

      writebuffer(desttype, buff, address, savelength);
      
      #if defined(WS2812) && defined(LED_DATA_FLASH)
        // led off
        pled = (uint8_t *) &led_off;
        __bl_showLED(pled,3);
      #endif
      
      // Mainly if we're there, we uploaded a sketch
      #ifdef EEP_FLAGS_FLASH_WRITE
      eeprom_update_byte(EEP_FLAGS_FLASH_WRITE, 0x03);
      #endif
      
    }
    /* Read memory block mode, length is big endian.  */
    else if(ch == STK_READ_PAGE) {
      uint8_t desttype;
      
      // reading from flash, green blink
      #if defined(WS2812) && defined(LED_DATA_FLASH)
        led.g = (uint8_t) LED_DATA_FLASH;
        led.r = led.b = 0;
        pled = (uint8_t *) &led; 

        // transmit values to 1 LED (3 values, R,G,B)
        __bl_showLED(pled,3);
      #endif
      

      GETLENGTH(length);
      desttype = getch();
      verifySpace();
	  
      read_mem(desttype, address, length);
      
      
      #if defined(WS2812) && defined(LED_DATA_FLASH)
        // led off
        pled = (uint8_t *) &led_off;
        __bl_showLED(pled,3);
      #endif
      

      #ifdef EEP_FLAGS_FLASH_READ
      eeprom_update_byte(EEP_FLAGS_FLASH_READ, 0x02);
      #endif
    }

    /* Get device signature bytes  */
    else if(ch == STK_READ_SIGN) {
      // READ SIGN - return what Avrdude wants to hear
      verifySpace();
      putch(SIGNATURE_0);
      putch(SIGNATURE_1);
      putch(SIGNATURE_2);
    
    }
    else if (ch == STK_LEAVE_PROGMODE) { /* 'Q' */
      #ifdef EEP_FLAGS_STK_LEAVE_PROG
      eeprom_update_byte(EEP_FLAGS_STK_LEAVE_PROG, 0x04);
      #endif
      // Adaboot no-wait mod
      watchdogConfig(WATCHDOG_16MS);
      verifySpace();
    }
    else {
      // This covers the response to commands like STK_ENTER_PROGMODE
      verifySpace();
    }
    putch(STK_OK);

  }
}

void putch(char ch) {
#ifndef SOFT_UART
  while (!(UART_SRA & _BV(UDRE0)));
  UART_UDR = ch;
#else
  __asm__ __volatile__ (
    "   com %[ch]\n" // ones complement, carry set
    "   sec\n"
    "1: brcc 2f\n"
    "   cbi %[uartPort],%[uartBit]\n"
    "   rjmp 3f\n"
    "2: sbi %[uartPort],%[uartBit]\n"
    "   nop\n"
    "3: rcall uartDelay\n"
    "   rcall uartDelay\n"
    "   lsr %[ch]\n"
    "   dec %[bitcnt]\n"
    "   brne 1b\n"
    :
    :
      [bitcnt] "d" (10),
      [ch] "r" (ch),
      [uartPort] "I" (_SFR_IO_ADDR(UART_PORT)),
      [uartBit] "I" (UART_TX_BIT)
    :
      "r25"
  );
#endif
}

uint8_t getch(void) {
  uint8_t ch;
  
// if WS2812B LED we replace flash LED with this one
// and don't use the onboad
#ifdef LED_DATA_FLASH
#ifndef WS2812
#if defined(__AVR_ATmega8__) || defined (__AVR_ATmega32__)
  LED_PORT ^= _BV(LED);
#else
  LED_PIN |= _BV(LED);
#endif
#endif
#endif

#ifdef SOFT_UART
    watchdogReset();
  __asm__ __volatile__ (
    "1: sbic  %[uartPin],%[uartBit]\n"  // Wait for start edge
    "   rjmp  1b\n"
    "   rcall uartDelay\n"          // Get to middle of start bit
    "2: rcall uartDelay\n"              // Wait 1 bit period
    "   rcall uartDelay\n"              // Wait 1 bit period
    "   clc\n"
    "   sbic  %[uartPin],%[uartBit]\n"
    "   sec\n"
    "   dec   %[bitCnt]\n"
    "   breq  3f\n"
    "   ror   %[ch]\n"
    "   rjmp  2b\n"
    "3:\n"
    :
      [ch] "=r" (ch)
    :
      [bitCnt] "d" (9),
      [uartPin] "I" (_SFR_IO_ADDR(UART_PIN)),
      [uartBit] "I" (UART_RX_BIT)
    :
      "r25"
);
#else
  while(!(UART_SRA & _BV(RXC0)))
    ;
  if (!(UART_SRA & _BV(FE0))) {
      /*
       * A Framing Error indicates (probably) that something is talking
       * to us at the wrong bit rate.  Assume that this is because it
       * expects to be talking to the application, and DON'T reset the
       * watchdog.  This should cause the bootloader to abort and run
       * the application "soon", if it keeps happening.  (Note that we
       * don't care that an invalid char is returned...)
       */
    watchdogReset();
  }
  
  ch = UART_UDR;
#endif


// if WS2812B LED we replace flash LED with this one
// and don't use the onboad
#ifdef LED_DATA_FLASH
#ifndef WS2812
#if defined(__AVR_ATmega8__) || defined (__AVR_ATmega32__)
  LED_PORT ^= _BV(LED);
#else
  LED_PIN |= _BV(LED);
#endif
#endif
#endif

  return ch;
}

#ifdef SOFT_UART
// AVR305 equation: #define UART_B_VALUE (((F_CPU/BAUD_RATE)-23)/6)
// Adding 3 to numerator simulates nearest rounding for more accurate baud rates
#define UART_B_VALUE (((F_CPU/BAUD_RATE)-20)/6)
#if UART_B_VALUE > 255
#error Baud rate too slow for soft UART
#endif

void uartDelay() {
  __asm__ __volatile__ (
    "ldi r25,%[count]\n"
    "1:dec r25\n"
    "brne 1b\n"
    "ret\n"
    ::[count] "M" (UART_B_VALUE)
  );
}
#endif

void getNch(uint8_t count) {
  do getch(); while (--count);
  verifySpace();
}

void verifySpace() {
  if (getch() != CRC_EOP) {
    
    watchdogConfig(WATCHDOG_16MS);    // shorten WD timeout

    while (1) // and busy-loop so that WD causes
      ;				 //  a reset and app start.
  }
  putch(STK_INSYNC);
}

// Watchdog functions. These are only safe with interrupts turned off.
void watchdogReset() {
  __asm__ __volatile__ (
    "wdr\n"
  );
}

void watchdogConfig(uint8_t x) {
  WDTCSR = _BV(WDCE) | _BV(WDE);
  WDTCSR = x;
}


/* ======================================================================
Function: __bl_setCoreFrequency
Purpose : set clock speed to a desired value in MHz
Input   : MHz value (1 for 1MHz .. 16 for 16MHz)
Output  : -
Comments: as __bl_showLED is dependent of bootlader F_CPU compiled for
          I decided to put all clock changes related into bootloader
          and keep all this stuff at the same place, this avoid changing
          clock to user before call to change WS2812B color
====================================================================== */
void __bl_setCoreFrequency(uint8_t freq)
{
  clock_div_t clock_div;
  
  // Remember ULPNode has a 16MHz crystal
  if (freq == 8)
    clock_div = clock_div_2;
  else if (freq == 4)
    clock_div = clock_div_4;
  else if (freq == 2)
    clock_div = clock_div_8;
  else if (freq == 1)
    clock_div = clock_div_16;
  else  // default or error => Full speed
    clock_div = clock_div_1;
  
  // Set the clock speed
  clock_prescale_set(clock_div);
}


// ==================================================================== WS2812 LED Code 
// Code for driving WS2812 leds array
// Grabbed from original Tim's library
// https://github.com/cpldcpu/light_ws2812

/*
  This routine writes an array of bytes with RGB values to the LED pin
  using the fast 800kHz clockless WS2811/2812 protocol.
*/

#ifdef WS2812

// Timing in ns
#define w_zeropulse   350
#define w_onepulse    900
#define w_totalperiod 1250

// Fixed cycles used by the inner loop
#define w_fixedlow    2
#define w_fixedhigh   4
#define w_fixedtotal  8   

// Insert NOPs to match the timing, if possible
#define w_zerocycles    (((F_CPU/1000)*w_zeropulse          )/1000000)
#define w_onecycles     (((F_CPU/1000)*w_onepulse    +500000)/1000000)
#define w_totalcycles   (((F_CPU/1000)*w_totalperiod +500000)/1000000)

// w1 - nops between rising edge and falling edge - low
#define w1 (w_zerocycles-w_fixedlow)
// w2   nops between fe low and fe high
#define w2 (w_onecycles-w_fixedhigh-w1)
// w3   nops to complete loop
#define w3 (w_totalcycles-w_fixedtotal-w1-w2)

#if w1>0
  #define w1_nops w1
#else
  #define w1_nops  0
#endif

// The only critical timing parameter is the minimum pulse length of the "0"
// Warn or throw error if this timing can not be met with current F_CPU settings.
#define w_lowtime ((w1_nops+w_fixedlow)*1000000)/(F_CPU/1000)
// ULPNode is 4MHz with WS2812B so avoid warning
#ifndef ULPNODE
#if w_lowtime>550
   #error "Light_ws2812: Sorry, the clock speed is too low. Did you set F_CPU correctly?"
#elif w_lowtime>450
   #warning "Light_ws2812: The timing is critical and may only work on WS2812B, not on WS2812(S)."
   #warning "Please consider a higher clockspeed, if possible"
#endif   
#endif

#if w2>0
#define w2_nops w2
#else
#define w2_nops  0
#endif

#if w3>0
#define w3_nops w3
#else
#define w3_nops  0
#endif

#define w_nop1  "nop      \n\t"
#define w_nop2  "rjmp .+0 \n\t"
#define w_nop4  w_nop2 w_nop2
#define w_nop8  w_nop4 w_nop4
#define w_nop16 w_nop8 w_nop8

/* ======================================================================
Function: __bl_showLED
Purpose : Transmit a led color array to the WS2812B LED
Input   : pointer on data (led color)
          size of data len (3 bytes for one led)
Output  : - 
Comments: !!!!!!!! MANDATORY !!!!!!!!
          booster and powering sensors should have been enabled before use
          clock speed is changed and restored after
          bootloader code is okay for that but pay attention when
          calling from sketch
====================================================================== */
void  __bl_showLED(uint8_t *data, uint16_t datlen)
{
  uint8_t curbyte,ctr,masklo,maskhi;
  uint8_t sreg_prev;
  
  // Save current clock speed 
  clock_div_t clock_prev = clock_prescale_get();
  
  // Set control pin as output
  WS2812_DDR |= _BV(WS2812); 

  masklo = ~_BV(WS2812) & WS2812_PORT;
  maskhi =  _BV(WS2812) | WS2812_PORT;
  
  // Fixed to compiled Frequency
  __bl_setCoreFrequency(FREQ_MHZ);
  
  // don't be interrupt intrusive, we'll restore current state
  sreg_prev=SREG;
  cli();  

  // loop thru all buffer
  while (datlen--) 
  {
    curbyte=*data++;
    
    asm volatile(
    "       ldi   %0,8  \n\t"
    "loop%=:            \n\t"
    "       out   %2,%3 \n\t"    //  '1' [01] '0' [01] - re
    #if (w1_nops&1)
    w_nop1
    #endif
    #if (w1_nops&2)
    w_nop2
    #endif
    #if (w1_nops&4)
    w_nop4
    #endif
    #if (w1_nops&8)
    w_nop8
    #endif
    #if (w1_nops&16)
    w_nop16
    #endif
    "       sbrs  %1,7  \n\t"    //  '1' [03] '0' [02]
    "       out   %2,%4 \n\t"    //  '1' [--] '0' [03] - fe-low
    "       lsl   %1    \n\t"    //  '1' [04] '0' [04]
    #if (w2_nops&1)
      w_nop1
    #endif
    #if (w2_nops&2)
      w_nop2
    #endif
    #if (w2_nops&4)
      w_nop4
    #endif
    #if (w2_nops&8)
      w_nop8
    #endif
    #if (w2_nops&16)
      w_nop16 
    #endif
    "       out   %2,%4 \n\t"    //  '1' [+1] '0' [+1] - fe-high
    #if (w3_nops&1)
    w_nop1
    #endif
    #if (w3_nops&2)
    w_nop2
    #endif
    #if (w3_nops&4)
    w_nop4
    #endif
    #if (w3_nops&8)
    w_nop8
    #endif
    #if (w3_nops&16)
    w_nop16
    #endif
    "       dec   %0    \n\t"    //  '1' [+2] '0' [+2]
    "       brne  loop%=\n\t"    //  '1' [+3] '0' [+4]
    :	"=&d" (ctr)
    :	"r" (curbyte), "I" (_SFR_IO_ADDR(WS2812_PORT)), "r" (maskhi), "r" (masklo)
    );
  }
  
  // Restore original IRQ state
  SREG=sreg_prev; 
  
  // Restore clock speed (in fact restore divider)
  clock_prescale_set(clock_prev);
}

#endif
// ============================================================= End Of WS2812 LED Code 


/* ======================================================================
Function: __bl_boosterMode
Purpose : enable booster and full power to 3V3 board or disable it
Input   : 1 to enable, 0 to disable
Output  : - 
Comments: here we know what're doing, this will trigger a wake IRQ that 
          we don't need, so we disable interrupts and clear associated
          IRQ flag to avoid this.
====================================================================== */
void __bl_boosterMode(uint8_t enable)
{
  #ifdef PWR_BOOST
    // save current interrupt state
    uint8_t oldSREG = SREG;

    // switch all interrupts off while messing with their settings  
    cli();
    
    // Enable Pull Up on input before setting it output high 
    // see section 14.2.3 of datasheet
    //PWR_BOOST_PORT |= _BV(PWR_BOOST); 
    
    if (enable!=0)
    {
      // Set booster pin as output 
      // Set pin LOW, this pull CD pin of NCP302 to LOW, triggering
      // reset ouput of NCP302 letting mosfet close and VCC going to enable pin of MCP1640
      // To prevent sketch misconfiguration, we always assume port config 
      // has been lost, so we configure it always from scratch
      PWR_BOOST_DDR  |= _BV(PWR_BOOST);  // Output
      PWR_BOOST_PORT &= ~_BV(PWR_BOOST); // Force Enabling booster 
    }
    else
    {
      // disable DC booster
      //PWR_BOOST_PORT &= ~_BV(PWR_BOOST); 
      
      // Set booster pin as input and disable pull up
      // CD pin of NCP302 will going HIGH, leaving it reset ouput and mosfet 
      // to be close, so enable pin of MCP1640 will go LOW thru pull down
      // To prevent sketch misconfiguration, we always assume port config 
      // has been lost, so we configure it always from scratch
      PWR_BOOST_DDR  &= ~_BV(PWR_BOOST); // Input
      PWR_BOOST_PORT &= ~_BV(PWR_BOOST); // Disable pullup
    }

    // clear any pending interrupts for Wake we triggered
    // ULPNode wake pin from booster is on INT1
    EIFR |= _BV(INT1);

    // restore interrupt previous state
    SREG = oldSREG; 
  #endif    
}

// here are our exported function mapping table (stored in section .functable)
// It start at end of bootloader 0x7ff0 just before optiboot stored version (0x7ffe)
// this will create a function table list with fixed addresses
// so we do not need to get address from .lst file at each bootloader compilation
// Add other here if you need more (space for 6 functions total)
// The 7th space is reserved where we put Frequency Bootloader was compiled for
// can be loaded to do specific action in user sketch

// I changed to a word table because depending on compiler sometimes function were 
// not placed in flash with the same order, doing this table solve the problem
uint16_t functable[7] __attribute__((section(".functable"))) = 
{
  #define FREQ_HZ (F_CPU/1000000)

  // address of __boot_showLED function
  #ifdef WS2812
    (uint16_t) &__bl_showLED,        
  #else 
    (uint16_t) 0x1111, // Dummy 1st Function
  #endif

  // address of __bl_boosterMode function
  #ifdef PWR_BOOST
    (uint16_t) &__bl_boosterMode,     
  #else 
    (uint16_t) 0x2222, // Dummy 2nd Function
  #endif
    // address of __bl_setCoreFrequency function
    (uint16_t) &__bl_setCoreFrequency,     
    (uint16_t) 0x4444, // Dummy 4th Function
    (uint16_t) 0x5555, // Dummy 5th Function
    (uint16_t) 0x6666, // Dummy 6th Function
    // F_CPU we compiled bootloader for (can be different than the Arduino Sketch, good to be able to know)
    (uint16_t) FREQ_MHZ
};


// ================================================================================================================================
void appStart(uint8_t rstFlags) {

  // Disable the booster 
  // __bl_boosterDisable();
  
  // Add our managed flags
  #ifdef EEP_FLAGS_MCU_RESET
  eeprom_update_byte(EEP_FLAGS_MCU_RESET, rstFlags);
  #endif
  
  #ifdef ULPNODE
    // be sure to set back ULPNode running at 4MHz
    __bl_setCoreFrequency(4);
    //clock_prescale_set(clock_div_4);
  #endif

  // save the reset flags in the designated register
  //  This can be saved in a main program by putting code in .init0 (which
  //  executes before normal c init code) to save R2 to a global variable.
  __asm__ __volatile__ ("mov r2, %0\n" :: "r" (rstFlags));

  // Disbale the watchdog
  watchdogConfig(WATCHDOG_OFF);

 // Start application
  __asm__ __volatile__ (
		#ifdef VIRTUAL_BOOT_PARTITION
		    // Jump to WDT vector
		    "ldi r30,4\n"
		    "clr r31\n"
		#else
		    // Jump to RST vector
		    "clr r30\n"
		    "clr r31\n"
		#endif
		    "ijmp\n"
		  );
}

/*
 * void writebuffer(memtype, buffer, address, length)
 */
static inline void writebuffer(int8_t memtype, uint8_t *mybuff,
			       uint16_t address, pagelen_t len)
{
    switch (memtype) {
    case 'E': // EEPROM
#if defined(SUPPORT_EEPROM) || defined(BIGBOOT)
        while(len--) {
	    eeprom_write_byte((uint8_t *)(address++), *mybuff++);
        }
#else
	/*
	 * On systems where EEPROM write is not supported, just busy-loop
	 * until the WDT expires, which will eventually cause an error on
	 * host system (which is what it should do.)
	 */
	while (1)
	    ; // Error: wait for WDT
#endif
	break;
    default:  // FLASH
	/*
	 * Default to writing to Flash program memory.  By making this
	 * the default rather than checking for the correct code, we save
	 * space on chips that don't support any other memory types.
	 */
	{
	    // Copy buffer into programming buffer
	    uint8_t *bufPtr = mybuff;
	    uint16_t addrPtr = (uint16_t)(void*)address;

	    /*
	     * Start the page erase and wait for it to finish.  There
	     * used to be code to do this while receiving the data over
	     * the serial link, but the performance improvement was slight,
	     * and we needed the space back.
	     */
	    __boot_page_erase_short((uint16_t)(void*)address);
	    boot_spm_busy_wait();

	    /*
	     * Copy data from the buffer into the flash write buffer.
	     */
	    do {
		uint16_t a;
		a = *bufPtr++;
		a |= (*bufPtr++) << 8;
		__boot_page_fill_short((uint16_t)(void*)addrPtr,a);
		addrPtr += 2;
	    } while (len -= 2);

	    /*
	     * Actually Write the buffer to flash (and wait for it to finish.)
	     */
	    __boot_page_write_short((uint16_t)(void*)address);
	    boot_spm_busy_wait();
#if defined(RWWSRE)
	    // Reenable read access to flash
	    boot_rww_enable();
#endif
	} // default block
	break;
    } // switch
}

static inline void read_mem(uint8_t memtype, uint16_t address, pagelen_t length)
{
    uint8_t ch;

    switch (memtype) {

#if defined(SUPPORT_EEPROM) || defined(BIGBOOT)
    case 'E': // EEPROM
	do {
	    putch(eeprom_read_byte((uint8_t *)(address++)));
	} while (--length);
	break;
#endif
    default:
	do {
#ifdef VIRTUAL_BOOT_PARTITION
        // Undo vector patch in bottom page so verify passes
	    if (address == 0)       ch=rstVect & 0xff;
	    else if (address == 1)  ch=rstVect >> 8;
	    else if (address == 8)  ch=wdtVect & 0xff;
	    else if (address == 9) ch=wdtVect >> 8;
	    else ch = pgm_read_byte_near(address);
	    address++;
#elif defined(RAMPZ)
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
	break;
    } // switch
}



