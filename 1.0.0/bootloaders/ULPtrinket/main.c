/* VUSBtinyBoot by me@frank-zhao.com
 *  
 * VUSBtinyBoot is a bootloader that emulates a USBtinyISP (from Adafruit Industries)
 *  
 * Trinket Pro (from Adafruit Industries) will use VUSBtinyBoot
 *
 * This code is heavily derived from USBaspLoader, but also from USBtiny, with USBtinyISP's settings
 
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

#include <avr/io.h>
#ifndef MCUSR
#define MCUSR MCUCSR // hack for enabling ATmega8 support
#endif
#define	SIGRD	5	// this is missing from some of the io.h files, this is a hack so avr/boot.h can be used
#include "avr_boot.h"
#include <avr/pgmspace.h>
//#include <avr/fuse.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
//#include <util/delay.h>
#include "pin_defs.h"
#include "optiboot.h"
#include <usbconfig.h>
#include <bootloaderconfig.h>
#include <usbdrv/usbdrv.c>	// must be included, because of static function declarations are being used, which saves flash space

// for ULPNode
#include <avr/eeprom.h>
#include <avr/power.h> 
#include <avr/sleep.h> 

// enable features here
#define ENABLE_FLASH_WRITING
#define ENABLE_FLASH_READING
#define ENABLE_EEPROM_WRITING
#define ENABLE_EEPROM_READING
//#define ENABLE_CHIP_ERASE
//#define ENABLE_CHIP_ERASE_EEPROM_FUSE_CHECK
#define ENABLE_SIG_READING
#define ENABLE_FUSE_READING
#define ENABLE_REQUEST_EXIT // note: enabling this actually decreases code size
#define ENABLE_CLEAN_EXIT // must be used with ENABLE_REQUEST_EXIT
#define ENABLE_OPTIBOOT
#define ENABLE_BLANK_CHECK

// timeout for the bootloader
#define USBBOOTLOADER_TIMEOUT 10
#define UARTBOOTLOADER_TIMEOUT 3

enum
{
	// Generic requests
	USBTINY_ECHO,		// echo test
	USBTINY_READ,		// read byte
	USBTINY_WRITE,		// write byte
	USBTINY_CLR,		// clear bit 
	USBTINY_SET,		// set bit
	// Programming requests
	USBTINY_POWERUP,	// apply power (wValue:SCK-period, wIndex:RESET)
	USBTINY_POWERDOWN,	// remove power from chip
	USBTINY_SPI,		// issue SPI command (wValue:c1c0, wIndex:c3c2)
	USBTINY_POLL_BYTES,	// set poll bytes for write (wValue:p1p2)
	USBTINY_FLASH_READ,	// read flash (wIndex:address)
	USBTINY_FLASH_WRITE,	// write flash (wIndex:address, wValue:timeout)
	USBTINY_EEPROM_READ,	// read eeprom (wIndex:address)
	USBTINY_EEPROM_WRITE,	// write eeprom (wIndex:address, wValue:timeout)
	USBTINY_DDRWRITE,		// set port direction
	USBTINY_SPI1			// a single SPI command
};

#if (FLASHEND) > 0xFFFF		// need long addressing for large flash
#	define CUR_ADDR			cur_addr.addr
#	define addr_t			uint32_t
#else
#	define CUR_ADDR			cur_addr.u16[0]
#	define addr_t			uint16_t
#endif

typedef union longConverter { // utility for manipulating address pointer with proper endianness
	addr_t		addr;
	uint16_t	u16[sizeof(addr_t)/2];
	uint8_t		u8[sizeof(addr_t)];
} longConverter_t;

#ifdef ENABLE_REQUEST_EXIT
static	uint8_t				req_boot_exit;
#endif
static	longConverter_t		cur_addr;
static	uchar				dirty = 0;			// if flash needs to be written
static	uchar				cmd0;				// current read/write command byte
static	uint8_t				remaining;			// bytes remaining in current transaction
static	uchar				buffer[8];			// talk via setup
static	uint8_t				timeout = 0;		// timeout counter for USB comm
volatile	char			usbHasRxed = 0;		// whether or not USB comm is active
#ifdef ENABLE_BLANK_CHECK
static uchar				isBlank;			// only allow exit if chip isn't blank
#endif

void (*app_start)(void) = 0x0000; // function at start of flash memory, call to exit bootloader

#ifdef ULPNODE
// Sketch call-able function 
void __bl_showLED(uint8_t *data, uint16_t datlen);
void __bl_boosterMode(uint8_t enable);

// Structure of the LED array
struct cRGB { uint8_t g; uint8_t r; uint8_t b; };

//SPI INIT
void SPI_init(void) 
{
 DDRB |= _BV(PB2) | _BV(PB3) | _BV(PB5); //OUTPUTS for SS, MOSI, SCK
 PORTB |= _BV(PB2); //set SS HIGH
  
 // Warning: if the SS pin ever becomes a LOW INPUT then SPI automatically switches to Slave, so the data direction of the SS pin MUST be kept as OUTPUT.
 SPCR |= _BV(MSTR) | _BV(SPE); //enable SPI and set SPI to MASTER mode
}

uint8_t SPI_transfer(uint8_t _data) 
{
  SPDR = _data;
  while (!(SPSR & _BV(SPIF)));
  return SPDR;
}

void RF_writeReg(uint8_t addr, uint8_t value)
{
  PORTB &= ~_BV(PB2); // set SS LOW 
  SPI_transfer(addr | 0x80);
  SPI_transfer(value);
  PORTB |= _BV(PB2); // set SS HIGH 
}

#endif

// ----------------------------------------------------------------------
// finishes a write operation if already started
// ----------------------------------------------------------------------
static void finalize_flash_if_dirty()
{
	if (dirty != 0)
	{
		#ifdef ENABLE_FLASH_WRITING
		cli();
		boot_page_write(CUR_ADDR - 2);
		sei();
		boot_spm_busy_wait();
		cli();
		boot_rww_enable();
		sei();
		#endif
		dirty = 0;
	}
}

#ifdef ENABLE_CHIP_ERASE
// ----------------------------------------------------------------------
// chip erase
// ----------------------------------------------------------------------
static void perform_chip_erase()
{
	addr_t i;

	for (i = 0; i
	#ifdef BOOTLOADER_ADDRESS
	< (addr_t)BOOTLOADER_ADDRESS;
	#else
	<= (addr_t)FLASHEND;
	#endif
	i += SPM_PAGESIZE)
	{
		boot_spm_busy_wait();
		cli();
		boot_page_erase(i);
		sei();
	}

	#ifdef ENABLE_CHIP_ERASE_EEPROM_FUSE_CHECK
	uint8_t hfuse = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
	if ((hfuse & (1 << 3)) != 0) // the bit location of EESAVE
	{
	#else
	{
	#endif
		for (i = 0; i <= E2END; i++)
		{
			eeprom_update_byte((uint8_t *)i, 0xFF);
		}
	}
}
#endif

// ----------------------------------------------------------------------
// Handle a non-standard SETUP packet.
// ----------------------------------------------------------------------
uchar	usbFunctionSetup ( uchar data[8] )
{
	uchar	req;
	usbRequest_t *rq = (void *)data;

	// reset the bootloader timeout timer
	timeout = 0;
	// indicate activity
	LED_PORT |= _BV(LED);

	// Generic requests
	req = data[1];
	if ( req == USBTINY_ECHO )
	{
		//usbMsgPtr = data;
		return 8;
	}
	/*
	else if ( req == USBTINY_SET || req == USBTINY_CLR || req == USBTINY_WRITE || req == USBTINY_DDRWRITE || req == USBTINY_POWERUP) {
		// do nothing
		return 0;
	}
	//*/
	else if ( req == USBTINY_READ) {
		// do nothing
		return 1;
	}
	else if ( req == USBTINY_POWERDOWN )
	{
		finalize_flash_if_dirty();
		#ifdef ENABLE_REQUEST_EXIT
		req_boot_exit = 1;
		#endif
		return 0;
	}
	else if ( req == USBTINY_SPI )
	{
		finalize_flash_if_dirty(); // partial page writes are not fully written unless this is called here, it must be HERE

		usbMsgPtr = (usbMsgPtr_t)buffer;

		// this tricks "usbtiny_cmd" into succeeding
		buffer[2] = data[3];

		// for the commands, refer to ATmega datasheet under "Serial Programming Instruction Set"
		// usage of avr/boot.h here is experimental

		#ifdef ENABLE_SIG_READING
		if (data[2] == 0x30 && data[3] == 0x00) {
			// read signature byte
			buffer[3] = boot_signature_byte_get(data[4] * 2);
		}
		#ifndef ENABLE_FUSE_READING
		else
		{
			buffer[3] = 0;
		}
		#endif
		#endif
		#ifdef ENABLE_FUSE_READING
		if (data[2] == 0x50 && data[3] == 0x00 && data[4] == 0x00) {
			// read LFUSE
			buffer[3] = boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
		}
		else if (data[2] == 0x58 && data[3] == 0x08 && data[4] == 0x00) {
			// read HFUSE
			buffer[3] = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
		}
		else if (data[2] == 0x50 && data[3] == 0x08 && data[4] == 0x00) {
			// read EFUSE
			buffer[3] = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
		}
		else if (data[2] == 0x58 && data[3] == 0x00 && data[4] == 0x00) {
			// read lock bits
			buffer[3] = boot_lock_fuse_bits_get(GET_LOCK_BITS);
		}
		else if (data[2] == 0x38 && data[3] == 0x00 && data[4] == 0x00) {
			// read calibration
			buffer[3] = boot_signature_byte_get(0);
		}
		#endif
		#if !defined(ENABLE_SIG_READING) && !defined(ENABLE_FUSE_READING)
		buffer[3] = 0;
		#endif

		#ifdef ENABLE_CHIP_ERASE
		if (data[2] == 0xAC && data[3] == 0x80 && data[4] == 0x00 && data[5] == 0x00) {
			perform_chip_erase();
		}
		#endif

		// all other commands are unhandled

		return 4;
	}
	else if ( req == USBTINY_SPI1 )
	{
		// I don't know what this is used for, there are no single SPI transactions in the ISP protocol
		finalize_flash_if_dirty();
		return 1;
	}
	else if ( req == USBTINY_POLL_BYTES )
	{
		finalize_flash_if_dirty();
		return 0;
	}
	CUR_ADDR = *((uint16_t*)(&data[4]));
	remaining = rq->wLength.bytes[0];
	if ( req >= USBTINY_FLASH_READ && req <= USBTINY_EEPROM_WRITE )
	{
		cmd0 = req;
		if ( cmd0 != USBTINY_FLASH_WRITE ) {
			finalize_flash_if_dirty();
		}
		return USB_NO_MSG;	// usbFunctionRead() or usbFunctionWrite() will be called to handle the data
	}

	// do nothing if nothing done
	return 0;
}

// ----------------------------------------------------------------------
// Handle an IN packet.
// ----------------------------------------------------------------------
uchar	usbFunctionRead ( uchar* data, uchar len )
{
	uchar	i;

	if(len > remaining) {
		len = remaining;
	}

	remaining -= len;

	for	( i = 0; i < len; i++ )
	{
		if (cmd0 == USBTINY_EEPROM_READ) {
			#ifdef ENABLE_EEPROM_READING
			*data = eeprom_read_byte((void *)cur_addr.u16[0]);
			#endif
		}
		else if (cmd0 == USBTINY_FLASH_READ) {
			#ifdef ENABLE_FLASH_READING
			*data = pgm_read_byte((void *)CUR_ADDR);
			#endif
		}
		data++;
		CUR_ADDR++;
	}
	return len;
}

// ----------------------------------------------------------------------
// Handle an OUT packet.
// ----------------------------------------------------------------------
uchar	usbFunctionWrite ( uchar* data, uchar len )
{
	uchar	i, isLast;

	if(len > remaining) {
		len = remaining;
	}
	remaining -= len;
	isLast = remaining == 0;

	if (cmd0 == USBTINY_EEPROM_WRITE)
	{
		#ifdef ENABLE_EEPROM_WRITING
		for	( i = 0; i < len; i++ ) {
			eeprom_write_byte((void *)(cur_addr.u16[0]++), *data++);
		}
		#endif
	}
	else if (cmd0 == USBTINY_FLASH_WRITE)
	{
		#ifdef ENABLE_FLASH_WRITING
		for ( i = 0; i < len; )
		{
			if ((cur_addr.u16[0] & (SPM_PAGESIZE - 1)) == 0) {
				// page start, erase
				cli();
				boot_page_erase(CUR_ADDR);
				sei();
				boot_spm_busy_wait();
			}

			dirty = 1;
			cli();
			boot_page_fill(CUR_ADDR, *(short *)data);
			sei();

			CUR_ADDR += 2;
			data += 2;
			i += 2;

			if ((cur_addr.u16[0] & (SPM_PAGESIZE - 1)) == 0) {
				// end of page
				finalize_flash_if_dirty();
			}
		}
		#endif
		#ifdef ENABLE_BLANK_CHECK
		isBlank = 0;
		#endif
	}

	return isLast;
}



/* ======================================================================
Function: __bl_setCoreFrequency
Purpose : set clock speed to a desired value in MHz
Input   : MHz value (1 for 1MHz .. 16 for 16MHz)
Output  : -
Comments: as __bl_showLED is dependent of bootlader F_CPU compiled for
          I decided to put all clock changes related into bootloader
====================================================================== */
void __bl_setCoreFrequency(uint8_t freq)
{
  clock_div_t clock_div;
  
/*  
  // conversion table of clock div values for clock frequencies
  // values are 1MHz, 2Mhz, 4Mhz, 8Mhz, 16MHz
   #if F_CPU == 16000000 
     #warn Bootloader compiled with 16MHz default F_CPU!
     clock_div_t clock_conv[] = { clock_div_16, clock_div_8, clock_div_4, clock_div_2, clock_div_1 };
  #elif F_CPU == 8000000 
    #warn Bootloader compiled with 8MHz default F_CPU! setting >8MHz will stay at 8Mhz
     clock_div_t clock_conv[] = { clock_div_8, clock_div_4, clock_div_2, clock_div_1, clock_div_1 };
  #elif F_CPU == 4000000 
    #warn Bootloader compiled with 4MHz default F_CPU! setting >4MHz will stay at 4Mhz
     clock_div_t clock_conv[] = { clock_div_4, clock_div_2, clock_div_1, clock_div_1, clock_div_1 };
  #else
    #error Bad CPU Frequency!
  #endif
*/
  
  
   #if F_CPU == 16000000 
    #warning Bootloader compiled with 16MHz default F_CPU!
    if (freq == 8)
      clock_div = clock_div_2;
    else if (freq == 4)
      clock_div = clock_div_4;
    else if (freq == 2)
      clock_div = clock_div_8;
    else if (freq == 1)
      clock_div = clock_div_16;
  #elif F_CPU == 8000000 
    #warning Bootloader compiled with 8MHz default F_CPU!
    else if (freq == 4)
      clock_div = clock_div_2;
    else if (freq == 2)
      clock_div = clock_div_4;
    else if (freq == 1)
      clock_div = clock_div_8;
  #elif F_CPU == 4000000 
    #warning Bootloader compiled with 4MHz default F_CPU!
    if (freq == 2)
      clock_div = clock_div_2;
    else if (freq == 1)
      clock_div = clock_div_4;
  #else
    #error Bad CPU Frequency!
  #endif
    else // default or error full speed
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
  clock_div_t clock_prev;
  
  // Set control pin as output
  WS2812_DDR |= _BV(WS2812); 

  masklo = ~_BV(WS2812) & WS2812_PORT;
  maskhi =  _BV(WS2812) | WS2812_PORT;
  
  // Save current clock speed 
  clock_prev = clock_prescale_get();
  
  #define FREQ_HZ F_CPU/1000000
  // Set clock speed to what're compiled for
  __bl_setCoreFrequency((uint8_t) FREQ_HZ);

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
// Add other here if you create more (space for 7 functions total)
// The 7th space is reserved where we put Frequency Bootloader was compiled for
uint16_t functable[7] __attribute__((section(".functable"))) = 
{
  
#define FREQ_HZ (F_CPU/1000000)

// address of __boot_showLED function
#ifdef WS2812
  (uint16_t) &__bl_showLED,        
#else 
  (uint16_t) 0x1111,
#endif

// address of __bl_boosterMode function
#ifdef PWR_BOOST
  (uint16_t) &__bl_boosterMode,     
#else 
  (uint16_t) 0x2222,
#endif
  // address of __bl_setCoreFrequency function
  (uint16_t) &__bl_setCoreFrequency,     
  (uint16_t) 0x4444,
  (uint16_t) 0x5555,
  (uint16_t) 0x6666,
  // F_CPU we compiled bootloader for (can be different than the Arduino Sketch, good to be able to know)
  (uint16_t) FREQ_HZ
};



// ----------------------------------------------------------------------
// Bootloader main entry point
// ----------------------------------------------------------------------
int	main ( void )
{
  struct cRGB led;
  
	// disable watchdog if previously enabled
	MCUSR &= ~(1 << WDRF);
	wdt_disable();
  
  // Clear LED values;
  led.r=led.g=led.b=0;

	#ifdef ENABLE_BLANK_CHECK
	if (pgm_read_word(0) == 0xFFFF) {
		isBlank = 1;
	}
	else {
		isBlank = 0;
	}
	#endif

	MCUCR = (1 << IVCE);	// enable change of interrupt vectors
	MCUCR = (1 << IVSEL);	// move interrupts to boot flash section
  
  
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
    PWR_RF_PORT |= _BV(PWR_RF); // Set pin to 1 (disable powering RF Module)
    //PWR_RF_PORT &= ~_BV(PWR_RF); // Set pin to 0 (enable powering RF Module)
    
    //DDRB |= _BV(PB2) | _BV(PB3) | _BV(PB5); //OUTPUTS for SS, MOSI, SCK
    //DDRB |= _BV(PB2) ; //OUTPUTS for SS
    DDRB &= ~_BV(PB2) ; //INPUT for SS
    //PORTB |= _BV(PB2); //set SS HIGH

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

  // Set full speed (16MHz)
  __bl_setCoreFrequency(16);

  
  // ULPNode have 1Mo pull down resistor, on RDX (PD0) so if
  // reading RXD pin is 1 this mean we're connected to a device, 
  // probably FTDI, so we will force entering bootloader mode 
  // because this is surely what we want to do, or at least using serial
  
  //DDRD &= ~_BV(PIND0); // Set RX pin as input (not needed already at reset)
  
  // Serial port RX is bit 0, if FTDI is connected, we will enter bootloader
  if ( PIND & _BV(PIND0) )
  {
    // prepare led RED value (avoid 255, we don't need all luminosity,
    // remember we want low power so avoid high consumption)
    led.r=32; 
  }
    
  // Clear out custom boot indicator flag
  #ifdef EEPROM_FLAGS
  eeprom_write_byte(EEPROM_FLAGS, 0);
  #endif

  // From now, sure we want to use the bootloader so prepare our job
  // after upload or time out, reset will be triggered by watchdog

  // powering the sensors and RGB LED if we want to blink it
  #if defined (PWR_SENSOR) && defined (WS2812) && (LED_START_FLASHES > 0)
    PWR_SENSOR_PORT &= ~_BV(PWR_SENSOR); // Set pin to 0 (enable powering module)
  #endif

  // Indicate we've entered into bootloading sequence
  #ifdef EEPROM_FLAGS
  eeprom_write_byte(EEPROM_FLAGS, EEP_FLAGS_BOOTLOADER);
  #endif
  
#endif  
  

	// start 16-bit timer1 for slow counting time
	TCCR1B = 0x05;

	LED_DDR |= _BV(LED); // LED pin on Trinket Pro
	LED_PORT |= _BV(LED);
  
  led.b=32;

  // transmit values to 1 LED (3 values, R,G,B)
  __bl_showLED((uint8_t *) &led,3);

	// start USB and force a re-enumeration by faking a disconnect
	usbInit();
	usbDeviceDisconnect();
	LED_PORT |= _BV(LED);
	while (TCNT1 < 4000);
	LED_PORT &= ~_BV(LED);
	usbDeviceConnect();
	sei();

  led.r=0;led.g=0;led.b=0;
  __bl_showLED((uint8_t *) &led,3);

	TCCR1B = 0x01; // speed up timer for PWM LED pulsing
	uint8_t t1ovf = 0;
	uint16_t duty = 0;
	char dutyDir = 0;

	#ifdef ENABLE_OPTIBOOT
	optiboot_init();
	#endif

/*  
  SPI_init() ;
  RF_writeReg(0x01, 0x84); // Operation Mode = Sequencer ON / Listen OFF / Standy Mode
  RF_writeReg(0x25, 0x00); // default DIO Mapping
  RF_writeReg(0x01, 0x80); // Operation Mode = Sequencer ON / Listen OFF / Sleep Mode
*/  

	// main program loop
	while (1)
	{
		usbPoll();

		#ifdef ENABLE_OPTIBOOT
		char ob = optibootPoll();
		if (ob == 1) {
			timeout = 0;
		}
		else if (ob == 2
		#ifdef ENABLE_BLANK_CHECK
		&& isBlank == 0
		#endif
		) {
			break;
		}
		#endif

		if ( ((usbHasRxed != 0) && (timeout > USBBOOTLOADER_TIMEOUT)
		#ifdef ENABLE_BLANK_CHECK
		&& isBlank == 0
		#endif
		    ) || ((usbHasRxed == 0) && (timeout > UARTBOOTLOADER_TIMEOUT)
		#ifdef ENABLE_BLANK_CHECK
		&& isBlank == 0
		#endif
			  )
		#ifdef ENABLE_REQUEST_EXIT
		|| req_boot_exit != 0
		#endif
		
		     )    {
			// requested exit
			// or timed out waiting for activity (timeout means not connected to computer)
			break;
		}

		uint16_t t = TCNT1;
		if ((TIFR1 & _BV(TOV1)) != 0) // if timer has overflowed
		{
			TIFR1 |= _BV(TOV1); // clear the flag
			if (usbHasRxed != 0)
			{
				LED_PORT |= _BV(LED);
				if (duty == 0) {
					dutyDir = dutyDir ? 0 : 1;
				}

				#define WAITING_LED_FADE_RATE 512 // must be power of 2
				if (dutyDir == 0) {
					duty -= WAITING_LED_FADE_RATE;
				}
				else {
					duty += WAITING_LED_FADE_RATE;
				}
			}
			t1ovf++;

			// roughly 1 second
			#if (F_CPU == 12000000)
			if (t1ovf > 183) {
			#elif (F_CPU == 16000000)
			if (t1ovf > 244) {
			#endif
				t1ovf = 0;
				timeout++;
			}
		}

		if (usbHasRxed != 0)
		{
			// fade the LED
			if (t > duty) {
				LED_PORT &= ~_BV(LED);
			}
		}
		else
		{
			#define WAITING_LED_BLINK_RATE 16
			if  ((t1ovf > (WAITING_LED_BLINK_RATE * 0)
			  &&  t1ovf < (WAITING_LED_BLINK_RATE * 1))
			  || (t1ovf > (WAITING_LED_BLINK_RATE * 2)
			  &&  t1ovf < (WAITING_LED_BLINK_RATE * 3))
			  #if (F_CPU == 16000000)
			  || (t1ovf > (WAITING_LED_BLINK_RATE * 4)
			  &&  t1ovf < (WAITING_LED_BLINK_RATE * 5))
			  #endif
			  ) {
			  LED_PORT |= _BV(LED);
			}
			else {
			  LED_PORT &= ~_BV(LED);
			}
		}
	}

	// turn off and return port to normal
	LED_PORT &= ~_BV(LED);
	LED_DDR  &= ~_BV(LED);

	#if defined(ENABLE_REQUEST_EXIT) && defined(ENABLE_CLEAN_EXIT)
	// wait to finish all USB comms, avoids "avrdude: error: usbtiny_transmit: usb_control_msg: sending control message failed"
	TCCR1B = 0x05; // slow down timer
	TCNT1 = 0;
	while (req_boot_exit != 0 && TCNT1 < 4000) usbPoll();
	#endif

	// cleanup!

	// reset timer
	TCCR1B = 0;
	TCNT1 = 0;
  
  
  // Disable powering Radio Module
  #ifdef PWR_RF
    PWR_RF_PORT |= _BV(PWR_RF); // Set pin to 1 (disable powering RF Module)
  #endif

	// deinitialize USB
	USB_INTR_ENABLE = 0;
	USB_INTR_CFG = 0;

	// move interrupt back
	MCUCR = (1 << IVCE);	// enable change of interrupt vectors
	MCUCR = (0 << IVSEL);	// move interrupts to app flash section

	cli();// disable interrupts

	app_start(); // jump to user app

	return 0;
}
