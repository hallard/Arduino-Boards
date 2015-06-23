@ECHO OFF
REM ====================================================
REM = windows Batch file to compile ULPNode bootloader =
REM = see http://hallard.me/ulp-bootloader             =
REM =																								   =
REM = V1.0 Charles-Henri Hallard http://hallard.me     =
REM =																								   =
REM = Atmega329 pin mapping to Arduino pin name        =
REM = as a reminder                                    =
REM ====================================================
REM AT -> Arduino
REM C5 -> A5
REM C4 -> A4
REM C3 -> A3
REM C2 -> A2
REM C1 -> A1
REM C0 -> A0
REM B5 -> D13
REM B4 -> D12
REM B3 -> D11
REM B2 -> D10
REM B1 -> D9
REM B0 -> D8
REM D7 -> D7
REM D6 -> D6
REM D5 -> D5
REM D3 -> D3
REM D2 -> D2

REM Define build environement 
REM Set this value for old Arduino environement version < 1.5
REM SET BUILDENV=..\..\..\tools\avr\utils\bin\

REM Set this value for new Arduino environement version >= 1.5 (latest)
REM Arduino >= 1.5
SET BUILDENV=..\..\..\..\tools\avr\utils\bin\

REM ==================================
REM = Set default values for ULPNode =
REM ==================================
REM Arduino classic LED pin on ULPNode Arduino D5
SET LED=LED=D5

REM WS2812B Data In avr pin Arduino D6
SET WS2812=WS2812=D6

REM Number of flash when entering bootloader
SET LED_START_FLASHES=LED_START_FLASHES=3

REM flash when uploading, value of LED_DATA_FLASH 
REM is RGB Brigthness from (0 off to 255 full bright)
SET LED_DATA_FLASH=LED_DATA_FLASH=64

REM Wake up switch avr pin Arduino D4
SET WAKE_SWITCH=WAKE_SWITCH=D4

REM power sensor avr pin Arduino D9
SET PWR_SENSOR=PWR_SENSOR=B1

REM power RF Radio module avr pin Arduino D7
SET PWR_RF=PWR_RF=D7

REM booster enable avr pin Arduino A2
SET PWR_BOOST=PWR_BOOST=C2

REM this will change optiboot version from 6.2 to 5.2 (6-1) (non existing version)
SET OPTIBOOT_CUSTOMVER=OPTIBOOT_CUSTOMVER=-1

REM this will set target file name prefix
SET ULPN_BOARD=ULPNode

REM =====================================
REM =      ULPNode @8MHz / 250Kbps      =
REM =====================================
SET AVR_FREQ=AVR_FREQ=8000000
SET BAUD_RATE=BAUD_RATE=250000
@ECHO ON
%BUILDENV%make OS=windows ENV=arduino %AVR_FREQ% %BAUD_RATE% %LED% %LED_START_FLASHES% %LED_DATA_FLASH% %WS2812% %WAKE_SWITCH% %PWR_SENSOR% %PWR_BOOST% %PWR_RF% %OPTIBOOT_CUSTOMVER% ulpnode %*
@ECHO OFF

REM =====================================
REM =     ULPNode @16MHz / 250Kbps      =
REM =====================================
SET AVR_FREQ=
SET AVR_FREQ=AVR_FREQ=16000000
REM all other as above
@ECHO ON
%BUILDENV%make OS=windows ENV=arduino %AVR_FREQ% %BAUD_RATE% %LED% %LED_START_FLASHES% %LED_DATA_FLASH% %WS2812% %WAKE_SWITCH% %PWR_SENSOR% %PWR_BOOST% %PWR_RF% %OPTIBOOT_CUSTOMVER% ulpnode %*
@ECHO OFF

REM =====================================
REM =     ULPNode @16MHz / 115Kbps      =
REM =        Act as classic UNO         =
REM =====================================
SET BAUD_RATE=
SET BAUD_RATE=BAUD_RATE=115200
REM all other as above
@ECHO ON
%BUILDENV%make OS=windows ENV=arduino %AVR_FREQ% %BAUD_RATE% %LED% %LED_START_FLASHES% %LED_DATA_FLASH% %WS2812% %WAKE_SWITCH% %PWR_SENSOR% %PWR_BOOST% %PWR_RF% %OPTIBOOT_CUSTOMVER% ulpnode %*
@ECHO OFF

REM Reset env variable
SET AVR_FREQ=
SET BAUD_RATE=
SET LED=
SET WS2812=
SET LED_START_FLASHES=
SET WAKE_SWITCH=
SET PWR_SENSOR=
SET PWR_RF=
SET PWR_BOOST=
SET OPTIBOOT_CUSTOMVER
SET ULPN_BOARD=


