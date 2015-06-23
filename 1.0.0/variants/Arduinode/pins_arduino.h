// Pins definitions for hallard's boards

#ifndef MyPins_Arduino_h
#define MyPins_Arduino_h

#include "../standard/pins_arduino.h"

#ifdef ARDUINO_AVR_ARDUINODE_V10
	#warning ================== Arduinode V1.0 ===================
	#define BOARD_ARDUINODE_V_1_0
#endif 

#ifdef ARDUINO_AVR_ARDUINODE_V13
	#warning ================== Arduinode V1.3 ===================
	#define BOARD_ARDUINODE_V_1_3

	#define RF_IRQ					2
	#define SW_IRQ					3
	#define LED_BLU					4
	#define LED_GRN					5
	#define LED_RED					6
	#define OLED_POWER_PIN	7
	#define RF_CE						8
	#define RF_POWER_PIN		9
	#define RF_SS						10

	#define VBATT							A1
	#define SENSOR_POWER_PIN	A2
	#define SENSOR_DHT_DQ			A3
	#define SENSOR_NTC				A6
	#define SENSOR_LIGHT			A7

 	#define rfPWR_ON()				digitalWrite(RF_POWER_PIN, LOW)
  #define rfPWR_OFF()				digitalWrite(RF_POWER_PIN, HIGH)

	#define sensorPWR_ON()		digitalWrite(SENSOR_POWER_PIN, HIGH)
	#define sensorPWR_OFF()		digitalWrite(SENSOR_POWER_PIN, LOW)
	#define oledPWR_ON()			digitalWrite(OLED_POWER_PIN, LOW)
	#define oledPWR_OFF()			digitalWrite(OLED_POWER_PIN, HIGH)

#endif // MyPins_Arduino_h
// vim:ai:cin:sts=2 sw=2 ft=cpp