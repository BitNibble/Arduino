/********************************************************************
	ATMEGA 32U4 MAPPING
Author: Sergio Manuel Santos
	<sergio.salazar.santos@gmail.com>
License: GNU General Public License
Hardware: Atmega32U4 by ETT ET-BASE
Date: 12052023
Comment: 
	Virtual Image Atmega 32U4 mapping.
*********************************************************************/
#ifndef _ATMEGA32U4MAPPING_H_
	#define _ATMEGA32U4MAPPING_H_

/*** Global Library ***/
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <inttypes.h>
// Comment out modules not being used
// RAW IMAGE
#include "atmega32U4.h"
// MODULES
//#include "atmega32U4analog.h"
//#include "atmegaeeprom.h"
//#include "atmega32U4interrupt.h"
//#include "atmega32U4timer.h"
//#include "atmega32U4twi.h"
//#include "atmega32U4spi.h"
//#include "atmega32U4uart.h"

/*** Global Variable ***/
//		PARAMETER
typedef struct {
	HighLowByte HLbyte;
} Atmega32U4Parameter;
//		SECOND LAYER
// GPWR
typedef struct {
	Atmega32U4GPWR_TypeDef* reg;
} Atmega32U4GPWR;

// Analog Comparator (AC)
typedef struct {
	Atmega32U4AnalogComparator_TypeDef* reg;
} Atmega32U4AnalogComparator;

// Analog to Digital Converter (ADC)
typedef struct {
	Atmega32U4AnalogToDigitalConverter_TypeDef* reg;
	#if defined(_ATMEGA32U4ANALOG_H_)
		ANALOG (*enable)( uint8_t Vreff, uint8_t Divfactor, int n_channel, ... );
	#endif
} Atmega32U4AnalogToDigitalConverter;

// Bootloader (BOOT_LOAD)
typedef struct {
	Atmega32U4Bootloader_TypeDef* reg;
} Atmega32U4Bootloader;

// CPU Register (CPU)
typedef struct {
	Atmega32U4CPURegister_TypeDef* reg;
	Atmega32U4ClockSelect_TypeDef* clk;
} Atmega32U4CPURegister;

// EEPROM (EEPROM)
typedef struct {
	Atmega32U4Eeprom_TypeDef* reg;
	#if defined(_ATMEGAEEPROM_H_)
		EEPROM (*enable)(void);
	#endif
} Atmega32U4Eeprom;

// External Interrupts (EXINT)
typedef struct {
	Atmega32U4ExternalInterrupts_TypeDef* reg;
	#if defined(_ATMEGA32U4INTERRUPT_H_)
		INTERRUPT (*enable)(void);
	#endif
} Atmega32U4ExternalInterrupts;

// I/O Port (PORTB)
typedef struct {
	Atmega32U4PORTB_TypeDef* reg;
} Atmega32U4PORTB;

// I/O Port (PORTC)
typedef struct {
	Atmega32U4PORTC_TypeDef* reg;
} Atmega32U4PORTC;

// I/O Port (PORTD)
typedef struct {
	Atmega32U4PORTD_TypeDef* reg;
} Atmega32U4PORTD;

// I/O Port (PORTE)
typedef struct {
	Atmega32U4PORTE_TypeDef* reg;
} Atmega32U4PORTE;

// I/O Port (PORTF)
typedef struct {
	Atmega32U4PORTF_TypeDef* reg;
} Atmega32U4PORTF;

// JTAG Interface (JTAG)
typedef struct {
	Atmega32U4JtagInterface_TypeDef* reg;
} Atmega32U4JtagInterface;

// Phase Locked Loop (PLL)
typedef struct {
	Atmega32U4PhaseLockedLoop_TypeDef* reg;
} Atmega32U4PhaseLockedLoop;

// Serial Peripheral Interface (SPI)
typedef struct {
	Atmega32U4SerialPeripherialInterface_TypeDef* reg;
	#if defined(_ATMEGA32U4SPI_H_)
		SPI (*enable)(uint8_t master_slave_select, uint8_t data_order,  uint8_t data_modes, uint8_t prescaler);
	#endif
} Atmega32U4SerialPeripherialInterface;

// Timer/Counter, 10-bit (TC4)
typedef struct {
	Atmega32U4TimerCounter4_TypeDef* reg;
	Atmega32U4CompareRegister4_TypeDef* comp;
	Atmega32U4TimerMask_TypeDef* mask;
	Atmega32U4TimerMask_TypeDef* flag;
	#if defined(_ATMEGA32U4TIMER_H_)
	TIMER_COUNTER4 (*enable)(unsigned char wavegenmode, unsigned char interrupt);
	#endif
} Atmega32U4TimerCounter4;

// Timer/Counter, 16-bit (TC1)
typedef struct {
	Atmega32U4TimerCounter1_TypeDef* reg;
	Atmega32U4CompareRegister1_TypeDef* comp;
	Atmega32U4TimerMask_TypeDef* mask;
	Atmega32U4TimerMask_TypeDef* flag;
	#if defined(_ATMEGA32U4TIMER_H_)
		TIMER_COUNTER1 (*enable)(unsigned char wavegenmode, unsigned char interrupt);
	#endif
} Atmega32U4TimerCounter1;

// Timer/Counter, 16-bit (TC3)
typedef struct {
	Atmega32U4TimerCounter3_TypeDef* reg;
	Atmega32U4CompareRegister3_TypeDef* comp;
	Atmega32U4TimerMask_TypeDef* mask;
	Atmega32U4TimerMask_TypeDef* flag;
	#if defined(_ATMEGA32U4TIMER_H_)
		TIMER_COUNTER3 (*enable)(unsigned char wavegenmode, unsigned char interrupt);
	#endif
} Atmega32U4TimerCounter3;

// Timer/Counter, 8-bit (TC0)
typedef struct {
	Atmega32U4TimerCounter0_TypeDef* reg;
	Atmega32U4CompareRegister0_TypeDef* comp;
	Atmega32U4TimerMask_TypeDef* mask;
	Atmega32U4TimerMask_TypeDef* flag;
	#if defined(_ATMEGA32U4TIMER_H_)
		TIMER_COUNTER0 (*enable)(unsigned char wavegenmode, unsigned char interrupt);
	#endif
} Atmega32U4TimerCounter0;

// Two Wire Serial Interface (TWI)
typedef struct {
	Atmega32U4TwoWireSerialInterface_TypeDef* reg;
	#if defined(_ATMEGA32U4TWI_H_)
		TWI (*enable)(uint8_t atmega_ID, uint8_t prescaler);
	#endif
} Atmega32U4TwoWireSerialInterface;

// USART (USART1)
typedef struct {
	Atmega32U4Usart1_TypeDef* reg;
	#if defined(_ATMEGA32U4UART_H_)
		USART1 (*enable)(unsigned int baudrate, unsigned int FDbits, unsigned int Stopbits, unsigned int Parity );
	#endif
} Atmega32U4Usart1;

// USB Device Register (USB_DEVICE)
typedef struct {
	Atmega32U4UsbDeviceRegister_TypeDef* reg;
} Atmega32U4UsbDeviceRegister;

// Watchdog Timer (WDT)
typedef struct {
	Atmega32U4WatchdogTimer_TypeDef* reg;
} Atmega32U4WatchdogTimer;


// ATMEGA 32U4 IMAGE
typedef struct {
	//		Parameter
	Atmega32U4Parameter par;
	//		Second Layer
	Atmega32U4GPWR gpwr;
	Atmega32U4AnalogComparator ac;
	Atmega32U4AnalogToDigitalConverter adc;
	Atmega32U4Bootloader boot_load;
	Atmega32U4CPURegister cpu;
	Atmega32U4Eeprom eeprom;
	Atmega32U4ExternalInterrupts exint;
	Atmega32U4PORTB portb;
	Atmega32U4PORTC portc;
	Atmega32U4PORTD portd;
	Atmega32U4PORTE porte;
	Atmega32U4PORTF portf;
	Atmega32U4JtagInterface jtag;
	Atmega32U4PhaseLockedLoop pll;
	Atmega32U4SerialPeripherialInterface spi;
	Atmega32U4TimerCounter4 tc4;
	Atmega32U4TimerCounter1 tc1;
	Atmega32U4TimerCounter3 tc3;
	Atmega32U4TimerCounter0 tc0;
	Atmega32U4TwoWireSerialInterface twi;
	Atmega32U4Usart1 usart1;
	Atmega32U4UsbDeviceRegister usb_device;
	Atmega32U4WatchdogTimer wdt;
	//		Function Pointer
	uint16_t (*readhlbyte)(HighLowByte reg);
	uint16_t (*readlhbyte)(HighLowByte reg);
	HighLowByte (*writehlbyte)(uint16_t val);
	HighLowByte (*writelhbyte)(uint16_t val);
	uint16_t (*swapbyte)(uint16_t num);
}ATMEGA32U4;

/*** Global Header ***/
ATMEGA32U4 ATMEGA32U4enable(void);

#endif

/***EOF***/

