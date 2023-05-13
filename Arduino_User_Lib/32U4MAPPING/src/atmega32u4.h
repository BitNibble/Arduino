/************************************************************************
	ATMEGA 32U4 REGISTERS
Author: Sergio Manuel Santos
	<sergio.salazar.santos@gmail.com>
License: GNU General Public License
Hardware: Atmega32U4 by ETT ET-BASE
Date: 12052023
Comment: 
	Virtual Image Atmega 32U4.
************************************************************************/
#ifndef _ATMEGA32U4_H_
	#define _ATMEGA32U4_H_

/*** Global Library ***/
#include <inttypes.h>

/*** Global Constant & Macro ***/
#define Atmega32U4GPWR_Address 0x0000
#define Atmega32U4AnalogComparator_Address 0x0050
#define Atmega32U4AnalogToDigitalConverter_Address 0x0078
#define Atmega32U4Bootloader_Address 0x0057
#define Atmega32U4CPURegister_Address 0x003E
#define Atmega32U4Eeprom_Address 0x003F
#define Atmega32U4ExternalInterrupts_Address 0x003B
#define Atmega32U4PORTB_Address 0x0023
#define Atmega32U4PORTC_Address 0x0026
#define Atmega32U4PORTD_Address 0x0029
#define Atmega32U4PORTE_Address 0x002C
#define Atmega32U4PORTF_Address 0x002F
#define Atmega32U4JtagInterface_Address 0x0051
#define Atmega32U4PhaseLockedLoop_Address 0x0049
#define Atmega32U4SerialPeripherialInterface_Address 0x004C
#define Atmega32U4TimerCounter4_Address 0x00BE
#define Atmega32U4TimerCounter1_Address 0x0080
#define Atmega32U4TimerCounter3_Address 0x0090
#define Atmega32U4TimerCounter0_Address 0x0043
#define Atmega32U4TwoWireSerialInterface_Address 0x00B8
#define Atmega32U4Usart1_Address 0x00C8
#define Atmega32U4UsbDeviceRegister_Address 0x00D7
#define Atmega32U4WatchdogTimer_Address 0x0060
// AUXILIAR
#define Atmega32U4TimerInterruptFlag_Address 0x0035
#define Atmega32U4TimerMask_Address 0x006E;
#define Atmega32U4CompareRegister0_Address 0x0047
#define Atmega32U4CompareRegister1_Address 0x0086
#define Atmega32U4CompareRegister3_Address 0x0096
#define Atmega32U4CompareRegister4_Address 0x00CF
#define Atmega32U4ClockSelect_Address 0x00C5

/*** Global Variable ***/
// HLbyte
typedef struct {
	uint8_t L; // Lower address
	uint8_t H; // Higher address
} HighLowByte;

//		MAIN HARDWARE LAYER
/************* Auxiliary *************/
// timer
typedef struct {
	uint8_t tifr0; // 0x35
	uint8_t tifr1; // 0x36
	uint8_t fill; // 0x37
	uint8_t tifr3; // 0x38
	uint8_t tifr4; // 0x39
} Atmega32U4TimerInterruptFlag_TypeDef;

// timer
typedef struct {
	uint8_t timsk0; // 0x6E
	uint8_t timsk1; // 0x6F
	uint8_t fill; // 0x70
	uint8_t timsk2; // 0x71
	uint8_t timsk3; // 0x72
} Atmega32U4TimerMask_TypeDef;

// timer 0
typedef struct {
	uint8_t ocr0a; // 0x47
	uint8_t ocr0b; // 0x48
} Atmega32U4CompareRegister0_TypeDef;

// timer 1
typedef struct {
	HighLowByte icr1; // 0x86 0x87
	HighLowByte ocr1a; // 0x88 0x89
	HighLowByte ocr1b; // 0x8A 0x8B
	HighLowByte ocr1c; // 0x8C 0x8D
} Atmega32U4CompareRegister1_TypeDef;

// timer 3
typedef struct {
	HighLowByte icr3; // 0x96 0x97
	HighLowByte ocr3a; // 0x98 0x99
	HighLowByte ocr3b; // 0x9A 0x9B
	HighLowByte ocr3c; // 0x9C 0x9D
} Atmega32U4CompareRegister3_TypeDef;

// timer 4
typedef struct {
	uint8_t ocr4a; // 0xCF
	uint8_t ocr4b; // 0xD0
	uint8_t ocr4c; // 0xD1
	uint8_t ocr4d; // 0xD2
	uint8_t fill; // (0xD4 - 0xD2) - 1
	uint8_t dt4; // 0xD4
} Atmega32U4CompareRegister4_TypeDef;

// cpu
typedef struct {
	uint8_t clksel0; // 0xC5
	uint8_t clksel1; // 0xC6
	uint8_t clksta; // 0xC7
} Atmega32U4ClockSelect_TypeDef;
/*************************************/

// GPWR
typedef struct {
	uint8_t r0;
	uint8_t r1;
	uint8_t r2;
	uint8_t r3;
	uint8_t r4;
	uint8_t r5;
	uint8_t r6;
	uint8_t r7;
	uint8_t r8;
	uint8_t r9;
	uint8_t r10;
	uint8_t r11;
	uint8_t r12;
	uint8_t r13;
	uint8_t r14;
	uint8_t r15;
	uint8_t r16;
	uint8_t r17;
	uint8_t r18;
	uint8_t r19;
	uint8_t r20;
	uint8_t r21;
	uint8_t r22;
	uint8_t r23;
	uint8_t r24;
	uint8_t r25;
	HighLowByte X;
	HighLowByte Y;
	HighLowByte Z;
} Atmega32U4GPWR_TypeDef;

// Analog Comparator (AC)
typedef struct {
	uint8_t acsr; // 0x50
	uint8_t fill1[42]; // (0x7B - 0x50) - 1
	uint8_t adcsrb; // 0x7B
	uint8_t fill2[3]; // (0x7F - 0x7B) - 1
	uint8_t didr1; // 0x7F
} Atmega32U4AnalogComparator_TypeDef;

// Analog to Digital Converter (ADC)
typedef struct {
	HighLowByte adc; // 0x78 0x79
	uint8_t adcsra; // 0x7A
	uint8_t adcsrb; // 0x7B
	uint8_t admux; // 0x7C
	uint8_t didr2; // 0x7D
	uint8_t didr0; // 0x7E
} Atmega32U4AnalogToDigitalConverter_TypeDef;

// Bootloader (BOOT_LOAD)
typedef struct {
	HighLowByte spmcsr; // 0x57 
} Atmega32U4Bootloader_TypeDef;

// CPU Register (CPU)
typedef struct {
	uint8_t gpior0; // 0x3E
	uint8_t fill1[11]; // (0x4A - 0x3E) - 1
	uint8_t gpior1; // 0x4A
	uint8_t gpior2; // 0x4B
	uint8_t fill2[7]; // (0x53 0x4B) - 1
	uint8_t smcr; // 0x53
	uint8_t mcusr; // 0x54
	uint8_t mcucr; // 0x55
	uint8_t fill3[5]; // (0x5B - 0x55) - 1
	uint8_t rampz; // 0x5B
	uint8_t eind; // 0x5C
	HighLowByte sp; // 0x5D 0x5E
	uint8_t sreg; // 0x5F
	uint8_t fill4; // (0x61 - 0x5F) - 1
	uint8_t clkpr; // 0x61
	uint8_t fill5[2]; // (0x64 - 0x61) - 1
	uint8_t prr0; // 0x64
	uint8_t prr1; // 0x65
	uint8_t osccal; // 0x66
	uint8_t rcctrl; // 0x67
} Atmega32U4CPURegister_TypeDef;

// EEPROM (EEPROM)
typedef struct {
	uint8_t eecr; // 0x3F
	uint8_t eedr; // 0x40
	uint16_t eear; // 0x41
} Atmega32U4Eeprom_TypeDef;

// External Interrupts (EXINT)
typedef struct {
	uint8_t pcifr; // 0x3B
	uint8_t eifr; // 0x3C
	uint8_t eimsk; // 0x3D
	uint8_t fill1[42]; // (0x68 - 0x3D) - 1
	uint8_t pcicr; // 0x68
	uint8_t eicra; // 0x69
	uint8_t eicrb; // 0x6A
	uint8_t pcmsk0; // 0x6B
} Atmega32U4ExternalInterrupts_TypeDef;

// I/O Port (PORTB)
typedef struct {
	uint8_t pin; // 0x23
	uint8_t ddr; // 0x24
	uint8_t port; //0x25
} Atmega32U4PORTB_TypeDef;

// I/O Port (PORTC)
typedef struct {
	uint8_t pin; // 0x26
	uint8_t ddr; // 0x27
	uint8_t port; // 0x28
} Atmega32U4PORTC_TypeDef;

// I/O Port (PORTD)
typedef struct {
	uint8_t pin; // 0x29
	uint8_t ddr; // 0x2A
	uint8_t port; // 0x2B
} Atmega32U4PORTD_TypeDef;

// I/O Port (PORTE)
typedef struct {
	uint8_t pin; // 0x2C
	uint8_t ddr; // 0x2D
	uint8_t port; // 0x2E
} Atmega32U4PORTE_TypeDef;

// I/O Port (PORTF)
typedef struct {
	uint8_t pin; // 0x2F
	uint8_t ddr; // 0x30
	uint8_t port; // 0x31
} Atmega32U4PORTF_TypeDef;

// JTAG Interface (JTAG)
typedef struct {
	uint8_t ocdr; // 0x51
	uint8_t fill1[2]; // (0x54 - 0x51) - 1
	uint8_t mcusr; // 0x54
	uint8_t mcucr; // 0x55
} Atmega32U4JtagInterface_TypeDef;

// Phase Locked Loop (PLL)
typedef struct {
	uint8_t pllcsr; // 0x49
	uint8_t fill1[2]; // (0x52 - 0x49) - 1
	uint8_t pllfrq; // 0x52
} Atmega32U4PhaseLockedLoop_TypeDef;

// Serial Peripheral Interface (SPI)
typedef struct {
	uint8_t spcr; // 0x4C
	uint8_t spsr; // 0x4D
	uint8_t spdr; // 0x4E
} Atmega32U4SerialPeripherialInterface_TypeDef;

// Timer/Counter, 10-bit (TC4)
typedef struct {
	uint8_t tcnt4; // 0xBE
	uint8_t tc4h; // 0xBF
	uint8_t tccr4a; // 0xC0
	uint8_t tccr4b; // 0xC1
	uint8_t tccr4c; // 0xC2
	uint8_t tccr4d; // 0xC3
	uint8_t tccr4e; // 0xC4
} Atmega32U4TimerCounter4_TypeDef;

// Timer/Counter, 16-bit (TC1)
typedef struct {
	uint8_t tccr1a; // 0x80
	uint8_t tccr1b; // 0x81
	uint8_t tccr1c; // 0x82
	uint8_t fill3; // (0x84 - 0x82) - 1
	HighLowByte tcnt1; // 0x84 0x85
} Atmega32U4TimerCounter1_TypeDef;

// Timer/Counter, 16-bit (TC3)
typedef struct {
	uint8_t tccr3a; // 0x90
	uint8_t tccr3b; // 0x91
	uint8_t tccr3c; // 0x92
	uint8_t fill3; // (0x94 - 0x92) - 1
	HighLowByte tcnt3; // 0x94 0x95
} Atmega32U4TimerCounter3_TypeDef;

// Timer/Counter, 8-bit (TC0)
typedef struct {
	uint8_t gtccr; // 0x43
	uint8_t tccr0a; // 0x44
	uint8_t tccr0b; // 0x45
	uint8_t tcnt0; // 0x46
} Atmega32U4TimerCounter0_TypeDef;

// Two Wire Serial Interface (TWI)
typedef struct {
	uint8_t twbr; // 0xB8
	uint8_t twsr; // 0xB9
	uint8_t twar; // 0xBA
	uint8_t twdr; // 0xBB
	uint8_t twcr; // 0xBC
	uint8_t twamr; // 0xBD
} Atmega32U4TwoWireSerialInterface_TypeDef;

// USART (USART1)
typedef struct {
	uint8_t ucsr1a; // 0xC8
	uint8_t ucsr1b; // 0xC9
	uint8_t ucsr1c; // 0xCA
	uint8_t ucsr1d; // 0xCB
	HighLowByte ubrr1; // 0xCC 0xCD
	uint8_t udr1; // 0xCE
} Atmega32U4Usart1_TypeDef;

// USB Device Register (USB_DEVICE)
typedef struct {
	uint8_t uhwcon; // 0xD7
	uint8_t usbcon; // 0xD8
	uint8_t usbsta; // 0xD9
	uint8_t usbint; // 0xDA
	uint8_t fill1[5]; // (0xE0 - 0xDA) - 1
	uint8_t udcon; //0xE0
	uint8_t udint; // 0xE1
	uint8_t udien; // 0xE2
	uint8_t udaddr; // 0xE3
	HighLowByte udfnum; // 0xE4 0xE5
	uint8_t udmfn; // 0xE6
	uint8_t fill2; // (0xE8 - 0xE6) - 1
	uint8_t ueintx; // 0xE8
	uint8_t uenum; // 0xE9
	uint8_t uerst; // 0xEA
	uint8_t ueconx; // 0xEB
	uint8_t uecfg0x; // 0xEC
	uint8_t uecfg1x; // 0xED
	uint8_t uesta0x; // 0xEE
	uint8_t uesta1x; // 0xEF
	uint8_t ueienx; // 0xF0
	uint8_t uedatx; // 0xF1
	uint8_t uebclx; // 0xF2
	uint8_t uebchx; // 0xF3
	uint8_t ueint; // 0xF4
} Atmega32U4UsbDeviceRegister_TypeDef;

// Watchdog Timer (WDT)
typedef struct {
	uint8_t wdtcr; // 0x60
} Atmega32U4WatchdogTimer_TypeDef;

#endif

/***EOF***/

