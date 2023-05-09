/********************************************************************
	ATMEGA 32U4 MAPPING
Author: Sergio Manuel Santos
	<sergio.salazar.santos@gmail.com>
License: GNU General Public License
Hardware: Atmega32U4 by ETT ET-BASE
Date: 09052023
Comment:
	Virtual Image Atmega 32U4 mapping and linking.
*********************************************************************/
/*** File Library ***/
#include "atmega32u4mapping.h"

/*** File Define & Macro ***/

/***File Variable***/
ATMEGA32U4 ret;

/***File Header***/
uint16_t ReadHLByte(HighLowByte reg);
uint16_t ReadLHByte(HighLowByte reg);
HighLowByte WriteHLByte(uint16_t val);
HighLowByte WriteLHByte(uint16_t val);
uint16_t SwapByte(uint16_t num);

/*** File Procedure & Function ***/
ATMEGA32U4 ATMEGA32U4enable(void){
	// Assign
	// GPWR
	ret.gpwr.reg = (Atmega32U4GPWR_TypeDef*) Atmega32U4GPWR_Address;
	// AC
	ret.ac.reg = (Atmega32U4AnalogComparator_TypeDef*) Atmega32U4AnalogComparator_Address;
	// ADC
	ret.adc.reg = (Atmega32U4AnalogToDigitalConverter_TypeDef*) Atmega32U4AnalogToDigitalConverter_Address;
	#if defined(_ATMEGA32U4ANALOG_H_)
		ret.adc.enable = ANALOGenable;
	#endif
	// BOOT_LOAD
	ret.boot_load.reg = (Atmega32U4Bootloader_TypeDef*) Atmega32U4Bootloader_Address;
	// CPU
	ret.cpu.reg = (Atmega32U4CPURegister_TypeDef*) Atmega32U4CPURegister_Address;
	// EEPROM
	ret.eeprom.reg = (Atmega32U4Eeprom_TypeDef*) Atmega32U4Eeprom_Address;
	#if defined(_ATMEGAEEPROM_H_)
		ret.eeprom.enable = EEPROMenable;
	#endif
	// EXINT
	ret.exint.reg = (Atmega32U4ExternalInterrupts_TypeDef*) Atmega32U4ExternalInterrupts_Address;
	#if defined(_ATMEGA32U4INTERRUPT_H_)
		ret.exint.enable = INTERRUPTenable;
	#endif
	// PORTB
	ret.portb.reg = (Atmega32U4PORTB_TypeDef*) Atmega32U4PORTB_Address;
	// PORTC
	ret.portc.reg = (Atmega32U4PORTC_TypeDef*) Atmega32U4PORTC_Address;
	// PORTD
	ret.portd.reg = (Atmega32U4PORTD_TypeDef*) Atmega32U4PORTD_Address;
	// PORTE
	ret.porte.reg = (Atmega32U4PORTE_TypeDef*) Atmega32U4PORTE_Address;
	// PORTF
	ret.portf.reg = (Atmega32U4PORTF_TypeDef*) Atmega32U4PORTF_Address;
	// JTAG
	ret.jtag.reg = (Atmega32U4JtagInterface_TypeDef*) Atmega32U4JtagInterface_Address;
	// PLL
	ret.pll.reg = (Atmega32U4PhaseLockedLoop_TypeDef*) Atmega32U4PhaseLockedLoop_Address;
	// SPI
	ret.spi.reg = (Atmega32U4SerialPeripherialInterface_TypeDef*) Atmega32U4SerialPeripherialInterface_Address;
	#if defined(_ATMEGA32U4SPI_H_)
		ret.spi.enable = SPIenable;
	#endif
	// TC4
	ret.tc4.reg = (Atmega32U4TimerCounter4_TypeDef*) Atmega32U4TimerCounter4_Address;
	#if defined(_ATMEGA32U4TIMER_H_)
		ret.tc4.enable = TIMER_COUNTER4enable;
	#endif
	// TC1
	ret.tc1.reg = (Atmega32U4TimerCounter1_TypeDef*) Atmega32U4TimerCounter1_Address;
	#if defined(_ATMEGA32U4TIMER_H_)
		ret.tc1.enable = TIMER_COUNTER1enable;
	#endif
	// TC3
	ret.tc3.reg = (Atmega32U4TimerCounter3_TypeDef*) Atmega32U4TimerCounter3_Address;
	#if defined(_ATMEGA32U4TIMER_H_)
		ret.tc3.enable = TIMER_COUNTER3enable;
	#endif
	// TC2
	ret.tc0.reg = (Atmega32U4TimerCounter0_TypeDef*) Atmega32U4TimerCounter0_Address;
	#if defined(_ATMEGA32U4TIMER_H_)
		ret.tc0.enable = TIMER_COUNTER0enable;
	#endif
	// TWI
	ret.twi.reg = (Atmega32U4TwoWireSerialInterface_TypeDef*) Atmega32U4TwoWireSerialInterface_Address;
	#if defined(_ATMEGA32U4TWI_H_)
		ret.twi.enable = TWIenable;
	#endif
	// USART1
	ret.usart1.reg = (Atmega32U4Usart1_TypeDef*) Atmega32U4Usart1_Address;
	#if defined(_ATMEGA32U4UART_H_)
		ret.usart1.enable = UARTenable;
	#endif
	// USB_DEVICE
	ret.usb_device.reg = (Atmega32U4UsbDeviceRegister_TypeDef*) Atmega32U4UsbDeviceRegister_Address;
	// WDT
	ret.wdt.reg = (Atmega32U4WatchdogTimer_TypeDef*) Atmega32U4WatchdogTimer_Address;
	// func
	ret.readhlbyte = ReadHLByte;
	ret.readlhbyte = ReadLHByte;
	ret.writehlbyte = WriteHLByte;
	ret.writelhbyte = WriteLHByte;
	ret.swapbyte = SwapByte;
	return ret;
}

// COMMON
uint16_t ReadHLByte(HighLowByte reg)
{
	return (reg.H << 8) | reg.L;
}

uint16_t ReadLHByte(HighLowByte reg)
{
	return (reg.L << 8) | reg.H;
}

HighLowByte WriteHLByte(uint16_t val) // AVR normal little endian
{
	HighLowByte reg; reg.H = (val >> 8); reg.L = val;
	return reg;
}

HighLowByte WriteLHByte(uint16_t val)
{
	HighLowByte reg; reg.L = (val >> 8); reg.H = val;
	return reg;
}

uint16_t SwapByte(uint16_t num)
{
	uint16_t tp;
	tp = (num << 8);
	return (num >> 8) | tp;
}

/*** File Interrupt ***/
// ISR(RESET_vect){}
// ISR(INT0_vect){}
// ISR(INT1_vect){}
// ISR(PCINT0_vect){}
// ISR(PCINT1_vect){}
// ISR(PCINT2_vect){}
// ISR(WDT_vect){}
// ISR(TIMER2_COMPA_vect){}
// ISR(TIMER2_COMPB_vect){}
// ISR(TIMER2_OVF_vect){}
// ISR(TIMER1_CAPT_vect){}
// ISR(TIMER1_COMPA_vect){}
// ISR(TIMER1_COMPB_vect){}
// ISR(TIMER1_OVF_vect){}
// ISR(TIMER0_COMPA_vect){}
// ISR(TIMER0_COMPB_vect){}
// ISR(TIMER0_OVF_vect){}
// ISR(SPI_STC_vect){}
// ISR(USART_RX_vect){}
// ISR(USART_UDRE_vect){}
// ISR(USART_TX_vect){}
// ISR(ADC_vect){}
// ISR(EE_READY_vect){}
// ISR(ANALOG_INTERRUPT)
// ISR(ANALOG_COMP_vect){}
// ISR(TWI_vect){}
// ISR(SPM_READY_vect){}

/***EOF***/

/******
1� Sequence
2� Scope
3� Pointer and Variable
4� Casting
******/
