/************************************************************************
	Arduino Leonardo
Author: Sergio Santos
	<sergio.salazar.santos@gmail.com>
License: GNU General Public License
Hardware: Arduino Leonardo
Date: 11052023
Comment:
	
************************************************************************/
/*** File Libraries ***/
extern "C" {
  #include <atmega32u4mapping.h>
  #include <explode.h>
  #include <rotenc.h>
}

/*** File Constant and Macros ***/

/*** File Variables ***/
ATMEGA32U4 mega;
ROTENC pot;

uint16_t analog;
uint8_t potread;
double num;
/*** File Header ***/
void PORTINIC(void);
void timer1setup(void);
void analog0setup(void);

void setup() {
  // put your setup code here, to run once:
  mega = ATMEGA32U4enable();
  pot = ROTENCenable(1, 2);
  PORTINIC();
  timer1setup();
  analog0setup();
  num = 16646655;
  // Turn on all Interrupt Handlers
  mega.cpu.reg->sreg |= (1 << 7);
  Serial.begin(9600);
}
/******* LOOOOOOP ********/
void loop() {
  // put your main code here, to run repeatedly: 
  delay(1000);
  analog = mega.readhlbyte(mega.adc.reg->adc);
  Serial.print((uint16_t) analog);
  Serial.print(" - ");
  Serial.print((uint16_t) potread);
  Serial.print(" - ");
  Serial.print((uint32_t) num);
  Serial.println("\r");
}


/*** Procedure and Function Definitions ***/
void PORTINIC(void)
{
  mega.portc.reg->ddr |= (1 << 7); // LED
  mega.portf.reg->ddr &= ~1; // PF0 pin 41 as input
  mega.portf.reg->port |= 1; // PF0 pin 41 pull up resister
  //rotary encoder potenciometer
  mega.portd.reg->ddr |= (1 << 3); // D1 or TX, [PD3 pin 21]
  mega.portd.reg->port &= ~(1 << 2);
  mega.portd.reg->ddr &= ~((1 << 1) | (1 << 2)); // D0 and D2, [PD2 Pin 20 and PD1 pin 19]
  mega.portd.reg->port |= (1 << 1) | (1 << 2);
}
void timer1setup(void)
{
  /****************** Timer 1 ********************/
  // Power up Timer 1
  mega.cpu.reg->prr0 &= ~(1 << 3);
  //         wavegen mode
  // normal
  //mega.tc1.reg->tccr1a &= ~((1 << WGM11) | (1 << WGM10));
  //mega.tc1.reg->tccr1b &= ~((1 << WGM13) | (1 << WGM12));
  // CTC
  mega.tc1.reg->tccr1a &= ~((1 << WGM11) | (1 << WGM10));
  mega.tc1.reg->tccr1b |= (1 << WGM12);
  mega.tc1.reg->tccr1b &= ~(1 << WGM13);
  //       Interrupt Handler
  // interrupt overflow and on compare match A
  mega.tc1.imask->timsk1 |= ((1 << TOIE1) | (1 << OCIE1A));
  //       Output mode
  // compoutmodeA disconnected
  mega.tc1.reg->tccr1a &= ~((1 << COM1A0) | (1 << COM1A1));
  // compoutmodeB disconnected
  mega.tc1.reg->tccr1a &= ~((1 << COM1B0) | (1 << COM1B1));
  // compoutmodeB disconnected
  mega.tc1.reg->tccr1a &= ~((1 << COM1C0) | (1 << COM1C1));
  //       Output Compare
  // compareA
  mega.tc1.comp->ocr1a = mega.writehlbyte(0x00FF);
  // compareB
  //mega.tc1.reg->ocr1b = mega.writehlbyte(0x00FF);
  // compareC
  //mega.tc1.reg->ocr1c = mega.writehlbyte(0x00FF);
  //       Prescaler
  //mega.tc1.reg->tccr1b |= (5 << CS10); // 1024
  mega.tc1.reg->tccr1b |= (1 << CS10); // 1
  /************************************************/
}
void analog0setup()
{
  /****************** Analog 0 ********************/
  //        Prescaler
  mega.adc.reg->adcsra &= ~(0x07); // Clear parameter field
  mega.adc.reg->adcsra |= 0x07; // Set Division Factor to 128
  //        MUX (ADC channel configure)
  mega.adc.reg->admux &= ~(0x1F); // Clear parameter field
  mega.adc.reg->admux &= ~(0x1F); // Set to ADC0, PF0, pin 41
  //        Reference Selection
  mega.adc.reg->admux &= -(0xC0); // Clear parameter field
  mega.adc.reg->admux |= -(0x40); // AVCC with external capacitor on AREF pin
  //        Enable ADC and Start Conversion
  mega.adc.reg->adcsra |= 0xC0;
  //        Trigger conversion enable
  mega.adc.reg->adcsra |= (1 << 5);
  //        Trigger source selection
  mega.adc.reg->adcsrb &= ~(0x0F); // Clear parameter field
  mega.adc.reg->adcsrb &= ~(0x0F); // Set to free running mode, controlled by ADIF
  /************************************************/
}

/*** Interrupt Handlers ***/
ISR(TIMER1_COMPA_vect){
  potread = pot.rte(&pot.par, mega.portd.reg->pin).num;
}

/*** EOF ***/


/******
1º Sequence
2º Scope
3º Pointer and Variable
4º Casting
******/

