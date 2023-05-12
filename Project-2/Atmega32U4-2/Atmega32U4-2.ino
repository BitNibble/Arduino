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
}
#include <inttypes.h>

/*** File Constant and Macros ***/

/*** File Variables ***/
ATMEGA32U4 mega;
EXPLODE pa;
uint8_t a;
uint8_t updown;
uint16_t count;
uint16_t count2;
uint16_t top;
uint16_t analog;

/*** File Header ***/
void PORTINIC(void);

void setup() {
  // put your setup code here, to run once:
  mega = ATMEGA32U4enable();
  pa = EXPLODEenable();
  a = 0;
  updown = 0;
  top = 85;
  count = 0;
  count2 = top;
  PORTINIC();
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

 // Turn on all Interrupt Handlers
 mega.cpu.reg->sreg |= (1 << 7);

 Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  pa.update(&pa.par, PORTB);

  delay(1000);
  analog = mega.readhlbyte(mega.adc.reg->adc);
  Serial.print((uint16_t) analog);
  Serial.println("\n");
  
  //a = pa.par.HL;

}

/*** Procedure and Function Definitions ***/
void PORTINIC(void)
{
  mega.portc.reg->ddr |= (1 << 7);
  mega.portf.reg->ddr &= ~1; // PF0 pin 41 as input
  mega.portf.reg->port |= 1; // PF0 pin 41 pull up resister
}

/*** Interrupt Handlers ***/
ISR(TIMER1_OVF_vect){ // TIMER0 is beeing used by arduino software as the delay procedure, what a waste.
  switch(updown){
    case 0:
      if(count++ > count2){ if(count2 > 0){ mega.portc.reg->port ^=  (1 << 7); count2--; count = 0; }else{ updown = 1;} }
      break;
    case 1:
      if(count++ > count2){ if(count2 < top){ mega.portc.reg->port ^=  (1 << 7); count2++; count = 0; }else{ updown = 0;} }
      break;
    defualt:
      break;
  }
}
ISR(TIMER1_COMPA_vect){
  switch(updown){
    case 0:
      if(count++ > count2){ if(count2 > 0){ mega.portc.reg->port ^=  (1 << 7); count2--; count = 0; }else{ updown = 1;} }
      break;
    case 1:
      if(count++ > count2){ if(count2 < top){ mega.portc.reg->port ^=  (1 << 7); count2++; count = 0; }else{ updown = 0;} }
      break;
    defualt:
      break;
  }
}

/*** EOF ***/


/******
1º Sequence
2º Scope
3º Pointer and Variable
4º Casting
******/

