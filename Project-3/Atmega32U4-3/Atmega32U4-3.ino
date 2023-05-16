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
#include <math.h>
/*** File Constant and Macros ***/
#define tamanhocas 6

/*** File Variables ***/
ATMEGA32U4 mega;
EXPLODE lala;
ROTENC pot;

uint16_t analog;
uint8_t potread;
double num;

typedef struct{
  uint8_t inputas[3];
  uint8_t outputas[2]; // 0 -set and 1 - clear, bits
}ram;

ram caputas[tamanhocas];
ram aqui;
uint8_t select;

/*** File Header ***/
void PORTINIC(void);
void timer1setup(void);
void analog0setup(void);
void laughter(ram* fonix, uint8_t lh, uint8_t hl);

void setup() {
  // put your setup code here, to run once:
  mega = ATMEGA32U4enable();
  lala = EXPLODEenable();
  pot = ROTENCenable(1, 2);
  PORTINIC();
  timer1setup();
  analog0setup();
  for(select=0;select<tamanhocas;caputas[select].inputas[0]=0,caputas[select].inputas[1]=0,caputas[select].inputas[2]=0,caputas[select].outputas[0]=0,caputas[select].outputas[1]=0,select++);

  //num = 16646655;
  caputas[0].inputas[0] = 0;caputas[0].inputas[1] = 0;caputas[0].inputas[2] = 16;
  caputas[0].outputas[0] = 224;
  caputas[0].outputas[1] = 0;

  caputas[1].inputas[0] = 224;caputas[1].inputas[1] = 0;caputas[1].inputas[2] = 16;
  caputas[1].outputas[0] = 0;
  caputas[1].outputas[1] = 224;

  caputas[2].inputas[0] = 0;caputas[2].inputas[1] = 0;caputas[2].inputas[2] = 32;
  caputas[2].outputas[0] = 96;
  caputas[2].outputas[1] = 0;

  caputas[3].inputas[0] = 96;caputas[3].inputas[1] = 0;caputas[3].inputas[2] = 32;
  caputas[3].outputas[0] = 0;
  caputas[3].outputas[1] = 96;

  caputas[4].inputas[0] = 224;caputas[4].inputas[1] = 0;caputas[4].inputas[2] = 32;
  caputas[4].outputas[0] = 0;
  caputas[4].outputas[1] = 32;

  caputas[5].inputas[0] = 192;caputas[4].inputas[1] = 0;caputas[4].inputas[2] = 32;
  caputas[5].outputas[0] = 0;
  caputas[5].outputas[1] = 192;

  // Turn on all Interrupt Handlers
  mega.cpu.reg->sreg |= (1 << 7);

  lala.par.XF=lala.par.XI=0xFF;
  aqui.inputas[0] = 0;

  Serial.begin(9600);
}


/***********************************************************************************/
/******* LOOOOOOP ********/
void loop() {
  // put your main code here, to run repeatedly:
  lala.update(&lala.par, mega.portb.reg->pin);

  laughter(&aqui, lala.par.LH & ~1, lala.par.HL & ~1);

  mega.portc.reg->port = aqui.inputas[0];

  delay(1000);
  analog = mega.readhlbyte(mega.adc.reg->adc);

  //if(Serial){
  //Serial.print((uint16_t) analog);
  //Serial.print(" - ");
  //Serial.print((uint16_t) potread);
  Serial.print(" i1 "); 
  Serial.print((uint8_t) aqui.inputas[0]);
  Serial.print(" i2 "); 
  Serial.print((uint8_t) aqui.inputas[1]);
  Serial.print(" i3 "); 
  Serial.print((uint8_t) aqui.inputas[2]);
  Serial.print(" -> ");
  Serial.print((uint8_t) aqui.inputas[0]);
  Serial.println("\r");
  //}
}
/***********************************************************************************/


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

  mega.portb.reg->ddr &= ~0xFE; // as inputs
  mega.portb.reg->port |= 0xFE; // with pullup


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
void laughter(ram* fonix, uint8_t lh, uint8_t hl)
{
  uint8_t index=0;
  fonix->inputas[1] = lh;
  fonix->inputas[2] = hl;
      for(index = 0; index < tamanhocas; index++){
        if((fonix->inputas[0]) == caputas[index].inputas[0]){
          if(fonix->inputas[1] == caputas[index].inputas[1]){
            if(fonix->inputas[2] == caputas[index].inputas[2]){
              fonix->inputas[0] |= caputas[index].outputas[0];
              fonix->inputas[0] &= ~caputas[index].outputas[1];;
              index=tamanhocas;
            }
          }
        }
      }
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

