extern "C" {
  #include <atmega32u4mapping.h>
  #include <explode.h>
}
#include <inttypes.h>
ATMEGA32U4 mega;
EXPLODE pa;
uint8_t a;
uint8_t count;
uint8_t count2;
uint8_t top;
void setup() {
  // put your setup code here, to run once:
  mega = ATMEGA32U4enable();
  pa = EXPLODEenable();
  a = 0;
  top = 64;
  count = top;
  count2 = count;
  mega.portc.reg->ddr |= (1 << 7);
 // Power up Timer 1
 mega.cpu.reg->prr0 &= ~(1 << 3);
 // Timer 1
 // wavegenmode normal
 mega.tc1.reg->tccr1a &= ~((1 << WGM11) | (1 << WGM10));
 mega.tc1.reg->tccr1b &= ~((1 << WGM13) | (1 << WGM12));
 // interrupt overflow
 mega.tc1.reg->timsk1 |= (1 << TOIE1);
 // compoutmodeA disconnected
 mega.tc1.reg->tccr1a &= ~((1 << COM1A0) | (1 << COM1A1));
 // compoutmodeB disconnected
 mega.tc1.reg->tccr1a &= ~((1 << COM1B0) | (1 << COM1B1));
 // compoutmodeB disconnected
 mega.tc1.reg->tccr1a &= ~((1 << COM1C0) | (1 << COM1C1));
 // compareA
 //mega.tc1.reg->ocr1a = mega.writehlbyte(0x00FF);
 // compareB
 //mega.tc1.reg->ocr1b = mega.writehlbyte(0x00FF);
 // compareC
 //mega.tc1.reg->ocr1c = mega.writehlbyte(0x00FF);
 // prescaler
 //mega.tc1.reg->tccr1b |= (5 << CS10); // 1024
 mega.tc1.reg->tccr1b |= (1 << CS10); // 1

// Turn on all Interrupt Hnadler
 mega.cpu.reg->sreg |= (1 << 7);

 Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  pa.update(&pa.par, PORTB);
  
  //Serial.println(": gfdgf");
  //Serial.print((int)mega.readhlbyte(mega.tc1.reg->tcnt1));
  Serial.print((int)count);
  Serial.println("\n");
  //a = pa.par.HL;

}

ISR(TIMER1_OVF_vect){
  if(count++ > count2){ mega.portc.reg->port ^=  (1 << 7); if(count2 > 0){ count2--; count = 0; }else{ count2 = top; count = 0; } }
}

