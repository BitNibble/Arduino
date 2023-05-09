extern "C" {
  #include <atmega32u4mapping.h>
  #include <explode.h>
}
#include <inttypes.h>
ATMEGA32U4 mega;
EXPLODE pa;
uint8_t a;
void setup() {
  // put your setup code here, to run once:
  mega = ATMEGA32U4enable();
  pa = EXPLODEenable();
  mega.portc.reg->ddr |= (1 << 7);
}

void loop() {
  // put your main code here, to run repeatedly:
  pa.update(&pa.par, PORTB);
  
  a = pa.par.HL;

  mega.portc.reg->port ^=  (1 << 7);
  delay(100);                      // wait for a second

}

