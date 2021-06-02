// C++ file of function definitions for Electronic Speed Controllers
// Library includes
#include <Servo.h>
#include <Arduino.h>
 
/** General Notes:
 * !!! This configuration is currently WIRED (i.e. no RF communication) !!!
 *  - .writeMicroseconds() has 1000 servo positions, ranging 1000-2000 (compared to .write() with 180 pos over 0-179)
 *  - "servo" controller will act as throttle for esc
 * 
 **/

#define TL_PIN 3
#define BR_PIN 9
#define TR_PIN 10
#define BL_PIN 11

const int ESC_COUNT = 4;

Servo esc_TL;   // D3
Servo esc_BR;   // D9
Servo esc_TR;   // D10
Servo esc_BL;   // D11

Servo ESCs[4]{esc_TL, esc_BR, esc_TR, esc_BL};

int throttle = 1000;

bool statusESC() {
  bool go = true;
    for(int i=0; i<ESC_COUNT; i++) {
        if(ESCs[i].attached()) {
            Serial.print("STATUS: OK   | WHOAMI: esc_0"); Serial.println(i+1);
        }
        else {
            Serial.print("STATUS: N/OK | WHOAMI: esc_0"); Serial.println(i+1);
            go = false;
        }
    }
    return go;
}
 
void beginESC() {
  esc_TL.attach(TL_PIN);           // attach esc pins
  esc_BR.attach(BR_PIN);
  esc_TR.attach(TR_PIN);
  esc_BL.attach(BL_PIN);

  esc_TL.writeMicroseconds(1000);  // set to lowest throttle
  esc_BR.writeMicroseconds(1000);
  esc_TR.writeMicroseconds(1000);
  esc_BL.writeMicroseconds(1000);
}
 
void readThrottle() {
  if(Serial.available() > 0) {
    throttle = (Serial.parseInt() - 1000) % 1001 + 1000;
  }
  /*
  int throttle = analogRead(THROTTLE_PIN);
  throttle = map(throttle, 0, 1023, 1000, 2000);    // maps potentiometer 10-bit ADC values 0-1023 over range of 1000-2000 positions
  //throttle = map(throttle, 0, 1023, 48, 179);     // .write configuration
  esc_TL.writeMicroseconds(throttle);
  */
  
  esc_TL.writeMicroseconds(throttle);
  esc_BR.writeMicroseconds(throttle);
  esc_TR.writeMicroseconds(throttle);
  esc_BL.writeMicroseconds(throttle);
}

void printThrottle() {
  Serial.println(throttle);
}
