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
const int ESC_COUNT = 4;

Servo esc_01;   // Create class for each ESC
Servo esc_02;
Servo esc_03;
Servo esc_04;

Servo ESCs[4]{esc_01, esc_02, esc_03, esc_04};

int THROTTLE_PIN = 0;    // A0 for LEFT joystick
int MANUEVER_PIN = 1;    // A1 for RIGHT joystick

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
  esc_01.attach(9);                // esc signal to pin 9
  esc_01.writeMicroseconds(1000);  // set to lowest throttle
}
 
void readThrottle() {
  int throttle = analogRead(THROTTLE_PIN);
  throttle = map(throttle, 0, 1023, 1000, 2000);    // maps potentiometer 10-bit ADC values 0-1023 over range of 1000-2000 positions
  //throttle = map(throttle, 0, 1023, 48, 179);     // .write configuration
  esc_01.writeMicroseconds(throttle);
}