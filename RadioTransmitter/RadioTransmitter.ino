/*
    GND    ->   GND
    Vcc    ->   3.3V
    CE     ->   D9
    CSN    ->   D10
    CLK    ->   D13
    MOSI   ->   D11
    MISO   ->   D12
*/

// Library includes
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define THROT_PIN 0
#define YAW_PIN 1
#define PITCH_PIN 2
#define ROLL_PIN 3

#define CE_PIN  7
#define CSN_PIN 8

RF24 radio(CE_PIN, CSN_PIN); // CE, CSN

const byte address[6] = "00001";

/* Trims extend left-side of range (defaults are over 1500 center) */
// const int THROT_TRIM = 20;
const int YAW_TRIM = 10;
// const int PITCH_TRIM = 0;
// const int ROLL_TRIM = 0;

unsigned long currTime;
unsigned long prevTime;
unsigned long deltaTime = 50;

// sizeof this struct should not exceed 32 bytes
struct TransmitPacket {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte sw1;
  byte sw2;
};
TransmitPacket data;


void resetData() {
  data.throttle = 0;
  data.yaw = 126;
  data.pitch = 126;
  data.roll = 126;
  data.sw1 = 0;
  data.sw2 = 0;
}

void readData() {
  data.throttle = map(analogRead(THROT_PIN), 0, 1023, 0, 255);
  data.yaw      = map(analogRead(YAW_PIN) - YAW_TRIM, 0 - YAW_TRIM, 1023, 0, 255);
  //data.pitch    = map(analogRead(PITCH_PIN), 0, 1023, 0, 255);
  //data.roll     = map(analogRead(ROLL_PIN), 0, 1023, 0, 255);
}

void printData() {
  Serial.print("Throttle: "); Serial.print(map(data.throttle, 0, 255, 1000, 2000)); Serial.print("\t");
  Serial.print("Yaw: ");      Serial.print(map(data.yaw, 0, 255, 1000, 2000));  Serial.print("\t");
  Serial.print("Pitch: ");      Serial.print(map(data.pitch, 0, 255, 1000, 2000));  Serial.print("\t");
  Serial.print("Roll: ");      Serial.println(map(data.roll, 0, 255, 1000, 2000));
}


//=============================================//


void setup() {
  Serial.begin(9600);

  resetData();

  radio.begin();
  radio.setAutoAck(false);
  radio.openWritingPipe(address); // open pipe 0 with specified address
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  currTime = millis();
  if (currTime - prevTime >= deltaTime) {
    readData();

    printData();

    radio.write(&data, sizeof(data));
    prevTime = millis();
  }
}
