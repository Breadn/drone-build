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

#define VRX_PIN 0
#define VRY_PIN 1
#define CE_PIN  7
#define CSN_PIN 8

RF24 radio(CE_PIN, CSN_PIN); // CE, CSN

const byte address[6] = "00001";

unsigned long currTime;
unsigned long prevTime;
unsigned long deltaTime = 50;

// sizeof this struct should not exceed 32 bytes
struct TransmitPacket {
  unsigned short ch1; 
};
TransmitPacket transmittedPacket;


void setup() {
  radio.begin();
  radio.setAutoAck(false);
  radio.openWritingPipe(address); // open pipe 0 with specified address
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  currTime = millis();
  if(currTime - prevTime >= deltaTime) {
    transmittedPacket.ch1 = map(analogRead(VRY_PIN), 0, 1023, 1000, 1075);
    radio.write(&transmittedPacket, sizeof(transmittedPacket));
    prevTime = millis();
  }
}
