/*
    GND    ->   GND
    Vcc    ->   3.3V
    CE     ->   D9
    CSN    ->   D10
    CLK    ->   D13
    MOSI   ->   D11
    MISO   ->   D12

This code transmits 1 channels with data from pins A0 POTENTIOMETER
*/

// Library includes
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>


#define VRX_PIN 0
#define VRY_PIN 1
#define CE_PIN  9
#define CSN_PIN 10

unsigned long currTime;
unsigned long prevTime;
unsigned long deltaTime = 5;

const byte receiverAddress[5] = {'R','x','B','B','B'};

RF24 radio(CE_PIN, CSN_PIN);  // Create a radio class

// sizeof this struct should not exceed 32 bytes
struct TransmitPacket {
  byte ch1; 
};

TransmitPacket transmittedPacket;



void setup() {
  Serial.begin(19200);
  // Radio begin config
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(false);
  radio.openWritingPipe(receiverAddress);
}


void loop() {
  currTime = millis();
  if(currTime - prevTime >= deltaTime) {
    transmittedPacket.ch1 = map(analogRead(VRY_PIN), 0, 1024, 0, 255);
    send(transmittedPacket);
    prevTime = millis();
  }
}


void send(TransmitPacket packet) {
  Serial.println(packet.ch1);
  radio.write(&packet, sizeof(packet));
  /*;
  if(success) {
    Serial.println("Packet send: Success!");
  }
  else {
    Serial.println("Packet send: Failed!");
  }
  */
}
