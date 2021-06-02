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

#include <Servo.h> // temp
#define TEST_PIN 5 // temp

RF24 radio(7, 8); // CE, CSN

Servo testESC;  // temp
int throttle = 1000;  // temp

const byte address[6] = "00001";

// The sizeof this struct should not exceed 32 bytes
struct ReceivePacket {
  unsigned short ch1;
};
ReceivePacket receivedPacket;


void setup() {
  Serial.begin(9600);
  
  testESC.attach(TEST_PIN); // temp
  testESC.writeMicroseconds(1000); // temp
  
  radio.begin();
  radio.setAutoAck(false);
  radio.openReadingPipe(0, address); // open pipe 0 with specified address
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    Serial.println("Data received!");
    radio.read(&receivedPacket, sizeof(receivedPacket));
    Serial.println(receivedPacket.ch1);
    testESC.writeMicroseconds(receivedPacket.ch1);  // temp
  }
}
