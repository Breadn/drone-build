/* 
    GND    ->   GND
    Vcc    ->   3.3V
    CE     ->   D9
    CSN    ->   D10
    CLK    ->   D13
    MOSI   ->   D11
    MISO   ->   D12

This receives 1 channels and prints the value on the serial monitor
*/


#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN  9
#define CSN_PIN 10

const byte receiverAddress[5] = {'R','x','B','B','B'};
RF24 radio(CE_PIN, CSN_PIN);

// The sizeof this struct should not exceed 32 bytes
struct ReceivePacket {
  byte ch1;
};

ReceivePacket receivedPacket;

bool newData = false;




void setup() {
    Serial.begin(19200);
    // Radio begin config
    radio.begin();
    radio.setDataRate(RF24_250KBPS);  
    radio.openReadingPipe(1,receiverAddress);
    
    // Start the radio comunication
    radio.startListening();
}


void loop() {
    receiveData();
    showData();
}


void receiveData() {
    if(radio.available()) {
      Serial.println("Data received!");
      radio.read(&receivedPacket, sizeof(ReceivePacket));
      newData = true;
    }
}

void showData() {
  if(newData) {
    int ch1_value = map(receivedPacket.ch1,0,255,1000,2000);
    Serial.println(ch1_value);
    newData = false;
  }
}
