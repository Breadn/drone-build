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


////////////////////// PPM CONFIGURATION//////////////////////////
#define NUM_OF_CHANNELS 6 // Number of channels
#define PPM_PIN 2         // PPM signal pin
#define PPM_FrmLen 27000   //set the PPM frame length in ms
#define PPM_PulseLen 400  //set the pulse length
//////////////////////////////////////////////////////////////////

int ppm[NUM_OF_CHANNELS];

RF24 radio(7, 8); // CE, CSN
const byte RF_ADDR[6] = "00001";

const int TIMEOUT_LIMIT = 1000;
unsigned long recvTime;

// The sizeof this struct should not exceed 32 bytes
struct ReceivePacket {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte sw1;
  byte sw2;
};
ReceivePacket packet;

void recvData() {
  if (radio.available()) {
    recvTime = millis();
    radio.read(&packet, sizeof(packet));

    loadPPM();
    printData();
  }
}

void printData() {
  Serial.print("Throttle: "); Serial.print(ppm[0]); Serial.print("\t");
  Serial.print("Yaw: "); Serial.println(ppm[1]);
}

void offPPM() {
  digitalWrite(PPM_PIN, 0);
}

void onPPM() {
  digitalWrite(PPM_PIN, 1);
}

void loadPPM() {
  ppm[0] = map(packet.throttle, 0, 255, 1000, 2000);
  ppm[1] = map(packet.yaw,      0, 255, 1000, 2000);
  ppm[2] = map(packet.pitch,    0, 255, 1000, 2000);
  ppm[3] = map(packet.roll,     0, 255, 1000, 2000);  
  ppm[4] = map(packet.sw1,      0, 1,   1000, 2000);
  ppm[5] = map(packet.sw2,      0, 1,   1000, 2000);
}

// TODO: Have throttle gradually decrease
void resetData() {
  packet.throttle = 0;
  packet.yaw = 127;
  packet.pitch = 127;
  packet.roll = 127;
  packet.sw1 = 0;
  packet.sw2 = 0;

  loadPPM();  
}


//=============================================//


void setup() {
  resetData();
  Serial.begin(9600);

  /***** PPM Setup *****/
  pinMode(PPM_PIN, OUTPUT);
  offPPM();
  //Configure the interruption registers that will create the PPM signal (using timer 1)
  cli();      // disallow interrupts
  TCCR1A = 0; // set entire (HB & LB) TCCR1 register to 0
  TCCR1B = 0;

  OCR1A = 100;  // compare match register (not very important, sets the timeout for the first interrupt)
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);   // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();      // reallow interrupts

  /***** RF Setup *****/
  radio.begin();
  radio.setAutoAck(false);
  radio.openReadingPipe(0, RF_ADDR); // open pipe 0 with specified address
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  recvData();

  if(millis() - recvTime > TIMEOUT_LIMIT) {
    Serial.println("Signal lost!");

    resetData();
  }
}


//=============================================//


#define clockMultiplier 2 // 8*2 = 16MHz (ard nano cpu max freq.)

//Interruption vector execution (to create the PPM signal)
ISR(TIMER1_COMPA_vect){
  static boolean state = true;

  TCNT1 = 0;

  if ( state ) {
    //end pulse
    offPPM(); //PORTD = PORTD & ~B00000100; // turn pin 2 off.
    OCR1A = PPM_PulseLen * clockMultiplier;
    state = false;
  }
  
  else {
    //start pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;

    onPPM(); //PORTD = PORTD | B00000100; // turn pin 2 on.
    state = true;

    // After cycling through all channels to transmit, reset to beginning
    if(cur_chan_numb >= NUM_OF_CHANNELS) {
      cur_chan_numb = 0;
      calc_rest += PPM_PulseLen;
      OCR1A = (PPM_FrmLen - calc_rest) * clockMultiplier;
      calc_rest = 0;
    }
    else {
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * clockMultiplier;
      calc_rest += ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}
