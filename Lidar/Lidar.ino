#include "LoRaWanMinimal_APP.h"
#include "Arduino.h"

// pin connexions
#define PIN_TRANSISTOR GPIO3

//Set these OTAA parameters to match your app/node in TTN
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x05, 0xDE, 0x13 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0xB9, 0x65, 0x4E, 0xAB, 0xAF, 0x60, 0x83, 0x16, 0x8E, 0x23, 0x50, 0xBC, 0x90, 0xB9, 0xE7, 0x11 };

int temps = 180; // Frequency to wake up the system, in seconds (DO NOT GO UNDER 300, 5min. Remeber of the fair use policy)


// Parameters for LoRaWAN
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };
static uint8_t counter=0;
uint8_t lora_data[5];
uint8_t downlink ;


// variables for Lidar
uint16_t count = 0;
uint16_t error = 0;



///////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  pinMode(PIN_TRANSISTOR, OUTPUT); //Polar HeartBeat input
  digitalWrite(PIN_TRANSISTOR, LOW); //Polar HeartBeat input
}


///////////////////////////////////////////////////
void loop()
{
  digitalWrite(PIN_TRANSISTOR, HIGH); //activate transistor
  delay(2000);
  // get distance with TF.Mini
  uint16_t distance = readTFMini();
  Serial.print(F("Distance IN en cm : "));
  Serial.println(distance);
  Serial.println("");
  digitalWrite(PIN_TRANSISTOR, LOW); //Turn off transistor
  delay(2000);
  distance = readTFMini();
  Serial.print(F("Distance OUT en cm : "));
  Serial.println(distance);
  Serial.println("");
}


uint16_t readTFMini() {
 uint8_t current;
 static uint8_t n = 0;
 static uint8_t sum = 0;
 static uint16_t distance = 0;
 static uint16_t distanceOut = 0;
 
 while(Serial1.available()) {
  current = Serial1.read();

  switch(n) {

   case 0:
    if(current == 0x59) {
     sum += current;
     n++;
    }
    break;

   case 1:
    if(current == 0x59) {
     sum += current;
     n++;
    } else
     n = 0;
    break;

   case 2:
    distance = current;
    sum += current;
    n++;
    break;

   case 3:
    distance += current << 8;
    sum += current;
    n++;
    break;

   case 4:
    sum += current;
    n++;
    break;

   case 5:
    sum += current;
    n++;
    break;

   case 6:
    sum += current;
    n++;
    break;

   case 7:
    sum += current;
    n++;
    break;

   case 8:
    if(sum == current) {
     if(distance != 65535) {
      distanceOut = distance;
      // We get a new measure chose what you want to do here 
      count++; // I choose to increment a global value ... 
     }
    } else {
     // Error occured, choose what you want to do here ... 
      error++; // I choose to increment a global value ... 
    }
    sum = 0;
    n = 0;
    break;

  }
 }
 return distanceOut;  // Allways return the latest good value 
}
