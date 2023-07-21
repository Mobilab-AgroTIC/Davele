#include "LoRaWanMinimal_APP.h"
#include "Arduino.h"

// pin connexions
#define PIN_TRANSISTOR GPIO2
#define PIN_POLAR GPIO7
#define PIN_PHOTOR ADC2

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

// variable for Polar
unsigned long duration;
float freq,sumDuration;
int countHB, bpm, err,startTime,startHB,loopTime;
#include "heartbeats.h"
long int startHB;
long int stopHB;
uint8_t countHB;

uint8_t flag = 0;
// variables for photoRes
int photoR, lux;

//Some utilities for going into low power mode
TimerEvent_t sleepTimer;
//Records whether our sleep/low power timer expired
bool sleepTimerExpired;
static void wakeUp()
{
  sleepTimerExpired=true;
}
static void lowPowerSleep(uint32_t sleeptime)
{
  sleepTimerExpired=false;
  TimerInit( &sleepTimer, &wakeUp );
  TimerSetValue( &sleepTimer, sleeptime );
  TimerStart( &sleepTimer );
  while (!sleepTimerExpired) lowPowerHandler();
  TimerStop( &sleepTimer );
}

///////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(PIN_POLAR, INPUT); //Polar HeartBeat input
  pinMode(PIN_TRANSISTOR, OUTPUT); //Polar HeartBeat input
  digitalWrite(PIN_TRANSISTOR, LOW); //Polar HeartBeat input

  LoRaWAN.begin(LORAWAN_CLASS, ACTIVE_REGION);
  LoRaWAN.setAdaptiveDR(true);
  while (1) {
    Serial.print("Joining... ");
    LoRaWAN.joinOTAA(appEui, appKey, devEui);
    if (!LoRaWAN.isJoined()) {
      Serial.println("JOIN FAILED! Sleeping for 30 seconds");
      lowPowerSleep(30000);
    } else {
      Serial.println("JOINED");
      break;
    }
  }
}


///////////////////////////////////////////////////
void loop()
{
  //WakeUp and get battery Voltage
  counter++; 
  delay(10);
  uint8_t voltage = getBatteryVoltage()/50; //Voltage, in % (approx)
  Serial.printf("\nVoltage : %d\n", voltage);
  lora_data[0] = voltage;

  digitalWrite(PIN_TRANSISTOR, HIGH); //activate transistor
  delay(2000);

  // get distance with TF.Mini
  uint16_t distance = readTFMini();
  Serial.print(F("Distance en cm : "));
  Serial.println(distance);
  Serial.println("");

  lora_data[1] = highByte(distance);
  lora_data[2] = lowByte(distance);


  // get 10 heartbeats and get frequency
  lora_data[3] = heartbeats(PIN_POLAR, 1);


  // get luminosity
  photoR = analogRead(PIN_PHOTOR);
  lux = photoR*5.00/4096.00;
  lora_data[4] = lux;
  Serial.printf("\n lux : %d\n", lux);

  digitalWrite(PIN_TRANSISTOR, LOW); //Turn off transistor
  
  //Now send the data.
  Serial.printf("\nSending packet with counter=%d\n", counter);
  Serial.printf("\nValue to send 1: %d\n", lora_data[1]);
  //Here we send confirmed packed (ACK requested) only for the first two (remember there is a fair use policy)
  bool requestack=counter<2?true:false;
  if (LoRaWAN.send(sizeof(lora_data), lora_data, 1, requestack)) {  // The parameters are "data size, data pointer, port, request ack"
    Serial.println("Send OK");
  } else {
    Serial.println("Send FAILED");
  }
  lowPowerSleep(temps*1000);  
  delay(10);
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

void timer1(void)
{
  startHB = millis();
  detachInterrupt(input_pin);
  flag = 1;
}

void timer2(void)
{
  countHB ++;
  if(countHB == hb_max)
  {
    detachInterrupt(input_pin);
    stopHB = millis();
    flag = 3;
  }
}

uint8_t heartbeats(int input_pin, int8_t hb_max)
{
  uint8_t data = 0;
  uint8_t err = 0;
  uint8_t loop = 1;
  while(loop)    // boucle infini à modif plus tard
  {
    switch(flag)
    {
      case 0 : 
        attachInterrupt(input_pin, timer1, RISING);
        break;

      case 1 :
        attachInterrupt(input_pin, timer2, RISING);
        flag++;
        break;

      case 2 :
        //do nothing
        break;

      case 3 :
        long int time = stopHB - startHB;              // valeur du temps en ms
        uint8_t bpm = (uint8_t)((countHB/(time/1000))*60);  // conversion temps/battements en bpm
        loop = 0;                         // on stop la boucle
        break;

      default :
        // do nothing
        break;
    }
    if((millis() - startHB) > hb_max*3000 && countHB <= hb_max) //si on prend des mesures et que l'on a que 1 battement au bout de 3s+ erreur
    {
      err == 1;
      loop = 0;       // on stop la boucle
    }
  }

  if (err==1)   //if the polar belt is not responding
  {  
    data = err;
  }
  else 
  {
    data = bpm;
  }
  return data;
}
