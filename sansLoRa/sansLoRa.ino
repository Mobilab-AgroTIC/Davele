/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.
  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/
#define PIN_TRANSISTOR GPIO2
#define PIN_POLAR GPIO7

// variables for Lidar
uint16_t count = 0;
uint16_t error = 0;

// variable for Polar
unsigned long duration,startTime,startHB,loopTime;
float freq,sumDuration;
int countHB, bpm, err;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital PIN_TRANSISTOR LED_BUILTIN as an output.
  Serial.begin(115200);
  Serial1.begin(115200);
  pinMode(PIN_POLAR, INPUT); //Polar HeartBeat input
  digitalWrite(PIN_TRANSISTOR, LOW); //Polar HeartBeat input

}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(PIN_TRANSISTOR, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(2000);                       // wait for a second
  // get distance with TF.Mini
  uint16_t distance = readTFMini();
  Serial.print(F("Distance ON en cm : "));
  Serial.println(distance);
  Serial.println("");
  digitalWrite(PIN_TRANSISTOR, LOW);    // turn the LED off by making the voltage LOW
  delay(2000);                       // wait for a second
  // get distance with TF.Mini
  distance = readTFMini();
  Serial.print(F("Distance OFF en cm : "));
  Serial.println(distance);
  Serial.println("");


  // get 10 heartbeats and get frequency
  countHB = 0;
  err = 0;
  while (countHB < 10){
    startHB = millis();
    startTime = millis();
    duration = pulseIn(PIN_POLAR, HIGH);
    loopTime = millis() - startTime;
    freq=60000/float(loopTime);
    Serial.printf("\n count : %d\n", countHB);
    Serial.printf("\n duration : %d\n", duration);
    Serial.printf("\n frequency : %d\n", freq);

    if (freq > 20 && freq < 240){
      countHB++;
      sumDuration += freq;
    }
    if (millis() - startTime > 30000) {
      err = 1;
      break;
    }
  }
  bpm = round(sumDuration/countHB);
  Serial.printf("\n bpm : %d\n", bpm);
  countHB = 0;
  sumDuration = 0;
    
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
