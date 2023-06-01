unsigned long startTime,loopTime;
float freq,sumDuration;
int count,duration;
void setup() {
  Serial.begin(115200);
  pinMode(GPIO7, INPUT);}

void loop() {
  for (int t=0; t<10; t++){
  startTime = millis();
  duration = pulseIn(GPIO7, HIGH);
  loopTime = millis() - startTime;
  freq=60000/float(loopTime);
  Serial.print("duration : ");
  Serial.println(duration);
  Serial.print("loopTime : ");
  Serial.println(loopTime);
  Serial.print("freq : ");
  Serial.println(freq);
  Serial.println("  ");
  if (freq > 20 && freq < 240){
    count++;
    sumDuration += freq;
  }
  }
  Serial.print("sumDuration : ");
  Serial.println(sumDuration);
  Serial.println(sumDuration/count);
  count = 0;
  sumDuration = 0;
}
