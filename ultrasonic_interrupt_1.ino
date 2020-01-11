const int triggerPin=13;
const int echoPin=15;

volatile long  travelTime=0;
volatile long startTime = 0;
volatile long endTime = 0;

void measure_distance();

void setup() {
  // put your setup code here, to run once:
  pinMode(triggerPin,OUTPUT);
  pinMode(echoPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(echoPin), measure_distance, CHANGE);
  Serial.begin(115200);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(5);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  Serial.print("The distance in cm is:");
  Serial.println(travelTime/58);
}

void measure_distance(){
  switch (digitalRead(echoPin)){

    case HIGH:
      travelTime = 0;
      startTime = micros();
      break;


    case LOW:
      endTime = micros();
      travelTime = endTime-startTime;
      break;
      
      
  }

  
}
