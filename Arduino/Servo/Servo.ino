#include <Servo.h> //Imports the library Servo
//GeeKee CeeBee
#define trigPin 3 // TriggerSensor
#define echoPin 2 // EchoSensor


#define servoPin 12


Servo servo; //Defines the object Servo of type(class) Servo
int angle = 0; // Defines an integer
int SCOOPDELAY = 5;
int MAXANG = 170;
int MINANG = 0;
int duration;
int distance = 10;
int scooping = 0;
int scoopRespawn = 1500;
unsigned long lastScoopMillestone = 0;

unsigned long currentMillis = 0;
void scoop() {
    // The following loop runs until the servo is turned to 180 degrees
    // Serial.println("scooping");
    currentMillis = millis();
    if(scooping == 1){
      if (currentMillis - lastScoopMillestone >= SCOOPDELAY) {
          angle++;
          servo.write(angle);
          lastScoopMillestone = currentMillis;
      }
      if(angle >= MAXANG){
        scooping = 2;
        lastScoopMillestone = currentMillis;
      }
    }
    if(scooping == 2){
      if (currentMillis - lastScoopMillestone >= 500) {
        scooping = 3;
        lastScoopMillestone = currentMillis;
      }
    }
    if(scooping == 3){
      currentMillis = millis();
      if (currentMillis - lastScoopMillestone >= SCOOPDELAY) {
          angle--;
          servo.write(angle);
          lastScoopMillestone = currentMillis;
      }
      if(angle <= MINANG){
        scooping = 4;
        lastScoopMillestone = currentMillis;
      }
    }
    if(scooping == 4){
      if (currentMillis - lastScoopMillestone >= 500) {
        scooping = 0;
      }
    }
}

void checkDistance()
{
    // Clear the trigPin by setting it LOW:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);

  // Trigger the sensor by setting the trigPin high for 10 microseconds:
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin, pulseIn() returns the duration (length of the pulse) in microseconds:
  duration = pulseIn(echoPin, HIGH);
  // Calculate the distance:
  distance = duration * 0.034 / 2;

  // Print the distance on the Serial Monitor (Ctrl+Shift+M):
}

void setup() {
  servo.attach(servoPin, 400, 2740);
  servo.write(MINANG);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);


  Serial.begin(9600);
}

void loop() { 
  Serial.println(distance);
  if(scooping == 0){
    checkDistance();
    if(distance < 9 || distance > 30){
      scooping =1;
    }
  }
  else{
    scoop();
  }
}
