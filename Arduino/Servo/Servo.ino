#include <Servo.h> //Imports the library Servo
//GeeKee CeeBee
#define trigPin 3 // TriggerSensor
#define echoPin 2 // EchoSensor


#define servoPin 8


Servo servo; //Defines the object Servo of type(class) Servo
int angle = 0; // Defines an integer
int SCOOPDELAY = 5;
int MAXANG = 170;
int MINANG = 0;
int duration;
int distance = 10;

int scoopRespawn = 1500;
void scoop()
{
    Serial.println("scooping");
     // The following for loop runs till the servo is turned till 180degrees
    for (angle = MINANG; angle < MAXANG; angle++)
    {
        servo.write(angle);
        delay(SCOOPDELAY);
    }
    delay(500);

    // The following for loop goes back till servo is turned till 10degrees
    for (angle = MAXANG; angle > MINANG; angle--)
    {
        servo.write(angle);
        delay(SCOOPDELAY);
    }
    delay(500);
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
  servo.attach(servoPin, 771, 2740);
  servo.write(MINANG);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);


  Serial.begin(115200);
}
int hola =0;

void loop() { 
  scoop();
  //checkDistance();
  Serial.println(distance);
  if(distance < 9 ){
    hola = hola + 1;
    if(hola > 10){
      //scoop();
      delay(1000);
    }
  }
  else{
    hola = 0;
  }
}
