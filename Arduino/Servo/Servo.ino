#include <Servo.h>

#define IRLed 7

#define redLed 51

#define servoPin 8

#define IRsensorPin 2

Servo servo; //Defines the object Servo of type(class) Servo
int angle = 0; // Defines an integer
int SCOOPDELAY = 15;
int MAXANG = 180;
int MINANG = 10;

void scoop()
{
    Serial.println("scooping");
     // The following for loop runs till the servo is turned till 180degrees
    for (angle = MINANG; angle < MAXANG; angle++)
    {
        servo.write(angle);
        delay(SCOOPDELAY);
    }

    // The following for loop goes back till servo is turned till 10degrees
    for (angle = MAXANG; angle > MINANG; angle--)
    {
        servo.write(angle);
        delay(SCOOPDELAY);
    }
}

void setup() {
  servo.attach(servoPin);
  servo.write(angle);

  scoop();

  Serial.begin(115200);
}


void loop() { 
}
