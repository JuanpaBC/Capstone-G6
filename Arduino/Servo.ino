#include <Servo.h> //Imports the library Servo

Servo servo;   // Defines the object Servo of type(class) Servo
int angle = 0; // Defines an integer

void setup()
{
    servo.attach(1);    // States that the servo is attached to pin 5
    servo.write(angle); // Sets the servo angle to 10degrees
}

void loop()
{

    // The following for loop runs till the servo is turned till 180degrees
    for (angle = 10; angle < 180; angle++)
    {
        servo.write(angle);
        delay(15);
    }

    // The following for loop goes back till servo is turned till 10degrees
    for (angle = 180; angle > 10; angle--)
    {
        servo.write(angle);
        delay(15);
    }
}