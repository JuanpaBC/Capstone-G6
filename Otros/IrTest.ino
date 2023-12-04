#include <IRremote.h>
#include <Servo.h>
//#define SW 8
#define IR_RECEIVE_PIN 2
#define IR_EMISOR_PIN 3

#define servoPin 8

Servo servo; //Defines the object Servo of type(class) Servo
int angle = 0; // Defines an integer
int SCOOPDELAY = 15;
int MAXANG = 180;
int MINANG = 0;
int pickup = 0;

IRsend irsend(IR_EMISOR_PIN);
IRrecv irrecv(IR_RECEIVE_PIN);
unsigned long previousMillis = 0;
unsigned long lastSensorDetection = 0;
const long interval = 500; 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  irrecv.enableIRIn();
  servo.attach(servoPin);
  servo.write(angle);
}

void scoop()
{
    Serial.println("scooping");
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
void loop() {
  // put your main code here, to run repeatedly:
  // analogWrite(LED, 125);
  unsigned long currentMillis = millis();

  if (IrReceiver.decode()) {
    IrReceiver.printIRResultShort(&Serial);
    IrReceiver.resume();
    lastSensorDetection = currentMillis;
  }
  if(pickup == 0 && currentMillis - lastSensorDetection > 2000){
    pickup = 1;
  }
  if(pickup == 1){
    scoop();
    pickup = 0;
    currentMillis =  millis();
    lastSensorDetection = currentMillis;
  }
  // Your code to send IR signals at a regular interval
  if (currentMillis - previousMillis >= interval) {
    irsend.sendNEC(0xF, 1); // Example NEC infrared code
    previousMillis = currentMillis;  // Save the last time the signal was sent
  }

}
