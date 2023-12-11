#include <Servo.h> //Imports the library Servo
//GeeKee CeeBee
#define trigPin 3 // TriggerSensor
#define echoPin 2 // EchoSensor

#define servoPin 12 // Servo pin
#define MAXANG 180 // Servo máx angle
#define MINANG 0 // Servo min angle
#define SCOOPDELAY 5

#define ENA 6 // D6
#define ENB 11

#define AIN1 4 // D4
#define AIN2 5 // D5
#define BIN1 10 // D10
#define BIN2 9 // D9

#define AC1 21 // D2
#define AC2 20 // D3
#define BC1 19 // D7
#define BC2 18 // D8

#define baud 115200

#define pi 3.1415

Servo servo; //Defines the object Servo of type(class) Servo
int angle = 0; // Defines an integer

// ************ DEFINITIONS A************
volatile long EncoderCountA = 0;
float ThetaA_prev, ThetaB_prev;
float RPM_A;
float vel_A;
int PWM_A_val;
float Dist_A;
// ************ DEFINITIONS B************
volatile long EncoderCountB = 0;
float ThetaA, ThetaB;
float RPM_B;
float vel_B;
int PWM_B_val;
float Dist_B;


float Pos_x, Pos_y;
float Vel_lin, Vel_x, Vel_y;
float Vel_ang, Theta;

unsigned long t, t_prev;
int dt;

float NFactor = 1500;
int PWM_min = 150;
int PWM_max = 255;

char msg[60];
float Diam_ruedas = 0.065;
float R_ruedas = Diam_ruedas/2;
float L_robot = (320-26)/2;

int instruction = -1;

int duration;
int distance;

unsigned long startScoop = 0;
unsigned long currentMillis = 0;

int agarro_castana = 0;
int scooping = 0;


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

void scoop() {
    // The following loop runs until the servo is turned to 180 degrees
    // Serial.println("scooping");
    if(scooping == 1){
      currentMillis = millis();
      if (currentMillis - startScoop >= SCOOPDELAY) {
          angle++;
          servo.write(angle);
          startScoop = currentMillis;
      }
      if(angle >= MAXANG){
        scooping = 2;
      }
    }
    if(scooping == 2){
      currentMillis = millis();
      if (currentMillis - startScoop >= SCOOPDELAY) {
          angle--;
          servo.write(angle);
          startScoop = currentMillis;
      }
      if(angle <= MINANG){
        scooping = 3;
      }
    }
}

int sign(int x) {
  if (x > 0) {
    return 1;
  } else if (x < 0) {
    return -1;
  } else {
    return 0;
  }
}


int CheckPWM(int PWM_val)
{
    if (abs(PWM_val) < PWM_min){
        return int(sign(PWM_val) * PWM_min);
    }
    if (abs(PWM_val) > PWM_max){
        return int(sign(PWM_val) * PWM_max);
    }
    return PWM_val;
}

void WriteDriverVoltageA(int PWM_val)
{
    if (PWM_val > 0){
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else if (PWM_val < 0){
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    else{
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
    }
    analogWrite(ENA, abs(PWM_val));
}
void WriteDriverVoltageB(int PWM_val)
{
    if (PWM_val < 0){
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    }
    else if (PWM_val > 0){
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
    }
    else{
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
    }
    analogWrite(ENB, abs(PWM_val));
}

void ISR_EncoderA2() {
  bool PinB = digitalRead(AC2);
  bool PinA = digitalRead(AC1);

  if (PinB == LOW) {
    if (PinA == HIGH) {
      EncoderCountA++;
    }
    else {
      EncoderCountA--;
    }
  }

  else {
    if (PinA == HIGH) {
      EncoderCountA--;
    }
    else {
      EncoderCountA++;
    }
  }
}

void ISR_EncoderA1() {
  bool PinB = digitalRead(AC2);
  bool PinA = digitalRead(AC1);

  if (PinA == LOW) {
    if (PinB == HIGH) {
      EncoderCountA--;
    }
    else {
      EncoderCountA++;
    }
  }

  else {
    if (PinB == HIGH) {
      EncoderCountA++;
    }
    else {
      EncoderCountA--;
    }
  }
}

void ISR_EncoderB2() {
  bool PinB = digitalRead(BC2);
  bool PinA = digitalRead(BC1);

  if (PinA == LOW) {
    if (PinB == HIGH) {
      EncoderCountB++;
    } else {
      EncoderCountB--;
    }
  } else {
    if (PinB == HIGH) {
      EncoderCountB--;
    } else {
      EncoderCountB++;
    }
  }
}

void ISR_EncoderB1() {
  bool PinB = digitalRead(BC2);
  bool PinA = digitalRead(BC1);

  if (PinA == LOW) {
    if (PinB == HIGH) {
      EncoderCountB--;
    } else {
      EncoderCountB++;
    }
  } else {
    if (PinB == HIGH) {
      EncoderCountB++;
    } else {
      EncoderCountB--;
    }
  }
}

// Function to get a specific value from a comma-separated string
String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i + 1 : i;
        }
    }

    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void setup()
{
    Serial.begin(9600);
    pinMode(AC1, INPUT_PULLUP);
    pinMode(AC2, INPUT_PULLUP);
    pinMode(BC1, INPUT_PULLUP);
    pinMode(BC2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(AC1), ISR_EncoderA1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(AC2), ISR_EncoderA2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BC1), ISR_EncoderB1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BC2), ISR_EncoderB2, CHANGE);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    servo.attach(servoPin, 771, 2740);
    servo.write(angle);
}

void loop()
{
    if (Serial.available() > 0)
    {
        String data = Serial.readStringUntil('\n');
        Serial.print("You sent me: ");

        // Convert the string values to integers
        PWM_A_val = getValue(data, ',', 0).toInt();
        PWM_B_val = getValue(data, ',', 1).toInt();
        Serial.print(PWM_A_val);
        Serial.print(',');
        Serial.println(PWM_B_val);
    }
    
    if(scooping == 0){
      WriteDriverVoltageA(PWM_A_val);
      WriteDriverVoltageB(PWM_B_val);
      checkDistance();
      if(distance < 9 || distance > 30){
        scooping = 1;
      }
    } else{
      WriteDriverVoltageA(0);
      WriteDriverVoltageB(0);
      scoop();
    }
}
