#include <Servo.h> //Imports the library Servo
//GeeKee CeeBee
#define trigPin 3 // TriggerSensor
#define echoPin 2 // EchoSensor

#define ServoPin 38 // Servo pin
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


byte buffer[3];
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

void readSerialPort() {
  if (Serial.available() > 0) {
    String msga= Serial.readStringUntil('\n');
    Serial.println(msga);
  }
}


void setup() {
  Serial.begin(baud);
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
  servo.attach(ServoPin); // States that the servo is attached to pin 5
  servo.write(angle);

}

void loop() {
  readSerialPort();
  if(scooping == 0){
    if(distance < 9){
      distance = 12;
      scooping = 1;
      startScoop = millis();
    }
  }
  
  if(scooping == 1 || scooping == 2){
    scoop();
  }
  
  if(scooping == 3){
    if(millis()-startScoop >= 1600){
      scooping = 0;
    }
  }
  if ((millis() - t_prev)>= 50) {
      if(scooping == 0){
        //checkDistance();
      }
      t = millis();
      ThetaA = EncoderCountA;
      ThetaB = EncoderCountB;
      Dist_A = Dist_A + ((ThetaA - ThetaA_prev)) / NFactor * pi * Diam_ruedas;
      Dist_B = Dist_B + ((ThetaB - ThetaB_prev)) / NFactor * pi * Diam_ruedas;
      dt = t - t_prev;
      RPM_A = 1000 * (ThetaA - ThetaA_prev)/ dt * 60.0 / NFactor;
      RPM_B = 1000 * (ThetaB - ThetaB_prev)/ dt * 60.0 / NFactor;
      if(scooping == 0){
        WriteDriverVoltageA(PWM_A_val);
        WriteDriverVoltageB(PWM_B_val);
      }
      else{
        WriteDriverVoltageA(0);
        WriteDriverVoltageB(0);
      }

      vel_A = RPM_A * pi * 2 / 60.0;
      vel_B = RPM_B * pi * 2 / 60.0;
      Vel_ang = R_ruedas * (vel_B - vel_A) / L_robot;
      Theta = Theta + Vel_ang*dt/1000;
      Vel_lin = R_ruedas * (vel_A + vel_B) / 2;
      Vel_x = Vel_lin * cos(Theta);
      Vel_y = Vel_lin * sin(Theta);
      Pos_x = Pos_x + (Vel_x*dt)/1000;
      Pos_y = Pos_y + (Vel_y*dt)/1000;

      Serial.print("PWMA: ");
      Serial.print(PWM_A_val);

      Serial.print("PWMB: ");
      Serial.println(PWM_B_val);
      
      //Serial.print("POSX: ");
      //Serial.print(Pos_x);
      //Serial.print("POSY: ");
      //Serial.print(Pos_y);
      //Serial.print("Theta");
      //sSerial.println(Theta);

      
      ThetaA_prev = ThetaA;
      ThetaB_prev = ThetaB;
      t_prev = t;
  }
}
