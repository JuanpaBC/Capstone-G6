#include <Servo.h> //Imports the library Servo
#include <HCSR04.h>

#define trigPin 3 // TriggerSensor
#define echoPin 2 // EchoSensor

#define servoPin 12 // Servo pin
#define MAXANG 180 // Servo máx angle
#define MINANG 0 // Servo min angle
#define SCOOPDELAY 5

#define ENA 6
#define ENB 11

#define AIN1 4
#define AIN2 5
#define BIN1 10
#define BIN2 9

#define AC1 21
#define AC2 20
#define BC1 19
#define BC2 18

#define baud 9600

#define pi 3.1415

Servo servo; //Defines the object Servo of type(class) Servo
HCSR04 hc(trigPin, echoPin);
int angle = 0; // Defines an integer

// **** DEFINITIONS A******
volatile long EncoderCountA = 0;
float ThetaA_prev, ThetaB_prev;
float RPM_A;
float vel_A;
int PWM_A_val;
float Dist_A;
// **** DEFINITIONS B******
volatile long EncoderCountB = 0;
float ThetaA, ThetaB;
float RPM_B;
float vel_B;
int PWM_B_val;
float Dist_B;

// ************ DEFINITIONS A************
float kp_A = 1.20;
float ki_A = 0.02;
float kd_A = 10.6;

// ************ DEFINITIONS A************
float kp_B = 1;
float ki_B = 0.01 ;
float kd_B = 10;


float e_A, e_prev_A;
float inte_A, inte_prev_A;
float e_B, e_prev_B;
float inte_B, inte_prev_B;
int dt;

int RPM_A_ref = 0;
int RPM_B_ref = 0;
float Pos_x, Pos_y;
float Vel_lin, Vel_x, Vel_y;
float Vel_ang, Theta;

unsigned long t, t_prev, lastScoopMillestone;

float NFactor = 1400;
int PWM_min = 150;
int PWM_max = 255;

char msg[60];
float Diam_ruedas = 0.1;
float R_ruedas = Diam_ruedas/2;
float L_robot = 0.370-0.055;

int instruction = -1;

int duration;
int distance;

unsigned long startScoop = 0;
unsigned long currentMillis = 0;

int agarro_castana = 0;
int scooping = 0;


void scoop() {
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
      if (currentMillis - lastScoopMillestone >= 2000) {
        distance = 12;
        scooping = 0;
      }
    }
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
    analogWrite(ENA, min(abs(PWM_val), 250));
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
    analogWrite(ENB, min(abs(PWM_val), 250));
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
int sign(int x) {
  if (x > 0) {
    return 1;
  } else if (x < 0) {
    return -1;
  } else {
    return 0;
  }
}
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
  digitalWrite(trigPin, LOW);
  servo.attach(servoPin, 400, 2740);
  servo.write(angle);
}
void loop() {
  if (Serial.available() > 0)
  {
      String data = Serial.readStringUntil('\n');

      // Convert the string values to integers
      RPM_A_ref = getValue(data, ',', 0).toInt();
      RPM_B_ref = getValue(data, ',', 1).toInt();

      Serial.print(t);
      Serial.print(", ");
      Serial.print("PosX: ");
      Serial.print(Pos_x);
      Serial.print(", ");

      Serial.print("POSY: ");
      Serial.print(Pos_y);
      //Serial.print("POSY: ");
      //Serial.print(Pos_y);
      Serial.print(", Theta");
      Serial.println(Theta*180/pi);
  }
  if(scooping == 0){
    if ((millis() - t_prev)>= 100) {
        distance = hc.dist();
        
        if(distance < 9 || distance > 30){
          distance = 12;
          scooping = 1;
        }
        t = millis();
        ThetaA = EncoderCountA;
        ThetaB = EncoderCountB;
        Dist_A = Dist_A + ((ThetaA - ThetaA_prev)) / NFactor * pi * Diam_ruedas;
        Dist_B = Dist_B + ((ThetaB - ThetaB_prev)) / NFactor * pi * Diam_ruedas;
        dt = t - t_prev;
        RPM_A = 1000 * (ThetaA - ThetaA_prev)/ dt * 60.0 / NFactor;
        RPM_B = 1000 * (ThetaB - ThetaB_prev)/ dt * 60.0 / NFactor;
        e_A = RPM_A_ref - RPM_A;
        e_B = RPM_B_ref - RPM_B;
        inte_A = inte_prev_A + (dt * (e_A + e_prev_A) / 2);
        inte_B = inte_prev_B + (dt * (e_B + e_prev_B) / 2);
        PWM_A_val = int(kp_A * e_A + ki_A * inte_A + (kd_A * (e_A - e_prev_A) / dt));
        PWM_B_val = int(kp_B * e_B + ki_B * inte_B + (kd_B * (e_B - e_prev_B) / dt));
        if(Pos_x < 3){
          WriteDriverVoltageA(PWM_A_val);
          WriteDriverVoltageB(PWM_B_val);
        }
  
        vel_A = RPM_A * pi * 2 / 60.0;// [rad/s]
        vel_B = RPM_B * pi * 2 / 60.0; // [rad/s]
        Vel_ang = R_ruedas * (vel_B - vel_A) / L_robot;
        Theta = Theta + Vel_ang*dt/1000;
        Vel_lin = R_ruedas * (vel_A + vel_B) / 2;
        Vel_x = Vel_lin * cos(Theta);
        Vel_y = Vel_lin * sin(Theta);
        Pos_x = Pos_x + Vel_x*dt/1000;
        Pos_y = Pos_y + Vel_y*dt/1000;
  
        
        ThetaA_prev = ThetaA;
        ThetaB_prev = ThetaB;
        t_prev = t;
    }
  } else{
    WriteDriverVoltageA(0);
    WriteDriverVoltageB(0);
    scoop();
    inte_A = 0;
    inte_B = 0;
    ThetaA_prev = EncoderCountA;
    ThetaB_prev = EncoderCountB;
    t_prev = millis();
    
  }
}
