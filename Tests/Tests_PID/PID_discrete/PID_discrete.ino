#include <Servo.h> //Imports the library Servo
//GeeKee CeeBee
#define ServoPin 8
#define trigPin 3 // TriggerSensor
#define echoPin 2 // EchoSensor

#define ServoPin 8 // Servo pin
#define MAXANG 180 // Servo máx angle
#define MINANG 0 // Servo min angle
#define SCOOPDELAY 15

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

#define baud 9600

#define pi 3.1416

Servo servo; //Defines the object Servo of type(class) Servo
int angle = 0; // Defines an integer
// ************ DEFINITIONS A************
float kp_A = 0.02;
float ki_A = 0.00005 ;
float kd_A = 0.0001;

// ************ DEFINITIONS A************
float kp_B = 0.02;
float ki_B = 0.0000 ;
float kd_B = 0.000;

unsigned long t = 0;
unsigned long t_prev = 0;

volatile long EncoderCountA = 0;
volatile long EncoderCountB = 0;
//--------------------------- Pines---------------------------

volatile unsigned long count = 0;
unsigned long count_prev = 0;

float ThetaA, ThetaB;
float ThetaA_prev, ThetaB_prev;
float RPM_A, RPM_B;
float RPM_A_ref, RPM_B_ref;
float NFactor = 1500;

float e_A, e_A_, e_A__;
float e_B, e_B_, e_B__;

int dt;

int PWM_A_val, PWM_B_val;
int PWM_A_val_, PWM_B_val_;
int PWM_min = 150;
int PWM_max = 255;

char msg[60];
//***Motor Driver Functions*****

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

void readSerialPort()
{
  memset(msg, 0, sizeof(msg));  // Clear the msg array
  if (Serial.available()) {
    delay(20);
    int i = 0;
    while (Serial.available() > 0 && i < sizeof(msg) - 1) {
      msg[i++] = Serial.read();
    }
    Serial.flush();
  }
}

void stringSplitter(char *msg, float *left_rpm, float *right_rpm) {
  char *token = strtok(msg, ",");
  for (int i = 0; i < 2; i++) {
    float floatValue = atof(token); // Use atof for floating-point values
    switch (i) {
      case 0:
        *left_rpm = floatValue;
        break;
      case 1:
        *right_rpm = floatValue;
        break;
    }
    token = strtok(NULL, ",");
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

void setup() {
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
  servo.attach(ServoPin); // States that the servo is attached to pin 5
  servo.write(angle);

}

void loop() {
  readSerialPort();
  if(msg[0] != '\0' && msg[0] != ' ' && msg != NULL) {
    stringSplitter(msg, &RPM_A_ref, &RPM_B_ref);
    msg[0] == '\0';
  }
  if ((millis() - t_prev)>= 100) {
      t = millis();
      ThetaA = EncoderCountA;
      ThetaB = EncoderCountB;
      dt = t - t_prev; // [s]
      RPM_A = 1000 * (ThetaA - ThetaA_prev)/ dt * 60 / NFactor;
      RPM_B = 1000 * (ThetaB - ThetaB_prev)/ dt * 60 / NFactor;
      e_A__ = e_A_;
      e_B__ = e_B_;
      e_A_ = e_A;
      e_B_ = e_B;
      e_A = RPM_A_ref - RPM_A;
      e_B = RPM_B_ref - RPM_B;
      PWM_A_val_ = PWM_A_val;
      PWM_B_val_ = PWM_B_val;
      PWM_A_val = int(PWM_A_val_ + kp_A*(1 + dt*ki_A/1000 + 1000*kd_A/dt)*e_A + -kp_A*(1 + 2000*kd_A/dt)*e_A_ + (1000*kp_A*kd_A/dt)*e_A__);
      PWM_B_val = int(PWM_B_val_ + kp_B*(1 + dt*ki_B/1000 + 1000*kd_B/dt)*e_B + -kp_B*(1 + 2000*kd_B/dt)*e_B_ + (1000*kp_B*kd_B/dt)*e_B__);
      PWM_A_val = CheckPWM(PWM_A_val);
      PWM_B_val = CheckPWM(PWM_B_val);
      WriteDriverVoltageA(PWM_A_val);
      WriteDriverVoltageB(PWM_B_val);

      Serial.print(count * 0.05);
      Serial.print(", ");
      Serial.print("refA: ");
      Serial.print(RPM_A_ref);
      // Serial.print("EncoderCountA: ");
      // Serial.print(EncoderCountA);
      Serial.print(" | RPMA: ");
      Serial.print(RPM_A);
      Serial.print(", ");

      Serial.print("refB: ");
      Serial.print(RPM_B_ref);
      // Serial.print("EncoderCountB: ");
      // Serial.print(EncoderCountB);
      Serial.print(" | RPM_B: ");
      Serial.print(RPM_B);
      Serial.println("");

      ThetaA_prev = ThetaA;
      ThetaB_prev = ThetaB;
      count_prev = count;
      t_prev = t;
  
  }
}
