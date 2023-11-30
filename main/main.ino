#include <Servo.h> //Imports the library Servo
#include <TimerOne.h>

#define trigPin 3 // TriggerSensor
#define echoPin 2 // EchoSensor

#define ServoPin 8 // Servo pin
#define MAXANG 180 // Servo máx angle
#define MINANG 0 // Servo min angle
#define SCOOPDELAY 15

#define ENA 6 // D6
#define ENB 13

#define AIN1 4 // D1
#define AIN2 5  // D0

#define BIN1 10
#define BIN2 9

#define AC1 21 // D4
#define AC2 20 // D3

#define BC1 19
#define BC2 18

#define redLed 22

#define baud 9600


Servo servo;   // Defines the object Servo of type(class) Servo
int angle = MINANG; // Defines an integer


char msg[40];
float scoop_debounce = 1000;
float last_scoop = 0;
int r = 32.5; // mm of wheel
int ratio_ruedas = 34.02;
int steps = 11;

unsigned long init_time = 0;
unsigned long time = 0;
unsigned long time_ = 0;
float delta_time = 0.1; //s
const int Period = 10000; // 10 ms = 100Hz
//const int Debounce = 500000; // 500ms
//unsigned long pressed_time = 0;

volatile bool scoopFlag = false; 


volatile long encoderAPos = 0;
volatile long encoderBPos = 0;
volatile long encoderAPos_ = 0;
volatile long encoderBPos_ = 0;

float velA; // [RPM]
float velB; // [RPM]
float error_velA;
float error_velB;
float error_velA_;
float error_velB_;
float error_velA__;
float error_velB__;
volatile float PWM_A;
volatile float PWM_B;
float PWM_A_;
float PWM_B_;

volatile int right_rpm_value;
volatile int left_rpm_value;
int right_dir_value;
int left_dir_value;

float Kp_L = 10;
float Ki_L = 0.01;
float Kd_L = 0.01;
float Kp_R = 10;
float Ki_R = 0.01;
float Kd_R = 0.01;


<<<<<<< HEAD
void callback()
{
    Serial.print("enc");
    Serial.print(encoderAPos);
    Serial.print(",");
    Serial.println(encoderBPos);
    velA = float(encoderAPos - encoderAPos_)/ delta_time / 60.0;
    velB = float(encoderBPos - encoderBPos_)/ delta_time / 60.0;
    error_velA__ = error_velA_;
    error_velB__ = error_velB_;
    error_velA_ = error_velA;
    error_velB_ = error_velB;
    error_velA = left_rpm_value - velA;
    error_velB = right_rpm_value - velB;
    PWM_A_ = PWM_A;
    PWM_B_ = PWM_B;
    PWM_A = (PWM_A_ 
               + (Kp_L + delta_time*Ki_L + Kd_L/delta_time) * error_velA
               + (-Kp_L - 2*Kd_L/delta_time) * error_velA_
               + (Kd_L/delta_time) * error_velA__);
    PWM_B = (PWM_B_ 
               + (Kp_R + delta_time*Ki_R + Kd_R/delta_time) * error_velB
               + (-Kp_R - 2*Kd_R/delta_time) * error_velB_
               + (Kd_R/delta_time) * error_velB__);
    encoderAPos_ = encoderAPos;
    encoderBPos_ = encoderBPos;
    Serial.print("velA: ");
    Serial.print(velA);
    Serial.print("velB: ");
    Serial.println(velB);
=======

void callback() {
  PID_L(&left_rpm_value, &encoderAPos_, &velA, &error_velA, &error_velA_, &error_velA__, &PWM_A, &PWM_A_); 
  Serial.print(", ");
  PID_R(&right_rpm_value, &encoderBPos_, &velB, &error_velB, &error_velB_, &error_velB__, &PWM_B, &PWM_B_); 
  Serial.println();
}


void PID_L(int *left_rpm_value, long *encoderAPos_, float *velA,
           float *error_velA, float *error_velA_, float *error_velA__, float *PWM_A, float *PWM_A_){      
  *velA = (encoderAPos- *encoderAPos_) / delta_time * 60 / steps;                
  //*velA = (float)(encoderAPos - *encoderAPos_) * (1.6033);                
  //*velA = (float)(encoderAPos - *encoderAPos_) * 60 / (steps * ratio_ruedas * delta_time);
  Serial.print("encoderAPos: ");
  Serial.print(encoderAPos);
  Serial.print(" | encoderAPos_: ");
  Serial.print(*encoderAPos_);
  *encoderAPos_ = encoderAPos;
  *error_velA__ = *error_velA_;
  *error_velA_ = *error_velA;
  *error_velA = *left_rpm_value - *velA;
  *PWM_A_ = *PWM_A;
  *PWM_A = (*PWM_A_ 
               + (Kp_L + delta_time*Ki_L + Kd_L/delta_time) * (*error_velA)
               + (-Kp_L - 2*Kd_L/delta_time) * (*error_velA_)
               + (Kd_L/delta_time) * (*error_velA__));
  if (*PWM_A > 250.0) {
    *PWM_A = 250.0;
  } else if (*PWM_A < -250.0) {
    *PWM_A = -250.0;
  }
  analogWrite(ENA, abs(int(round(*PWM_A))));
  Serial.print(" | velA: ");
  Serial.print(*velA);
  Serial.print(" | PWM_A: ");
  Serial.print(abs(int(round(*PWM_A))));
}

void PID_R(int *right_rpm_value, long *encoderBPos_, float *velB,
           float *error_velB, float *error_velB_, float *error_velB__, float *PWM_B, float *PWM_B_){      
  *velB = (encoderBPos - *encoderBPos_) / delta_time * 60 / steps;                             
  //*velB = (float)(encoderBPos - *encoderBPos_) * (1.6033);
  //*velB = (float)(encoderBPos - *encoderBPos_) * 60 / (steps * ratio_ruedas * delta_time);
  *encoderBPos_ = encoderBPos;
  *error_velB__ = *error_velB_;
  *error_velB_ = *error_velB;
  *error_velB = *right_rpm_value - *velB;
  *PWM_B_ = *PWM_B;
  *PWM_B = (*PWM_B_ 
               + (Kp_R + delta_time*Ki_R + Kd_R/delta_time) * (*error_velB)
               + (-Kp_R - 2*Kd_R/delta_time) * (*error_velB_)
               + (Kd_R/delta_time) * (*error_velB__));
  if (*PWM_B > 250.0) {
    *PWM_B = 250.0;
  } else if (*PWM_B < -250.0) {
    *PWM_B = -250.0;
  }
  analogWrite(ENB, abs(int(round(*PWM_B))));
  Serial.print(" | velB: ");
  Serial.print(*velB);
  Serial.print(" | PWM_B: ");
  Serial.print(abs(int(round(*PWM_B))));
>>>>>>> e013f8077e1b88ddeda8f89027be55abe4b91f3c
}



void ISR_EncoderA2() {
  bool PinB = digitalRead(AC2);
  bool PinA = digitalRead(AC1);

  if (PinB == LOW) {
    if (PinA == HIGH) {
      EncoderACount++;
    }
    else {
      EncoderACount--;
    }
  }

  else {
    if (PinA == HIGH) {
      EncoderACount--;
    }
    else {
      EncoderACount++;
    }
  }
}

void ISR_EncoderA1() {
  bool PinB = digitalRead(AC2);
  bool PinA = digitalRead(AC1);

  if (PinA == LOW) {
    if (PinB == HIGH) {
      EncoderACount--;
    }
    else {
      EncoderACount++;
    }
  }

  else {
    if (PinB == HIGH) {
      EncoderACount++;
    }
    else {
      EncoderACount--;
    }
  }
}


void doEncoderB1()
{
    if (digitalRead(BC1) == digitalRead(BC2))
    {
        encoderBPos++;
    }
    else
    {
        encoderBPos--;
    }
}

void doEncoderB2()
{
    if (digitalRead(BC1) == digitalRead(BC2))
    {
        encoderBPos--;
    }
    else
    {
        encoderBPos++;
    }
}


// ************** Función para mover la pala ***************
void scoop()
{
    Serial.println("pala");
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
    Serial.println("scoopedLol");
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

void stringSplitter(char *msg, int *right_rpm, int *right_dir, int *left_rpm, int *left_dir)
{
  char *token = strtok(msg, ",");
  for (int i = 0; i < 4; i++) {
    int intValue = atoi(token);
    switch (i) {
      case 0:
        *right_rpm = intValue;
        break;
      case 1:
        *right_dir = intValue;
        break;
      case 2:
        *left_rpm = intValue;
        break;
      case 3:
        *left_dir = intValue;
        break;
    }
    token = strtok(NULL, ",");
  }
}


void checkDistance()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;

  Serial.println(distance);
  if (distance < 8) {
    scoop();
    delay(scoop_debounce);
  }
}


void setup() {
    digitalWrite(redLed, LOW);

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    pinMode(ENA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    pinMode(redLed, OUTPUT);
   
    pinMode(AC1, INPUT_PULLUP);
    digitalWrite(AC1, HIGH);
    pinMode(AC2, INPUT_PULLUP);
    digitalWrite(AC2, HIGH);

    pinMode(BC1, INPUT_PULLUP);
    digitalWrite(BC1, HIGH);
    pinMode(BC2, INPUT_PULLUP);
    digitalWrite(BC2, HIGH);

    attachInterrupt(digitalPinToInterrupt(AC1), doEncoderA1, CHANGE); // encoder 0 PIN A
    attachInterrupt(digitalPinToInterrupt(AC2), doEncoderA2, CHANGE); // encoder 0 PIN B

    attachInterrupt(digitalPinToInterrupt(BC1), doEncoderB1, CHANGE); // encoder 0 PIN A
    attachInterrupt(digitalPinToInterrupt(BC2), doEncoderB2, CHANGE); // encoder 0 PIN B

    Serial.begin(baud);

    Timer1.initialize(100000); // initialize timer1, and set a 0.1 second period
    Timer1.attachInterrupt(callback); // attaches callback() as a timer overflow interrupt

    init_time = millis();
}


void loop() {
  time = millis();

  readSerialPort();
<<<<<<< HEAD
  if (strcmp(msg, manual) == 0) {
    auto_mode = false;
  }
  else if (strcmp(msg, automatic) == 0) {
    auto_mode = true;
  }

  if(auto_mode)  {
    if(msg[0] != '\0' && msg[0] != ' ' && msg != NULL) {
      stringSplitter(msg, &right_rpm_value, &right_dir_value, &left_rpm_value, &left_dir_value);
      
      if(left_dir_value == -1){
        digitalWrite(redLed, HIGH);
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
      }
      else {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
      }
      analogWrite(ENA, int(0));

      if(right_dir_value == 1){
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
      }
      else {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
      }
      analogWrite(ENB, int(0));

=======
  if(msg[0] != '\0' && msg[0] != ' ' && msg != NULL) {
    Serial.println(msg);
    stringSplitter(msg, &right_rpm_value, &right_dir_value, &left_rpm_value, &left_dir_value);
    if(left_dir_value == -1){
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
>>>>>>> e013f8077e1b88ddeda8f89027be55abe4b91f3c
    }
    else {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
    }
    if(right_dir_value == 1){
      digitalWrite(redLed, HIGH);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
    }
    else {
      digitalWrite(redLed, LOW);
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
    }
  }
  msg[0] = '\0';
}
