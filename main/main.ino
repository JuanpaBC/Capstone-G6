#include <Servo.h> //Imports the library Servo
//GeeKee CeeBee
#define trigPin 3 // TriggerSensor
#define echoPin 2 // EchoSensor

#define ServoPin 8 // Servo pin
#define MAXANG 180 // Servo máx angle
#define MINANG 0 // Servo min angle
#define SCOOPDELAY 1

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

#define pi 3.1415

Servo servo; //Defines the object Servo of type(class) Servo
int angle = 0; // Defines an integer

// ************ DEFINITIONS A************
float kp_A = 1.55;
float ki_A = 0.001;
float kd_A = 0.0005;
volatile long EncoderCountA = 0;
float ThetaA_prev, ThetaB_prev;
float RPM_A, RPM_A_ref;
float vel_A;
float e_A, e_prev_A;
float inte_A, inte_prev_A;
int PWM_A_val;
float Dist_A;
// ************ DEFINITIONS B************
float kp_B = 1.58;
float ki_B = 0.001 ;
float kd_B = 0.0005;
volatile long EncoderCountB = 0;
float ThetaA, ThetaB;
float RPM_B, RPM_B_ref;
float vel_B;
float e_B, e_prev_B;
float inte_B, inte_prev_B;
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

int explorer_mode = 1;
int instruction = -1;
float largo = 3.0;
int advance = 0;
int state = 0;

int duration;
int distance;

int scooping = 0;
unsigned long startScoop = 0;
unsigned long currentMillis = 0;
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
  // Serial.print("Distance = ");
  // Serial.print(distance);
  // Serial.println(" cm");
}

void scoop() {
    // The following loop runs until the servo is turned to 180 degrees
    // Serial.println("scooping");
    if(scooping == 1){
      currentMillis = millis();
      if (currentMillis - startScoop >= SCOOPDELAY) {
          servo.write(angle);
          angle++;
          angle++;
          startScoop = currentMillis;
      }
      if(angle >= MAXANG){
        scooping = 2;
      }
    }
    if(scooping == 2){
      currentMillis = millis();
      if (currentMillis - startScoop >= SCOOPDELAY) {
          servo.write(angle);
          angle--;
          angle--;
          startScoop = currentMillis;
      }
      if(angle <= MINANG){
        scooping = 3;
      }
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

void stringSplitter(char *msg, int *instruction, float *left_rpm, float *right_rpm) {
  char *token = strtok(msg, ",");
  for (int i = 0; i < 2; i++) {
    int intValue = 0;
    float floatValue = 0.0;
    if(i == 0){
      intValue = atoi(token);
    }
    else{
      floatValue = atof(token); // Use atof for floating-point values
    }
    if(*instruction == 1) break;
    switch (i) {
      case 0:
        *instruction = intValue;
        break;
      case 1:
        *left_rpm = floatValue;
        break;
      case 2:
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
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  servo.attach(ServoPin); // States that the servo is attached to pin 5
  servo.write(angle);

}

void loop() {
  readSerialPort();
  if(msg[0] != '\0' && msg[0] != ' ' && msg != NULL) {
    stringSplitter(msg, &instruction, &RPM_A_ref, &RPM_B_ref);
    msg[0] == '\0';
  }
  if(instruction == 0){
    explorer_mode = 0;
  }
  else if(instruction == 1){
    explorer_mode = 1;
  }
  if(scooping == 0){
    checkDistance();
    if(distance < 9){
      scooping = 1;
      startScoop = millis();
    }
  }
  
  if(scooping == 1 || scooping == 2){
    scoop();
  }
  
  if(scooping == 3){
    if(millis()-startScoop >= 1000){
      scooping = 0;
    }
  }
  if(explorer_mode  == 1){
    
    if(state == 0){
      RPM_A_ref = 200;
      RPM_B_ref = 200;
      if ((Dist_A+Dist_B)/2 >= largo){
        state = 1;
      }
    }
    else if(state == 1){
      RPM_A_ref = 0;
      RPM_B_ref = 0;
    }
    /*else if(state == 1){
      RPM_A_ref = 200;
      RPM_B_ref = 0;
      if (Dist_A >= 2*largo){
        state = 2;
      }
    }
    else if(sstate == 2){
      RPM_A_ref = 0;
      RPM_B_ref = 0;
      if (Dist_A >= 3*largo){
        state = 3;
      }
    }
    else if(state == 3){
      RPM_A_ref = -100;
      RPM_B_ref = -100;
      if (Dist_A >= 4*largo){
        state = 4;
      }
    }
    else if(state == 4){
      RPM_A_ref = 0;
      RPM_B_ref = 0;
      if (Dist_A >= 5*largo){
        state = 5;
      }
    }
    else if(state == 5){
      RPM_A_ref = 200;
      RPM_B_ref = 200;
      if (Dist_A >= 6*largo){
        state = 6;
      }
    }
    else if(state == 6){
      RPM_A_ref = 200;
      RPM_B_ref = 0;
      if (Dist_A >= 7*largo){
        state = 7;
      }
    }
    else if(state == 7){
      RPM_A_ref = 0;
      RPM_B_ref = 0;
      if (Dist_A >= 8*largo){
        state = 8;
      }
    }
    else if(state == 8){
      RPM_A_ref = -100;
      RPM_B_ref = -100;
      if (Dist_A >= 9*largo){
        state = 9;
      }
    }
    else if(state == 9){
      RPM_A_ref = 0;
      RPM_B_ref = 0;
      if (Dist_A >= 10*largo){
        state = 10;
      }
    }
    else if(state == 10){
      RPM_A_ref = 200;
      RPM_B_ref = 200;
      if (Dist_A >= 11*largo){
        state = 11;
      }
    }
    else if(state == 11){
      RPM_A_ref = 200
    }*/
  }
  if ((millis() - t_prev)>= 100) {
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
      //PWM_A_val = CheckPWM(PWM_A_val);
      //PWM_B_val = CheckPWM(PWM_B_val);
      WriteDriverVoltageA(PWM_A_val);
      WriteDriverVoltageB(PWM_B_val);

      vel_A = RPM_A * pi * 2 / 60.0;
      vel_B = RPM_B * pi * 2 / 60.0;
      Vel_ang = R_ruedas * (vel_B - vel_A) / L_robot;
      Theta = Theta + Vel_ang*dt/1000;
      Vel_lin = R_ruedas * (vel_A + vel_B) / 2;
      Vel_x = Vel_lin * cos(Theta);
      Vel_y = Vel_lin * sin(Theta);
      Pos_x = Pos_x + (Vel_x*dt)/1000;
      Pos_y = Pos_y + (Vel_y*dt)/1000;

      Serial.print(t);
      Serial.print(", ");
      Serial.print("refA: ");
      Serial.print(RPM_A_ref);
      Serial.print(" | EncoderCountA: ");
      Serial.print(EncoderCountA);
      Serial.print(" | RPMA: ");
      Serial.print(RPM_A);
      Serial.print(", ");
      
      

      Serial.print("refB: ");
      Serial.print(RPM_B_ref);
      Serial.print(" | EncoderCountB: ");
      Serial.print(EncoderCountB);
      Serial.print(" | RPM_B: ");
      Serial.print(RPM_B);
      Serial.println("");

      //Serial.print("POSX: ");
      //Serial.print(Pos_x);
      //Serial.print("POSY: ");
      //Serial.print(Pos_y);
      //Serial.print("Theta");
      //sSerial.println(Theta);

      
      ThetaA_prev = ThetaA;
      ThetaB_prev = ThetaB;
      t_prev = t;
      inte_prev_A = inte_A;
      inte_prev_B = inte_B;
      e_prev_A = e_A;
      e_prev_B = e_B;
  }
}