#include <Servo.h> //Imports the library Servo
//GeeKee CeeBee
#define trigPin 3 // TriggerSensor
#define echoPin 2 // EchoSensor

#define ServoPin 8 // Servo pin
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
int angle = 0; // Defines an integer


// ************ DEFINITIONS A (L)************
volatile long EncoderCountA = 0;
float ThetaA, ThetaA_prev;
float Dist_A, vel_A, RPM_A;
int PWM_A_val;
float kp_A = 1.24;
float ki_A = 0.02;
float kd_A = 10.8;

// ************ DEFINITIONS B (R)************
volatile long EncoderCountB = 0;
float ThetaB, ThetaB_prev;
float Dist_B, vel_B, RPM_B;
int PWM_B_val;
float kp_B = 1;
float ki_B = 0.01 ;
float kd_B = 10;

// ************ VARIABES PID RPM************
float e_A, e_prev_A;
float inte_A, inte_prev_A;
float e_B, e_prev_B;
float inte_B, inte_prev_B;
int RPM_A_ref, RPM_B_ref;



// ************ VARIABES PID POSE************
float Pose_X, Pose_Y, Pose_Theta, Pose_Theta_ref;
float Pose_X_final = 0;
float Pose_Y_final = 0;
float Pose_Theta_final = pi/2;
int state_pid_pose = 0;

// ************ VARIABES PID ROT************
float kp_giro = 1;
float ki_giro = 0.1;
float e_giro, e_prev_giro, inte_giro, inte_prev_giro;
float Vel_Rot_ref, RPM_Rot_ref;
float e_giro_margin = 0.01;
float e_giro_margin_factor = 1.1;

// ************ VARIABES PID DESP************
float kp_desp = 0.05;
float ki_desp = 1;
float e_desp, e_prev_desp, inte_desp, inte_prev_desp;
float Vel_Desp_ref, RPM_Desp_ref;
float e_desp_margin = 0.05;


// ************ ODOMETRIA************
float Pos_x, Pos_y, Theta;
float Vel_ang, Vel_lin, Vel_x, Vel_y;

// ************ TIEMPO************
int dt;
unsigned long t, t_prev;


char msg[60];

// PARAMETROS FISICOS ROBOT
float Diam_ruedas = 0.09;
float R_ruedas = Diam_ruedas/2;
float L_robot = 0.370-0.055;
float NFactor = 1400;
int PWM_min = 150;
int PWM_max = 255;

int instruction = -1;

int duration;
int distance;

unsigned long startScoop = 0;
unsigned long currentMillis = 0;

int agarro_castana = 0;
int scooping = 0;


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
  state_pid_pose = 0;
}
void loop() {
  if ((millis() - t_prev)>= 100) {
      t = millis();
      ThetaA = EncoderCountA;
      ThetaB = EncoderCountB;
      Dist_A = Dist_A + ((ThetaA - ThetaA_prev)) / NFactor * pi * Diam_ruedas;
      Dist_B = Dist_B + ((ThetaB - ThetaB_prev)) / NFactor * pi * Diam_ruedas;
      dt = t - t_prev;
      RPM_A = 1000 * (ThetaA - ThetaA_prev)/ dt * 60.0 / NFactor;
      RPM_B = 1000 * (ThetaB - ThetaB_prev)/ dt * 60.0 / NFactor;
      vel_A = RPM_A * pi * 2 / 60.0;  // [rad/s]
      vel_B = RPM_B * pi * 2 / 60.0;  // [rad/s]
      Vel_ang = R_ruedas * (vel_B - vel_A) / L_robot;  // [rad/s]
      Theta = Theta + Vel_ang*dt/1000;  // [rad]
      Vel_lin = R_ruedas * (vel_A + vel_B) / 2; // [m/s]
      Vel_x = Vel_lin * cos(Theta); // [m/s]
      Vel_y = Vel_lin * sin(Theta); // [m/s]
      Pos_x = Pos_x + (Vel_x*dt)/1000; // [m]
      Pos_y = Pos_y + (Vel_y*dt)/1000; // [m]

      Pose_X = Pos_x;
      Pose_Y = Pos_y;
      Pose_Theta = Theta;

      e_desp = sqrt((Pose_X_final - Pose_X)*(Pose_X_final - Pose_X) + (Pose_Y_final - Pose_Y)*(Pose_Y_final - Pose_Y));
      if (state_pid_pose == 0){// 0: girando para alinear
          Pose_Theta_ref = atan2(Pose_Y_final, Pose_X_final);
          e_giro = Pose_Theta_ref - Pose_Theta;
          if (abs(e_giro) < e_giro_margin){
              state_pid_pose = 1;
              e_prev_giro = 0;
              inte_prev_giro = 0;
              Serial.println("CAMGIO A 1");
          } else {
              inte_giro = inte_prev_giro + (dt * (e_giro + e_prev_giro) / 2);
              Vel_Rot_ref = float(kp_giro * e_giro + ki_giro * inte_giro); // rad/s
              RPM_Rot_ref = Vel_Rot_ref * L_robot / 2 * 60 / (2*pi); //
              RPM_A_ref = -RPM_Rot_ref; 
              RPM_B_ref = RPM_Rot_ref; 
              e_prev_giro = e_giro;
              inte_prev_giro = inte_giro;
          }
      }
      
      else if (state_pid_pose == 1){ // 1: aavanzado apra llegara
          Pose_Theta_ref = atan2(Pose_Y_final, Pose_X_final);
          e_giro = Pose_Theta_ref - Pose_Theta;
          if (abs(e_giro) > e_giro_margin*e_giro_margin_factor){
              state_pid_pose = 0;
          } else {
          
              if (e_desp < e_desp_margin){
                  state_pid_pose = 2;
                  e_prev_desp = 0;
                  inte_prev_desp = 0;
                  Serial.println("CAMGIO A 2");
              } else {
                  inte_desp = inte_prev_desp + (dt * (e_desp + e_prev_desp) / 2);
                  Vel_Desp_ref = float(kp_desp * e_desp + ki_desp * inte_desp); // m/s
                  RPM_Desp_ref = Vel_Desp_ref * 60 / (2*pi);
                  RPM_A_ref = RPM_Desp_ref; 
                  RPM_B_ref = RPM_Desp_ref; 
                  e_prev_desp = e_desp;
                  inte_prev_desp = inte_desp;
              }
          }
      }
      
      else if (state_pid_pose == 2){// 2: girando para llegar
          e_giro = Pose_Theta_final - Pose_Theta;
          if (e_giro < e_giro_margin){
              state_pid_pose = 3;
              Serial.println("CAMGIO A 3");
          } else{
              inte_giro = inte_prev_giro + (dt * (e_giro + e_prev_giro) / 2);
              Vel_Rot_ref = float(kp_giro * e_giro + ki_giro * inte_giro); // rad/s
              RPM_Rot_ref = Vel_Rot_ref * L_robot / 2 * 60 / (2*pi); //
              RPM_A_ref = -RPM_Rot_ref; 
              RPM_B_ref = RPM_Rot_ref; 
              e_prev_giro = e_giro;
              inte_prev_giro = inte_giro;
          }
      }

      else if (state_pid_pose == 3){ // 3: idlee
          RPM_A_ref = 0; 
          RPM_B_ref = 0;    
      }
      

      e_A = RPM_A_ref - RPM_A;
      e_B = RPM_B_ref - RPM_B;
      inte_A = inte_prev_A + (dt * (e_A + e_prev_A) / 2);
      inte_B = inte_prev_B + (dt * (e_B + e_prev_B) / 2);
      PWM_A_val = int(kp_A * e_A + ki_A * inte_A + (kd_A * (e_A - e_prev_A) / dt));
      PWM_B_val = int(kp_B * e_B + ki_B * inte_B + (kd_B * (e_B - e_prev_B) / dt));
      e_prev_A = e_A;
      e_prev_B = e_B;
      inte_prev_A = inte_A;
      inte_prev_B = inte_B;

    
      WriteDriverVoltageA(PWM_A_val);
      WriteDriverVoltageB(PWM_B_val);

      Serial.print(t);
      Serial.print(", ");
      Serial.print("state: ");
      Serial.print(state_pid_pose);
      //Serial.print("RPMROT:");
      //Serial.print(round(RPM_Rot_ref));
      //Serial.print(" RPMDESP:");
      //Serial.print(round(RPM_Desp_ref));
      //Serial.print(" PWMA:");
      //Serial.print(PWM_A_val);
      //Serial.print(" PWMB:");
      //Serial.print(PWM_B_val);

      Serial.print(" Egiro:");
      Serial.print(e_giro);
      Serial.print(" Edesp:");
      Serial.print(e_desp);
      
      Serial.print(" PosX");
      Serial.print(Pose_X);
      Serial.print(" PosY");
      Serial.print(Pose_Y);
      Serial.print(" Pos_Theta");
      Serial.print(round(Pose_Theta*180/pi));
      Serial.println("");

      ThetaA_prev = ThetaA;
      ThetaB_prev = ThetaB;
      t_prev = t;
  }
}
