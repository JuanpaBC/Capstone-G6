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


float Pos_x, Pos_y;
float Vel_lin, Vel_x, Vel_y;
float Vel_ang, Theta;

unsigned long t, t_prev;
int dt;

float NFactor = 1400;
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

float Pose_X, Pose_Y, Pose_Theta;
float Pose_X_final, Pose_Y_final, Pose_Theta_final, Pose_Theta_ref;

float kp_giro = 10;
float ki_giro = 1;
float e_giro, e_prev_giro, inte_giro, inte_prev_giro;
float Vel_Rot_ref, RPM_Rot_ref;

float kp_desp = 10;
float ki_desp = 1;
float e_desp, e_prev_desp, inte_desp, inte_prev_desp;
float Vel_Desp_ref, RPM_Desp_ref;

float e_giro_margin = 5*pi/180;
float e_desp_margin = 0.005;


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


int giro(float Pose_Theta_final, ){

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
      Serial.print("PosX: ");
      Serial.print(Pos_x);
      Serial.println("");

      //Serial.print("POSX: ");
      //Serial.print(Pos_x);
      //Serial.print("POSY: ");
      //Serial.print(Pos_y);
      //Serial.print("Theta");
      //sSerial.println(Theta);



      

      // DESPLAZAMIENTO CONTROLADO
      RPM_ref_A = RPM_ref; 
      RPM_ref_B = RPM_ref; 


      e_desp = sqrt((Pose_X_final - Pose_X)^2 + (Pose_Y_final - Pose_Y)^2);
      if (e_desp > e_desp_margin){
        Pose_Theta_ref = atan2(Pose_Y_final, Pose_X_final);
        e_giro = Pose_Theta_ref - Pose_Theta;
        if (e_giro > e_giro_margin){ //ALINEAR PARA AVANZAR
          inte_giro = inte_prev_giro + (dt * (e_giro + e_prev_giro) / 2);
          Vel_Rot_ref = float(kp_giro * e_giro + ki_giro * inte_giro); // rad/s
          RPM_Rot_ref = Vel_Rot_ref * L_robot / 2 * 60 / (2*pi); //
          RPM_ref_A = RPM_Rot_ref; 
          RPM_ref_B = -RPM_Rot_ref; 
          e_prev_giro = e_giro;
          inte_prev_giro = inte_giro;
        } else { // AVANZAR EN LINEA RECTA
          inte_desp = inte_prev_desp + (dt * (e_desp + e_prev_desp) / 2);
          Vel_Desp_ref = float(kp_desp * e_desp + ki_desp * inte_desp); // m/s
          RPM_Desp_ref = Vel_Desp_ref * 60 / (2*pi);
          RPM_ref_A = RPM_Desp_ref; 
          RPM_ref_B = RPM_Desp_ref; 
          e_prev_desp = e_desp;
          inte_prev_desp = inte_desp;
          e_prev_giro = 0;
          inte_prev_giro = 0;
        }
      } else {//ALINEAR EN EL LUGAR
        Pose_Theta_ref = Pose_Theta_final;
        e_giro = Pose_Theta_ref - Pose_Theta;
        inte_giro = inte_prev_giro + (dt * (e_giro + e_prev_giro) / 2);
        Vel_Rot_ref = float(kp_giro * e_giro + ki_giro * inte_giro); // rad/s
        RPM_Rot_ref = Vel_Rot_ref * L_robot / 2 * 60 / (2*pi); //
        RPM_ref_A = RPM_Rot_ref; 
        RPM_ref_B = -RPM_Rot_Ref; 
        e_prev_giro = e_giro;
        inte_prev_giro = inte_giro;
      }

      if(Pos_x > 1){
        WriteDriverVoltageA(0);
        WriteDriverVoltageB(0);
      }
      else {
        WriteDriverVoltageA(150);
        WriteDriverVoltageB(150);
      }


      ThetaA_prev = ThetaA;
      ThetaB_prev = ThetaB;
      t_prev = t;
  }
}
