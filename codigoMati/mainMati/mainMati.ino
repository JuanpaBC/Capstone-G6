#include <Servo.h> //Imports the library Servo
//GeeKee CeeBee
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

#define pi 3.1415

int left_pwm;
int PWM_A_val;
int PWM_B_val;

byte buffer[3];
char msg[10];
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


void readSerialPort(int *left_pwm, int *right_pwm)
{
  if (Serial.available()) {
    Serial.readBytes(buffer, 2); 
    *left_pwm = buffer[0];
    *right_pwm = buffer[1];
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

}

void loop() {
    readSerialPort(&PWM_A_val, &PWM_B_val);
    WriteDriverVoltageA(PWM_A_val);
    WriteDriverVoltageB(PWM_B_val);
}
