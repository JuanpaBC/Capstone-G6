#include <Servo.h> //Imports the library Servo

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

#define redLed 51

#define baud 9600

Servo servo; //Defines the object Servo of type(class) Servo
int angle = 0; // Defines an integer

int d = 23; // mm of wheel
int ratio_ruedas = 500;
int steps = 12;
int state = 10;
int delay_time = 2000;
float vueltas_ruedasA;
float vueltas_ruedasB;
float avance;
int enable = 0;
bool pressed = true;
int PWM = 250;
volatile long encoderAPos = 0;
volatile long encoderBPos = 0;

long newpositionA;
long oldpositionA = 0;
float velA; // Velocidad del motor 0 en RPM

long newpositionB;
long oldpositionB = 0;
float velB; // Velocidad del motor 0 en RPM
int countdown = 5;
int printedCountdown = -1;

unsigned long ref_time = 0;
unsigned long time_ant = 0;
unsigned long newtime;
unsigned long now;
const int Period = 10000; // 10 ms = 100Hz
//const int Debounce = 500000; // 500ms
//unsigned long pressed_time = 0;

// ************** Función para avanzar ***************
void Avanzar(int pwm_ref)
{
    Serial.println("Adelante");
    // Avanzar motor A
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(ENA, pwm_ref);
    
    // Avanzar motor B
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(ENB, pwm_ref);
}

// ************** Función para parar ***************
void Parar()
{
    Serial.println("Parar");
    // Detener motor A
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(ENA, 0);

    // Detener motor B
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(ENB, 0);
}
// ************** Función para ir hacia atras ***************

void Atras(int pwm_ref)
{
    Serial.println("Atras");
    // Retroceder motor A
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(ENA, pwm_ref);

    // Retroceder motor B
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(ENB, pwm_ref);
}

// ************** Función para doblar a la derecha ***************
void Doblar_derecha(int pwm_ref)
{
    Serial.println("Doblar Derecha");
    // Avanzar motor A
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(ENA, pwm_ref);
    
    // Avanzar motor B
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(ENB, pwm_ref);
}

// ************** Función para doblar a la izquierda ***************
void Doblar_izquierda(int pwm_ref)
{
    Serial.println("Doblar Izquierda");
    // Avanzar motor A
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(ENA, pwm_ref);
    
    // Avanzar motor B
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(ENB, pwm_ref);
}

void doEncoderA1()
{
    if (digitalRead(AC1) == digitalRead(AC2))
    {
        encoderAPos++;
    }
    else
    {
        encoderAPos--;
    }
}

void doEncoderA2()
{
    if (digitalRead(AC1) == digitalRead(AC2))
    {
        encoderAPos--;
    }
    else
    {
        encoderAPos++;
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

void setup()
{
    pinMode(ENA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    pinMode(AC1, INPUT_PULLUP);
    digitalWrite(AC1, HIGH);
    pinMode(AC2, INPUT_PULLUP);
    digitalWrite(AC2, HIGH);

    pinMode(BC1, INPUT_PULLUP);
    digitalWrite(BC1, HIGH);
    pinMode(BC2, INPUT_PULLUP);
    digitalWrite(BC2, HIGH);

    pinMode(redLed, OUTPUT);

    servo.attach(ServoPin); // States that the servo is attached to pin 5
    servo.write(angle);

    attachInterrupt(digitalPinToInterrupt(AC1), doEncoderA1, CHANGE); // encoder 0 PIN A
    attachInterrupt(digitalPinToInterrupt(AC2), doEncoderA2, CHANGE); // encoder 0 PIN B

    attachInterrupt(digitalPinToInterrupt(BC1), doEncoderB1, CHANGE); // encoder 0 PIN A
    attachInterrupt(digitalPinToInterrupt(BC2), doEncoderB2, CHANGE); // encoder 0 PIN B

    Serial.begin(baud);
}
int donothing(){
  int i = 0;
  i++;
  delay(900);  
}

void loop()
{
    if (((micros() - time_ant) >= Period) && state != 10 && state != 9)
    { // Cada Period de tiempo hace el calculo
        newtime = micros();
        vueltas_ruedasA = (float)encoderAPos / (steps * ratio_ruedas);
        vueltas_ruedasB = (float)encoderBPos / (steps * ratio_ruedas);
        newpositionA = encoderAPos;
        newpositionB = encoderBPos;
        float rpm = 249500;                                                      
        velA = (float)(newpositionA - oldpositionA) * rpm / (newtime - time_ant);
        oldpositionA = newpositionA;

        velB = (float)(newpositionB - oldpositionB) * rpm / (newtime - time_ant);
        oldpositionB = newpositionB;

        // Calculate linear velocity (in mm/s)
        float distanceA = vueltas_ruedasA * d * 3.1415 / 10;
        float distanceB = vueltas_ruedasB * d * 3.1415 / 10;

        // Calculate the average linear velocity of both wheels
        avance = (distanceA + distanceB) / 2;

        time_ant = newtime;

        Serial.print(vueltas_ruedasA);
        Serial.print(" vueltas A, ");
        Serial.print(vueltas_ruedasB);
        Serial.print(" vueltas B, ");
        Serial.print(velA);
        Serial.print(" RPM A, ");
        Serial.print(velB);
        Serial.print(" RPM B, ");
        Serial.print(avance);
        Serial.print(" cm de avance, ");
        Serial.print((float)newtime * 0.000001);
        Serial.print(" s.");
        Serial.print(state);
        Serial.println(" Estado");
    }
    switch (state)
    {
    case 0:
        if (avance > 20)
        {
            Parar();
            ref_time = micros();
            state = 1;
        }
        break;

    case 1:
        if ((micros() - ref_time) * 0.000001 > 5)
        {
            state = 2;
            Atras(PWM);
        }
        break;

    case 2:
        if (avance < 0)
        {
            Parar();
            ref_time = micros();
            state = 3;
        }
        break;

    case 3:
        if ((micros() - ref_time) * 0.000001 > 3)
        {
            ref_time = micros();
            Doblar_derecha(150);
            state = 4;
        }
        break;
        
    case 4:
        if ((micros() - ref_time) * 0.000001 > 3)
        {
            ref_time = micros();
            Parar();
            state = 5;
        }
        break;
    case 5:
        if ((micros() - ref_time) * 0.000001 > 3)
        {
            ref_time = micros();
            Doblar_izquierda(150);
            state = 6;
        }
        break;
    case 6:
        if ((micros() - ref_time) * 0.000001 > 3)
        {
          Parar();
          state = 7;
          digitalWrite(redLed, HIGH);
        }
        break;
    case 7:
      for(angle = 0; angle < 180; angle++) {                                  
        servo.write(angle);               
        delay(15);                   
      } 
      if(angle >= 180){
        state = 8;
      }
    case 8:
      for(angle = 180; angle > 0; angle--){                                
        servo.write(angle);           
        delay(15);       
      } 
      if(angle <= 0){
        state = 9;
      }
    case 9:
      donothing();
    case 10:
        digitalWrite(redLed, HIGH);
        int c =  round(countdown - micros()* 0.000001);
        if(c != printedCountdown){
          printedCountdown = c;
        }
        if ((micros()) * 0.000001 > 5)
        {
          Avanzar(PWM);
          state = 0;
          digitalWrite(redLed, LOW);
        }
        break;
    default:
        break;
    }
}
