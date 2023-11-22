#include <Servo.h> //Imports the library Servo

#define trigPin 3 // TriggerSensor
#define echoPin 2 // EchoSensor

#define ServoPin 8 // Servo pin
#define MAXANG 180 // Servo máx angle
#define MINANG 0 // Servo min angle
#define SCOOPDELAY 15

#define ENA 6 // D6
#define ENB 11

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

int right_pwm_value;
int right_dir_value;
int left_pwm_value;
int left_dir_value;
Servo servo;   // Defines the object Servo of type(class) Servo
int angle = MINANG; // Defines an integer

// Python commands
char left[] = "L";
char right[] = "R";
char forward[] = "U";
char backwards[] = "D";
char scoopeSignal[] = "S";
char manual[] = "M";
char automatic[] = "N";

char msg[40];
bool auto_mode = true;

int d = 23; // mm of wheel
int ratio_ruedas = 10;
int steps = 12;
int state = 8;
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
float distancePerStep = 0.0;
float distanceTraveled = 0.0;
const int Period = 10000; // 10 ms = 100Hz
//const int Debounce = 500000; // 500ms
//unsigned long pressed_time = 0;

volatile bool scoopFlag = false; 

// ************** Función para retroceder ***************
void Atras(int pwm_ref)
{
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
    // Detener motor A
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(ENA, 0);

    // Detener motor B
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(ENB, 0);
}
// ************** Función para ir hacia avanzar ***************

void Avanzar(int pwm_ref)
{
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

void readSerialPort() {
  memset(msg, 0, sizeof(msg));  // Clear the msg array
  if (Serial.available()) {
    delay(10);
    int i = 0;
    while (Serial.available() > 0 && i < sizeof(msg) - 1) {
      msg[i++] = Serial.read();
    }
    Serial.flush();
  }
}

void stringSplitter(char *msg, int *right_pwm, int *right_dir, int *left_pwm, int *left_dir) {
  char *token = strtok(msg, ",");

  for (int i = 0; i < 4; i++) {
    int intValue = atoi(token);
    switch (i) {
      case 0:
        *right_pwm = intValue;
        break;
      case 1:
        *right_dir = intValue;
        break;
      case 2:
        *left_pwm = intValue;
        break;
      case 3:
        *left_dir = intValue;
        break;
    }
    token = strtok(NULL, ",");
  }
}


void checkDistance() {


  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;

  if (auto_mode && distance < 10) {
    scoop();
  }
}

void setup() {
    digitalWrite(redLed, LOW);

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    servo.attach(ServoPin);    // States that the servo is attached to pin 5
    servo.write(angle); // Sets the servo angle to 0 degrees

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
}

void loop() {

  readSerialPort();

  if(msg == manual){
    auto_mode = false;
  }
  else if(msg == automatic){
    auto_mode = true;
    
  }

  if(auto_mode){
    if(msg[0] != '\0') {
      stringSplitter(msg, &right_pwm_value, &right_dir_value, &left_pwm_value, &left_dir_value);
    }
    if(right_pwm_value == -1){
      float distancePerStep = (3.141592653589793 * d * ratio_ruedas) / steps; // Assuming PI = 3.141592653589793
      avance = encoderAPos * distancePerStep;

      checkDistance();

      if (avance >= 30.0) {
        checkDistance();
        Parar();
        encoderAPos = 0;
        distanceTraveled = 0.0;
      } else {
        // Continue advancing
        Avanzar(PWM);
      }
    }
    if(left_dir_value == -1){
      digitalWrite(redLed, HIGH);
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
    }
    else {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
    }
    analogWrite(ENA, left_pwm_value);

    if(right_dir_value == 1){
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
    }
    else {
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
    }
    analogWrite(ENB, right_pwm_value);
  }
  else {
    digitalWrite(redLed, LOW);
    if(msg == forward){
      checkDistance();
      Avanzar(PWM);
    }
    else if(msg == backwards){
      Atras(PWM);
    }
    else if(msg == left){
      Doblar_izquierda(PWM);
    }
    else if(msg == right){
      Doblar_derecha(PWM);
    }
    else if(msg == scoopeSignal){
      scoop();
    }
    else if(msg[0] == '\0') {
      Parar();
    }
  }
  strcpy(msg, "");
}
