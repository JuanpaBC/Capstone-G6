#include <Servo.h> //Imports the library Servo

#define SLP 7 // Sensor light pin
#define SRP 2 // Sensor receptor pin

#define SP 8 // Servo pin
#define MAXANG 180 // Servo máx angle
#define MINANG 0 // Servo min angle
#define SCOOPDELAY 15

#define ENA 9 // D6
#define ENB 6

#define AIN1 11 // D1
#define AIN2 10  // D0

#define BIN1 5
#define BIN2 4

#define AC1 21 // D4
#define AC2 20 // D3

#define BC1 19
#define BC2 18

Servo servo;   // Defines the object Servo of type(class) Servo
int angle = MINANG; // Defines an integer

// Python commands
String left = "L";
String right = "R";
String forward = "U";
String backwards = "D";
String activate_scoop = "S";
String manual = "M";
String automatic = "N";

String msg;
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
const int Period = 10000; // 10 ms = 100Hz
//const int Debounce = 500000; // 500ms
//unsigned long pressed_time = 0;

// ************** Función para avanzar ***************
void Atras(int pwm_ref)
{
    //Serial.println("Adelante");
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
    //Serial.println("Parar");
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

void Avanzar(int pwm_ref)
{
    //Serial.println("Adelante");
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
    //Serial.println("Doblar Derecha");
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
    //Serial.println("Doblar Izquierda");
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
    //Serial.println("Pala");
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
}

void readSerialPort(String *msg) {
  *msg = "";
  char receivedChar;
  if (Serial.available()) {
      delay(10);
      while (Serial.available()) {
        receivedChar = Serial.read();
        if (receivedChar == '\n') {
          Serial.println("ACK\n");
          break;
        }
          *msg += receivedChar;
      }
  }
}

void stringSplitter(String msg, int *right_pwm, int *right_dir, int *left_pwm, int *left_dir){
  char inputBuffer[20];
  // Copy the string into a character array
  msg.toCharArray(inputBuffer, sizeof(inputBuffer));

  // Initialize strtok with the comma as the delimiter
  char *token = strtok(inputBuffer, ",");

  for(int i=0; i<4; i++) {
    int intValue = atoi(token); // Convert the token to an integer
    switch (i){
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
  }
}

void setup() {

    pinMode(SRP, INPUT);
    attachInterrupt(digitalPinToInterrupt(SRP), scoop, FALLING);

    servo.attach(SP);    // States that the servo is attached to pin 5
    servo.write(angle); // Sets the servo angle to 0 degrees

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

    attachInterrupt(digitalPinToInterrupt(AC1), doEncoderA1, CHANGE); // encoder 0 PIN A
    attachInterrupt(digitalPinToInterrupt(AC2), doEncoderA2, CHANGE); // encoder 0 PIN B

    attachInterrupt(digitalPinToInterrupt(BC1), doEncoderB1, CHANGE); // encoder 0 PIN A
    attachInterrupt(digitalPinToInterrupt(BC2), doEncoderB2, CHANGE); // encoder 0 PIN B

    Serial.begin(115200);
}

void loop() {

  readSerialPort(&msg);

  if (msg == manual) {
    auto_mode = false;
  } else if (msg == automatic) {
    auto_mode = true;
  }

  if (auto_mode) {

    int right_pwm;
    int right_dir;
    int left_pwm;
    int left_dir;
    stringSplitter(msg, &right_pwm, &right_dir, &left_pwm, &left_dir);

    // Implement motor control based on received values here.
    if (right_dir == 1) {
      // Set motor A direction based on right_dir
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
    } else {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
    }
    analogWrite(ENA, left_pwm);

    if (left_dir == 0) {
      // Set motor B direction based on left_dir
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
    } else {
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
    }
    analogWrite(ENB, right_pwm);
  }
  else {
    if(msg == forward){
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
    else if(msg == activate_scoop){
      scoop();
    }
    else if(msg == "") {
      Parar();
    }
  }
  msg = "";
}