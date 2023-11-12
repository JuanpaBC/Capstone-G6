

#define ENA 9 // D6
#define ENB 6

//#define STBY 8  // D2
#define AIN1 11 // D1
#define AIN2 10  // D0

#define BIN1 4
#define BIN2 5

#define AC1 23 // D4
#define AC2 22 // D3

#define BC1 13
#define BC2 12

#define switchPin 1

// GPIO comunication with raspberry
#define interruptPin 2

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

    pinMode(switchPin, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(AC1), doEncoderA1, CHANGE); // encoder 0 PIN A
    attachInterrupt(digitalPinToInterrupt(AC2), doEncoderA2, CHANGE); // encoder 0 PIN B

    attachInterrupt(digitalPinToInterrupt(BC1), doEncoderB1, CHANGE); // encoder 0 PIN A
    attachInterrupt(digitalPinToInterrupt(BC2), doEncoderB2, CHANGE); // encoder 0 PIN B

    attachInterrupt(digitalPinToInterrupt(interruptPin), readGPIO, RISING);
    Serial.begin(115200);
}

void loop()
{
    if (((micros() - time_ant) >= Period) && state != 8)
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

        Serial.println(digitalRead(AC1));
  
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
            state = 8;
        }
        break;

    case 3:
        if ((micros() - ref_time) * 0.000001 > 3)
        {
            ref_time = micros();
            Doblar_derecha(PWM);
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
            Doblar_izquierda(PWM);
            state = 6;
        }
        break;
    case 6:
        if ((micros() - ref_time) * 0.000001 > 3)
        {
          Parar();
          state = 7;
        }
        break;
    case 8:
        int c =  round(countdown - micros()* 0.000001);
        if(c != printedCountdown){
          Serial.println(c);
          printedCountdown = c;
        }
        if ((micros()) * 0.000001 > 5)
        {
          Avanzar(PWM);
          state = 0;
        }
        break;
    default:
        break;
    }
}

// ************** Función para avanzar ***************
void Atras(int pwm_ref)
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

void Avanzar(int pwm_ref)
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
    Serial.println(encoderAPos);
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
    Serial.println(encoderAPos);
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
    Serial.println(encoderBPos);
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
    Serial.println(encoderBPos);
    if (digitalRead(BC1) == digitalRead(BC2))
    {
        encoderBPos--;
    }
    else
    {
        encoderBPos++;
    }
}
int binaryToInt(int b3, int b2, int b1, int b0)
{
    int decimal = b3 * 8 + b2 * 4 + b1 * 2 + b0;
    return decimal;
}

void readGPIO()
{
    b0 = digitalRead(p0);
    b1 = digitalRead(p1);
    b2 = digitalRead(p2);
    b3 = digitalRead(p3);
    mode_1 = digitalRead(p4);
    mode_2 = digitalRead(p5);
    mode_3 = digitalRead(p6);
    int pwm = binaryToInt(b3, b2, b1, b0); // convert byte to int
}