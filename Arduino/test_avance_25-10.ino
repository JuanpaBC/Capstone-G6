/*
  TB6612FNG-Dual-Driver
  made on 28 oct 2020
  by Amir Mohammad Shojaee @ Electropeak
  Home

*/

#define PWM1 11 // D6
#define PWM2 

#define STBY 8  // D2
#define AIN1 12 // D1
#define AIN2 4  // D0

#define BIN1 
#define BIN2

#define AC1 2 // D4
#define AC2 5 // D3

#define BC1
#define BC2

#define LED 7
#define btn 13

int d = 23; // mm of wheel
int ratio_ruedas = 10;
int steps = 12;
int state = 0;
int delay_time = 2000;
float vueltas_ruedasA;
float vueltas_ruedasB;
float avance;
bool enable = true;
int PWM = 60;
volatile long encoderAPos = 0;
volatile long encoderBPos = 0;

long newpositionA;
long oldpositionA = 0;
float velA; // Velocidad del motor 0 en RPM

long newpositionB;
long oldpositionB = 0;
float velB; // Velocidad del motor 0 en RPM


unsigned long ref_time = 0;
unsigned long time_ant = 0;
unsigned long newtime;
const int Period = 10000; // 10 ms = 100Hz

// ************** Función para avanzar ***************
void Adelante(int pwm_ref)
{
    Serial.println("Adelante");
    // Avanzar motor A
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWM1, pwm_ref);
    
    // Avanzar motor B
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWM2, pwm_ref);
}

// ************** Función para parar ***************
void Parar()
{
    Serial.println("Parar");
    // Detener motor A
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWM1, 0);

    // Detener motor B
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWM2, 0);
}
// ************** Función para ir hacia atras ***************

void Atras(int pwm_ref)
{
    Serial.println("Atras");
    // Retroceder motor A
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWM1, pwm_ref);

    // Retroceder motor B
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWM2, pwm_ref);
}

// ************** Función para doblar a la derecha ***************
void Doblar_derecha(int pwm_ref)
{
    Serial.println("Doblar Derecha");
    // Avanzar motor A
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWM1, pwm_ref);
    
    // Avanzar motor B
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWM2, pwm_ref);
}

// ************** Función para doblar a la izquierda ***************
void Doblar_izquierda(int pwm_ref)
{
    Serial.println("Doblar Izquierda");
    // Avanzar motor A
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWM1, pwm_ref);
    
    // Avanzar motor B
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWM2, pwm_ref);
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
    pinMode(PWM1, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWM2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    pinMode(LED, OUTPUT);

    Adelante(PWM);

    digitalWrite(STBY, HIGH);

    pinMode(AC1, INPUT_PULLUP);
    digitalWrite(AC1, HIGH);
    pinMode(AC2, INPUT_PULLUP);
    digitalWrite(AC2, HIGH);

    pinMode(BC1, INPUT_PULLUP);
    digitalWrite(BC1, HIGH);
    pinMode(BC2, INPUT_PULLUP);
    digitalWrite(BC2, HIGH);

    pinMode(btn, INPUT_PULLUP); 

    attachInterrupt(digitalPinToInterrupt(AC1), doEncoderA1, CHANGE); // encoder 0 PIN A
    attachInterrupt(digitalPinToInterrupt(AC2), doEncoderA2, CHANGE); // encoder 0 PIN B

    attachInterrupt(digitalPinToInterrupt(BC1), doEncoderB1, CHANGE); // encoder 0 PIN A
    attachInterrupt(digitalPinToInterrupt(BC2), doEncoderB2, CHANGE); // encoder 0 PIN B

    Serial.begin(115200);
}

void loop()
{
    if ((micros() - time_ant) >= Period)
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
        float averageLinearVelocity = (distanceA + distanceB) / 2;

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
        Serial.println(" s.");
    }

    if (digitalRead(btn) == LOW) {
        enable = !enable;
        delay(100);
    }

    if(enable){
        digitalWrite(LED, HIGH);
    } else{
        Parar()
        digitalWrite(LED, LOW);
    }
    switch (state)
    {
    case 0:
        if (avance > 500)
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
        if ((micros() - ref_time) * 0.000001 > 5)
        {
            ref_time = micros();
            Doblar_derecha(PWM);
            state = 4;
        }
        break;
    case 4:
        if ((micros() - ref_time) * 0.000001 > 5)
        {
            ref_time = micros();
            Doblar_izquierda(PWM);
            state = 5;
        }
        break;
    case 5:
        Parar();
        state = 6;
        break;
    default:
        break;
    }
}
