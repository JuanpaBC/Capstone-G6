/*
  TB6612FNG-Dual-Driver
  made on 28 oct 2020
  by Amir Mohammad Shojaee @ Electropeak
  Home

*/

#define PWM1 11 // D6
#define STBY 8  // D2
#define AIN1 12 // D1
#define AIN2 4  // D0

#define C1 2 // D4
#define C2 5 // D3

#define LED 7
#define btn 13

int d = 23; // mm of wheel
int ratio_ruedas = 10;
int steps = 12;
int state = 0;
int delay_time = 2000;
float vueltas_ruedas;
float avance;
int PWM = 60;
volatile long encoderAPos = 0;

long newposition0;
long oldposition0 = 0;
float vel0; // Velocidad del motor 0 en RPM

unsigned long ref_time = 0;
unsigned long time_ant = 0;
unsigned long newtime;
const int Period = 10000; // 10 ms = 100Hz

void Adelante()
{
    // Para controlar el motor A
    // Serial.println("Adelante");
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWM1, PWM);
}

// ************** Función para parar ***************
void Parar()
{
    // Direccion motor A
    Serial.println("Parar");
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWM1, 0);
}
// ************** Función para ir hacia atras ***************
void Atras()
{
    // Para controlar el motor A
    Serial.println("Atras");
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWM1, PWM);
}

void doEncoderA1()
{
    if (digitalRead(C1) == digitalRead(C2))
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
    if (digitalRead(C1) == digitalRead(C2))
    {
        encoderAPos--;
    }
    else
    {
        encoderAPos++;
    }
}

void setup()
{
    pinMode(PWM1, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(STBY, OUTPUT);

    Adelante();
    digitalWrite(STBY, HIGH);

    pinMode(C1, INPUT_PULLUP);
    digitalWrite(C1, HIGH);
    pinMode(C2, INPUT_PULLUP);
    digitalWrite(C2, HIGH);

    attachInterrupt(digitalPinToInterrupt(C1), doEncoderA1, CHANGE); // encoder 0 PIN A
    attachInterrupt(digitalPinToInterrupt(C2), doEncoderA2, CHANGE); // encoder 0 PIN B

    Serial.begin(115200);
}

void loop()
{
    if ((micros() - time_ant) >= Period)
    { // Cada Period de tiempo hace el calculo
        newtime = micros();
        vueltas_ruedas = (float)encoderAPos / (steps * ratio_ruedas);
        newposition0 = encoderAPos;
        float rpm = 249500;                                                       // Para los 240 pasos por vuelta y para microsegundos a minutos
        vel0 = (float)(newposition0 - oldposition0) * rpm / (newtime - time_ant); // RPM
        oldposition0 = newposition0;
        avance = vueltas_ruedas * d * 3.1415 / 10;

        time_ant = newtime;
        // Serial.println(String(DigitalRead(encoder0PinA))+','+String(digitalRead(encoder0PinB)));
        Serial.print(vueltas_ruedas);
        Serial.print(" vueltas, ");
        Serial.print(vel0);
        Serial.print(" RPM, ");
        Serial.print(avance);
        Serial.print(" cm de avance, ");
        Serial.print((float)newtime * 0.000001);
        Serial.println(" s.");
    }
    if (avance > 500 && state == 0)
    {
        Parar();
        ref_time = micros();
        state = 1;
    }
    if (((micros() - ref_time) * 0.000001 > 5) && state == 1)
    {
        Serial.println(((micros() - ref_time) * 0.000001));
        state = 2;
        Atras();
    }
    if (avance < 0 && state == 2)
    {
        Parar();
        state = 3;
    }
}
