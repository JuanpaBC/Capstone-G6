// #include <Servo.h> //Imports the library Servo
// GeeKee CeeBee

#define trigPin 3 // TriggerSensor
#define echoPin 2 // EchoSensor

#define ServoPin 8 // Servo pin
#define MAXANG 180 // Servo máx angle
#define MINANG 0   // Servo min angle
#define SCOOPDELAY 15

#define ENA 6 // D6
#define ENB 12

#define AIN1 4  // D4
#define AIN2 5  // D5
#define BIN1 10 // D10
#define BIN2 9  // D9

#define AC1 21 // D2
#define AC2 20 // D3
#define BC1 19 // D7
#define BC2 18 // D8

#define baud 9600

#define pi 3.1416

// ************ DEFINITIONS A************
float kp_A = 0.02;
float ki_A = 0.00015;
float kd_A = 0;

// ************ DEFINITIONS A************
float kp_B = 0.022;
float ki_B = 0.00001;
float kd_B = 0;

unsigned long t;
unsigned long t_prev = 0;

volatile long EncoderCountA = 0;
volatile long EncoderCountB = 0;
//--------------------------- Pines---------------------------

volatile unsigned long count = 0;
unsigned long count_prev = 0;

float ThetaA, ThetaB;
float RPM_A, RPM_B;
float RPM_A_ref, RPM_B_ref;
float ThetaA_prev = 0;
float ThetaB_prev = 0;
int dt;

int PWM_A_val, PWM_B_val;
int PWM_min = 150;
int PWM_max = 255;

float e_A, e_prev_A = 0, inte_A, inte_prev_A = 0;
float e_B, e_prev_B = 0, inte_B, inte_prev_B = 0;

char msg[60];
int ratio_ruedas = 34.02;


//***Motor Driver Functions*****

void CheckPWM(int PWM_val)
{
    if (abs(PWM_val) < PWM_min){
        return int(PWM_val/abs(PWM_val) * PWM_min)
    }
    if (abs(PWM_val) > PWM_max){
        return int(PWM_val/abs(PWM_val) * PWM_max)
    }
    return PWM_val
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

void ISR_EncoderA2()
{
    bool PinB = digitalRead(AC2);
    bool PinA = digitalRead(AC1);

    if (PinB == LOW){
        if (PinA == HIGH){
            EncoderCountA++;
        }
        else{
            EncoderCountA--;
        }
    }
    else{
        if (PinA == HIGH){
            EncoderCountA--;
        }
        else{
            EncoderCountA++;
        }
    }
}
void ISR_EncoderA1()
{
    bool PinB = digitalRead(AC2);
    bool PinA = digitalRead(AC1);

    if (PinA == LOW){
        if (PinB == HIGH){
            EncoderCountA--;
        }
        else{
            EncoderCountA++;
        }
    }
    else{
        if (PinB == HIGH){
            EncoderCountA++;
        }
        else{
            EncoderCountA--;
        }
    }
}
void ISR_EncoderB2()
{
    bool PinB = digitalRead(BC2);
    bool PinA = digitalRead(BC1);

    if (PinA == LOW){
        if (PinB == HIGH){
            EncoderCountB++;
        }
        else{
            EncoderCountB--;
        }
    }
    else{
        if (PinB == HIGH){
            EncoderCountB--;
        }
        else{
            EncoderCountB++;
        }
    }
}
void ISR_EncoderB1()
{
    bool PinB = digitalRead(BC2);
    bool PinA = digitalRead(BC1);

    if (PinA == LOW){
        if (PinB == HIGH){
            EncoderCountB--;
        }
        else{
            EncoderCountB++;
        }
    }
    else{
        if (PinB == HIGH){
            EncoderCountB++;
        }
        else{
            EncoderCountB--;
        }
    }
}

void readSerialPort()
{
    memset(msg, 0, sizeof(msg)); // Clear the msg array
    if (Serial.available()){
        delay(20);
        int i = 0;
        while (Serial.available() > 0 && i < sizeof(msg) - 1){
            msg[i++] = Serial.read();
        }
        Serial.flush();
    }
}

void stringSplitter(char *msg, float *left_rpm, float *right_rpm)
{
    char *token = strtok(msg, ",");
    for (int i = 0; i < 2; i++)
    {
        float floatValue = atof(token); // Use atof for floating-point values
        switch (i)
        {
        case 0:
            *left_rpm = floatValue;
            break;
        case 1:
            *right_rpm = floatValue;
            break;
        }
        token = strtok(NULL, ",");
    }
}

float sign(float x)
{
    if (x > 0)
    {
        return 1;
    }
    else if (x < 0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

void setup()
{
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

    cli();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 12499; // Prescaler = 64
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS11 | 1 << CS10);
    TIMSK1 |= (1 << OCIE1A);
    sei();
}

void loop()
{
    readSerialPort();
    if (msg[0] != '\0' && msg[0] != ' ' && msg != NULL)
    {
        stringSplitter(msg, &kp_B, &RPM_d_B);
        msg[0] == '\0';
    }
    if (count > count_prev)
    {
        t = millis();
        ThetaA = EncoderCountA;
        ThetaB = EncoderCountB;
        dt = (t - t_prev)/1000; // [s]
        RPM_A = (ThetaA - ThetaA_prev)/11 / (3 * dt) * 60;
        RPM_B = (ThetaB - ThetaB_prev)/11 / (3 * dt) * 60;
        e_A = RPM_A_ref - RPM_A;
        e_B = RPM_B_ref - RPM_B;
        inte_A = inte_prev_A + (dt * (e_A + e_prev_A) / 2);
        inte_B = inte_prev_B + (dt * (e_B + e_prev_B) / 2);
        PWM_A_val = int(kp_A * e_A + ki_A * inte_A + (kd_A * (e_A - e_prev_A) / dt));
        PWM_B_val = int(kp_B * e_B + ki_B * inte_B + (kd_B * (e_B - e_prev_B) / dt));
        PWM_A_val = CheckPWM(PWM_A_val);
        PWM_B_val = CheckPWM(PWM_B_val);
        inte_A = inte_prev_A;
        inte_B = inte_prev_B;
        WriteDriverVoltageA(PWM_A_val);
        WriteDriverVoltageB(PWM_B_val);

        Serial.print(count * 0.05);
        Serial.print(", ");
        Serial.print("refA: ");
        Serial.print(RPM_A_ref);
        // Serial.print("EncoderCountA: ");
        // Serial.print(EncoderCountA);
        Serial.print(" | RPMA: ");
        Serial.print(RPM_A);
        Serial.print(", ");

        Serial.print("refB: ");
        Serial.print(RPM_B_ref);
        // Serial.print("EncoderCountB: ");
        // Serial.print(EncoderCountB);
        Serial.print(" | RPM_B: ");
        Serial.print(RPM_B);
        Serial.println("");

        ThetaA_prev = ThetaA;
        ThetaB_prev = ThetaB;
        count_prev = count;
        t_prev = t;
        inte_prev_A = inte_A;
        inte_prev_B = inte_B;
        e_prev_A = e_A;
        e_prev_B = e_B;
    }
}

ISR(TIMER1_COMPA_vect)
{
    count++;
    /*Serial.print(count * 0.05);
    Serial.print(": ");*/
}