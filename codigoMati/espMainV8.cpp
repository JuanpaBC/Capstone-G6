#include <Arduino.h>
#include <Wire.h>
#include <cstdlib>
#include <iostream>
#include <AccelStepper.h>
#include <cstring>

#define en 27 //Motor enable

#define dirPan  25
#define stepPan  26

#define dirTilt  33
#define stepTilt  32

#define microstep 32
#define stepsPerRevolution 200*microstep

#define RXD2 16
#define TXD2 17

AccelStepper PanStepper(AccelStepper::DRIVER, stepPan, dirPan);
AccelStepper TiltStepper(AccelStepper::DRIVER, stepTilt, dirTilt);

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;

byte buffer[3];
byte buffer2[3];

// Variables para control de tiempos
//https://www.youtube.com/watch?v=YNBphNvJ0Sw
uint16_t delay_ms = 100; // para tiempos de espera
ulong current_t1, previous_t1;
ulong current_t2, previous_t2;
ulong current_t3, previous_t3;
ulong current_t4, previous_t4;

ulong startTime3, endTime3;
ulong startTime4, endTime4;

int k1 = 0, k2 = 0, k3 = 0, k4 = 0;

unsigned long currentTimeXISE, previousTimeXISE;
double elapsedTimeXISE;
unsigned long currentTimeXITSE, previousTimeXITSE;
double elapsedTimeXITSE;
unsigned long currentTimeXIAE, previousTimeXIAE;
double elapsedTimeXIAE;
unsigned long currentTimeXITAE, previousTimeXITAE;
double elapsedTimeXITAE;
unsigned long currentTimeYISE, previousTimeYISE;
double elapsedTimeYISE;
unsigned long currentTimeYITSE, previousTimeYITSE;
double elapsedTimeYITSE;
unsigned long currentTimeYIAE, previousTimeYIAE;
double elapsedTimeYIAE;
unsigned long currentTimeYITAE, previousTimeYITAE;
double elapsedTimeYITAE;
double ISEX, IAEX, ITSEX, ITAEX;
double ISE_cumX, IAE_cumX, ITSE_cumX, ITAE_cumX;
double ISEY, IAEY, ITSEY, ITAEY;
double ISE_cumY, IAE_cumY, ITSE_cumY, ITAE_cumY;

// Variables para control de tiempos
unsigned long currentTime_pan1, previousTime_pan1;
double elapsedTime_pan1;
double lastError_pan1, cumError_pan1, rateError_pan1;
double output_pan1;

unsigned long currentTime_pan2, previousTime_pan2;
double elapsedTime_pan2;
double lastError_pan2, cumError_pan2, rateError_pan2;
double output_pan2;

unsigned long currentTime_tilt1, previousTime_tilt1;
double elapsedTime_tilt1;
double lastError_tilt1, cumError_tilt1, rateError_tilt1;
double output_tilt1;

unsigned long currentTime_tilt2, previousTime_tilt2;
double elapsedTime_tilt2;
double lastError_tilt2, cumError_tilt2, rateError_tilt2;
double output_tilt2;

// Constantes para referencia (ficticia por el momento);
int xR = 256/2;
int yR = 192/2;
int X = 256/2;
int Y = 192/2;
// error para cada eje particular
double errorX;
double errorY;

// PID variables
float pan_kP = 0.8, pan_kI = 1e-8, pan_kD = 5e-9; // (1, 1e-9, 0)
// float pan_kP = 0.4, pan_kI = 1e-8, pan_kD = 1e-9;
float tilt_kP = 0.8, tilt_kI = 2e-8, tilt_kD = 5e-9; //(1, 1e-9, 0)
double pan, tilt; //outputs de los PID
//---------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------//
double PID_Pan1(double errorPIDX1)
{
  currentTime_pan1 = millis();
  // elapsedTime_pan = (double)(100);                                  // obtener el tiempo actual
  elapsedTime_pan1 = (double)(currentTime_pan1 - previousTime_pan1);     // calcular el tiempo transcurrido                                   // determinar el error entre la consigna y la medición
  cumError_pan1 += errorPIDX1 * elapsedTime_pan1;                        // calcular la integral del error
  rateError_pan1 = (errorPIDX1 - lastError_pan1) / elapsedTime_pan1;          // calcular la derivada del error
 
  double output_pan1 = pan_kP*errorPIDX1 + pan_kI*cumError_pan1 + pan_kD*rateError_pan1;     // calcular la salida del PID

  lastError_pan1 = errorPIDX1;                                      // almacenar error anterior
  previousTime_pan1 = currentTime_pan1;                             // almacenar el tiempo anterior
 
  return output_pan1;
}

double PID_Pan2(double errorPIDX2)
{
  currentTime_pan2 = millis();
  // elapsedTime_pan = (double)(100);                                  // obtener el tiempo actual
  elapsedTime_pan2 = (double)(currentTime_pan2 - previousTime_pan2);     // calcular el tiempo transcurrido                                   // determinar el error entre la consigna y la medición
  cumError_pan2 += errorPIDX2 * elapsedTime_pan2;                        // calcular la integral del error
  rateError_pan2 = (errorPIDX2 - lastError_pan2) / elapsedTime_pan2;          // calcular la derivada del error
 
  double output_pan2 = pan_kP*errorPIDX2 + pan_kI*cumError_pan2 + pan_kD*rateError_pan2;     // calcular la salida del PID

  lastError_pan2 = errorPIDX2;                                      // almacenar error anterior
  previousTime_pan2 = currentTime_pan2;                             // almacenar el tiempo anterior
 
  return output_pan2;
}

double PID_Tilt1(double errorPIDY1)
{
  currentTime_tilt1 = millis();
  // elapsedTime_tilt = (double)(100);                                   // obtener el tiempo actual
  elapsedTime_tilt1 = (double)(currentTime_tilt1 - previousTime_tilt1);     // calcular el tiempo transcurrido                                   
  // determinar el error entre la consigna y la medición
  cumError_tilt1 += errorPIDY1 * elapsedTime_tilt1;                        // calcular la integral del error
  rateError_tilt1 = (errorPIDY1 - lastError_tilt1) / elapsedTime_tilt1;          // calcular la derivada del error
 
  double output_tilt1 = tilt_kP*errorPIDY1 + tilt_kI*cumError_tilt1 + tilt_kD*rateError_tilt1;     // calcular la salida del PID

  lastError_tilt1 = errorPIDY1;                                      // almacenar error anterior
  previousTime_tilt1 = currentTime_tilt1;                             // almacenar el tiempo anterior
 
  return output_tilt1;
}

double PID_Tilt2(double errorPIDY2)
{
  currentTime_tilt2 = millis();
  // elapsedTime_tilt = (double)(100);                                   // obtener el tiempo actual
  elapsedTime_tilt2 = (double)(currentTime_tilt2 - previousTime_tilt2);     // calcular el tiempo transcurrido                                   
  // determinar el error entre la consigna y la medición
  cumError_tilt2 += errorPIDY2 * elapsedTime_tilt2;                        // calcular la integral del error
  rateError_tilt2 = (errorPIDY2 - lastError_tilt2) / elapsedTime_tilt2;          // calcular la derivada del error
 
  double output_tilt2 = tilt_kP*errorPIDY2 + tilt_kI*cumError_tilt2 + tilt_kD*rateError_tilt2;     // calcular la salida del PID

  lastError_tilt2 = errorPIDY2;                                      // almacenar error anterior
  previousTime_tilt2 = currentTime_tilt2;                             // almacenar el tiempo anterior
 
  return output_tilt2;
}

// void desempenoXISE(double errorX){
//   currentTimeXISE = millis();
//   while (currentTimeXISE < 5000)
//   {                                 // obtener el tiempo actual
//     elapsedTimeXISE = (double)(currentTimeXISE - previousTimeXISE);     // calcular el tiempo transcurrido  
//     ISEX = ((errorX*errorX) - (lastError_pan*lastError_pan))/2;
//     previousTimeXISE = currentTimeXISE;
//     ISE_cumX += ISEX;
  
//   }
//   // return ISE_cumX;
// }

// void desempenoXITSE(double errorX){

//   currentTimeXITSE = millis();    
//   while (currentTimeXITSE < 5000)
//   {                             // obtener el tiempo actual
//     elapsedTimeXITSE = (double)(currentTimeXITSE - previousTimeXITSE);     // calcular el tiempo transcurrido  
//     ITSEX = k1*((errorX*errorX) - (lastError_pan*lastError_pan))/2;
//     previousTimeXITSE = currentTimeXISE;
//     ITSE_cumX += ITSEX;
//     k1 = k1 + 1; 
//   }
//   // return ISE_cumX;
// }

// void desempenoXIAE(double errorX){
//   currentTimeXIAE = millis();  
//   while (currentTimeXIAE < 5000)
//   {                               // obtener el tiempo actual
//     elapsedTimeXIAE = (double)(currentTimeXIAE - previousTimeXIAE);     // calcular el tiempo transcurrido  
//     IAEX = (errorX - lastError_pan)/2;
//     previousTimeXIAE = currentTimeXIAE;
//     IAE_cumX += IAEX;
//   }
//   // return IAE_cumX;
// }

// void desempenoXITAE(double errorX){
//   currentTimeXITAE = millis();
//   while (currentTimeXITAE < 5000)
//   {                                 // obtener el tiempo actual
//     elapsedTimeXITAE = (double)(currentTimeXITAE - previousTimeXITAE);     // calcular el tiempo transcurrido  
//     ITAEX = k2*(errorX - lastError_pan)/2;

//     previousTimeXITAE = currentTimeXITAE;
//     ITAE_cumX += ITAEX;
//     k2 = k2 + 1;
//   }
//   // return IAE_cumX;
// }

// void desempenoYISE(double errorY){
//   while (currentTimeXITAE < 5000)
//   {
//   currentTimeYISE = millis();                                 // obtener el tiempo actual
//   elapsedTimeYISE = (double)(currentTimeYISE - previousTimeYISE);     // calcular el tiempo transcurrido  
//   ISEY = ((errorY*errorY) - (lastError_tilt*lastError_tilt))/2;
//   previousTimeYISE = currentTimeYISE;
//   ISE_cumY += ISEY;
//   // return ISE_cumY;
//   }
// }

// void desempenoYITSE(double errorX){

//   currentTimeYITSE = millis();    
//   while (currentTimeYITSE < 5000)
//   {                             // obtener el tiempo actual
//     elapsedTimeYITSE = (double)(currentTimeYITSE - previousTimeYITSE);     // calcular el tiempo transcurrido  
//     ITSEY = k3*((errorY*errorY) - (lastError_pan*lastError_pan))/2;
//     previousTimeYITSE = currentTimeYISE;
//     ITSE_cumY += ITSEY;
//     k3 = k3 + 1; 
//   }
//   // return ISE_cumX;
// }

// void desempenoYIAE(double errorY){
//   while (currentTimeXITAE < 5000)
//   {
//   currentTimeYIAE = millis();                                 // obtener el tiempo actual
//   elapsedTimeYIAE = (double)(currentTimeYIAE - previousTimeYIAE);     // calcular el tiempo transcurrido  
//   IAEY = (errorY - lastError_tilt)/2;

//   previousTimeYIAE = currentTimeYIAE;
//   IAE_cumY += IAEY;
//   // return IAE_cumY;
//   }
// }

// void desempenoYITAE(double errorX){
//   currentTimeYITAE = millis();
//   while (currentTimeYITAE < 5000)
//   {                                 // obtener el tiempo actual
//     elapsedTimeYITAE = (double)(currentTimeYITAE - previousTimeYITAE);     // calcular el tiempo transcurrido  
//     ITAEY = k4*(errorY - lastError_pan)/2;

//     previousTimeYITAE = currentTimeYITAE;
//     ITAE_cumY += ITAEY;
//     k4 = k4 + 1;
//   }
//   // return IAE_cumX;
// }

void tarea1(void *parameter) 
{   

  while (true) 
  {
    previous_t1 = micros();
    Serial.readBytes(buffer, 3); Serial2.readBytes(buffer2, 3);  
    if (buffer[2] == 'E')
    {
      X = buffer[0];
      Y = buffer[1];
    }
    if (buffer2[2] == 'E')
    {
      xR = buffer2[0];
      yR = buffer2[1];
    }
    current_t1 = micros();
    // Serial.println(current_t1 - previous_t1);
  }
}

void tarea3(void *parameter) // Proceso PID + motor pan
{ // A largo plazo deben ser dos tareas distintas, uno por cada motor.
  while (true) 
  {
    // startTime3 = micros();
    current_t3 = millis();
    if (current_t3 - previous_t3 > delay_ms && digitalRead(en))
    {
      previous_t3 = current_t3;
      errorX = xR - X;
      pan = PID_Pan2(PID_Pan1(errorX));
      PanStepper.move(pan);
    }
   PanStepper.run(); 
  //  endTime3 = micros();
  //  Serial.println(endTime3 - startTime3);
  }
}
//----------------------------------------------------//
//----------------------------------------------------//
void tarea4(void *parameter) // Proceso PID + motor tilt
{ // A largo plazo deben ser dos tareas distintas, uno por cada motor.
  while (true) 
  {
    // startTime4 = micros();

    current_t4 = millis();
    if (current_t4 - previous_t4 > delay_ms && digitalRead(en))
    {
      previous_t4 = current_t4;
      errorY = Y - yR;
      tilt = PID_Tilt2(PID_Tilt1(errorY));
      TiltStepper.move(tilt);
    }
    TiltStepper.run(); 
    // endTime4 = micros();
    // Serial.println(endTime4 - startTime4);
  }
}
//----------------------------------------------------//
//----------------------------------------------------//
void setup() 
{
//----------------------------------------------------//
//--------SERIAL Y TIME SETUP-------------------------//
  Serial.begin(115200); Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  previous_t1 = millis();
  previous_t2 = millis();
  previous_t3 = millis();
  previous_t4 = millis();  
  pinMode(en, INPUT_PULLDOWN);
//----------------------------------------------------//
//----------------------------------------------------//
//--------MOTORES SETUP-------------------------------//
  PanStepper.setMaxSpeed(stepsPerRevolution);     TiltStepper.setMaxSpeed(stepsPerRevolution);
  PanStepper.setAcceleration(stepsPerRevolution/2); TiltStepper.setAcceleration(stepsPerRevolution/2);
  PanStepper.setCurrentPosition(0);                 TiltStepper.setCurrentPosition(0);
//----------------------------------------------------//
//----------------------------------------------------//

//----------------------------------------------------//
//----------------------------------------------------//
//--------TASK SETUP----------------------------------//
  xTaskCreatePinnedToCore
  ( // Tarea para control de recepcion de informacion
    tarea1,      // Función de la tarea
    "Task1",    // Nombre de la tarea
    4096,      // Tamaño de la pila
    NULL,       // Parámetros de la tarea
    1,          // Prioridad
    &Task1,     // Manejador de la tarea
    0           // Núcleo 0
  );

  xTaskCreatePinnedToCore
  ( // PID process + motor pan
    tarea3,      // Función de la tarea
    "Task3",    // Nombre de la tarea
    4096,      // Tamaño de la pila
    NULL,       // Parámetros de la tarea
    1,          // Prioridad
    &Task3,     // Manejador de la tarea
    1           // Núcleo 1
  );

  xTaskCreatePinnedToCore
  ( // PID process + motor tilt
    tarea4,      // Función de la tarea
    "Task4",    // Nombre de la tarea
    4096,      // Tamaño de la pila
    NULL,       // Parámetros de la tarea
    1,          // Prioridad
    &Task4,     // Manejador de la tarea
    1           // Núcleo 1
  );
}
//----------------------------------------------------//
//----------------------------------------------------//

void loop() {
  // El bucle principal está vacío, ya que las tareas se ejecutan en paralelo.
}