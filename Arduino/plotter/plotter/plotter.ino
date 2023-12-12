// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

float theta = 0.0;
float theta_old = 0.0;
int t  = 0;
int t_old = 0;
float dt;
void setup(void) {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(100);
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  t = millis();
  /* Print out the values */
  //Serial.print("AccelX:");
  //Serial.print(a.acceleration.x);
  //Serial.print(",");
  //Serial.print("AccelY:");
  //Serial.print(a.acceleration.y);
  //Serial.print(",");
  //Serial.print("AccelZ:");
  //Serial.print(a.acceleration.z);
  //Serial.print(", ");
  Serial.print("GyroZ:");
  Serial.print(g.gyro.z);
  //-------------Calculo de Theta-----------------
  dt = (t-t_old);
  theta = (g.gyro.z*dt)/1000+theta_old;
  
  Serial.print(",");
  Serial.print("dt:");
  Serial.print(t-t_old);
  Serial.print(",");
  Serial.print("dtSecs:");
  Serial.print((t-t_old)/1000);
  Serial.print(",");
  Serial.print("Theta:");
  Serial.print(theta);
  Serial.println("");
  t_old = t;
  theta_old = theta;
  delay(10);
}