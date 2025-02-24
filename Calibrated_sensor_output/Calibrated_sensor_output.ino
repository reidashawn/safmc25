#include "FastIMU.h"
#include <Wire.h>
#include <ArduinoJson.h>
#define POT_PIN 26
#define BUT1_PIN 35
#define BUT2_PIN 32
#define BUT3_PIN 33
#define BUT4_PIN 25
#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
MPU6050 IMU;               //Change to the name of any supported IMU! 

// Currently supported IMUS: MPU9255 MPU9250 MPU6886 MPU6500 MPU6050 ICM20689 ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL QMI8658

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;
JsonDocument controllerData;


void setup() {

  Wire.begin();
  Wire.setClock(400000); //400khz clock
  Serial.begin(115200);
  pinMode(POT_PIN, INPUT);
  pinMode(BUT1_PIN, INPUT);
  pinMode(BUT2_PIN, INPUT);
  pinMode(BUT3_PIN, INPUT);
  pinMode(BUT4_PIN, INPUT);

  while (!Serial) {
    ;
  }
  Serial.print("help me");
  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  
#ifdef PERFORM_CALIBRATION
  Serial.println("FastIMU calibration & data example");
  Serial.println("Keep IMU level.");
  delay(5000);
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  delay(3000);
  IMU.init(calib, IMU_ADDRESS);
#endif

  //err = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  //err = IMU.setAccelRange(2);       //THESE TWO SET THE GYRO RANGE TO ±500 DPS AND THE ACCELEROMETER RANGE TO ±2g
  
  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
}

void loop() {
  IMU.update();
  IMU.getAccel(&accelData);
  
  controllerData["accelX"] = accelData.accelX;
  controllerData["accelY"] = accelData.accelY;
  controllerData["accelZ"] = accelData.accelZ;
  
  controllerData["gyroX"] = gyroData.gyroX;
  controllerData["gyroY"] = gyroData.gyroY;
  controllerData["gyroZ"] = gyroData.gyroZ;

  controllerData["pot"] = analogRead(POT_PIN);
  controllerData["but1"] = digitalRead(BUT1_PIN);
  controllerData["but2"] = digitalRead(BUT2_PIN);
  controllerData["but3"] = digitalRead(BUT3_PIN);
  controllerData["but4"] = digitalRead(BUT4_PIN);

  serializeJson(controllerData, Serial);
  // Serial.print(accelData.accelX);
  // Serial.print(", ");
  // Serial.print(accelData.accelY);
  // Serial.print(", ");
  // Serial.print(accelData.accelZ);
  // Serial.print(", ");
  // IMU.getGyro(&gyroData);
  // Serial.print(gyroData.gyroX);
  // Serial.print(", ");
  // Serial.print(gyroData.gyroY);
  // Serial.print(", ");
  // Serial.print(gyroData.gyroZ);
  // Serial.print(", ");
  // Serial.print(analogRead(POT_PIN));
  // Serial.print(", ");
  // Serial.print(digitalRead(BUT1_PIN));
  // Serial.println();
  Serial.println();
  delay(100);
}
