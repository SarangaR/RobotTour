#include "Arduino.h"
#include "Motor.h"
#include "Encoder.h"
#include "Drivetrain.h"
#include "DistanceUtiil.h"
#include "ICM_20948.h"

// ============== PINS ==============
#define L_PIN1 1
#define L_PIN2 2
#define L_ENCA 3
#define L_ENCB 4

#define R_PIN1 5
#define R_PIN2 6
#define R_ENCA 7
#define R_ENCB 8

#define WIRE_PORT Wire
#define AD0_VAL 1


// ============== GLOBAL VARIABLES ==============
Motor left;
Motor right;

Encoder leftEncoder;
Encoder rightEncoder;

Drivetrain drivetrain;

ICM_20948_I2C imu;


// ============== FUNCTION DECLARATIONS ==============
void left_encoder_update();
void right_encoder_update();

void setup() {
  pinMode(L_PIN1, OUTPUT);
  pinMode(L_PIN2, OUTPUT);
  pinMode(L_ENCA, INPUT);
  pinMode(L_ENCB, INPUT);

  pinMode(R_PIN1, OUTPUT);
  pinMode(R_PIN2, OUTPUT);
  pinMode(R_ENCA, INPUT);
  pinMode(R_ENCB, INPUT);

  attachInterrupt(digitalPinToInterrupt(L_ENCA), left_encoder_update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENCA), right_encoder_update, CHANGE);

  attachInterrupt(digitalPinToInterrupt(L_ENCB), left_encoder_update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENCB), right_encoder_update, CHANGE);

  leftEncoder = Encoder(L_ENCA, L_ENCB);
  rightEncoder = Encoder(R_ENCA, R_ENCB);

  double left_pid_constants[] = {LEFT_KP, LEFT_KI, LEFT_KD};
  double right_pid_constants[] = {RIGHT_KP, RIGHT_KI, RIGHT_KD};

  left = Motor(L_PIN1, L_PIN2, &leftEncoder, left_pid_constants);
  right = Motor(R_PIN1, R_PIN2, &rightEncoder, right_pid_constants);

  Serial.begin(9600);
  while (!Serial) {}

  Wire.begin();
  Wire.setClock(400000);

  bool imu_initialized = false;
  while (!imu_initialized) {
    imu.begin(WIRE_PORT, AD0_VAL);
    Serial.print("IMU init status: ");
    Serial.println(imu.statusString());
    if (imu.status != ICM_20948_Stat_Ok) {
      Serial.println("IMU initialization failed! Retrying...");
      delay(1000);
    }
    else {
      imu_initialized = true;
      Serial.println("IMU initialization successful!");
    }
  }

  drivetrain.update();
}

void loop() {
  // Sample path
  drivetrain.driveDistance(12, INCH);
  drivetrain.turn(90);
  drivetrain.driveDistance(12, INCH);

  // Update PID
  drivetrain.update();
}

// ============== FUNCTION DEFINITIONS ==============
void left_encoder_update() {
  leftEncoder.update();
}

void right_encoder_update() {
  rightEncoder.update();
}
