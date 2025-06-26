#include <Wire.h>
#include "mpu6500.h"

float factor_speed = 0.8;
// Encoder counts
int countM1 = 0;
int countM2 = 0;

// MPU6500 object
bfs::Mpu6500 imu;
/* Calibration offsets */
float accel_x_offset = 0, accel_y_offset = 0, accel_z_offset = 0;
bool calibrated = false;

// Kalman filter class
class KalmanFilter {
  public:
    float Q_angle = 0.001;
    float Q_bias = 0.003;
    float R_measure = 0.03;

    float angle = 0;
    float bias = 0;
    float rate = 0;

    float P[2][2] = {{0, 0}, {0, 0}};

    float getAngle(float newAngle, float newRate, float dt) {
      rate = newRate - bias;
      angle += dt * rate;

      P[0][0] += dt * (dt*P[1][1] - P[1][0] - P[0][1] + Q_angle);
      P[0][1] -= dt * P[1][1];
      P[1][0] -= dt * P[1][1];
      P[1][1] += Q_bias * dt;

      float y = newAngle - angle;
      float S = P[0][0] + R_measure;
      float K[2];
      K[0] = P[0][0] / S;
      K[1] = P[1][0] / S;

      angle += K[0] * y;
      bias += K[1] * y;

      float P00_temp = P[0][0];
      float P01_temp = P[0][1];

      P[0][0] -= K[0] * P00_temp;
      P[0][1] -= K[0] * P01_temp;
      P[1][0] -= K[1] * P00_temp;
      P[1][1] -= K[1] * P01_temp;

      return angle;
    }
};

KalmanFilter kalmanPitch;
KalmanFilter kalmanRoll;

//PID control constants
float Kp = 81.054, Ki = 604.88, Kd = 2.715;
//  float Kp = 103.7/4, Ki = 1037/8, Kd = 8;//53.924 ;

float targetAngle = 0;
float integral = 0;
float lastError = 0;
float lastTime = 0;

// Motor 1 pins and encoder
const short M1ENCA = 2; 
const short M1ENCB = 3; 
const short ENA = 5;
const short INA = 8;
const short INB = 9;

// Motor 2 pins and encoder
const short M2ENCA = 2; 
const short M2ENCB = 3; 
const short ENB = 6;
const short INC = 10;
const short IND = 11;

// Encoder interrupt handlers
void M1Arising(){
  if(digitalRead(M1ENCB) != digitalRead(M1ENCA)) {
    countM1 ++;
  } else {
    countM1 --;
  }
}
void M1Brising(){
  if(digitalRead(M1ENCB) == digitalRead(M1ENCA)) {
    countM1 ++;
  } else {
    countM1 --;
  }
}
void M2Arising(){
  if(digitalRead(M2ENCB) != digitalRead(M2ENCA)) {
    countM2 ++;
  } else {
    countM2 --;
  }
}
void M2Brising(){
  if(digitalRead(M2ENCB) == digitalRead(M2ENCA)) {
    countM2 ++;
  } else {
    countM2 --;
  }
}

void setupMotors() {
  pinMode(ENA, OUTPUT);
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(INC, OUTPUT);
  pinMode(IND, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(M1ENCA), M1Arising, RISING);
  attachInterrupt(digitalPinToInterrupt(M1ENCB), M1Brising, RISING);
  attachInterrupt(digitalPinToInterrupt(M2ENCA), M2Arising, RISING);
  attachInterrupt(digitalPinToInterrupt(M2ENCB), M2Brising, RISING);
}

void setMotor1Speed(int speed) {
  speed = int(speed*factor_speed);
  speed = constrain(speed, -255, 255);
  Serial.println("Speed M1: ");
  Serial.println(speed);

  if (speed < 0) {digitalWrite(INA, HIGH);digitalWrite(INB, LOW);}
  else if (speed >= 0) {digitalWrite(INA, LOW);digitalWrite(INB, HIGH);} else {digitalWrite(INA, LOW);digitalWrite(INB, LOW);}
  analogWrite(ENA, abs(speed));
}
void setMotor2Speed(int speed) {
  speed = int(speed*factor_speed);
  speed = constrain(speed, -255, 255);
  Serial.println("Speed M1: ");
  Serial.println(speed);

  if (speed < 0) {digitalWrite(INA, HIGH);digitalWrite(INB, LOW);}
  else if (speed >= 0) {digitalWrite(INA, LOW);digitalWrite(INB, HIGH);} else {digitalWrite(INA, LOW);digitalWrite(INB, LOW);}
  analogWrite(ENA, abs(speed));
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  imu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
  if (!imu.Begin()) {
    Serial.println("Error initializing IMU");
    while (1);
  }
  // Set the sample rate divider , srd =19
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configuring SRD");
    while (1);
  }

  Serial.println("Calibrating...");
  float ax_sum = 0, ay_sum = 0, az_sum = 0;
  for (int i = 0; i < 200; i++) {
    while (!imu.Read()) {}
    ax_sum += imu.accel_x_mps2();
    ay_sum += imu.accel_y_mps2();
    az_sum += imu.accel_z_mps2();
    delay(10);
  }
  accel_x_offset = ax_sum / 200;
  accel_y_offset = ay_sum / 200;
  accel_z_offset = az_sum / 200 - 9.81;
  calibrated = true;
  
  Serial.println("Calibration complete.");
  delay(500);
  Serial.println("Accel X Offset: ");
  Serial.println(accel_x_offset, 6);
  Serial.println("Accel Y Offset: ");
  Serial.println(accel_y_offset, 6);
  Serial.println("Accel Z Offset (gravity removed): ");
  Serial.println(accel_z_offset, 6);

  setupMotors();

  lastTime = millis();
}

void loop() {
  if (imu.Read() && calibrated) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    float ax = imu.accel_x_mps2() - accel_x_offset;
    float ay = imu.accel_y_mps2() - accel_y_offset;
    float az = imu.accel_z_mps2() - accel_z_offset;
    float gx = imu.gyro_x_radps() * 180.0 / PI;
    float gy = imu.gyro_y_radps() * 180.0 / PI;
    // Accelerometer-based pitch and roll (degrees)
    float pitchAcc = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
    float rollAcc  = atan2(-ax, az) * 180.0 / PI;

    // Kalman filter output
    float pitch = kalmanPitch.getAngle(pitchAcc, gx, dt);
    float roll = kalmanRoll.getAngle(rollAcc, gy, dt);
    // Output
    Serial.print("Pitch: ");
    Serial.print(pitch, 2);
    Serial.print(" °\tRoll: ");
    Serial.print(roll, 2);
  
  //  Emergency stop if tilt too large
  if (abs(pitch) > 15.0 && abs(pitch) < 25.0) {
    float x = pitch/(abs(pitch));
       setMotor1Speed(-0.9*x*255/factor_speed);
       setMotor2Speed(-0.9*x*255/factor_speed);
       Serial.println("EXCESSIVE TILT");
       return;
     }
     else if (abs(pitch) > 25.0) {
       setMotor1Speed(0);
       setMotor2Speed(0);
       Serial.println("EMERGENCY STOP");
       return;
     }

    float threshold = 4;
    if (abs(pitch) > threshold) {
      float error = targetAngle - pitch;
      integral += error * dt;
      float derivative = (error - lastError) / dt;
      float output = Kp * error + Ki * integral + Kd * derivative;
      lastError = error;
 
      Serial.print("error: ");
      Serial.println(error);
      Serial.println(output);
      int motorSpeed = constrain(output, -255, 255);
      setMotor1Speed(motorSpeed);
      setMotor2Speed(motorSpeed);
      Serial.print("Pitch:");
Serial.print(pitch);
    }  
  else {
     setMotor1Speed(0);
     setMotor1Speed(0);
     integral = 0;
   }
  }
}