
#include <Wire.h>
#include "mpu6500.h"

float factor_speed = 0.8;

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


// Motor 1 pins and encoder
//const short M1ENCA = 2; 
//const short M1ENCB = 3; 
const short ENA = 5;
const short INA = 8;
const short INB = 9;

// Motor 2 pins and encoder
//const short M2ENCA = 2; 
//const short M2ENCB = 3; 
const short ENB = 6;
const short INC = 10;
const short IND = 11;

void setupMotors() {
  pinMode(ENA, OUTPUT);
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(INC, OUTPUT);
  pinMode(IND, OUTPUT);
}

void setMotorSpeed(int speed) {
  speed = int(speed*factor_speed);
  speed = constrain(speed, -255, 255);
  Serial.println("\tspeed: ");
  Serial.println(speed);

  if (speed < 0) {
    digitalWrite(INA, HIGH);
    digitalWrite(INB, LOW);
    digitalWrite(INC, HIGH);
    digitalWrite(IND, LOW);
  } else if (speed > 0) {
    digitalWrite(INA, LOW);
    digitalWrite(INB, HIGH);
    digitalWrite(INC, LOW);
    digitalWrite(IND, HIGH);
  } else {
    digitalWrite(INA, LOW);
    digitalWrite(INB, LOW);
    digitalWrite(INC, LOW);
    digitalWrite(IND, LOW);
  }
  analogWrite(ENA, abs(speed));
  analogWrite(ENB, abs(speed));
}
float lastTime = 0;
void setup() {
  
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  imu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
  if (!imu.Begin()) {
    Serial.println("Error initializing IMU");
    while (1);
  }
  /* Set the sample rate divider */
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
  Serial.print("Accel X Offset: ");
  Serial.println(accel_x_offset, 6);
  Serial.print("Accel Y Offset: ");
  Serial.println(accel_y_offset, 6);
  Serial.print("Accel Z Offset (gravity removed): ");
  Serial.println(accel_z_offset, 6);
  //setupMotors();
  lastTime = millis();
}

// PID control parameters
// PID constants
float Kp = 632.97, Ki = 2434.5, Kd = 41.14;
// float Kp = 81.054, Ki = 604.88, Kd = 2.715;
//  float Kp = 103.7/4, Ki = 1037/8, Kd = 8;//53.924 ;

float targetAngle = 0;
float integral = 0;
float lastError = 0;
float threshold = 1.7; 
float factor = 10;

void pid(float pitch,float dt) {
  // Emergency stop if tilt too large
  if (abs(pitch) > 10.0 && abs(pitch) < 25.0) {
    float x = pitch/(abs(pitch));
       setMotorSpeed(-0.9*x*255/factor_speed);
       Serial.println("EXCESSIVE TILT");
       return;
     }
     else if (abs(pitch) > 25.0) {
       setMotorSpeed(0);
       Serial.println("EMERGENCY STOP");
       return;
     }
  // PID control
    if (abs(pitch) > threshold && abs(pitch)<10.0) {
      float error = (targetAngle - pitch)*PI/180.0;
      integral += error * dt;
      float derivative = (error - lastError) / dt;
      float output = Kp * error + Ki * integral + Kd * derivative;
      output*=(factor);
      lastError = error;
      int motorSpeed = constrain(output, -255, 255);

      if (abs(integral) >10000){integral = 0;motorSpeed *=-1;}
      setMotorSpeed(motorSpeed);
      }  
  else {
     setMotorSpeed(0);
     integral = 0;
   }
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
    Serial.print("dt:");
    Serial.print(dt, 4);
    Serial.print("\tPitch: ");
    Serial.print(pitch, 2);
    Serial.print(" °\tRoll: ");
    Serial.print(roll, 2);
  
  pid(pitch,dt);
     
  }
}

// #include <Wire.h>
// #include "mpu6500.h"

// float factor_speed = 0.8;

// // MPU6500 object
// bfs::Mpu6500 imu;
// /* Calibration offsets */
// float accel_x_offset = 0, accel_y_offset = 0, accel_z_offset = 0;
// bool calibrated = false;

// // Kalman filter class
// class KalmanFilter {
//   public:
//     float Q_angle = 0.001;
//     float Q_bias = 0.003;
//     float R_measure = 0.03;

//     float angle = 0;
//     float bias = 0;
//     float rate = 0;

//     float P[2][2] = {{0, 0}, {0, 0}};

//     float getAngle(float newAngle, float newRate, float dt) {
//       rate = newRate - bias;
//       angle += dt * rate;

//       P[0][0] += dt * (dt*P[1][1] - P[1][0] - P[0][1] + Q_angle);
//       P[0][1] -= dt * P[1][1];
//       P[1][0] -= dt * P[1][1];
//       P[1][1] += Q_bias * dt;

//       float y = newAngle - angle;
//       float S = P[0][0] + R_measure;
//       float K[2];
//       K[0] = P[0][0] / S;
//       K[1] = P[1][0] / S;

//       angle += K[0] * y;
//       bias += K[1] * y;

//       float P00_temp = P[0][0];
//       float P01_temp = P[0][1];

//       P[0][0] -= K[0] * P00_temp;
//       P[0][1] -= K[0] * P01_temp;
//       P[1][0] -= K[1] * P00_temp;
//       P[1][1] -= K[1] * P01_temp;

//       return angle;
//     }
// };

// KalmanFilter kalmanPitch;
// KalmanFilter kalmanRoll;

// // PID constants
// float Kp = 81.054, Ki = 604.88, Kd = 2.715;
// //  float Kp = 103.7/4, Ki = 1037/8, Kd = 8;//53.924 ;

//  float targetAngle = 0;
// float integral = 0;
// float lastError = 0;
// float lastTime = 0;

// // Motor 1 pins and encoder
// //const short M1ENCA = 2; 
// //const short M1ENCB = 3; 
// const short ENA = 5;
// const short IN1 = 8;
// const short IN2 = 9;

// // Motor 2 pins and encoder
// //const short M2ENCA = 2; 
// //const short M2ENCB = 3; 
// const short ENB = 6;
// const short IN3 = 10;
// const short IN4 = 11;

// void setupMotors() {
//   pinMode(ENA, OUTPUT);
//   pinMode(IN1, OUTPUT);
//   pinMode(IN2, OUTPUT);
//   pinMode(ENB, OUTPUT);
//   pinMode(IN3, OUTPUT);
//   pinMode(IN4, OUTPUT);
// }

// void setMotorSpeed(float speed) {
//   speed = int(speed*factor_speed);
//   speed = constrain(speed, -255, 255);
//   Serial.println("\tspeed: ");
//   Serial.println(speed);

//   if (speed < 0) {
//     digitalWrite(IN1, HIGH);
//     digitalWrite(IN2, LOW);
//     digitalWrite(IN3, HIGH);
//     digitalWrite(IN4, LOW);
//   } else if (speed > 0) {
//     digitalWrite(IN1, LOW);
//     digitalWrite(IN2, HIGH);
//     digitalWrite(IN3, LOW);
//     digitalWrite(IN4, HIGH);
//   } else {
//     digitalWrite(IN1, LOW);
//     digitalWrite(IN2, LOW);
//     digitalWrite(IN3, LOW);
//     digitalWrite(IN4, LOW);
//   }
//   analogWrite(ENA, abs(speed));
//   analogWrite(ENB, abs(speed));
// }

// void setup() {
//   Serial.begin(115200);
//   Wire.begin();
//   Wire.setClock(400000);

//   imu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
//   if (!imu.Begin()) {
//     Serial.println("Error initializing IMU");
//     while (1);
//   }
//   /* Set the sample rate divider */
//   if (!imu.ConfigSrd(19)) {
//     Serial.println("Error configuring SRD");
//     while (1);
//   }

//   Serial.println("Calibrating...");
//   float ax_sum = 0, ay_sum = 0, az_sum = 0;
//   for (int i = 0; i < 200; i++) {
//     while (!imu.Read()) {}
//     ax_sum += imu.accel_x_mps2();
//     ay_sum += imu.accel_y_mps2();
//     az_sum += imu.accel_z_mps2();
//     delay(10);
//   }
//   accel_x_offset = ax_sum / 200;
//   accel_y_offset = ay_sum / 200;
//   accel_z_offset = az_sum / 200 - 9.81;
//   calibrated = true;
  
//   Serial.println("Calibration complete.");
//   delay(500);
//   Serial.print("Accel X Offset: ");
//   Serial.println(accel_x_offset, 6);
//   Serial.print("Accel Y Offset: ");
//   Serial.println(accel_y_offset, 6);
//   Serial.print("Accel Z Offset (gravity removed): ");
//   Serial.println(accel_z_offset, 6);
//   //setupMotors();
//   lastTime = millis();
// }

// void loop() {
//   if (imu.Read() && calibrated) {
//     unsigned long now = millis();
//     float dt = (now - lastTime) / 1000.0;
//     lastTime = now;

//     float ax = imu.accel_x_mps2() - accel_x_offset;
//     float ay = imu.accel_y_mps2() - accel_y_offset;
//     float az = imu.accel_z_mps2() - accel_z_offset;
//     float gx = imu.gyro_x_radps() * 180.0 / PI;
//     float gy = imu.gyro_y_radps() * 180.0 / PI;
//     // Accelerometer-based pitch and roll (degrees)
//     float pitchAcc = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
//     float rollAcc  = atan2(-ax, az) * 180.0 / PI;

//     // Kalman filter output
//     float pitch = kalmanPitch.getAngle(pitchAcc, gx, dt);
//     float roll = kalmanRoll.getAngle(rollAcc, gy, dt);
//         // Output
//     Serial.print("Pitch: ");
//     Serial.print(pitch, 2);
//     Serial.print(" °\tRoll: ");
//     Serial.print(roll, 2);
//   // // Emergency stop if tilt too large
//   if (abs(pitch) > 8.5 && abs(pitch) < 25.0) {
//     float x = pitch/(abs(pitch));
//        setMotorSpeed(-0.9*x*255/factor_speed);
//        Serial.println("EXCESSIVE TILT");
//        return;
//      }
//      else if (abs(pitch) > 25.0) {
//        setMotorSpeed(0);
//        Serial.println("EMERGENCY STOP");
//        return;
//      }

//     float threshold = 4; 
//     if (abs(pitch) > threshold && abs(pitch)<8.5) {
//       float error = targetAngle - pitch;
//       integral += error * dt;
//       float derivative = (error - lastError) / dt;
//       //loat output = Kp * error + Ki * integral + Kd * derivative;
//       //output /=5;
//       pitch = pitch *PI/180;
//       pitch *=7;
//       float gx = pitch/dt;
//       float output = - (69.60 * pitch/2 + 5.14 * gx);
//       lastError = error;
 
//       int motorSpeed = constrain(output,-255,255);
//       if (abs(motorSpeed)==255) motorSpeed/=factor_speed;
//       setMotorSpeed(motorSpeed);
     
//     }  
//   else {
//      setMotorSpeed(0);
//      integral = 0;
//    }
   
//   }
  
// }