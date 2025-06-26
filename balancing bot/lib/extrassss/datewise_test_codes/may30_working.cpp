#include <Wire.h>
#include "mpu6500.h"


// MPU6500 object
bfs::Mpu6500 imu;
// Calibration offsets
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


// Motor 0 pins and encoder
//const short M1ENCA = 2; 
//const short M1ENCB = 3; 
const short ENA = 5;
const short INA = 8;
const short INB = 9;

// Motor 1 pins and encoder
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


//0 : ena motor
//1 : enb motor
void setMotorSpeed(int speed,bool motor) {
  speed = int(speed);
  speed = constrain(speed, -255, 255);

  if (motor){
  if (speed < 0) {
    digitalWrite(INA, HIGH);
    digitalWrite(INB, LOW);
  } else {
    digitalWrite(INA, LOW);
    digitalWrite(INB, HIGH);
  } 
  analogWrite(ENA, abs(speed));
}
else {
  if (speed < 0) {
    digitalWrite(INC, HIGH);
    digitalWrite(IND, LOW);
  } else{
    digitalWrite(INC, LOW);
    digitalWrite(IND, HIGH);
  } 
  analogWrite(ENB, abs(speed));
}
}

// PID control parameters
// PID constants
float Kt = 0;//1.5; // back calculation for integral term
float Kp = 838.5, Ki = 700.2, Kd = -0.2651; 

// Angle Errors
float targetAngle = 0.0;
float error =0.0;
float lastError = 0;

// Time
unsigned int current_time = 0;
unsigned int lastTime = 0;
float dt =0;

// I/D components
float derivative =0;
float integral = 0;
float I =0;
float output =0;
int motorSpeed = 0;

// Angle thresholds
float low_threshold = 1.7; 
float mid_threshold = 10.0;
float high_threshold = 25.0;

//float factor = 7.2; // for PI
float factor =1; // for PID
float x = 0.0;

void pid(float pitch,float dt) {
  // Emergency stop if tilt too large
  if (abs(pitch) > mid_threshold && abs(pitch) < high_threshold) {
    float sign = pitch/(abs(pitch));
       setMotorSpeed(0.8*sign*255,0);
       setMotorSpeed(0.8*sign*255,1);
       Serial.println("EXCESSIVE TILT");
       return;
     }
     else if (abs(pitch) > high_threshold) {
       setMotorSpeed(0,0);
       setMotorSpeed(0,1);
       Serial.println("EMERGENCY STOP");
       return;
     }
  // PID control
    if (abs(pitch) > low_threshold && abs(pitch)<mid_threshold) {
      error = (targetAngle - pitch)*PI/180.0; // making it radians
      integral += Ki*(error);
      derivative = (error - lastError) / dt;
      I = integral+ Kt*(x);
      output = Kp * error + I*dt + Kd * derivative;
      output*=(-1*factor); // -1 due to imu orientation
      lastError = error;
      motorSpeed = constrain(output, -255, 255);
      x = output - motorSpeed; // back calculation error for integral term
      setMotorSpeed(motorSpeed,0);
      setMotorSpeed(motorSpeed,1);
    }
    else {
       setMotorSpeed(0,0);
       setMotorSpeed(0,1);
       integral = 0;
     }
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
  // Set the sample rate divider
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configuring SRD");
    while (1);
  }

  Serial.println("Calibrating...");
  float ax_sum = 0, ay_sum = 0, az_sum = 0;
  for (int i = 0; i < 200; i++) {
    while (!imu.Read()) {}
    ax_sum+= imu.accel_x_mps2();
    ay_sum+= imu.accel_y_mps2();
    az_sum+= imu.accel_z_mps2();
    delay(10);
  }
  accel_x_offset = ax_sum/200;
  accel_y_offset = ay_sum/200;
  accel_z_offset = az_sum/200 - 9.81;
  calibrated = true;
  
  Serial.println("Calibration complete.");
  delay(500);
  Serial.print("Accel X Offset: ");
  Serial.println(accel_x_offset, 6);
  Serial.print("Accel Y Offset: ");
  Serial.println(accel_y_offset, 6);
  Serial.print("Accel Z Offset (gravity removed): ");
  Serial.println(accel_z_offset, 6);
delay(500);
lastTime = millis();
}

void loop() {
  if (imu.Read() && calibrated) {
    current_time = millis();
    dt = (current_time - lastTime) / 1000.0;
    lastTime = current_time;

    float ax = imu.accel_x_mps2() - accel_x_offset;
    float ay = imu.accel_y_mps2() - accel_y_offset;
    float az = imu.accel_z_mps2() - accel_z_offset;
    float gx = imu.gyro_x_radps() * 180.0/PI;
    float gy = imu.gyro_y_radps() * 180.0/PI;
    // Accelerometer-based pitch and roll (degrees)
    float pitchAcc = atan2(ay, sqrt(ax*ax + az*az)) * 180.0/PI;
    float rollAcc  = atan2(-ax, az) * 180.0/PI;

    // Kalman filter output
    float pitch = kalmanPitch.getAngle(pitchAcc, gx, dt);
    float roll = kalmanRoll.getAngle(rollAcc, gy, dt);
        // Output
    Serial.print("\tPitch: ");
    Serial.print(pitch, 2);
    Serial.print(" °\tRoll: ");
    Serial.print(roll, 2);
    pid(pitch,dt); // (pitch in deg). converted to rad in pid func
    Serial.print("\tspeed: ");
    Serial.println(motorSpeed);
  }
}