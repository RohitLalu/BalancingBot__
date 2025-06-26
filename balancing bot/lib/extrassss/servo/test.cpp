//TEST CODE

#include <Wire.h>
#include <Arduino.h>
#include <string.h> 
#include "mpu6500.h"

#define MA Serial1       // Motor A (RX1/TX1)
#define MB Serial3       // Motor B (RX3/TX3)
#define debug Serial     // USB Serial for debug

#define MA_BAUD 9600   // Motor A baud rate
#define MB_BAUD 9600   // Motor B baud rate
#define debug_BAUD 9600 // Debug Serial baud rate

// Send a command 
void sendMACommand(const char* cmd) {
  MA.print(cmd);
  MA.write('\r');          // carriage return 
  debug.println();
  debug.print("→MA_WRITE: ");
  debug.println(cmd);
}
void sendMBCommand(const char* cmd) {
  MB.print(cmd);
  MB.write('\r');          
  debug.println();
  debug.print("→MB_WRITE: ");
  debug.println(cmd);
}

// Read any motor reply within timeout (ms)
void readMotorReply(HardwareSerial& motor, unsigned long timeout = 200) {
  unsigned long start = millis();
  String response = "";

  while (millis() - start < timeout) {
    while (motor.available()) {
      char c = motor.read();
      if (c == '\r' || c == '\n') {
        if (response.length() > 0) {
          if (motor == MA) {
            debug.print("→MA_READ: ");
          } else if (motor == MB) {
            debug.print("→MB_READ: ");
          }
          debug.println(response);
          response = "";
        }
      } else {
        response += c;
      }
    }
  }

}

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


//0 : motor MA
//1 : motor MB
void setMotorSpeed(int speed,bool motor) {
  speed = constrain(speed, -200, 200);
  String cmd = "S" + String(speed);
  if (motor == 0) {
    sendMACommand(cmd.c_str());
  } else if (motor == 1) {
    sendMBCommand(cmd.c_str());
  } else {
    debug.println("Invalid motor selection");
    return;
  }
  readMotorReply(motor == 0 ? MA : MB); 

}


// PID control parameters
// PID constants
float Kt = 0;//1.5; // back calculation for integral term
float Kp = 130.7723, Ki = 396.6, Kd = 4.9993; 

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
float factor =0.35; // for PID
float x = 0.0;

void pid(float pitch,float dt) {
  // Emergency stop if tilt too large
  if (abs(pitch) > mid_threshold && abs(pitch) < high_threshold) {
    float sign = pitch/(abs(pitch));
       setMotorSpeed(-0.4*sign*255,0);
       setMotorSpeed(0.4*sign*255,1);
       debug.println("EXCESSIVE TILT");
       return;
     }
    else if (abs(pitch) > high_threshold) {
       setMotorSpeed(0,0);
       setMotorSpeed(0,1);
       Serial.println("EMERGENCY STOP");
       return;
     }
  // PID control
    else if (abs(pitch) > low_threshold && abs(pitch)<mid_threshold) {
      error = (targetAngle - pitch)*PI/180.0; // making it radians
      integral += Ki*(error);
      derivative = (error - lastError) / dt;
      I = integral+ Kt*(x);
      output = Kp * error + I*dt + Kd * derivative;
      output*=(-1*factor); // -1 due to imu orientation
      lastError = error;
      motorSpeed = constrain(output, -255, 255);
      x = output - motorSpeed; // back calculation error for integral term
      setMotorSpeed(-motorSpeed,0);
      setMotorSpeed(motorSpeed,1);
    } 
    else {
       setMotorSpeed(0,0);
       setMotorSpeed(0,1);
       integral = 0;
     }
}


void setup() {
  Wire.begin();
  Wire.setClock(400000); 
  debug.begin(debug_BAUD);
  MA.begin(MA_BAUD);    
  MB.begin(MB_BAUD);    

  imu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
  if (!imu.Begin()) {
    debug.println("Error initializing IMU");
    while (1);
  }
  // Set the sample rate divider
  if (!imu.ConfigSrd(19)) {
    debug.println("Error configuring SRD");
    while (1);
  }

  debug.println("Calibrating...");
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
  
  debug.println("Calibration complete.");
  delay(500);
  debug.print("Accel X Offset: ");
  debug.println(accel_x_offset, 6);
  debug.print("Accel Y Offset: ");
  debug.println(accel_y_offset, 6);
  debug.print("Accel Z Offset (gravity removed): ");
  debug.println(accel_z_offset, 6);
delay(500);
lastTime = millis();
  debug.println("RMCS-2207 Motor Test");
  debug.println("Initialising motors...");

  sendMACommand("S0");
  sendMBCommand("S0");
  delay(500);
  readMotorReply(MA);
  readMotorReply(MB);
  debug.println("Motors initialised.");
}

float ax, ay, az, gx, gy;
float pitchAcc, rollAcc;
float pitch, roll;
void loop() {
if (imu.Read() && calibrated) {
    current_time = millis();
    dt = (current_time - lastTime) / 1000.0;
    lastTime = current_time;

    ax = imu.accel_x_mps2() - accel_x_offset;
    ay = imu.accel_y_mps2() - accel_y_offset;
    az = imu.accel_z_mps2() - accel_z_offset;
    gx = imu.gyro_x_radps() * 180.0/PI;
    gy = imu.gyro_y_radps() * 180.0/PI;
    // Accelerometer-based pitch and roll (degrees)
    pitchAcc = atan2(ay, sqrt(ax*ax + az*az)) * 180.0/PI;
    rollAcc  = atan2(-ax, az) * 180.0/PI;

    // Kalman filter output
    pitch = kalmanPitch.getAngle(pitchAcc, gx, dt);
    roll = kalmanRoll.getAngle(rollAcc, gy, dt);
    // Output
    debug.print("\tPitch: ");
    debug.print(pitch, 2);
    debug.print(" °\tRoll: ");
    debug.print(roll, 2);
    pid(pitch,dt); // (pitch in deg). converted to rad in pid func
    debug.print("\tspeed: ");
    debug.println(motorSpeed);
  }
  

}