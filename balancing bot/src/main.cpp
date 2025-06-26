#include <Wire.h>
#include "mpu6500.h"

#define ENA 2 //motor 0 (right)
#define INA 12
#define INB 11
#define ENB 3 //motor 1(left)
#define INC 6
#define IND 7
#define pin_left 49
#define pin_mid 53
#define pin_right 51

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

void setupMotors() {
  pinMode(ENA, OUTPUT);
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(INC, OUTPUT);
  pinMode(IND, OUTPUT);
}

void setIRs_line(){
  pinMode(pin_left, INPUT);
  pinMode(pin_mid, INPUT);
  pinMode(pin_right, INPUT);
  delay(50);
}
int line_checker_ir(bool ir_left,bool ir_mid,bool ir_right){
  if(ir_left==0 && ir_mid==0 && ir_right==1){return 4;} // turn sharp right
  else if(ir_left==0 && ir_mid==1 && ir_right==0){return 1;} // go straight
  else if(ir_left==0 && ir_mid==1 && ir_right==1){return 5;} // turn right
  else if(ir_left==1 && ir_mid==0 && ir_right==0){return 2;} // turn sharp left
  else if(ir_left==1 && ir_mid==1 && ir_right==0){return 3;} // turn left
  else{return 0;} // stop
}

//0 : ena motor (right motor)
//1 : enb motor (left motor)
void setMotorSpeed(int speed,bool motor) {
  speed = int(speed);
  speed = constrain(speed, -250, 250);

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
int left_or_right = 1; //1 or -1 based on testing
int straight_speed = 60; // Speed for straight movement
int low_turn_speed = 75; // Speed for turns
int  sharp_turn_speed = 75; // Speed for sharp turns
int motorSpeed = 0;
int delayTime = 3600;

int left_speed=0;
int right_speed=0;

void motor_speed_ctrl(int line_case,int motorSpeed){
  switch(line_case) {
    case 0: // stability
    left_speed =motorSpeed;
    right_speed =motorSpeed;
    break;
    case 1: // straight
    left_speed = motorSpeed + (left_or_right * straight_speed);
    right_speed = motorSpeed + (left_or_right * straight_speed);
    break;
    case 2: // turn right
    left_speed = motorSpeed + (left_or_right * low_turn_speed);
    right_speed = motorSpeed - (left_or_right * low_turn_speed);
    break;
    case 3: // turn sharp right
    left_speed = motorSpeed + (left_or_right * sharp_turn_speed);
    right_speed = motorSpeed - (left_or_right * sharp_turn_speed);
    break;
    case 4: // turn left
    left_speed = motorSpeed - (left_or_right * low_turn_speed);
    right_speed = motorSpeed + (left_or_right * low_turn_speed);
    break;
    case 5: // turn sharp left
    left_speed = motorSpeed - (left_or_right * sharp_turn_speed);
    right_speed = motorSpeed + (left_or_right * sharp_turn_speed);
    break;
    default: // stability
    left_speed = motorSpeed;
    right_speed = motorSpeed;
    break;
  }
  setMotorSpeed(left_speed,1);
  setMotorSpeed(right_speed,0);
  delayMicroseconds(delayTime);
}

// PID control parameters
float Kt = 0;//1.5; // anti windup term
//float Kp = 49.65/1.2, Ki = 278.91/2, Kd = 1.5; //no battery case
//float Kp = 622.612, Ki = 740.048, Kd = 1.0; // battery up case
//float Kp = 286.133, Ki = 596.11, Kd = 32.84/1.7; //battery down case

//testing
float Kp = 1200.0, Ki = 1500.0, Kd = 18.0; 

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


// Angle thresholds
int tilt_marker =0; //0: no tilt, 1: tilt detected
int debug_reader =0;
float low_threshold = 1.2; //1.7; 
float low2_threshold = 2.5; // for PI
float mid_threshold = 8.5;
float high_threshold = 17.5;
float highest_threshold = 35.0; // for emergency stop

float factor =1.0; 
float x = 0.0;
short unsigned int first_max_speed = 230; 
short unsigned int final_max_speed = 250; 

void debug_print(int debug_reader){
switch(debug_reader){
  case 0: Serial.println("\tSTABILITY"); break;
  case 1: Serial.println("\tGOING STRAIGHT"); break;
  case 3: Serial.println("\tTURNING SHARP RIGHT"); break;
  case 2: Serial.println("\tTURNING RIGHT"); break;
  case 5: Serial.println("\tTURNING SHARP LEFT"); break; 
  case 4: Serial.println("\tTURNING LEFT"); break;
  default: Serial.println("\tINVALID CASE: STABILITY"); break;
}
}

void pid(float pitch,float dt) {
  // Emergency stop if tilt too large
  if (abs(pitch) > mid_threshold && abs(pitch) < high_threshold) {
    tilt_marker = 1;
    float sign = pitch/(abs(pitch));
    motorSpeed = sign*first_max_speed;
    delayTime = 2900;
    Serial.println("\tEXCESSIVE TILT 1");
  }
  else if (abs(pitch) > high_threshold && abs(pitch) < highest_threshold) {
    tilt_marker = 1;
    float sign = pitch/(abs(pitch));
    motorSpeed = sign*final_max_speed;
    delayTime = 2900;
    Serial.println("\tEXCESSIVE TILT 2");
  }
  else if (abs(pitch) > highest_threshold) {
    tilt_marker = 1;
    motorSpeed = 0;
    delayTime = 2000;
    Serial.println("\tEMERGENCY STOP");
     }
  // PID control
  else if (abs(pitch) > low_threshold && abs(pitch)<mid_threshold) {
    tilt_marker = 0;
    delayTime = 3600;
    if(pitch <0) {factor =1.0;}
    else {factor = 1.0;}
    if (abs(pitch) > low2_threshold || (pitch<0)) 
      {
        error = (targetAngle - pitch)*PI/180.0; // making it radians
      integral += Ki*(error);
      derivative = (error - lastError) / dt;
      I = integral+ Kt*(x);
      output = Kp * error + I*dt + Kd * derivative;
      output*=(-1*factor); // -1 due to imu orientation
      lastError = error;
      motorSpeed = constrain(output, -250, 250);
      //x = output - motorSpeed; // back calculation error for integral term
      }
      else motorSpeed = 0;
    }
  else {
    tilt_marker = 0;
    delayTime = 3600;
    motorSpeed = 0;
    integral = 0;
     }
}

void setup() {
  setIRs_line();
  setupMotors();
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
    dt = (current_time - lastTime) / 1000.0; //seconds
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
    float pitch = kalmanPitch.getAngle(pitchAcc, gx, dt); // Adjusted for initial tilt
    float roll = kalmanRoll.getAngle(rollAcc, gy, dt);
    // Output
    Serial.print("\tPitch: ");
    Serial.print(pitch, 3);
    Serial.print(" °\tRoll: ");
    Serial.print(roll, 2);
    pid(pitch,dt); // (pitch in deg). converted to rad in pid func
    if (tilt_marker ==1){
      debug_reader = 0; 
    }
    else {
      debug_reader = line_checker_ir(digitalRead(pin_left),digitalRead(pin_mid),digitalRead(pin_right));
    }
    motor_speed_ctrl(debug_reader,motorSpeed);
    Serial.print("\tleft speed: ");
    Serial.print(left_speed);
    Serial.print("\tright speed: ");
    Serial.print(right_speed);
    debug_print(debug_reader);
    Serial.println();
  }
}

