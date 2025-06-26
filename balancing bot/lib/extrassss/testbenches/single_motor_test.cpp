//TEST CODE: SINGLE MOTOR TEST

#include <Wire.h>
#include <Arduino.h>

#define MA Serial1       // Motor A (RX1/TX1)
#define debug Serial     // USB Serial for debug
#define MA_BAUD 9600   // Motor A baud rate
#define debug_BAUD 9600 // Debug Serial baud rate

// Send a command string + single CR terminator
void sendCommand(HardwareSerial& motor, const char* cmd) {
  motor.print(cmd);
  motor.write('\r');          // carriage return only
  debug.print("MA_WRITE: ");
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
          debug.print("→MA_READ: ");
          debug.println(response);
          response = "";
        }
      } else {
        response += c;
      }
    }
  }

}


void setup() {
  debug.begin(debug_BAUD); // Initialize USB Serial for debug output
  MA.begin(MA_BAUD);    // Baud matched to RMCS-2207 UART
  debug.println("RMCS-2207 Motor Test");
  debug.println("Initializing motors...");

  sendCommand(MA, "EN");
  readMotorReply(MA);
  sendCommand(MA, "S0");
  readMotorReply(MA);
  delay(500);
  readMotorReply(MA);
}

void loop() {

  sendCommand(MA, "S-30");
  readMotorReply(MA);
  delay(3000);

  sendCommand(MA, "S0");
  readMotorReply(MA);
  delay(2000);

  sendCommand(MA, "S30");
  readMotorReply(MA);
  delay(3000);

  sendCommand(MA, "S0");
  readMotorReply(MA);
  delay(2000);

}