//TEST CODE:  DOUBLE MOTOR TEST

#include <Wire.h>
#include <Arduino.h>

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
  debug.print("→MA_WRITE: ");
  debug.println(cmd);
}
void sendMBCommand(const char* cmd) {
  MB.print(cmd);
  MB.write('\r');          
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
          } else {
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

void setup() {
  debug.begin(debug_BAUD);
  MA.begin(MA_BAUD);    // Baud matched to RMCS-2207 UART
  MB.begin(MB_BAUD);    // Baud matched to RMCS-2207 UART
  debug.println("RMCS-2207 Motor Test");
  debug.println("Initialising motors...");

  sendMACommand("S0");
  sendMBCommand("S0");
  delay(500);
  readMotorReply(MA);
}

void loop() {

  sendMACommand("S0");
  sendMBCommand("S0");
  readMotorReply(MA);
  readMotorReply(MB);
  delay(3000);

  sendMACommand("S30");
  sendMBCommand("S30");
  readMotorReply(MA);
  readMotorReply(MB);
  delay(2000);

  sendMACommand("S0");
  sendMBCommand("S0");
  readMotorReply(MA);
  readMotorReply(MB);
  delay(3000);

  sendMACommand("S-30");
  sendMBCommand("S-30");
  readMotorReply(MA);
  readMotorReply(MB);
  delay(2000);

}