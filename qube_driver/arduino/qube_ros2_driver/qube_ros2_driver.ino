#include "QUBE.hpp"

QUBE qube;

byte inputBuffer[10];

void setup() {
  // Set up serial communication
  Serial.begin(115200);
  // Set up communication with Qube
  qube.begin();
  qube.resetMotorEncoder();
  qube.resetPendulumEncoder();
  qube.setMotorVoltage(0);
  qube.setRGB(0, 999, 999);
  qube.update();
}

// Receives the data from the computer
bool receiveData() {
  if (Serial.available() > 2) {  // Check if there is data available
    int motorCommand_MSB = Serial.read();
    int motorCommand_LSB = Serial.read();
    // Combine the two bytes to get the motor command
    int motorCommand = (motorCommand_MSB << 8) + motorCommand_LSB - 999;

    // Set the motor speed
    qube.setMotorSpeed(motorCommand);
    return true;
  }
  return false;
}

// Sends the encoder data through the serial port
void sendEncoderData(bool encoder) {
  // Get the encoder angle
  float encoderAngle = qube.getMotorAngle(false);
  // Get the number of revolutions
  long revolutions = (long)encoderAngle/360.0;

  // Get the relative angle
  float _angle = encoderAngle - revolutions*360.0;
  long angle = (long)_angle; // represent the angle in integer value
  long angleDecimal = (_angle - angle) * 100; // represents the two decimal digits .xx

  // Set the direction bit
  if (encoderAngle < 0) { 
    revolutions = abs(revolutions);
    angle = abs(angle);
    angleDecimal = abs(angleDecimal);

    revolutions |= (1<<15); // 0bx000 0000 0000 0000 - MSB is sign (1 means negative), bits (14-0) are revolution count
  }
  // Combine the angle and angleDecimal
  angle = (angle << 7) | angleDecimal; // angle range is from 0-360 an requires 9 bits, angleDecimal is from 0-100 and requires 7 bits. Bits (15-7) represent int, bits (6-0) represent dec.

  // Encode the data
  byte ang_MSB = angle >> 8;
  byte ang_LSB = angle & 0xFF;

  Serial.write(highByte(revolutions));
  Serial.write(lowByte(revolutions));
  Serial.write(ang_MSB);
  Serial.write(ang_LSB);
}

// Sends the RPM data through the serial port
void sendRPMData() {
  // Get the RPM value
  long rpm = (long)qube.getRPM();
  // Set the direction bit
  bool dir = rpm < 0;

  // If the RPM is negative, make it positive and set the direction bit
  if (dir) {
    rpm = abs(rpm);
    rpm |= 1 << 15;
  }

  // Encode the RPM value
  byte rpm_MSB = rpm >> 8;
  byte rpm_LSB = rpm & 0xFF;

  // Send the data
  Serial.write(rpm_MSB);
  Serial.write(rpm_LSB);
}

// Sends data to the computer
void sendData() {
  sendEncoderData(0);
  sendRPMData();
}

void loop() {
  qube.update();

  bool received = false;
  while (!received) {
    received = receiveData();
  }
  sendData();
}
