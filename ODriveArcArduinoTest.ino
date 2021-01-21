#include <Servo.h>
//#include "SendOnlySoftwareSerial.h"
//------------------------------------------------------------------

//This script only works with the ODrive
//runnning firmware v0.4.12

//------------------------------------------------------------------
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

// Serial to the ODrive
SoftwareSerial odrive_serial(8, 9); //RX (ODrive TX), TX (ODrive RX)
// Note: you must also connect GND on ODrive to GND on Arduino!

// ODrive object
ODriveArduino odrive(odrive_serial);
//------------------------------------------------------------------

// The first digital port that is usable on this controller
#define _PortStart 2

// The last digital port on this controller that is usable
#define _PortCnt 14

// The number of analog ports
#define _AnalogPorts 6

// The firmware version that is reported to EZ-Builder to notify of capabilities
#define _FIRMWARE_ID 0x00000005

// The communication baud rate
#define _BAUD_RATE 115200 //57600

// The primary communication interface between EZ-Builder and this controller
#define COMMUNICATION_PORT Serial

// The amount of RX buffer on the communication interface for EZ-Builder
#define _BUFFER_SIZE 1024

// --------------------------------------------------------------------------

byte         _INPUT_BUFFER[_BUFFER_SIZE];
unsigned int _WRITE_POSITION = 0;
unsigned int _READ_POSITION = 0;

int _BAUD_RATES [] = {
  4800,
  9600,
  19200,
  38400,
  57600,
  115200,
  115200
};

Servo Servos[_PortCnt];

#define CmdUnknown            0
#define CmdReleaseAllServos   1
#define CmdGetUniqueID        2
#define CmdEZBv3              3
#define CmdEZBv4              4
#define CmdSoundBeep          5
#define CmdEZServo            6
#define CmdI2CWrite           10
#define CmdI2CRead            11
#define CmdBootLoader         14
#define CmdSetPWMSpeed        15
#define CmdSetServoSpeed      39
#define CmdPing               0x55
#define CmdSetDigitalPortOn   100
#define CmdSetDigitalPortOff  124
#define CmdGetDigitalPort     148
#define CmdSetServoPosition   172
#define CmdGetADCValue        196
#define CmdSendSerial         204
#define CmdHC_SR04            228
#define CmdGetFirwareID       253
#define CmdSoundStreamCmd     254

// CmdEZBv4 Commands
// ----------------------------------------------------------------------------------
#define CmdV4SetLipoBatteryProtectionState 0
#define CmdV4SetBatteryMonitorVoltage      1
#define CmdV4GetBatteryVoltage             2
#define CmdV4GetCPUTemp                    3

#define CmdV4UARTExpansion0Init            5
#define CmdV4UARTExpansion0Write           6
#define CmdV4UARTExpansion0AvailableBytes  7
#define CmdV4UARTExpansion0Read            8

#define CmdV4UARTExpansion1Init            9
#define CmdV4UARTExpansion1Write           10
#define CmdV4UARTExpansion1AvailableBytes  11
#define CmdV4UARTExpansion1Read            12

#define CmdV4UARTExpansion2Init            13
#define CmdV4UARTExpansion2Write           14
#define CmdV4UARTExpansion2AvailableBytes  15
#define CmdV4UARTExpansion2Read            16

#define CmdV4I2CClockSpeed                 17
#define CmdV4UARTClockSpeed                18
#define CmdV4ResetToDefaults               19

// CmdSoundStreamCmd Commands
// ----------------------------------------------------------------------------------
#define CmdSoundInitStop 0
#define CmdSoundLoad     1
#define CmdSoundPlay     2

bool IsAvail() {

  return _WRITE_POSITION != _READ_POSITION || COMMUNICATION_PORT.available();
}

byte ReadByte() {

  while (_WRITE_POSITION == _READ_POSITION && COMMUNICATION_PORT.available() == 0);

  while (COMMUNICATION_PORT.available()) {

    _WRITE_POSITION++;

    _INPUT_BUFFER[_WRITE_POSITION % _BUFFER_SIZE] = COMMUNICATION_PORT.read();
  }

  _READ_POSITION++;

  return _INPUT_BUFFER[_READ_POSITION % _BUFFER_SIZE];
}

void setup() {
  odrive_serial.begin(115200);
  COMMUNICATION_PORT.begin(_BAUD_RATE);
  odrive.SetPosition(0, 0);
  delay(5);
}

void loop() {

  doEZProtocol();
}

void Write32(long val) {

  COMMUNICATION_PORT.write((byte)(val & 0xff));
  COMMUNICATION_PORT.write((byte)((val >> 8) & 0xff));
  COMMUNICATION_PORT.write((byte)((val >> 16) & 0xff));
  COMMUNICATION_PORT.write((byte)((val >> 24) & 0xff));
}

void Write16(int val) {

  COMMUNICATION_PORT.write((byte)(val & 0xff));
  COMMUNICATION_PORT.write((byte)((val >> 8) & 0xff));
}

#define UNKNOWN_PIN 0xFF

uint8_t getPinMode(uint8_t pin) {

  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);

  // I don't see an option for mega to return this, but whatever...
  if (NOT_A_PIN == port)
    return UNKNOWN_PIN;

  // Is there a bit we can check?
  if (0 == bit)
    return UNKNOWN_PIN;

  // Is there only a single bit set?
  if (bit & bit - 1)
    return UNKNOWN_PIN;

  volatile uint8_t *reg, *out;
  reg = portModeRegister(port);
  out = portOutputRegister(port);

  if (*reg & bit)
    return OUTPUT;
  else if (*out & bit)
    return INPUT_PULLUP;
  else
    return INPUT;
}

void doEZProtocol() {

  if (IsAvail()) {

    byte cmd = ReadByte();

    if (cmd == CmdPing) {

      // return as a "Capability Controller"
      COMMUNICATION_PORT.write(222);
    } else if (cmd == CmdGetFirwareID) {

      Write32(_FIRMWARE_ID);
    } else if (cmd == CmdReleaseAllServos) {

      for (int port = _PortStart; port < _PortCnt; port++)
        if (Servos[port].attached())
          Servos[port].detach();
    } else if (cmd >= CmdSetServoPosition && cmd <= CmdSetServoPosition + 23) {

      byte port = cmd - CmdSetServoPosition;
      byte pos = ReadByte();
      //odrive.SetPosition(0, pos*1000);
      odrive.SetPosition(0, map(pos, 0, 100, 0, 180000));
      delay(5);
      if (port >= _PortStart && port <= _PortCnt) {

        if (pos == 0 && Servos[port].attached()) {

          Servos[port].detach();
        } else {

          if (!Servos[port].attached())
            Servos[port].attach(port);

          Servos[port].write(pos);
        }
      }
    } else if (cmd >= CmdSetPWMSpeed && cmd <= CmdSetPWMSpeed + 23) {

      byte port = cmd - CmdSetPWMSpeed;
      byte pos = ReadByte();

      if (port >= _PortStart && port <= _PortCnt) {

        if (Servos[port].attached())
          Servos[port].detach();

        if (getPinMode(port) != OUTPUT)
          pinMode(port, OUTPUT);

        analogWrite(port, map(pos, 0, 100, 0, 255));
      }
    } else if (cmd >= CmdSetDigitalPortOn && cmd <= CmdSetDigitalPortOn + 23) {

      byte port = cmd - CmdSetDigitalPortOn;

      if (port >= _PortStart && port <= _PortCnt) {

        if (Servos[port].attached())
          Servos[port].detach();

        if (getPinMode(port) != OUTPUT)
          pinMode(port, OUTPUT);

        digitalWrite(port, HIGH);
      }
    } else if (cmd >= CmdSetDigitalPortOff && cmd <= CmdSetDigitalPortOff + 23) {

      byte port = cmd - CmdSetDigitalPortOff;

      if (port >= _PortStart && port <= _PortCnt) {

        if (Servos[port].attached())
          Servos[port].detach();

        if (getPinMode(port) != OUTPUT)
          pinMode(port, OUTPUT);

        digitalWrite(port, LOW);
      }
    } else if (cmd >= CmdGetDigitalPort && cmd <= CmdGetDigitalPort + 23) {

      byte port = cmd - CmdGetDigitalPort;

      if (port >= _PortStart && port <= _PortCnt) {

        if (Servos[port].attached())
          Servos[port].detach();

        if (getPinMode(port) != INPUT)
          pinMode(port, INPUT);

        COMMUNICATION_PORT.write(digitalRead(port));
      } else {

        COMMUNICATION_PORT.write((byte)0);
      }
    } else if (cmd >= CmdGetADCValue && cmd <= CmdGetADCValue + 7) {

      byte port = cmd - CmdGetADCValue;

      if (port >= 0 && port <= _AnalogPorts)
        Write16(analogRead(port));
      else
        Write16(0);
    }  else if (cmd >= CmdHC_SR04 && cmd < CmdHC_SR04 + 23) {

      uint8_t triggerPort = cmd - CmdHC_SR04;
      uint8_t echoPort = ReadByte();
      uint8_t distance = 0;

      if ((triggerPort >= _PortStart && triggerPort <= _PortCnt) && (echoPort >= _PortStart && echoPort <= _PortCnt)) {

        if (getPinMode(triggerPort) != OUTPUT)
          pinMode(triggerPort, OUTPUT);

        if (getPinMode(echoPort) != INPUT)
          pinMode(echoPort, INPUT);
        
        digitalWrite(triggerPort, LOW);
        delayMicroseconds(2);
        digitalWrite(triggerPort, HIGH);
        delayMicroseconds(10);
        digitalWrite(triggerPort, LOW);

        // Reads the echoPin, returns the sound wave travel time in microseconds
        // Converts to CM
        distance = pulseIn(echoPort, HIGH, 30000) / 58.0;
      }

      COMMUNICATION_PORT.write(distance);
    }
  }
}
