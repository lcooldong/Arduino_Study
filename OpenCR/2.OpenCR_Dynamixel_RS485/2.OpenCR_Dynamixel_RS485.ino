#include <Dynamixel2Arduino.h>

#define USER1_LED   22  // SINK (LOW -> TUNR ON)
#define USER2_LED   23  // SINK
#define USER3_LED   24  // SINK
#define USER4_LED   25  // SINK
#define STATUS_LED  36  // SINK, Upload 할 때 깜빡임
#define ARDUINO_LED 13  // Source (HIGH -> TURN ON)

#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.

int ledPins[] = {USER1_LED, USER2_LED, USER3_LED, USER4_LED, STATUS_LED, ARDUINO_LED};
const int ledCount = sizeof(ledPins)/ sizeof(ledPins[0]);
bool ledStatus[ledCount] = {0,};
int ledCurrent = 0;

unsigned long prevMillis = 0;
unsigned long pingMillis = 0;

const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
bool dxlLedStatus = false;

using namespace ControlTableItem;

void setup() {

  DEBUG_SERIAL.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(DXL_ID);

  pinMode(USER1_LED, OUTPUT);
  pinMode(USER2_LED, OUTPUT);
  pinMode(USER3_LED, OUTPUT);
  pinMode(USER4_LED, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(ARDUINO_LED, OUTPUT);

  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT_BASED_POSITION);
  dxl.torqueOn(DXL_ID);
  
}

void loop() {

  unsigned long currMillis = millis();

  if(currMillis - prevMillis >= 100)
  {
    prevMillis = currMillis;
    
    ledStatus[ledCurrent] = !ledStatus[ledCurrent];
    digitalWrite(ledPins[ledCurrent], ledStatus[ledCurrent]);
    ledCurrent++;
    if(ledCurrent >= ledCount){ledCurrent = 0;}
  }
  else if(currMillis - pingMillis >= 500)
  {
    pingMillis = currMillis;
    dxlLedStatus = !dxlLedStatus;
    if(dxlLedStatus)
    {
       dxl.ledOn(DXL_ID);
    }
    else
    {
       dxl.ledOff(DXL_ID);
    }
  }

  dxl.setGoalCurrent(DXL_ID, 200);
  dxl.setGoalPosition(DXL_ID, 350.0, UNIT_DEGREE);

  // Print present current
  delay(500);
  DEBUG_SERIAL.print("Present Current(raw) : ");
  DEBUG_SERIAL.println(dxl.getPresentCurrent(DXL_ID));
  
  delay(5000);

  // Set Goal Current 3.0% using percentage (-100.0 [%] ~ 100.0[%])
  dxl.setGoalCurrent(DXL_ID, 3.0, UNIT_PERCENT);
  dxl.setGoalPosition(DXL_ID, 0);

  // Print present current in percentage
  delay(500);
  DEBUG_SERIAL.print("Present Current(ratio) : ");
  DEBUG_SERIAL.println(dxl.getPresentCurrent(DXL_ID, UNIT_PERCENT));
  
  delay(5000);


}
