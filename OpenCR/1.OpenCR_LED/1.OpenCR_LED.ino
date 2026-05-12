#define USER1_LED   22  // SINK (LOW -> TUNR ON)
#define USER2_LED   23  // SINK
#define USER3_LED   24  // SINK
#define USER4_LED   25  // SINK
#define STATUS_LED  36  // SINK, Upload 할 때 깜빡임
#define ARDUINO_LED 13  // Source (HIGH -> TURN ON)

int ledPins[] = {USER1_LED, USER2_LED, USER3_LED, USER4_LED, STATUS_LED, ARDUINO_LED};
const int ledCount = sizeof(ledPins)/ sizeof(ledPins[0]);
bool ledStatus[ledCount] = {0,};
int ledCurrent = 0;

unsigned long prevMillis = 0;


void setup() {
  pinMode(USER1_LED, OUTPUT);
  pinMode(USER2_LED, OUTPUT);
  pinMode(USER3_LED, OUTPUT);
  pinMode(USER4_LED, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(ARDUINO_LED, OUTPUT);
  
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

}
