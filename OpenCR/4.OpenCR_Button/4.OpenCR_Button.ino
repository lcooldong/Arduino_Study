#include "Button.h"

#define USER_BUTTON 3

Button myBtn(USER_BUTTON, true, 50);

void setup() {
  Serial.begin(115200);
  pinMode(USER_BUTTON, INPUT_PULLUP);


}

void loop() {
  // Serial.printf("Button State: %d\r\n", digitalRead(USER_BUTTON));
  // delay(50);
  myBtn.read();

  if(myBtn.wasPressed())
  {
    Serial.println("Button wasPressed");
  }
}
