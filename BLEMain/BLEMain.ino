#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Arduino.h>

#define SERVICE_UUID "4dc7dcfd-5e73-4d23-b8b0-326faa87d0a8"
#define CHARACTERISTIC_UUID "2baf92cf-8d28-43a2-af5a-dbbfb99bcb32"

struct InputPin 
{
  const uint8_t pin;
  bool active;
};
struct OutputPin 
{
  const uint8_t pin;
  bool active;
};

InputPin BUTTON_POWER = {14,false};
InputPin BUTTON_NONEMERG = {21,false};
InputPin BUTTON_EMERG = {20,false};
OutputPin LED_RED = {15,false};
OutputPin LED_YELLOW = {27,false};

// Current state (0 = off, 1 = on, unpaired, 2 = paired, 3 = nonemergency, 4 = emergency)
uint8_t state = 0;
// Is this bluetooth paired or not
bool paired = false;
String message = "";
uint8_t loopCounter = 0;

void IRAM_ATTR isr(void* arg) {
    InputPin* s = static_cast<InputPin*>(arg);
    s->active = true;
}
void IRAM_ATTR isr() {
    BUTTON_NONEMERG.active = true;
}
void IRAM_ATTR isr() {
    BUTTON_EMERG.active = true;
}

void setup() {
   Serial.begin(115200);
      pinMode(BUTTON_POWER.pin, INPUT_PULLUP);
      pinMode(BUTTON_NONEMERG.pin, INPUT_PULLUP);
      pinMode(BUTTON_EMERG.pin, INPUT_PULLUP);
      attachInterruptArg(BUTTON_POWER.pin, isr, &BUTTON_POWER, FALLING);
      attachInterruptArg(BUTTON_NONEMERG.pin, isr, FALLING);
      attachInterruptArg(BUTTON_EMERG.pin, isr, FALLING);
      pinMode(LED_RED.pin, OUTPUT);
      pinMode(LED_YELLOW.pin, OUTPUT);
}
// Change state by pressing the power button...
uint8_t PressPowerButton()
{
  if(state == 0)
  {
    state = 1;
  }
  else if(state == 1)
  {
    state = 0;
  }
  else if(state == 2)
  {
    state = 0;
  }
  else if(state == 3)
  {
    state = 2;
  }
  else if(state == 4)
  {
    state = 2;
  }

  return state;
}

void UpdateState()
{
  if(state == 0)
  {
    digitalWrite(LED_YELLOW.pin,LOW);
    digitalWrite(LED_RED.pin,LOW);
  }
  else if(state == 1)
  {
    // Flash yellow LED
    digitalWrite(LED_YELLOW.pin, (millis() / 500) % 2);
    digitalWrite(LED_RED.pin, LOW);
  }
  else if(state == 2)
  {
    digitalWrite(LED_RED.pin, LOW);
    digitalWrite(LED_YELLOW.pin, HIGH);
  }
  else if(state == 3)
  {
    digitalWrite(LED_RED.pin, (millis() / 500) % 2);
    digitalWrite(LED_YELLOW.pin, LOW);
  }
  else if(state == 4)
  {
    digitalWrite(LED_RED.pin, HIGH);
    digitalWrite(LED_YELLOW.pin, LOW);
  }

  if(state!=0&&!paired)
  {
    //state = 1;
  }

  if(paired)
  {
      //state = 2;
  }
  message += state;
  Serial.print(message);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Loop counter increases by 1 every 50 ms
  loopCounter++;
  message = "";
  
  if(BUTTON_EMERG.active)
  {
    state = 3;
    BUTTON_NONEMERG.active = false;
  }
  else if(BUTTON_EMERG.active)
  {
    state = 4;
    BUTTON_EMERG.active = false;
  }
  else if(BUTTON_POWER.active)
  {
    PressPowerButton();
    BUTTON_POWER.active = false;
  }
  UpdateState();
  
  delay(20);
}
