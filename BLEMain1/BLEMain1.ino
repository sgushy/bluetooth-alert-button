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
  uint8_t lastInput;
  uint8_t curInput;
};
struct OutputPin 
{
  const uint8_t pin;
  bool active;
};

InputPin BUTTON_POWER = {14,false,0,0};
InputPin BUTTON_NONEMERG = {21,false,0,0};
InputPin BUTTON_EMERG = {17,false,0,0};
OutputPin LED_RED = {15,false};
OutputPin LED_YELLOW = {27,false};

// Current state (0 = off, 1 = on, unpaired, 2 = paired, 3 = nonemergency, 4 = emergency)
uint8_t state = 0;
uint8_t desiredState = 1;

BLEServer* pServer = NULL;
BLEService* pService = NULL;
BLECharacteristic* pCharacteristic = NULL;

// Is this bluetooth paired or not
bool BTON = false;
bool paired = false;
String message = "";
uint8_t loopCounter = 0;

// BT Interrupt
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      paired = true;
    };

    void onDisconnect(BLEServer* pServer) {
      paired = false;
    }
};

// Starts Bluetooth Service
bool startBT()
{
   Serial.begin(115200);
  Serial.println("Starting BLE work!");
  BLEDevice::init("ME-100_Project");
  pServer = BLEDevice::createServer();
  pService = pServer->createService(SERVICE_UUID);
  pServer->setCallbacks(new MyServerCallbacks());
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setValue("Toastzan");
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
  return true;
}

bool stopBT()
{
  pService->stop();
  return false;
}
// Try to pair devices
bool tryToPair(){
    return paired;
}

void BTLoop(){
  
}

void setup() {
   Serial.begin(115200);
      pinMode(BUTTON_POWER.pin, INPUT_PULLUP);
      pinMode(BUTTON_NONEMERG.pin, INPUT_PULLUP);
      pinMode(BUTTON_EMERG.pin, INPUT_PULLUP);

      pinMode(LED_RED.pin, OUTPUT);
      pinMode(LED_YELLOW.pin, OUTPUT);
}
// Change state by pressing the power button...
uint8_t PressPowerButton()
{
  switch(state){
    case 0:
    state = 1;
    break;
    case 1:
    state = 0;
    break;
    case 2:
    state = 0;
    break;
    case 3:
    state = 2;
    break;
    case 4:
    state = 2;
    break;
    default:
    state = 0;break;
  }
  return state;
}

uint8_t UpdateState()
{
  switch(state)
  {
    case 0  : 
      digitalWrite(LED_YELLOW.pin,LOW);
      digitalWrite(LED_RED.pin,LOW);
      if(BTON){
        BTON = stopBT();
      } 
      break;
    case 1  : 
      // Flash yellow LED
      digitalWrite(LED_YELLOW.pin, (millis() / 500) % 2);
      digitalWrite(LED_RED.pin, LOW);
      if(!BTON){
        BTON = startBT();
      }
      else if(paired){
        state = desiredState;
      }
      else if(!paired){
        //paired = tryToPair();
      } 
      break;
    case 2  : 
      digitalWrite(LED_RED.pin, LOW);
      digitalWrite(LED_YELLOW.pin, HIGH);
      break;
    case 3  : 
      if(!paired){
          digitalWrite(LED_RED.pin, (millis() / 500) % 2);
          digitalWrite(LED_YELLOW.pin, (millis() / 500) % 2);
        }
        else{
          digitalWrite(LED_RED.pin, (millis() / 500) % 2);
          digitalWrite(LED_YELLOW.pin, HIGH);
          message.concat("c:N;");
        }
      break;
    case 4  : 
      if(!paired){
        digitalWrite(LED_RED.pin, HIGH);
        digitalWrite(LED_YELLOW.pin, (millis() / 500) % 2);
      }
      else{
        digitalWrite(LED_RED.pin, HIGH);
        digitalWrite(LED_YELLOW.pin, HIGH);
        message.concat("c:E;");
      }
      break;
    default : state = 0; break;
  }
  return state;
}

void loop() {
  // put your main code here, to run repeatedly:
  //Loop counter increases by 1 every 50 ms
  loopCounter++;
  message = "";

  if(paired)
  {
    message.concat("s:");
    message.concat(state);
    message.concat(";");
  }

  BUTTON_POWER.curInput = digitalRead(BUTTON_POWER.pin);
  BUTTON_EMERG.curInput = digitalRead(BUTTON_EMERG.pin);
  BUTTON_NONEMERG.curInput = digitalRead(BUTTON_NONEMERG.pin);

  // compare the buttonState to its previous state
  if (BUTTON_POWER.curInput != BUTTON_POWER.lastInput) {
    // if the state has changed, increment the counter
    if (BUTTON_POWER.curInput == HIGH) {
      // if the current state is HIGH then the button went from off to on:
       PressPowerButton();
    } else {
      // if the current state is LOW then the button went from on to off:
      Serial.println("off");
    }
  }
  // compare the buttonState to its previous state
  if (BUTTON_EMERG.curInput != BUTTON_EMERG.lastInput) {
    // if the state has changed, increment the counter
    if (BUTTON_EMERG.curInput == HIGH) {
      // if the current state is HIGH then the button went from off to on:
       state = 4;
    } else {
      // if the current state is LOW then the button went from on to off:
      Serial.println("off");
    }
  }
  // compare the buttonState to its previous state
  if (BUTTON_NONEMERG.curInput != BUTTON_NONEMERG.lastInput) {
    // if the state has changed, increment the counter
    if (BUTTON_NONEMERG.curInput == HIGH) {
      // if the current state is HIGH then the button went from off to on:
       state = 3;
    } else {
      // if the current state is LOW then the button went from on to off:
      Serial.println("off");
    }
  }

    BUTTON_POWER.lastInput = BUTTON_POWER.curInput;
    BUTTON_NONEMERG.lastInput = BUTTON_NONEMERG.curInput;
    BUTTON_EMERG.lastInput = BUTTON_EMERG.curInput;
 
  UpdateState();

  if(paired)
  {
    int l = message.length();
    char msg[l];
    message.toCharArray(msg,l);
    pCharacteristic->setValue((unsigned char*)&msg,l);
    delay(10);
  }
  delay(50);
}
