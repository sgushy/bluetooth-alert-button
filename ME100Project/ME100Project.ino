//#include <BLEDevice.h>
//#include <BLEUtils.h>
//#include <BLEServer.h>
#include <Arduino.h>
#include "BluetoothSerial.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "MPU9250.h"
#define SERVICE_UUID "4dc7dcfd-5e73-4d23-b8b0-326faa87d0a8"
#define CHARACTERISTIC_UUID "2baf92cf-8d28-43a2-af5a-dbbfb99bcb32"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// The input pin structure (e.g. for buttons)
struct InputPin
{
  const uint8_t pin;
  bool active;
  uint8_t lastInput;
  uint8_t curInput;
};
// Output pin structure (e.g. for led bulbs)
struct OutputPin
{
  const uint8_t pin;
  bool active;
};
// Set up buttons and LEDs
InputPin BUTTON_POWER = {14, false, 0, 0};
InputPin BUTTON_NONEMERG = {21, false, 0, 0};
InputPin BUTTON_EMERG = {17, false, 0, 0};
InputPin MOTIONSENSOR_WAKE = {4, false, 0, 0};
OutputPin LED_RED = {15, false};
OutputPin LED_YELLOW = {27, false};

// Set up accelerometer sensor
MPU9250 IMU(Wire, 0x68);
int status;

// Current state (0 = off, 1 = on, unpaired, 2 = paired, 3 = nonemergency, 4 = emergency)
uint8_t state = 0;
uint8_t desiredState = 1;

// Preload Bluetooth configurations (set up+turn on in startBT())
BluetoothSerial SerialBT;
// Is this bluetooth paired or not
bool BTON = false;
bool paired = false;
String message = "";
uint8_t loopCounter = 0;

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    Serial.println("Client Connected");
    paired = true;
  }
  if (event == ESP_SPP_CLOSE_EVT) {
    Serial.println("Client Disconnected");
    paired = false;
  }
}

// Starts Bluetooth Service
bool startBT()
{
  if (BTON == false) {

    if (!SerialBT.begin("ME100Project")) {
      Serial.println("An error occurred initializing Bluetooth");
      return false;
    } else {
      Serial.println("Bluetooth initialized");
    }
    SerialBT.register_callback(callback);
    esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
    BTON = true;
    Serial.println("Bluetooth enabled...");
  }
  return true;
}

// Try to pair devices
bool tryToPair() {
  return paired;
}

void BTLoop() {
  if (loopCounter % 10 == 0) {
    Serial.println("Searching for devices...");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_POWER.pin, INPUT_PULLUP);
  pinMode(BUTTON_NONEMERG.pin, INPUT_PULLUP);
  pinMode(BUTTON_EMERG.pin, INPUT_PULLUP);

  pinMode(MOTIONSENSOR_WAKE.pin, INPUT_PULLUP);
  Serial.println("Buttons initiailized");
  pinMode(LED_RED.pin, OUTPUT);
  pinMode(LED_YELLOW.pin, OUTPUT);
  Serial.println("LEDs initialized");

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
  }
  status = IMU.enableWakeOnMotion(150, MPU9250::LP_ACCEL_ODR_31_25HZ);
}
// Change state upon pressing the power button...
uint8_t PressPowerButton() {
  switch (state) {
    case 0:
      state = 1;
      break;
    case 1:
      state = 0;
      if (BTON) {
        BTON = false;
        Serial.println("Turning off Bluetooth!");
      }
      break;
    case 2:
      state = 0;
      if (BTON) {
        BTON = false;
        Serial.println("Turning off Bluetooth!");
      }
      break;
    case 3:
      state = 2;
      break;
    case 4:
      state = 2;
      break;
    default:
      state = 0;
      break;
  }
  return state;
}

uint8_t UpdateState()
{
  // read the sensor
  // IMU.readSensor();

  switch (state)
  {
    case 0  :
      digitalWrite(LED_YELLOW.pin, LOW);
      digitalWrite(LED_RED.pin, LOW);
      break;
    case 1  :
      // Flash yellow LED
      digitalWrite(LED_YELLOW.pin, (millis() / 500) % 2);
      digitalWrite(LED_RED.pin, LOW);
      if (!BTON) {
        startBT();
      }
      else if (paired) {
        state = 2;
      }
      else if (!paired) {
        BTLoop();
      }
      break;
    case 2  :
      digitalWrite(LED_RED.pin, LOW);
      digitalWrite(LED_YELLOW.pin, HIGH);
      if (!paired) {
        state = 1;
      }
      break;
    case 3  :
      if (!BTON) {
        startBT();
      }
      if (!paired) {
        digitalWrite(LED_RED.pin, (millis() / 500) % 2);
        digitalWrite(LED_YELLOW.pin, (millis() / 500) % 2);
      }
      else {
        digitalWrite(LED_RED.pin, (millis() / 500) % 2);
        digitalWrite(LED_YELLOW.pin, HIGH);
        message.concat("c:N;");
      }
      break;
    case 4  :
      if (!BTON) {
        startBT();
      }
      if (!paired) {
        digitalWrite(LED_RED.pin, HIGH);
        digitalWrite(LED_YELLOW.pin, (millis() / 500) % 2);
      }
      else {
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
  loopCounter++;

  message = "";
  message.concat("s:");
  message.concat(state);
  message.concat(";");
  message.concat("p:");
  if (paired) {
    message.concat("1;");
  } else {
    message.concat("0;");
  }

  BUTTON_POWER.curInput = digitalRead(BUTTON_POWER.pin);
  BUTTON_EMERG.curInput = digitalRead(BUTTON_EMERG.pin);
  BUTTON_NONEMERG.curInput = digitalRead(BUTTON_NONEMERG.pin);
  MOTIONSENSOR_WAKE.curInput = digitalRead(MOTIONSENSOR_WAKE.pin);
  if (MOTIONSENSOR_WAKE.curInput == MOTIONSENSOR_WAKE.lastInput)
  {
    if (MOTIONSENSOR_WAKE.curInput == HIGH)
    {
      Serial.println("Activated");
      state = 3;
    }
  }
  // compare the buttonState to its previous state
  if (BUTTON_POWER.curInput != BUTTON_POWER.lastInput) {
    // if the state has changed, increment the counter
    if (BUTTON_POWER.curInput == HIGH) {
      PressPowerButton();
    } else {
    }
  }
  // compare the buttonState to its previous state
  if (BUTTON_EMERG.curInput != BUTTON_EMERG.lastInput) {
    // if the state has changed, increment the counter
    if (BUTTON_EMERG.curInput == HIGH) {
      state = 4;
    }
  }
  // compare the buttonState to its previous state
  if (BUTTON_NONEMERG.curInput != BUTTON_NONEMERG.lastInput) {
    // if the state has changed, increment the counter
    if (BUTTON_NONEMERG.curInput == HIGH) {
      state = 3;
    }
  }
  // Recalibrate buttons
  BUTTON_POWER.lastInput = BUTTON_POWER.curInput;
  BUTTON_NONEMERG.lastInput = BUTTON_NONEMERG.curInput;
  BUTTON_EMERG.lastInput = BUTTON_EMERG.curInput;
  MOTIONSENSOR_WAKE.lastInput = MOTIONSENSOR_WAKE.curInput;
  UpdateState();

  // Transmit/receive messages if paired
  if (paired && loopCounter % 10 == 0 && state!=0) {
    if (Serial.available()) {
      SerialBT.write(Serial.read());
    }
    if (SerialBT.available()) {
      Serial.write(SerialBT.read());
    }
    Serial.println(message);
    SerialBT.println(message);
  }
  delay(50);
}
