#include <Arduino.h>          // Basic Needs
#include <ArduinoJson.h>      // Needed for the Bubble APP
#include <ESP_FlexyStepper.h> // Current Motion Control
#include <Encoder.h>          // Used for the Remote Encoder Input
#include <HTTPClient.h>       // Needed for the Bubble APP
#include <WiFiManager.h>      // Used to provide easy network connection  https://github.com/tzapu/WiFiManager
#include <Wire.h>             // Used for i2c connections (Remote OLED Screen)
#include <OSSM-BLE.h>
#include <list>

#include "FastLED.h"     // Used for the LED on the Reference Board (or any other pixel LEDS you may add)
#include "OSSM_Config.h" // START HERE FOR Configuration
#include "OSSM_PinDef.h" // This is where you set pins specific for your board
#include "OssmUi.h"      // Separate file that helps contain the OLED screen functions

const char* FIRMWARE_VERSION = "v1.1";

///////////////////////////////////////////
////
////  To Debug or not to Debug
////
///////////////////////////////////////////

// Uncomment the following line if you wish to print DEBUG info
#define DEBUG

#ifdef DEBUG
#define LogDebug(...) Serial.println(__VA_ARGS__)
#define LogDebugFormatted(...) Serial.printf(__VA_ARGS__)
#else
#define LogDebug(...) ((void)0)
#define LogDebugFormatted(...) ((void)0)
#endif

// Homing
volatile bool g_has_not_homed = true;
bool REMOTE_ATTACHED = false;

// Encoder
Encoder g_encoder(ENCODER_A, ENCODER_B);

// Display
OssmUi g_ui(REMOTE_ADDRESS, REMOTE_SDA, REMOTE_CLK);

///////////////////////////////////////////
////
////
////  Encoder functions & scaling
////
////
///////////////////////////////////////////

IRAM_ATTR void encoderPushButton()
{
    // TODO: Toggle position mode
    // g_encoder.write(0);       // Reset on Button Push
    // g_ui.NextFrame();         // Next Frame on Button Push
    LogDebug("Encoder Button Push");
}

float getEncoderPercentage()
{
    const int encoderFullScale = 100;
    int position = g_encoder.read();
    float positionPercentage;
    if (position < 0)
    {
        g_encoder.write(0);
        position = 0;
    }
    else if (position > encoderFullScale)
    {
        g_encoder.write(encoderFullScale);
        position = encoderFullScale;
    }

    positionPercentage = 100.0 * position / encoderFullScale;

    return positionPercentage;
}

///////////////////////////////////////////
////
////
////  WIFI Management
////
////
///////////////////////////////////////////

// Wifi Manager
WiFiManager wm;

// create the stepper motor object
ESP_FlexyStepper stepper;

// Current command state
volatile float strokePercentage = 0;
volatile float speedPercentage = 0;
volatile float deceleration = 0;
volatile int targetPosition;
volatile int targetDuration;
volatile int targetStepperPosition = 0;
volatile int remainingCommandTime = 0;
volatile float accelspeed = 0;

// Create tasks for checking pot input or web server control, and task to handle
// planning the motion profile (this task is high level only and does not pulse
// the stepper!)
TaskHandle_t wifiTask = nullptr;
TaskHandle_t getInputTask = nullptr;
TaskHandle_t motionTask = nullptr;
TaskHandle_t estopTask = nullptr;
TaskHandle_t oledTask = nullptr;
TaskHandle_t bleTask = nullptr;
TaskHandle_t blemTask = nullptr;

#define BRIGHTNESS 170
#define LED_TYPE WS2811
#define COLOR_ORDER GRB
#define LED_PIN 25
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// Declarations
// TODO: Document functions
void setLedRainbow(CRGB leds[]);
void getUserInputTask(void *pvParameters);
void motionCommandTask(void *pvParameters);
void wifiConnectionTask(void *pvParameters);
void bleConnectionTask(void *pvParameters);
void blemotionTask(void *pvParameters);
void estopResetTask(void *pvParameters);
float getAnalogAverage(int pinNumber, int samples);
bool setInternetControl(bool wifiControlEnable);
bool getInternetSettings();

bool stopSwitchTriggered = 0;

/**
 * the iterrupt service routine (ISR) for the emergency swtich
 * this gets called on a rising edge on the IO Pin the emergency switch is
 * connected it only sets the stopSwitchTriggered flag and then returns. The
 * actual emergency stop will than be handled in the loop function
 */
void ICACHE_RAM_ATTR stopSwitchHandler()
{
    stopSwitchTriggered = 1;
    vTaskSuspend(motionTask);
    vTaskSuspend(getInputTask);
    stepper.emergencyStop();
}

///////////////////////////////////////////
////
////
////  BLE Management
////
////
///////////////////////////////////////////

// Other
bool deviceConnected = false;
std::list<std::string> pendingCommands = {};
bool stepperMoving = false;
bool moveto = false;

void updateSettingsCharacteristic();
void processCommand(std::string msg);
void moveTo(int targetPosition, int targetDuration);

// Read actions and numeric values from T-Code command
void processCommand(std::string msg) {
  
  char command = NULL;
  int channel = 0;
  int targetAmount = 0;
  int targetDuration = 0;
  int numBeingRead = NUM_NONE;

  for (char c : msg) {
    switch (c) {
      case 'l':
      case 'L':
        command = 'L';
        numBeingRead = NUM_CHANNEL;
        break;
      case 'i':
      case 'I':
        numBeingRead = NUM_DURATION;
        break;
      case 'D':
      case 'd':
        command = 'D';
        numBeingRead = NUM_CHANNEL;
        break;
      case 'v':
      case 'V':
        numBeingRead = NUM_VALUE;
        break;
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
        int num = c - '0'; // convert from char to numeric int
        switch (numBeingRead) {
          case NUM_CHANNEL:
            channel = num;
            numBeingRead = NUM_PERCENT;
            break;
          case NUM_PERCENT:
            targetAmount = targetAmount * 10 + num;
            break;
          case NUM_DURATION:
            targetDuration = targetDuration * 10 + num;
            break;
        }
        break;
    }
  }
  // if amount was less than 5 digits increase to 5 digits
  // t-code message is a value to the right of the decimal place so we need a consistent length to work with in moveTo command
  // ex L99 means 0.99000 and L10010 means 0.10010
  if (command == 'L' && channel == 1) {
    moveTo(targetAmount, targetDuration);
    //} 
  } else if (command == 'D') { // not handling currently
  } else {
    Serial.print("Invalid command: ");
    Serial.println(msg.c_str());
  }

}

void moveTo(int targetPosition, int targetDuration){
        stepper.releaseEmergencyStop();
        float currentStepperPosition = stepper.getCurrentPositionInMillimeters();
        float targetxStepperPosition;

        targetxStepperPosition = map(targetPosition, 0, 100, (maxStrokeLengthMm -(strokeZeroOffsetmm * 0.5)), 0.0);
        float travelInMM = targetxStepperPosition -currentStepperPosition;
        float targetspeed = (abs(travelInMM) / targetDuration) * xtoySpeedScaling;
       
        accelspeed = map(targetspeed, 0.0, maxSpeedMmPerSecond, 0, 100);
        Serial.print("currentStepperPosition: ");
        Serial.println(currentStepperPosition); 
        Serial.print("targetspeed: ");
        Serial.println(targetspeed); 
        Serial.print("targetxStepperPosition: ");
        Serial.println(targetxStepperPosition); 
        if(targetxStepperPosition < (maxStrokeLengthMm + (strokeZeroOffsetmm * 0.5)) || targetxStepperPosition >= 0.0){
        stepper.setSpeedInMillimetersPerSecond(targetspeed);
        stepper.setAccelerationInMillimetersPerSecondPerSecond(xtoyAccelartion);
        stepper.setTargetPositionInMillimeters(targetxStepperPosition);
        } else {
          LogDebugFormatted("Position out of Safety %ld \n", static_cast<long int>(targetxStepperPosition));
        }
}

// Received request to update a setting
class SettingsCharacteristicCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) {
    std::string msg = characteristic->getValue();

    LogDebug("Received command: ");
    Serial.println(msg.c_str());

    std::size_t pos = msg.find(':');
    std::string settingKey = msg.substr(0, pos);
    std::string settingValue = msg.substr(pos + 1, msg.length());

    if (settingKey == "maxIn") {
      maxInPosition = atoi(settingValue.c_str());
      preferences.putInt("maxIn", maxInPosition);
    }
    if (settingKey == "maxOut") {
      maxOutPosition = atoi(settingValue.c_str());
      preferences.putInt("maxOut", maxOutPosition);
    }
    if (settingKey == "maxSpeed") {
      maxSpeed = atoi(settingValue.c_str());
      preferences.putInt("maxSpeed", maxSpeed);
    }
    if (settingKey == "minSpeed") {
      minSpeed = atoi(settingValue.c_str());
      preferences.putInt("minSpeed", minSpeed);
    }
    Serial.print("Setting pref ");
    Serial.print(settingKey.c_str());
    Serial.print(" to ");
    Serial.println(settingValue.c_str());
    updateSettingsCharacteristic();
  }
};

// Received T-Code command
class ControlCharacteristicCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) {
    std::string msg = characteristic->getValue();

    Serial.print("Received command: ");
    Serial.println(msg.c_str());

    // check for messages that might need to be immediately handled
    if (msg == "DSTOP") { // stop request
      Serial.println("STOP");
      stepper.emergencyStop();
      pendingCommands.clear();
      return;
    } else if (msg == "DENABLE") { // enable stepper motor
      Serial.println("ENABLE");
      stepper.releaseEmergencyStop();
      pendingCommands.clear();
      //stepperEnabled = true;
      return;
    } else if (msg == "DDISABLE") { // disable stepper motor
      Serial.println("DISABLE");
      stepper.emergencyStop();
      pendingCommands.clear();
      return;
    }
    if (msg.front() == 'D') { // device status message, process immediately (technically the code isn't handling any T-Code 'D' messages currently
      return;
    }
    if (msg.back() == 'C') { // movement command includes a clear existing commands flag, clear queue and process new movement command immediately
      pendingCommands.clear();
      pendingCommands.push_back(msg);
      return;
    }
    // probably a normal movement command, store it to be run after other movement commands are finished
    if (pendingCommands.size() < 100) {
      pendingCommands.push_back(msg);
      Serial.print("# of pending commands: ");
      Serial.println(pendingCommands.size());
    } else {
      Serial.print("Too many commands in queue. Dropping: ");
      Serial.println(msg.c_str());
    }
  }
};

// Client connected to OSSM over BLE
class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("BLE Connected");
    vTaskSuspend(motionTask);
    vTaskSuspend(getInputTask);
    vTaskSuspend(estopTask);
    vTaskResume(blemTask);
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("BLE Disconnected");
    pServer->startAdvertising();
    vTaskSuspend(blemTask);
    vTaskResume(motionTask);
    vTaskResume(getInputTask);
    vTaskResume(estopTask);
  }
};


void updateSettingsCharacteristic() {
  String settingsInfo = String("maxIn:") + maxInPosition + ",maxOut:" + maxOutPosition + ",maxSpeed:" + maxSpeed + ",minSpeed:" + minSpeed;
  settingsCharacteristic->setValue(settingsInfo.c_str());
}
///////////////////////////////////////////
////
////
////  VOID SETUP -- Here's where it's hiding
////
////
///////////////////////////////////////////

void setup()
{
    Serial.begin(115200);
    LogDebug("\n Starting");
    delay(200);

    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(150);
    setLedRainbow(leds);
    FastLED.show();
    stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);

    preferences.begin("OSSM", false);
    maxSpeed = preferences.getInt("maxSpeed", DEFAULT_MAX_SPEED);
    minSpeed = preferences.getInt("minSpeed", DEFAULT_MIN_SPEED);

    float stepsPerMm = motorStepPerRevolution / (pulleyToothCount * beltPitchMm); // GT2 belt has 2mm tooth pitch
    stepper.setStepsPerMillimeter(stepsPerMm);
    // initialize the speed and acceleration rates for the stepper motor. These
    // will be overwritten by user controls. 100 values are placeholders
    stepper.setSpeedInStepsPerSecond(200);
    stepper.setAccelerationInMillimetersPerSecondPerSecond(100);
    stepper.setDecelerationInStepsPerSecondPerSecond(100000);
    stepper.setLimitSwitchActive(LIMIT_SWITCH_PIN);

    // Start the stepper instance as a service in the "background" as a separate
    // task and the OS of the ESP will take care of invoking the processMovement()
    // task regularly on core 1 so you can do whatever you want on core 0
    stepper.startAsService(); // Kinky Makers - we have modified this function
                              // from default library to run on core 1 and suggest
                              // you don't run anything else on that core.

    WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
    // put your setup code here, to run once:
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    pinMode(WIFI_RESET_PIN, INPUT_PULLUP);
    pinMode(WIFI_CONTROL_TOGGLE_PIN, WIFI_CONTROLLER); // choose between WIFI_CONTROLLER and LOCAL_CONTROLLER
    // test

    // set the pin for the emegrency switch to input with inernal pullup
    // the emergency switch is connected in a Active Low configuraiton in this
    // example, meaning the switch connects the input to ground when closed
    pinMode(STOP_PIN, INPUT_PULLUP);
    // attach an interrupt to the IO pin of the switch and specify the handler
    // function
    attachInterrupt(digitalPinToInterrupt(STOP_PIN), stopSwitchHandler, RISING);
    // Set analog pots (control knobs)

    pinMode(SPEED_POT_PIN, INPUT);
    adcAttachPin(SPEED_POT_PIN);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db); // allows us to read almost full 3.3V range

    // This is here in case you want to change WiFi settings - pull IO low
    if (digitalRead(WIFI_RESET_PIN) == LOW)
    {
        // reset settings - for testing
        wm.resetSettings();
        LogDebug("settings reset");
    }

    // OLED SETUP
    g_ui.Setup();
    g_ui.UpdateOnly();

    // Rotary Encoder Pushbutton
    pinMode(ENCODER_SWITCH, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(ENCODER_SWITCH), encoderPushButton, RISING);
    
    //start the WiFi connection task so we can be doing something while homing!
    xTaskCreatePinnedToCore(wifiConnectionTask,   /* Task function. */
                            "wifiConnectionTask", /* name of task. */
                            10000,                /* Stack size of task */
                            NULL,                 /* parameter of the task */
                            1,                    /* priority of the task */
                            &wifiTask,            /* Task handle to keep track of created task */
                            0);                   /* pin task to core 0 */
    delay(100);

    if (g_has_not_homed == true)
    {
        LogDebug("OSSM will now home");
        g_ui.UpdateMessage("Finding Home");
        stepper.setSpeedInMillimetersPerSecond(20);
        stepper.moveToHomeInMillimeters(1, 30, 300, LIMIT_SWITCH_PIN);
        LogDebug("OSSM has homed, will now move out to max length");
        g_ui.UpdateMessage("Moving to Max");
        stepper.setSpeedInMillimetersPerSecond(20);
        stepper.moveToPositionInMillimeters((-1 * maxStrokeLengthMm) - strokeZeroOffsetmm);
        LogDebug("OSSM has moved out, will now set new home?");
        stepper.setCurrentPositionAsHomeAndStop();
        LogDebug("OSSM should now be home and happy");
        g_has_not_homed = false;
    }

    xTaskCreatePinnedToCore(blemotionTask,   /* Task function. */
                            "blemotionTask", /* name of task. */
                            1000,              /* Stack size of task */
                            NULL,               /* parameter of the task */
                            1,                 /* priority of the task */
                            &blemTask,      /* Task handle to keep track of created task */
                            0);                 /* pin task to core 0 */
    vTaskSuspend(blemTask);
    delay(100);
    
    
    //start the BLE connection for clean homing
    xTaskCreatePinnedToCore(bleConnectionTask,   /* Task function. */
                            "bleConnectionTask", /* name of task. */
                            10000,                /* Stack size of task */
                            NULL,                 /* parameter of the task */
                            1,                    /* priority of the task */
                            &bleTask,            /* Task handle to keep track of created task */
                            0);                   /* pin task to core 0 */
    delay(100);

    // Kick off the http and motion tasks - they begin executing as soon as they
    // are created here! Do not change the priority of the task, or do so with
    // caution. RTOS runs first in first out, so if there are no delays in your
    // tasks they will prevent all other code from running on that core!

    xTaskCreatePinnedToCore(getUserInputTask,   /* Task function. */
                            "getUserInputTask", /* name of task. */
                            10000,              /* Stack size of task */
                            NULL,               /* parameter of the task */
                            1,                  /* priority of the task */
                            &getInputTask,      /* Task handle to keep track of created task */
                            0);                 /* pin task to core 0 */
    delay(100);
    xTaskCreatePinnedToCore(motionCommandTask,   /* Task function. */
                            "motionCommandTask", /* name of task. */
                            10000,               /* Stack size of task */
                            NULL,                /* parameter of the task */
                            1,                   /* priority of the task */
                            &motionTask,         /* Task handle to keep track of created task */
                            0);                  /* pin task to core 0 */

    delay(100);
    xTaskCreatePinnedToCore(estopResetTask,   /* Task function. */
                            "estopResetTask", /* name of task. */
                            10000,            /* Stack size of task */
                            NULL,             /* parameter of the task */
                            1,                /* priority of the task */
                            &estopTask,       /* Task handle to keep track of created task */
                            0);               /* pin task to core 0 */

    delay(100);

    g_ui.UpdateMessage("OSSM Ready to Play");
}

///////////////////////////////////////////
////
////
////   VOID LOOP - Hides here
////
////
///////////////////////////////////////////

void loop()
{
    g_ui.UpdateState(static_cast<int>(speedPercentage), static_cast<int>(strokePercentage + 0.5f));
    g_ui.UpdateScreen();

    // debug
    static bool is_connected = false;
    if (!is_connected && g_ui.DisplayIsConnected())
    {
        LogDebug("Display Connected");
        is_connected = true;
    }
    else if (is_connected && !g_ui.DisplayIsConnected())
    {
        LogDebug("Display Disconnected");
        is_connected = false;
    }
}

///////////////////////////////////////////
////
////
////  freeRTOS multitasking
////
////
///////////////////////////////////////////

void estopResetTask(void *pvParameters)
{
    for (;;)
    {
        if (stopSwitchTriggered == 1)
        {
            while ((getAnalogAverage(SPEED_POT_PIN, 50) > 2))
            {
                vTaskDelay(1);
            }
            stopSwitchTriggered = 0;
            vTaskResume(motionTask);
            vTaskResume(getInputTask);
        }
        vTaskDelay(100);
    }
}

void bleConnectionTask(void *pvParameters){

Serial.println("Initializing BLE Server...");
  BLEDevice::init("OSSM");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  
  infoService = pServer->createService(BLEUUID((uint16_t) 0x180a));
  BLE2904* softwareVersionDescriptor = new BLE2904();
  softwareVersionDescriptor->setFormat(BLE2904::FORMAT_UINT8);
  softwareVersionDescriptor->setNamespace(1);
  softwareVersionDescriptor->setUnit(0x27ad);

  softwareVersionCharacteristic = infoService->createCharacteristic((uint16_t) 0x2a28, BLECharacteristic::PROPERTY_READ);
  softwareVersionCharacteristic->addDescriptor(softwareVersionDescriptor);
  softwareVersionCharacteristic->addDescriptor(new BLE2902());
  softwareVersionCharacteristic->setValue(FIRMWARE_VERSION);
  infoService->start();
  
  pService = pServer->createService(SERVICE_UUID);
  controlCharacteristic = pService->createCharacteristic(
                                         CONTROL_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  controlCharacteristic->addDescriptor(new BLE2902());
  controlCharacteristic->setValue("");
  controlCharacteristic->setCallbacks(new ControlCharacteristicCallback());
  
  settingsCharacteristic = pService->createCharacteristic(
                                         SETTINGS_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  settingsCharacteristic->addDescriptor(new BLE2902());
  settingsCharacteristic->setValue("");
  settingsCharacteristic->setCallbacks(new SettingsCharacteristicCallback());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  updateSettingsCharacteristic();
  vTaskDelete(NULL);
}


void wifiConnectionTask(void *pvParameters)
{
    wm.setConfigPortalTimeout(100);
    wm.setConfigPortalBlocking(false);
    // here we try to connect to WiFi or launch settings hotspot for you to enter
    // WiFi credentials
    if (!wm.autoConnect("OSSM-setup"))
    {
        // TODO: Set Status LED to indicate failure
        LogDebug("failed to connect and hit timeout");
    }
    else
    {
        // TODO: Set Status LED to indicate everything is ok!
        LogDebug("Connected!");
    }
    for (;;)
    {
        wm.process();
        vTaskDelay(1);

        // delete this task once connected!
        if (WiFi.status() == WL_CONNECTED)
        {
            vTaskDelete(NULL);
        }
    }
}

void blemotionTask(void *pvParameters)
{
    for (;;) // tasks should loop forever and not return - or will throw error in
             // OS
    {
        while (stepper.getDistanceToTargetSigned() != 0)
        {
            vTaskDelay(5); // wait for motion to complete
        }
        stepper.releaseEmergencyStop();
        stepper.setDecelerationInMillimetersPerSecondPerSecond(xtoyAccelartion);
        vTaskDelay(1);
        if (pendingCommands.size() > 0) { 
        std::string command = pendingCommands.front();
        processCommand(command);
        pendingCommands.pop_front();
        }  
        vTaskDelay(1);
    }    
}

// Task to read settings from server - only need to check this when in WiFi
// control mode
void getUserInputTask(void *pvParameters)
{
    bool wifiControlEnable = false;
    for (;;) // tasks should loop forever and not return - or will throw error in
             // OS
    {
        // LogDebug("Speed: " + String(speedPercentage) + "\% Stroke: " + String(strokePercentage) +
        //          "\% Distance to target: " + String(stepper.getDistanceToTargetSigned()) + " steps?");
        if (speedPercentage > 1)
        {
            stepper.releaseEmergencyStop();
        }
        else
        {
            stepper.emergencyStop();
            // LogDebug("FULL STOP CAPTAIN");
        }

        if (digitalRead(WIFI_CONTROL_TOGGLE_PIN) == HIGH) // TODO: check if wifi available and handle gracefully
        {
            if (wifiControlEnable == false)
            {
                // this is a transition to WiFi, we should tell the server it has
                // control
                wifiControlEnable = true;
                if (WiFi.status() != WL_CONNECTED)
                {
                    delay(5000);
                }
                setInternetControl(wifiControlEnable);
            }
            getInternetSettings(); // we load speedPercentage and strokePercentage in
                                   // this routine.
      } 
       else
       {
            if (wifiControlEnable == true)
            {
                // this is a transition to local control, we should tell the server it
                // cannot control
                wifiControlEnable = false;
                setInternetControl(wifiControlEnable);
            }
            speedPercentage = getAnalogAverage(SPEED_POT_PIN,
                                               50); // get average analog reading, function takes pin and # samples
            // strokePercentage = getAnalogAverage(STROKE_POT_PIN, 50);
            strokePercentage = getEncoderPercentage();
        }

        // We should scale these values with initialized settings not hard coded
        // values!     
        if (speedPercentage > commandDeadzonePercentage)
        {
            stepper.setSpeedInMillimetersPerSecond(maxSpeedMmPerSecond * speedPercentage / 100.0);
            stepper.setAccelerationInMillimetersPerSecondPerSecond(maxSpeedMmPerSecond * speedPercentage *
                                                                   speedPercentage / accelerationScaling);
            // We do not set deceleration value here because setting a low decel when
            // going from high to low speed causes the motor to travel a long distance
            // before slowing. We should only change decel at rest
        }
        vTaskDelay(100); // let other code run!
    }
}

void motionCommandTask(void *pvParameters)
{
    for (;;) // tasks should loop forever and not return - or will throw error in
             // OS
    {
        // poll at 200Hz for when motion is complete
        while ((stepper.getDistanceToTargetSigned() != 0) || (strokePercentage < commandDeadzonePercentage) ||
               (speedPercentage < commandDeadzonePercentage))
        {
            vTaskDelay(5); // wait for motion to complete and requested stroke more than zero
        }  
        float targetPosition = (strokePercentage / 100.0) * maxStrokeLengthMm;
        LogDebugFormatted("Moving stepper to position %ld \n", static_cast<long int>(targetPosition));
        vTaskDelay(1);
        stepper.setDecelerationInMillimetersPerSecondPerSecond(maxSpeedMmPerSecond * speedPercentage * speedPercentage /
                                                               accelerationScaling);
        stepper.setTargetPositionInMillimeters(targetPosition);
        vTaskDelay(1);

        while ((stepper.getDistanceToTargetSigned() != 0) || (strokePercentage < commandDeadzonePercentage) ||
               (speedPercentage < commandDeadzonePercentage))
        {
            vTaskDelay(5); // wait for motion to complete, since we are going back to
                           // zero, don't care about stroke value
        }
        targetPosition = 0;
        // Serial.printf("Moving stepper to position %ld \n", targetPosition);
        vTaskDelay(1);
        stepper.setDecelerationInMillimetersPerSecondPerSecond(maxSpeedMmPerSecond * speedPercentage * speedPercentage /
                                                               accelerationScaling);
        stepper.setTargetPositionInMillimeters(targetPosition);
        vTaskDelay(1);
  }
}


float getAnalogAverage(int pinNumber, int samples)
{
    float sum = 0;
    float average = 0;
    float percentage = 0;
    for (int i = 0; i < samples; i++)
    {
        // TODO: Possibly use fancier filters?
        sum += analogRead(pinNumber);
    }
    average = sum / samples;
    // TODO: Might want to add a deadband
    percentage = 100.0 * average / 4096.0; // 12 bit resolution
    return percentage;
}

bool setInternetControl(bool wifiControlEnable)
{
    // here we will SEND the WiFi control permission, and current speed and stroke
    // to the remote server. The cloudfront redirect allows http connection with
    // bubble backend hosted at app.researchanddesire.com

    String serverNameBubble = "http://d2g4f7zewm360.cloudfront.net/ossm-set-control"; // live server
    // String serverNameBubble =
    // "http://d2oq8yqnezqh3r.cloudfront.net/ossm-set-control"; // this is
    // version-test server

    // Add values in the document to send to server
    StaticJsonDocument<200> doc;
    doc["ossmId"] = ossmId;
    doc["wifiControlEnabled"] = wifiControlEnable;
    doc["stroke"] = strokePercentage;
    doc["speed"] = speedPercentage;
    String requestBody;
    serializeJson(doc, requestBody);

    // Http request
    HTTPClient http;
    http.begin(serverNameBubble);
    http.addHeader("Content-Type", "application/json");
    // post and wait for response
    int httpResponseCode = http.POST(requestBody);
    String payload = "{}";
    payload = http.getString();
    http.end();

    // deserialize JSON
    StaticJsonDocument<200> bubbleResponse;
    deserializeJson(bubbleResponse, payload);

    // TODO: handle status response
    // const char *status = bubbleResponse["status"]; // "success"

    const char *wifiEnabledStr = (wifiControlEnable ? "true" : "false");
    LogDebugFormatted("Setting Wifi Control: %s\n%s\n%s\n", wifiEnabledStr, requestBody.c_str(), payload.c_str());
    LogDebugFormatted("HTTP Response code: %d\n", httpResponseCode);

    return true;
}

bool getInternetSettings()
{
    // here we will request speed and stroke settings from the remote server. The
    // cloudfront redirect allows http connection with bubble backend hosted at
    // app.researchanddesire.com

    String serverNameBubble = "http://d2g4f7zewm360.cloudfront.net/ossm-get-settings"; // live server
    // String serverNameBubble =
    // "http://d2oq8yqnezqh3r.cloudfront.net/ossm-get-settings"; // this is
    // version-test
    // server

    // Add values in the document
    StaticJsonDocument<200> doc;
    doc["ossmId"] = ossmId;
    String requestBody;
    serializeJson(doc, requestBody);

    // Http request
    HTTPClient http;
    http.begin(serverNameBubble);
    http.addHeader("Content-Type", "application/json");
    // post and wait for response
    int httpResponseCode = http.POST(requestBody);
    String payload = "{}";
    payload = http.getString();
    http.end();

    // deserialize JSON
    StaticJsonDocument<200> bubbleResponse;
    deserializeJson(bubbleResponse, payload);

    // TODO: handle status response
    // const char *status = bubbleResponse["status"]; // "success"
    strokePercentage = bubbleResponse["response"]["stroke"];
    speedPercentage = bubbleResponse["response"]["speed"];

    // debug info on the http payload
    LogDebug(payload);
    LogDebugFormatted("HTTP Response code: %d\n", httpResponseCode);

    return true;
}

void setLedRainbow(CRGB leds[])
{
    // int power = 250;

    for (int hueShift = 0; hueShift < 350; hueShift++)
    {
        int gHue = hueShift % 255;
        fill_rainbow(leds, NUM_LEDS, gHue, 25);
        FastLED.show();
        delay(4);
    }
}