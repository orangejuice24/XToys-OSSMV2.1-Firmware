#include <Arduino.h>

#include "OSSM.h"

// Task handles
TaskHandle_t uiTask = nullptr;
TaskHandle_t motionTask = nullptr;
TaskHandle_t estopTask = nullptr;

// Task function declarations
void uiTaskFunc(void *);
void motionCommandTask(void *);

// Create the OSSM hardware object
OSSM ossm;

///////////////////////////////////////////
////
////  Main setup
////
///////////////////////////////////////////

void setup()
{
    Serial.begin(115200);

    ossm.Setup();

    // Kick off the http and motion tasks - they begin executing as soon as they
    // are created here! Do not change the priority of the task, or do so with
    // caution. RTOS runs first in first out, so if there are no delays in your
    // tasks they will prevent all other code from running on that core!
    // start the BLE connection after homing for clean homing when reconnecting

    xTaskCreatePinnedToCore(uiTaskFunc,   /* Task function. */
                            "uiTaskFunc", /* name of task. */
                            10000,        /* Stack size of task */
                            NULL,         /* parameter of the task */
                            1,            /* priority of the task */
                            &uiTask,      /* Task handle to keep track of created task */
                            PROC_CORE);
    delay(100);

    xTaskCreatePinnedToCore(motionCommandTask, "motionCommandTask", 20000, NULL, 2, &motionTask, PROC_CORE);

    delay(100);

    LogDebug("Setup complete");
}

void loop() { vTaskDelay(1000); }

void motionCommandTask(void *)
{
    LogDebug("Starting motion command task");

    // Tasks should loop forever and not return
    for (;;)
    {
        ossm.RunMotionCommands();
    }

    LogError("Motion command task exiting");
}

void uiTaskFunc(void *)
{
    LogDebug("Starting UI task");

    for (;;)
    {
        ossm.RunUI();
    }

    LogError("UI task exiting");
}
