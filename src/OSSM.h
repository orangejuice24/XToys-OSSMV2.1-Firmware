#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include <Encoder.h>
#include <FastAccelStepper.h>
#include <StrokeEngine.h>
#include <stdint.h>
#include <forward_list>
#include <limits>

#define FASTLED_INTERNAL  // Disable messages when compiling
#include <FastLED.h>

#ifdef OSSM_XTOYS
#include "OSSM_XToys.h"
#endif

#include "OSSM_Config.h"
#include "OSSM_PinDef.h"
#include "OssmUi.h"

#define BRIGHTNESS 170
#define LED_TYPE WS2811
#define COLOR_ORDER GRB
#define LED_PIN 25
#define NUM_LEDS 1

class OSSM
{
public:
    OSSM();

    // Setup function
    void Setup();

    // User input and display function, called from main
    void RunUI();

    // Motion control function, called from main
    void RunMotionCommands();

private:
    // Stepper set up function
    void SetupStepper();

    // LEDs-related functions
    void StartLeds();
    void SetLedsError();

    // Hardware helper functions
    void InitInputs();

    // Homing functions    
    bool FindHome();
    bool DoSensorlessHoming();
    bool DoSensorHoming();
    
    // More set up functions
    void PromptForRunMode();
    void ModeSpecificSetup();

    // Run functions for the different modes
    void RunPenetrate();
    void RunStrokeEngine();

#ifdef OSSM_XTOYS
    void RunXToys();
#endif

    // EEPROM-related functions
    int ReadEepromSettings();
    void WriteEepromSettings();

    // I/O related functions
    void UpdateAnalogInputs();
    void UpdateEncoderButton();
    float GetEncoderPercentage();
    
    enum class RunMode
    {
        Simple,
        StrokeEngine,

#ifdef OSSM_XTOYS
        XToys,
#endif

        // Invalid mode, sentinel value
        MAX,
    };

    // Stepper control for homing, simple penetrate and Stroke Engine
    FastAccelStepperEngine engine_;
    FastAccelStepper* stepper_{nullptr};

#ifdef OSSM_XTOYS
    // Wrapper around XToys and bluetooth code
    OssmXToys xtoys_;
#endif

    // Encoder on remote
    Encoder encoder_;

    // Oled screen on remote
    OssmUi ui_;

    // RGB LED on board
    CRGB ossm_leds_[NUM_LEDS];

    // Track operating mode
    RunMode active_run_mode_{RunMode::Simple};

    // Whether setup is complete or not
    bool setup_done_{false};

    // Machine geometry and dimension variables
    // NOTE: Using StrokeEngine types here to make common code simpler
    motorProperties motor_properties_;
    endstopProperties endstop_properties_;
    machineGeometry geometry_;

    // Derived limits
    int max_steps_per_second_;
    int max_acceleration_in_steps_;

    // Derived travel variables
    int travel_in_mm_;
    int travel_in_steps_;

    // Simple penetrate variables, both are values from 0 to 1 to track 0% - 100%
    float speed_percent_{0.0};
    float stroke_percent_{0.0};

    // Stroke Engine variables
    StrokeEngine stroke_engine_;
    int stroke_engine_pattern_{0};
    float last_target_speed_{0.0};
    float last_target_stroke_{0.0};

    // Encoder-related variables
    unsigned long last_encoder_change_ms_ {0};
    bool encoder_switch_changed_ {false};
    bool encoder_switch_state_ {false};
    bool last_encoder_switch_state_ {false};

    // Persistant variables and statistics
    int hardware_version_ = 10; // V2.7 = integer value 27
    float lifetime_strokes_ = 0;
    float lifetime_travel_distance_meters_ = 0;
    float lifetime_seconds_on_ = 0;
};
