#pragma once

#define DEBUG

#ifdef DEBUG
#define LogDebug(...) Serial.println(__VA_ARGS__)
#define LogDebugFormatted(fmt, ...) Serial.printf(fmt "\n", ##__VA_ARGS__)

#define LogError(...) Serial.println(__VA_ARGS__)
#define LogErrorFormatted(fmt, ...) Serial.printf(fmt "\n", ##__VA_ARGS__)

#else
#define LogDebug(...)
#define LogDebugFormatted(...)

#define LogError(...)
#define LogErrorFormatted(...)
#endif

#define SW_VERSION "0.21"
#define HW_VERSION 21 // Divide by 10 for real hw version
#define EEPROM_SIZE 200

// Should only be defined at initial burn to configure HW version
// #define INITIAL_SETUP

// Top linear speed of the device.
constexpr float MAX_SPEED_IN_MM_PER_SECOND = 900.0f;

// This should match the step/rev of your stepper or servo.
// N.b. the iHSV57 has a table on the side for setting the DIP switches to your preference.
constexpr float MOTOR_STEPS_PER_REVOLUTION = 800.0f;

// Number of teeth the pulley that is attached to the servo/stepper shaft has.
constexpr float PULLEY_TOOTH_COUNT = 20.0f;

// Set to your belt pitch (Distance between two teeth on the belt) (E.g. GT2 belt has 2mm tooth pitch)
constexpr float BELT_PITCH_IN_MM = 2.0f;

// This is in millimeters, and is what's used to define how much of
// your rail is usable.
// The absolute max your OSSM would have is the distance between the belt attachments subtract
// the linear block holder length (75mm on OSSM)
// Recommended to also subtract e.g. 20mm to keep the backstop well away from the device.
constexpr float MAX_STROKE_LENGTH_IN_MM = 120.f;

//
// Web Config
//

// This should be unique to your device. You will use this on the
// web portal to interact with your OSSM.
// there is NO security other than knowing this name, make this unique to avoid
// collisions with other users
extern const char *ossmId;

// Xtoys Config
#ifdef OSSM_XTOYS

// Scaling how fast Xtoy can travel it is limted by maxSpeedMmPerSecond at top.
constexpr float XTOYS_SPEED_SCALING = 1000.0f;
constexpr float XTOYS_ACCELERATION_IN_MM_PER_SS = 40000.0f; // Hard Coded Acceleration.
constexpr float XTOYS_DECELERATION_IN_MM_PER_SS = 80000.0f; // Hard Coded Decceleration.

#endif

// Advanced Config

// After homing this is the physical buffer distance from the effective zero to the home switch
// This is to stop the home switch being smacked constantly
constexpr float STROKE_KEEPOUT_IN_MM = 8.0f;

// The minimum value of the pot in percent
// prevents noisy pots registering commands when turned down to zero by user
constexpr float POT_DEADZONE_PERCENT = 0.01f;

// NOTE: Keep this under 100000 to prevent glitches in FastAccelStepper's ramp generation
constexpr float MAX_ACCELERATION_IN_MM_PER_SS = 10000.0;

constexpr int STROKE_ENGINE_HYSTERESIS = 5;
constexpr float STROKE_ENGINE_MAX_STROKES_PER_MIN = 300;

// NOTE: These are different from the OSSM defaults because bluetooth always runs on core 1
constexpr uint8_t MOTION_CORE = 0;
constexpr uint8_t PROC_CORE = 1;