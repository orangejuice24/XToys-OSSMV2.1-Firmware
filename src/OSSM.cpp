#include "OSSM.h"

#include <errno.h>

namespace
{

struct MovingAvg
{
    static const int N_SAMPLES = 256;

    uint16_t sample_history_[N_SAMPLES] {0};
    int sample_index_ {0};

    uint32_t Value()
    {
        uint32_t sum = 0;

        for (int i = 0; i < N_SAMPLES; ++i)
        {
            sum += sample_history_[i];
        }

        return sum / N_SAMPLES;
    }

    void AddSample(uint16_t sample_value)
    {
        sample_history_[sample_index_] = sample_value;
        sample_index_ = (sample_index_ + 1) % N_SAMPLES;
    }
};

MovingAvg CurrentSenseAvg;

// Returns analog average as a percent from 0 to 1
float GetAnalogAveragePercent(int pin_number, uint16_t sample_count)
{
    uint32_t sum = 0;

    for (int i = 0; i < sample_count; ++i)
    {
        sum += analogRead(pin_number);
    }

    // Calculate average sample value
    const float average = static_cast<float>(sum) / static_cast<float>(sample_count);

    return average / 4096.0;
}

} // namespace

OSSM::OSSM() : encoder_(ENCODER_A, ENCODER_B), ui_(REMOTE_ADDRESS, REMOTE_SDA, REMOTE_CLK)
{
    motor_properties_ = {.maxSpeed = 70 * (MAX_SPEED_IN_MM_PER_SECOND / (PULLEY_TOOTH_COUNT * BELT_PITCH_IN_MM)),
                         .maxAcceleration = MAX_ACCELERATION_IN_MM_PER_SS,
                         .stepsPerMillimeter = MOTOR_STEPS_PER_REVOLUTION / (PULLEY_TOOTH_COUNT * BELT_PITCH_IN_MM),
                         .invertDirection = true,
                         .enableActiveLow = true,
                         .stepPin = MOTOR_STEP_PIN,
                         .directionPin = MOTOR_DIRECTION_PIN,
                         .enablePin = MOTOR_ENABLE_PIN};

    endstop_properties_ = {.homeToBack = false, .activeLow = true, .endstopPin = TOY_LIMIT_SWITCH_PIN, .pinMode = INPUT};

    geometry_ = {.physicalTravel = abs(MAX_STROKE_LENGTH_IN_MM), .keepoutBoundary = STROKE_KEEPOUT_IN_MM};

    travel_in_mm_ = geometry_.physicalTravel - (2 * geometry_.keepoutBoundary);
    travel_in_steps_ = int(0.5 + travel_in_mm_ * motor_properties_.stepsPerMillimeter);

    max_steps_per_second_ = int(0.5 + motor_properties_.maxSpeed * motor_properties_.stepsPerMillimeter);
    max_acceleration_in_steps_ = int(0.5 + motor_properties_.maxAcceleration * motor_properties_.stepsPerMillimeter);
}

void OSSM::Setup()
{
    // Do this first so the user has some feedback that we are booting
    StartLeds();

    LogDebug("Software version");
    LogDebug(SW_VERSION);

    LogDebugFormatted("Geom: Physical Travel: %0.2f, Keepout: %0.2f", geometry_.physicalTravel,
                      geometry_.keepoutBoundary);
    LogDebugFormatted("Travel in mm: %d, Travel in steps: %d", travel_in_mm_, travel_in_steps_);
    LogDebugFormatted("Max steps per sec: %d, Max accel in steps^2: %d", max_steps_per_second_,
                      max_acceleration_in_steps_);

    ui_.Setup();
    ui_.UpdateOnly();

    delay(50);

    ui_.UpdateMessage("Booting up!");

    ReadEepromSettings();

    InitInputs();

    SetupStepper();

    delay(500);

    if (!FindHome())
    {
        SetLedsError();

        ui_.UpdateMessage("Failed to find home!");

        for (;;)
        {
        }
    }

    PromptForRunMode();

    ModeSpecificSetup();

    delay(250);

    setup_done_ = true;

    // Ensure that stroke has not been set while selecting mode
    encoder_.write(0);
    stroke_percent_ = 0.0;
}

void OSSM::RunUI()
{
    UpdateAnalogInputs();
    UpdateEncoderButton();

    const auto speed_percent = static_cast<int>(speed_percent_ * 100);
    const auto stroke_percent = static_cast<int>(stroke_percent_ * 100 + 0.5f);

    // LogDebugFormatted("Update end @ %ld: Speed: %d, Stroke: %d", millis(), speed_percent, stroke_percent);

    if (!setup_done_)
    {
        return;
    }

    if (active_run_mode_ == RunMode::StrokeEngine)
    {
        // If the switch changed, and it was released, and it was not a long press
        if (encoder_switch_changed_ && !encoder_switch_state_ && !is_long_press_)
        {
            LogDebug("Incrementing pattern");

            auto next_pattern = stroke_engine_pattern_ + 1;
            if (next_pattern >= stroke_engine_.getNumberOfPattern())
            {
                next_pattern = 0;
            }
            stroke_engine_pattern_ = next_pattern;

            ui_.UpdateMessage(stroke_engine_.getPatternName(next_pattern));
        }

        ui_.UpdateEncoderSelection(encoder_switch_state_ && is_long_press_);
        ui_.UpdateSensation(sensation_value_);
    }

    ui_.UpdateState(speed_percent, stroke_percent);
    ui_.UpdateScreen();

    vTaskDelay(25);
}

void OSSM::RunMotionCommands()
{
    const auto mode = active_run_mode_;

    switch (mode)
    {
        case OSSM::RunMode::Simple:
            RunPenetrate();
            break;

        case OSSM::RunMode::StrokeEngine:
            RunStrokeEngine();
            break;

#ifdef OSSM_XTOYS
        case OSSM::RunMode::XToys:
            RunXToys();
            break;
#endif

        default:
            LogDebugFormatted("Invalid mode %d", static_cast<int>(mode));
            SetLedsError();
            return;
    }
}

void OSSM::SetupStepper()
{
    LogDebug("Initializing stepper");

    engine_.init(MOTION_CORE);

    stepper_ = engine_.stepperConnectToPin(MOTOR_STEP_PIN);
    if (stepper_)
    {
        stepper_->setDirectionPin(MOTOR_DIRECTION_PIN, true);
        stepper_->setEnablePin(MOTOR_ENABLE_PIN, true);
        stepper_->setAutoEnable(false);
        stepper_->disableOutputs();
        stepper_->setAcceleration(max_acceleration_in_steps_);
    }
    // TODO: Handle failure to set up stepper

    LogDebug("Stepper initialized");
}

void OSSM::StartLeds()
{
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(ossm_leds_, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(10);
}

void OSSM::SetLedsError()
{
    // Sets the onboard LED to red to indicate an error
    FastLED.showColor(CRGB::Red);
}

void OSSM::InitInputs()
{
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    pinMode(WIFI_RESET_PIN, INPUT_PULLDOWN);
    pinMode(WIFI_CONTROL_TOGGLE_PIN, LOCAL_CONTROLLER); // choose between WIFI_CONTROLLER and LOCAL_CONTROLLER

    // Set analog pots (control knobs)
    pinMode(SPEED_POT_PIN, INPUT);
    adcAttachPin(SPEED_POT_PIN);

    pinMode(CURRENT_SENSOR_PIN, INPUT);
    adcAttachPin(CURRENT_SENSOR_PIN);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db); // This allows us to read almost full 3.3V range

    // Rotary Encoder Pushbutton
    pinMode(ENCODER_SWITCH, INPUT_PULLDOWN);

    pinMode(TOY_LIMIT_SWITCH_PIN, INPUT_PULLUP);
    pinMode(REAR_LIMIT_SWITCH_PIN, INPUT_PULLUP);
}

bool OSSM::FindHome()
{
#ifdef FORCE_SENSOR_HOMING
    return DoSensorHoming();
#else
    if (hardware_version_ >= 20)
    {
        return DoSensorlessHoming();
    }
    else
    {
        return DoSensorHoming();
    }
#endif
}

bool OSSM::DoSensorlessHoming()
{
    // Current limit in counts
    constexpr int32_t CURRENT_LIMIT = 100;

    // Set acceleration and enable stepper
    stepper_->setAcceleration(1000 * motor_properties_.stepsPerMillimeter);
    stepper_->enableOutputs();

    // Take a large average with motor enabled but not moving
    for (int i = 0; i < CurrentSenseAvg.N_SAMPLES; ++i)
    {
        CurrentSenseAvg.AddSample(analogRead(CURRENT_SENSOR_PIN));
    }

    const int32_t current_sensor_offset = CurrentSenseAvg.Value();

    ui_.UpdateMessage("Finding Home Sensorless");

    int32_t current_value = 0;

    LogDebugFormatted("Finding home sensorless, Current: Offset: %d, Value: %d",
                      current_sensor_offset, current_value);

    stepper_->setSpeedInHz(25 * motor_properties_.stepsPerMillimeter);

    while (current_value < CURRENT_LIMIT)
    {
        stepper_->move(25 * motor_properties_.stepsPerMillimeter);

        while (stepper_->isRunning() && current_value < CURRENT_LIMIT)
        {
            CurrentSenseAvg.AddSample(analogRead(CURRENT_SENSOR_PIN));

            current_value = static_cast<int32_t>(CurrentSenseAvg.Value()) - current_sensor_offset;
        }

        if (current_value >= CURRENT_LIMIT)
        {
            break;
        }
    }

    LogDebugFormatted("Current limit hit %d >= %d", current_value, CURRENT_LIMIT);

    // We are currently at the end of travel + the keep out boundary
    const uint32_t new_position =
        motor_properties_.stepsPerMillimeter * (geometry_.physicalTravel + geometry_.keepoutBoundary);

    LogDebugFormatted("Setting position to %u", new_position);

    // Set home position
    stepper_->forceStopAndNewPosition(new_position);

    // Drive free of the switch to the end of travel
    const int32_t stop_position = geometry_.physicalTravel * motor_properties_.stepsPerMillimeter;

    stepper_->moveTo(stop_position, true);

    // HACK: Need to call this twice or the stepper won't move
    if (stepper_->moveTo(stop_position, true) != MOVE_OK)
    {
        LogError("Failed to move to 0");
        SetLedsError();
        for (;;)
        {
        }
    }

    LogDebug("Homing complete");
    ui_.UpdateMessage("Homing complete");

    vTaskDelay(100);

    return true;
}

bool OSSM::DoSensorHoming()
{
    LogDebug("Finding home with Sensors");
    ui_.UpdateMessage("Finding Home with Sensors");

    stepper_->setAcceleration(max_acceleration_in_steps_ / 10);
    stepper_->setSpeedInHz(10 * motor_properties_.stepsPerMillimeter);
    stepper_->enableOutputs();

    ui_.UpdateMessage("Finding toy end");

    // TODO: Can we use the StrokeEngine homing procedure instead of this?

    // NOTE: This was pretty much copied from StrokeEngine

    // Check if we are already at the toy end
    if (digitalRead(TOY_LIMIT_SWITCH_PIN) == LOW)
    {
        LogDebug("Limit switch already activated");

        // Move the keepout distance away (negative)
        stepper_->move(motor_properties_.stepsPerMillimeter * 2 * geometry_.keepoutBoundary * -1, true);

        // Re-find the limit
        stepper_->move(-motor_properties_.stepsPerMillimeter * 4 * geometry_.keepoutBoundary * -1);
    }
    else
    {
        float f = -motor_properties_.stepsPerMillimeter * geometry_.physicalTravel * -1;

        LogDebugFormatted("Moving to max travel: %0.2f (%d)", f, (int32_t)f);

        // Move MAX_TRAVEL towards the homing switch
        stepper_->move((int32_t)f);
    }

    // Poll homing switch
    while (stepper_->isRunning())
    {
        // Switch is active low
        if (digitalRead(TOY_LIMIT_SWITCH_PIN) == LOW)
        {
            // We are currently at the end of travel + the keep out boundary
            const uint32_t new_position =
                motor_properties_.stepsPerMillimeter * (geometry_.physicalTravel + geometry_.keepoutBoundary);

            LogDebugFormatted("Setting position to %u", new_position);

            // Set home position
            stepper_->forceStopAndNewPosition(new_position);

            // Drive free of the switch to the end of travel
            const int32_t stop_position = geometry_.physicalTravel * motor_properties_.stepsPerMillimeter;

            stepper_->moveTo(stop_position, true);

            // HACK: Need to call this twice or the stepper won't move
            if (stepper_->moveTo(stop_position, true) != MOVE_OK)
            {
                LogError("Failed to move to 0");
                SetLedsError();
                for (;;)
                {
                }
            }

            LogDebug("Limit hit");

            // Break loop, home was found
            break;
        }

        // Pause the task for 20ms to allow other tasks
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }

    LogDebug("Homing Done");

    ui_.UpdateMessage("Homing done");

    return true;
}

void OSSM::PromptForRunMode()
{
    while (!encoder_switch_state_)
    {
        const auto encoderVal = abs(encoder_.read()) / 2;

        constexpr auto RUN_MODE_COUNT = static_cast<int>(RunMode::MAX);
        const auto newRunMode = static_cast<RunMode>(encoderVal % RUN_MODE_COUNT);

        switch (newRunMode)
        {
            case RunMode::Simple:
                ui_.UpdateMessage("Simple Penetration");
                break;

            case RunMode::StrokeEngine:
                ui_.UpdateMessage("Stroke Engine");
                break;

#ifdef OSSM_XTOYS
            case RunMode::XToys:
                ui_.UpdateMessage("XToys with Bluetooth");
                break;
#endif

            default:
                ui_.UpdateMessage("Invalid Mode");
                SetLedsError();
                for (;;)
                {
                }
        }

        active_run_mode_ = newRunMode;

        vTaskDelay(50);

        // Run the encoder update function because the UI thread hasn't started yet
        UpdateEncoderButton();
    }

    // Reset encoder to zero
    encoder_.write(0);
}

void OSSM::ModeSpecificSetup()
{
    LogDebugFormatted("%s: Mode: %d", __FUNCTION__, static_cast<int>(active_run_mode_));

    const auto mode = active_run_mode_;

    switch (mode)
    {
        case OSSM::RunMode::Simple:
            stepper_->enableOutputs();
            ui_.UpdateMessage("OSSM Ready to Play");
            break;

        case OSSM::RunMode::StrokeEngine:
            // Takes control of stepper
            stroke_engine_.begin(&geometry_, &motor_properties_, stepper_);

            stroke_engine_.setSpeed(0, true);
            stroke_engine_.setDepth(travel_in_mm_, true);
            stroke_engine_.setStroke(0, true);

            // We have driven free from the limit switch so we are at MAX_TRAVEL for Stroke Engine
            stroke_engine_.thisIsHome(5.0, false, true);

            ui_.SwitchToStrokeEngineUi();
            ui_.UpdateMessage(stroke_engine_.getPatternName(stroke_engine_.getPattern()));

            stroke_engine_.startPattern();
            break;

#ifdef OSSM_XTOYS
        case OSSM::RunMode::XToys:
            ui_.UpdateMessage("OSSM XToys Mode Ready");
            stepper_->setAcceleration(max_acceleration_in_steps_);
            xtoys_.Setup();
            break;
#endif

        default:
            LogDebugFormatted("Invalid mode %d", static_cast<int>(mode));
            return;
    }
}

void OSSM::RunPenetrate()
{
    if ((stroke_percent_ <= POT_DEADZONE_PERCENT) || (speed_percent_ <= POT_DEADZONE_PERCENT))
    {
        vTaskDelay(5);
        return;
    }

    int32_t next_target_position;

    // NOTE: Using travel_in_steps_ instead of 0 here to invert stroke, so it starts at the minimum and goes deeper

    if (stepper_->getCurrentPosition() != travel_in_steps_)
    {
        next_target_position = travel_in_steps_;
        LogDebug("Moving stepper home");
    }
    else
    {
        next_target_position = travel_in_steps_ - (stroke_percent_ * travel_in_steps_);
        LogDebug("Moving stepper to end");
    }

    const int32_t target_speed = speed_percent_ * max_steps_per_second_;

    LogDebugFormatted("Moving stepper to position %d at speed %d", next_target_position, target_speed);
    vTaskDelay(50);

    if (stepper_->setAcceleration(max_acceleration_in_steps_) < 0)
    {
        LogErrorFormatted("Failed to set acceleration to %d", max_acceleration_in_steps_);
    }

    if (stepper_->setSpeedInHz(target_speed) < 0)
    {
        LogErrorFormatted("Failed to set speed to %d", target_speed);
    }

    const auto move_result = stepper_->moveTo(next_target_position, true);
    if (move_result != MOVE_OK)
    {
        LogErrorFormatted("Failed to move %d", move_result);
    }
}

void OSSM::RunStrokeEngine()
{
    const auto last_pattern = stroke_engine_.getPattern();

    const float target_speed = speed_percent_ * STROKE_ENGINE_MAX_STROKES_PER_MIN;
    const float target_stroke = stroke_percent_ * travel_in_mm_;
    const float target_sensation = static_cast<float>(sensation_value_);

    bool pattern_changed = false;

    if (last_pattern != stroke_engine_pattern_)
    {
        LogDebugFormatted("Changing pattern from %d to %d", last_pattern, stroke_engine_pattern_);
        stroke_engine_.setPattern(stroke_engine_pattern_, false);
        stroke_engine_.startPattern();
        pattern_changed = true;
    }

    if (pattern_changed || (abs(target_speed - last_target_speed_) > STROKE_ENGINE_HYSTERESIS))
    {
        LogDebugFormatted("Changing speed from %0.2f to %0.2f", last_target_speed_, target_speed);
        stroke_engine_.setSpeed(target_speed, true);
        last_target_speed_ = target_speed;
    }

    if (pattern_changed || (abs(target_stroke - last_target_stroke_) > STROKE_ENGINE_HYSTERESIS))
    {
        LogDebugFormatted("Changing speed from %0.2f to %0.2f", last_target_stroke_, target_stroke);
        stroke_engine_.setStroke(target_stroke, true);
        last_target_stroke_ = target_stroke;
    }

    if (pattern_changed || (abs(target_sensation - last_sensation_value_) > STROKE_ENGINE_HYSTERESIS))
    {
        LogDebugFormatted("Changing sensation from %0.2f to %0.2f", last_sensation_value_, target_sensation);
        stroke_engine_.setSensation(target_sensation, true);
        last_sensation_value_ = target_sensation;
    }

    vTaskDelay(200);
}

#ifdef OSSM_XTOYS
void OSSM::RunXToys()
{
    uint8_t position_percent;
    uint16_t duration_ms;

    const auto cmdType = xtoys_.PopCommand(position_percent, duration_ms, 25);

    switch (cmdType)
    {
        case OssmXToys::CmdType::NONE:
            return;

        case OssmXToys::CmdType::ENABLE:
            stepper_->enableOutputs();
            return;

        case OssmXToys::CmdType::DISABLE:
            stepper_->disableOutputs();
            return;

        case OssmXToys::CmdType::STOP:
            if (stepper_->isRunning())
            {
                stepper_->setAcceleration(max_acceleration_in_steps_);
                stepper_->applySpeedAcceleration();
                stepper_->stopMove();
            }
            return;

        default:
            return;

        case OssmXToys::CmdType::MOVE:
            // Fallthrough to code below
            break;
    }

    const int32_t current_position = stepper_->getCurrentPosition();

    // Calculate the target position
    // NOTE: The 'out' variables intentionally invert the range. Zero is all the way extended
    const int32_t target_position = map(position_percent, 0, 100, travel_in_steps_, 0);

    const uint32_t position_difference = static_cast<uint32_t>(std::abs<int32_t>(target_position - current_position));

    LogDebugFormatted("Position: Cur: %d, Target: %d, Diff: %d", current_position, target_position,
                      position_difference);

    // Calculated speed in steps / s
    constexpr uint32_t MS_PER_S = 1000;
    const uint32_t calculated_speed = (MS_PER_S * position_difference) / static_cast<uint32_t>(duration_ms);
    const uint32_t constrained_speed = constrain(calculated_speed, 0, max_steps_per_second_);

    LogDebugFormatted("Calc speed %ld, Speed: %d", static_cast<long int>(calculated_speed), constrained_speed);

    stepper_->setSpeedInHz(constrained_speed);
    stepper_->setAcceleration(max_acceleration_in_steps_);
    stepper_->moveTo(target_position);
}
#endif

int OSSM::ReadEepromSettings()
{
    LogDebug("Read EEPROM");

    EEPROM.begin(EEPROM_SIZE);
    EEPROM.get(0, hardware_version_);
    EEPROM.get(4, lifetime_strokes_);
    EEPROM.get(12, lifetime_travel_distance_meters_);
    EEPROM.get(20, lifetime_seconds_on_);

    return hardware_version_;
}

void OSSM::WriteEepromSettings()
{
    // Be very careful with this so you don't break your configuration!
    LogDebug("Write EEPROM");

    EEPROM.begin(EEPROM_SIZE);
    EEPROM.put(0, HW_VERSION);
    EEPROM.put(4, 0);
    EEPROM.put(12, 0);
    EEPROM.put(20, 0);
    EEPROM.commit();

    LogDebug("EEPROM written");
}

void OSSM::UpdateAnalogInputs()
{
    const bool is_stroke_engine = active_run_mode_ == RunMode::StrokeEngine;

    // Change the sensation when the button is held down
    const bool adj_sensation = is_stroke_engine && encoder_switch_state_ && is_long_press_;

    speed_percent_ = GetAnalogAveragePercent(SPEED_POT_PIN, 50);

    const auto encoder_value = encoder_.read();

    if (!adj_sensation)
    {
        // Adjust the stroke value if we are not adjusting the sensation
        last_stroke_encoder_position_ = constrain(last_stroke_encoder_position_ + encoder_value, 0, 100);
        stroke_percent_ = static_cast<float>(last_stroke_encoder_position_) / 100.0;
    }
    else
    {
        // NOTE: Sensation value is ranged from -100 to 100, so we just work with that here
        sensation_value_ = constrain(sensation_value_ + encoder_value, -100, 100);
    }

    // Reset encoder for tracking changes instead of absolute position
    encoder_.write(0);
}

void OSSM::UpdateEncoderButton()
{
    constexpr unsigned long DEBOUNCE_INTERVAL_MS = 50;
    constexpr unsigned long LONG_PRESS_INTERVAL_MS = 250;

    const auto new_state = digitalRead(ENCODER_SWITCH);

    if (new_state != last_encoder_switch_state_)
    {
        last_encoder_change_ms_ = millis();
    }
    else
    {
        // Only valid for the duration from now till the next call of this function
        encoder_switch_changed_ = false;
    }

    // If we are currently pressing the button and the long press interval has expired
    if (encoder_switch_state_ && !is_long_press_ && (millis() - last_press_ms_) >= LONG_PRESS_INTERVAL_MS)
    {
        LogDebugFormatted("Long press started %ld", millis());
        is_long_press_ = true;
    }

    // Debounce button
    if ((millis() - last_encoder_change_ms_) >= DEBOUNCE_INTERVAL_MS)
    {
        if (new_state != encoder_switch_state_)
        {
            if (new_state)
            {
                // Save the time that we had a press and reset the long press state
                last_press_ms_ = millis();
                is_long_press_ = false;
            }

            // NOTE: Don't clear is_long_press_ on release, we use it to inform client code
            //       that the release event was a result of a long press and that they
            //       shouldn't react to it normally

            encoder_switch_state_ = new_state;
            encoder_switch_changed_ = true;

            LogDebugFormatted("Change detected: %ld, %s, %s", last_encoder_change_ms_,
                              encoder_switch_state_ ? "true" : "false", encoder_switch_changed_ ? "true" : "false");
        }
    }
    last_encoder_switch_state_ = new_state;
}
