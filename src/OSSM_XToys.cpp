#include "OSSM_XToys.h"

#include <Arduino.h>
#include <BLE2902.h>
#include <BLE2904.h>
#include <BLEDevice.h>

#include "OSSM_Config.h"
#include "Utilities.h"

constexpr auto FIRMWARE_VERSION = "v1.1";

#define SERVICE_UUID "e556ec25-6a2d-436f-a43d-82eab88dcefd"
#define CONTROL_CHARACTERISTIC_UUID "c4bee434-ae8f-4e67-a741-0607141f185b"
#define SETTINGS_CHARACTERISTIC_UUID "fe9a02ab-2713-40ef-a677-1716f2c03bad"

// WRITE
// T-Code messages in the format:
// ex. L199I100 = move linear actuator to the 99% position over 100ms
// ex. L10I100 = move linear actuator to the 0% position over 100ms
// DSTOP = stop
// DENABLE = enable motor (non-standard T-Code command)
// DDISABLE = disable motor (non-standard T-Code command)

// WRITE
// Preferences in the format:
// minSpeed:200 = set minimum speed of half-stroke to 200ms (used by XToys client)
// maxSpeed:2000 = set maximum speed of half-stroke to 2000ms (used by XToys client)
// READ
// Returns all preference values in the format:
// minSpeed:200,maxSpeed:2000,maxOut:0,maxIn:1000

class SettingsCallbacks : public BLECharacteristicCallbacks
{
public:
    SettingsCallbacks(OssmXToys* ossmXToys) : ossmXToys_(ossmXToys) {}

    void onWrite(BLECharacteristic* characteristic)
    {
        ossmXToys_->HandleSettingsWrite(characteristic->getValue());
    }

private:
    OssmXToys* ossmXToys_;
};

// Received T-Code command
class ControlCallbacks : public BLECharacteristicCallbacks
{
public:
    ControlCallbacks(OssmXToys* ossmXToys) : ossmXToys_(ossmXToys) {}

    void onWrite(BLECharacteristic* characteristic)
    {
        ossmXToys_->HandleControlWrite(characteristic->getValue());
    }

private:
    OssmXToys* ossmXToys_;
};

// Client connected to OSSM over BLE
class ServerCallbacks : public BLEServerCallbacks
{
public:
    ServerCallbacks(OssmXToys* ossmXToys) : ossmXToys_(ossmXToys) {}

    void onConnect(BLEServer* server, esp_ble_gatts_cb_param_t* param)
    {
        ossmXToys_->HandleConnect(server, param);
    };

    void onDisconnect(BLEServer* server)
    {
        ossmXToys_->HandleDisconnect(server);
    }

private:
    OssmXToys* ossmXToys_;
};

void OssmXToys::Setup()
{
    // TODO: Move somewhere else and split out size
    command_queue_ = xQueueCreate(32, sizeof(Cmd));

    UBaseType_t uxHighWaterMark;

    LogDebug("Initializing BLE Server");

    settings_callbacks_ = new SettingsCallbacks(this);
    control_callbacks_ = new ControlCallbacks(this);
    server_callbacks_ = new ServerCallbacks(this);

    BLEDevice::init("OSSM");

    ble_server_ = BLEDevice::createServer();
    ble_server_->setCallbacks(server_callbacks_);

    LogDebug("Initializing Info Service");

    info_service_ = ble_server_->createService(BLEUUID((uint16_t)0x180a));

    BLE2904* softwareVersionDescriptor = new BLE2904();
    softwareVersionDescriptor->setFormat(BLE2904::FORMAT_UINT8);
    softwareVersionDescriptor->setNamespace(1);
    softwareVersionDescriptor->setUnit(0x27ad);

    software_version_characteristic_ =
        info_service_->createCharacteristic((uint16_t)0x2a28, BLECharacteristic::PROPERTY_READ);
    software_version_characteristic_->addDescriptor(softwareVersionDescriptor);
    software_version_characteristic_->addDescriptor(new BLE2902());
    software_version_characteristic_->setValue(FIRMWARE_VERSION);

    LogDebug("Starting Info Service");

    info_service_->start();

    LogDebug("Initializing BLE Service");

    ble_service_ = ble_server_->createService(SERVICE_UUID);

    control_characteristic_ = ble_service_->createCharacteristic(
        CONTROL_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    control_characteristic_->addDescriptor(new BLE2902());
    control_characteristic_->setValue("");
    control_characteristic_->setCallbacks(control_callbacks_);

    settings_characteristic_ = ble_service_->createCharacteristic(
        SETTINGS_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
    settings_characteristic_->addDescriptor(new BLE2902());
    settings_characteristic_->setValue("");
    settings_characteristic_->setCallbacks(settings_callbacks_);

    LogDebug("Starting BLE Service");

    ble_service_->start();

    LogDebug("Configuring advertising");

    auto* advertising = BLEDevice::getAdvertising();

    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
    advertising->setMinPreferred(0x12);

    LogDebug("Starting advertising");

    BLEDevice::startAdvertising();

    UpdateSettingsCharacteristic();

    LogDebug("BLE Server initialized...");

    uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    LogDebugFormatted("Ble Free Stack size %ld \n", static_cast<long int>(uxHighWaterMark));
}

void OssmXToys::HandleConnect(BLEServer*, esp_ble_gatts_cb_param_t* param)
{
    LogDebug("BLE Connected");

    device_connected_ = true;

    esp_ble_conn_update_params_t conn_params = {0};
    memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));

    // For the IOS system, please reference the apple official documents about the ble
    // connection parameters restrictions
    conn_params.latency = 0;
    conn_params.max_int = 0x12; // max_int = 0x48*1.25ms
    conn_params.min_int = 0x12; // min_int = 0x24*1.25ms
    conn_params.timeout = 800;  // timeout = *10ms

    // start sent the update connection parameters to the peer device.
    esp_ble_gap_update_conn_params(&conn_params);
}

void OssmXToys::HandleDisconnect(BLEServer* server)
{
    LogDebug("BLE Disconnected");

    device_connected_ = true;

    server->startAdvertising();
}

const char* OssmXToys::BuildSettingsCString()
{
    // Buffer must be large enough to hold 4 ints plus the text for each setting and a NULL
    // 4 * len('4294967295') + 4 * 16 + 1 = 105, round up to 128
    constexpr size_t BUF_SIZE = 128;
    static char buf[BUF_SIZE];

    const auto sz = snprintf(buf, BUF_SIZE, "maxIn:%d,maxOut:%d,maxSpeed:%d,minSpeed:%d", max_in_position_,
                             max_out_position_, max_speed_, min_speed_);

    if (sz < 0)
    {
        LogError("Failed to snprintf for settings");
        return nullptr;
    }

    if (sz >= BUF_SIZE)
    {
        LogErrorFormatted("Failed to build settings string into buffer, needed %zu bytes, had %zu", sz, BUF_SIZE);
        return nullptr;
    }

    return buf;
}

void OssmXToys::UpdateSettingsCharacteristic()
{
    const auto newSettingsCStr = BuildSettingsCString();
    if (newSettingsCStr)
    {
        settings_characteristic_->setValue(newSettingsCStr);
    }
}

void OssmXToys::HandleSettingsWrite(const std::string& value)
{
    LogDebugFormatted("%s: '%s'", __FUNCTION__, value.data());

    const auto colon_pos = value.find(':');
    if (colon_pos == std::string::npos)
    {
        LogErrorFormatted("Failed to find colon in setting '%s'", value.data());
        return;
    }

    const auto setting_key = value.substr(0, colon_pos);
    const auto setting_value = value.substr(colon_pos + 1);

    // Gets sent before the actual value
    if (setting_value.empty())
    {
        return;
    }

    int setting_value_int;
    if (!ParseInt(setting_value, setting_value_int))
    {
        LogErrorFormatted("Failed to parse setting '%s' value '%s'", setting_key.data(), setting_value.data());
        return;
    }

    // NOTE: Original comment, not sure how to change the settings that XToys uses
    // Keeping this for not breaking xtoy Side until I get second integration with hardcoded settings.
    if (setting_key == "maxIn")
    {
        max_in_position_ = setting_value_int;
    }
    if (setting_key == "maxOut")
    {
        max_out_position_ = setting_value_int;
    }
    if (setting_key == "maxSpeed")
    {
        max_speed_ = setting_value_int;
    }
    if (setting_key == "minSpeed")
    {
        min_speed_ = setting_value_int;
    }

    UpdateSettingsCharacteristic();
}

void OssmXToys::HandleControlWrite(const std::string& value)
{
    LogDebugFormatted("%s: '%s'", __FUNCTION__, value.data());

    const bool is_clear_cmd = value.back() == 'C';

    bool clear_queue = false;

    Cmd cmd;
    cmd.type_ = CmdType::MOVE;

    // Check for high priority commands first

    if (value == "DSTOP")
    {
        // LogDebug("Stop Motor");

        cmd.type_ = CmdType::STOP;
        clear_queue = true;
    }
    else if (value == "DENABLE")
    {
        // LogDebug("Enable Motor");

        cmd.type_ = CmdType::ENABLE;
        clear_queue = true;
    }
    else if (value == "DDISABLE")
    {
        // LogDebug("Disable Motor");

        cmd.type_ = CmdType::DISABLE;
        clear_queue = true;
    }
    else if (value.front() == 'D')
    {
        // Handle other device status message (the code isn't handling any additional T-Code 'D' messages currently)

        LogDebugFormatted("Unhandled device status code '%s'", value.data());
        return;
    }
    else if (value.front() == 'L')
    {
        // Handle all motion commands

        cmd.type_ = CmdType::MOVE;

        constexpr size_t POSITION_START_POS = 2;

        const auto i_pos = value.find('I', POSITION_START_POS);

        if (i_pos == std::string::npos)
        {
            LogErrorFormatted("Failed to find I in value '%s'", value.data());
            return;
        }

        auto position_str = value.substr(POSITION_START_POS, i_pos - POSITION_START_POS);
        if (position_str.size() > 2)
        {
            // TODO: Is it right to just cap this off?
            position_str.resize(2);
        }

        int position_value;
        if (!ParseInt(position_str, position_value))
        {
            LogErrorFormatted("Failed to parse position '%s'", position_str.data());
            return;
        }

        auto str_size = value.size();
        if (is_clear_cmd)
        {
            --str_size;
        }

        const auto duration_str = value.substr(i_pos + 1, str_size - (i_pos + 1));

        int duration_value;
        if (!ParseInt(duration_str, duration_value))
        {
            LogErrorFormatted("Failed to parse duration '%s'", duration_str.data());
            return;
        }

        // LogDebugFormatted("Command - Pos: %d, Dur: %d", position_value, duration_value);

        cmd.position_percent_ = static_cast<uint8_t>(position_value);
        cmd.duration_ms_ = static_cast<uint16_t>(duration_value);
    }

    // Movement command includes a clear existing commands flag, clear queue start slow movement counter
    if (is_clear_cmd)
    {
        // LogDebugFormatted("Clear queue command '%s'", value.data());
        clear_queue = true;
    }

    if (clear_queue)
    {
        // LogDebug("Clearing queue");
        xQueueReset(command_queue_);
    }

    // LogDebug("Queueing command");
    xQueueSendToBack(command_queue_, &cmd, portMAX_DELAY);
}

OssmXToys::CmdType OssmXToys::PopCommand(uint8_t& position_percent, uint16_t& duration_ms, TickType_t ticks_to_wait)
{
    Cmd cmd;

    if (xQueueReceive(command_queue_, &cmd, ticks_to_wait) == pdTRUE)
    {
        if (cmd.type_ == CmdType::MOVE)
        {
            position_percent = map(cmd.position_percent_, 0, 100, max_in_position_, max_out_position_);
            duration_ms = constrain(cmd.duration_ms_, max_speed_, min_speed_);
        }

        return cmd.type_;
    }

    return CmdType::NONE;
}
