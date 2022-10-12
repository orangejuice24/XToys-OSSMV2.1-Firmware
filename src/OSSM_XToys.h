#pragma once

#include <Arduino.h>
#include <BLEServer.h>
#include <BLEService.h>
#include <BLEUtils.h>
#include <list>
#include <string>

class SettingsCallbacks;
class ControlCallbacks;
class ServerCallbacks;

class OssmXToys
{
public:
    enum class CmdType : uint8_t
    {
        NONE,
        STOP,
        ENABLE,
        DISABLE,
        MOVE,
    };

    void Setup();

    void HandleConnect(BLEServer*, esp_ble_gatts_cb_param_t*);
    void HandleDisconnect(BLEServer*);

    void HandleSettingsWrite(const std::string&);
    void HandleControlWrite(const std::string&);

    CmdType PopCommand(uint8_t& position_percent, uint16_t& duration_ms, TickType_t ticks_to_wait);

private:
    const char* BuildSettingsCString();
    void UpdateSettingsCharacteristic();

    BLEServer* ble_server_;
    BLEService* ble_service_;

    BLECharacteristic* control_characteristic_;
    BLECharacteristic* settings_characteristic_;
    BLECharacteristic* software_version_characteristic_;

    BLEService* info_service_;

    SettingsCallbacks* settings_callbacks_;
    ControlCallbacks* control_callbacks_;
    ServerCallbacks* server_callbacks_;

    bool device_connected_{false};

    int max_in_position_{0};
    int max_out_position_{100};
    int max_speed_{50};
    int min_speed_{1000};

    struct Cmd
    {
        CmdType type_;

        // Position in percent of travel
        uint8_t position_percent_;

        uint16_t duration_ms_;
    };

    QueueHandle_t command_queue_;
};
