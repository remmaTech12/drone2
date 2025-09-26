#ifndef Transmit_h
#define Transmit_h
#include "BLEDevice.h"
#include "BLEScan.h"
#include "BLEAdvertisedDevice.h"
#include "BLEClient.h"
#include "Arduino.h"
#include "InputCmd.h"
#include "def_system.h"

class MyAdvertisedDeviceCallbacks;
class MyClientCallback;

class Transmit {
    friend class MyAdvertisedDeviceCallbacks;
    friend class MyClientCallback;
    public:
        Transmit();
        InputCmd input;

        void setup();
        void transmit_data();

    private:
        int m_sampling_time_ms = SAMPLING_TIME_MS;
        bool connected = false;
        bool doConnect = false;
        bool doScan = false;
        
        BLEClient* pClient = nullptr;
        BLERemoteCharacteristic* pCharacteristic = nullptr;
        BLEAdvertisedDevice* myDevice = nullptr;
        
        // 特定のBLEアドレスを指定する場合（オプション）
        // BLEAddress targetAddress = BLEAddress("XX:XX:XX:XX:XX:XX");

        void bluetooth_setup();
        void notify_bluetooth_setup_finished();
        uint8_t calculate_checksum(uint8_t *data);
        uint8_t pack_switch_data();
        bool connectToServer();
        void scanForBLE();
};
#endif  // #ifndef tranmit_h
