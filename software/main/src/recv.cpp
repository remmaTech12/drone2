#include "../include/recv.h"
#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

Receiver::Receiver() {}

// BLE Server Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("BLE Client connected");
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("BLE Client disconnected");
    }
};

// BLE Characteristic Callbacks
class MyCallbacks: public BLECharacteristicCallbacks {
private:
    Receiver* receiver;
    
public:
    MyCallbacks(Receiver* recv) : receiver(recv) {}
    
    void onWrite(BLECharacteristic *pCharacteristic) {
        String rxValue = pCharacteristic->getValue();
        
        if (rxValue.length() >= RECEIVE_DATA_SIZE) {
            receiver->handleBLEWrite(rxValue);
        }
    }
};

void Receiver::setup() {
    Serial.begin(115200);
    
    // Initialize BLE with power optimization
    BLEDevice::init("ESP32_Drone_BLE");
    
    // 電力最適化設定
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);  // 最大送信電力
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P3);      // アドバタイズ時の電力
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P3);    // スキャン時の電力
    
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(BLEUUID("12345678-1234-1234-1234-123456789abc"));
    
    pCharacteristic = pService->createCharacteristic(
        BLEUUID("87654321-4321-4321-4321-cba987654321"),
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    
    pCharacteristic->setCallbacks(new MyCallbacks(this));
    pCharacteristic->addDescriptor(new BLE2902());
    
    pService->start();
    
    // 電力最適化されたアドバタイズ設定
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLEUUID("12345678-1234-1234-1234-123456789abc"));
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // 接続間隔の最小値 (7.5ms)
    pAdvertising->setMaxPreferred(0x12);  // 接続間隔の最大値 (22.5ms)
    
    // アドバタイズ間隔を長くして電力消費を削減
    pAdvertising->setMinInterval(0x20);   // 20ms
    pAdvertising->setMaxInterval(0x40);   // 40ms
    
    BLEDevice::startAdvertising();
    
    Serial.println("BLE Server started with power optimization, waiting for connections...");
    notify_bluetooth_setup_finished();
}

void Receiver::notify_bluetooth_setup_finished() {
    pinMode(BUILTIN_LED, OUTPUT);
    char blink_times = 3;
    for (int i = 0; i < blink_times; i++) {
        digitalWrite(BUILTIN_LED, HIGH);
        delay(50);
        digitalWrite(BUILTIN_LED, LOW);
        delay(50);
    }
}

uint8_t Receiver::calculate_checksum() {
  uint8_t checksum = 0;
  checksum |= 0b11000000 & recv_data[1];
  checksum |= 0b00110000 & recv_data[2];
  checksum |= 0b00001100 & recv_data[3];
  checksum |= 0b00000011 & recv_data[4];

  return checksum;
}

bool Receiver::is_left_switch_pressed() {
    uint8_t left_sw_data = 0x01 & recv_data[5];

    if (left_sw_data == 0x00 && left_sw_data != pre_left_sw_data) {
        pre_left_sw_data = left_sw_data;
#ifdef DEBUG_RECV_SWITCH
        Serial.println("Left switch is pressed.");
#endif
        return true;
    }
    pre_left_sw_data = left_sw_data;

    return false;
}

bool Receiver::is_right_switch_pressed() {
    uint8_t right_sw_data = 0x02 & recv_data[5];

    if (right_sw_data == 0x00 && right_sw_data != pre_right_sw_data) {
        pre_right_sw_data = right_sw_data;
#ifdef DEBUG_RECV_SWITCH
        Serial.println("Right switch is pressed.");
#endif
        return true;
    }
    pre_right_sw_data = right_sw_data;

    return false;
}

void Receiver::update_data() {
    // BLE接続状態の管理
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("Start advertising");
        oldDeviceConnected = deviceConnected;
    }
    
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
    
    // データはMyCallbacks::onWrite()で受信されるため、ここでは接続状態のみ管理
    if (!deviceConnected) {
        disconnect_count++;
    }

    is_data_available = deviceConnected && checksum_success;
    
    // デバッグ情報を追加
    static unsigned long last_debug_time = 0;
    if (millis() - last_debug_time > 1000) { // 1秒ごとにデバッグ情報を出力
        Serial.print("BLE Status - Connected: ");
        Serial.print(deviceConnected);
        Serial.print(", Checksum OK: ");
        Serial.print(checksum_success);
        Serial.print(", Data Available: ");
        Serial.println(is_data_available);
        last_debug_time = millis();
    }
    
    if (is_data_available) {
#ifdef DEBUG_RECV_JOYSTICK
        Serial.print("Left Joystick x: ");
        Serial.print(recv_data[1]);
        Serial.print(", Left Joystick y: ");
        Serial.print(recv_data[2]);
        Serial.print(", Right Joystick x: ");
        Serial.print(recv_data[3]);
        Serial.print(", Right Joystick y: ");
        Serial.print(recv_data[4]);
        Serial.print("\n");
#endif
    }
}

void Receiver::get_command(int data[4]) {
    if (!is_data_available) {
        // データが利用できない場合はデフォルト値を設定
        data[0] = 0; // thrust
        data[1] = 0; // yaw
        data[2] = 0; // pitch
        data[3] = 0; // roll
        return;
    }
    data[0] = recv_data[1]; // thrust
    data[1] = recv_data[2]; // yaw
    data[2] = recv_data[3]; // pitch
    data[3] = recv_data[4]; // roll
}

void Receiver::set_arm_status(Arm &arm) {
    int left_x_val = recv_data[1];
    int left_y_val = recv_data[2];
    if (left_x_val <= 50 && left_y_val >= 230 ) {
        arm.set_arm_status(true);
    }
}

void Receiver::emergency_stop(Arm &arm, Motor &motor) {
    if (is_left_switch_pressed()) {
        arm.set_arm_status(false);
        motor.stop_motor();
    }
    if (!deviceConnected || disconnect_count > 10 || checksum_success == false || first_byte_check == false) {
        arm.set_arm_status(false);
        motor.stop_motor();
    }
}

void Receiver::handleBLEWrite(String rxValue) {
    if (rxValue.length() >= RECEIVE_DATA_SIZE) {
        memcpy(recv_data, rxValue.c_str(), RECEIVE_DATA_SIZE);
        
        if (recv_data[0] != 'T') {
            Serial.print("Receive error!");
            first_byte_check = false;
            return;
        }

        if (recv_data[6] != calculate_checksum()) {
            Serial.print("Decode error!");
            checksum_success = false;
            return;
        }
        
        checksum_success = true;
        first_byte_check = true;
        disconnect_count = 0;
    }
}