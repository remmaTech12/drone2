#include "Transmit.h"

// BLE Callbacks
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
private:
    Transmit* transmit;
    
public:
    MyAdvertisedDeviceCallbacks(Transmit* tx) : transmit(tx) {}
    
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        Serial.print("BLE Advertised Device found: ");
        Serial.println(advertisedDevice.toString().c_str());
        
        // ESP32_Drone_BLEデバイスを探す（UUIDとデバイス名の両方で確認）
        if (advertisedDevice.haveServiceUUID() && 
            advertisedDevice.isAdvertisingService(BLEUUID("12345678-1234-1234-1234-123456789abc")) &&
            advertisedDevice.haveName() && advertisedDevice.getName() == "ESP32_Drone_BLE") {
            
            Serial.println("Found target BLE device with matching UUID and name");
            Serial.print("Device Address: ");
            Serial.println(advertisedDevice.getAddress().toString().c_str());
            
            BLEDevice::getScan()->stop();
            transmit->myDevice = new BLEAdvertisedDevice(advertisedDevice);
            transmit->doConnect = true;
            transmit->doScan = true;
        }
    }
};

class MyClientCallback : public BLEClientCallbacks {
private:
    Transmit* transmit;
    
public:
    MyClientCallback(Transmit* tx) : transmit(tx) {}
    
    void onConnect(BLEClient* pclient) {
        transmit->connected = true;
        Serial.println("Connected to BLE Server");
    }

    void onDisconnect(BLEClient* pclient) {
        transmit->connected = false;
        Serial.println("Disconnected from BLE Server");
    }
};

Transmit::Transmit() {}

void Transmit::setup() {
    input.setup();

    bluetooth_setup();
    notify_bluetooth_setup_finished();
}

void Transmit::bluetooth_setup() {
    Serial.println("Starting BLE Client...");
    
    BLEDevice::init("");
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallback(this));
    
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(this));
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    
    Serial.println("Scanning for BLE devices...");
    pBLEScan->start(5, false);
}

void Transmit::notify_bluetooth_setup_finished() {
    pinMode(BUILTIN_LED, OUTPUT);
    for (int i = 0; i < 3; i++) {
        digitalWrite(BUILTIN_LED, HIGH);
        delay(50);
        digitalWrite(BUILTIN_LED, LOW);
        delay(50);
    }
}

uint8_t Transmit::calculate_checksum(uint8_t *data) {
  uint8_t checksum = 0;
  checksum |= 0b11000000 & data[1];
  checksum |= 0b00110000 & data[2];
  checksum |= 0b00001100 & data[3];
  checksum |= 0b00000011 & data[4];

  return checksum;
}

uint8_t Transmit::pack_switch_data() {
    uint8_t left_sw_data  = 0x01 & input.get_left_sw_val();
    uint8_t right_sw_data = 0x02 & (input.get_right_sw_val() << 1);
    return left_sw_data | right_sw_data;
}

void Transmit::transmit_data() {
    // BLE接続の管理
    if (doConnect == true) {
        if (connectToServer()) {
            Serial.println("We are now connected to the BLE Server.");
        } else {
            Serial.println("We have failed to connect to the server; there is nothing more we will do.");
        }
        doConnect = false;
    }

    // 接続が切れた場合の再スキャン
    if (connected == false && doScan == true) {
        BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect, most of the times you want to do it in callback
        doScan = false;
    }

    // デバッグ情報を追加
    static unsigned long last_debug_time = 0;
    if (millis() - last_debug_time > 1000) { // 1秒ごとにデバッグ情報を出力
        Serial.print("Transmitter Status - Connected: ");
        Serial.print(connected);
        Serial.print(", DoConnect: ");
        Serial.print(doConnect);
        Serial.print(", DoScan: ");
        Serial.println(doScan);
        last_debug_time = millis();
    }

    // 接続されていない場合は送信しない
    if (!connected) {
        delay(m_sampling_time_ms);
        return;
    }

    uint8_t send_data[TRANSMIT_DATA_SIZE];

    input.sense_value();
#ifdef DEBUG_BUILD
    Serial.print("Left Switch: ");
    Serial.print(input.get_left_sw_val());
    Serial.print(", ");
    Serial.print("Left X-axis: ");
    Serial.print(input.get_left_x_val());
    Serial.print(", ");
    Serial.print("Left Y-axis: ");
    Serial.print(input.get_left_y_val());
    Serial.print(", ");

    Serial.print("Right Switch: ");
    Serial.print(input.get_right_sw_val());
    Serial.print(", ");
    Serial.print("Right X-axis: ");
    Serial.print(input.get_right_x_val());
    Serial.print(", ");
    Serial.print("Right Y-axis: ");
    Serial.print(input.get_right_y_val());
    Serial.print("\n\n");
#endif

#ifdef ADDITIONAL_BUTTONS
    Serial.print("Left Switch2: ");
    Serial.print(input.get_left_sw2_val());
    Serial.print(", ");
    Serial.print("Right Switch2: ");
    Serial.print(input.get_right_sw2_val());
    Serial.print(", ");
#endif

    send_data[0] = 'T';
    send_data[1] = input.get_left_y_val();
    send_data[2] = input.get_left_x_val();
    send_data[3] = input.get_right_y_val();
    send_data[4] = input.get_right_x_val();
    send_data[5] = pack_switch_data();
    send_data[6] = calculate_checksum(send_data);
    
    // BLEでデータ送信
    if (pCharacteristic != nullptr) {
        pCharacteristic->writeValue(send_data, TRANSMIT_DATA_SIZE);
    }

    delay(m_sampling_time_ms);
}

bool Transmit::connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    pClient->connect(myDevice);
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(BLEUUID("12345678-1234-1234-1234-123456789abc"));
    if (pRemoteService == nullptr) {
        Serial.print("Failed to find our service UUID: ");
        Serial.println("12345678-1234-1234-1234-123456789abc");
        pClient->disconnect();
        return false;
    }
    Serial.println(" - Found our service");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pCharacteristic = pRemoteService->getCharacteristic(BLEUUID("87654321-4321-4321-4321-cba987654321"));
    if (pCharacteristic == nullptr) {
        Serial.print("Failed to find our characteristic UUID: ");
        Serial.println("87654321-4321-4321-4321-cba987654321");
        pClient->disconnect();
        return false;
    }
    Serial.println(" - Found our characteristic");

    // BLERemoteCharacteristicでは直接データを送信するだけでOK
    Serial.println(" - Ready to send data");

    return true;
}

void Transmit::scanForBLE() {
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->start(5, false);
}
