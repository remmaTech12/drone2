#include "../include/recv.h"

BluetoothSerial SerialBT;

Receiver::Receiver() {}

void Receiver::setup() {
    Serial.begin(115200);

    btStop();
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    btStart();

    SerialBT.begin("ESP32test");  // Bluetooth device name
    Serial.println("The device started, now you can pair it with bluetooth!");
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
    if (SerialBT.available()) {
        disconnect_count = 0; 
        SerialBT.readBytes(recv_data, RECEIVE_DATA_SIZE);

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
    } else {
        disconnect_count++;
    }
}

void Receiver::get_command(int data[4]) {
    data[0] = recv_data[1]; // thrust
    data[1] = recv_data[2]; // yaw
    data[2] = recv_data[3]; // pitch
    data[3] = recv_data[4]; // roll
}

void Receiver::set_arm_status(Arm &arm) {
    constexpr int left_x_val_thre = 50;
    constexpr int left_y_val_thre = 230;
    const int left_x_val = recv_data[1];
    const int left_y_val = recv_data[2];
    if (left_x_val <= left_x_val_thre && left_y_val >= left_y_val_thre ) {
        arm.set_arm_status(true);
    }
}

void Receiver::emergency_stop(Arm &arm) {
    if (is_left_switch_pressed()) {
        arm.set_arm_status(false);
    }
    constexpr int disconnect_count_thre = 10;
    if (disconnect_count > disconnect_count_thre
        || checksum_success == false
        || first_byte_check == false) {
        arm.set_arm_status(false);
    }
}