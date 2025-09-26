#include "./include/recv.h"
#include "./include/imu_bno055.h"
#include "./include/pid.h"
#include "./include/motor.h"
#include "./include/def_system.h"
#include "./include/emergency.h"
#include "./include/control.h"

imu_bno055 imu_sensor;
Receiver receiver;
PID pid;
Motor motor;
Arm arm;
Emergency emergency;
Control control;
unsigned long previous_ms = 0;

void setup() {
    // Serial.begin(115200);  // メモリ節約のため無効化

    imu_sensor.setup();
    receiver.setup();  // BLE通信を有効化
    motor.setup();
    emergency.setup();

    pinMode(LED_DEBUG_PIN, OUTPUT);
    // blink LED_DEBUG_PIN 3 times
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_DEBUG_PIN, HIGH);
        delay(100);
        digitalWrite(LED_DEBUG_PIN, LOW);
        delay(100);
    }

    delay(300);
}

int cmd_data[4];
float ang_data[3];
float angvel_data[3];
float ctl_data[3];

void loop() {
    unsigned long current_ms = millis();
    if (current_ms - previous_ms < SAMPLING_TIME_MS) return;
    previous_ms = current_ms;

    emergency.emergency_stop(arm, motor);

    receiver.update_data();
    receiver.get_command(cmd_data);
    motor.test_control(cmd_data[0]);
    receiver.set_arm_status(arm);
    receiver.emergency_stop(arm, motor);

    imu_sensor.get_attitude_data(ang_data);
    imu_sensor.get_angvel_data(angvel_data);

    control.calculate_pid_ang(cmd_data, ang_data);
    control.calculate_pid_angvel(angvel_data);
    control.get_control_val(ctl_data);

    // motor.control(cmd_data, ctl_data, arm);
}