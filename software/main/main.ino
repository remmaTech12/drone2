#include "./include/recv.h"
#include "./include/imu_bno055.h"
#include "./include/pid.h"
#include "./include/motor.h"
#include "./include/def_system.h"
#include "./include/emergency.h"
#include "./include/control.h"
#include "./include/flow_pmw3901.h"

imu_bno055 imu_sensor;
flow_pmw3901 flow_sensor;
Receiver receiver;
PID pid;
Motor motor;
Arm arm;
Emergency emergency;
Control control;
unsigned long previous_ms = 0;

void setup() {
    Serial.begin(115200);

    imu_sensor.setup();
    flow_sensor.setup();
    receiver.setup();
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

void loop() {
    unsigned long current_ms = millis();
    if (current_ms - previous_ms < SAMPLING_TIME_MS) return;
    previous_ms = current_ms;

    int cmd_data[4];
    float ang_data[3];
    float angvel_data[3];
    float ctl_data[3];
    int flow_data[2];

    emergency.emergency_stop(arm, motor);

    receiver.update_data();
    receiver.get_command(cmd_data);
    receiver.set_arm_status(arm);
    receiver.emergency_stop(arm, motor);

    imu_sensor.get_attitude_data(ang_data);
    imu_sensor.get_angvel_data(angvel_data);

    flow_sensor.readMotionCount(flow_data);
    flow_sensor.printMotionCount();

    control.calculate_pid_ang(cmd_data, ang_data);
    control.calculate_pid_angvel(angvel_data);
    control.get_control_val(ctl_data);

    // motor.control(cmd_data, ctl_data, arm);
    motor.test_control(cmd_data[0]);
}