#include "./include/recv.h"
#include "./include/imu_bno055.h"
#include "./include/pid.h"
#include "./include/motor.h"
#include "./include/led.h"
#include "./include/def_system.h"
#include "./include/emergency.h"
#include "./include/control.h"
#include "./include/flow_pmw3901.h"
#include "./include/tof_vl53l1x.h"

imu_bno055 imu_sensor;
flow_pmw3901 flow_sensor;
tof_vl53l1x tof_sensor;
Receiver receiver;
PID pid;
Motor motor;
Arm arm;
Emergency emergency;
Control control;
led led;
unsigned long previous_ms = 0;

void setup() {
    Serial.begin(115200);

    imu_sensor.setup();
    flow_sensor.setup();
    tof_sensor.setup();
    receiver.setup();
    motor.setup();
    emergency.setup();
    led.setup();

    delay(300);
}

int cmd_data[4];
float ang_data[3];
float angvel_data[3];
float ctl_data[3];
int flow_data[2];
int16_t distance;

void loop() {
    unsigned long current_ms = millis();
    if (current_ms - previous_ms < SAMPLING_TIME_MS) return;
    previous_ms = current_ms;

    receiver.update_data();
    receiver.get_command(cmd_data);
    receiver.set_arm_status(arm);
    receiver.emergency_stop(arm, motor);

    if (arm.get_arm_status()) led.on();
    else led.off();

    imu_sensor.get_attitude_data(ang_data);
    imu_sensor.get_angvel_data(angvel_data);

    flow_sensor.readMotionCount(flow_data);
    //flow_sensor.printMotionCount();

    tof_sensor.readDistance(distance);
    //tof_sensor.printDistance();

    control.calculate_pid_ang(cmd_data, ang_data);
    control.calculate_pid_angvel(angvel_data);
    control.get_control_val(ctl_data);

    // motor.control(cmd_data, ctl_data, arm);
    motor.test_control(cmd_data[0]);
    //motor.test_control(distance);
}