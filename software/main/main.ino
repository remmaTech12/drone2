#include "./include/recv.h"
#include "./include/imu_bno055.h"
#include "./include/motor.h"
#include "./include/led.h"
#include "./include/def_system.h"
#include "./include/control.h"
#include "./include/flow_pmw3901.h"
#include "./include/tof_vl53l1x.h"

imu_bno055 imu_sensor;
flow_pmw3901 flow_sensor;
tof_vl53l1x tof_sensor;
Receiver receiver;
Motor motor;
Arm arm;
Control control;
led led;
unsigned long previous_position_control_ms = 0;
unsigned long previous_outer_ms = 0;
unsigned long previous_inner_ms = 0;

void setup() {
    Serial.begin(115200);

    imu_sensor.setup();
    flow_sensor.setup();
    tof_sensor.setup();
    receiver.setup();
    motor.setup();
    led.setup();

    delay(300);
}

int cmd_data[4] = {0, 127, 127, 127};
float ang_data[3];
float angvel_data[3];
float ctl_data[3];
int flow_data[2];
float xy_data[2];
float ref_xy_data[2] = {0.0f, 0.0f};
int16_t distance;
double height;

void loop() {
    unsigned long current_ms = millis();
    if (current_ms - previous_position_control_ms > SAMPLING_POSITION_CONTROL_TIME_MS) {
        tof_sensor.readDistance(distance);
        height = tof_sensor.getDistance();
        //tof_sensor.printDistance();
        flow_sensor.readMotionCount(flow_data);
        flow_sensor.calculate_velocity_position(height, ang_data);
        flow_sensor.get_position_data(xy_data);
        //flow_sensor.printMotionCount();

        control.calculate_pid_pos(cmd_data, xy_data);

        previous_position_control_ms = current_ms;
    }

    if (current_ms - previous_outer_ms > SAMPLING_OUTER_TIME_MS) {
        receiver.update_data();
        receiver.get_command(cmd_data);
        receiver.set_arm_status(arm);
        receiver.emergency_stop(arm, motor);

        if (arm.get_arm_status()) led.on();
        else {
            cmd_data[2] = 127;
            cmd_data[3] = 127;
            led.off();
        }

        imu_sensor.get_attitude_data(ang_data);
        imu_sensor.emergency_stop(arm);

        cmd_data[1] = 127.0f;
        control.calculate_pid_ang(cmd_data, ang_data);

        previous_outer_ms = current_ms;
    }

    if (current_ms - previous_inner_ms > SAMPLING_INNER_TIME_MS) {
        imu_sensor.get_angvel_data(angvel_data);
        control.calculate_pid_angvel(angvel_data);
        control.get_control_val(ctl_data);

        motor.control(cmd_data, ctl_data, arm, height);
        //motor.test_control(cmd_data[0]);

        previous_inner_ms = current_ms;
    }
}