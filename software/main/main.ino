#include "./include/pin_allocation.hpp"
#include "./include/util.hpp"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <VL53L1X.h>
#include "Bitcraze_PMW3901.h"
#include "BluetoothSerial.h"

Util util;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
VL53L1X vl53l1x;
Bitcraze_PMW3901 pmw3901(PMW3901_CS_PIN);
BluetoothSerial SerialBT;

unsigned long previous_ms = 0;

void setup()
{
  // serial communication
  Wire.begin();
  Serial.begin(115200);

  // motor
  pinMode(MOTOR1_PWM_PIN, OUTPUT);
  pinMode(MOTOR2_PWM_PIN, OUTPUT);
  pinMode(MOTOR3_PWM_PIN, OUTPUT);
  pinMode(MOTOR4_PWM_PIN, OUTPUT);
  pinMode(MOTOR5_PWM_PIN, OUTPUT);
  pinMode(MOTOR6_PWM_PIN, OUTPUT);

  // initialize IMU sensor
  while (!bno.begin())
  {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }

  // initialize ToF sensor
  vl53l1x.setTimeout(500);
  while (!vl53l1x.init()) {
    Serial.println("Ooops, no VL53L1X detected ... Check your wiring or I2C ADDR!");
  }
  vl53l1x.setDistanceMode(VL53L1X::Long);
  vl53l1x.setMeasurementTimingBudget(50000);
  vl53l1x.startContinuous(50);

  // initialize optical flow sensor
  while (!pmw3901.begin())
  {
    Serial.println("Ooops, no PMW3901 detected ... Check the SPI connection!");
  }

  // bluetooth
  SerialBT.begin("ESP32_drone");  // Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  // debug pins setup
  pinMode(LED_DEBUG_PIN, OUTPUT);
  pinMode(SW_DEBUG_PIN, INPUT);

  // notification
  util.blink_led();

  delay(300);
}

void loop()
{
  // timer
  const unsigned long interval_ms = 50;
  unsigned long current_ms = millis();
  if (current_ms - previous_ms < interval_ms) return;
  previous_ms = current_ms;

  // communication
  uint8_t recv_data[2] = {0x0, 0x0};
  if (SerialBT.available()) {
    SerialBT.readBytes(recv_data, 2);
  }

  // turn on motors when a switch is pressed
  int motor_pwm = 255;
  if (util.is_builtin_button_pressed()) {
    util.on_led();
    analogWrite(MOTOR1_PWM_PIN, motor_pwm);
    analogWrite(MOTOR2_PWM_PIN, motor_pwm);
    analogWrite(MOTOR3_PWM_PIN, motor_pwm);
    analogWrite(MOTOR4_PWM_PIN, motor_pwm);
    analogWrite(MOTOR5_PWM_PIN, motor_pwm);
    analogWrite(MOTOR6_PWM_PIN, motor_pwm);
  } else {
    util.off_led();
    analogWrite(MOTOR1_PWM_PIN, 0);
    analogWrite(MOTOR2_PWM_PIN, 0);
    analogWrite(MOTOR3_PWM_PIN, 0);
    analogWrite(MOTOR4_PWM_PIN, 0);
    analogWrite(MOTOR5_PWM_PIN, 0);
    analogWrite(MOTOR6_PWM_PIN, 0);
  }

  // ToF sensor
  Serial.print("ToF sensor output: ");
  Serial.print(vl53l1x.read());
  Serial.println("mm");

  // imu
  sensors_event_t accelerometerData, angVelocityData;
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  util.printIMUEvent(&angVelocityData);
  util.printIMUEvent(&accelerometerData);
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  // optical flow sensor
  int16_t dx, dy;
  pmw3901.readMotionCount(&dx, &dy);
  Serial.print("Optical Flow sensor output: dx: ");
  Serial.print(dx);
  Serial.print(" dy: ");
  Serial.println(dy);
}
