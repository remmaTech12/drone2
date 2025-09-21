// PIN allccation
#define MOTOR_PWM1 32
#define MOTOR_PWM2 33
#define MOTOR_PWM3 25
#define MOTOR_PWM4 26
#define MOTOR_PWM5 27
#define MOTOR_PWM6 14

#define BUILTIN_LED 2
#define LED_DEBUG_PIN 12
#define SW_DEBUG_PIN 34

#define EMERGENCY_SWITCH 34
#define PMW3901_CS_PIN 5

// System setting
#define SAMPLING_TIME_MS 20
#define RECEIVE_DATA_SIZE 7
#define PID_MAX 2000
#define LIMIT_MOTOR 255

// Bias removal time
#define IMU_CNT_START_NUM 100
#define IMU_CNT_TOTAL_NUM 100
#define CTL_CNT_START_NUM 150
#define CTL_CNT_TOTAL_NUM 100

// Debug setting
//#define DEBUG_RECV_SWITCH
//#define DEBUG_RECV_JOYSTICK
//#define DEBUG_PID_COMMAND
//#define DEBUG_RECV_COMMAND
//#define DEBUG_MOTOR_COMMAND