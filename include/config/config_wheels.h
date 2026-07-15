#define MotorFL_A PA1
#define MotorFL_B PA0

#define MotorFR_A PA2
#define MotorFR_B PA3

#define MotorRL_A PA6
#define MotorRL_B PA7

#define MotorRR_A PB0
#define MotorRR_B PB1

#define EncFL_A PB7
#define EncFL_B PB6

#define EncFR_A PB4
#define EncFR_B PB5

#define EncRL_A PB15
#define EncRL_B PB14

#define EncRR_A PB12
#define EncRR_B PB13

#define MAX_RPM 300
#define PULSE_PER_REV 844.8

#define LR_WHEELS_DISTANCE 0.395
#define FR_WHEELS_DISTANCE 0.420
#define WHEEL_RADIUS 0.1016
#define WHEEL_DIAMETER (WHEEL_RADIUS * 2.0f)

// PWM range for 8-bit resolution (0-255)
#define PWM_MIN -255
#define PWM_MAX 255

#define stepDelay 10

#define FL_K_P 0.65
#define FL_K_I 0.0
#define FL_K_D 0.01

#define FR_K_P 0.65
#define FR_K_I 0.0
#define FR_K_D 0.01

#define RL_K_P 0.65
#define RL_K_I 0.0
#define RL_K_D 0.01

#define RR_K_P 0.65
#define RR_K_I 0.0
#define RR_K_D 0.01