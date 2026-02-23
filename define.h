//POS CONTROL VALUE
#define MAX_ANGLE 45
#define MAX_YAW_ANGULAR 45

#define START_SPEED 1200
#define MIN_SPEED 1000
#define MAX_SPEED 2000

int LED = A2;
unsigned long LED_TIMER = 0;

//===========================ISR===========================//

volatile unsigned long channal_read_timer;
volatile unsigned long ch1_timer, ch2_timer, ch3_timer, ch4_timer, ch5_timer, ch6_timer;

volatile boolean ch1_s, ch2_s, ch3_s, ch4_s, ch5_s, ch6_s;

//===========================FAILSAFE===========================//

int FAIL_CODE = 0;

int SIG_fail_safe_counter = 0;
volatile bool SIG_fail_safe = 0;
unsigned long SIG_fail_safe_timer = 0;

bool IMU_fail_safe = 0;

//===========================IMU===========================//
double IMU_ROTATION_DEGREE = 90;
float imu_sin, imu_cos; // pre-computed in setup()

long gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;

float gyro_pitch, gyro_roll, gyro_yaw;

float PITCH, ROLL, YAW;

float pitch_base = 0;
float roll_base = 0;
float yaw_base = 0;

int temperature;

unsigned long IMU_request_timer;
unsigned long ahrs_timer;

//--- Mahony AHRS state ---
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // gyro bias estimator
float mahonyKp = 2.0f;   // proportional gain
float mahonyKi = 0.005f; // integral gain

//===========================POSE===========================//
//PID
float CONTROL_GAIN = 5;

float POS_P = 0.15;
float POS_I = 0.15;
float POS_D = 0.5;

int POS_I_MAX = 100;
int POS_D_MAX = 100;
int GYRO_PID_MAX = 500;

float YAW_P = 3;
float YAW_I = 0;
float YAW_D = 1;
int YAW_I_MAX = 20;
int YAW_D_MAX = 20;

float HEIGHT_P = 0.1;
float HEIGHT_I = 0;
float HEIGHT_D = 0;

//PID working state
float PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL;
float ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL;
float YAW_P_VAL, YAW_I_VAL, YAW_D_VAL;

float gyro_pitch_prev, gyro_roll_prev, gyro_yaw_prev; // D-on-measurement

long PITCH_PID_OUTPUT, ROLL_PID_OUTPUT, YAW_PID_OUTPUT;

unsigned long PID_LOOP_TIMER = 0;

//===========================RC===========================//
int MODE = 1;

int CH1_MAX = 1932, CH1_MIN = 1108;
int CH2_MAX = 1932, CH2_MIN = 1108;
int CH3_MAX = 1932, CH3_MIN = 1108;
int CH4_MAX = 1932, CH4_MIN = 1108;
int CH5_MAX = 2000, CH5_MIN = 1000;
int CH6_MAX = 2000, CH6_MIN = 1000;

volatile int CH[6] = { 0,0,0,0,0,0 };

//===========================Serial===========================//
char DATA[64];
uint8_t DATA_idx = 0;

bool READ_START = 0;
bool REMOTE_CONTROLL = 0;
bool ANG_PID_ADJUST = 0;
bool GYR_PID_ADJUST = 0;
bool RX_END = 0;

bool debug = 0;
int DEBUGGING_DATA[30];

struct DEBUG_COLLECTION {
    float GYRO_PITCH;
    float GYRO_ROLL;
    float GYRO_YAW;
    float ANGLE_PITCH;
    float ANGLE_ROLL;
    float PITCH;
    float ROLL;
    float YAW;
    float PITCH_P_VAL;
    float PITCH_I_VAL;
    float PITCH_D_VAL;
    int PITCH_PID_OUTPUT;
}data;

//===========================OUTPUT===========================//
int M1_VAL, M2_VAL, M3_VAL, M4_VAL, M5_VAL, M6_VAL;

unsigned long esc_output_timer;
unsigned long esc_start_timer;
unsigned long esc_ch_timer[6];

unsigned long ARM_TIMER;
boolean ARM;

//===========================sys_data===========================//

#define EEPROM_MAGIC_VALUE 0xA5

typedef struct EEPROM_STRUCT {

	uint8_t magic;

	float CONTROL_GAIN;

	float POS_P;
	float POS_I;
	float POS_D;

	int POS_I_MAX;
	int POS_D_MAX;
	int GYRO_PID_MAX;

	float YAW_P;
	float YAW_I;
	float YAW_D;
	int YAW_I_MAX;
	int YAW_D_MAX;

	float HEIGHT_P;
	float HEIGHT_I;
	float HEIGHT_D;

	double IMU_ROTATION_DEGREE;

	float mahonyKp;
	float mahonyKi;

	float pitch_base;
	float roll_base;
	float yaw_base;
}; EEPROM_STRUCT memory;

bool have_written = 1;

//===========================BATTERY===========================//
#define VBAT_PIN A0
#define VBAT_SCALE 5.7f        // voltage divider ratio (adjust to your hardware)
#define VBAT_CELL_MIN 3.3f     // low voltage per cell (V)
#define VBAT_CELL_FULL 4.2f    // full charge per cell (V)

float battery_voltage = 0;
uint8_t battery_cells = 0;
float battery_low_threshold = 0;
bool battery_low = false;
