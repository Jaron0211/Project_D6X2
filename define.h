//POS CONTROL VALUE
#define MAX_ANGLE 45
#define MAX_YAW_ANGULAR 45

#define START_SPEED 1200
#define MIN_SPEED 1000
#define MAX_SPEED 2000

int LED = A2;
unsigned long LED_TIMER = 0;

//===========================IMU===========================//
double IMU_ROTATION_DEGREE = 90;
double IMU_ROTATION_ANGLE = IMU_ROTATION_DEGREE / 180 * PI;

long gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector, acc_total_vector_level;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;

long gyro_pitch,gyro_roll,gyro_yaw;

float angle_pitch, angle_roll, angle_yaw;
float angle_roll_acc, angle_pitch_acc;
float PITCH, ROLL, YAW;

float pitch_cal, roll_cal, yaw_base ;
float pitch_base = 0;
float roll_base = 0;

int temperature;

bool set_gyro_angles;

bool IMU_request;
unsigned long IMU_request_timer;
long IMU_read_frequence;

bool IMU_fail_safe = 0;

//===========================POSE===========================//
//PID
float CONTROL_GAIN = 5;

float POS_P = 0.35;
float POS_I = 0.001;
float POS_D = 0;

int POS_I_MAX = 50;
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


//define
float PITCH_P_VAL , PITCH_I_VAL , PITCH_D_VAL;
float ROLL_P_VAL , ROLL_I_VAL , ROLL_D_VAL;
float YAW_P_VAL , YAW_I_VAL , YAW_D_VAL;

float PITCH_error_pre, ROLL_error_pre, YAW_error_pre;

long PITCH_PID_OUTPUT, ROLL_PID_OUTPUT, YAW_PID_OUTPUT;

float FL_ROLL, FR_ROLL, BL_ROLL, BR_ROLL;
float FL_PITCH, FR_PITCH, BL_PITCH, BR_PITCH;
float FL_YAW, FR_YAW, BL_YAW, BR_YAW;
int FL_out, FR_out, BL_out, BR_out;

float ANG_PITCH_P_VAL , ANG_PITCH_I_VAL , ANG_PITCH_D_VAL;
float ANG_ROLL_P_VAL , ANG_ROLL_I_VAL , ANG_ROLL_D_VAL;
float ANG_YAW_P_VAL , ANG_YAW_I_VAL , ANG_YAW_D_VAL;

float ANG_PITCH_error_pre, ANG_ROLL_error_pre, ANG_YAW_error_pre;

long ANG_PITCH_PID_OUTPUT, ANG_ROLL_PID_OUTPUT, ANG_YAW_PID_OUTPUT;

//===========================RC===========================//
int MODE = 1;


//===========================Serial===========================//
float PID_VAL[3] = {POS_P, POS_I, POS_D};

String DATA;

bool READ_START = 0;
bool REMOTE_CONTROLL = 0;
bool ANG_PID_ADJUST = 0;
bool GYR_PID_ADJUST = 0;
bool RX_END = 0;

bool debug = 0;
int DEBUGGING_DATA[30];

//===========================OUTPUT===========================//
int M1_VAL, M2_VAL, M3_VAL, M4_VAL, M5_VAL, M6_VAL;

unsigned long esc_output_timer;
unsigned long esc_start_timer;
unsigned long esc_ch_timer[6];

unsigned long ARM_TIMER;
boolean ARM;
