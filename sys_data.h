typedef struct EEPROM_STRUCT {

	float CONTROL_GAIN;

	float POS_P;
	double POS_I;
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
}; EEPROM_STRUCT memory;

void COLLECT_DATA() {

	memory.CONTROL_GAIN = CONTROL_GAIN;
	memory.POS_P = POS_P;
	memory.POS_I = POS_I;
	memory.POS_D = POS_D;

	memory.POS_I_MAX = POS_I_MAX;
	memory.POS_D_MAX = POS_D_MAX;
	memory.GYRO_PID_MAX = GYRO_PID_MAX;

	memory.YAW_P = YAW_P;
	memory.YAW_I = YAW_I;
	memory.YAW_D = YAW_D;
	memory.YAW_I_MAX = YAW_I_MAX;
	memory.YAW_D_MAX = YAW_D_MAX;

	memory.HEIGHT_P = HEIGHT_P;
	memory.HEIGHT_I = HEIGHT_I;
	memory.HEIGHT_D = HEIGHT_D;

	memory.IMU_ROTATION_DEGREE = IMU_ROTATION_DEGREE;
}

void WRITE_EEPROM() {
	EEPROM.put(0, memory);
}

void READ_EEPROM() {
	EEPROM.get(0, memory);
}