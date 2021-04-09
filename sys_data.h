
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

void SET_DATA() {

	CONTROL_GAIN = memory.CONTROL_GAIN;
	POS_P = memory.POS_P;
	POS_I = memory.POS_I;
	POS_D = memory.POS_D;

	POS_I_MAX = memory.POS_I_MAX;
	POS_D_MAX = memory.POS_D_MAX;
	GYRO_PID_MAX = memory.GYRO_PID_MAX;

	YAW_P = memory.YAW_P;
	YAW_I = memory.YAW_I;
	YAW_D = memory.YAW_D;
	YAW_I_MAX = memory.YAW_I_MAX;
	YAW_D_MAX = memory.YAW_D_MAX;

	HEIGHT_P = memory.HEIGHT_P;
	HEIGHT_I = memory.HEIGHT_I;
	HEIGHT_D = memory.HEIGHT_D;

	IMU_ROTATION_DEGREE = memory.IMU_ROTATION_DEGREE;
}

void WRITE_EEPROM() {
	COLLECT_DATA();
	EEPROM.put(0, memory);
}

void READ_EEPROM() {
	EEPROM.get(0, memory);
	SET_DATA();
}