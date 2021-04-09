
void STABLE() {
	int pos_value[4] = { 0, 0, 0, 0 };

	if (CH[4] > 1500) {
		ARM = 0;
	}

	if (ARM) {
		digitalWrite(LED, 1);
	}
	else {
		digitalWrite(LED, 0);
	}

	if (!ARM) {
		if (!((CH[2] < 1200) and (CH[3] > 1900))) {
			ARM_TIMER = millis();
		}
		if (millis() - ARM_TIMER > 3000) {
			ARM = 1;
			set_gyro_angles = 0;
		}
	}
	else {
		if (!((CH[2] < 1200) and (CH[3] < 1200))) {
			ARM_TIMER = millis();
		}
		if (millis() - ARM_TIMER > 1000) {
			ARM = 0;
		}
	}

	if (ARM) {

		if (abs(CH[1] - 1500) > 30) {
			pos_value[0] = -(CH[1] - 1500);
		}
		else {
			pos_value[0] = 0;
		}

		if (abs(CH[0] - 1500) > 30) {
			pos_value[1] = -(CH[0] - 1500);
		}
		else {
			pos_value[1] = 0;
		}

		pos_value[2] = constrain(CH[2], MIN_SPEED, MAX_SPEED);

		if (abs(CH[3] - 1500) > 30) {
			pos_value[3] = CH[3] - 1500;
		}
		else {
			pos_value[3] = 0;
		}

		if (pos_value[2] > 1200) {
			ang_control(pos_value[0], pos_value[1], pos_value[3], 0);
			M1_VAL = constrain((pos_value[2] + PITCH_PID_OUTPUT - ROLL_PID_OUTPUT + YAW_PID_OUTPUT), START_SPEED, MAX_SPEED);
			M2_VAL = constrain((pos_value[2] - PITCH_PID_OUTPUT + ROLL_PID_OUTPUT + YAW_PID_OUTPUT), START_SPEED, MAX_SPEED);
			M3_VAL = constrain((pos_value[2] + PITCH_PID_OUTPUT + ROLL_PID_OUTPUT - YAW_PID_OUTPUT), START_SPEED, MAX_SPEED);
			M4_VAL = constrain((pos_value[2] - PITCH_PID_OUTPUT - ROLL_PID_OUTPUT - YAW_PID_OUTPUT), START_SPEED, MAX_SPEED);

		}
		else {
			ang_control(0, 0, 0, 1);
			M1_VAL = START_SPEED;
			M2_VAL = START_SPEED;
			M3_VAL = START_SPEED;
			M4_VAL = START_SPEED;
		}
	}
	else {
		M1_VAL = MIN_SPEED;
		M2_VAL = MIN_SPEED;
		M3_VAL = MIN_SPEED;
		M4_VAL = MIN_SPEED;
	}
}

void ESC_cali() {
	M1_VAL = CH[2];
	M2_VAL = CH[2];
	M3_VAL = CH[2];
	M4_VAL = CH[2];
}

void FAILSAFE() {

	//check channal read proprely
	if ((micros() - ch1_timer) > 25000) { SIG_fail_safe_counter += 1; }
	else if ((micros() - ch2_timer) > 25000) { SIG_fail_safe_counter += 1; }
	else if ((micros() - ch3_timer) > 25000) { SIG_fail_safe_counter += 1; }
	else if ((micros() - ch4_timer) > 25000) { SIG_fail_safe_counter += 1; }
	//check channal read wide proprely
	else if ((CH[0] < 900) or (CH[0] > 2200)) { SIG_fail_safe_counter += 1; }
	else if ((CH[1] < 900) or (CH[1] > 2200)) { SIG_fail_safe_counter += 1; }
	else if ((CH[2] < 900) or (CH[2] > 2200)) { SIG_fail_safe_counter += 1; }
	else if ((CH[3] < 900) or (CH[3] > 2200)) { SIG_fail_safe_counter += 1; }
	else {SIG_fail_safe_timer = millis();}

	if (millis() - SIG_fail_safe_timer > 5000) {
		SIG_fail_safe_counter = 0;
	}
	else if (SIG_fail_safe_counter > 100) {
		SIG_fail_safe = 1;
	}

	if (SIG_fail_safe) FAIL_CODE = 0x01;
	if (IMU_fail_safe) FAIL_CODE = 0x02;

	if (FAIL_CODE != 0) {
		MODE = 0;
	}
	Serial.println(FAIL_CODE, HEX);

}
