
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
			// Reset PID state on arm
			PITCH_I_VAL = 0; ROLL_I_VAL = 0; YAW_I_VAL = 0;
			gyro_pitch_prev = gyro_pitch;
			gyro_roll_prev = gyro_roll;
			gyro_yaw_prev = gyro_yaw;
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

		// Compute dt for PID
		unsigned long now = micros();
		float pid_dt = (now - PID_LOOP_TIMER) / 1000000.0f;
		PID_LOOP_TIMER = now;
		if (pid_dt <= 0.0f || pid_dt > 0.5f) pid_dt = 0.005f;

		if (pos_value[2] > 1200) {
			ang_control(pos_value[0], pos_value[1], pos_value[3], 0, pid_dt);

			// Compute raw motor values
			int m1_raw = pos_value[2] + PITCH_PID_OUTPUT - ROLL_PID_OUTPUT + YAW_PID_OUTPUT;
			int m2_raw = pos_value[2] - PITCH_PID_OUTPUT + ROLL_PID_OUTPUT + YAW_PID_OUTPUT;
			int m3_raw = pos_value[2] + PITCH_PID_OUTPUT + ROLL_PID_OUTPUT - YAW_PID_OUTPUT;
			int m4_raw = pos_value[2] - PITCH_PID_OUTPUT - ROLL_PID_OUTPUT - YAW_PID_OUTPUT;

			// Proportional scaling: preserve attitude authority over throttle
			int max_motor = max(max(m1_raw, m2_raw), max(m3_raw, m4_raw));
			int min_motor = min(min(m1_raw, m2_raw), min(m3_raw, m4_raw));

			if (max_motor > MAX_SPEED) {
				int excess = max_motor - MAX_SPEED;
				m1_raw -= excess;
				m2_raw -= excess;
				m3_raw -= excess;
				m4_raw -= excess;
			}
			if (min_motor < START_SPEED) {
				int deficit = START_SPEED - min_motor;
				m1_raw += deficit;
				m2_raw += deficit;
				m3_raw += deficit;
				m4_raw += deficit;
			}

			M1_VAL = constrain(m1_raw, START_SPEED, MAX_SPEED);
			M2_VAL = constrain(m2_raw, START_SPEED, MAX_SPEED);
			M3_VAL = constrain(m3_raw, START_SPEED, MAX_SPEED);
			M4_VAL = constrain(m4_raw, START_SPEED, MAX_SPEED);
		}
		else {
			ang_control(0, 0, 0, 1, pid_dt);
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

	//check channel read properly
	if ((micros() - ch1_timer) > 25000) { SIG_fail_safe_counter += 1; }
	else if ((micros() - ch2_timer) > 25000) { SIG_fail_safe_counter += 1; }
	else if ((micros() - ch3_timer) > 25000) { SIG_fail_safe_counter += 1; }
	else if ((micros() - ch4_timer) > 25000) { SIG_fail_safe_counter += 1; }
	//check channel read range properly
	else if ((CH[0] < 900) or (CH[0] > 2200)) {	SIG_fail_safe_counter += 1;	}
	else if ((CH[1] < 900) or (CH[1] > 2200)) {	SIG_fail_safe_counter += 1; }
	else if ((CH[2] < 900) or (CH[2] > 2200)) {	SIG_fail_safe_counter += 1;	}
	else if ((CH[3] < 900) or (CH[3] > 2200)) {	SIG_fail_safe_counter += 1;	}
	// Check CH5 range (was previously in ISR)
	else if ((CH[4] < 900) or (CH[4] > 2200)) { SIG_fail_safe_counter += 1; }
	else {SIG_fail_safe_timer = millis();}

	if (millis() - SIG_fail_safe_timer < 2000) {SIG_fail_safe_counter = 0;}

	if (SIG_fail_safe_counter > 30) { SIG_fail_safe = 1;}
	else { SIG_fail_safe = 0; }

	if (SIG_fail_safe) { FAIL_CODE = 0x07; }
	else if (IMU_fail_safe) { FAIL_CODE = 0x08; }
	else if (battery_low) { FAIL_CODE = 0x09; }
	else { FAIL_CODE = 0x00; }

	// Failsafe action: disarm and cut motors
	if (FAIL_CODE != 0) {
		ARM = 0;
		M1_VAL = MIN_SPEED;
		M2_VAL = MIN_SPEED;
		M3_VAL = MIN_SPEED;
		M4_VAL = MIN_SPEED;
		M5_VAL = MIN_SPEED;
		M6_VAL = MIN_SPEED;
	}
}
