void ang_control(float DES_PIT, float DES_ROL, float DES_YAW, bool RESET, float dt) {

    // Outer loop: angle error -> rate setpoint
    float PITCH_ANGLE_error = DES_PIT - PITCH * 15;
    float ROLL_ANGLE_error = DES_ROL - ROLL * 15;

    // Inner loop: rate PID
    float PITCH_error = gyro_pitch - PITCH_ANGLE_error;
    float ROLL_error = gyro_roll - ROLL_ANGLE_error;
    float YAW_error = gyro_yaw - (DES_YAW / 10);

    //=== PITCH PID ===//
    PITCH_P_VAL = PITCH_error;

    PITCH_I_VAL += PITCH_error * POS_I * dt;
    if (RESET) {
        PITCH_I_VAL = 0;
    }
    PITCH_I_VAL = constrain(PITCH_I_VAL, -POS_I_MAX, POS_I_MAX);

    // D-term on measurement (gyro rate), not on error
    PITCH_D_VAL = -(gyro_pitch - gyro_pitch_prev) / dt;
    gyro_pitch_prev = gyro_pitch;
    PITCH_D_VAL = constrain(PITCH_D_VAL, -POS_D_MAX, POS_D_MAX);

    data.PITCH_P_VAL = PITCH_P_VAL;
    data.PITCH_I_VAL = PITCH_I_VAL;
    data.PITCH_D_VAL = PITCH_D_VAL;

    PITCH_PID_OUTPUT = POS_P * PITCH_P_VAL + PITCH_I_VAL + POS_D * PITCH_D_VAL;
    PITCH_PID_OUTPUT = constrain(PITCH_PID_OUTPUT, -GYRO_PID_MAX, GYRO_PID_MAX);

    data.PITCH_PID_OUTPUT = PITCH_PID_OUTPUT;

    //=== ROLL PID ===//
    ROLL_P_VAL = ROLL_error;

    ROLL_I_VAL += ROLL_error * POS_I * dt;
    if (RESET) {
        ROLL_I_VAL = 0;
    }
    ROLL_I_VAL = constrain(ROLL_I_VAL, -POS_I_MAX, POS_I_MAX);

    // D-term on measurement
    ROLL_D_VAL = -(gyro_roll - gyro_roll_prev) / dt;
    gyro_roll_prev = gyro_roll;
    ROLL_D_VAL = constrain(ROLL_D_VAL, -POS_D_MAX, POS_D_MAX);

    ROLL_PID_OUTPUT = POS_P * ROLL_P_VAL + ROLL_I_VAL + POS_D * ROLL_D_VAL;
    ROLL_PID_OUTPUT = constrain(ROLL_PID_OUTPUT, -GYRO_PID_MAX, GYRO_PID_MAX);

    //=== YAW PID ===//
    YAW_P_VAL = YAW_error;

    YAW_I_VAL += YAW_I * YAW_error * dt;
    if (RESET) {
        YAW_I_VAL = 0;
    }
    YAW_I_VAL = constrain(YAW_I_VAL, -YAW_I_MAX, YAW_I_MAX);

    // D-term on measurement
    YAW_D_VAL = -(gyro_yaw - gyro_yaw_prev) / dt;
    gyro_yaw_prev = gyro_yaw;
    YAW_D_VAL = constrain(YAW_D_VAL, -YAW_D_MAX, YAW_D_MAX);

    YAW_PID_OUTPUT = YAW_P * YAW_P_VAL + YAW_I_VAL + YAW_D * YAW_D_VAL;
}
