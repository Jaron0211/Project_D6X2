void ang_control(float DES_PIT = 0, float DES_ROL = 0, float DES_YAW = 0, bool RESET = 0) {

    float PITCH_ANGLE_error = DES_PIT - PITCH * 15  ;
    float ROLL_ANGLE_error = DES_ROL  - ROLL * 15 ;

    //=============================================================================//

    float PITCH_error = gyro_pitch - PITCH_ANGLE_error;
    float ROLL_error = gyro_roll - ROLL_ANGLE_error;
    float YAW_error = gyro_yaw - (DES_YAW /10);

    //PITCH
    PITCH_P_VAL = PITCH_error;

    PITCH_I_VAL += PITCH_error * POS_I/100;
    if (RESET) {
        PITCH_I_VAL = 0;
    }
    PITCH_I_VAL = constrain(PITCH_I_VAL, -POS_I_MAX, POS_I_MAX);

    data.PITCH_I_VAL = PITCH_I_VAL;

    PITCH_D_VAL = PITCH_error - PITCH_error_pre;
    PITCH_D_VAL = constrain(PITCH_D_VAL, -POS_D_MAX, POS_D_MAX);
    PITCH_error_pre = PITCH_error;
    
    data.PITCH_D_VAL = PITCH_D_VAL; 
    
    PITCH_PID_OUTPUT = POS_P * PITCH_P_VAL + PITCH_I_VAL + POS_D * PITCH_D_VAL;
    PITCH_PID_OUTPUT = constrain(PITCH_PID_OUTPUT, -GYRO_PID_MAX, GYRO_PID_MAX);

    data.PITCH_PID_OUTPUT = PITCH_PID_OUTPUT;

    //ROLL
    ROLL_P_VAL = ROLL_error;

    data.PITCH_P_VAL = ROLL_P_VAL;

    ROLL_I_VAL += ROLL_error * POS_I/100;
    if (RESET) {
        ROLL_I_VAL = 0;
    }
    ROLL_I_VAL = constrain(ROLL_I_VAL, -POS_I_MAX, POS_I_MAX);

    data.PITCH_I_VAL = ROLL_I_VAL;

    ROLL_D_VAL = ROLL_error - ROLL_error_pre;
    ROLL_D_VAL = constrain(ROLL_D_VAL, -POS_D_MAX, POS_D_MAX);
    ROLL_error_pre = ROLL_error;

    data.PITCH_D_VAL = ROLL_D_VAL;

    ROLL_PID_OUTPUT = POS_P * ROLL_P_VAL + ROLL_I_VAL + POS_D * ROLL_D_VAL;
    ROLL_PID_OUTPUT = constrain(ROLL_PID_OUTPUT, -GYRO_PID_MAX, GYRO_PID_MAX);

    data.PITCH_PID_OUTPUT = ROLL_PID_OUTPUT;

    //YAW
    YAW_P_VAL = YAW_error;
    YAW_I_VAL += YAW_I * YAW_error/100;
    if (RESET) {
        YAW_I_VAL = 0;
    }
    YAW_I_VAL = constrain(YAW_I_VAL, -YAW_I_MAX, YAW_I_MAX);
    YAW_D_VAL = YAW_error - YAW_error_pre;
    YAW_D_VAL = constrain(YAW_D_VAL, -YAW_D_MAX, YAW_D_MAX);
    YAW_error_pre = YAW_error;
    YAW_PID_OUTPUT = YAW_P * YAW_P_VAL + YAW_I_VAL + YAW_D * YAW_D_VAL;

}

