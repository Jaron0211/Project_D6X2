void ang_control(float DES_PIT = 0, float DES_ROL = 0, float DES_YAW = 0, bool RESET = 0) {


    float PITCH_ANGLE_error = DES_PIT - PITCH;
    float ROLL_ANGLE_error = DES_ROL - ROLL;

    //=============================================================================//

    float PITCH_error = (-cos(IMU_ROTATION_ANGLE) * gyro_x + sin(IMU_ROTATION_ANGLE) * gyro_y) - (PITCH_ANGLE_error);
    float ROLL_error = (-sin(IMU_ROTATION_ANGLE) * gyro_x + cos(IMU_ROTATION_ANGLE) * gyro_y) - (ROLL_ANGLE_error);
    float YAW_error = gyro_yaw - DES_YAW;

    //PITCH
    PITCH_P_VAL = PITCH_error;
    PITCH_I_VAL += PITCH_error * POS_I;
    if (RESET) {
        PITCH_I_VAL = 0;
    }
    PITCH_I_VAL = constrain(PITCH_I_VAL, -POS_I_MAX, POS_I_MAX);

    PITCH_D_VAL = PITCH_error - PITCH_error_pre;
    PITCH_D_VAL = constrain(PITCH_D_VAL, -POS_D_MAX, POS_D_MAX);
    PITCH_error_pre = PITCH_error;
    PITCH_PID_OUTPUT = POS_P * PITCH_P_VAL + PITCH_I_VAL + POS_D * PITCH_D_VAL;
    PITCH_PID_OUTPUT = constrain(PITCH_PID_OUTPUT, -GYRO_PID_MAX, GYRO_PID_MAX);


    //ROLL
    ROLL_P_VAL = ROLL_error;
    ROLL_I_VAL += ROLL_error * POS_I;
    if (RESET) {
        ROLL_I_VAL = 0;
    }
    ROLL_I_VAL = constrain(ROLL_I_VAL, -POS_I_MAX, POS_I_MAX);
    ROLL_D_VAL = ROLL_error - ROLL_error_pre;
    ROLL_D_VAL = constrain(ROLL_D_VAL, -POS_D_MAX, POS_D_MAX);
    ROLL_error_pre = ROLL_error;
    ROLL_PID_OUTPUT = POS_P * ROLL_P_VAL + ROLL_I_VAL + POS_D * ROLL_D_VAL;
    ROLL_PID_OUTPUT = constrain(ROLL_PID_OUTPUT, -GYRO_PID_MAX, GYRO_PID_MAX);

    //YAW
    YAW_P_VAL = YAW_error;
    YAW_I_VAL += YAW_I * YAW_error;
    if (RESET) {
        YAW_I_VAL = 0;
    }
    YAW_I_VAL = constrain(YAW_I_VAL, -YAW_I_MAX, YAW_I_MAX);
    YAW_D_VAL = YAW_error - YAW_error_pre;
    YAW_D_VAL = constrain(YAW_D_VAL, -YAW_D_MAX, YAW_D_MAX);
    YAW_error_pre = YAW_error;
    YAW_PID_OUTPUT = YAW_P * YAW_P_VAL + YAW_I_VAL + YAW_D * YAW_D_VAL;

}


void ang_control_old(float DES_PIT = 0 , float DES_ROL = 0, float DES_YAW = 0,bool RESET = 0) {


  float PITCH_ANGLE_error = DES_PIT - PITCH;
  float ROLL_ANGLE_error = DES_ROL - ROLL;

  //PITCH
  ANG_PITCH_P_VAL = PITCH_ANGLE_error;
 
  
  ANG_PITCH_I_VAL += PITCH_ANGLE_error*(ANG_I/100);
  if(RESET){
    ANG_PITCH_I_VAL = 0;
  }
  ANG_PITCH_I_VAL = constrain(ANG_PITCH_I_VAL,-ANG_I_MAX,ANG_I_MAX);
  
  ANG_PITCH_D_VAL =  PITCH_ANGLE_error - ANG_PITCH_error_pre;
  ANG_PITCH_D_VAL = constrain(ANG_PITCH_D_VAL,-ANG_D_MAX,ANG_D_MAX);
  ANG_PITCH_error_pre = PITCH_ANGLE_error;
  
  ANG_PITCH_PID_OUTPUT = ANG_P * ANG_PITCH_P_VAL + ANG_PITCH_I_VAL + ANG_D * ANG_PITCH_D_VAL;
  ANG_PITCH_PID_OUTPUT = constrain(ANG_PITCH_PID_OUTPUT,-ANG_PID_MAX,ANG_PID_MAX);
  
  //ROLL
  ANG_ROLL_P_VAL = ROLL_ANGLE_error;
  
  ANG_ROLL_I_VAL += ROLL_ANGLE_error*ANG_I/100;
  if(RESET){
    ANG_ROLL_I_VAL = 0;
  }
  ANG_ROLL_I_VAL = constrain(ANG_ROLL_I_VAL,-ANG_I_MAX,ANG_I_MAX);
  
  ANG_ROLL_D_VAL =  ROLL_ANGLE_error - ANG_ROLL_error_pre;
  ANG_ROLL_D_VAL = constrain(ANG_ROLL_D_VAL,-ANG_D_MAX,ANG_D_MAX);
  ANG_ROLL_error_pre = ROLL_ANGLE_error;
  
  ANG_ROLL_PID_OUTPUT = ANG_P * ANG_ROLL_P_VAL + ANG_ROLL_I_VAL + ANG_D * ANG_ROLL_D_VAL;
  ANG_ROLL_PID_OUTPUT = constrain(ANG_ROLL_PID_OUTPUT,-ANG_PID_MAX,ANG_PID_MAX);
  
  //=============================================================================//

  int PITCH_error = (-cos(IMU_ROTATION_ANGLE) * gyro_x + sin(IMU_ROTATION_ANGLE) * gyro_y) - ( ANG_PITCH_PID_OUTPUT );
  int ROLL_error = (-sin(IMU_ROTATION_ANGLE) * gyro_x + cos(IMU_ROTATION_ANGLE) * gyro_y) - ( ANG_ROLL_PID_OUTPUT );
  int YAW_error = gyro_yaw - DES_YAW;

  //PITCH
  PITCH_P_VAL = PITCH_error;
  PITCH_I_VAL += PITCH_error*POS_I;
  if(RESET){
    PITCH_I_VAL = 0;
  }
  PITCH_I_VAL = constrain(PITCH_I_VAL, -POS_I_MAX, POS_I_MAX);

  PITCH_D_VAL = PITCH_error - PITCH_error_pre;
  PITCH_D_VAL = constrain(PITCH_D_VAL,-POS_D_MAX,POS_D_MAX);
  PITCH_error_pre = PITCH_error;
  PITCH_PID_OUTPUT = POS_P * PITCH_P_VAL +  PITCH_I_VAL + POS_D * PITCH_D_VAL;
  PITCH_PID_OUTPUT = constrain(PITCH_PID_OUTPUT, -GYRO_PID_MAX, GYRO_PID_MAX);

 
  //ROLL
  ROLL_P_VAL = ROLL_error;
  ROLL_I_VAL += ROLL_error*POS_I;
  if(RESET){
    ROLL_I_VAL = 0;
  }
  ROLL_I_VAL = constrain(ROLL_I_VAL, -POS_I_MAX, POS_I_MAX);
  ROLL_D_VAL = ROLL_error - ROLL_error_pre;
  ROLL_D_VAL = constrain(ROLL_D_VAL,-POS_D_MAX,POS_D_MAX);
  ROLL_error_pre = ROLL_error;
  ROLL_PID_OUTPUT = POS_P * ROLL_P_VAL +  ROLL_I_VAL + POS_D * ROLL_D_VAL;
  ROLL_PID_OUTPUT = constrain(ROLL_PID_OUTPUT, -GYRO_PID_MAX, GYRO_PID_MAX);
  
  //YAW
  YAW_P_VAL = YAW_error;
  YAW_I_VAL += YAW_I * YAW_error;
  if(RESET){
    YAW_I_VAL = 0;
  }
  YAW_I_VAL = constrain(YAW_I_VAL, -YAW_I_MAX, YAW_I_MAX);
  YAW_D_VAL = YAW_error - YAW_error_pre;
  YAW_D_VAL = constrain(YAW_D_VAL, -YAW_D_MAX, YAW_D_MAX);
  YAW_error_pre = YAW_error;
  YAW_PID_OUTPUT = YAW_P * YAW_P_VAL +  YAW_I_VAL + YAW_D * YAW_D_VAL;

}
