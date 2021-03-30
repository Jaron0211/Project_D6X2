void Serial_RX() {
  char INCOME_CHAR ;
  if (Serial.available()) {
    INCOME_CHAR = Serial.read();
    if (String(INCOME_CHAR) == "a") {
      ANG_PID_ADJUST = 1;
      GYR_PID_ADJUST = 0;
    } else if (String(INCOME_CHAR) == "g") {
      ANG_PID_ADJUST = 0;
      GYR_PID_ADJUST = 1;
    } else if (String(INCOME_CHAR) == "#") {
      RX_END = 1;
    }
  }

  if (ANG_PID_ADJUST) {
    if (!RX_END) {
      DATA += INCOME_CHAR;
    } else {
      int MARK1 = 1;
      int MARK2 = 0;
      int i = 0;
      while (1) {
        MARK2 = DATA.indexOf(",", MARK1);
        if (MARK2 == -1)break;
        String ia = DATA.substring(MARK1, MARK2);
        PID_VAL[i] = ia.toFloat();
        Serial.println(PID_VAL[i]);
        MARK1 = MARK2 + 1;
        i++;
        if (i == 3)break;
      }
      ANG_P = PID_VAL[0];
      ANG_I = PID_VAL[1];
      ANG_D = PID_VAL[2];
      DATA = "";
      ANG_PID_ADJUST = 0;
      RX_END = 0;
    }
  } else if (GYR_PID_ADJUST) {
    if (!RX_END) {
      DATA += INCOME_CHAR;
    } else {
      int MARK1 = 1;
      int MARK2 = 0;
      int i = 0;
      while (1) {
        MARK2 = DATA.indexOf(",", MARK1);
        if (MARK2 == -1)break;
        String ia = DATA.substring(MARK1, MARK2);
        PID_VAL[i] = ia.toFloat();
        Serial.println(PID_VAL[i]);
        MARK1 = MARK2 + 1;
        i++;
        if (i == 3)break;
      }
      POS_P = PID_VAL[0];
      POS_I = PID_VAL[1];
      POS_D = PID_VAL[2];
      DATA = "";
      GYR_PID_ADJUST = 0;
      RX_END = 0;
    }
  }

}


void debug_data_collect() {
  DEBUGGING_DATA[0] = gyro_x;
  DEBUGGING_DATA[1] = gyro_y;
  DEBUGGING_DATA[2] = gyro_z;

  DEBUGGING_DATA[3] = acc_x;
  DEBUGGING_DATA[4] = acc_y;
  DEBUGGING_DATA[5] = acc_z;

  DEBUGGING_DATA[6] = PITCH;
  DEBUGGING_DATA[7] = ROLL;
  DEBUGGING_DATA[8] = YAW;

  DEBUGGING_DATA[9]  = M1_VAL;
  DEBUGGING_DATA[10] = M2_VAL;
  DEBUGGING_DATA[11] = M3_VAL;
  DEBUGGING_DATA[12] = M4_VAL;

  DEBUGGING_DATA[13] = CH[0];
  DEBUGGING_DATA[14] = CH[1];
  DEBUGGING_DATA[15] = CH[2];
  DEBUGGING_DATA[16] = CH[3];
  DEBUGGING_DATA[17] = CH[4];
  DEBUGGING_DATA[18] = CH[5];

  DEBUGGING_DATA[19] = ARM;

  DEBUGGING_DATA[20] = ANG_PITCH_PID_OUTPUT;
  DEBUGGING_DATA[21] = ANG_P * ANG_PITCH_P_VAL;
  DEBUGGING_DATA[22] = ANG_PITCH_I_VAL;
  DEBUGGING_DATA[23] = ANG_D * ANG_PITCH_D_VAL;

  DEBUGGING_DATA[24] = ANG_ROLL_PID_OUTPUT;
  DEBUGGING_DATA[25] = ANG_P * ANG_ROLL_P_VAL;
  DEBUGGING_DATA[26] = ANG_ROLL_I_VAL;
  DEBUGGING_DATA[27] = ANG_D * ANG_ROLL_D_VAL;

  DEBUGGING_DATA[28] = PITCH_PID_OUTPUT;
  DEBUGGING_DATA[29] = POS_P * PITCH_P_VAL;
  DEBUGGING_DATA[30] = PITCH_I_VAL;
  DEBUGGING_DATA[31] = POS_D * PITCH_D_VAL;

  DEBUGGING_DATA[32] = ROLL_PID_OUTPUT;
  DEBUGGING_DATA[33] = POS_P * ROLL_P_VAL;
  DEBUGGING_DATA[34] = ROLL_I_VAL;
  DEBUGGING_DATA[35] = POS_D * ROLL_D_VAL;

  DEBUGGING_DATA[36] = MODE;
}


void DEBUG_PRINT() {
  
    Serial.print(PITCH);
    Serial.print(",");
    Serial.print(ROLL);
    Serial.print(",");
    Serial.print(PITCH_PID_OUTPUT);
    Serial.println();
  
}
