
void Serial_RX() {
  char INCOME_CHAR ;
  if (Serial.available()) {
    INCOME_CHAR = Serial.read();
    if (String(INCOME_CHAR) == "a") {
      GYR_PID_ADJUST = 1;
    } else if (String(INCOME_CHAR) == "#") {
      RX_END = 1;
    }
  }
  
  if (GYR_PID_ADJUST) {
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

void DEBUG_PRINT() {
    
    Serial.print(data.GYRO_PITCH);
    Serial.print(",");
    Serial.print(data.GYRO_ROLL);
    Serial.print(",");
    Serial.print(data.GYRO_YAW);
    Serial.print(",");
    Serial.print(data.ANGLE_PITCH);
    Serial.print(",");
    Serial.print(data.ANGLE_ROLL);
    Serial.print(",");
    Serial.print(data.PITCH);
    Serial.print(",");
    Serial.print(data.ROLL);
    Serial.print(",");
    Serial.print(data.YAW);
    Serial.print(",");
    Serial.print(data.PITCH_P_VAL);
    Serial.print(",");
    Serial.print(data.PITCH_I_VAL);
    Serial.print(",");
    Serial.print(data.PITCH_D_VAL);
    Serial.print(",");
    Serial.print(data.PITCH_PID_OUTPUT);
    Serial.println(",");
  
}
