/*
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

*/

void DEBUG_PRINT() {
  
    Serial.print(PITCH);
    Serial.print(",");
    Serial.print(ROLL);
    Serial.print(",");
    Serial.print(PITCH_PID_OUTPUT);
    Serial.println();
  
}
