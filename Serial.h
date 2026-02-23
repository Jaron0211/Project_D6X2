
void Serial_RX() {
  if (!Serial.available()) return;

  char c = Serial.read();

  // Start marker
  if (c == 'a') {
    GYR_PID_ADJUST = 1;
    DATA_idx = 0;
    return;
  }

  // End marker
  if (c == '#') {
    RX_END = 1;
  }

  if (GYR_PID_ADJUST) {
    if (!RX_END) {
      if (DATA_idx < sizeof(DATA) - 1) {
        DATA[DATA_idx++] = c;
      }
    } else {
      DATA[DATA_idx] = '\0'; // null-terminate

      Serial.println(DATA);

      // Parse comma-separated values using strtok
      float vals[3] = {0, 0, 0};
      uint8_t i = 0;
      char *token = strtok(DATA, ",");
      while (token != NULL && i < 3) {
        vals[i++] = atof(token);
        token = strtok(NULL, ",");
      }

      POS_P = vals[0];
      POS_I = vals[1];
      POS_D = vals[2];
      have_written = 0;
      DATA_idx = 0;
      GYR_PID_ADJUST = 0;
      RX_END = 0;
    }
  }
}


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
    Serial.println();

}

void DEBUG_PRINT_CHANNAL() {
    Serial.print(CH[0]);
    Serial.print(",");
    Serial.print(CH[1]);
    Serial.print(",");
    Serial.print(CH[2]);
    Serial.print(",");
    Serial.print(CH[3]);
    Serial.print(",");
    Serial.print(CH[4]);
    Serial.print(",");
    Serial.print(CH[5]);
    Serial.print("\r\n");
}
