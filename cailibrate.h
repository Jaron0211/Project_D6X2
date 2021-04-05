bool acc_cal_start;

int ACC_step = 0;

long Acc_front, Acc_back, Acc_left, Acc_right, Acc_top, Acc_down;
long Acc_X_bias, Acc_Y_bias, Acc_Z_bias;

void Gyro_cal() {
  bool LED_STATE = 0;
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {
    read_mpu_6050_data();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;
    acc_total_vector_level += sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));

    if ((abs(gyro_x) > 400) or (abs(gyro_y) > 400 )or (abs(gyro_z) > 400)) {
        gyro_x_cal = 0;
        gyro_y_cal = 0;
        gyro_z_cal = 0;
        cal_int = 0;
    }
    delay(3);
    if (millis() - LED_TIMER > 200) {
        digitalWrite(LED, LED_STATE);
        LED_STATE = !LED_STATE;
        LED_TIMER = millis();
    }
  }
  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;
  acc_total_vector_level /= 2000;

}

void Acc_cal() {

  if (Serial.available()) {
    acc_cal_start = 1;
  }

  if (acc_cal_start) {
    switch (ACC_step)
    {
      //ACC_X cailibration
      case 0:
        Acc_front = 0;
        Acc_back = 0;
        Acc_left = 0;
        Acc_right = 0;
        Acc_top = 0;
        Acc_down = 0;
        ACC_step = 1;
        acc_cal_start = 0;
        break;
      case 1:
        for (int cal_int = 0; cal_int < 333 ; cal_int ++) {
          read_mpu_6050_data();
          Acc_front += acc_x;
          delay(3);
        }
        Acc_front = Acc_front / 333;
        ACC_step = 2;
        acc_cal_start = 0;
        break;

      case 2:
        for (int cal_int = 0; cal_int < 333 ; cal_int ++) {
          read_mpu_6050_data();
          Acc_back += acc_x;
          delay(3);
        }
        Acc_back = Acc_back / 333;
        ACC_step = 3;
        acc_cal_start = 0;
        break;
      //ACC_Y cailibration
      case 3:
        for (int cal_int = 0; cal_int < 333 ; cal_int ++) {
          read_mpu_6050_data();
          Acc_left += acc_y;
          delay(3);
        }
        Acc_left = Acc_left / 333;
        ACC_step = 4;
        acc_cal_start = 0;
        break;

      case 4:
        for (int cal_int = 0; cal_int < 333 ; cal_int ++) {
          read_mpu_6050_data();
          Acc_right += acc_y;
          delay(3);
        }
        Acc_right = Acc_right / 333;
        ACC_step = 5;
        acc_cal_start = 0;
        break;
      //ACC_Z cailibration
      case 5:
        for (int cal_int = 0; cal_int < 333 ; cal_int ++) {
          read_mpu_6050_data();
          Acc_top += acc_z;
          delay(3);
        }
        Acc_top = Acc_top / 333;
        ACC_step = 6;
        acc_cal_start = 0;
        break;

      case 6:
        for (int cal_int = 0; cal_int < 333 ; cal_int ++) {
          read_mpu_6050_data();
          Acc_down += acc_y;
          delay(3);
        }
        Acc_down = Acc_down / 333;
        ACC_step = 7;
        acc_cal_start = 0;
        break;
      //caiculate total bias
      case 7:
        Acc_X_bias = (Acc_front + Acc_back) / 2;
        Acc_Y_bias = (Acc_left + Acc_right) / 2;
        Acc_Z_bias = (Acc_top + Acc_down) / 2;
        break;
      case 27:
        break;
    }
  }


}
