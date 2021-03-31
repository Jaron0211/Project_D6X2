
void setup_mpu_6050_registers() {

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  /*
    0 ± 2g
    1 ± 4g
    2 ± 8g
    3 ± 16g
  */
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  /*
    0 ± 250 °/s
    1 ± 500 °/s
    2 ± 1000 °/s
    3 ± 2000 °/s
  */
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  //Digital Low Pass Filter to ~43Hz
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
}

void read_mpu_6050_data() {

  if (!IMU_request) {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 14);
    IMU_request = 1;
    IMU_request_timer = millis();
  } else {
    if (Wire.available() >= 14) {
      acc_x = Wire.read() << 8 | Wire.read();
      acc_y = Wire.read() << 8 | Wire.read();
      acc_z = Wire.read() << 8 | Wire.read();
      temperature = Wire.read() << 8 | Wire.read();
      gyro_x = Wire.read() << 8 | Wire.read();
      gyro_y = Wire.read() << 8 | Wire.read();
      gyro_z = Wire.read() << 8 | Wire.read();
      IMU_request = 0;
      IMU_read_frequence = 1 / (millis() - IMU_request_timer) * 1000;
    } else if (millis() - IMU_request_timer > 5000) {
      IMU_fail_safe = 1;
    }
  }
}

void angle_read() {

  read_mpu_6050_data();

  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  gyro_pitch = gyro_pitch * 0.7 + (-cos(IMU_ROTATION_ANGLE) * gyro_x + sin(IMU_ROTATION_ANGLE) * gyro_y) / 65.5 * 0.3;
  gyro_roll = gyro_roll * 0.7 + (-sin(IMU_ROTATION_ANGLE) * gyro_x + cos(IMU_ROTATION_ANGLE) * gyro_y) / 65.5 * 0.3;
  gyro_yaw = gyro_yaw * 0.7 + gyro_z / 65.5 * 0.3;

  data.GYRO_PITCH = gyro_pitch;
  data.GYRO_ROLL = gyro_roll;
  data.GYRO_YAW = gyro_yaw;

  angle_pitch = ( angle_pitch + gyro_x * 0.0000611 ) * 1 + angle_pitch * 0;//1/250hz/65.5lsb/s)
  angle_roll = ( angle_roll + gyro_y * 0.0000611 ) * 1 + angle_roll * 0;

  data.ANGLE_PITCH = angle_pitch;
  data.ANGLE_ROLL = angle_roll;

  /*
  gyro_pitch = gyro_pitch * 0.7 + gyro_y / 65.5 * 0.3;
  gyro_roll = gyro_roll * 0.7 - gyro_x / 65.5 * 0.3;
  gyro_yaw = gyro_yaw * 0.7 + gyro_z / 65.5 * 0.3;

  angle_pitch = ( angle_pitch + (cos(IMU_ROTATION_ANGLE) * gyro_x + sin(IMU_ROTATION_ANGLE) * gyro_y) * 0.0000611 ) * 0.8 + angle_pitch * 0.2;//1/250hz/65.5lsb/s)
  angle_roll = ( angle_roll + (cos(IMU_ROTATION_ANGLE) * gyro_x + sin(IMU_ROTATION_ANGLE) * gyro_y) * 0.0000611 ) * 0.8 + angle_roll * 0.2;
  */

  if ((gyro_z * 0.0000611 > 0.01) or (gyro_z * 0.0000611 < -0.01)) {
    angle_yaw += gyro_z * 0.0000611 ;
  }

  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);

  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));

  angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;
  angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;
  
  /*
  angle_pitch_acc = (cos(IMU_ROTATION_ANGLE) * asin((float)acc_y / acc_total_vector) - sin(IMU_ROTATION_ANGLE) * asin((float)acc_x / acc_total_vector) )* 57.296;
  angle_roll_acc = (sin(IMU_ROTATION_ANGLE) * asin((float)acc_y / acc_total_vector) - cos(IMU_ROTATION_ANGLE) * asin((float)acc_x / acc_total_vector) )* 57.296;
  */
  
  angle_pitch_acc -= pitch_base;
  angle_roll_acc -= roll_base;

  if (set_gyro_angles) {

    angle_pitch = angle_pitch * 0.999 + angle_pitch_acc * 0.001;
    angle_roll = angle_roll * 0.999 + angle_roll_acc * 0.001;

    /*
    PITCH = angle_roll;// * 0.2 + PITCH * 0.8;
    ROLL = angle_pitch;// *0.2 + ROLL * 0.8;
    YAW = angle_yaw ;
    */

    PITCH = (-cos(IMU_ROTATION_ANGLE) * angle_pitch + sin(IMU_ROTATION_ANGLE) * angle_roll);// *0.5 + PITCH * 0.5;
    ROLL = (-sin(IMU_ROTATION_ANGLE) * angle_pitch + cos(IMU_ROTATION_ANGLE) * angle_roll);// *0.5 + ROLL * 0.5;
    YAW = angle_yaw ;

    data.PITCH = PITCH;
    data.ROLL = ROLL;
    data.YAW = YAW;
        
    if (YAW <= 0) {
      YAW += 360;
    } else if (YAW >= 360) {
      YAW -= 360;
    }

  } else {
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    PITCH = -cos(IMU_ROTATION_ANGLE) * angle_pitch - sin(IMU_ROTATION_ANGLE) * angle_roll;
    ROLL = -sin(IMU_ROTATION_ANGLE) * angle_pitch - cos(IMU_ROTATION_ANGLE) * angle_roll;
    YAW = 0;
    set_gyro_angles = true;
  }
}
