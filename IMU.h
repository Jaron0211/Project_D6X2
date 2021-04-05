
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


    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 14);

    if (Wire.available() >= 14) {
      acc_x = Wire.read() << 8 | Wire.read();
      acc_y = Wire.read() << 8 | Wire.read();
      acc_z = Wire.read() << 8 | Wire.read();
      temperature = Wire.read() << 8 | Wire.read();
      gyro_x = Wire.read() << 8 | Wire.read();
      gyro_y = Wire.read() << 8 | Wire.read();
      gyro_z = Wire.read() << 8 | Wire.read();
      IMU_read_frequence = 1000000/(micros() - IMU_request_timer);
      data.IMU_read_frequence = IMU_read_frequence;
      IMU_request_timer = micros();
  }
}

void angle_read() {

  read_mpu_6050_data();

  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  gyro_pitch = gyro_pitch * 0.8 + ((-cos(IMU_ROTATION_ANGLE) * gyro_x + sin(IMU_ROTATION_ANGLE) * gyro_y) / 65.5 )* 0.2;
  gyro_roll = gyro_roll * 0.8 + ((-sin(IMU_ROTATION_ANGLE) * gyro_x + cos(IMU_ROTATION_ANGLE) * gyro_y) / 65.5 )* 0.2;
  gyro_yaw = gyro_yaw * 0.8 + (gyro_z / 65.5 )* 0.2;

  if (gyro_yaw < -180) {
      gyro_yaw = 180;
  }
  else if (gyro_yaw > 180) {
      gyro_yaw = -180;
  }

  data.GYRO_PITCH = gyro_x;
  data.GYRO_ROLL = gyro_y;
  data.GYRO_YAW = gyro_z;

  angle_pitch += gyro_x * 1 / IMU_read_frequence /65.5;// 1/250hz/65.5lsb/s)
  angle_roll += gyro_y * 1 / IMU_read_frequence / 65.5;

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

  angle_roll -= angle_pitch * sin(radians( gyro_z * 0.0000611));
  angle_pitch += angle_roll * sin(radians(gyro_z * 0.0000611));

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

    angle_pitch = angle_pitch * 0.9995 + angle_pitch_acc * 0.0005;
    angle_roll = angle_roll * 0.9995 + angle_roll_acc * 0.0005;

    /*
    PITCH = angle_roll;// * 0.2 + PITCH * 0.8;
    ROLL = angle_pitch;// *0.2 + ROLL * 0.8;
    YAW = angle_yaw ;
    */

    PITCH = (-cos(IMU_ROTATION_ANGLE) * angle_pitch +  sin(IMU_ROTATION_ANGLE) * angle_roll);// *0.5 + PITCH * 0.5;
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
    PITCH = -cos(IMU_ROTATION_ANGLE) * angle_pitch + sin(IMU_ROTATION_ANGLE) * angle_roll;
    ROLL = -sin(IMU_ROTATION_ANGLE) * angle_pitch + cos(IMU_ROTATION_ANGLE) * angle_roll;
    YAW = 0;
    set_gyro_angles = true;
  }
}
