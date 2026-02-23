
void setup_mpu_6050_registers() {

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  /*
    0 +/- 2g
    1 +/- 4g
    2 +/- 8g
    3 +/- 16g
  */
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  /*
    0 +/- 250 deg/s
    1 +/- 500 deg/s
    2 +/- 1000 deg/s
    3 +/- 2000 deg/s
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
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        IMU_fail_safe = 1;
        return;
    }

    Wire.requestFrom(0x68, 14);
    unsigned long i2c_timeout = micros();
    while (Wire.available() < 14) {
        if (micros() - i2c_timeout > 1000) { // 1ms timeout
            IMU_fail_safe = 1;
            return;
        }
    }

    acc_x = Wire.read() << 8 | Wire.read();
    acc_y = Wire.read() << 8 | Wire.read();
    acc_z = Wire.read() << 8 | Wire.read();
    temperature = Wire.read() << 8 | Wire.read();
    gyro_x = Wire.read() << 8 | Wire.read();
    gyro_y = Wire.read() << 8 | Wire.read();
    gyro_z = Wire.read() << 8 | Wire.read();
    IMU_request_timer = micros();
    IMU_fail_safe = 0;
}

// Mahony AHRS update (IMU only, no magnetometer)
// gx, gy, gz: gyro in rad/s (body frame, rotation-corrected)
// ax, ay, az: accel in raw units (will be normalized)
void mahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt) {

  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Normalize accelerometer (ignore if zero to avoid NaN)
  float accNorm = ax * ax + ay * ay + az * az;
  if (accNorm > 0.0f) {
    recipNorm = 1.0f / sqrt(accNorm);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated gravity direction from quaternion
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = 0.5f - q1 * q1 - q2 * q2;

    // Error is cross product between estimated and measured gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Apply integral feedback (gyro bias estimation)
    if (mahonyKi > 0.0f) {
      integralFBx += mahonyKi * halfex * dt;
      integralFBy += mahonyKi * halfey * dt;
      integralFBz += mahonyKi * halfez * dt;
      gx += integralFBx;
      gy += integralFBy;
      gz += integralFBz;
    }

    // Apply proportional feedback
    gx += mahonyKp * halfex;
    gy += mahonyKp * halfey;
    gz += mahonyKp * halfez;
  }

  // Integrate quaternion rate
  gx *= 0.5f * dt;
  gy *= 0.5f * dt;
  gz *= 0.5f * dt;
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalize quaternion
  recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void angle_read() {

  read_mpu_6050_data();

  // Compute dt in seconds
  unsigned long now = micros();
  float dt = (now - ahrs_timer) / 1000000.0f;
  ahrs_timer = now;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.005f; // guard against bad timing

  // Subtract gyro calibration offsets
  float gx_raw = (float)(gyro_x - gyro_x_cal);
  float gy_raw = (float)(gyro_y - gyro_y_cal);
  float gz_raw = (float)(gyro_z - gyro_z_cal);

  // Convert to deg/s (65.5 LSB per deg/s at +/-500 deg/s)
  float gx_dps = gx_raw / 65.5f;
  float gy_dps = gy_raw / 65.5f;
  float gz_dps = gz_raw / 65.5f;

  // Rotate gyro to body frame using pre-computed sin/cos
  float gx_body = -imu_cos * gx_dps + imu_sin * gy_dps;
  float gy_body = -imu_sin * gx_dps - imu_cos * gy_dps;
  float gz_body = gz_dps;

  // Filtered gyro rates for PID (lighter filter: 0.7/0.3)
  gyro_pitch = gyro_pitch * 0.7f + gx_body * 0.3f;
  gyro_roll  = gyro_roll  * 0.7f + gy_body * 0.3f;
  gyro_yaw   = gyro_yaw   * 0.7f + gz_body * 0.3f;

  data.GYRO_PITCH = gyro_pitch;
  data.GYRO_ROLL = gyro_roll;
  data.GYRO_YAW = gyro_yaw;

  // Rotate accel to body frame
  float ax_body = -imu_cos * (float)acc_x + imu_sin * (float)acc_y;
  float ay_body = -imu_sin * (float)acc_x - imu_cos * (float)acc_y;
  float az_body = (float)acc_z;

  // Convert gyro to rad/s for Mahony
  float gx_rad = gx_body * (PI / 180.0f);
  float gy_rad = gy_body * (PI / 180.0f);
  float gz_rad = gz_body * (PI / 180.0f);

  // Run Mahony AHRS update
  mahonyUpdate(gx_rad, gy_rad, gz_rad, ax_body, ay_body, az_body, dt);

  // Extract Euler angles from quaternion (in degrees)
  // Pitch = rotation about X axis
  // Roll  = rotation about Y axis
  // YAW   = rotation about Z axis
  PITCH = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * (180.0f / PI);
  ROLL  = asin(constrain(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f)) * (180.0f / PI);
  YAW   = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * (180.0f / PI);

  // Adjust RPY calibration baseline
  PITCH -= pitch_base;
  ROLL  -= roll_base;
  YAW   -= yaw_base;

  if (YAW < 0) {
    YAW += 360.0f;
  } else if (YAW >= 360.0f) {
    YAW -= 360.0f;
  }

  data.ANGLE_PITCH = PITCH;
  data.ANGLE_ROLL = ROLL;
  data.PITCH = PITCH;
  data.ROLL = ROLL;
  data.YAW = YAW;
}
