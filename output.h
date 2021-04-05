
void STABLE() {
    int pos_value[4] = { 0, 0, 0, 0 };

    if (CH[4] > 1500) {
        ARM = 0;
    }

    if (ARM) {
        digitalWrite(LED, 1);
    }
    else {
        digitalWrite(LED, 0);
    }
        
      if(!ARM){
        if(!((CH[2]<1200)and(CH[3]>1900))){
          ARM_TIMER = millis();
        }
        if(millis() - ARM_TIMER > 3000){
          ARM = 1;
          set_gyro_angles = 0;
        }
      }else{
        if(!((CH[2]<1200)and(CH[3]<1200))){
          ARM_TIMER = millis();
        }
        if(millis() - ARM_TIMER > 1000){
          ARM = 0;
        }
      }
      
      if (ARM) {

          if (abs(CH[1] - 1500) > 30) {
              pos_value[0] = -(CH[1] - 1500);
          }
          else {
              pos_value[0] = 0;
          }

          if (abs(CH[0] - 1500) > 30) {
              pos_value[1] = -(CH[0] - 1500);
          }
          else {
              pos_value[1] = 0;
          }

          pos_value[2] = constrain(CH[2], MIN_SPEED, MAX_SPEED);

          if (abs(CH[3] - 1500) > 30) {
              pos_value[3] = CH[3] - 1500;
          }
          else {
              pos_value[3] = 0;
          }

        if (pos_value[2] > 1200) {
          ang_control(pos_value[0], pos_value[1], pos_value[3], 0);
          M1_VAL = constrain((pos_value[2] + PITCH_PID_OUTPUT - ROLL_PID_OUTPUT + YAW_PID_OUTPUT), START_SPEED, MAX_SPEED);
          M2_VAL = constrain((pos_value[2] - PITCH_PID_OUTPUT + ROLL_PID_OUTPUT + YAW_PID_OUTPUT), START_SPEED, MAX_SPEED);
          M3_VAL = constrain((pos_value[2] + PITCH_PID_OUTPUT + ROLL_PID_OUTPUT - YAW_PID_OUTPUT), START_SPEED, MAX_SPEED);
          M4_VAL = constrain((pos_value[2] - PITCH_PID_OUTPUT - ROLL_PID_OUTPUT - YAW_PID_OUTPUT), START_SPEED, MAX_SPEED);

        } else {
          ang_control(0, 0, 0, 1);
          M1_VAL = START_SPEED;
          M2_VAL = START_SPEED;
          M3_VAL = START_SPEED;
          M4_VAL = START_SPEED;
        }
      } else {
        M1_VAL = MIN_SPEED;
        M2_VAL = MIN_SPEED;
        M3_VAL = MIN_SPEED;
        M4_VAL = MIN_SPEED;
      }
 }

void ESC_cali() {
    M1_VAL = CH[2];
    M2_VAL = CH[2];
    M3_VAL = CH[2];
    M4_VAL = CH[2];
}
