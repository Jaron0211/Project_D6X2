#include <Wire.h>
#include <Servo.h>
#include <EEPROM.h>
#include <avr/wdt.h>

#include "define.h"
#include "Serial.h"
#include "IMU.h"
#include "POSE.h"
#include "cailibrate.h"
#include "output.h"
#include "battery.h"

#include "sys_data.h"

bool LED_STATE;

void setup() {

	pinMode(LED, OUTPUT);

	Serial.begin(115200);

	Wire.begin();
	TWBR = 12;

	//motor setup
	DDRD |= B11111100;
	PORTD |= B11111100;
	delay(1);
	PORTD &= B00000011;

	setup_mpu_6050_registers();

	// Pre-compute IMU rotation constants (before calibration uses them)
	imu_sin = sin(radians(IMU_ROTATION_DEGREE));
	imu_cos = cos(radians(IMU_ROTATION_DEGREE));

	Gyro_cal();

	// Initialize AHRS timer before first angle_read
	ahrs_timer = micros();
	angle_read();

	digitalWrite(LED, HIGH);

	//ISR SETUP
	PCICR |= (1 << PCIE0);
	DDRB = B00000000;
	PCMSK0 = B11111111;

	esc_output_timer = micros();
	ARM_TIMER = millis();

	digitalWrite(LED, LOW);

	PID_LOOP_TIMER = micros();

	READ_EEPROM();
	battery_init();
	digitalWrite(LED, LOW);
	LED_STATE = 0;

	wdt_enable(WDTO_250MS);
}

void loop() {
	wdt_reset();

	unsigned long esc_elapsed = micros() - esc_start_timer;

	if (esc_elapsed >= 20000) {
		PORTD |= B11111100;
		esc_start_timer = micros();
	}
	else {
		//esc signal output (single timestamp to minimize jitter)
		if (esc_elapsed >= M1_VAL) { PORTD &= B11111011; }
		if (esc_elapsed >= M2_VAL) { PORTD &= B11110111; }
		if (esc_elapsed >= M3_VAL) { PORTD &= B11101111; }
		if (esc_elapsed >= M4_VAL) { PORTD &= B11011111; }
		if (esc_elapsed >= M5_VAL) { PORTD &= B10111111; }
		if (esc_elapsed >= M6_VAL) { PORTD &= B01111111; }
		//calculation
		if (esc_elapsed > 2500) {

			//DEBUG_PRINT();
			//DEBUG_PRINT_CHANNAL();
			Serial_RX();
			angle_read();
			battery_update();
			FAILSAFE();

			if (FAIL_CODE != 0) {
				Serial.println(FAIL_CODE, HEX);
			}

			if (!ARM and !have_written) {
				WRITE_EEPROM();
				Serial.println("DATA SAVED");
				have_written = 1;
			}

			switch (MODE) {
			default:
				M1_VAL = MIN_SPEED;
				M2_VAL = MIN_SPEED;
				M3_VAL = MIN_SPEED;
				M4_VAL = MIN_SPEED;
				break;
			case 0:
				M1_VAL = MIN_SPEED;
				M2_VAL = MIN_SPEED;
				M3_VAL = MIN_SPEED;
				M4_VAL = MIN_SPEED;
				if (millis() - LED_TIMER > 500) {
					digitalWrite(LED, LED_STATE);
					LED_STATE = !LED_STATE;
					LED_TIMER = millis();
				}
				break;
			case 1:
				STABLE();
				break;
			case 2:
				ESC_cali();
				break;
			}
		}
	}
}


ISR(PCINT0_vect) {
	channal_read_timer = micros();
	//ch1
	if (PINB & B00100000) {
		if (ch1_s == 0) {
			ch1_s = 1;
			ch1_timer = channal_read_timer;
		}
	}
	else if (ch1_s == 1) {
		ch1_s = 0;
		CH[0] = channal_read_timer - ch1_timer;
	}

	//ch2
	if (PINB & B00010000) {
		if (ch2_s == 0) {
			ch2_s = 1;
			ch2_timer = channal_read_timer;
		}
	}
	else if (ch2_s == 1) {
		ch2_s = 0;
		CH[1] = channal_read_timer - ch2_timer;
	}

	//ch3
	if (PINB & B00001000) {
		if (ch3_s == 0) {
			ch3_s = 1;
			ch3_timer = channal_read_timer;
		}
	}
	else if (ch3_s == 1) {
		ch3_s = 0;
		CH[2] = channal_read_timer - ch3_timer;
	}

	//ch4
	if (PINB & B00000100) {
		if (ch4_s == 0) {
			ch4_s = 1;
			ch4_timer = channal_read_timer;
		}
	}
	else if (ch4_s == 1) {
		ch4_s = 0;
		CH[3] = channal_read_timer - ch4_timer;
	}

	//ch5
	if (PINB & B00000010) {
		if (ch5_s == 0) {
			ch5_s = 1;
			ch5_timer = channal_read_timer;
		}
	}
	else if (ch5_s == 1) {
		ch5_s = 0;
		CH[4] = channal_read_timer - ch5_timer;
	}

	//ch6
	if (PINB & B00000001) {
		if (ch6_s == 0) {
			ch6_s = 1;
			ch6_timer = channal_read_timer;
		}
	}
	else if (ch6_s == 1) {
		ch6_s = 0;
		CH[5] = channal_read_timer - ch6_timer;
	}

}
