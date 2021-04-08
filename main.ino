#include <Wire.h>
#include <Servo.h>
#include <EEPROM.h>

#include "define.h"
#include "Serial.h"
#include "RC.h"
#include "IMU.h"
#include "POSE.h"
#include "cailibrate.h"
#include "output.h"
#include "sys_data.h"

bool terminal_mode = 0;
bool rotor_enable = 1;

unsigned long channal_read_timer;
unsigned long ch1_timer, ch2_timer, ch3_timer, ch4_timer, ch5_timer, ch6_timer;

boolean ch1_s, ch2_s, ch3_s, ch4_s, ch5_s, ch6_s;

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
	Gyro_cal();
	angle_read();

	//ISR SETUP
	PCICR |= (1 << PCIE0);
	DDRB = B00000000;
	PCMSK0 = B11111111;

	esc_output_timer = micros();
	ARM_TIMER = millis();

	digitalWrite(LED, LOW);

	PID_LOOP_TIMER = millis();

	COLLECT_DATA();
	WRITE_EEPROM();
	delay(2000);
	READ_EEPROM();
	Serial.println(__FILE__);
	Serial.println(__TIME__);
	while (1) {
		Serial.println("OK");
		Serial.println(memory.POS_P);
		Serial.println(memory.POS_I, 5);
		Serial.println(memory.POS_D);
		delay(1000);
	}

}

void loop() {

	if (micros() - esc_start_timer >= 20000) {
		PORTD |= B11111100;
		esc_start_timer = micros();
	}
	else {
		//esc signal output
		if (micros() - esc_start_timer >= M1_VAL) {
			PORTD &= B11111011;
		}
		if (micros() - esc_start_timer >= M2_VAL) {
			PORTD &= B11110111;
		}
		if (micros() - esc_start_timer >= M3_VAL) {
			PORTD &= B11101111;
		}
		if (micros() - esc_start_timer >= M4_VAL) {
			PORTD &= B11011111;
		}
		if (micros() - esc_start_timer >= M5_VAL) {
			PORTD &= B10111111;
		}
		if (micros() - esc_start_timer >= M6_VAL) {
			PORTD &= B01111111;
		}
		//caculation
		if (micros() - esc_start_timer > 2500) {

			DEBUG_PRINT();
			Serial_RX();
			angle_read();

			switch (MODE) {
			default:
				M1_VAL = MIN_SPEED;
				M2_VAL = MIN_SPEED;
				M3_VAL = MIN_SPEED;
				M4_VAL = MIN_SPEED;
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
