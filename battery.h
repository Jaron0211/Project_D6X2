
float read_battery_voltage() {
	return analogRead(VBAT_PIN) * (5.0f / 1023.0f) * VBAT_SCALE;
}

void battery_init() {
	// Average multiple reads for stable initial measurement
	float sum = 0;
	for (uint8_t i = 0; i < 10; i++) {
		sum += read_battery_voltage();
		delay(10);
	}
	battery_voltage = sum / 10.0f;

	// Auto-detect cell count (1S-6S)
	if (battery_voltage > 1.0f) {
		battery_cells = (uint8_t)ceil(battery_voltage / VBAT_CELL_FULL);
		if (battery_cells > 6) battery_cells = 6;
		if (battery_cells < 1) battery_cells = 1;
		battery_low_threshold = battery_cells * VBAT_CELL_MIN;
	} else {
		// No battery detected or voltage too low
		battery_cells = 0;
		battery_low_threshold = 0;
	}
}

void battery_update() {
	if (battery_cells == 0) return; // no battery detected

	// Lightweight IIR filter (slow response to avoid noise triggering failsafe)
	float raw = read_battery_voltage();
	battery_voltage = battery_voltage * 0.95f + raw * 0.05f;

	battery_low = (battery_voltage < battery_low_threshold);
}
