void IR_Read() {
  uint16_t value = analogRead (IR_PIN);
  uint16_t range = get_gp2d12 (value);
  int temp_range = range / 10;
  if (temp_range < 100) {
    ir_distance = temp_range;
  }
}

uint16_t get_gp2d12 (uint16_t value) {
  if (value < 10) value = 10;
  return ((67870.0 / (value - 3.0)) - 40.0);
}

