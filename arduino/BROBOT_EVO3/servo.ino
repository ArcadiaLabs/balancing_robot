// Default servo definitions
//#define SERVO_AUX_NEUTRAL 1500  // Servo neutral position
//#define SERVO_MIN_PULSEWIDTH 700
//#define SERVO_MAX_PULSEWIDTH 2300

// Init servo on T4 timer. Output OC4B / PB6 (Micro / Leonardo Pin10)
// We configure the Timer4 for 11 bits PWM (enhanced precision) and 16.3ms period (OK for most servos)
// Resolution: 8us per step (this is OK for servos, around 175 steps for typical servo)
void BROBOT_initServo()
{
int temp;

  // Initialize Timer4 as Fast PWM
  TCCR4A = (1<<PWM4A)|(1<<PWM4B);
  TCCR4B = 0;
  TCCR4C = (1<<PWM4D);
  TCCR4D = 0;
  TCCR4E = (1<<ENHC4); // Enhanced -> 11 bits

  temp = 1500>>3;
  TC4H = temp >> 8;
  OCR4B = temp & 0xff;

  // Reset timer
  TC4H = 0;
  TCNT4 = 0;

  // Set TOP to 1023 (10 bit timer)
  TC4H = 3;
  OCR4C = 0xFF;

  // OC4B = PB6 (Pin10)
  // Set pin as output
  DDRB |= (1 << 6);  // OC4B = PB6 (Pin10 on Leonardo board)

  //Enable OC4B / PB6 (Pin10) output
  TCCR4A |= (1<<COM4B1); 

  // -- ORIGINAL BB-ROBOT --
  // OC4A = PC7 (Pin13)  OC4B = PB6 (Pin10)   OC4D = PD7 (Pin6)
  // Set pins as outputs
//  DDRB |= (1 << 6);  // OC4B = PB6 (Pin10 on Leonardo board)
//  DDRC |= (1 << 7);  // OC4A = PC7 (Pin13 on Leonardo board)
//  DDRD |= (1 << 7);  // OC4D = PD7 (Pin6 on Leonardo board)

//  //Enable OC4A and OC4B and OCR4D output
//  TCCR4A |= (1<<COM4B1)|(1<<COM4A1);
//  TCCR4C |= (1<<COM4D1);
  // -- ORIGINAL BB-ROBOT END --

  // set prescaler to 256 and enable timer    16Mhz/256/1024 = 61Hz (16.3ms)
  TCCR4B = (1 << CS43)|(1 << CS40);
}

void BROBOT_moveServo(int pwm)
{
  pwm = constrain(pwm,SERVO_MIN_PULSEWIDTH,SERVO_MAX_PULSEWIDTH)>>3;  // Check max values and Resolution: 8us
  // 11 bits => 3 MSB bits on TC4H, LSB bits on OCR4B
  TC4H = pwm>>8;
  OCR4B = pwm & 0xFF;
}

