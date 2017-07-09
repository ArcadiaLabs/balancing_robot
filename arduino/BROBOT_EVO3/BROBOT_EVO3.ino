// BROBOT EVO 2
// SELF BALANCE ARDUINO ROBOT WITH STEPPER MOTORS

// -- HARDWARE --
// BROBOT ELECTRONIC BRAIN SHIELD : Arduino pro micro + MPU9250/6050 (I2C) IMU
//      + HC-05 / HC-06 Bluetooth module + 2x A4988 STEPPER MOTOR drivers
// STEPPER MOTORS : 2x NEMA-17
// OPTIONAL : 1x 12g servo (ES08MA-II recommended)

// -- HOW TO USE --
// The robot needs at least 10-15 seconds with no motion (robot steady) at startup to give good values...
// Put the robot at horizontal then switch it ON. It will indicate when ready with 3 small motors and servo movement.

// The robot is OFF when the angle is high (robot is horizontal). When you start raising up the robot, it
// automatically switches ON and starts a RAISE UP procedure.
// You could RAISE UP the robot manually or using the robot servo actuated arm (Servo button on the interface)
// To switch OFF the robot you could manually put the robot down on the floor (horizontal)

// -- ADDITIONAL DETAILS --
// This code doesn´t need external libraries except Wire.h

// This code requires 2x 16-bit timers for motors control, so a ATmega32u4 based arduino is mandatory (Micro, Leonardo)
// An ATmega168/328 WON'T WORK ! (only 1x 16-bit timer)

// Angle estimation using complementary filter (fusion between gyro and accel)
// Angle calculations and control part is running at 100Hz

// We use a standard PID controllers (Proportional, Integral derivative controller) for robot stability
// We have a PI controller for speed control and a PD controller for stability (robot angle)
// The output of the control (motors speed) is integrated so it´s really an acceleration, not an speed.

// Based on BROBOT EVO 2 code by JJROBOTS (Version 2.8 dated 08/03/2017)
// Author: JJROBOTS.COM
// License: GPL v2
// Original Project URL: http://jjrobots.com/b-robot

#include <Wire.h>

// -- SETTINGS --
// NORMAL MODE SETTINGS (MAXIMUN SETTINGS)
#define MAX_THROTTLE 550
#define MAX_STEERING 140
#define MAX_TARGET_ANGLE 14

// PRO MODE = MORE AGGRESSIVE (MAXIMUN SETTINGS)
#define MAX_THROTTLE_PRO 860 
#define MAX_STEERING_PRO 280
#define MAX_TARGET_ANGLE_PRO 20 
//#define MAX_TARGET_ANGLE_PRO 32 

// Default control terms
#define KP 0.32       
#define KD 0.050     
#define KP_THROTTLE 0.075 
#define KI_THROTTLE 0.1 
#define KP_POSITION 0.06  
#define KD_POSITION 0.45  

// Control gains for raiseup (the raiseup movement requiere special control parameters)
#define KP_RAISEUP 0.1   
#define KD_RAISEUP 0.16   
#define KP_THROTTLE_RAISEUP 0   // No speed control on raiseup
#define KI_THROTTLE_RAISEUP 0.0

#define MAX_CONTROL_OUTPUT 500
#define ITERM_MAX_ERROR 30   // Iterm windup constants for PI control 
#define ITERM_MAX 10000

#define ANGLE_OFFSET -1.0  // Offset angle for balance (to compensate robot own weitght distribution)

#define ZERO_SPEED 65535
#define MAX_ACCEL 15      // Maximun motor acceleration (MAX RECOMMENDED VALUE: 20) (default:15)

#define MICROSTEPPING 16   // 8 or 16 for 1/8 or 1/16 driver microstepping (default:16)

// Servo definitions
#define SERVO_AUX_NEUTRAL 1630  // Servo neutral position
#define SERVO_MIN_PULSEWIDTH 700
#define SERVO_MAX_PULSEWIDTH 2500

#define ANGLE_READY_SERVO 80    // Startup angle using servo raiseup
#define ANGLE_READY_NOSERVO 76  // Startup angle using normal raiseup

//*** PINS
// MOTORS
int MOTORS_ENABLE = 8;
// If you change these pins, please also adapt functions in 'Control' page accordingly
int MOTOR1_STEP = 7;  // STEP MOTOR 1 PORTE,6
int MOTOR1_DIR = 6;   // DIR MOTOR 1  PORTD,7
int MOTOR2_STEP = 5;  // STEP MOTOR 2 PORTC,6
int MOTOR2_DIR = 4;   // DIR MOTOR 2  PORTD,4
// gp2y0a21f sensor
int IR_PIN = A3;
// SERVO
// If you change this pin, please also adapt functions in 'servo' page accordingly
int SERVO = 10;  // SERVO

// AUTONOMOUS MODE DISTANCE PARAMETERS
int OBSTACLE_DISTANCE_MIN = 50;
int WALK_DISTANCE_MIN = 80;

#define DEBUG 0   // 0 = No debug info (default) DEBUG 1 for console output

// AUX definitions
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))
#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

uint8_t cascade_control_loop_counter = 0;
uint8_t fast_loop_counter;      // To generate a fast loop 40Hz
uint8_t medium_loop_counter;           // To generate a medium loop 7.5Hz
uint8_t slow_loop_counter;      // To generate a slow loop 2Hz

uint8_t sendBattery_counter; // To send battery status

int16_t BatteryValue;

long timer_old;
long timer_value;
float debugVariable;
float dt;

// Angle of the robot (used for stability control)
float angle_adjusted;
float angle_adjusted_Old;

// Default control values from constant definitions
float Kp = KP;
float Kd = KD;
float Kp_thr = KP_THROTTLE;
float Ki_thr = KI_THROTTLE;
float Kp_user = KP;
float Kd_user = KD;
float Kp_thr_user = KP_THROTTLE;
float Ki_thr_user = KI_THROTTLE;
float Kp_position = KP_POSITION;
float Kd_position = KD_POSITION;
bool newControlParameters = false;
bool modifing_control_parameters = false;
int16_t position_error_sum_M1;
int16_t position_error_sum_M2;
float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;
float target_angle;
int16_t throttle;
float steering;
float max_throttle = MAX_THROTTLE;
float max_steering = MAX_STEERING;
float max_target_angle = MAX_TARGET_ANGLE;
float control_output;

boolean positionControlMode = false;
uint8_t mode;  // mode = 0 Normal mode, mode = 1 Pro mode (More agressive)

int16_t motor1;
int16_t motor2;

// position control
volatile int32_t steps1;
volatile int32_t steps2;
int32_t target_steps1;
int32_t target_steps2;
int16_t motor1_control;
int16_t motor2_control;

// motors speeds
int16_t speed_M1, speed_M2;        // Actual speed of motors
int8_t  dir_M1, dir_M2;            // Actual direction of steppers motors
int16_t actual_robot_speed;        // overall robot speed (measured from steppers speed)
int16_t actual_robot_speed_Old;
float estimated_speed_filtered;    // Estimated robot speed

// IR sensor distance
long ir_distance = 0;  // IR sensor real distance (cm)

// autonomous mode
uint8_t autonomous_mode_status;   // 1: NORMAL STATUS=Walking  2: 
int16_t autonomous_mode_counter;  
int16_t autonomous_mode_distance;

boolean motors_on=false;
boolean acro_on=false;
boolean auto_on=false;
boolean ir_on=false;
boolean servo_on=false;

// INITIALIZATION
void setup()
{
  // VOLTAGE PIN INIT
  pinMode(A0, INPUT); 
  
  // STEPPERS PINS INIT
  pinMode(MOTORS_ENABLE, OUTPUT); // ENABLE MOTORS
  
  pinMode(MOTOR1_STEP, OUTPUT); 
  pinMode(MOTOR1_DIR, OUTPUT); 

  pinMode(MOTOR2_STEP, OUTPUT); 
  pinMode(MOTOR2_DIR, OUTPUT); 

  // SONAR PINS INIT
  pinMode(IR_PIN, INPUT);

  // SERVO INIT
  pinMode(SERVO, OUTPUT);
  
//  digitalWrite(MOTORS_ENABLE, HIGH);  // Disable motors
  Motors_Enable(false);

  Serial.begin(115200);     // Serial output to console
  Serial1.begin(57600);    // Serial telemetry / controls to bluetooth

  // Init virtual LEDs
  Serial1.print("*LR255G0B0*"); // *Ready* virtual LED : red
  Serial1.print("*BR0G0B0*");   // *Battery low* virtual LED : black (off)
  Serial1.print("*WR0G0B0*");   // *Battery warning* virtual LED : black (off)
  
  // Initialize I2C bus (MPU6050 is connected via I2C)
  Wire.begin();

  #if DEBUG > 0
    delay(9000);
  #else
    delay(1000);
  #endif
    Serial.println("BROBOT by JJROBOTS v2.8");

  delay(200);
  Serial.println("Don't move for 10 sec...");
  MPU6050_setup();  // setup MPU6050 IMU
  delay(500);

  // Calibrate gyros
  MPU6050_calibrate();

  // SERVO INITIALIZATION
  Serial.println("Servo init");
  BROBOT_initServo();
  BROBOT_moveServo(SERVO_AUX_NEUTRAL);

  // STEPPER MOTORS INITIALIZATION
  Serial.println("Steppers init");
  // MOTOR1 => TIMER1
  TCCR1A = 0;                       // Timer1 CTC mode 4, OCxA,B outputs disconnected
  TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler=8, => 2Mhz
  OCR1A = ZERO_SPEED;               // Motor stopped
  dir_M1 = 0;
  TCNT1 = 0;

  // MOTOR2 => TIMER3
  TCCR3A = 0;                       // Timer3 CTC mode 4, OCxA,B outputs disconnected
  TCCR3B = (1 << WGM32) | (1 << CS31); // Prescaler=8, => 2Mhz
  OCR3A = ZERO_SPEED;   // Motor stopped
  dir_M2 = 0;
  TCNT3 = 0;
  delay(200);

  // Enable stepper drivers and TIMER interrupts
  Motors_Enable(true);
  // Enable TIMERs interrupts
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 interrupt
  TIMSK3 |= (1 << OCIE1A); // Enable Timer1 interrupt

  // Little motor vibration and servo move to indicate that robot is ready
  for (uint8_t k = 0; k < 5; k++)
  {
    setMotorSpeedM1(5);
    setMotorSpeedM2(5);
    BROBOT_moveServo(SERVO_AUX_NEUTRAL + 100);
    delay(200);
    setMotorSpeedM1(-5);
    setMotorSpeedM2(-5);
    BROBOT_moveServo(SERVO_AUX_NEUTRAL - 100);
    delay(200);
  }
  BROBOT_moveServo(SERVO_AUX_NEUTRAL);

  Serial.println("Start...");
  timer_old = micros();

  // *Ready* virtual LED : green
  Serial1.print("*LR0G255B0*");
}


// MAIN LOOP
void loop()
{

  BT_MsgRead();
  
  // AUTONOMOUS MODE
  if (auto_on == true){
    autonomousMode();
  }
  
  timer_value = micros();
    
  // New IMU data?
  if (MPU6050_newData())
  {
    MPU6050_read_3axis();

//    // Increment loop counters
    fast_loop_counter++;
    medium_loop_counter++;
    slow_loop_counter++;
    
    dt = (timer_value - timer_old) * 0.000001; // dt in seconds
    timer_old = timer_value;

    angle_adjusted_Old = angle_adjusted;
    // Get new orientation angle from IMU (MPU6050)
    angle_adjusted = MPU6050_getAngle(dt) + ANGLE_OFFSET;

    

#if DEBUG==1
    Serial.print(dt);
    Serial.print(" ");
    Serial.println(angle_adjusted);
#endif
    //Serial.print("\t");

    // We calculate the estimated robot speed:
    // Estimated_Speed = angular_velocity_of_stepper_motors(combined) - angular_velocity_of_robot(angle measured by IMU)
    actual_robot_speed = (speed_M1 + speed_M2) / 2; // Positive: forward  

    int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 25.0; // 25 is an empirical extracted factor to adjust for real units
    int16_t estimated_speed = -actual_robot_speed + angular_velocity;
    estimated_speed_filtered = estimated_speed_filtered * 0.9 + (float)estimated_speed * 0.1; // low pass filter on estimated speed

#if DEBUG==2
    Serial.print(angle_adjusted);
    Serial.print(" ");
    Serial.println(estimated_speed_filtered);
#endif

    if (positionControlMode)
    {
      
      // POSITION CONTROL. INPUT: Target steps for each motor. Output: motors speed
      motor1_control = positionPDControl(steps1, target_steps1, Kp_position, Kd_position, speed_M1);
      motor2_control = positionPDControl(steps2, target_steps2, Kp_position, Kd_position, speed_M2);

      // Convert from motor position control to throttle / steering commands
      throttle = (motor1_control + motor2_control) / 2;
      throttle = constrain(throttle, -190, 190);
      steering = motor2_control - motor1_control;
      steering = constrain(steering, -50, 50);

    }

    // ROBOT SPEED CONTROL: This is a PI controller.
    //    input:user throttle(robot speed), variable: estimated robot speed, output: target robot angle to get the desired speed
    target_angle = speedPIControl(dt, estimated_speed_filtered, throttle, Kp_thr, Ki_thr);
    target_angle = constrain(target_angle, -max_target_angle, max_target_angle); // limited output


#if DEBUG==3
    Serial.print(angle_adjusted);
    Serial.print(" ");
    Serial.print(estimated_speed_filtered);
    Serial.print(" ");
    Serial.println(target_angle);
#endif

    // Stability control (100Hz loop): This is a PD controller.
    //    input: robot target angle(from SPEED CONTROL), variable: robot angle, output: Motor speed
    //    We integrate the output (sumatory), so the output is really the motor acceleration, not motor speed.
    control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
    control_output = constrain(control_output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT); // Limit max output from control

    // The steering part from the user is injected directly to the output
    motor1 = control_output + steering;
    motor2 = control_output - steering;

    // Limit max speed (control output)
    motor1 = constrain(motor1, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
    motor2 = constrain(motor2, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

    int angle_ready;
    if (servo_on == true) {    // If we press the SERVO button we start to move
      angle_ready = ANGLE_READY_SERVO;
//      angle_ready = 80;
    }
    else {
      angle_ready = ANGLE_READY_NOSERVO;
//      angle_ready = 74;  // Default angle
    }
    if ((angle_adjusted < angle_ready) && (angle_adjusted > -angle_ready)) // Is robot ready (upright?)
    {
            
      // NORMAL MODE
      Motors_Enable(true);
      // NOW we send the commands to the motors
      setMotorSpeedM1(motor1);
      setMotorSpeedM2(motor2);
      

      #if DEBUG==4
        Serial.print(angle_adjusted);
        Serial.print(" ");
        Serial.print(estimated_speed_filtered);
        Serial.print(" ");
        Serial.print(target_angle);
        Serial.print(" ");
        Serial.print(throttle);
        Serial.print(" ");
        Serial.println(steering);
      #endif
    }
    else   // Robot not ready (flat), angle > angle_ready => ROBOT OFF
    {
//      digitalWrite(MOTORS_ENABLE, HIGH);  // Disable motors
      Motors_Enable(false);
//      setMotorSpeedM1(0);
//      setMotorSpeedM2(0);
      PID_errorSum = 0;  // Reset PID I term
      Kp = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
      Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;
      // RESET steps
      steps1 = 0;
      steps2 = 0;

      #if DEBUG==4
        Serial.print(angle_adjusted);
        Serial.print(" ");
        Serial.print(estimated_speed_filtered);
        Serial.print(" ");
        Serial.print(target_angle);
        Serial.println(" 0 0");
      #endif
    }

    // Push1 Move servo arm
    if (servo_on == true)  // Move arm
    {
      if (angle_adjusted > -40) {
        BROBOT_moveServo(SERVO_MIN_PULSEWIDTH);
      }
      else {
        BROBOT_moveServo(SERVO_MAX_PULSEWIDTH);
      }
    }
    else {
      BROBOT_moveServo(SERVO_AUX_NEUTRAL);
    }

    // Normal condition?
    if ((angle_adjusted < 56) && (angle_adjusted > -56))
    {
      Kp = Kp_user;            // Default user control gains
      Kd = Kd_user;
      Kp_thr = Kp_thr_user;
      Ki_thr = Ki_thr_user;
    }
    else    // We are in the raise up procedure => we use special control parameters
    {
      Kp = KP_RAISEUP;         // CONTROL GAINS FOR RAISE UP
      Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;
    }
    
    BT_MsgSend();

  } // End of new IMU data

//  //*** Fast loop 40Hz
  if (fast_loop_counter >= 5)
  {
    fast_loop_counter = 0;
  } // End of Fast loop
  
  //*** Medium loop 7.5Hz
  if (medium_loop_counter >= 15)
  {
    medium_loop_counter = 0;
    if (ir_on == true){
      IR_Read();
    }  
  } // End of medium loop

  //*** Slow loop 1Hz
  else if (slow_loop_counter >= 1000) // 1Hz
  {
//    Serial.println("loop");
    slow_loop_counter = 0;
  }  // End of slow loop

}

void Motors_Enable(bool state){
  if (state == true){
    digitalWrite(MOTORS_ENABLE, LOW);  // Enable motors
    motors_on = true;
  }
  else{
    digitalWrite(MOTORS_ENABLE, HIGH);  // Disable motors
    setMotorSpeedM1(0);
    setMotorSpeedM2(0);
    motors_on = false;
  }
}
