// TLC Summer Intern Robot Challenge template software, Version 2019-07-02
// Lyle Kipp and Alejandro Kapauan
//
// Software for a Parallax BoE-Bot controlled by an Arduino Uno, including:
//  Dead reckoning odometry using wheel encoders
//  PID navigation controller
//  HC-SR04 ultrasonic distance measurements
//  IR beacon reception
//
// Acknowledgements
//  Odometry code draws from Parallax application note ApplyEncoder.pdf
//  For processing efficiency, the robot heading is maintained as
//  sin(heading) and cos(heading), which allows position and heading
//  to be updated without calling any trig functions.
//
//  Ultrasonic "ping" functions and configuration of the Interrupt Service
//  Routine for Arduino Timer 2 are directly derrived from the "New Ping"
//  Arduino library by Tim Eckel.
//
// Software organization notes
//  This software is organized as several mostly independent modules.
//  For editing convenience, they are all contained in a single file.
//
//  Each module has a unique prefix used for its identifier names that have
//  scope beyond a single function (constants, global variables, functions).
//  The modules and prefixes are as follows:
//
//  Module
//  Prefix  Description
//  -----------------------------------------------------------------------
//  HW  Arduino pins and interfaces; servo hardware
//  DRIVE Drive wheel control, including wheel encoders and odometry
//  NAV Navigation to a point or heading, including PID controller
//  TURRET  Turret angle control and scanning
//  PING  Ultrasonic ping sensor control
//  IRRX  Infra-red receiver control
//  FSM Finite State Machine, including general purpose & transient states
//  DIAG  Diagnostic input and output via serial interface to terminal window
//  CTRL  Top-level control functions, including init, base level, and ISR
//
//  Also, each type of identifer follows a capitalization convention:
//
//  Identifier Type   Naming Convention Example
//  -----------------------------------------------------------------------
//  Compile-time constants  ALL_CAPS_SNAKE_CASE PREFIX_CONSTANT_NAME
//  Function names    Upper_Snake_Case  Prefix_Function_Name
//  Global variables  lower_snake_case  prefix_variable_name
//
//  Note: Due to Arduino IDE naming conventions, two of the functions in
//  the CTRL module, setup() and loop(), do not include the Ctrl_ prefix.
//
//  This code has been edited using the traditional tab spacing of 8.
//  To set the Arduino IDE to match:
//   1. Exit the IDE
//   2. Edit {User}\AppData\Roaming\Arduino\preferences.txt
//   3. Modify the lines:
//    editor.tabs.expand=false
//    editor.tabs.size=8

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <Servo.h>

#define FALSE   0
#define TRUE    1

// This structure is used to make certain local variables visible for diagnostic I/O.
// It is placed here for compiler scoping reasons.
struct {
  int32_t loop_count  = 0;
  int32_t loop_time_max = 0;
  int32_t isr_count = 0;
  int16_t drive_encode_L  = 0;
  int16_t drive_encode_R  = 0;
  int8_t  nav_new_target  = FALSE;  // Flag that a new target has been set (and should be printed)
  float nav_PIDterm;
  int16_t targ_tickdiff;
} diag_data;

// This array can be used for special CPU timing studies
// Element 10 saves the start time
// Elements 0-9 contain the computed elapsed times (printed by Diag output #9)
uint32_t diag_time[11];

#if 0
// Example use:
/*** BEGIN TIMING ***/  diag_time[10] = micros();
/*** END TIMING ***/  diag_time[0 .. 9] = micros() - diag_time[10];
#endif

/******************************************************************************
 ******************************************************************************
   HARDWARE (HW)
    Arduino pins and interfaces; servo hardware
 ******************************************************************************
 ******************************************************************************/

// Digital Pins
//         0  // Reserved for USB and Serial I/O
//         1  // Reserved for USB and Serial I/O
#define HW_PIN_USER2     2  // User input pin 2
#define HW_PIN_USER3     3  // User input pin 3

#define HW_PIN_IRRX    5  // Infra-red receiver
#define HW_PIN_DRIVE_SERVO_HALT  6  // Wheel servo override
#define HW_PIN_TURRET_SERVO_HALT 7  // Turret servo override

#define HW_PIN_PING_TRIGGER  9  // Ping sensor trigger pin
#define HW_PIN_PING_ECHO  10  // Ping sensor echo pin
#define HW_PIN_TURRET_SERVO 11  // Turret servo
#define HW_PIN_DRIVE_SERVO_R  12  // Right wheel servo
#define HW_PIN_DRIVE_SERVO_L  13  // Left  wheel servo

// Analog Pins
#define HW_PIN_DRIVE_ENCODE_R  4  // Right wheel encoder
#define HW_PIN_DRIVE_ENCODE_L  5  // Left  wheel encoder

// Servos
Servo hw_drive_servo_L;
Servo hw_drive_servo_R;
Servo hw_turret_servo;

#define HW_DRIVE_SERVO_CENTER_L 1500
#define HW_DRIVE_SERVO_CENTER_R 1500
#define HW_TURRET_OFFSET  0 // Fine tuning.  Do not exceed +/- 9

/******************************************************************************
 ******************************************************************************
   DRIVE
    Primary robot drive wheel control.
    Includes wheel encoder measurement and odometry calculations
 ******************************************************************************
 ******************************************************************************/
//  Functions
//    Drive_Set_Speed
//    Drive_Calibrate_Wheels
//    Drive_Reset_Odometry
//    Drive_Do_Odometry
//      Drive_Crabwalk_State, Drive_Update_XY, Drive_Update_Angle

// Physical robot geometry parameters
// Three physical aspects of the robot determine odometry constants:
//  48  "T" = Encoder ticks per wheel revolution
//  8.250 "C" = Wheel circumference
//  4.270 "L" = Wheel spacing
//
// The math:
//  2*PI*L    Circumference of circle traveled by one wheel if the other is stationary
//  (C/T)/(2*PI*L)  Fraction of the above circle traveled when one wheel moves one tick
//  (C/T)/L   "DELTA" = Radians that the robot turns when one wheel turns one tick
//  SIN(DELTA)*(L/2)  Distance that the robot travels when one wheel turns one tick
//      Which by the small angle approximation ~= (C/T)/2

#define DRIVE_DELTA 0.040251756 // DELTA, as defined above.
#define DRIVE_SIN_DELTA 0.040240888 // Sin of DELTA
#define DRIVE_COS_DELTA 0.999190007 // Cos of DELTA
#define DRIVE_TICK_DIST 0.085937500 // Distance moved when one wheel turns one tick (using C/T/2)

#define DRIVE_SIDE_LEFT   (-1)
#define DRIVE_SIDE_RIGHT  ( 1)
#define DRIVE_SPEED_LIMIT 100 // Servos are only linear up to +/- 100.

double  drive_pos_x;    // Odometry position variables
double  drive_pos_y;
double  drive_pos_sin;    // == sin(drive_pos_heading)
double  drive_pos_cos;    // == cos(drive_pos_heading)
double  drive_pos_heading;  // == atan2(drive_pos_sin, drive_pos_cos)

int32_t drive_tickcount_l;  // Cummulative wheel encoder tick counts
int32_t drive_tickcount_r;

int16_t drive_speed_L;    // Currently applied servo power
int16_t drive_speed_R;

/******************************************************************************/

void Drive_Set_Speed (int16_t left, int16_t right)
{
  if (left  >  DRIVE_SPEED_LIMIT) left  =  DRIVE_SPEED_LIMIT;
  if (left  < -DRIVE_SPEED_LIMIT) left  = -DRIVE_SPEED_LIMIT;
  if (right >  DRIVE_SPEED_LIMIT) right =  DRIVE_SPEED_LIMIT;
  if (right < -DRIVE_SPEED_LIMIT) right = -DRIVE_SPEED_LIMIT;

  drive_speed_L = left;
  drive_speed_R = right;

  if (digitalRead(HW_PIN_DRIVE_SERVO_HALT) == LOW) {
    hw_drive_servo_L.writeMicroseconds (HW_DRIVE_SERVO_CENTER_L); // Stop servos, regardless of software speed
    hw_drive_servo_R.writeMicroseconds (HW_DRIVE_SERVO_CENTER_R);
  } else {
    hw_drive_servo_L.writeMicroseconds (HW_DRIVE_SERVO_CENTER_L + drive_speed_L); // Set left servo speed
    hw_drive_servo_R.writeMicroseconds (HW_DRIVE_SERVO_CENTER_R - drive_speed_R); // Set right servo speed
  }
}  /* Drive_Set_Speed */

/******************************************************************************/
// Drive_Calibrate_Wheels
//  Rotate wheels to be immediately after a High(1) --> Low(0) encoder tick

void Drive_Calibrate_Wheels ()
{
  int8_t  speed_l, speed_r;
  uint32_t  timeout;

  timeout = micros() + 5000000; // Stop after 5 seconds if something goes wrong

  speed_l = 30;
  speed_r = 30;
  while (speed_l > 0 || speed_r > 0) {
    if (analogRead(HW_PIN_DRIVE_ENCODE_L) > 600)  speed_l = 0;
    if (analogRead(HW_PIN_DRIVE_ENCODE_R) > 600)  speed_r = 0;
    Drive_Set_Speed (speed_l, speed_r);
    if (micros() > timeout) break;
  }

  speed_l = 30;
  speed_r = 30;
  while (speed_l > 0 || speed_r > 0) {
    if (analogRead(HW_PIN_DRIVE_ENCODE_L) < 300)  speed_l = 0;
    if (analogRead(HW_PIN_DRIVE_ENCODE_R) < 300)  speed_r = 0;
    Drive_Set_Speed (speed_l, speed_r);
    if (micros() > timeout) break;
  }

  Drive_Set_Speed (0, 0);

}  /* Drive_Calibrate_Wheels */

/******************************************************************************/
// Drive_Reset_Odometry
//  Resets odometry variables to initial state.
//  Potentially useful to reset when robot has attained an intermediate goal
//  Chaos will result if called while Navigation controller is running.

void Drive_Reset_Odometry ()
{
  drive_pos_x = 0.0;
  drive_pos_y = 0.0;
  drive_pos_sin = 1.0;  // assume robot is headed in the positive y axis direction
  drive_pos_cos = 0.0;
  drive_pos_heading = M_PI / 2;
  drive_tickcount_l = 0;
  drive_tickcount_r = 0;
}  /* Drive_Reset_Odometry */

/******************************************************************************/
// Drive_Crabwalk_State
//  Detect alternating-side ticks on the wheel encoders and avoid crab-walk odometry:
//  If the previous tick started a sequence (state == 0) and current tick is on the opposite side
//  Then the current tick is considered the second step of a sequence (state == 1).
//  Else the current tick starts a new sequence.

inline int8_t Drive_Crabwalk_State (int8_t tick_side, int8_t tick_dir)
{
  static int8_t state = 1;
  static int8_t prev_side = DRIVE_SIDE_RIGHT;

  if (tick_dir < 0)  tick_side = -tick_side;  // Reverse left/right roles if backwards tick

  if ((state == 0) && (prev_side != tick_side)) state = 1;
  else            state = 0;

  prev_side = tick_side;
  return (state);
}  /* Drive_Crabwalk_State */

/******************************************************************************/

inline void Drive_Update_XY (int8_t tick_dir)
{
  drive_pos_x += (float) tick_dir * drive_pos_cos * DRIVE_TICK_DIST;
  drive_pos_y += (float) tick_dir * drive_pos_sin * DRIVE_TICK_DIST;
}  /* Drive_Update_XY */

/******************************************************************************/

inline void Drive_Update_Angle (int8_t tick_dir)
{
  double tmp_cos;
  tmp_cos       = (drive_pos_cos * DRIVE_COS_DELTA) - (float) tick_dir * (drive_pos_sin * DRIVE_SIN_DELTA);
  drive_pos_sin = (drive_pos_sin * DRIVE_COS_DELTA) + (float) tick_dir * (drive_pos_cos * DRIVE_SIN_DELTA);
  drive_pos_cos = tmp_cos;

  drive_pos_heading += (float) tick_dir * DRIVE_DELTA;
}  /* Drive_Update_Angle */

/******************************************************************************/
// Drive_Do_Odometry
//  Reads wheel encoders and perform odometry for any ticks
//
//  The wheel encoder photosensors generate an analog voltage signal.
//  analogRead() returns the voltage on the input pin in units of
//  approximately 5 millivolts.
//   < 1.5 volts ( < 300 from analogRead) is 0/LOW
//   > 3.0 volts ( > 600 from analogRead) is 1/HIGH
//   Inbetween is ambiguous and treated as no change

void Drive_Do_Odometry ()
{
  static int8_t encode_R = 0, encode_L = 0;
  int8_t    prev_R, prev_L;
  int16_t   reading;
  int8_t    tick_dir = 0;

  // Read the encoders
  prev_R = encode_R;
  reading = analogRead(HW_PIN_DRIVE_ENCODE_R);
  if (reading < 300)  encode_R = 0;
  else if (reading > 600) encode_R = 1;
  diag_data.drive_encode_R = reading;

  prev_L = encode_L;
  reading = analogRead(HW_PIN_DRIVE_ENCODE_L);
  if (reading < 300)  encode_L = 0;
  else if (reading > 600) encode_L = 1;
  diag_data.drive_encode_L = reading;

  // Check for right wheel tick first
  if (encode_R != prev_R) {
    if (drive_speed_R >= 0) tick_dir =  1;
    else      tick_dir = -1;

    if (Drive_Crabwalk_State (DRIVE_SIDE_RIGHT, tick_dir) == 0) {
      Drive_Update_XY    (tick_dir);    // Update position then angle
      Drive_Update_Angle (tick_dir);
    } else {
      Drive_Update_Angle (tick_dir);    // Update angle then position
      Drive_Update_XY    (tick_dir);
    }
    drive_tickcount_r += tick_dir;
  }

  // Check for left wheel tick second
  // Note that a forward tick on the left wheel causes a negative angular rotation of the robot,
  // hence the negation of tick_dir when calling Drive_Update_Angle.
  if (encode_L != prev_L) {
    if (drive_speed_L >= 0) tick_dir =  1;
    else      tick_dir = -1;

    if (Drive_Crabwalk_State (DRIVE_SIDE_LEFT, tick_dir) == 0) {
      Drive_Update_XY    (tick_dir);    // Update position then angle
      Drive_Update_Angle (-tick_dir);
    } else {
      Drive_Update_Angle (-tick_dir);   // Update angle then position
      Drive_Update_XY    (tick_dir);
    }
    drive_tickcount_l += tick_dir;
  }

  if (tick_dir != 0) {  // At least one side ticked, bound drive_pos_heading
    if  (drive_pos_heading >  M_PI) drive_pos_heading -= 2 * M_PI;
    else if (drive_pos_heading < -M_PI) drive_pos_heading += 2 * M_PI;
  }
}  /* Drive_Do_Odometry */

/******************************************************************************
 ******************************************************************************
   NAV
    Driving navigation to a point or heading, including PID controller
 ******************************************************************************
 ******************************************************************************/
//  Functions
//    Nav_Get_Target
//    Nav_Set_Target
//    Nav_Next_Target
//    Nav_Do_Controller

#define NAV_PID_KP  30.0
#define NAV_PID_KI  15.0
#define NAV_PID_KD   0.0
#define NAV_PID_MAX_SUM (M_PI/8.0)

#define NAV_CONTROLLER_INIT 0
#define NAV_CONTROLLER_RUN  1

enum  nav_state_enum {
  NAV_STATE_IDLE,   // Nav not running (or perhaps paused)
  NAV_STATE_XY,   // Driving to a (x,y) destination
  NAV_STATE_TURN_L, // Turning left  by a given amount
  NAV_STATE_TURN_R, // Turning right by a given amount
  NAV_STATE_ARRIVED,  // Was in "XY" or "TURN" mode, but has arrived
};

int8_t  nav_state = NAV_STATE_IDLE;
int8_t  nav_route_auto = FALSE; // Flag to automatically sequence through nav_route table.

int16_t nav_target_speed;
float nav_target_x;
float nav_target_y;
float nav_target_turn;
float nav_target_ratio;
float nav_target_tol;

int8_t  nav_target_optimize;
float nav_target_heading;
float nav_target_distance;

#define NAV_COORD 1
#define NAV_FORWARD 2
#define NAV_HEAD  3
#define NAV_TURN  4

struct nav_target_t {
  int8_t  type;
  int16_t speed;
  float param1;
  float param2;
  float tol;
};

//  type    speed param1  param2  tol
//  -----------------------------------------------
//  NAV_COORD speed x y tol
//  NAV_FORWARD speed dist
//  NAV_HEAD  speed head  ratio
//  NAV_TURN  speed turn  ratio

PROGMEM const struct nav_target_t nav_route[] = {
  {NAV_FORWARD,  50,    36 },
  {NAV_COORD, -50,   0.0,  0.0, -3.0 },
  {NAV_TURN,   50,   360,  0.5 },
  {NAV_TURN,   50,  -360,  0.5 },
  {NAV_COORD,  50,   0.0, 24.0,  2.5 },
  {NAV_COORD,  30,  24.0, 24.0,  2.5 },
  {NAV_COORD,  30,  24.0,  0.0,  3.0 },
  {NAV_COORD,  75,   0.0,  0.0, -3.0 },
};

/******************************************************************************/
// Nav_Get_Target
//  Copy the next nav_target_t structure from the nav_route table
//  (in program flash memory) to the provided stucture (in RAM).

int8_t Nav_Get_Target (struct nav_target_t *target)
{
  static int16_t  idx = 0;
  int16_t i;
  char *pgm_ptr;
  char *ram_ptr;

  if (idx >= (sizeof (nav_route) / sizeof (struct nav_target_t))) {
    return (0);
  }

  pgm_ptr = (char *) & (nav_route[idx++]);
  ram_ptr = (char *) target;

  for (i = 0; i < sizeof(struct nav_target_t); i++)
    *ram_ptr++ = pgm_read_byte_near(pgm_ptr++);

  return (1);
}  /* Nav_Get_Target */

/******************************************************************************/
// Nav_Set_Target
//  Initialize the navigation module based on the nav_target_t parameter
//
//  NAV_COORD and NAV_FORWARD both set nav_state=NAV_STATE_XY, which uses
//  the PID controller.
//  For NAV_FORWARD, compute the (x,y) coordinates of a point 4 inches
//  beyond the requested target.  Then drive with a 4-inch tollerance.
//
//  NAV_HEAD and NAV_TURN both set nav_state=NAV_STATE_TURN_[L,R].
//  Both compute the number of radians to turn (>0 for left, <0 for right).
//  That is converted into units of wheel ticks in Nav_Do_Controller.

void Nav_Set_Target (struct nav_target_t *target)
{
  float temp;

  switch (target->type) {
    case NAV_COORD:
      nav_target_speed  = target->speed;
      nav_target_x    = target->param1;
      nav_target_y    = target->param2;
      nav_target_tol    = target->tol;
      nav_state   = NAV_STATE_XY;
      break;

    case NAV_FORWARD:
      nav_target_speed  = target->speed;
      if (target->param1 > 0) temp = target->param1 + 4.0;
      else      temp = target->param1 - 4.0;
      nav_target_x    = drive_pos_x + drive_pos_cos * temp;
      nav_target_y    = drive_pos_y + drive_pos_sin * temp;
      nav_target_tol    = 4.0;
      nav_state   = NAV_STATE_XY;
      break;

    // +++HEAD and TURN do not currently work with negative speeds.
    case NAV_HEAD:
      nav_target_speed  = target->speed;
      nav_target_turn   = target->param1 * (M_PI / 180.0) - drive_pos_heading;
      if  (nav_target_turn >  M_PI) nav_target_turn -= 2 * M_PI;
      else if (nav_target_turn < -M_PI) nav_target_turn += 2 * M_PI;
      nav_target_ratio  = target->param2;
      nav_state   = (nav_target_turn >= 0 ? NAV_STATE_TURN_L : NAV_STATE_TURN_R);
      break;

    case NAV_TURN:
      nav_target_speed  = target->speed;
      nav_target_turn   = target->param1 * (M_PI / 180.0);
      nav_target_ratio  = target->param2;
      nav_state   = (nav_target_turn >= 0 ? NAV_STATE_TURN_L : NAV_STATE_TURN_R);
      break;
  }

  Nav_Do_Controller (NAV_CONTROLLER_INIT);
  diag_data.nav_new_target = TRUE;
}  /* Nav_Set_Target */

/******************************************************************************/

void Nav_Set_Target2 (int8_t type, int16_t speed, float param1, float param2, float tol)
{
  struct nav_target_t target;

  target.type = type;
  target.speed  = speed;
  target.param1 = param1;
  target.param2 = param2;
  target.tol  = tol;

  Nav_Set_Target (&target);
}  /* Nav_Set_Target2 */

/******************************************************************************/

int8_t Nav_Next_Target ()
{
  struct nav_target_t target;
  int8_t  rtn;

  rtn = Nav_Get_Target (&target);

  if (rtn > 0) {
    Nav_Set_Target (&target);
  } else {
    nav_state = NAV_STATE_IDLE; // No more targets
  }
  return (rtn);
}  /* Nav_Next_Target */

/******************************************************************************/
// Nav_Do_Controller
//  Navigation to Target controller function
//  Includes PID controller for X-Y navigation.

void Nav_Do_Controller (int8_t operation)
{
  double diff_x, diff_y;    // Local PID (NAV_STATE_XY) variables
  static double error = 0.0;
  static double sum_error = 0.0;
  double old_error;
  double Pterm, Iterm, Dterm, PIDterm;

  static int32_t  tickdiff; // Target tick difference for NAV_STATE_TURN_[L,R]

  if (operation == NAV_CONTROLLER_INIT) {
    switch (nav_state) {
      case NAV_STATE_XY:
        error = 0.0;
        sum_error = 0.0;
        if (nav_target_tol < 0.0) {
          nav_target_tol = -nav_target_tol;
          nav_target_optimize = TRUE;
        } else {
          nav_target_optimize = FALSE;
        }
        break;

      case NAV_STATE_TURN_L:
      case NAV_STATE_TURN_R:
        tickdiff = drive_tickcount_r - drive_tickcount_l
                   + lround (nav_target_turn * (1.0 / DRIVE_DELTA));
        diag_data.targ_tickdiff = tickdiff;
        break;
    }

    return;
  }

  switch (nav_state) {
    case NAV_STATE_IDLE:
    case NAV_STATE_ARRIVED:
      return;
      break;

    case NAV_STATE_XY:
      diff_x = nav_target_x - drive_pos_x;
      diff_y = nav_target_y - drive_pos_y;
      nav_target_distance = hypot(diff_y, diff_x);
      nav_target_heading = atan2(diff_y, diff_x); // Absolute heading of target

      old_error = error;
      error = nav_target_heading - drive_pos_heading; // Relative heading of target
      if (nav_target_speed < 0) error += M_PI;  // Need to face away from target when driving backwards
      if  (error >  M_PI) error -= 2 * M_PI;
      else if (error < -M_PI) error += 2 * M_PI;

      if (abs(sum_error + 0.01 * error) <= NAV_PID_MAX_SUM) // Compute error integral, with integral wind-up check
        sum_error += 0.01 * error;

      if (nav_target_distance <= nav_target_tol) {
        if (!nav_target_optimize) {   // Target aquired; no need to try to get closer
          nav_state = NAV_STATE_ARRIVED;
          return;
        }

        // Target aquired;  See if the robot will get closer by continuing.  The trig:
        // If the robot drives straight on its current heading, then the closest point to the target will be reached in
        //  nav_target_distance * cos(error)
        // So, stop if (nav_target_distance * cos(error) <= DRIVE_TICK_DIST); otherwise keep going
        // Note that both wheels could tick, so we can't divide the tick distance by 2
        //
        // To avoid trig functions, use sin with the small angle approximation
        // Even for large angles, the error from this approximation is negligible ( < 0.5 * DRIVE_TICK_DIST)
        //
        // Since the angle error can become large as the distance goes to zero, wheel speeds are not adjusted for error

        Drive_Set_Speed (nav_target_speed, nav_target_speed);   // Drive straight
        diag_data.nav_PIDterm = 0.0;

        if (nav_target_distance * ((M_PI / 2) - abs(error)) <= DRIVE_TICK_DIST) {
          nav_state = NAV_STATE_ARRIVED;
        }
        return;
      }

      Pterm = NAV_PID_KP * error;
      Iterm = NAV_PID_KI * sum_error;
      Dterm = NAV_PID_KD * (error - old_error);
      PIDterm = Pterm + Iterm + Dterm;
      diag_data.nav_PIDterm = PIDterm;

      Drive_Set_Speed (nav_target_speed - PIDterm, nav_target_speed + PIDterm);

      break;

    case NAV_STATE_TURN_L:
      if (drive_tickcount_r - drive_tickcount_l >= tickdiff) {
        nav_state = NAV_STATE_ARRIVED;
        return;
      }

      Drive_Set_Speed (nav_target_speed * nav_target_ratio, nav_target_speed);

      break;

    case NAV_STATE_TURN_R:
      if (drive_tickcount_r - drive_tickcount_l <= tickdiff) {
        nav_state = NAV_STATE_ARRIVED;
        return;
      }

      Drive_Set_Speed (nav_target_speed, nav_target_speed * nav_target_ratio);

      break;
  }
}  /* Nav_Do_Controller */

/******************************************************************************
 ******************************************************************************
   TURRET
    Turret angle control and scanning
 ******************************************************************************
 ******************************************************************************/
//  Functions
//    Turret_Set_Angle
//    Turret_Scan

#define TURRET_MAX_ANGLE  90

enum  turret_state_enum {
  TURRET_STATE_IDLE,  // Turret scanning functionality not running
  TURRET_STATE_SCANNING,  // Turret scanning functionality is running
};

int8_t  turret_state = TURRET_STATE_IDLE;

int8_t  turret_angle = 0;
uint32_t  turret_arrive_time = 0;

/******************************************************************************/
// Turret_Set_Angle
//  Set turret to the specified angle.  (-90:left, 0:center, 90:right)

void Turret_Set_Angle (int8_t new_angle)
{
  uint32_t  now;

  if (new_angle < -TURRET_MAX_ANGLE)  new_angle = -TURRET_MAX_ANGLE;
  if (new_angle >  TURRET_MAX_ANGLE)  new_angle =  TURRET_MAX_ANGLE;

  if (digitalRead(HW_PIN_TURRET_SERVO_HALT) != LOW) {
    hw_turret_servo.write(90 - new_angle + HW_TURRET_OFFSET);
  }

  now = micros();
  if (turret_arrive_time < now)
    turret_arrive_time = now;
  turret_arrive_time += abs (new_angle - turret_angle) * 4000 + 25000;
  turret_angle = new_angle;
}  /* Turret_Set_Angle */

/******************************************************************************/

void Turret_Scan ()
{
  static int8_t increment = 10;

  if (turret_state != TURRET_STATE_SCANNING)
    return;

  if (abs(turret_angle + increment) > TURRET_MAX_ANGLE)
    increment = -increment;

  Turret_Set_Angle (turret_angle + increment);
}  /* Turret_Scan */

/******************************************************************************
 ******************************************************************************
   PING
    Ultrasonic ping sensor control
    Includes sending ping, ISR echo check, and table of recent results
 ******************************************************************************
 ******************************************************************************/
//  Function
//    Ping_Setup
//    Ping_Send
//    Ping_Echo_Check
//    Ping_Results

#define PING_MAX_TRIGGER_TIME 2000  // Maximum usec to wait for sensor to start a ping
#define PING_US_ROUNDTRIP_IN  146 // Microseconds (uS) it takes sound to travel round-trip 1 inch (2 inches total).
#define PING_MAX_DISTANCE 100 // Maximum distance (in inches) to detect
#define PING_MAX_TIME   ((PING_MAX_DISTANCE) * (PING_US_ROUNDTRIP_IN))

struct {
  uint8_t triggerBit;
  uint8_t echoBit;
  volatile uint8_t *triggerOutput;
  volatile uint8_t *triggerMode;
  volatile uint8_t *echoInput;
} ping_hw;

enum  ping_state_enum {
  PING_STATE_INITFAIL1  = -3, // Init fail (check hardware)
  PING_STATE_INITFAIL2  = -2, // Init fail (check hardware)
  PING_STATE_NORESULT = -1, // No result generated (check ISR)
  PING_STATE_IDLE   =  0, // Ping functionality not running
  PING_STATE_INPROG,    // Ping in progress
  PING_STATE_ECHO,    // Ping completed, echo received
  PING_STATE_NOECHO = 127, // Ping completed, no echo received
};

volatile int8_t ping_state = PING_STATE_IDLE;
volatile uint32_t ping_echo_time;
volatile uint32_t ping_timeout;

int8_t  ping_dist[10];    // Data for last 10 pings; [0] is most recent.

// 1-126 is measured distance in inches
// -3, -2, -1, 0, & 127 are same as ping_state.
int8_t  ping_angle[10];   // Turret angle setting when ping was done.
int8_t  ping_sort5 [5];   // Indicies into results arrays of most recent  5 pings (sorted)
int8_t  ping_sort10[10];  // Indicies into results arrays of most recent 10 pings (sorted)
float  ping_avg5 = 0;        // Average of last 5 pings
float  ping_avg10 = 0;       // Average of last 10 pings

/******************************************************************************/

void Ping_Setup () {
  // Based on New Ping "sonar()" constructor
  int8_t  i;

  ping_hw.triggerBit  = digitalPinToBitMask(HW_PIN_PING_TRIGGER); // Get the port register bitmask for the trigger pin.
  ping_hw.echoBit   = digitalPinToBitMask(HW_PIN_PING_ECHO);  // Get the port register bitmask for the echo pin.

  ping_hw.triggerOutput = portOutputRegister(digitalPinToPort(HW_PIN_PING_TRIGGER));  // Get the output port register for the trigger pin.
  ping_hw.echoInput = portInputRegister(digitalPinToPort(HW_PIN_PING_ECHO));  // Get the input port register for the echo pin.

  ping_hw.triggerMode = (uint8_t *) portModeRegister(digitalPinToPort(HW_PIN_PING_TRIGGER));  // Get the port mode register for the trigger pin.
  *ping_hw.triggerMode  |= ping_hw.triggerBit;  // Set trigger pin to output.

  for (i = 0; i < 10; i++) {
    ping_dist[i] = 0;
    ping_angle[i] = 0;
    ping_sort10[i] = i;
    if (i < 5)  ping_sort5[i] = i;
  }
}  /* Ping_Setup */

/******************************************************************************/

void Ping_Send () {
  // Based on New Ping functions ping_timer() and ping_trigger()

  uint32_t  guard_timer;

  *ping_hw.triggerOutput  &= ~ping_hw.triggerBit; // Set the trigger pin low, should already be low, but this will make sure it is.
  delayMicroseconds(4);       // Wait for pin to go low, testing shows it needs 4uS to work every time.
  *ping_hw.triggerOutput  |= ping_hw.triggerBit;  // Set trigger pin high, this tells the sensor to send out a ping.
  delayMicroseconds(10);        // Wait long enough for the sensor to realize the trigger pin is high. Sensor specs say to wait 10uS.
  *ping_hw.triggerOutput  &= ~ping_hw.triggerBit; // Set trigger pin back to low.

  guard_timer = micros() + PING_MAX_TRIGGER_TIME;   // Set a timeout for the ping to trigger.
  while (*ping_hw.echoInput & ping_hw.echoBit)    // Wait for echo pin to clear.
    if (micros() > guard_timer) {     // Something went wrong, abort.
      ping_state = PING_STATE_INITFAIL1;
      return;
    }
  while (!(*ping_hw.echoInput & ping_hw.echoBit))   // Wait for ping to start.
    if (micros() > guard_timer) {     // Something went wrong, abort.
      ping_state = PING_STATE_INITFAIL2;
      return;
    }

  ping_timeout  = micros() + PING_MAX_TIME;   // Ping started, set the timeout.
  ping_state  = PING_STATE_INPROG;
}  /* Ping_Send */

/******************************************************************************/
// Ping_Echo_Check
//  Called from the ISR to check for a ping echo

inline void Ping_Echo_Check () {
  // Based on New Ping function check_timer()

  if (ping_state == PING_STATE_INPROG) {
    if (micros() > ping_timeout) {        // Timed out without an echo
      ping_state = PING_STATE_NOECHO;
    } else if (!(*ping_hw.echoInput & ping_hw.echoBit)) { // Ping echo received.
      ping_echo_time = (micros() - (ping_timeout - PING_MAX_TIME) - 9); // Calculate ping time, 9uS of overhead.
      ping_state = PING_STATE_ECHO;
    } else {
      // still waiting
    }
  }
}  /* Ping_Echo_Check */

/******************************************************************************/

void Ping_Results () {
  int8_t  i, j;

  for (i = 9; i > 0; i--) {
    ping_dist[i]  = ping_dist[i - 1];
    ping_angle[i] = ping_angle[i - 1];
  }

  switch (ping_state) {
    case PING_STATE_ECHO:   // Ping came back; Convert ping_echo_time into a distance
      ping_dist[0] = ping_echo_time / PING_US_ROUNDTRIP_IN;
      break;

    case PING_STATE_IDLE:   // Apparently ping never started, or results were previously read
    case PING_STATE_INPROG: // Echo should have come back or timed out by now; might be ISR error
      ping_dist[0] = PING_STATE_NORESULT;
      break;

    default:      // Other values of ping_state (e.g. errors) copied directly into result array.
      ping_dist[0] = ping_state;
      break;

  }

  ping_angle[0] = turret_angle;
  ping_state = PING_STATE_IDLE;
  ping_echo_time = 0;

  // Generate the sorted array indices.
  // First, add 1 to previous indices to match data slide; also, find the entry that slid off the end
  j = 9;
  for (i = 0; i < 10; i++) {
    if (ping_sort10[i] == 9)
      j = i;
    else
      ping_sort10[i]++;
  }

  // j points to the empty slot; move it to the end

  for (; j < 9; j++) {
    ping_sort10[j] = ping_sort10[j + 1];
  }


  // The last element of ping_sort10[] is now unused.  Do an insertion sort
  i = 9;
  while ((i > 0) && (ping_dist[ping_sort10[i - 1]] > ping_dist[0])) {
    ping_sort10[i] = ping_sort10[i - 1];
    i--;
  }
  ping_sort10[i] = 0;

  // Generate ping_sort5 from ping_sort10
  j = 0;
  for (i = 0; i < 10; i++) {
    if (ping_sort10[i] <= 4)
      ping_sort5[j++] = ping_sort10[i];
  }

  // Calculate averages
  int16_t sum5 = 0;
  int16_t sum10 = 10;
  for(i = 0; i<10; i++){
    if(i<5){
      sum5 += ping_dist[i];
    }
    sum10 += ping_dist[i];
  }
  ping_avg5 = sum5 / 5.0;
  ping_avg10 = sum10 / 10.0;
}  /* Ping_Results */

/******************************************************************************
 ******************************************************************************
   IRRX
    Infra-red receiver control
 ******************************************************************************
 ******************************************************************************/
//  Functions
//    Irrx_Setup
//    Irrx_Code_Check
//    Irrx_Results

struct {
  volatile uint8_t  *irrxInput;
  uint8_t irrxBit;
} irrx_hw;

volatile int8_t isr_ir_value = 0;
volatile int8_t isr_ir_available = 0;


/******************************************************************************/

void Irrx_Setup () {
  irrx_hw.irrxInput = portInputRegister(digitalPinToPort(HW_PIN_IRRX));
  irrx_hw.irrxBit = digitalPinToBitMask(HW_PIN_IRRX);
}  /* Irrx_Setup */

/******************************************************************************/

inline void Irrx_Code_Check () {
  static int8_t count = 0;
  static int8_t input_pin = 0;
  static int8_t preamble_count = 0;
  static int8_t poll_bits = 0;

  count++;
  // Perform polling tasks below every 10 interrupts or 500 us
  // Stagger tasks to limit real time used during any one interrupt

  // Task 1 Check for preamble
  if (count == 5) {
    if (*irrx_hw.irrxInput & irrx_hw.irrxBit) {
      input_pin = 0;  // no carrier
      if (preamble_count == 9) {
        // if preamble count was pegged when
        // carrier goes away then start polling bits
        poll_bits = 1;
        isr_ir_available = 0;
        isr_ir_value = 0;
      }
      preamble_count = 0;
    } else {
      input_pin = 1;  // carrier present
      preamble_count++;
      // Preamble should be at least 5 ms long or 10 x 500 us
      // If carrier has been around that long then peg the
      // preamble counter and clear the bit polling counter
      if (preamble_count > 9) {
        preamble_count = 9;
        poll_bits = 0;
      }
    }

    // Task 2 Accumulate bits
  } else if (count > 9) {
    count = 0;
    if (poll_bits > 0) {
      poll_bits++;
      // Sample bits at 2.0, 5.0, 8.0 and 12.0 ms
      // after end of preamble
      if (poll_bits == 6) {
        if (input_pin) isr_ir_value |= 8;
      } else if (poll_bits == 12) {
        if (input_pin) isr_ir_value |= 4;
      } else if (poll_bits == 18) {
        if (input_pin) isr_ir_value |= 2;
      } else if (poll_bits == 24) {
        if (input_pin) isr_ir_value |= 1;
        isr_ir_available = 1;
      }
      if (poll_bits > 32) poll_bits = 32;
    }
  }
}  /* Irrx_Code_Check */

/******************************************************************************/

void Irrx_Results () {
#if 0
  if (isr_ir_available) {
    isr_ir_available = 0;
    Serial.println(isr_ir_value & 0x0f);
  } else {
    Serial.println("0");
  }
#endif
}  /* Irrx_Results */

/******************************************************************************
 ******************************************************************************
   FSM
    Finite State Machine
    Includes general purpose states and special transient wait states
 ******************************************************************************
 ******************************************************************************/
//  Functions
//    Fsm_Run

enum  fsm_state_enum {
  // The first three (negative value) states are special transient states.
  // FSM will automatically advance to next state when condition is satisfied.

  FSM_STATE_WAIT_NAV  = -3, // Wait for nav_state == ARRIVED
  FSM_STATE_WAIT_TURRET = -2, // Wait for turret rotation completion
  FSM_STATE_WAIT_MICROS = -1, // Wait for micros() >= fsm_micros_timeout
  //----- Auto-advance states above; regular states below
  FSM_STATE_IDLE    =  0,
  FSM_STATE_START,    // Initial state when FSM is used
  FSM_STATE_TO_SQUARE, // Travel to edge of square
  FSM_STATE_FACE_EAST, // Face east
  FSM_STATE_DRIVE_SQUARE1, // Drive around square
  FSM_STATE_FACE_SOUTH, // Face south
  FSM_STATE_DRIVE_SQUARE2, // Drive around square
  FSM_STATE_FACE_WEST, // Face west
  FSM_STATE_DRIVE_SQUARE3, // Drive around square
  FSM_STATE_FACE_NORTH, // Face north
  FSM_STATE_DRIVE_SQUARE4, // Drive around square
  FSM_STATE_FACE_EAST2, // Face south
  FSM_STATE_DRIVE_SQUARE5, // Drive around square
  FSM_STATE_FACE_SOUTH2, // Face south before scanning
  FSM_STATE_WAIT_BEFORE_SCAN, // Wait before scanning
  FSM_STATE_TURN_TO_SCAN, // Turn turret to scan_angle
  FSM_STATE_SCAN,         // Read ping_dist, act accordingly
  FSM_STATE_FACE_PARK,    // Face final reflector
  FSM_STATE_PARK,         // Park at final reflector
  FSM_STATE_DONE = 99, // Terminal state
};

int8_t  fsm_state = FSM_STATE_START;
int8_t  fsm_next_state; // Used only for Auto-advance states
uint32_t  fsm_micros_timeout; // Used for WAIT_MICROS state
int8_t scan_angle = -60;
float reflector_dist, reflector_angle;
const int8_t DRIVE_SPEED = 100;
const int8_t SQUARE_S = 66;

/******************************************************************************/
void Fsm_Run ()
{
  if (fsm_state < 0) {
    switch (fsm_state) {
      case FSM_STATE_WAIT_NAV:
        if (nav_state == NAV_STATE_ARRIVED)
          fsm_state = fsm_next_state;
        break;

      case FSM_STATE_WAIT_TURRET:
        if (micros() >= turret_arrive_time)
          fsm_state = fsm_next_state;
        break;

      case FSM_STATE_WAIT_MICROS:
        if (micros() >= fsm_micros_timeout)
          fsm_state = fsm_next_state;
        break;

      default:
        // Should never get here.
        break;
    }

    if (fsm_state < 0) {    // Still waiting
      return;
    }
  }
  switch (fsm_state) {
    case FSM_STATE_IDLE:
      break;

    case FSM_STATE_START:
      turret_state = TURRET_STATE_IDLE;
      Drive_Set_Speed(0, 0);
      fsm_state = FSM_STATE_TO_SQUARE;
      break;

    case FSM_STATE_TO_SQUARE:
      Serial.println("Moving to square edge");
      Nav_Set_Target2(NAV_COORD, DRIVE_SPEED, 0, SQUARE_S/2, -4);
      fsm_state = FSM_STATE_WAIT_NAV;
      fsm_next_state = FSM_STATE_FACE_EAST;
      break;

    case FSM_STATE_FACE_EAST:
      Serial.println("Facing East");
      Nav_Set_Target2(NAV_HEAD, 50, 0, -1, -4);
      fsm_state = FSM_STATE_WAIT_NAV;
      fsm_next_state = FSM_STATE_DRIVE_SQUARE1;
      break;

    case FSM_STATE_DRIVE_SQUARE1:
      Serial.println("Drive Square 1");
      Nav_Set_Target2(NAV_COORD, DRIVE_SPEED, SQUARE_S/2, SQUARE_S/2, -4);
      fsm_state = FSM_STATE_WAIT_NAV;
      fsm_next_state = FSM_STATE_FACE_SOUTH;
      break;

    case FSM_STATE_FACE_SOUTH:
      Serial.println("Facing South");
      Nav_Set_Target2(NAV_HEAD, 50, 270, -1, -4);
      fsm_state = FSM_STATE_WAIT_NAV;
      fsm_next_state = FSM_STATE_DRIVE_SQUARE2;
      break;

    case FSM_STATE_DRIVE_SQUARE2:
      Serial.println("Drive Square 2");
      Nav_Set_Target2(NAV_COORD, DRIVE_SPEED, SQUARE_S/2, -SQUARE_S/2, -4);
      fsm_state = FSM_STATE_WAIT_NAV;
      fsm_next_state = FSM_STATE_FACE_WEST;
      break;

    case FSM_STATE_FACE_WEST:
      Serial.println("Facing West");
      Nav_Set_Target2(NAV_HEAD, 50, 180, -1, -4);
      fsm_state = FSM_STATE_WAIT_NAV;
      fsm_next_state = FSM_STATE_DRIVE_SQUARE3;
      break;

    case FSM_STATE_DRIVE_SQUARE3:
      Serial.println("Drive Square 3");
      Nav_Set_Target2(NAV_COORD, DRIVE_SPEED, -SQUARE_S/2, -SQUARE_S/2, -4);
      fsm_state = FSM_STATE_WAIT_NAV;
      fsm_next_state = FSM_STATE_FACE_NORTH;
      break;

    case FSM_STATE_FACE_NORTH:
      Serial.println("Facing North");
      Nav_Set_Target2(NAV_HEAD, 50, 90, -1, -4);
      fsm_state = FSM_STATE_WAIT_NAV;
      fsm_next_state = FSM_STATE_DRIVE_SQUARE4;
      break;

    case FSM_STATE_DRIVE_SQUARE4:
      Serial.println("Drive Square 4");
      Nav_Set_Target2(NAV_COORD, DRIVE_SPEED, -SQUARE_S/2, SQUARE_S/2, -4);
      fsm_state = FSM_STATE_WAIT_NAV;
      fsm_next_state = FSM_STATE_FACE_EAST2;
      break;

    case FSM_STATE_FACE_EAST2:
      Serial.println("Facing East 2");
      Nav_Set_Target2(NAV_HEAD, 50, 0, -1, -4);
      fsm_state = FSM_STATE_WAIT_NAV;
      fsm_next_state = FSM_STATE_DRIVE_SQUARE5;
      break;

    case FSM_STATE_DRIVE_SQUARE5:
      Serial.println("Drive Square 5");
      Nav_Set_Target2(NAV_COORD, DRIVE_SPEED, 0, SQUARE_S/2, -4);
      fsm_state = FSM_STATE_WAIT_NAV;
      fsm_next_state = FSM_STATE_FACE_SOUTH2;
      break;

    case FSM_STATE_FACE_SOUTH2:
      Serial.println("Facing South 2");
      Nav_Set_Target2(NAV_HEAD, 50, 270, -1, -4);
      fsm_state = FSM_STATE_WAIT_NAV;
      fsm_next_state = FSM_STATE_WAIT_BEFORE_SCAN;
      break;

    case FSM_STATE_WAIT_BEFORE_SCAN:
      Serial.println("Waiting");
      Drive_Set_Speed(0, 0);
      fsm_state = FSM_STATE_WAIT_MICROS;
      fsm_micros_timeout = micros() + 5000000;
      fsm_next_state = FSM_STATE_TURN_TO_SCAN;

    case FSM_STATE_TURN_TO_SCAN:
      Serial.print("Scanning at angle: ");
      Serial.println(scan_angle);
      Turret_Set_Angle(scan_angle);
      fsm_state = FSM_STATE_WAIT_MICROS;
      fsm_micros_timeout = micros() + 750000;
      fsm_next_state = FSM_STATE_SCAN;
      if(scan_angle > 60){
        // TODO: change this to make a second circle around if scan fails
        Serial.println("Scan failed.");
        fsm_state = FSM_STATE_DONE;
      }
      break;
    
    case FSM_STATE_SCAN:
      if(ping_avg5 < 50 && ping_avg5 > 0){
        Serial.print("Scan reads YES: ");
        Serial.println(ping_avg5);
        reflector_angle = 270 - turret_angle;
        reflector_dist = ping_avg5;
        fsm_state = FSM_STATE_FACE_PARK;
      }
      else{
        Serial.print("Scan reads NO: ");
        Serial.println(ping_avg5);
        scan_angle += 30;
        fsm_state = FSM_STATE_TURN_TO_SCAN;
      }
      break;

    case FSM_STATE_FACE_PARK:
      Serial.println("Reflector found");
      Nav_Set_Target2(NAV_HEAD, 50, reflector_angle, -1, 1);
      fsm_state = FSM_STATE_WAIT_NAV;
      fsm_next_state = FSM_STATE_PARK;
      break;

    case FSM_STATE_PARK:
      Serial.println("Parking...");
      Nav_Set_Target2(NAV_FORWARD, 100, reflector_dist - 5, 0, 0);
      fsm_state = FSM_STATE_WAIT_NAV;
      fsm_next_state = FSM_STATE_DONE;
      break;
    
    case FSM_STATE_DONE:
      Drive_Set_Speed (0, 0);
      turret_state = TURRET_STATE_IDLE;
      break;

    default:
      // Probably should never get here.
      break;
  }
} /* Fsm_Run */

/******************************************************************************
 ******************************************************************************
   DIAG
    Diagnostic input and output via serial interface to terminal window
    Includes print routines and shadow copies of internal variables
 ******************************************************************************
 ******************************************************************************/
//  Functions
//    Diag_Input
//    Diag_Output

uint16_t  diag_output_bitmap = 0x00;
uint8_t diag_output_frequency = 1;

/******************************************************************************/

void Diag_Input ()
{
  int inchar;
  int16_t new_L = drive_speed_L;
  int16_t new_R = drive_speed_R;
  int16_t new_angle = turret_angle;
  static const int8_t freq_arr[] = {0, 1, 2, 5, 10, 20, 50, 100};
  static int8_t   freq_idx = 1;
  static int8_t   nav_prev;
  static int8_t   turret_prev;
  static int8_t   fsm_prev;


  if (Serial.available() <= 0)
    return;

  inchar = Serial.read();
  switch (inchar) {
    case '0':
      diag_output_bitmap = 0x00;
      freq_idx = 1;
      diag_output_frequency = freq_arr[freq_idx];
      break;

    case '1': case '2': case '3': case '4': case '5':
    case '6': case '7': case '8': case '9': case ':':
      diag_output_bitmap ^= (0x1 << (inchar - '0'));
      break;

    case '+':
      if (freq_idx < 7)   freq_idx++;
      diag_output_frequency = freq_arr[freq_idx];
      break;

    case '-':
      if (freq_idx > 0)   freq_idx--;
      diag_output_frequency = freq_arr[freq_idx];
      break;

    case ' ':
      nav_prev = nav_state;   nav_state = NAV_STATE_IDLE;
      turret_prev = turret_state; turret_state = TURRET_STATE_IDLE;
      fsm_prev = fsm_state;   fsm_state = FSM_STATE_IDLE;
      new_L = 0;
      new_R = 0;
      break;

    case 'r':
      nav_state = nav_prev;
      turret_state = turret_prev;
      fsm_state = fsm_prev;
      break;

    case 'q':
      new_L += 10;
      break;

    case 'a':
      new_L -= 10;
      break;

    case 'p':
      new_R += 10;
      break;

    case 'l':
      new_R -= 10;
      break;

    case 'm':
      new_angle = 0;
      break;

    case ',':
      new_angle -= 10;
      break;

    case '.':
      new_angle += 10;
      break;
  }

  Drive_Set_Speed (new_L, new_R);
  Turret_Set_Angle (new_angle);
}  /* Diag_Input */

/******************************************************************************/
// Diag_Output
//  The following #defines are used to include output separators.
//  It is generally frowned upon to use the C preprocessor to circumvent
//  the language syntax, but in this case the result seems cleaner.
#define COMMA Serial.write(',')
#define TAB Serial.write('\t')
#define NEWLINE Serial.println("")

void Diag_Output (int8_t subtract)
{
  static int8_t bitnum = 0;
  static int8_t countdown = 100;
  int8_t  i;

  if (diag_output_bitmap == 0)
    return;

  if ( (countdown -= subtract) > 0)
    return;

  countdown = 100;

  while ( (diag_output_bitmap & (0x1 << bitnum)) == 0) {
    bitnum++;
    if (bitnum > 10)
      bitnum = 0;
  }

  switch (bitnum++) {
    case 1:     // Primary entry counts and timing
      Serial.print(F("1-Time  Loop#,Max,ISR#,micros "));
      Serial.print(diag_data.loop_count);   TAB;
      Serial.print(diag_data.loop_time_max);    TAB;
      Serial.print(diag_data.isr_count);    TAB;
      Serial.print(micros());       NEWLINE;
      diag_data.loop_time_max = 0;
      break;

    case 2:     // HW
      Serial.print(F("2-HW  Encode_L,R,Pins2,3,6,7  "));
      Serial.print(diag_data.drive_encode_L);   TAB;
      Serial.print(diag_data.drive_encode_R);   TAB;
      Serial.print(digitalRead(HW_PIN_USER2));  TAB;
      Serial.print(digitalRead(HW_PIN_USER3));  TAB;
      Serial.print(digitalRead(HW_PIN_DRIVE_SERVO_HALT)); TAB;
      Serial.print(digitalRead(HW_PIN_TURRET_SERVO_HALT));  NEWLINE;
      break;

    case 3:     // Drive odometry
      Serial.print(F("3-Odom  X,Y,Head,Sin,Cos  "));
      Serial.print(drive_pos_x, 2);     TAB;
      Serial.print(drive_pos_y, 2);     TAB;
      Serial.print(drive_pos_heading * (180 / M_PI), 2);  TAB;
      Serial.print(drive_pos_sin, 4);     TAB;
      Serial.print(drive_pos_cos, 4);     NEWLINE;
      break;

    case 4:     // Drive wheel data
      Serial.print(F("4-Wheel Speed_L,R,Ticks_L,R "));
      Serial.print(drive_speed_L);      TAB;
      Serial.print(drive_speed_R);      TAB;
      Serial.print(drive_tickcount_l);    TAB;
      Serial.print(drive_tickcount_r);    NEWLINE;
      break;

    case 5:     // Nav target data
      if (diag_data.nav_new_target) {
        if (nav_state == NAV_STATE_XY) {
          Serial.print(F("5-Targ  State,Speed,X,Y,Tol "));
          Serial.print(nav_state);      TAB;
          Serial.print(nav_target_speed);     TAB;
          Serial.print(nav_target_x, 2);      TAB;
          Serial.print(nav_target_y, 2);      TAB;
          Serial.print(nav_target_tol, 2);    NEWLINE;
        } else if (nav_state == NAV_STATE_TURN_L || nav_state == NAV_STATE_TURN_R) {
          Serial.print(F("5-Targ  State,Speed,Turn,Ratio  "));
          Serial.print(nav_state);      TAB;
          Serial.print(nav_target_speed);     TAB;
          Serial.print(nav_target_turn, 2);   TAB;
          Serial.print(nav_target_ratio, 2);    NEWLINE;
        } else {
          Serial.print(F("5-Nav State...........  "));
          Serial.print(nav_state);      NEWLINE;
        }
        diag_data.nav_new_target = FALSE;
      } else {
        if (nav_state == NAV_STATE_XY) {
          Serial.print(F("5-Nav State,Dist,Head,PID "));
          Serial.print(nav_state);      TAB;
          Serial.print(nav_target_distance, 2);   TAB;
          Serial.print(nav_target_heading * (180 / M_PI), 2); TAB;
          Serial.print(diag_data.nav_PIDterm, 2);   NEWLINE;
        } else if (nav_state == NAV_STATE_TURN_L || nav_state == NAV_STATE_TURN_R) {
          Serial.print(F("5-Nav State,Tickdiff..  "));
          Serial.print(nav_state);      TAB;
          Serial.print(drive_tickcount_r - drive_tickcount_l - diag_data.targ_tickdiff);  NEWLINE;
        } else {
          Serial.print(F("5-Nav State...........  "));
          Serial.print(nav_state);      NEWLINE;
        }
      }
      break;

    case 6:     // Turret & Ping
      Serial.print(F("6-Ping  Dist[3],Angle[3]  "));
      Serial.print(ping_dist[0]);     TAB;
      Serial.print(ping_dist[1]);     TAB;
      Serial.print(ping_dist[2]);     TAB;
      Serial.print(ping_angle[0]);      TAB;
      Serial.print(ping_angle[1]);      TAB;
      Serial.print(ping_angle[2]);      NEWLINE;
      break;

    case 7:     // IRRX
      Serial.print(F("7-IRRX  ................  "));
      Serial.print("TBD");        NEWLINE;
      break;

    case 8:     // FSM
      Serial.print(F("8-FSM State,..........  "));
      Serial.print(fsm_state);      NEWLINE;
      break;

    case 9:     // CPU time measurements
      for (i = 0; i < 10; i++) {
        Serial.print(diag_time[i]);   TAB;
      }
      NEWLINE;
      break;

    case 10:      // Bot Plot telemetry
      Serial.print(drive_pos_x, 2);     COMMA;
      Serial.print(drive_pos_y, 2);     COMMA;
      Serial.print(drive_pos_heading * (180 / M_PI), 2);  COMMA;
      Serial.print(nav_target_x, 2);      COMMA;
      Serial.print(nav_target_y, 2);      COMMA;
      Serial.print(drive_speed_L);      COMMA;
      Serial.print(drive_speed_R);      NEWLINE;
      break;
  }
}  /* Diag_Output */

/******************************************************************************
 ******************************************************************************
   CTRL
    Main software control
    Includes initialization, base loop, and interrupt service routine
 ******************************************************************************
 ******************************************************************************/
//  Functions
//    setup
//    loop
//    Ctrl_ISR_Setup
//    Ctrl_ISR_50usec

#define CTRL_LOOP_PERIOD  10000 // Base level loop period in microseconds
#define CTRL_ISR_PERIOD   50  // ISR period in microseconds

/******************************************************************************/
// setup
//  Initialize various data and configuration and calibrates wheel sensors

void setup()
{
  Serial.begin(115200);

  pinMode(HW_PIN_USER2,     INPUT_PULLUP);
  pinMode(HW_PIN_USER3,     INPUT_PULLUP);
  pinMode(HW_PIN_DRIVE_SERVO_HALT,  INPUT_PULLUP);
  pinMode(HW_PIN_TURRET_SERVO_HALT, INPUT_PULLUP);

  hw_drive_servo_L.attach (HW_PIN_DRIVE_SERVO_L);
  hw_drive_servo_R.attach (HW_PIN_DRIVE_SERVO_R);
  hw_turret_servo.attach  (HW_PIN_TURRET_SERVO);
  delay(100);

  Turret_Set_Angle (0);
  Ping_Setup ();
  Irrx_Setup ();
  Ctrl_ISR_Setup ();

  Drive_Calibrate_Wheels ();
  Drive_Reset_Odometry ();

  turret_state = TURRET_STATE_IDLE; // TURRET_STATE_ [IDLE, SCANNING]
  nav_route_auto = FALSE;     // [FALSE, TRUE]
  fsm_state = FSM_STATE_START;    // FSM_STATE_ [IDLE, START]

  Turret_Set_Angle (-10);   // "Shake head" when done with initialization
  delay(100);
  Turret_Set_Angle (10);
  delay(100);
  Turret_Set_Angle (0);
  delay(2000);

  if (nav_route_auto) {   // Set initial target if auto route mode
    Nav_Next_Target ();
  }

}  /* setup */

/******************************************************************************/
// loop
//  This is the base level loop within the program.
//  It is designed to run 100 iterations per second (period = 10,000 usec)

void loop()
{
  static uint32_t loop_counter = 0;
  static uint32_t start_time = 0;

  /*** BEGIN TIMING ***/  diag_time[10] = micros();

  if (start_time == 0)
    start_time = micros();  // Initialize on first iteration

  Drive_Do_Odometry ();     // DRIVE functionality:  poll encoders and perform odometry
  Nav_Do_Controller (NAV_CONTROLLER_RUN); // NAV functionality:  Run Destination / Heading controller

  if (nav_state == NAV_STATE_ARRIVED && nav_route_auto) {
    Nav_Next_Target ();
    if (nav_state == NAV_STATE_IDLE)  // No more targets in route table
      Drive_Set_Speed (0, 0);
  }

  switch (loop_counter % 10) {    // Spread tasks across iterations of loop()
    case 0:       // Send ping
      Ping_Send ();
      break;

    case 1:       // Spare
      break;

    case 2:       // Process IR received
      Irrx_Results ();
      break;

    case 3:       // Process ping response
      Ping_Results ();
      break;

    case 4:       // Run state machine
      Fsm_Run ();
      break;

    case 5:       // Update turret
      Turret_Scan ();
      break;

    case 6:       // Spare
      break;

    case 7:       // Diagnostic I/O
      Diag_Input ();
      if (diag_output_frequency <= 10)
        Diag_Output (diag_output_frequency * 10);
      break;

    case 8:       // 1 Hz tasks
      switch ((loop_counter / 10) % 10) {
        case 0:
          break;
      }  /* 1 Hz task switch */
      break;

    case 9:       // Spare
      break;

  }  /* 10 Hz task switch */

  if (diag_output_frequency > 10)
    Diag_Output (diag_output_frequency);

  loop_counter++;

  /*** END TIMING ***/  diag_time[loop_counter % 10] = micros() - diag_time[10];

  // Manage loop() iteration speed

  uint32_t  now = micros();

  diag_data.loop_count = loop_counter;
  diag_data.loop_time_max = max(diag_data.loop_time_max, (now - start_time));

  start_time += CTRL_LOOP_PERIOD; // Compute when this loop should end & next begin
  if (now >= start_time) {
    start_time = now; // loop took too long already, so return
    return;
  }

  while (micros() < start_time) {
    // Do stuff or kill time
  }
}  /* loop */

/******************************************************************************/
// Ctrl_ISR_Setup
//  Configures the ISR for detecting Ping echo and IRRX.
//  -- Configures hardware Timer 2 to generate periodic interrupts
//  -- Registers Ctrl_ISR_50usec() as the associated interrupt handler

void Ctrl_ISR_Setup () {
  // Based on New Ping functions timer_us(), timer_setup(), and timer_stop()

  TIMSK2 &= ~(1 << OCIE2A); // Disable Timer2 interrupt.
  ASSR &= ~(1 << AS2); // Set clock, not pin.
  TCCR2A = (1 << WGM21); // Set Timer2 to CTC mode.
  TCCR2B = (1 << CS22); // Set Timer2 prescaler to 64 (4uS/count, 4uS-1020uS range).
  TCNT2 = 0;    // Reset Timer2 counter.
  OCR2A = min((CTRL_ISR_PERIOD >> 2) - 1, 255); // Every count is 4uS, so divide by 4 (bitwise shift right 2) subtract one, then make sure we don't go over 255 limit.
  TIMSK2 |= (1 << OCIE2A); // Enable Timer2 interrupt.
}  /* Ctrl_ISR_Setup */

ISR(TIMER2_COMPA_vect) {  // This registers the ISR.  Note:  This is a compiler directive, not a C function.
  Ctrl_ISR_50usec();
}

/******************************************************************************/
// Ctrl_ISR_50usec
//  Timer2 interrupt calls this function every 50 microseconds to process
//  Ping echo detection and IRRX code reception
//  Interrupts are allowed so Timer 1 ISR (servo pulse timing) can run

void Ctrl_ISR_50usec() {

  diag_data.isr_count++;
  sei();      // Allow interrupts
  Ping_Echo_Check (); // Check for ping echo.  50 usec interval yields 0.33 inch accuracy
  Irrx_Code_Check (); // Check for infrared code

}  /* Ctrl_ISR_50usec */

/******************************************************************************/
