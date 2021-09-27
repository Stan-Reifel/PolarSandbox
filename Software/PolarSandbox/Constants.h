//      ******************************************************************
//      *                                                                *
//      *                      Constant declarations                     *
//      *                                                                *
//      *               Copyright (c) S. Reifel & Co,  2019              *
//      *                                                                *
//      ******************************************************************


#ifndef Constants_h
#define Constants_h

//
// IO pin definitions
//
const int LED_PIN               = 31;
const int MOTOR_RADIUS_STEP_PIN = 27;     // MOTOR A
const int MOTOR_RADIUS_DIR_PIN  = 26;     // MOTOR A
const int MOTOR_RADIUS_HOME_PIN = 3;      // HOME A
const int MOTOR_THETA_STEP_PIN  = 25;     // MOTOR B
const int MOTOR_THETA_DIR_PIN   = 24;     // MOTOR B
const int MOTOR_THETA_HOME_PIN  = 2;      // HOME B
const int MOTOR_ENABLE_PIN      = 4;
const int MOTOR_MICROSTEP_0_PIN = 5;
const int MOTOR_MICROSTEP_1_PIN = 6;
const int MOTOR_MICROSTEP_2_PIN = 7;
const int LED_LIGHTING_PIN      = 23;
const int TEST_1_PIN            = 29;


//
// assigment of motor numbers
//
const int RADUIS_MOTOR = 0;
const int THETA_MOTOR = 1;


//
// motion constants
//
const int NUMBER_OF_MOTORS = 2;
const float MICRO_STEPPING = 16.0;
const float THETA_STEPS_PER_REVOLUTION = 200 * MICRO_STEPPING * 7.5;
const float THETA_STEPS_PER_DEGREE = THETA_STEPS_PER_REVOLUTION / 360.0;
const float RADIUS_STEPS_PER_REVOLUTION_OF_THETA = 200 * MICRO_STEPPING;
const float RADIUS_STEPS_PER_DEGREE_OF_THETA = RADIUS_STEPS_PER_REVOLUTION_OF_THETA / 360.0;
const float RADIUS_PINION_PITCH_CIRCUMFERENCE_MM = 42.40;
const float RADIUS_STEPS_PER_MM = (200.0 * MICRO_STEPPING) / RADIUS_PINION_PITCH_CIRCUMFERENCE_MM;
const float MAX_THETA_MOTOR_STEPS_PER_SECOND = THETA_STEPS_PER_REVOLUTION * 0.4;
const float MAX_RADIUS_MOTOR_STEPS_PER_SECOND = RADIUS_STEPS_PER_MM * 50;


//
// physical constraints of the plotter
//
//const float SANDBOX_MIN_RADIUS_MM = 6.0;  // mechanically it can go in to 6mm, but magnet over pinion
const float SANDBOX_MIN_RADIUS_MM = 7.0;
const float SANDBOX_MAX_RADIUS_MM = 278.0;


//
// direction constants for the spirals
//
const boolean SPIRAL_IN = false;
const boolean SPIRAL_OUT = true;


//
// starting position for radial drawings
//
const boolean START_IN_END_IN = false;
const boolean START_OUT_END_OUT = true;


//
// values returned from drawing functions
//
const int CONTINUE_DRAWING = 0;
const int CANCEL_DRAWING = 1;


//
// storage locations in EEPROM for configuration values settable below
//
const int EEPROM_LED_BACKLIGHT_BRIGHTNESS = 0;                          // byte requires 2 bytes of EEPROM storage
const int EEPROM_BALL_SPEED = EEPROM_LED_BACKLIGHT_BRIGHTNESS + 2;      // int requires 5 bytes of EEPROM storage
const int EEPROM_RUNNING_TIME = EEPROM_BALL_SPEED + 5;                  // int requires 5 bytes of EEPROM storage                  
const int EEPROM_nextValue = EEPROM_RUNNING_TIME + 5;                     


// ------------------------------------ End ---------------------------------
#endif
