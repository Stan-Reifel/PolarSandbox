//      ******************************************************************
//      *                                                                *
//      *   Stepper Driver for 2, 3 or 4 Axes with Coordinated Motion    *
//      *                                                                *
//      *            Stan Reifel                     6/19/2019           *
//      *                                                                *
//      ******************************************************************


// To do and clean up:
//    * Remove the dump diag functions
//    * Clean up TMC2208 vs DRV8825 direction pin usage
//    * Add stuff to make homing possible


//
// This library in its entirety was developed and written by the top copyright
// author in 2019.  However in that process, inspiration, insight, technique and 
// math were taken from GRBL, an open source software package for controlling 
// CNC machines.  The original version of GRBL was developed by Simen Svale 
// Skogsrud.  As a result, his copyright has been included here.
//
// MIT License:
// Copyright (c) 2019 Stanley Reifel & Co.
// Copyright (c) 2009-2011 Simen Svale Skogsrud
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is furnished
// to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


//
// This module is used to drive 2, 3, or 4 stepper motors in coordination.  The main
// application calls this module with a series of coordinates that form a path of 
// motion.  The units of these coordinates are in Steps.  Along with each coordinate,  
// the application specifies a "Desired Speed" (with units in Steps/Second).  This driver  
// will attempt to accelerate up to that speed as it moves between each coordinate in  
// the series.  For long paths it will usually reach the Desired Speed, but there are  
// several reasons that can cause it to move slower: 1) When starting a new path, it will 
// begin at a speed of zero, then accelerate up.  2) When reaching the end of a path,  
// it will automatically slow down as it comes to a stop.  3) When a path changes 
// direction it will slow down, a little bit for small turns, much more for sharp
// turns.
//
// Here we name the coordinates given by the main application "Waypoints".  This 
// module stores the path of Waypoints in a Ring Buffer.  It also maintains a list 
// of points called "ISR Segments" in separate Ring Buffer.  An ISR Segment is similar 
// to a Waypoint in that it's used to build a sequence of moves for the stepper motors. 
// Both contain how many steps to move and at what speed.  But they differ in the other 
// data they contain.  A Waypoint includes information necessary for planning speeds, 
// while an ISR Segment has values needed when actually stepping the motors. 
//
// The flow of data is as follows:  The main application generates a sequence of path 
// points, calling addWaypoint() with each new one.  This creates a list of Waypoints.
// A side effect of calling addWaypoint() is that each time the entire list of points  
// is examined and their speeds are updated, slowing down or speeding up as needed
// when changing directions. When the waypoint buffer becomes full, the oldest entry 
// is then transferred to the list of ISR Segments.  When this transfer is made, new 
// data is added to the segment that will be needed by the Stepper's Interrupt Service 
// Routine.
//
// It is the Interrupt Service Routine that actually steps the motors.  It gets the
// first segment to move to from the isrSegments buffer.  Then using this data, steps
// each motor at the correct rate so they all arrive at the given coordinate together.
// When the segment's motion is complete, the ISR deletes that segment, then loads 
// and executes the next one.  
//
// To coordinate the stepping of each motor, the Bresenham Line Algorithm is used.  
// For an explanation of this algorithm, see: 
//   http://graphics.idav.ucdavis.edu/education/GraphicsNotes/Bresenhams-Algorithm.pdf
//

//
// Notes: 
// 1) Step rates over 100,000 steps/second for all axis are possible
//

#include "TeensyCoordinatedStepper.h"


//
// forward declarations
//
void stepperISR(void);
void initializeISRForNewSegment(void);
void fallingEdgeISR(void);


// ---------------------------------------------------------------------------------
//                             Datatypes and Structures 
// ---------------------------------------------------------------------------------

//
// Define a waypoint
//
// Note: Distances values are in steps.  All speeds are for the dominate motor
// and are in units of steps/second. (The dominate motor is the one that travels 
// farthest to this waypoint from the last one)
//
typedef struct {
  //
  // Array of how far each motor must go to this waypoint from the last one.
  // Signed values with units in steps.
  //
  int32_t motorSteps[MAX_NUMBER_OF_MOTORS];

  //
  // The number of steps the dominate motor must go to this waypoint from the last 
  // one. Unsigned with units in steps.
  //
  uint32_t dominateMotorSteps;

  //
  // The speed that was specified for traveling to this waypoint.  The actual speed 
  // can be lower, but not higher.  It will be lower when accelerating from a start,
  // decelerating to a stop, or when changing directions.  Units in steps/second.
  //
  float desiredSpeed;

  //
  // The actual speed used at the instant when starting the move to this waypoint.
  // It is the exact same speed used when exiting the previous waypoint.  This value
  // is squared with units in steps/second.
  //
  float initialSpeed_Sqr;

  //
  // This value is used to help compute the "initialSpeed" above.  It represents the
  // fastest speed that it can have.  Value squared with units in steps/second.
  //
  float maxInitialSpeed_Sqr;
} waypoint_t;



//
// Define an ISR Segment
//
// Note: Distances values are in steps.  All speeds are for the dominate motor
// and are in units of steps/second.
//
typedef struct {
  //
  // Array of how many steps each motor must go in this segment, unsigned values 
  // with units in steps
  //
  int32_t stepsToMove[MAX_NUMBER_OF_MOTORS];

  //
  // Array indicating the direction each motor should go: 1 CW, -1 CCW
  //
  int32_t directionScaler[MAX_NUMBER_OF_MOTORS];

  //
  // The number of steps the dominate motor must go to this waypoint from the last 
  // one. Unsigned with units in steps.
  //
  uint32_t dominateMotorSteps;

  //
  // Period in US for the first step of the segment
  //
  float initialStepPeriod_InUS;

  //
  // While executing the velocity profile: the fastest step rate to reach during acceleration
  //
  float fastestStepPeriodWhileAccelerating_InUS;

  //
  // While executing the velocity profile: the slowest step rate to reach during deceleration
  //
  float slowestStepPeriodWhileDecelerating_InUS;

  //
  // While executing the velocity profile: accelerate until this step number is reached
  //
  uint32_t accelerateUntil_InSteps;

  //
  // While executing the velocity profile: begin to decelerate after this step number is reached
  //
  uint32_t decelerateAfter_InSteps;

} isrSegment_t;


//
// conditional compilation constants
//
const boolean INCLUDE_DIAGNOSTIC_FUNCTIONS = false;


// ---------------------------------------------------------------------------------
//             Globals & constants only available inside this library 
// ---------------------------------------------------------------------------------


//
// number of motors (axes) (2 - 4)
//
uint8_t numberOfMotors;


//
// step and direction pin numbers for each motor
//
uint8_t stepPin[MAX_NUMBER_OF_MOTORS];
uint8_t directionPin[MAX_NUMBER_OF_MOTORS];


//
// flag for each motor indicating if its axis direction should be reversed
//
boolean reverseMotorDirection[MAX_NUMBER_OF_MOTORS];


//
// current position in absolute coordinates of the motors, this value updates while in motion, units in steps
//
volatile int32_t currentCoordinate_InSteps[MAX_NUMBER_OF_MOTORS];


//
// position in absolute coordinates the last waypoint added
//
int32_t lastWaypointCoordinate_InSteps[MAX_NUMBER_OF_MOTORS];


//
// Waypoints are stored in a Ring Buffer.  An entry contains the number of steps needed to move
// each motor to get to that point, along with the desired speed the Dominate motor (the one 
// that the farthest) should step.  The other motors will steps at a speed so they arrive at the 
// given point simultaneously with the dominate motor.
//
waypoint_t waypoints[MAX_NUMBER_OF_WAYPOINTS];
int firstWaypointIdx;             
int lastWaypointIdx;  


//
// ISR Segments are stored in a Ring Buffer.  An entry contains the number of steps needed to move
// each motor to get to that point.
//
isrSegment_t isrSegments[MAX_NUMBER_OF_ISR_SEGMENTS];
volatile int8_t firstIsrSegmentIdx;             
volatile int8_t lastIsrSegmentIdx;  


//
// the acceleration rate used when speeding up, or slowing down, in steps/second/second
//
float accelerationRate; 


//
// the acceleration rate used when speeding up, or slowing down, in steps/US/US
//
float acceleration_InStepsPerUSPerUS;


//
// This constant defines how fast the motors will move when changing directions.  Small
// changes in direction will need just a small reduction in speed, while sharp turns
// will require a large reduction in speed. For example if two waypoints are at a right 
// angle to each other, the combined motion of all the motors must slow down to make the 
// turn.  Effectively this is sets the cornering speed.  The larger the value, the faster
// the corner speed.  Units are in steps.
//
float junctionDeviation; 


//
// The unit vector for the last waypoint in the waypoint buffer.  It represents the direction
// of travel to that waypoint 
//
float buffersLastWaypoint_UnitVector[MAX_NUMBER_OF_MOTORS]; 


//
// The desired speed squared, of the last waypoint in the waypoint buffer.
//
float buffersLastWaypoint_DesiredSpeed_Sqr; 


//
// Initial speed squared for the first waypoint in the buffer.  This is effectively the final
// speed of the waypoint that came before, what is now the first waypoint.  If there are no 
// waypoints in the buffer, then this value is set to 0, indicating that the series of   
// waypoints should start moving from a stop.
//
float buffersFirstWaypoint_InitialSpeed_Sqr;


//
// this flag indicates if the stepper ISR is running
//
volatile boolean stepperIsrRunningFlg;


//
// the step period used when stopping or starting
//
float stepPeriodfromStop_InUS;


//
// the slowest allowed step rate
//
const float slowestSpeed_InStepsPerSecond = 0.01;


// ---------------------------------------------------------------------------------
//                                 Setup functions 
// ---------------------------------------------------------------------------------


//
// constructor for the stepper class
//
TeensyCoordinatedStepper::TeensyCoordinatedStepper()
{
}



//
// configure the driver for 2 motors and assign step & direction pin numbers
//  Enter:  motor0_StepPin          = IO pin number for motor 0 step
//          motor0_DirectionPin     = IO pin number for motor 0 direction
//          motor0_ReverseDirection = true to reverse motor 0's direction
//          motor1_StepPin          = IO pin number for motor 1 step
//          motor1_DirectionPin     = IO pin number for motor 1 direction
//          motor1_ReverseDirection = true to reverse motor 1's direction
//
void TeensyCoordinatedStepper::connectToPins(
  byte motor0_StepPin, byte motor0_DirectionPin, boolean motor0_ReverseDirection, 
  byte motor1_StepPin, byte motor1_DirectionPin, boolean motor1_ReverseDirection)
{
  numberOfMotors = 2;
  
  stepPin[0] = motor0_StepPin;
  directionPin[0] = motor0_DirectionPin;
  reverseMotorDirection[0] = motor0_ReverseDirection;
  
  stepPin[1] = motor1_StepPin;
  directionPin[1] = motor1_DirectionPin;
  reverseMotorDirection[1] = motor1_ReverseDirection;

  initializePins();
  initialize();
}



//
// configure the driver for 3 motors and assign step & direction pin numbers
//  Enter:  motor0_StepPin          = IO pin number for motor 0 step
//          motor0_DirectionPin     = IO pin number for motor 0 direction
//          motor0_ReverseDirection = true to reverse motor 0's direction
//          motor1_StepPin          = IO pin number for motor 1 step
//          motor1_DirectionPin     = IO pin number for motor 1 direction
//          motor1_ReverseDirection = true to reverse motor 1's direction
//          motor2_StepPin          = IO pin number for motor 2 step
//          motor2_DirectionPin     = IO pin number for motor 2 direction
//          motor2_ReverseDirection = true to reverse motor 2's direction
//
void TeensyCoordinatedStepper::connectToPins(
  byte motor0_StepPin, byte motor0_DirectionPin, boolean motor0_ReverseDirection, 
  byte motor1_StepPin, byte motor1_DirectionPin, boolean motor1_ReverseDirection, 
  byte motor2_StepPin, byte motor2_DirectionPin, boolean motor2_ReverseDirection)
{
  numberOfMotors = 3;
  
  stepPin[0] = motor0_StepPin;
  directionPin[0] = motor0_DirectionPin;
  reverseMotorDirection[0] = motor0_ReverseDirection;

  stepPin[1] = motor1_StepPin;
  directionPin[1] = motor1_DirectionPin;
  reverseMotorDirection[1] = motor1_ReverseDirection;

  stepPin[2] = motor2_StepPin;
  directionPin[2] = motor2_DirectionPin;
  reverseMotorDirection[2] = motor2_ReverseDirection;

  initializePins();
  initialize();
}



//
// configure the driver for 4 motors and assign step & direction pin numbers
//  Enter:  motor0_StepPin          = IO pin number for motor 0 step
//          motor0_DirectionPin     = IO pin number for motor 0 direction
//          motor0_ReverseDirection = true to reverse motor 0's direction
//          motor1_StepPin          = IO pin number for motor 1 step
//          motor1_DirectionPin     = IO pin number for motor 1 direction
//          motor1_ReverseDirection = true to reverse motor 1's direction
//          motor2_StepPin          = IO pin number for motor 2 step
//          motor2_DirectionPin     = IO pin number for motor 2 direction
//          motor2_ReverseDirection = true to reverse motor 2's direction
//          motor3_StepPin          = IO pin number for motor 3 step
//          motor3_DirectionPin     = IO pin number for motor 3 direction
//          motor3_ReverseDirection = true to reverse motor 3's direction
//
void TeensyCoordinatedStepper::connectToPins(
  byte motor0_StepPin, byte motor0_DirectionPin, boolean motor0_ReverseDirection, 
  byte motor1_StepPin, byte motor1_DirectionPin, boolean motor1_ReverseDirection, 
  byte motor2_StepPin, byte motor2_DirectionPin, boolean motor2_ReverseDirection,
  byte motor3_StepPin, byte motor3_DirectionPin, boolean motor3_ReverseDirection)
{
  numberOfMotors = 4;
  
  stepPin[0] = motor0_StepPin;
  directionPin[0] = motor0_DirectionPin;
  reverseMotorDirection[0] = motor0_ReverseDirection;

  stepPin[1] = motor1_StepPin;
  directionPin[1] = motor1_DirectionPin;
  reverseMotorDirection[1] = motor1_ReverseDirection;

  stepPin[2] = motor2_StepPin;
  directionPin[2] = motor2_DirectionPin;
  reverseMotorDirection[2] = motor2_ReverseDirection;

  stepPin[3] = motor3_StepPin;
  directionPin[3] = motor3_DirectionPin;
  reverseMotorDirection[3] = motor3_ReverseDirection;

  initializePins();
  initialize();
}



//
// initialize the pins used for step and direction
//
void TeensyCoordinatedStepper::initializePins(void)
{
  for (int motorNumber = 0; motorNumber < numberOfMotors; motorNumber++)
  {
    pinMode(stepPin[motorNumber], OUTPUT);
    digitalWrite(stepPin[motorNumber], LOW);
    
    pinMode(directionPin[motorNumber], OUTPUT);
    digitalWrite(directionPin[motorNumber], LOW);
  }
}


//
// initialize this driver
//
void TeensyCoordinatedStepper::initialize(void)
{
  stepperIsrRunningFlg = false;
  junctionDeviation = 1.0;                // bigger the number, the faster it corners 
  firstWaypointIdx = 0;             
  lastWaypointIdx = 0;  
  firstIsrSegmentIdx = 0;             
  lastIsrSegmentIdx = 0;  
  setAcceleration(100.0);

  //
  // set initial values for each motor
  //
  for(int motorNumber = 0; motorNumber < numberOfMotors; motorNumber++)
  {
    //
    // set the starting position of each motor at 0
    //
    currentCoordinate_InSteps[motorNumber] = 0;
    lastWaypointCoordinate_InSteps[motorNumber] = 0;
  }
}



//
// Set the acceleration rate in steps / second / second.  This value can only be changed 
// when all motion is stopped.
//
void TeensyCoordinatedStepper::setAcceleration(float _accelerationRate)
{
  //
  // ignore this function call if the motors are currently running
  //
  if (stepperIsrRunningFlg)
    return;
    
  accelerationRate = _accelerationRate;
  acceleration_InStepsPerUSPerUS = accelerationRate / 1E12;
  stepPeriodfromStop_InUS = 1000000.0 / sqrt(2.0 * accelerationRate);
}



//
// get the acceleration rate
//  Exit:   acceleration rate returned with units in steps / second / second
//
float TeensyCoordinatedStepper::getAcceleration(void)
{    
  return(accelerationRate);
}



//
// The "Junction Deviation" defines how much the motors will slow down when changing 
// direction.  Small changes in direction typically need only a small reduction in speed, 
// while sharp turns require a much larger reduction in speed.  For example, if the path 
// between two waypoints are at a right angle to each other, the combined motion of all 
// the motors must slow down significately to make this sharp turn.  Setting the
// Junction Deviation constant controls how much the motors will slow down as they 
// head into, and out of turns.  Effectively this sets the cornering speed.  But
// this does not "round" corners, in that it will always complete all steps to reach 
// the exact waypoint coordinate.  The default value is set around 1.0.  The larger
// the value, the less it slows down when changing direction.  When moving a large
// mass, or with a very ridge system, a smaller number may be needed.  If the number
// is too large, it may cause a loss of steps as sharp turns are made.  Units are in 
// steps. This value can only be changed when all motion is stopped.
//
void TeensyCoordinatedStepper::setJunctionDeviation(float _junctionDeviation)
{
  //
  // ignore this function call if the motors are currently running
  //
  if (stepperIsrRunningFlg)
    return;
    
  junctionDeviation = _junctionDeviation;
}



//
// get the "Junction Deviation"
//  Exit:   returnedJunction Deviation
//
float TeensyCoordinatedStepper::getJunctionDeviation(void)
{    
  return(junctionDeviation);
}


// ---------------------------------------------------------------------------------
//                          Stop motion in progress functions 
// ---------------------------------------------------------------------------------


//
// Stop all motors and flush the waypoint and segment buffers.  This function can
// be used as an emergency stop or in homing procedures.  After calling this function, 
// getCurrentCoordinate() can be used to read the motor positions at the time this
// function was executed.  Notes: This function does not decelerate to a stop, it 
// simply terminates stepping.  It's possible that inertia causes the motor's motions
// to continue beyond the final value returned by getCurrentCoordinate().
//
void TeensyCoordinatedStepper::emergencyStop(void)
{
  //
  // disable the ISR and flush all coordinate buffers
  //
  noInterrupts();
  stepperIsrRunningFlg = false;
  firstWaypointIdx = 0;             
  lastWaypointIdx = 0;  
  firstIsrSegmentIdx = 0;             
  lastIsrSegmentIdx = 0;  

  //
  // set the value for the "Last Waypoint Set" to the current position.
  //
  for(int motorNumber = 0; motorNumber < numberOfMotors; motorNumber++)
    lastWaypointCoordinate_InSteps[motorNumber] = currentCoordinate_InSteps[motorNumber] ;  

  interrupts();
}



// ---------------------------------------------------------------------------------
//                                 Waypoint functions 
// ---------------------------------------------------------------------------------

//
// Add a new waypoint to the buffer of waypoints.  Initially the stepper ISR will be stopped.
// If adding this waypoints causes the buffers to be full, the ISR is automatically started
// (the motors begin turning).  If the path is short and the buffer never fill, the ISR is
// started after the first call to finishMovingToFinalWaypoints().
//
// Note: If this function is called and the buffers are already full, it will wait until there 
// is room to add this waypoint.
//
//  Enter:  coordinate -> waypoint coordinate to move to, stored as an array of absolute motor
//            position values, each value is signed with units in steps, the array should be 
//            dimensioned equal the number of motors
//          desiredSpeed = the desired speed for the motor that moves the farthest, "desired" 
//            because it may not be possible to accelerate up to this speed, units steps/second
//
void TeensyCoordinatedStepper::addWaypoint(int32_t *coordinate, float desiredSpeed)
{
  waypoint_t *thisWaypoint;
  int32_t stepsSigned;
  uint32_t stepsUnsigned;
  float euclideanStepCount = 0;
  float thisWaypoint_UnitVector[MAX_NUMBER_OF_MOTORS];
  float maxInitialSpeed_Sqr;
  float desiredSpeed_Sqr;


  //
  // check if the waypoint buffer is full, if so wait until it's not full
  //
  while(isWaypointsBufferFull())
    sendWaypointsToSteppers();

  
  //
  // get pointer to where this new waypoint will be stored in the waypoint buffer
  //
  thisWaypoint = &waypoints[lastWaypointIdx];


  //
  // save the desired speed to the waypoint
  //
  if (desiredSpeed < slowestSpeed_InStepsPerSecond)
    desiredSpeed = slowestSpeed_InStepsPerSecond;
    
  thisWaypoint->desiredSpeed = desiredSpeed;


  //
  // for each motor, set how far it moves to this waypoint
  //
  thisWaypoint->dominateMotorSteps = 0;
  for(int motorNumber = 0; motorNumber < numberOfMotors; motorNumber++)
  {
    //
    // convert absolute steps for this motor to relative steps
    //
    stepsSigned = coordinate[motorNumber] - lastWaypointCoordinate_InSteps[motorNumber];
    thisWaypoint->motorSteps[motorNumber] = stepsSigned;
    lastWaypointCoordinate_InSteps[motorNumber] = coordinate[motorNumber];

    //
    // determine if this is the dominate motor (one that moves the farthest)
    //
    if (stepsSigned < 0) 
      stepsUnsigned = -stepsSigned;
    else
      stepsUnsigned = stepsSigned;

    if (thisWaypoint->dominateMotorSteps < stepsUnsigned)
       thisWaypoint->dominateMotorSteps = stepsUnsigned;

    //
    // incrementally build a Unit Vector that represents the direction to travel 
    // to this waypoint
    //
    thisWaypoint_UnitVector[motorNumber] = stepsSigned;

    //
    // incrementally determine the Euclidean distance from motions of all motors to this 
    // new point
    //
    euclideanStepCount += (stepsSigned * stepsSigned);
  }


  //
  // don't add this waypoint if none of the motors move any distance
  //
  if (thisWaypoint->dominateMotorSteps == 0)
    return;


  //
  // finish computation to find Euclidean distance from all motors to this new point
  //
  euclideanStepCount = sqrt(euclideanStepCount); 


  //
  // finish building the Unit Vector that represents the direction to travel to this waypoint
  //
  for(int motorNumber = 0; motorNumber < numberOfMotors; motorNumber++)
    thisWaypoint_UnitVector[motorNumber] = thisWaypoint_UnitVector[motorNumber] / euclideanStepCount;

  //
  // check if this waypoint is the first one added to the buffer
  //
  if (waypointsCount() == 0)
  {
    //
    // waypoint buffer is empty, so this waypoint must start from a stopped position, 
    // so the initial speed will be zero
    //
    buffersFirstWaypoint_InitialSpeed_Sqr = 0.0;
    maxInitialSpeed_Sqr = 0.0;
  }

  else
  {
    //
    // There are 1 or more waypoints already in the buffer, indicating that this waypoint is 
    // continuing the motion from previous waypoints.  Here we compute how much the motors
    // must slow down as a result of a change in direction from the previous waypoint.
    //
    // Compute the maximum speed the dominate motor can turn, as it begins moving to this new 
    // waypoint.  We do this with some magic math:
    //
    // Let a circle be tangent to both previous and current waypoint segments, where the 
    // "Junction Deviation" is defined as the distance from the junction to the closest edge of 
    // the circle, collinear with the circle center.  The circular segment joining the two paths 
    // represents the path of centripetal acceleration. Solve for max velocity based on max  
    // acceleration about the radius of the circle, defined indirectly by Junction Deviation.  
    // This approach does not actually deviate from path, but used as a robust way to compute  
    // cornering speeds, as it takes into account the nonlinearities of both the junction 
    // angle and junction velocity.
    //

    //
    // compute the cosine of angle between the path taken to previous waypoint, and the path to 
    // this one;  the cosine of this angle is determined by taking the Dot Product of their unit 
    // vectors
    //
    float cosineOfAngleBetweenThisWaypointAndPreviousWaypoint = 0;
    for(int motorNumber = 0; motorNumber < numberOfMotors; motorNumber++)
    {
      cosineOfAngleBetweenThisWaypointAndPreviousWaypoint -= 
        thisWaypoint_UnitVector[motorNumber] * buffersLastWaypoint_UnitVector[motorNumber];
    }


    //
    // if cosine between waypoints is +1, that indicates this waypoint is traveling in the
    // exact opposite directions of the last one;  here we will need to Stop as we move between them
    //
    if (cosineOfAngleBetweenThisWaypointAndPreviousWaypoint > 0.999999) 
    {
      //
      //  For a 0 degree acute junction, just set minimum junction speed. 
      //
      maxInitialSpeed_Sqr = 0.0;
    }

    else
    {
      //
      // make sure the cosine of the angle is not less than -1 due to rounding errors
      //
      if (cosineOfAngleBetweenThisWaypointAndPreviousWaypoint < -0.999999)
        cosineOfAngleBetweenThisWaypointAndPreviousWaypoint = -0.999999;

      //
      // use the Trig Half Angle Formula: sin(theta / 2) = +/- sqrt((1 - cos(theta)) / 2)
      //
      float sineOfHalfAngleBetweenThisWaypointAndPreviousWaypoint = 
        sqrt((1.0 - cosineOfAngleBetweenThisWaypointAndPreviousWaypoint) / 2.0);


      //
      // use magic math to determine the fastest we can go as a result of changing direction
      //
      maxInitialSpeed_Sqr = 
        (accelerationRate * junctionDeviation * sineOfHalfAngleBetweenThisWaypointAndPreviousWaypoint) / 
        (1.0 - sineOfHalfAngleBetweenThisWaypointAndPreviousWaypoint);

      if (maxInitialSpeed_Sqr < 0.0)
        maxInitialSpeed_Sqr = 0.0;
    } 
  }


  //
  // set the waypoint's maximum initial speed to the minimum value of: this waypoint's desired speed,
  // the previous waypoint's desired speed, and the "max speed from change in direction"
  //
  desiredSpeed_Sqr = desiredSpeed * desiredSpeed;

  if (maxInitialSpeed_Sqr > desiredSpeed_Sqr)
    maxInitialSpeed_Sqr = desiredSpeed_Sqr;

  if (maxInitialSpeed_Sqr > buffersLastWaypoint_DesiredSpeed_Sqr)
    maxInitialSpeed_Sqr = buffersLastWaypoint_DesiredSpeed_Sqr;

  thisWaypoint->maxInitialSpeed_Sqr = maxInitialSpeed_Sqr;
  thisWaypoint->initialSpeed_Sqr = maxInitialSpeed_Sqr;         // this value is updated later in the code

  //
  // this waypoint is fully entered into the waypoint buffer, advance the buffer's pointer
  // 
  lastWaypointIdx = nextWaypointIdx(lastWaypointIdx);

  //
  // this waypoint's Unit Vector now becomes the buffer's last Unit Vector
  //
  for(int motorNumber = 0; motorNumber < numberOfMotors; motorNumber++)
     buffersLastWaypoint_UnitVector[motorNumber] = thisWaypoint_UnitVector[motorNumber];

  //
  // this waypoint's desired speed now becomes the buffer's last desired speed
  //
  buffersLastWaypoint_DesiredSpeed_Sqr = desiredSpeed_Sqr;

  //
  // adjust all the waypoint's initial speeds based on adding this waypoint
  //
  setWaypointsInitialSpeeds();

  //
  // check if the stepper ISR is not running yet, if so consider transferring a waypoint 
  // to the segments buffer
  //
  if (stepperIsrRunningFlg != true)
  {
    //
    // if the ISR is not running and the waypoint buffer is now full, transfer
    // the oldest waypoint to the isrSegements buffer
    //
    if (isWaypointsBufferFull())
    {
       //
       // full: transfer one waypoint to the isrSegments buffer, then start the ISR 
       // if the segments buffer is full too
       //
       transferOneWaypointToIsrSegment();
       if (isIsrSegmentsBufferFull())
         startStepperISR();
    }
  }
  else
  {
    //
    // ISR is running, transfer as many waypoints into the isrSegments buffer as possible
    //
    sendWaypointsToSteppers();
  }
}



//
// After addWaypoint() has been called with the last coordinate, this function must be
// called continously until the motion has completed (until stopped at the last waypoint).
// The longest period between calls to this function should not exceed the time to execute
// 3 or 4 waypoints.
//    Exit:   true returned if motion is complete
//
boolean TeensyCoordinatedStepper::finishMovingToFinalWaypoints(void)
{
  //
  // determine the current state of moving
  //
  int status = getMotionStatus();

  //
  // check if the motion is now complete
  //
  if (status == MOTION_STATUS_DONE)
    return(true);

  //
  // check if the motion has not started yet
  //
  if (status == MOTION_STATUS_WAITING_FOR_MORE_POINTS)
  {
    //
    // start the ISR and motors moving
    //
    sendWaypointsToSteppers();
    startStepperISR();
    return(false);
  }

  //
  // otherwise we are moving, keep ISR segment buffer full by transferring waypoints to it
  //
  sendWaypointsToSteppers();
  return(false);
}


    
// 
// Set the initialSpeed for each waypoint in the waypoint buffer.  
//  
// When a waypoint is added to the buffer, a desired speed is specified.  Here we will attempt 
// to use that speed while moving to the waypoint.  However it may not be possible due to 
// acceleration limits.  In determining the waypoint's initialSpeed we must consider: 1) How
// fast it can accelerate from previous waypoint's speeds. 2) How fast it can decelerate 
// to a stop in future waypoints. 3) How fast it can change directions.
//  
// When the waypoint was first added, we determined the maximum value for its initialSpeed.  
// Here we will look at each waypoints in the waypoint buffer and adjust it's initialSpeed for
// reasons of acceleration/deceleration. First we will look at them in reverse order, starting 
// from the most recently added waypoint.  In this pass, we may lower the initialSpeed to insure
// it can decelerate down to the next waypoint's initialSpeed.
//  
// Note: When consider the waypoint that was added last, we will not know the speed of the
// waypoint after it, so we will assume that it must come to a stop.  As a result, a speed of
// zero is used for its "next waypoint's initialSpeed".
// 
// When the reverse pass is complete, the initialSpeed of each waypoint will be adjusted again.
// This time we will scan through the waypoint buffer in the forward direction (from the 
// earliest added to the last added waypoint. Here we look at how they can accelerate in speed 
// over time. From previous scans, we have already set the initialSpeed in the buffer's very  
// first waypoint.
//
// Note: On a Teensy3.6 at 180MHz, with a buffer length of 50 waypoints, this function takes 16us
//  
void TeensyCoordinatedStepper::setWaypointsInitialSpeeds(void)
{
  int thisWaypointIdx;
  int finalWaypointIdx;
  waypoint_t *thisWaypoint;
  waypoint_t *nextWaypoint;
  float thisWaypoint_InitialSpeed_Sqr;
  float thisWaypoint_FinalSpeed_Sqr;
  float nextWaypoint_InitialSpeed_Sqr;
  float initialSpeed_Sqr;


  //
  // need at least 2 waypoints
  //
  if (waypointsCount() < 2)
    return;


  //
  // scan in the reverse order, starting with the last waypoint (the most recently added), 
  // assume it may need to come to a stop
  //
  finalWaypointIdx = previousWaypointIdx(lastWaypointIdx);                        // start with most recent added waypoint
  thisWaypointIdx = finalWaypointIdx;
  nextWaypoint_InitialSpeed_Sqr = 0.0;


  //
  // run through the buffer of waypoints in the reverse order, limiting their speeds so they can
  //  decelerate to a stop at the last waypoint
  //
  while(thisWaypointIdx != firstWaypointIdx)
  {
    //
    // Compute the fastest speed to this waypoint, such that we can decelerate to the next waypoint's initialSpeed.
    // Note 1: The to compute the kinematics: V^2 = Vinitial^2 + (2 * A * DeltaDistance)
    // Note 2: To make the math faster, we are avoiding using the sqrt() function by keeping speeds values squared
    //
    thisWaypoint = &waypoints[thisWaypointIdx];

    initialSpeed_Sqr = nextWaypoint_InitialSpeed_Sqr + (2.0 * accelerationRate * thisWaypoint->dominateMotorSteps);
    
    if (initialSpeed_Sqr > thisWaypoint->maxInitialSpeed_Sqr)
      initialSpeed_Sqr = thisWaypoint->maxInitialSpeed_Sqr;

    thisWaypoint->initialSpeed_Sqr = initialSpeed_Sqr;

   //
   // select the previous waypoint
   //
   thisWaypointIdx = previousWaypointIdx(thisWaypointIdx);
   nextWaypoint_InitialSpeed_Sqr = initialSpeed_Sqr;
  }


  //
  // we have previously determine the initialSpeed of the very first waypoint in the buffer
  //
  thisWaypoint_InitialSpeed_Sqr = buffersFirstWaypoint_InitialSpeed_Sqr;


  //
  // now run through the waypoints in the forward order, limiting their speeds by how fast they
  // can accelerate from the first waypoint
  //
  thisWaypointIdx = firstWaypointIdx;
  while(true)
  {
    //
    // get pointers to this and the next waypoint
    //
    thisWaypoint = &waypoints[thisWaypointIdx];
    nextWaypoint = &waypoints[nextWaypointIdx(thisWaypointIdx)];


    //
    // set this waypoint's initial speed
    //
    thisWaypoint->initialSpeed_Sqr = thisWaypoint_InitialSpeed_Sqr;

    //
    // check if there's no more waypoints after this one
    //
    if (thisWaypointIdx == finalWaypointIdx)
      break;

    //
    // Compute this waypoint's final speed by how much it can accelerate up while traveling to the waypoint
    // Note 1: The to compute the kinematics: V^2 = Vinitial^2 + (2 * A * DeltaDistance)
    // Note 2: To make the math faster, we are avoiding using the sqrt() function by keeping speeds values squared
    //
    thisWaypoint_FinalSpeed_Sqr = (thisWaypoint_InitialSpeed_Sqr) + (2.0 * accelerationRate * thisWaypoint->dominateMotorSteps);

    if (thisWaypoint_FinalSpeed_Sqr > nextWaypoint->initialSpeed_Sqr)
      thisWaypoint_FinalSpeed_Sqr = nextWaypoint->initialSpeed_Sqr;

    //
    // select the next waypoint
    //
    thisWaypointIdx = nextWaypointIdx(thisWaypointIdx);
    thisWaypoint_InitialSpeed_Sqr = thisWaypoint_FinalSpeed_Sqr;
  }
}



//
// remove the first waypoint in the waypoint buffer
//
void TeensyCoordinatedStepper::removeWaypoint(void)
{
  if (waypointsCount() == 0)
    return;

  firstWaypointIdx = nextWaypointIdx(firstWaypointIdx);

  buffersFirstWaypoint_InitialSpeed_Sqr = waypoints[firstWaypointIdx].initialSpeed_Sqr; 
}



//
// get the current motion status
//  Exit:   MOTION_STATUS_DONE returned if motors are stopped & no waypoints in the buffers
//          MOTION_STATUS_WAITING_FOR_MORE_POINTS returned if motors are currently stopped and
//            waiting for more waypoints before starting the motors
//          MOTION_STATUS_RUNNING returned if the motors are in motion
//
int TeensyCoordinatedStepper::getMotionStatus(void)
{
  if (stepperIsrRunningFlg)
    return(MOTION_STATUS_RUNNING);

  if ((waypointsCount() > 0) || (isrSegmentsCount() > 0))
    return(MOTION_STATUS_WAITING_FOR_MORE_POINTS);

  return(MOTION_STATUS_DONE);
}



//
// get the current position in absolute coordinates of the motors, this value updates 
// while in motion, it is signed with units in steps
//  Enter:  currentCoordinate -> an array that is filled in here with the current position
//            of each motor, the array should be dimensioned to equal the number of 
//            motors, the values are signed with units in steps and are updated while
//            moving
//
void TeensyCoordinatedStepper::getCurrentCoordinate(int32_t *currentCoordinate)
{
  noInterrupts();
  for(int motorNumber = 0; motorNumber < numberOfMotors; motorNumber++)
    currentCoordinate[motorNumber] = currentCoordinate_InSteps[motorNumber];
  interrupts();
}



//
// set the current position in absolute coordinates of the motors, this function can only
// be called when the motors are stopped, values are signed with units in steps
//  Enter:  newCoordinate -> an array filled with the positions to set each motor to, 
//            the array should be dimensioned to equal the number of motors, the values 
//            are signed with units in steps
//
void TeensyCoordinatedStepper::setCurrentCoordinate(int32_t *newCoordinate)
{
  //
  // ignore this function call if the motors are currently running
  //
  if (stepperIsrRunningFlg)
    return;

  for(int motorNumber = 0; motorNumber < numberOfMotors; motorNumber++)
  {
    currentCoordinate_InSteps[motorNumber] = newCoordinate[motorNumber];
    lastWaypointCoordinate_InSteps[motorNumber] = newCoordinate[motorNumber];
  }
}



//
// given an index into the waypoint buffer, find the next waypoint index
//
int TeensyCoordinatedStepper::nextWaypointIdx(int waypointIdx)
{
  waypointIdx++;
  if (waypointIdx == MAX_NUMBER_OF_WAYPOINTS)
    waypointIdx = 0;

  return(waypointIdx);
}



//
// given an index into the waypoint buffer, find the previous waypoint index
//
int TeensyCoordinatedStepper::previousWaypointIdx(int waypointIdx)
{
  if (waypointIdx == 0)
    waypointIdx = MAX_NUMBER_OF_WAYPOINTS;

  waypointIdx--;
  return(waypointIdx);
}



//
// check if the waypoints buffer is full
//
boolean TeensyCoordinatedStepper::isWaypointsBufferFull(void)
{
  if (waypointsCount() == MAX_NUMBER_OF_WAYPOINTS-1)
    return(true);
    
  return(false);
}



//
// return the number of waypoints in the waypoint buffer
//
int TeensyCoordinatedStepper::waypointsCount(void)
{
  int count = lastWaypointIdx - firstWaypointIdx;
  if (count < 0)
    count += MAX_NUMBER_OF_WAYPOINTS;
  return(count);
}



// ---------------------------------------------------------------------------------
//                       Stepper ISR and supporting functions 
// ---------------------------------------------------------------------------------

//
// interval timer objects for running the stepper ISR at precise times
//
IntervalTimer stepTimer;
IntervalTimer fallingEdgeTimer;

//
// static vars used by the ISR
//
isrSegment_t *currentIsrSegment;
uint32_t dominateMotorSteps;
int32_t nextStepAccumulator[MAX_NUMBER_OF_MOTORS];
uint32_t remainingStepsCounter;
uint32_t stepsCompletedThisSegment;
float nextStepPeriod_InUS;


// ---------------------------------------------------------------------------------

//
// start the stepper's ISR, this will begin stepping the motors and continue until the 
// isrSegements buffer is empty
//
void  TeensyCoordinatedStepper::startStepperISR(void)
{
  stepperIsrRunningFlg = true;
  initializeISRForNewSegment();
}



//
// transfer as many waypoints as possible to the isrSegments buffer
//
void TeensyCoordinatedStepper::sendWaypointsToSteppers(void)
{
  while(!isIsrSegmentsBufferFull() && (waypointsCount() > 0))
    transferOneWaypointToIsrSegment();
}



//
// transfer 1 Waypoint from the waypoints buffer to an ISR Segment in the isrSegments buffer
//
void TeensyCoordinatedStepper::transferOneWaypointToIsrSegment(void)
{
  waypoint_t *thisWaypoint;
  isrSegment_t *thisIsrSegment;
  float finalSpeed_InStepsPerSec;
  float initialSpeed_InStepsPerSec;
  uint32_t deceleration_InSteps;
  float finalStepPeriod_InUS;

  //
  // get point for the waypoint being transferred, and storage into the isrSegments buffer
  //
  thisWaypoint = &waypoints[firstWaypointIdx];
  thisIsrSegment = &isrSegments[lastIsrSegmentIdx];

  //
  // copy how far each motor goes from the waypoint to the ISR Segment
  //
  for(int motorNumber = 0; motorNumber < numberOfMotors; motorNumber++)
  {
    int32_t steps = thisWaypoint->motorSteps[motorNumber];
    if (steps >= 0)
    {
      thisIsrSegment->stepsToMove[motorNumber] = steps;
      thisIsrSegment->directionScaler[motorNumber] = 1;
    }
    else
    {
      thisIsrSegment->stepsToMove[motorNumber] = -steps;
      thisIsrSegment->directionScaler[motorNumber] = -1;
    }
  }

  //
  // set how many steps, the motor that moves the farthest will go
  //
  thisIsrSegment->dominateMotorSteps = thisWaypoint->dominateMotorSteps;

  //
  // set the initial speeds for this segment in US/step, don't divide by zero
  //
  initialSpeed_InStepsPerSec = sqrt(thisWaypoint->initialSpeed_Sqr);

  if (initialSpeed_InStepsPerSec < slowestSpeed_InStepsPerSecond)
    thisIsrSegment->initialStepPeriod_InUS = stepPeriodfromStop_InUS;
  else
    thisIsrSegment->initialStepPeriod_InUS = 1000000.0 / initialSpeed_InStepsPerSec;

  //
  // set final speeds for this segment in US/step, set speed close to zero if this is 
  // final waypoint
  //
  if(waypointsCount() == 1)                           // is this the last waypoint?
    finalSpeed_InStepsPerSec = 0;
  else
    finalSpeed_InStepsPerSec = sqrt(waypoints[nextWaypointIdx(firstWaypointIdx)].initialSpeed_Sqr);

  if (finalSpeed_InStepsPerSec < slowestSpeed_InStepsPerSecond)
    finalStepPeriod_InUS = stepPeriodfromStop_InUS;
  else
    finalStepPeriod_InUS = 1000000.0 / finalSpeed_InStepsPerSec;

    
  //
  // There are 7 velocity profiles that can occur while traveling to an endpoint:
  //    1) Constant acceleration
  //    2) Accelerate to Desired Speed, then constant velocity
  //    3) Accelerate to Desired Speed, then constant velocity, then decelerate (trapezoidal profile)
  //    4) Accelerate, then decelerate (triangular profile)
  //    5) Constant velocity, then decelerate
  //    6) Constant deceleration
  //    7) Constant velocity only
  // Next, determine which profile, then setup the ISR for that profile
  //
  uint32_t distance_InSteps = thisWaypoint->dominateMotorSteps;
  float desiredSpeed_InSteps_Sqr = thisWaypoint->desiredSpeed * thisWaypoint->desiredSpeed;
  float initialSpeed_InStepsPerSec_Sqr = initialSpeed_InStepsPerSec * initialSpeed_InStepsPerSec;
  float finalSpeed_InStepsPerSec_Sqr = finalSpeed_InStepsPerSec * finalSpeed_InStepsPerSec;

  float intersectDistance_InSteps = 0.5 * ((float) distance_InSteps + 
    (initialSpeed_InStepsPerSec_Sqr - finalSpeed_InStepsPerSec_Sqr) / (2.0 * accelerationRate));

  if (intersectDistance_InSteps > 0.01)
  {
    if (intersectDistance_InSteps < distance_InSteps)
    {
      deceleration_InSteps = (desiredSpeed_InSteps_Sqr - finalSpeed_InStepsPerSec_Sqr) / (2.0 * accelerationRate);
      if (deceleration_InSteps < intersectDistance_InSteps) 
      {
        if (initialSpeed_InStepsPerSec == thisWaypoint->desiredSpeed)
        {
          //
          // Profile: Constant velocity only   or 
          //          Constant velocity, then decelerate
          //
          thisIsrSegment->accelerateUntil_InSteps = 0L;
          thisIsrSegment->decelerateAfter_InSteps = distance_InSteps - deceleration_InSteps;
          thisIsrSegment->fastestStepPeriodWhileAccelerating_InUS = thisIsrSegment->initialStepPeriod_InUS;
          thisIsrSegment->slowestStepPeriodWhileDecelerating_InUS = finalStepPeriod_InUS;
        }
        
        else
        {
          //
          // Profile: Accelerate to Desired Speed, then constant velocity, then decelerate (trapezoidal)   or
          //          Accelerate to Desired Speed, then constant velocity
          //
          thisIsrSegment->accelerateUntil_InSteps = distance_InSteps - deceleration_InSteps;
          thisIsrSegment->decelerateAfter_InSteps = distance_InSteps - deceleration_InSteps;
          thisIsrSegment->fastestStepPeriodWhileAccelerating_InUS = 1000000.0 / thisWaypoint->desiredSpeed;
          thisIsrSegment->slowestStepPeriodWhileDecelerating_InUS = finalStepPeriod_InUS;
        }
      }

      else
      {
        //
        // Profile: Accelerate, then decelerate (triangular)
        //
        uint32_t accelerateUntil_InSteps = distance_InSteps - (uint32_t) intersectDistance_InSteps;
        float maxSpeed_InStepsPerSecond = 
          sqrt((2.0 * accelerationRate * intersectDistance_InSteps) + finalSpeed_InStepsPerSec_Sqr);

        thisIsrSegment->accelerateUntil_InSteps = accelerateUntil_InSteps;
        thisIsrSegment->decelerateAfter_InSteps = accelerateUntil_InSteps;        
        thisIsrSegment->fastestStepPeriodWhileAccelerating_InUS = 1000000.0 / maxSpeed_InStepsPerSecond;
        thisIsrSegment->slowestStepPeriodWhileDecelerating_InUS = finalStepPeriod_InUS;
      }
    }

    else
    {
      //
      // Profile: Constant deceleration
      //
      thisIsrSegment->accelerateUntil_InSteps = 0L;
      thisIsrSegment->decelerateAfter_InSteps = 0L;
      thisIsrSegment->fastestStepPeriodWhileAccelerating_InUS = thisIsrSegment->initialStepPeriod_InUS;
      thisIsrSegment->slowestStepPeriodWhileDecelerating_InUS = finalStepPeriod_InUS;
    }
  }

  else
  {
    //
    // Profile: Constant acceleration
    //
    thisIsrSegment->accelerateUntil_InSteps = distance_InSteps;
    thisIsrSegment->decelerateAfter_InSteps = distance_InSteps;
    thisIsrSegment->fastestStepPeriodWhileAccelerating_InUS = 1000000.0 / thisWaypoint->desiredSpeed;
    thisIsrSegment->slowestStepPeriodWhileDecelerating_InUS = finalStepPeriod_InUS;
  }


  //
  // now this ISR Segment is fully entered into the buffer, advance the buffer's pointer
  // 
  lastIsrSegmentIdx = nextIsrSegmentIdx(lastIsrSegmentIdx);

  //
  // remove this waypoint from its buffer
  //
  removeWaypoint();
}



//
// given an index into the isrSegments buffer, find the next isrSegment index
//
int TeensyCoordinatedStepper::nextIsrSegmentIdx(int isrSegmentIdx)
{
  isrSegmentIdx++;
  if (isrSegmentIdx == MAX_NUMBER_OF_ISR_SEGMENTS)
    isrSegmentIdx = 0;

  return(isrSegmentIdx);
}



//
// given an index into the isrSegments buffer, find the previous isrSegment index
//
int TeensyCoordinatedStepper::previousIsrSegmentIdx(int isrSegmentIdx)
{
  if (isrSegmentIdx == 0)
    isrSegmentIdx = MAX_NUMBER_OF_ISR_SEGMENTS;

  isrSegmentIdx--;
  return(isrSegmentIdx);
}



//
// check if the isrSegments buffer is full
//
boolean TeensyCoordinatedStepper::isIsrSegmentsBufferFull(void)
{
  int8_t count;

  noInterrupts();
  count = lastIsrSegmentIdx - firstIsrSegmentIdx;
  interrupts();
  
  if (count < 0)
    count += MAX_NUMBER_OF_ISR_SEGMENTS;

  if (count == MAX_NUMBER_OF_ISR_SEGMENTS-1)
    return(true);
  
  return(false);
}



//
// return the number of isrSegments in the isrSegments buffer
//
int TeensyCoordinatedStepper::isrSegmentsCount(void)
{
  int8_t count;

  noInterrupts();
  count = lastIsrSegmentIdx - firstIsrSegmentIdx;
  interrupts();
  
  if (count < 0)
    count += MAX_NUMBER_OF_ISR_SEGMENTS;
  return((int) count);
}



//
// stepper ISR: this function is called at precise time intervals to step the motors
//
// Note: On a Teensy3.6 at 180MHz, this ISR takes between 2.5us and 5us
//
void stepperISR(void)
{
  //
  // return if the ISR is not supposed to be running
  //
  if (!stepperIsrRunningFlg)
    return;
  
  //
  // for each motor, check if it is time to step
  //
  for(uint8_t motorNumber = 0; motorNumber < numberOfMotors; motorNumber++)
  {
    //
    // using the Bresenhams algorithm, check if on this cycle, this motor moves 
    //
    nextStepAccumulator[motorNumber] += currentIsrSegment->stepsToMove[motorNumber];  // advance this axis step accumulator
    if (nextStepAccumulator[motorNumber] > (int32_t) dominateMotorSteps)              // check if accumulated enough to move
    {
      digitalWrite(stepPin[motorNumber], HIGH);                                       // step this axis on the rising edge
      nextStepAccumulator[motorNumber] -= dominateMotorSteps;                         // update this axis accumulator
                                                                                      // update the absolute position of this motor
      currentCoordinate_InSteps[motorNumber] += currentIsrSegment->directionScaler[motorNumber];
    }
  }

  //
  // determine the period for the next step based on if we are accelerating, decelerating
  // or going at a constant velocity
  //
  // The math used to calculate the "step next period" is based Aryeh Elderman's paper 
  // "Real Time Stepper Motor Linear Ramping Just By Addition and Multiplication".  It
  // approximates the value, rather than computing the exact value using square roots. 
  // See:  www.hwml.com/LeibRamp.pdf
  //
  if (stepsCompletedThisSegment <= currentIsrSegment->accelerateUntil_InSteps)
  {
    nextStepPeriod_InUS = nextStepPeriod_InUS * 
      (1.0 - acceleration_InStepsPerUSPerUS * nextStepPeriod_InUS * nextStepPeriod_InUS);

    if (nextStepPeriod_InUS < currentIsrSegment->fastestStepPeriodWhileAccelerating_InUS)
      nextStepPeriod_InUS = currentIsrSegment->fastestStepPeriodWhileAccelerating_InUS;
  }
  
  else if (stepsCompletedThisSegment > currentIsrSegment->decelerateAfter_InSteps)
  {
    nextStepPeriod_InUS = nextStepPeriod_InUS * 
      (1.0 + acceleration_InStepsPerUSPerUS * nextStepPeriod_InUS * nextStepPeriod_InUS);
    
    if (nextStepPeriod_InUS > currentIsrSegment->slowestStepPeriodWhileDecelerating_InUS)
      nextStepPeriod_InUS = currentIsrSegment->slowestStepPeriodWhileDecelerating_InUS;
  }

  //
  // reset the ISR time for the next step period
  //
  stepTimer.begin(stepperISR, nextStepPeriod_InUS);


  //
  // check if this segment has reached its final position
  //
  stepsCompletedThisSegment++;
  remainingStepsCounter--;
  if (remainingStepsCounter == 0)
  {
    //
    // this segment is now complete, remove it from the segments buffer
    //
    firstIsrSegmentIdx++;
    if (firstIsrSegmentIdx == MAX_NUMBER_OF_ISR_SEGMENTS)
      firstIsrSegmentIdx = 0;

    //
    // initialize to run the next segment
    //
    initializeISRForNewSegment();
  }


  //
  // start a timer that will cause the step pins to return to LOW
  //
  fallingEdgeTimer.begin(fallingEdgeISR, 2);
}



//
// starting a new segment, pre-compute as much as possible, this is called from the ISR
//
void initializeISRForNewSegment(void)
{
  //
  // check if there are no segments if the buffer
  //
  if (lastIsrSegmentIdx == firstIsrSegmentIdx)
  {
    //
    // turn off the interrupt timer
    //
    stepTimer.end();
    stepperIsrRunningFlg = false;
    return;
  }

  
  //
  // get a pointer to the current segment for the ISR to use
  //
  currentIsrSegment = &isrSegments[firstIsrSegmentIdx];

  //
  // initialize Bresenhams algorithm counters
  //
  dominateMotorSteps = currentIsrSegment->dominateMotorSteps;
  remainingStepsCounter = dominateMotorSteps;
  uint32_t halfSteps = (dominateMotorSteps >> 1) + 1;
  stepsCompletedThisSegment = 0;
  
  //
  // setup each variable for each motor
  //
  for(uint8_t motorNumber = 0; motorNumber < numberOfMotors; motorNumber++)
  {
    //
    // set the stepper driver's direction pin for each motor, using an XOR operation
    //
    if ((currentIsrSegment->directionScaler[motorNumber] >= 0) != (reverseMotorDirection[motorNumber]))
       digitalWrite(directionPin[motorNumber], LOW);
    else
       digitalWrite(directionPin[motorNumber], HIGH);


    //
    // initialize the Bresenhams algorithm step accumulators to the half a step point 
    //
    nextStepAccumulator[motorNumber] = halfSteps;
  }

  //
  // set the period for the first step and start the timer
  //
  nextStepPeriod_InUS = currentIsrSegment->initialStepPeriod_InUS;
  stepTimer.begin(stepperISR, nextStepPeriod_InUS);
}



//
// stepper pins falling edge ISR: this function drives the Step pins back low after the stepperISR 
// set them high
//
void fallingEdgeISR(void)
{
  //
  // turn off this ISR so it does not repeat
  //
  fallingEdgeTimer.end();

  //
  // for each motor, set the step pin low
  //
  for(uint8_t motorNumber = 0; motorNumber < numberOfMotors; motorNumber++)
    digitalWrite(stepPin[motorNumber], LOW);
}


// ---------------------------------------------------------------------------------
//          Diagnostics functions, not to be included in library version
// ---------------------------------------------------------------------------------


#if INCLUDE_DIAGNOSTIC_FUNCTIONS

void TeensyCoordinatedStepper::dumpWaypointBuffer(void)
{
  int thisWaypointIdx;
  waypoint_t *waypoint;

  Serial.print("WayIndex #,");
  Serial.print("Mot 0,");
  Serial.print("Mot 1,");
  Serial.print("DomSteps,");
  Serial.print("DesireSpd,");
  Serial.print("MaxInitSpd,");
  Serial.print("InitialSpd,");
  Serial.println("");
  
  thisWaypointIdx = firstWaypointIdx;
  while(thisWaypointIdx != lastWaypointIdx)
  {
    waypoint = &waypoints[thisWaypointIdx];

    Serial.print(thisWaypointIdx);                  Serial.print(",");
    Serial.print(waypoint->motorSteps[0]);          Serial.print(",");
    Serial.print(waypoint->motorSteps[1]);          Serial.print(",");
    Serial.print(waypoint->dominateMotorSteps);     Serial.print(",");
    Serial.print(waypoint->desiredSpeed);           Serial.print(",");
    Serial.print(waypoint->maxInitialSpeed_Sqr);    Serial.print(",");
    Serial.print(waypoint->initialSpeed_Sqr);       Serial.print(",");
    Serial.println("");

    thisWaypointIdx = nextWaypointIdx(thisWaypointIdx);
  }
}



void TeensyCoordinatedStepper::dumpIsrSegmentsBuffer(void)
{
  int thisIsrSegmentIdx;
  isrSegment_t *isrSegment;

  Serial.print("SegIndex #,");
  Serial.print("Mot 0,");
  Serial.print("Mot 1,");
  Serial.print("InitialUS,");
  Serial.print("FastestUS,");
  Serial.print("SlowestUS,");
  Serial.print("AccUntil,");
  Serial.print("DecAfter,");
  Serial.println("");
  
  thisIsrSegmentIdx = firstIsrSegmentIdx;
  while(thisIsrSegmentIdx != lastIsrSegmentIdx)
  {
    isrSegment = &isrSegments[thisIsrSegmentIdx];

    Serial.print(thisIsrSegmentIdx);                                     Serial.print(",");
    Serial.print(isrSegment->stepsToMove[0]);                            Serial.print(",");
    Serial.print(isrSegment->stepsToMove[1]);                            Serial.print(",");
    Serial.print(isrSegment->initialStepPeriod_InUS);                    Serial.print(",");
    Serial.print(isrSegment->fastestStepPeriodWhileAccelerating_InUS);   Serial.print(",");
    Serial.print(isrSegment->slowestStepPeriodWhileDecelerating_InUS);   Serial.print(",");
    Serial.print(isrSegment->accelerateUntil_InSteps);                   Serial.print(",");
    Serial.print(isrSegment->decelerateAfter_InSteps);                   Serial.print(",");
    Serial.println("");

    thisIsrSegmentIdx = nextIsrSegmentIdx(thisIsrSegmentIdx);
  }
}



void TeensyCoordinatedStepper::dumpMotionStatus(void)
{
  switch(getMotionStatus())
  {
    case MOTION_STATUS_DONE:
      Serial.println("Motion done.");
      break;
    case MOTION_STATUS_WAITING_FOR_MORE_POINTS:
      Serial.println("Motion waiting for more points.");
      break;
    case MOTION_STATUS_RUNNING:
      Serial.println("Motion running.");
      break;
  }
}

#endif


// -------------------------------------- End --------------------------------------
