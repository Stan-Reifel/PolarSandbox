//      ******************************************************************
//      *                                                                *
//      *                        Support Functions                       *
//      *                                                                *
//      *               Copyright (c) S. Reifel & Co,  2019              *
//      *                                                                *
//      ******************************************************************

#include <arduino.h>
#include "Constants.h"
#include "SettingsToolsCmd.h"
#include "SupportFuncs.h"
#include <EEPROM.h>
#include <TeensyUserInterface.h>
#include <font_Arial.h>
#include <font_ArialBold.h>
#include "TeensyCoordinatedStepper.h"


//
// external references from the main application
//
extern TeensyUserInterface ui;
extern TeensyCoordinatedStepper steppers;
extern int ballSpeedMMperSec;


//
// forward function declarations
//
int displayPausedScreen(void);
void moveUntilHomeSensorSignal(float moveToR, float moveToTheta, float speedStepsPerSecond, boolean waitForThisSignal);
void moveRelativeForHoming(float moveToR, float moveToTheta, float speedStepsPerSecond);

//
// constants
//
const float SPEED_SCALER_FOR_INITAL_MOVE = 0.75;


// ---------------------------------------------------------------------------------
//                                  Plotting Functions
// ---------------------------------------------------------------------------------

//
//
// plot a polar function with increasing theta, solving for R
//    Exit:   CONTINUE_DRAWING returned normally
//            CANCEL_DRAWING returned if user has cancelled the remaining drawing
//
int plotPolarFuncWithIncreasingTheta(float (*equation_PlotPolarFuncWithIncreasingTheta)(float),
         float  beginAtAbsoluteTheta, float startingTheta, float endingTheta, float pointSpacingMM, 
         float pointSpacingTolerance, boolean clipPlotToSandboxRadiusFlag,
         float *finalAbsoluteTheta)
{
  float theta;
  float thetaDelta;
  float newTheta;
  float rLast;
  float thetaDeltaTooLow;
  float thetaDeltaTooHigh;
  float r;
  float rDeltaMM;
  float arcMM;
  float segmentLengthSqr;
  float segmentLengthMaxSqr;
  float segmentLengthMinSqr;
  int loopCounter;
  float thetaOffsetForStartingAngle;
  int results = CONTINUE_DRAWING;


  //
  // initialize loops
  //
  segmentLengthMaxSqr = pointSpacingMM + (pointSpacingMM * pointSpacingTolerance);
  segmentLengthMaxSqr *= segmentLengthMaxSqr;
  
  segmentLengthMinSqr = pointSpacingMM - (pointSpacingMM * pointSpacingTolerance);
  segmentLengthMinSqr *= segmentLengthMinSqr;
  
  theta = startingTheta;
  loopCounter = 0;
  thetaDelta = 0.1;
  thetaOffsetForStartingAngle = startingTheta - beginAtAbsoluteTheta;


  //
  // set the starting coordinate, then move there
  //
  r = (*equation_PlotPolarFuncWithIncreasingTheta)(theta);  
  if ((clipPlotToSandboxRadiusFlag) && (r > SANDBOX_MAX_RADIUS_MM)) 
    r = SANDBOX_MAX_RADIUS_MM;
  rLast = r;

  results = addPolarWaypoint(r, theta - thetaOffsetForStartingAngle, (float) ballSpeedMMperSec * SPEED_SCALER_FOR_INITAL_MOVE);
  if (results == CANCEL_DRAWING)
    return(CANCEL_DRAWING);


  //
  // loop to plot each line segment until the outer side of the sandbox is reached
  //
  while(true)
  {
    //
    // loop to find the next point to plot
    //
    thetaDeltaTooLow = -1;
    thetaDeltaTooHigh = -1;
    
    while(true)
    {
      r = (*equation_PlotPolarFuncWithIncreasingTheta)(theta + thetaDelta);
  
      if ((clipPlotToSandboxRadiusFlag) && (r > SANDBOX_MAX_RADIUS_MM)) 
        r = SANDBOX_MAX_RADIUS_MM;
 
      rDeltaMM = r - rLast;
      arcMM = (2 * 3.14159 * r) * (thetaDelta / 360);
      segmentLengthSqr = (rDeltaMM * rDeltaMM) + (arcMM * arcMM);
      loopCounter++;
    
      if (segmentLengthSqr < segmentLengthMinSqr) 
      {
        thetaDeltaTooLow = thetaDelta;
        if (thetaDeltaTooHigh == -1)
            thetaDelta = thetaDelta * 2;
        else
            thetaDelta = (thetaDeltaTooLow + thetaDeltaTooHigh) / 2;
            
        continue;
      }
  
      if(segmentLengthSqr > segmentLengthMaxSqr) 
      {
        thetaDeltaTooHigh = thetaDelta;
        if(thetaDeltaTooLow == -1)
            thetaDelta = thetaDelta / 2;
        else
            thetaDelta = (thetaDeltaTooLow + thetaDeltaTooHigh) / 2;

        continue;
      }
      
      break;
    }


    //
    // found the next point, check if at the ending value of theta
    //
    newTheta = theta + thetaDelta;
    if (newTheta > endingTheta) 
      break;

    //
    // check if R beyond the sandbox radius
    //
    if (r > SANDBOX_MAX_RADIUS_MM) 
      break;

    //
    // check if R is less than zero
    //
    if (r < 0) 
      break;


    //
    // plot the point
    //
    theta = newTheta;
    rLast = r;
    results = addPolarWaypoint(r, theta - thetaOffsetForStartingAngle, (float) ballSpeedMMperSec);
    if (results == CANCEL_DRAWING)
      break;

    plottingDisplay_PlotPoint(r, theta - thetaOffsetForStartingAngle);
  }


  //
  // wait for the motion to finish, then normalize theta
  //
  while(!steppers.finishMovingToFinalWaypoints());        // wait for the motion to finish
  normalizeCurrentThetaAngle();


  //
  // return the absolute theta for the ending position
  //
  *finalAbsoluteTheta = normalizeAngle(theta - thetaOffsetForStartingAngle);
  return(results);
}



//
// plot a polar function with increasing theta, solving for X & Y (for drawing spirographs)
//    Exit:   CONTINUE_DRAWING returned normally
//            CANCEL_DRAWING returned if user has cancelled the remaining drawing
//
int plotPolarXYFuncWithIncreasingTheta(void(*equation_PlotPolarXYFuncWithIncreasingTheta)(float, float*, float*),
         float  beginAtAbsoluteTheta, float startingTheta, float endingTheta, float pointSpacingMM, 
         float pointSpacingTolerance, float *finalAbsoluteTheta)
{
  float theta;
  float thetaDelta;
  float thetaDeltaTooLow;
  float thetaDeltaTooHigh;
  float x, y;
  float xLast, yLast;
  float segmentLengthSqr;
  float segmentLengthMaxSqr;
  float segmentLengthMinSqr;
  int loopCounter;
  float moveToR;
  float polarAngle;
  float lastPolarAngle;
  float moveToTheta;
  float rotationCount;
  int results = CONTINUE_DRAWING;


  //
  // initialize loops
  //
  segmentLengthMaxSqr = pointSpacingMM + (pointSpacingMM * pointSpacingTolerance);
  segmentLengthMaxSqr *= segmentLengthMaxSqr;
  
  segmentLengthMinSqr = pointSpacingMM - (pointSpacingMM * pointSpacingTolerance);
  segmentLengthMinSqr *= segmentLengthMinSqr;
  
  theta = startingTheta;
  loopCounter = 0;
  thetaDelta = 0.1;
  rotationCount = 0;


  //
  // set the starting coordinate, then move there
  //
  (*equation_PlotPolarXYFuncWithIncreasingTheta)(theta, &x, &y);  
 
  moveToR = sqrt(x*x + y*y);
  polarAngle = Atan2D(x, y);
  lastPolarAngle = polarAngle;
  moveToTheta = polarAngle + beginAtAbsoluteTheta;
  xLast = x;
  yLast = y;

  results = addPolarWaypoint(moveToR, moveToTheta, (float) ballSpeedMMperSec * SPEED_SCALER_FOR_INITAL_MOVE);
  if (results == CANCEL_DRAWING)
    return(CANCEL_DRAWING);


  //
  // loop to plot each line segment until the outer side of the sandbox is reached
  //
  while(true)
  {
    //
    // loop to find the next point to plot
    //
    thetaDeltaTooLow = -1;
    thetaDeltaTooHigh = -1;
    
    while(true)
    {
      (*equation_PlotPolarXYFuncWithIncreasingTheta)(theta + thetaDelta, &x, &y);  

      segmentLengthSqr = (x - xLast)*(x - xLast) + (y - yLast)*(y - yLast);
      loopCounter++;
    
      if (segmentLengthSqr < segmentLengthMinSqr) 
      {
        thetaDeltaTooLow = thetaDelta;
        if (thetaDeltaTooHigh == -1)
            thetaDelta = thetaDelta * 2;
        else
            thetaDelta = (thetaDeltaTooLow + thetaDeltaTooHigh) / 2;
            
        continue;
      }
  
      if(segmentLengthSqr > segmentLengthMaxSqr) 
      {
        thetaDeltaTooHigh = thetaDelta;
        if(thetaDeltaTooLow == -1)
            thetaDelta = thetaDelta / 2;
        else
            thetaDelta = (thetaDeltaTooLow + thetaDeltaTooHigh) / 2;

        continue;
      }
      
      break;
    }


    //
    // found the next point, check if at the ending value of theta
    //
    if (theta > endingTheta) 
      break;


    //
    // convert XY to polar
    //
    theta = theta + thetaDelta;
    moveToR = sqrt(x*x + y*y);
    polarAngle = Atan2D(x, y);

    //
    // check if R beyond the sandbox radius
    //
    if (moveToR > SANDBOX_MAX_RADIUS_MM) 
      break;

    //
    // check if R is less than zero
    //
    if (moveToR < 0) 
      break;


    //
    // plot the point
    //
    xLast = x;
    yLast = y;

    //
    // ATan2D generates an angle between 0 - 360.  We want the SandBox rotation arm angle to be generally increasing,
    // this math insures the angle doesn't rotate backward when commanded to go from 359 degrees to 1
    //
    if (polarAngle - lastPolarAngle < -180) 
      rotationCount++;
    if (polarAngle - lastPolarAngle > 180) 
      rotationCount--;
    moveToTheta = polarAngle + rotationCount * 360.0 + beginAtAbsoluteTheta;
    lastPolarAngle = polarAngle;

    //
    // move to the point
    //    
    results = addPolarWaypoint(moveToR, moveToTheta, (float) ballSpeedMMperSec);
    if (results == CANCEL_DRAWING)
      break;

    plottingDisplay_PlotPoint(moveToR, moveToTheta);
  }


  //
  // wait for the motion to finish, then normalize theta
  //
  while(!steppers.finishMovingToFinalWaypoints());        // wait for the motion to finish
  normalizeCurrentThetaAngle();


  //
  // return the absolute theta for the ending position
  //
  *finalAbsoluteTheta = normalizeAngle(moveToTheta);
  return(results);
}
  


//
// plot a radial function with varying R, solving for theta
//    Exit:   CONTINUE_DRAWING returned normally
//            CANCEL_DRAWING returned if user has cancelled the remaining drawing
//
int plotRadialFuncByRadius(float (*Equation_PlotRadialFuncByRadius)(float), float beginAtAbsoluteTheta, 
      float DivisionsPerCircle, float StartingRadius, float EndingRadius, float pointSpacingMM, 
      float pointSpacingTolerance, boolean startEndPosition, float *finalAbsoluteTheta)
{
  float rDelta;
  float rDeltaTooLow;
  float rDeltaTooHigh;
  float LoopCounter;
  float arcMM;
  float SegmentLengthSqr;
  float SegmentLengthMaxSqr;
  float SegmentLengthMinSqr;
  float AngularSeparation;
  float thetaRadialLine;
  float thetaDelta;
  float theta;
  float thetaLast;
  float r;
  float newR;
  float Direction;
  float moveToTheta;
  int results = CONTINUE_DRAWING;


  //
  // initialize loops
  //
  SegmentLengthMaxSqr = pointSpacingMM + (pointSpacingMM * pointSpacingTolerance);
  SegmentLengthMaxSqr *= SegmentLengthMaxSqr;
  
  SegmentLengthMinSqr = pointSpacingMM - (pointSpacingMM * pointSpacingTolerance);
  SegmentLengthMinSqr *= SegmentLengthMinSqr;

  rDelta = 1.0;
  LoopCounter = 0;


  //
  // set the starting coordinate, then move there
  //
  if (startEndPosition == START_IN_END_IN)
  {
    Direction = 1;
    r = StartingRadius;
  }
  else
  {
    Direction = -1;
    r = EndingRadius;
  }

  theta = (*Equation_PlotRadialFuncByRadius)(r);
  thetaLast = theta;
  moveToTheta = theta + beginAtAbsoluteTheta;

  results = addPolarWaypoint(r, moveToTheta, (float) ballSpeedMMperSec * SPEED_SCALER_FOR_INITAL_MOVE);
  if (results == CANCEL_DRAWING)
    return(CANCEL_DRAWING);


  //
  // loop to plot each radial line
  //
  AngularSeparation = 360.0 / DivisionsPerCircle;
  for(thetaRadialLine = 0.0;  thetaRadialLine <= 360.1 - AngularSeparation;  thetaRadialLine += AngularSeparation)
  {
    //
    // loop to plot each line segment until the outer side of the sandbox is reached
    //
    while(true)
    {
      //
      // loop to find the next point to plot
      //
      rDeltaTooLow = -1;
      rDeltaTooHigh = -1;

      while(true)
      {
        theta = (*Equation_PlotRadialFuncByRadius)(r + rDelta * Direction);

        thetaDelta = theta - thetaLast;
        arcMM = (2.0 * 3.14159 * (r + rDelta * Direction)) * (thetaDelta / 360);
        SegmentLengthSqr = (rDelta * rDelta) + (arcMM * arcMM);
        LoopCounter = LoopCounter + 1;

        if(SegmentLengthSqr < SegmentLengthMinSqr) 
        {
          rDeltaTooLow = rDelta;
          if(rDeltaTooHigh == -1)
            rDelta = rDelta * 2.0;
          else
            rDelta = (rDeltaTooLow + rDeltaTooHigh) / 2;

          continue;
        }

        if(SegmentLengthSqr > SegmentLengthMaxSqr) 
        {
          rDeltaTooHigh = rDelta;
          if(rDeltaTooLow == -1)
              rDelta = rDelta / 2.0;
          else
              rDelta = (rDeltaTooLow + rDeltaTooHigh) / 2.0;
         
          continue;
        }

        break;
      }


      //
      // found the next point, check if at the ending value of r
      //
      newR = r + rDelta * Direction;
      if (Direction > 0) 
      {
        if (newR > EndingRadius)
          break;
      }
      else
      {
        if (newR < StartingRadius) 
          break;  
      } 


      //
      // plot the point
      //
      r = newR;
      thetaLast = theta;
      moveToTheta = theta + thetaRadialLine + beginAtAbsoluteTheta;
      
      //
      // move to the point
      //    
      results = addPolarWaypoint(r, moveToTheta, (float) ballSpeedMMperSec);
      if (results == CANCEL_DRAWING)
        break;

      plottingDisplay_PlotPoint(r, moveToTheta);
    }

    if (results == CANCEL_DRAWING)
        break;

    Direction = -Direction;
  }


  //
  // wait for the motion to finish, then normalize theta
  //
  while(!steppers.finishMovingToFinalWaypoints());        // wait for the motion to finish
  normalizeCurrentThetaAngle();


  //
  // return the absolute theta for the ending position
  //
  *finalAbsoluteTheta = normalizeAngle(moveToTheta);
  return(results);
}



//
// plot a radial function with varying R, solving for theta, this backs up on the same path, 
// used to move sand from the outside in
//    Exit:   CONTINUE_DRAWING returned normally
//            CANCEL_DRAWING returned if user has cancelled the remaining drawing
//
int plotRadialFuncByRadiusBackingUpOnSamePath(float (*Equation_PlotRadialFuncByRadius)(float), 
      float beginAtAbsoluteTheta, float DivisionsPerCircle, float StartingRadius, float EndingRadius, 
      float pointSpacingMM, float pointSpacingTolerance, boolean startEndPosition, float *finalAbsoluteTheta)
{
  float rDelta;
  float rDeltaTooLow;
  float rDeltaTooHigh;
  float LoopCounter;
  float arcMM;
  float SegmentLengthSqr;
  float SegmentLengthMaxSqr;
  float SegmentLengthMinSqr;
  float AngularSeparation;
  float thetaRadialLine;
  float thetaDelta;
  float theta;
  float thetaLast;
  float r;
  float newR;
  float Direction;
  float moveToTheta;
  int results = CONTINUE_DRAWING;
  boolean repeatBackwards;


  //
  // initialize loops
  //
  SegmentLengthMaxSqr = pointSpacingMM + (pointSpacingMM * pointSpacingTolerance);
  SegmentLengthMaxSqr *= SegmentLengthMaxSqr;
  
  SegmentLengthMinSqr = pointSpacingMM - (pointSpacingMM * pointSpacingTolerance);
  SegmentLengthMinSqr *= SegmentLengthMinSqr;

  rDelta = 1.0;
  LoopCounter = 0;


  //
  // set the starting coordinate, then move there
  //
  if (startEndPosition == START_IN_END_IN)
  {
    Direction = 1;
    r = StartingRadius;
  }
  else
  {
    Direction = -1;
    r = EndingRadius;
  }

  repeatBackwards = true;
  theta = (*Equation_PlotRadialFuncByRadius)(r);
  thetaLast = theta;
  moveToTheta = theta + beginAtAbsoluteTheta;

  results = addPolarWaypoint(r, moveToTheta, (float) ballSpeedMMperSec * SPEED_SCALER_FOR_INITAL_MOVE);
  if (results == CANCEL_DRAWING)
    return(CANCEL_DRAWING);


  //
  // loop to plot each radial line
  //
  AngularSeparation = 360.0 / DivisionsPerCircle;
//  for(thetaRadialLine = 0.0;  thetaRadialLine <= 360.1 - AngularSeparation;  thetaRadialLine += AngularSeparation)
  thetaRadialLine = 0.0;
  while(true)
  {
    //
    // loop to plot each line segment until the outer side of the sandbox is reached
    //
    while(true)
    {
      //
      // loop to find the next point to plot
      //
      rDeltaTooLow = -1;
      rDeltaTooHigh = -1;

      while(true)
      {
        theta = (*Equation_PlotRadialFuncByRadius)(r + rDelta * Direction);

        thetaDelta = theta - thetaLast;
        arcMM = (2.0 * 3.14159 * (r + rDelta * Direction)) * (thetaDelta / 360);
        SegmentLengthSqr = (rDelta * rDelta) + (arcMM * arcMM);
        LoopCounter = LoopCounter + 1;

        if(SegmentLengthSqr < SegmentLengthMinSqr) 
        {
          rDeltaTooLow = rDelta;
          if(rDeltaTooHigh == -1)
            rDelta = rDelta * 2.0;
          else
            rDelta = (rDeltaTooLow + rDeltaTooHigh) / 2;

          continue;
        }

        if(SegmentLengthSqr > SegmentLengthMaxSqr) 
        {
          rDeltaTooHigh = rDelta;
          if(rDeltaTooLow == -1)
              rDelta = rDelta / 2.0;
          else
              rDelta = (rDeltaTooLow + rDeltaTooHigh) / 2.0;
         
          continue;
        }

        break;
      }


      //
      // found the next point, check if at the ending value of r
      //
      newR = r + rDelta * Direction;
      if (Direction > 0) 
      {
        if (newR > EndingRadius)
          break;
      }
      else
      {
        if (newR < StartingRadius) 
          break;  
      } 


      //
      // plot the point
      //
      r = newR;
      thetaLast = theta;
      moveToTheta = theta + thetaRadialLine + beginAtAbsoluteTheta;
      
      //
      // move to the point
      //    
      results = addPolarWaypoint(r, moveToTheta, (float) ballSpeedMMperSec);
      if (results == CANCEL_DRAWING)
        break;

      plottingDisplay_PlotPoint(r, moveToTheta);
    }

    //
    // completed this radial drawing, move to the next angle and repeat
    //
    Direction = -Direction;
    if (!repeatBackwards)
       thetaRadialLine += AngularSeparation;
    repeatBackwards = !repeatBackwards;
    
    //
    // check if the user has canceled the rest of this drawing
    //
    if (results == CANCEL_DRAWING)
        break;

    //
    // check if we have completed all rotations
    //
    if (thetaRadialLine >= 360.1 - AngularSeparation)
      break;
  }


  //
  // wait for the motion to finish, then normalize theta
  //
  while(!steppers.finishMovingToFinalWaypoints());        // wait for the motion to finish
  normalizeCurrentThetaAngle();


  //
  // return the absolute theta for the ending position
  //
  *finalAbsoluteTheta = normalizeAngle(moveToTheta);
  return(results);
}


// ---------------------------------------------------------------------------------
//                                 Homing Functions
// ---------------------------------------------------------------------------------

//
// find the home position using the homing sensors
//
void homeAllAxes(void)
{
  const float THETA_HOMING_SPEED_STEPS_PER_SEC = 0.02 * THETA_STEPS_PER_REVOLUTION;
  const float THETA_FINE_HOMING_SPEED_STEPS_PER_SEC = 0.02 * THETA_STEPS_PER_REVOLUTION;
  const float RADIUS_HOMING_SPEED_MM_PER_SEC = 35.0 * RADIUS_STEPS_PER_MM;
  const float RADIUS_FINE_HOMING_SPEED_MM_PER_SEC = 6.0 * RADIUS_STEPS_PER_MM;

  const float RADIUS_MAX_INWARD_HOMING_DISTANCE_MM = 215.0;
  const float RADIUS_MAX_OUTWARD_HOMING_DISTANCE_MM = 75.0;

  const boolean THETA_HOME_SIGNAL_WHEN_DETECTED = HIGH;
  const boolean RADIUS_HOME_SIGNAL_WHEN_DETECTED = LOW;
  const boolean RADIUS_HOME_SIGNAL_WHEN_NOT_DETECTED = HIGH;

//const float ANGULAR_OFFSET_BETWEEN_HOMING_SENSORS = -86;
  const float ANGULAR_OFFSET_BETWEEN_HOMING_SENSORS = 0;
  const float RADIAL_POSITION_OF_RADIUS_HOMING_SENSOR = 75.0;
  const float ANGULAR_POSITION_OF_RADIUS_HOMING_SENSOR = -6.0;

  float r;
  float theta;

  //
  // rotate theta until its home sensor is detected
  //
  r = 0.0; theta = 360.0;
  moveUntilHomeSensorSignal(r, theta, THETA_HOMING_SPEED_STEPS_PER_SEC, THETA_HOME_SIGNAL_WHEN_DETECTED);

  //
  // on the theta home sensor now, back off it
  //
  r = 0.0; theta = -15.0;
  moveRelativeForHoming(r, theta, THETA_HOMING_SPEED_STEPS_PER_SEC);
  
  //
  // rotate theta until its home sensor is detected again, this time slowly
  //
  r = 0.0; theta = 360.0;
  moveUntilHomeSensorSignal(r, theta, THETA_FINE_HOMING_SPEED_STEPS_PER_SEC, THETA_HOME_SIGNAL_WHEN_DETECTED);

  //
  // found theta home, backup over to the R sensor
  //
  r = 0; theta = ANGULAR_OFFSET_BETWEEN_HOMING_SENSORS;
  moveRelativeForHoming(r, theta, THETA_HOMING_SPEED_STEPS_PER_SEC);

  //
  // check if Radius axis is already on top of its homing sensor
  //
  if (digitalRead(MOTOR_RADIUS_HOME_PIN) == RADIUS_HOME_SIGNAL_WHEN_DETECTED)
  {
    //
    // pull R in until no longer detected
    //
    r = -RADIUS_MAX_INWARD_HOMING_DISTANCE_MM; theta = 0.0;
    moveUntilHomeSensorSignal(r, theta, RADIUS_HOMING_SPEED_MM_PER_SEC, RADIUS_HOME_SIGNAL_WHEN_NOT_DETECTED);
  }
  else
  {
    //
    // push R out until detected
    //
    r = RADIUS_MAX_OUTWARD_HOMING_DISTANCE_MM; theta = 0.0;
    moveUntilHomeSensorSignal(r, theta, RADIUS_HOMING_SPEED_MM_PER_SEC, RADIUS_HOME_SIGNAL_WHEN_DETECTED);
  }

  //
  // pull R back in away from the sensor
  //
  r = -6.0; theta = 0.0;
  moveRelativeForHoming(r, theta, RADIUS_HOMING_SPEED_MM_PER_SEC);

  //
  // push R out until detected again, this time slowly
  //
  r = RADIUS_MAX_OUTWARD_HOMING_DISTANCE_MM; theta = 0.0;
  moveUntilHomeSensorSignal(r, theta, RADIUS_FINE_HOMING_SPEED_MM_PER_SEC, RADIUS_HOME_SIGNAL_WHEN_DETECTED);
 

  //
  // it is homed now, set its absolute position, then move to (0, 0) coordinate
  //
  r = RADIAL_POSITION_OF_RADIUS_HOMING_SENSOR; 
  theta = ANGULAR_POSITION_OF_RADIUS_HOMING_SENSOR;
  setCurrentPolarCoordinate(r, theta);

  r = SANDBOX_MIN_RADIUS_MM + 7; 
  theta = 0;
  addPolarWaypointWithDominateMotorSpeedNoRangeChecking(r, theta, RADIUS_HOMING_SPEED_MM_PER_SEC);
  while(!steppers.finishMovingToFinalWaypoints());
}



//
// move relative until change in homing sensor is detected.  Note: if motion runs to 
// completion, assume homing sensor is not working and blink error forever.
//
void moveUntilHomeSensorSignal(float moveToR, float moveToTheta, float speedStepsPerSecond, boolean waitForThisSignal)
{
  //
  // set initial position to (0, 0)
  //
  setCurrentPolarCoordinate(0, 0);

  //
  // move the given axis
  //
  addPolarWaypointWithDominateMotorSpeedNoRangeChecking(moveToR, moveToTheta, speedStepsPerSecond);

  //
  // start the move, checking if it completes
  //
  while(!steppers.finishMovingToFinalWaypoints())
  {
    //
    // while moving, check for the homing sensor signal
    //
    if (moveToR != 0)
    {
       if (digitalRead(MOTOR_RADIUS_HOME_PIN) == waitForThisSignal)
       {
          steppers.emergencyStop();                         // stop motors now
          delay(100);
          return;
       }
    }
    else
    {
       if (digitalRead(MOTOR_THETA_HOME_PIN) == waitForThisSignal)
       {
          steppers.emergencyStop();                         // stop motors now
          delay(100);
          return;
       }
    }
  }

  //
  // move finished to full given distance indicating that the sensor signal never detected,
  // loop forever blinking error
  //
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                    // disable the motors
  
  while(true)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(70);
    digitalWrite(LED_PIN, LOW);
    delay(70);
  }
}



//
// move relative to current position when homing
//
void moveRelativeForHoming(float moveToR, float moveToTheta, float speedStepsPerSecond)
{
  //
  // set initial position to (0, 0)
  //
  setCurrentPolarCoordinate(0, 0);

  //
  // move the given axis
  //
  addPolarWaypointWithDominateMotorSpeedNoRangeChecking(moveToR, moveToTheta, speedStepsPerSecond);


  //
  // start the move, checking if it completes
  //
  while(!steppers.finishMovingToFinalWaypoints());
  delay(100);
}


// ---------------------------------------------------------------------------------
//                           Stepper Interface Functions
// ---------------------------------------------------------------------------------

float lastWaypointR = SANDBOX_MIN_RADIUS_MM;
float lastWaypointTheta = 0.0;


//
// add the waypoint given a polar coordinate and aproximate endpoint linear speed,
// note speed calculate only works for small changes in r and angular movement
//    Enter:  r = radius coordinate in MM
//            theta = angular coordinate in degrees
//            speedMMperSec = speed in MM/second
//    Exit:   CONTINUE_DRAWING returned normally
//            CANCEL_DRAWING returned if user has cancelled the remaining drawing
//
int addPolarWaypoint(float r, float theta, float speedMMperSec)
{
  float speedStepsPerSecond;
  
  float deltaThetaDegrees = theta - lastWaypointTheta;
  float deltaRMM = r - lastWaypointR;

  float averageR = (r + lastWaypointR) / 2.0;
  
  float deltaThetaDistanceMM = averageR * 2.0 * 3.14159 * (deltaThetaDegrees / 360.0);
  float rDistanceMM = deltaRMM;
  
  float distanceMM = sqrt((deltaThetaDistanceMM * deltaThetaDistanceMM) + (rDistanceMM * rDistanceMM));
  float timeToTravelSec = distanceMM / speedMMperSec;

  float thetaSteps = abs(deltaThetaDegrees * THETA_STEPS_PER_DEGREE);
  float rSteps = abs((RADIUS_STEPS_PER_DEGREE_OF_THETA * deltaThetaDegrees) + RADIUS_STEPS_PER_MM * deltaRMM);

  if (thetaSteps > rSteps)
  {
    speedStepsPerSecond = thetaSteps / timeToTravelSec;
    if (speedStepsPerSecond > MAX_THETA_MOTOR_STEPS_PER_SECOND)
      speedStepsPerSecond = MAX_THETA_MOTOR_STEPS_PER_SECOND;
  }
  else
  {
    speedStepsPerSecond = rSteps / timeToTravelSec;
     if (speedStepsPerSecond > MAX_RADIUS_MOTOR_STEPS_PER_SECOND)
      speedStepsPerSecond = MAX_RADIUS_MOTOR_STEPS_PER_SECOND;
  }

  //
  // and the waypoint, check if the user has press "Cancel"
  //
  if (addPolarWaypointWithDominateMotorSpeed(r, theta, speedStepsPerSecond) == CANCEL_DRAWING)
     return(CANCEL_DRAWING);

  return(CONTINUE_DRAWING);
}



//
// add the waypoint given a polar coordinate and dominate motor speed
//    Enter:  r = radius coordinate in MM
//            theta = angular coordinate in degrees
//            speedStepsPerSecond = speed of the dominate motor in steps/second/second
//              (dominate motor is the one that travels the most steps)
//    Exit:   CONTINUE_DRAWING returned normally
//            CANCEL_DRAWING returned if user has cancelled the remaining drawing
//
int addPolarWaypointWithDominateMotorSpeed(float r, float theta, float speedStepsPerSecond)
{
  int32_t coordinate[NUMBER_OF_MOTORS];

  //
  // check if the user has pressed the "Pause" button, if so they will choose between "Resume" and "Cancel"
  //
  if (plottingDisplay_CheckButtons() == CANCEL_DRAWING)
    return(CANCEL_DRAWING);

  //
  // make sure the coordinate is in range, then convert it to raw units and add the waypoint
  //
  if (r < SANDBOX_MIN_RADIUS_MM) r = SANDBOX_MIN_RADIUS_MM;
  if (r > SANDBOX_MAX_RADIUS_MM) r = SANDBOX_MAX_RADIUS_MM;

  coordinate[THETA_MOTOR] =  THETA_STEPS_PER_DEGREE * theta;
  coordinate[RADUIS_MOTOR] = (RADIUS_STEPS_PER_DEGREE_OF_THETA * theta) + RADIUS_STEPS_PER_MM * r;
  steppers.addWaypoint(coordinate, speedStepsPerSecond);

  lastWaypointR = r;
  lastWaypointTheta = theta;
  return(CONTINUE_DRAWING);
}



//
// add the waypoint given a polar coordinate, using dominate motor speed, without range checking
//    Enter:  r = radius coordinate in MM
//            theta = angular coordinate in degrees
//            speedStepsPerSecond = speed of the dominate motor in steps/second/second
//              (dominate motor is the one that travels the most steps)
//
void addPolarWaypointWithDominateMotorSpeedNoRangeChecking(float r, float theta, float speedStepsPerSecond)
{
  int32_t coordinate[NUMBER_OF_MOTORS];

  coordinate[THETA_MOTOR] =  THETA_STEPS_PER_DEGREE * theta;
  coordinate[RADUIS_MOTOR] = (RADIUS_STEPS_PER_DEGREE_OF_THETA * theta) + RADIUS_STEPS_PER_MM * r;
  steppers.addWaypoint(coordinate, speedStepsPerSecond);

  lastWaypointR = r;
  lastWaypointTheta = theta;
}




//
// normalize current value of theta to between 0 and 360 degrees, this must be executed
// when all motion is stopped
//
void normalizeCurrentThetaAngle(void)
{
  float r;
  float theta;

  //
  // get the current values for r & theta
  //
  getCurrentPolarCoordinate(&r, &theta);

  //
  // normalize theta to between 0 - 360, then set that theta to the current position
  //
  theta = normalizeAngle(theta);
  setCurrentPolarCoordinate(r, theta);
}



//
// get the current polar coordinate (this value changes while in motion)
//    Enter:  r -> storage to return radius coordinate in MM
//            theta -> storage to return angular coordinate in degrees
//
void getCurrentPolarCoordinate(float *r, float *theta)
{
  int32_t coordinate[NUMBER_OF_MOTORS];

  steppers.getCurrentCoordinate(coordinate);
  *theta = coordinate[THETA_MOTOR] / THETA_STEPS_PER_DEGREE;
  *r = (coordinate[RADUIS_MOTOR] - (RADIUS_STEPS_PER_DEGREE_OF_THETA * (*theta))) / RADIUS_STEPS_PER_MM;
}




//
// set the current polar coordinate (this can only be called while all motion is stopped)
//    Enter:  r = radius coordinate in MM
//            theta = angular coordinate in degrees
//
void setCurrentPolarCoordinate(float r, float theta)
{
  int32_t coordinate[NUMBER_OF_MOTORS];

  coordinate[THETA_MOTOR] =  THETA_STEPS_PER_DEGREE * theta;
  coordinate[RADUIS_MOTOR] = (RADIUS_STEPS_PER_DEGREE_OF_THETA * theta) + RADIUS_STEPS_PER_MM * r;
  steppers.setCurrentCoordinate(coordinate);

  lastWaypointR = r;
  lastWaypointTheta = theta;
}



// ---------------------------------------------------------------------------------
//                         The "LCD Plotting Display" Functions 
// ---------------------------------------------------------------------------------

BUTTON pauseButton;
int LCDPlotCenterX;
int LCDPlotCenterY;
int LCDPlotRadius;


//
// initialize the "Plotting Display" screen
//
void plottingDisplay_Init(void)
{
  LCDPlotCenterX = ui.lcdWidth/2 - 1;
  LCDPlotCenterY = ui.lcdHeight/2;
  LCDPlotRadius = ui.lcdHeight/2 - 1;

  //
  // clear the LCD screen before drawing a new figure
  //
  ui.lcdClearScreen(LCD_NAVY);

//  //
//  // draw a circle where the polar graph will be displayed
//  //
//  ui.lcdDrawCircle(LCDPlotCenterX, LCDPlotCenterY, LCDPlotRadius + 1, LCD_LIGHTBLUE);

  //
  // create and display a "Pause" button
  //
  const int buttonWidth = 60;
  const int buttonHeight = 30;
  const int buttonX = ui.displaySpaceWidth - buttonWidth/2 - 3; 
  const int buttonY = ui.displaySpaceBottomY - buttonHeight/2 - 3;
  pauseButton  = {"Pause", buttonX, buttonY, buttonWidth, buttonHeight};
  ui.drawButton(pauseButton);
}



//
// check if the Pause button has been pressed on the "Plotting Display", if so pause
//    Exit:   CONTINUE_DRAWING returned normally
//            CANCEL_DRAWING returned if user has cancelled the remaining drawing
//
int plottingDisplay_CheckButtons(void)
{
  //
  // check if the pause button has been pressed
  //
  ui.getTouchEvents();
  if (ui.checkForButtonClicked(pauseButton))
  {
    //
    // pause button has been pressed, let the user know we are finishing the points already queued
    //
    ui.lcdClearScreen(LCD_NAVY);
    ui.lcdSetCursorXY(ui.lcdWidth/2, ui.lcdHeight/2 - 8);
    ui.lcdPrintCentered("Finishing the current motion...");

    while(!steppers.finishMovingToFinalWaypoints());        // wait for the queued waypoints to finish
    stopMeasuringRunningTime();                             // stop recording the running time

    //
    // display the "paused" screen, getting new instructions from the user
    //
    return(displayPausedScreen());
  }

  return(CONTINUE_DRAWING);
}



//
// display the dialog shown when drawing is paused
//    Exit:   CONTINUE_DRAWING returned normally
//            CANCEL_DRAWING returned if user has cancelled the remaining drawing
//
int displayPausedScreen(void)
{
  const int buttonWidth = 220;
  const int buttonHeight = 35;
  const int buttonX = ui.lcdWidth/2; 
  const int buttonSpacingY = 49;
  
//  const int buttonWidth = 220;
//  const int buttonHeight = 38;
//  const int buttonX = ui.lcdWidth/2; 
//  const int buttonSpacingY = 51;
//  const int topLineY = 14;



  while(true)
  {
    //
    // draw a new screen, giving the user the choices of what to do while paused
    //
//    int y = topLineY;
//    ui.lcdClearScreen(LCD_NAVY);
//    ui.lcdSetCursorXY(ui.lcdWidth/2, y);
//    ui.lcdPrintCentered("Drawing has been paused...");
//    y += 48;




    
    ui.drawTitleBar("Drawing has been paused...");
    ui.clearDisplaySpace();
    int y = ui.displaySpaceTopY + 28;

    BUTTON settingsButton = {"Settings...", buttonX, y, buttonWidth, buttonHeight};
    ui.drawButton(settingsButton);
    y += buttonSpacingY;

    BUTTON displayRuntimeButton = {"Display runtime...", buttonX, y, buttonWidth, buttonHeight};
    ui.drawButton(displayRuntimeButton);
    y += buttonSpacingY;

    BUTTON resumeButton = {"Resume this drawing", buttonX, y, buttonWidth, buttonHeight};
    ui.drawButton(resumeButton);
    y += buttonSpacingY;
  
    BUTTON cancelButton = {"Back to the menu", buttonX, y, buttonWidth, buttonHeight};
    ui.drawButton(cancelButton);

    //
    // process touch events, check which button the user chooses
    //
    while(true)
    {
      ui.getTouchEvents();                           // check for touch events
      
      if (ui.checkForButtonClicked(settingsButton))  // check if the user presses "Settings"
      {
        settingsCommand();                           // display the "settings" dialog
        break;                                       // break so the screen is redrawn
      }
      
      if (ui.checkForButtonClicked(displayRuntimeButton)) // check if the user presses "Display Runtime"
      {
        displayRuntimeCommand();                     // show the "runtime" dialog
        break;                                       // break so the screen is redrawn
      }
      
      if (ui.checkForButtonClicked(resumeButton))    // check if the user presses "Resume"
      {
        plottingDisplay_Init();
        startMeasuringRunningTime();                 // resume recording the running time
        return(CONTINUE_DRAWING);
      }
      
      if (ui.checkForButtonClicked(cancelButton))    // check if the user presses "Cancel"
      {
        plottingDisplay_Init();
        return(CANCEL_DRAWING);
      }
    }
  }
}



//
// plot a polar point on the "Plotting Display" screen
//
void plottingDisplay_PlotPoint(float r, float theta)
{  
  float LCDScaler = ((float) LCDPlotRadius) / SANDBOX_MAX_RADIUS_MM;

  theta += 215;         // transform theta so orientation of LCD and sand table match
  
  float x = r * CosD(theta);
  float y = r * SinD(theta);
  
  int xInt = (int)round(x * LCDScaler) + LCDPlotCenterX;
  int yInt = (int)round(y * LCDScaler) + LCDPlotCenterY;

  if (xInt < 0) xInt = 0;
  if (xInt > ui.lcdWidth-1) xInt = ui.lcdWidth-1;
  if (yInt < 0) yInt = 0;
  if (yInt > ui.lcdHeight-1) yInt = ui.lcdHeight-1;
 
  ui.lcdDrawPixel(xInt, yInt, LCD_WHITE);
}


// ---------------------------------------------------------------------------------
//                                Math Functions 
// ---------------------------------------------------------------------------------

//
// return the sin of an angle in degrees
//
float SinD(float ThetaInDegrees)
{
    return(sin(ThetaInDegrees * 3.1415 / 180.0));
}



//
// return the cos of an angle in degrees
//
float CosD(float ThetaInDegrees)
{
    return(cos(ThetaInDegrees * 3.1415 / 180.0));
}



//
// return the atan2 angle in degrees (0 - 360) of an XY coord
//
float Atan2D(float x, float y)
{
  float theta = atan2(x, y) * (180 / 3.1415);
  theta = normalizeAngle(theta);
  return(theta);
}



//
// normalize an angle to between 0 and 360 degrees
//
float normalizeAngle(float theta)
{ 
  long thetaX100 = (long) (theta * 100.0);
  thetaX100 = thetaX100 % 36000L;
  if (thetaX100 < 0) thetaX100 += 36000L;
  theta = (float)thetaX100 / 100.0;
  return(theta);
}



// ---------------------------------------------------------------------------------
//                                Running Time Functions 
// ---------------------------------------------------------------------------------

//
// storage for the "running time" calculations
//
unsigned long recordedRunningTime_Seconds;
elapsedMillis runtimeForCurrentOperation_MS;
boolean runningFlag = false;


//
// read the current "running time" from EEPROM
//
void readRunningTimeFromEEPROM(void)
{
  recordedRunningTime_Seconds = (unsigned long) ui.readConfigurationInt(EEPROM_RUNNING_TIME, 0);
  runningFlag = false;
}



//
// save the total "running time" in EEPROM
//
void saveRunningTimeInEEPROM(void)
{
  ui.writeConfigurationInt(EEPROM_RUNNING_TIME, (int) getTotalRunningTimeInSeconds());
}



//
// get the total running time in seconds
//
unsigned long getTotalRunningTimeInSeconds(void)
{
  unsigned long totalTime = recordedRunningTime_Seconds;

  if (runningFlag)
    totalTime += runtimeForCurrentOperation_MS / 1000;

  return(totalTime);
}



//
// start measuring the running time
//
void startMeasuringRunningTime(void)
{
  runtimeForCurrentOperation_MS = 0L;
  runningFlag = true;
}



//
// stop measuring the running time
//
void stopMeasuringRunningTime(void)
{
  if (runningFlag)
    recordedRunningTime_Seconds += runtimeForCurrentOperation_MS / 1000;
    
  runtimeForCurrentOperation_MS = 0L;
  runningFlag = false;
}



// ---------------------------------------------------------------------------------
//                                Hardware Functions 
// ---------------------------------------------------------------------------------

//
// set micro stepping for a DRV8825 stepper driver
//    Enter:  microStepping = 1 for 200 steps/rev, 2 for 400 steps/rev, 4 for 800 steps/rev, 8 for 1600 steps/rev
//
void setMicroSteppingDRV8825(int microStepping)
{
  pinMode(MOTOR_MICROSTEP_0_PIN, OUTPUT);   
  pinMode(MOTOR_MICROSTEP_1_PIN, OUTPUT);   
  pinMode(MOTOR_MICROSTEP_2_PIN, OUTPUT);   

  switch(microStepping)
  {
    case 1:
      digitalWrite(MOTOR_MICROSTEP_0_PIN, 0);
      digitalWrite(MOTOR_MICROSTEP_1_PIN, 0);
      digitalWrite(MOTOR_MICROSTEP_2_PIN, 0);
      break;

    case 2:
      digitalWrite(MOTOR_MICROSTEP_0_PIN, 1);
      digitalWrite(MOTOR_MICROSTEP_1_PIN, 0);
      digitalWrite(MOTOR_MICROSTEP_2_PIN, 0);
      break;

    case 4:
      digitalWrite(MOTOR_MICROSTEP_0_PIN, 0);
      digitalWrite(MOTOR_MICROSTEP_1_PIN, 1);
      digitalWrite(MOTOR_MICROSTEP_2_PIN, 0);
      break;

    case 8:
      digitalWrite(MOTOR_MICROSTEP_0_PIN, 1);
      digitalWrite(MOTOR_MICROSTEP_1_PIN, 1);
      digitalWrite(MOTOR_MICROSTEP_2_PIN, 0);
      break;

    case 16:
      digitalWrite(MOTOR_MICROSTEP_0_PIN, 0);
      digitalWrite(MOTOR_MICROSTEP_1_PIN, 0);
      digitalWrite(MOTOR_MICROSTEP_2_PIN, 1);
      break;

    case 32:
      digitalWrite(MOTOR_MICROSTEP_0_PIN, 1);
      digitalWrite(MOTOR_MICROSTEP_1_PIN, 0);
      digitalWrite(MOTOR_MICROSTEP_2_PIN, 1);
      break;
    
    default:
      digitalWrite(MOTOR_MICROSTEP_0_PIN, 0);
      digitalWrite(MOTOR_MICROSTEP_1_PIN, 0);
      digitalWrite(MOTOR_MICROSTEP_2_PIN, 0);
      break;
  }
}



//
// set micro stepping for a TMC2208 stepper driver
//    Enter:  microStepping = 2 for 400 steps/rev, 4 for 800 steps/rev, 8 for 1600 steps/rev, 16 for 3200 steps/rev
//
void setMicroSteppingTMC2208(int microStepping)
{
  pinMode(MOTOR_MICROSTEP_0_PIN, OUTPUT);   
  pinMode(MOTOR_MICROSTEP_1_PIN, OUTPUT);   
  pinMode(MOTOR_MICROSTEP_2_PIN, INPUT);   

  switch(microStepping)
  {
    case 2:                                       // stealthChop with 2x microstepping and 1/256 interpolation
      digitalWrite(MOTOR_MICROSTEP_0_PIN, 1);
      digitalWrite(MOTOR_MICROSTEP_1_PIN, 0);
      break;

    case 4:                                       // stealthChop with 4x microstepping and 1/256 interpolation
      digitalWrite(MOTOR_MICROSTEP_0_PIN, 0);
      digitalWrite(MOTOR_MICROSTEP_1_PIN, 1);
      break;

    case 8:                                       // stealthChop with 8x microstepping and 1/256 interpolation
      digitalWrite(MOTOR_MICROSTEP_0_PIN, 0);
      digitalWrite(MOTOR_MICROSTEP_1_PIN, 0);
      break;

    case 16:                                       // stealthChop with 16x microstepping and 1/256 interpolation
      digitalWrite(MOTOR_MICROSTEP_0_PIN, 1);
      digitalWrite(MOTOR_MICROSTEP_1_PIN, 1);
      break;
    
    default:
      digitalWrite(MOTOR_MICROSTEP_0_PIN, 1);
      digitalWrite(MOTOR_MICROSTEP_1_PIN, 1);
      break;
  }
}
// -------------------------------------- End --------------------------------------
