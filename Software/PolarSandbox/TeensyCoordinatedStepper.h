//      ******************************************************************
//      *                                                                *
//      *          Header file for TeensyCoordinatedStepper.cpp          *
//      *                                                                *
//      *               Copyright (c) S. Reifel & Co,  2019              *
//      *                                                                *
//      ******************************************************************


// MIT License
// 
// Copyright (c) 2019 Stanley Reifel & Co.
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


#ifndef TeensyCoordinatedStepper_h
#define TeensyCoordinatedStepper_h

#include <Arduino.h>

const int MAX_NUMBER_OF_MOTORS = 4;
const int MAX_NUMBER_OF_WAYPOINTS = 50;
const int MAX_NUMBER_OF_ISR_SEGMENTS = 10;


//
// values returned from getMotionStatus()
//
const int MOTION_STATUS_DONE                     = 0;
const int MOTION_STATUS_WAITING_FOR_MORE_POINTS  = 1;
const int MOTION_STATUS_RUNNING                  = 2;


//
// the TeensyCoordinatedStepper class
//
class TeensyCoordinatedStepper
{
  public:
    //
    // public functions
    //
    TeensyCoordinatedStepper();
    void connectToPins(byte motor0_StepPin, byte motor0_DirectionPin, boolean motor0_ReverseDirection, byte motor1_StepPin, byte motor1_DirectionPin, boolean motor1_ReverseDirection);
    void connectToPins(byte motor0_StepPin, byte motor0_DirectionPin, boolean motor0_ReverseDirection, byte motor1_StepPin, byte motor1_DirectionPin, boolean motor1_ReverseDirection, byte motor2_StepPin, byte motor2_DirectionPin, boolean motor2_ReverseDirection);
    void connectToPins(byte motor0_StepPin, byte motor0_DirectionPin, boolean motor0_ReverseDirection, byte motor1_StepPin, byte motor1_DirectionPin, boolean motor1_ReverseDirection, byte motor2_StepPin, byte motor2_DirectionPin, boolean motor2_ReverseDirection, byte motor3_StepPin, byte motor3_DirectionPin, boolean motor3_ReverseDirection);
    void setAcceleration(float _accelerationRate);
    void setJunctionDeviation(float _junctionDeviation);
    void emergencyStop(void);
    void addWaypoint(int32_t *motorSteps, float desiredSpeed);
    boolean finishMovingToFinalWaypoints(void);
    int getMotionStatus(void);
    void getCurrentCoordinate(int32_t *currentCoordinate);
    void setCurrentCoordinate(int32_t *newCoordinate);
    void dumpWaypointBuffer(void);
    void dumpIsrSegmentsBuffer(void);
    void dumpMotionStatus(void);
    boolean isWaypointsBufferFull(void);
    void sendWaypointsToSteppers(void);


  private:
    //
    // private functions
    //
    void initializePins(void);
    void initialize(void);
    void removeWaypoint(void);
    void setWaypointsInitialSpeeds(void);
    int nextWaypointIdx(int waypointIdx);
    int previousWaypointIdx(int waypointIdx);
    int waypointsCount(void);
    void startStepperISR(void);
    void transferOneWaypointToIsrSegment(void);
    int nextIsrSegmentIdx(int isrSegmentIdx);
    int previousIsrSegmentIdx(int isrSegmentIdx);
    boolean isIsrSegmentsBufferFull(void);
    int isrSegmentsCount(void);
};

// ------------------------------------ End ---------------------------------
#endif
