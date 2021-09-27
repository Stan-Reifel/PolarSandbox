//      ******************************************************************
//      *                                                                *
//      *                  Header file for SupportFuncs.h                 *
//      *                                                                *
//      *               Copyright (c) S. Reifel & Co,  2019              *
//      *                                                                *
//      ******************************************************************


#ifndef SupportFuncs_h
#define SupportFuncs_h


//
// function declarations
//
int plotPolarFuncWithIncreasingTheta(float (*equation_PlotPolarFuncWithIncreasingTheta)(float),
         float speed, float  beginAtAbsoluteTheta, float startingTheta, float endingTheta, 
         float pointSpacingMM, float pointSpacingTolerance, boolean clipPlotToSandboxRadiusFlag,
         float *finalAbsoluteTheta);

int plotRadialFuncByRadius(float (*Equation_PlotRadialFuncByRadius)(float), float speed, 
      float beginAtAbsoluteTheta, float DivisionsPerCircle, float StartingRadius, float EndingRadius, 
      float pointSpacingMM, float pointSpacingTolerance, boolean startEndPosition, float *finalAbsoluteTheta);

int plotPolarXYFuncWithIncreasingTheta(void(*equation_PlotPolarXYFuncWithIncreasingTheta)(float, float*, float*),
         float speed, float  beginAtAbsoluteTheta, float startingTheta, float endingTheta, float pointSpacingMM, 
         float pointSpacingTolerance, float *finalAbsoluteTheta);

int plotRadialFuncByRadiusBackingUpOnSamePath(float (*Equation_PlotRadialFuncByRadius)(float), float speed, 
      float beginAtAbsoluteTheta, float DivisionsPerCircle, float StartingRadius, float EndingRadius, 
      float pointSpacingMM, float pointSpacingTolerance, boolean startEndPosition, float *finalAbsoluteTheta);

void homeAllAxes(void);
int addPolarWaypoint(float r, float theta, float speedMMperSec);
int addPolarWaypointWithDominateMotorSpeed(float r, float theta, float speed);
void addPolarWaypointWithDominateMotorSpeedNoRangeChecking(float r, float theta, float speedStepsPerSecond);
void normalizeCurrentThetaAngle(void);
void getCurrentPolarCoordinate(float *r, float *theta);
void setCurrentPolarCoordinate(float r, float theta);
void plottingDisplay_Init(void);
int plottingDisplay_CheckButtons(void);
void plottingDisplay_PlotPoint(float r, float theta);
float SinD(float ThetaInDegrees);
float CosD(float ThetaInDegrees);
float Atan2D(float x, float y);
float normalizeAngle(float theta);
void readRunningTimeFromEEPROM(void);
void saveRunningTimeInEEPROM(void);
unsigned long getTotalRunningTimeInSeconds(void);
void startMeasuringRunningTime(void);
void stopMeasuringRunningTime(void);
void setMicroSteppingDRV8825(int microStepping);
void setMicroSteppingTMC2208(int microStepping);



// ------------------------------------ End ---------------------------------
#endif
