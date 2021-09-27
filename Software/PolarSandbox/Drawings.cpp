//      ******************************************************************
//      *                                                                *
//      *                         Drawing Designs                        *
//      *                                                                *
//      *               Copyright (c) S. Reifel & Co,  2019              *
//      *                                                                *
//      ******************************************************************

#include <arduino.h>
#include "Constants.h"
#include "Drawings.h"
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
void addXYWaypoint(float X, float Y, float speed);
float SinD(float ThetaInDegrees);
float CosD(float ThetaInDegrees);


//
// globals 
//
float finalThetaFromLastPlot = 0.0;


//
// constants used by the polar equations
//
static boolean spiralDirection;
static float spiralPitchMM;
static float amplitudeModMM;
static float petalsMultiplier;
static float numberSides;
static float twistAngle;
static float cloverLeaves;
static float numberScalops;
static float amplitudeOffset;
static float amplitudeScalerWithRotations;
static float starPoints;
static float angularSweep;
static float inwardThetaOffset;
static float outerRadius;
static float interRadius;
static float penRadius;
static float triangleAmplitude;
static float trianglePeriod;
static float xySizeScaler;
static float endTheta;


// ---------------------------------------------------------------------------------
//                                       Spiral 
// ---------------------------------------------------------------------------------


//
// polar function: Spiral
//
float Equation_Spiral(float theta)
{
  if (spiralDirection == SPIRAL_IN)
    theta = inwardThetaOffset - theta;
  float r = (spiralPitchMM * theta / 360.0);
  return(r);
}



//
// draw: "Spiral" 
//
int drawSpiral(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  spiralPitchMM = 14;
  float startingTheta = 180;
  float endingTheta = 7148;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = false;


  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_Spiral, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}



//
// draw: "Fine Pitch Spiral"
//
int drawSpiralHD(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();
  
  //
  // draw a fine pitch spiral moving in
  //
  spiralPitchMM = 6;
  float startingTheta = 1000;
  float endingTheta = 16679;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = false;


  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_Spiral, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}



//
// draw: "Loose Spiral" AKA "Rewind"
//
int drawLooseSpiral(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  spiralPitchMM = 65;
  float startingTheta = 60;
  float endingTheta = 1539;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = false;


  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_Spiral, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}

// ---------------------------------------------------------------------------------
//                                      Petals
// ---------------------------------------------------------------------------------

//
// polar function: Petals (Done)
//
float Equation_Petals(float theta)
{
  if (spiralDirection == SPIRAL_IN)
    theta = inwardThetaOffset - theta;
  float r = (spiralPitchMM / 360.0 * theta) + petalsMultiplier * theta / 360.0 * SinD(theta * 5);
  return(r);
}



//
// draw: "Petals" 
//
int drawPetals(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  spiralPitchMM = 11;
  petalsMultiplier = 6;
  float startingTheta = 600;
  float endingTheta = 5919;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = false;


  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_Petals, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}


// ---------------------------------------------------------------------------------
//                                      Polygons
// ---------------------------------------------------------------------------------


//
// polar function: Polygons
//
float Equation_Polygons(float theta)
{
  if (spiralDirection == SPIRAL_IN)
    theta = inwardThetaOffset - theta;
  theta = theta + (theta / 360 * twistAngle);
  float r = (spiralPitchMM / 360.0 * theta) * CosD(180 / numberSides) / CosD(theta - 360 / numberSides * floor((numberSides * theta + 180) / 360));
  return(r);
}



//
// draw: "Trangles"
//
int drawTriangles(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  spiralPitchMM = 18;
  numberSides = 3;
  twistAngle = 3;
  float startingTheta = 380;
  float endingTheta = 5533;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = false;

  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_Polygons, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}



//
// draw: "Trangles - Clipped"
//
int drawTrianglesClipped(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  spiralPitchMM = 18;
  numberSides = 3;
  twistAngle = 3;
  float startingTheta = 380;
  float endingTheta = 11340;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = true;


  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_Polygons, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}




//
// draw: "Squares" 
//
int drawSquares(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  spiralPitchMM = 17;
  numberSides = 4;
  twistAngle = 0;
  float startingTheta = 180;
  float endingTheta = 5894;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = false;

  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_Polygons, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}



//
// draw: "Squares clipped" 
//
int drawSquaresClipped(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  spiralPitchMM = 17;
  numberSides = 4;
  twistAngle = 0;
  float startingTheta = 200;
  float endingTheta = 8220 + 225;       // Added 225 to clean out the outer perimeter
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = true;

  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_Polygons, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}




//
// draw: "Hexagons"
//
int drawHexagons(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  spiralPitchMM = 17;
  numberSides = 6;
  twistAngle = 0;
  float startingTheta = 270;
  float endingTheta = 5909-54;        // -54 added to interface with next plot
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = false;

  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_Polygons, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}




//
// draw: "Polygon super twist"
//
int drawPolygonSuperTwist(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  spiralPitchMM = 10;           // this looks nice, problably look better if clipped
  numberSides = 5;
  twistAngle = -5;
  float startingTheta = 700;
  float endingTheta = 10184;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = false;


  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_Polygons, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}


// ---------------------------------------------------------------------------------
//                                      Clovers
// ---------------------------------------------------------------------------------

//
// polar function: Clovers
//
float Equation_Clovers(float theta)
{
  if (spiralDirection == SPIRAL_IN)
    theta = inwardThetaOffset - theta;
  theta = theta + (theta / 360 * twistAngle);
  float amplitude = amplitudeOffset + amplitudeScalerWithRotations * (theta / 360);
  float r = (spiralPitchMM * theta / 360.0) + abs((amplitude * SinD(theta * cloverLeaves / 2)));
  return(r);
}


//
// draw: "Clovers" 
// I changed this to a 6 sided clover so it matches with the hexagons below it
//
int drawClovers(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();
  
  //
  // set parameters for this drawing
  //  
  spiralPitchMM = 9;
  cloverLeaves = 6;
  amplitudeOffset = 15;
  amplitudeScalerWithRotations = 9;
  twistAngle = 0;
  float startingTheta = 360;
  float endingTheta = 5304;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = false;

  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_Clovers, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}



//
// draw: "Clovers twisted" 
//
int drawCloversTwisted(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  spiralPitchMM = 14;
  cloverLeaves = 5;
  amplitudeOffset = 40;
  amplitudeScalerWithRotations = 1;
  twistAngle = 2;
  float startingTheta = 152;
  float endingTheta = 5697;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = false;


  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_Clovers, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}



//
// draw: "Clovers super twist" 
//
int drawCloversSuperTwist(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  spiralPitchMM = 6;              // changed from 4 to 5 lobes to match previous drawing
  cloverLeaves = 5;
  amplitudeOffset = 20;
  amplitudeScalerWithRotations = 2.5;
  twistAngle = -3;
  float startingTheta = 370;
  float endingTheta = 11065;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = false;


  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_Clovers, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}



//
// draw: "Little clovers" 
//
int drawLittleClovers(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  spiralPitchMM = 12;
  cloverLeaves = 20;
  amplitudeOffset = 6;
  amplitudeScalerWithRotations = 1.1;
  twistAngle = 0;
  float startingTheta = 360;
  float endingTheta = 7478;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = false;

  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_Clovers, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}




//
// draw: "Little clovers twisted" 
//
int drawLittleCloversTwisted(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  spiralPitchMM = 12;
  cloverLeaves = 22;
  amplitudeOffset = 6;
  amplitudeScalerWithRotations = 0.9;
  twistAngle = 3;
  float startingTheta = 360;
  float endingTheta = 7537;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = false;

  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_Clovers, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}


// ---------------------------------------------------------------------------------
//                                       Stars
// ---------------------------------------------------------------------------------

//
// polar function: Stars
//
float Equation_Stars(float theta)
{
  if (spiralDirection == SPIRAL_IN)
    theta = inwardThetaOffset - theta;
  theta = theta + (theta / 360 * twistAngle);
  float amplitude = amplitudeOffset + amplitudeScalerWithRotations * (theta / 360);
  float r = (spiralPitchMM * theta / 360.0) - abs((amplitude * SinD(theta * starPoints / 2)));
  return(r);
}



//
// draw: "Stars" without twist 
//
int drawStars(void)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();


  //
  // start by spiraling a polygon in
  //
  spiralPitchMM = 12;
  numberSides = 9;
  twistAngle = 0;
  float startingTheta = 4520;
  float endingTheta = 8378;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = false;

  spiralDirection = SPIRAL_IN;
  inwardThetaOffset = startingTheta + endingTheta;
  if (plotPolarFuncWithIncreasingTheta(&Equation_Polygons, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
          startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
          &finalThetaFromLastPlot) == CANCEL_DRAWING)
    return(CANCEL_DRAWING);


  //
  // now draw the Stars on top of the Polygon
  //
  spiralPitchMM = 20;
  starPoints = 9;
  amplitudeOffset = 10;
  amplitudeScalerWithRotations = 9;
  twistAngle = 0;
  startingTheta = 710;
  endingTheta = 4900;
  endpointSpacingMM = 2;
  endpointSpacingTolerance = 0.2;
  clipPlotToSandboxRadiusFlag = false;

  spiralDirection = SPIRAL_IN;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_Stars, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}



//
// draw: "Stars twisted" 
//
int drawStarsTwisted(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  spiralPitchMM = 26;
  starPoints = 5;
  amplitudeOffset = 10;
  amplitudeScalerWithRotations = 13;
  twistAngle = 4.2;
  float startingTheta = 480;
  float endingTheta = 3842;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = false;


  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_Stars, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}



//
// draw: "Spider Web" 
//
int drawSpiderWeb(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  spiralPitchMM = 12;
  starPoints = 19;
  amplitudeOffset = 5;
  amplitudeScalerWithRotations = 1.3;
  twistAngle = 0;
  float startingTheta = 600;
  float endingTheta = 8355;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = false;


  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_Stars, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}



//
// draw: "Star flower" 
//
int drawStarFlower(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  spiralPitchMM = 20;
  starPoints = 3;
  amplitudeOffset = -10;
  amplitudeScalerWithRotations = 15;
  twistAngle = 0;
  float startingTheta = 300;
  float endingTheta = 5039;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = false;


  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_Stars, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}


// ---------------------------------------------------------------------------------
//                                      Scalops
// ---------------------------------------------------------------------------------

//
// polar function: Scalops
//
float Equation_Scalops(float theta)
{
  if (spiralDirection == SPIRAL_IN)
    theta = inwardThetaOffset - theta;
  theta = theta + (theta / 360 * twistAngle);
  float r = (spiralPitchMM / 360.0 * theta) * CosD(180 / numberScalops) / CosD(theta - 360 / numberScalops * floor((numberScalops * theta + 180) / 360));
  theta = theta + (180 / numberScalops);
  float amplitude = amplitudeOffset + amplitudeScalerWithRotations * (theta / 360);
  r = r - abs((amplitude * SinD((theta) * numberScalops / 2)));
  return(r);
}




//
// draw: "Scalops" with a large twist
//
int drawScalopsTwisted(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  spiralPitchMM = 10;
  numberScalops = 5;
  amplitudeOffset = 30;
  amplitudeScalerWithRotations = 0;
  twistAngle = 4;
  float startingTheta = 2000;
  float endingTheta = 9933;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = false;

  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_Scalops, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}



//
// draw: "Scalops" with a large twist and clipped to enclose in a circle
//
int drawScalopsTwistedClipped(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  spiralPitchMM = 10;
  numberScalops = 5;
  amplitudeOffset = 30;
  amplitudeScalerWithRotations = 0;
  twistAngle = 4;
  float startingTheta = 2000;
  float endingTheta = 13000;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = true;

  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_Scalops, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}



// ---------------------------------------------------------------------------------
//                                  Radial Drawings
// ---------------------------------------------------------------------------------

//
// radial function: Sun Rays
//
float Equation_SunRays(float r)
{
  float theta = amplitudeModMM * CosD((r / SANDBOX_MAX_RADIUS_MM) * angularSweep);
  return(theta);
}



//
// draw: "Sun Rays - Zen" 
//    Enter: startEndPosition = START_IN_END_IN to start/end on the inside
//                            = START_OUT_END_OUT to start/end on the outside
//
int drawSunRays(boolean startEndPosition)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();


  //
  // set parameters for this drawing
  //
  float numberOfRays = 60;
  amplitudeModMM = 22;
  angularSweep = 130;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
 

  //
  // draw the figure
  //
  const float minRadiusMM = 24.0;
  return(plotRadialFuncByRadius(&Equation_SunRays, (float) ballSpeedMMperSec, finalThetaFromLastPlot, numberOfRays, 
    minRadiusMM, SANDBOX_MAX_RADIUS_MM,endpointSpacingMM, endpointSpacingTolerance, startEndPosition, 
    &finalThetaFromLastPlot));
}



//
// draw: "Hot Sun" 
//    Enter: startEndPosition = START_IN_END_IN to start/end on the inside
//                            = START_OUT_END_OUT to start/end on the outside
//
int drawHotSun(boolean startEndPosition)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  float numberOfRays = 70;
  amplitudeModMM = 4;
  angularSweep = 1000;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;

  //
  // draw the figure
  //
  const float minRadiusMM = 24.0;
  return(plotRadialFuncByRadius(&Equation_SunRays, (float) ballSpeedMMperSec, finalThetaFromLastPlot, numberOfRays, 
    minRadiusMM, SANDBOX_MAX_RADIUS_MM,endpointSpacingMM, endpointSpacingTolerance, startEndPosition, 
    &finalThetaFromLastPlot));
}



//
// draw: "Psycho" 
//    Enter: startEndPosition = START_IN_END_IN to start/end on the inside
//                            = START_OUT_END_OUT to start/end on the outside
//
int drawPsycho(boolean startEndPosition)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  amplitudeModMM = 35;
  angularSweep = 600;
  float numberOfRays = 32;
  float endpointSpacingMM = 0.1;
  float endpointSpacingTolerance = 0.2;
 

  //
  // draw the figure
  //
  const float minRadiusMM = 24.0;
  return(plotRadialFuncByRadius(&Equation_SunRays, (float) ballSpeedMMperSec, finalThetaFromLastPlot, numberOfRays, 
    minRadiusMM, SANDBOX_MAX_RADIUS_MM,endpointSpacingMM, endpointSpacingTolerance, startEndPosition,
    &finalThetaFromLastPlot));
}



//
// draw: "Spiral rays" 
//    Enter: startEndPosition = START_IN_END_IN to start/end on the inside
//                            = START_OUT_END_OUT to start/end on the outside
//
int drawSpiralRays(boolean startEndPosition)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  float numberOfRays = 8;
  amplitudeModMM = -150;
  angularSweep = 90;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;

  //
  // draw the figure
  //
  const float minRadiusMM = 24.0;
  return(plotRadialFuncByRadius(&Equation_SunRays, (float) ballSpeedMMperSec, finalThetaFromLastPlot, numberOfRays, 
    minRadiusMM, SANDBOX_MAX_RADIUS_MM, endpointSpacingMM, endpointSpacingTolerance, startEndPosition,
    &finalThetaFromLastPlot));
}




//
// draw such that sand is moved from the outside perimeter inward 
//
int drawMoveSandInward(void)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();


  //
  // start by doing a few laps around the outside perimeter
  //
  spiralPitchMM = 3;
  float startingTheta = 32500;
  float endingTheta = 33800;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = true;

  spiralDirection = SPIRAL_OUT;
  inwardThetaOffset = startingTheta + endingTheta;
  if (plotPolarFuncWithIncreasingTheta(&Equation_Spiral, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
       startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
       &finalThetaFromLastPlot) == CANCEL_DRAWING)
    return(CANCEL_DRAWING);


  //
  // draw strokes from the outside in
  //
  float numberOfRays = 60;
  amplitudeModMM = 22;
  angularSweep = 130;
  endpointSpacingMM = 2;
  endpointSpacingTolerance = 0.2;

//const float minRadiusMM = 50.0;
  const float minRadiusMM = 55.0;
  return(plotRadialFuncByRadiusBackingUpOnSamePath(&Equation_SunRays, (float) ballSpeedMMperSec, finalThetaFromLastPlot,  
    numberOfRays, minRadiusMM, SANDBOX_MAX_RADIUS_MM, endpointSpacingMM, endpointSpacingTolerance, START_OUT_END_OUT,
    &finalThetaFromLastPlot));
}



// ---------------------------------------------------------------------------------
//                                      Spirographs
// ---------------------------------------------------------------------------------

//
// spirograph function
//
void Equation_Spirograph(float theta, float *x, float *y)
{
  //
  // equations taken from: http://www.mathematische-basteleien.de/spirographs.htm
  //
  theta = -theta;
  float thetaRad = theta * 3.14159 / 180.0;
  float OminusI = outerRadius - interRadius;
  float IoverO = interRadius / outerRadius;
  float j = IoverO * thetaRad;
  float k = (1 - IoverO) * thetaRad;
  *y = (OminusI * cos(j) + penRadius * cos(k)) * xySizeScaler;
  *x = (OminusI * sin(j) - penRadius * sin(k)) * -xySizeScaler;
}



//
// draw spirograph: "Six Blade" 
//
int drawSixBlade(void)
{ 
  float absoluteStartingAngle;
  
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();
  
  //
  // set parameters for this drawing
  //
  outerRadius = 0.8;
  interRadius = 0.39506;
  penRadius = 0.55;
  xySizeScaler = 287;
  float startingTheta = 0;
  float endingTheta = 3600;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;


  //
  // draw the figure
  //
  absoluteStartingAngle = finalThetaFromLastPlot;
  if (plotPolarXYFuncWithIncreasingTheta(Equation_Spirograph, (float) ballSpeedMMperSec, absoluteStartingAngle, 
       startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, &finalThetaFromLastPlot) == CANCEL_DRAWING)
    return(CANCEL_DRAWING);

  absoluteStartingAngle -= 60.0;
  if (plotPolarXYFuncWithIncreasingTheta(Equation_Spirograph, (float) ballSpeedMMperSec, absoluteStartingAngle, 
      startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, &finalThetaFromLastPlot) == CANCEL_DRAWING)
    return(CANCEL_DRAWING);
    
  absoluteStartingAngle -= 60.0;
  return(plotPolarXYFuncWithIncreasingTheta(Equation_Spirograph, (float) ballSpeedMMperSec, absoluteStartingAngle, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, &finalThetaFromLastPlot));
}



//
// draw spirograph: "loops" 
//
int drawLoops(void)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();
  
  //
  // set parameters for this drawing
  //
  outerRadius = 0.9;
  interRadius = 0.3094;
  penRadius = 0.4;
  xySizeScaler = 278;
  float startingTheta = 0;
  float endingTheta = 11510;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;


  //
  // draw the figure
  // 
  return(plotPolarXYFuncWithIncreasingTheta(Equation_Spirograph, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, &finalThetaFromLastPlot));
}



//
// draw spirograph: "perimeter skirt" 
//
int drawPerimeterSkirt(void)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();
  
  //
  // set parameters for this drawing
  //
  outerRadius = 1;
  interRadius = 0.01333333333;
  penRadius = 0.1;
  xySizeScaler = 255;
  float startingTheta = 0;
  float endingTheta = 26996;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;


  //
  // draw the figure
  // 
  return(plotPolarXYFuncWithIncreasingTheta(Equation_Spirograph, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, &finalThetaFromLastPlot));
}



// ---------------------------------------------------------------------------------
//                             Narrowing spirographs
// ---------------------------------------------------------------------------------

//
// spirograph function
//
void Equation_NarrowingSpirograph(float theta, float *x, float *y)
{
  penRadius = 0.365 * theta / endTheta;
  outerRadius = (1 - penRadius) / 0.6;
  interRadius = outerRadius + penRadius - 1.0;

  //
  // equations taken from: http://www.mathematische-basteleien.de/spirographs.htm
  //
  theta = -theta;
  float thetaRad = theta * 3.14159 / 180.0;
  float OminusI = outerRadius - interRadius;
  float IoverO = interRadius / outerRadius;
  float j = IoverO * thetaRad;
  float k = (1 - IoverO) * thetaRad;
  *y = (OminusI * cos(j) + penRadius * cos(k)) * xySizeScaler;
  *x = (OminusI * sin(j) - penRadius * sin(k)) * -xySizeScaler;
}



//
// draw spirograph: "narrowing spirograph" 
//
int drawNarrowingSpirograph(void)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();
  
  //
  // set parameters for this drawing
  //
  xySizeScaler = SANDBOX_MAX_RADIUS_MM - 1;     
  float startingTheta = 0;
  float endingTheta = 12 * 1800;
  endTheta = endingTheta;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;


  //
  // draw the figure
  // 
  return(plotPolarXYFuncWithIncreasingTheta(Equation_NarrowingSpirograph, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, &finalThetaFromLastPlot));
}

// ---------------------------------------------------------------------------------
//                                      Hearts
// ---------------------------------------------------------------------------------

//
// polar function: Petals (Done)
//
void Equation_Hearts(float theta, float *x, float *y)
{
  //
  // heart equations taken from: https://www.wolframalpha.com/input/?i=x+%3D+16+sin%5E3+t%2C+y+%3D+%2813+cos+t+-+5+cos+2t+-+2+cos+3t+-+cos+4t%29
  //
  float t = theta * 3.14159 / 180.0;
  float X = 16.0 * pow((sin(t)),  3.0);
  float Y = 13.0 * cos(t) - 5.0 * cos(2.0 * t) - 2.0 * cos(3.0 * t) - cos(4 * t);
  X = X / 17.0;         // scale X & Y to aprox +/-1
  Y = Y / 17.0;

  //
  // scale for spiral out
  //
  *x = X * spiralPitchMM * theta / 360.0;
  *y = Y * spiralPitchMM * theta / 360.0;
}



//
// draw: "Hearts" 
//
int drawHearts(void)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  spiralPitchMM = 14;
  float startingTheta = 400;
//float endingTheta = 7025;     // full size
  float endingTheta = 5930;     // smaller for skirt of loops
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;


  //
  // draw the figure
  //
  return(plotPolarXYFuncWithIncreasingTheta(Equation_Hearts, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, &finalThetaFromLastPlot));
}


// ---------------------------------------------------------------------------------
//                             Scalops Triple Twist 
// ---------------------------------------------------------------------------------

//
// polar function: Scalops Triple Twist
//
float Equation_ScalopsTripleTwist(float theta)
{
  if (spiralDirection == SPIRAL_IN)
    theta = inwardThetaOffset - theta;

  float triangleValue = 4 * triangleAmplitude / trianglePeriod * abs(((long)(theta - trianglePeriod / 4) % (long)trianglePeriod) - trianglePeriod / 2) - triangleAmplitude;
  theta = theta + triangleValue;
  float r = (spiralPitchMM / 360.0 * theta) * CosD(180 / numberScalops) / CosD(theta - 360 / numberScalops * floor((numberScalops * theta + 180) / 360));
  theta = theta + (180 / numberScalops);
  float amplitude = amplitudeOffset + amplitudeScalerWithRotations * (theta / 360);
  r = r - abs((amplitude * SinD((theta) * numberScalops / 2)));
  return(r);
 }



//
// draw: "Scalops Triple Twist" 
//
int drawScalopsTripleTwist(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  spiralPitchMM = 10;
  numberScalops = 5;
  amplitudeOffset = 30;
  amplitudeScalerWithRotations = 0;
  triangleAmplitude = 12;
  trianglePeriod = 360 * 16;
  float startingTheta = 1800;
  float endingTheta = 10055;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = false;


  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_ScalopsTripleTwist, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}




//
// draw: "Scalops Triple Twist Clipped" 
//
int drawScalopsTripleTwistClipped(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  spiralPitchMM = 10;
  numberScalops = 5;
  amplitudeOffset = 30;
  amplitudeScalerWithRotations = 0;
  triangleAmplitude = 12;
  trianglePeriod = 360 * 16;
  float startingTheta = 1800;
  float endingTheta = 13800;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = true;


  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_ScalopsTripleTwist, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}



// ---------------------------------------------------------------------------------
//                             Clovers Triple Twist 
// ---------------------------------------------------------------------------------

//
// polar function: Clovers Triple Twist
//
float Equation_CloversTripleTwist(float theta)
{
  if (spiralDirection == SPIRAL_IN)
    theta = inwardThetaOffset - theta;

  float triangleValue = 4 * triangleAmplitude / trianglePeriod * abs(((long)(theta - trianglePeriod / 4) % (long)trianglePeriod) - trianglePeriod / 2) - triangleAmplitude;
  theta = theta + triangleValue;
  float amplitude = amplitudeOffset + amplitudeScalerWithRotations * (theta / 360);
  float r = (spiralPitchMM * theta / 360.0) + abs((amplitude * SinD(theta * cloverLeaves / 2)));
  return(r);
 }



//
// draw: "Clovers Triple Twist Clipped" 
//
int drawCloversTripleTwistClipped(boolean direction)
{ 
  //
  // prep the display screen for what's shown while drawing 
  //
  plottingDisplay_Init();

  //
  // set parameters for this drawing
  //
  spiralPitchMM = 7;
  cloverLeaves = 5;
  amplitudeOffset = 0;
  amplitudeScalerWithRotations = 2.5;
  triangleAmplitude = 12;
  trianglePeriod = 360 * 17;
  float startingTheta = 650;
  float endingTheta = 14000;
  float endpointSpacingMM = 2;
  float endpointSpacingTolerance = 0.2;
  boolean clipPlotToSandboxRadiusFlag = true;


  //
  // draw the figure
  //
  spiralDirection = direction;
  inwardThetaOffset = startingTheta + endingTheta;
  return(plotPolarFuncWithIncreasingTheta(&Equation_CloversTripleTwist, (float) ballSpeedMMperSec, finalThetaFromLastPlot, 
    startingTheta, endingTheta, endpointSpacingMM, endpointSpacingTolerance, clipPlotToSandboxRadiusFlag,
    &finalThetaFromLastPlot));
}


// -------------------------------------- End --------------------------------------
