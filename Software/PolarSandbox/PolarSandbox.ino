//      ******************************************************************
//      *                                                                *
//      *                          Polar Sandbox                         *
//      *                                                                *
//      *            Stan Reifel                     10/26/2019          *
//      *               Copyright (c) S. Reifel & Co,  2019              *
//      *                                                                *
//      ******************************************************************

#include <EEPROM.h>
#include "Constants.h"
#include "SettingsToolsCmd.h"
#include "Drawings.h"
#include "SupportFuncs.h"
#include <TeensyUserInterface.h>
#include <font_Arial.h>
#include <font_ArialBold.h>
#include "TeensyCoordinatedStepper.h"


//
// create the TeensyUserInterface object
//
TeensyUserInterface ui;


//
// create the coordinated motion stepper driver object
//
TeensyCoordinatedStepper steppers;


//
// globals
//
const int defaultBallSpeedMMperSec = 30;
int ballSpeedMMperSec = defaultBallSpeedMMperSec;
boolean spiralDirectionBtn = SPIRAL_OUT;
byte ledBacklightBrightness = 0;


//
// forward declarations for the menus & sub menus
//
extern MENU_ITEM mainMenu[];
extern MENU_ITEM singleDrawingsMenu[];
extern MENU_ITEM sprialDrawingsMenu[];
extern MENU_ITEM radialDrawingsMenu[];
extern MENU_ITEM spirographDrawingsMenu[];
extern MENU_ITEM settingsAndToolsMenu[];
extern MENU_ITEM toolsMenu[];


//
// forward function declarations
//
void runForeverCommand(void);
void spiralDirectionCallback(void);
void lcdPlotCallback(void);

void spiralCommand(void);
void spiralHDCommand(void);
void trianglesClippedCommand(void);
void hexagonsCommand(void);
void heartsCommand(void);
void polygonSuperTwistCommand(void);
void cloversSuperTwistCommand(void);
void cloversTripleTwistClippedCommand(void);
void scalopsTwistedCommand(void);
void scalopsTripleTwistCommand(void);
void sunRaysCommand(void);
void hotSunCommand(void);
void psychoCommand(void);
void spiralRaysCommand(void);
void loopsCommand(void);
void perimeterSkirtCommand(void);
void sixBladeCommand(void);
void narrowingSpirographCommand(void);



//
// the main menu
//
MENU_ITEM mainMenu[] = {
  {MENU_ITEM_TYPE_MAIN_MENU_HEADER,  "Polar Sandbox",        MENU_COLUMNS_1,      mainMenu},
  {MENU_ITEM_TYPE_COMMAND,           "Run forever",          runForeverCommand,   NULL},
  {MENU_ITEM_TYPE_SUB_MENU,          "Single drawings",      NULL,                singleDrawingsMenu},
  {MENU_ITEM_TYPE_SUB_MENU,          "Settings and tools",   NULL,                settingsAndToolsMenu},
  {MENU_ITEM_TYPE_END_OF_MENU,       "",                     NULL,                NULL}
};



//
// menu to select Single Drawing
//
MENU_ITEM singleDrawingsMenu[] = {
  {MENU_ITEM_TYPE_SUB_MENU_HEADER,   "Single Drawings",       MENU_COLUMNS_1,     mainMenu},
  {MENU_ITEM_TYPE_SUB_MENU,          "Spiral drawings",       NULL,               sprialDrawingsMenu},
  {MENU_ITEM_TYPE_SUB_MENU,          "Radial drawings",       NULL,               radialDrawingsMenu},
  {MENU_ITEM_TYPE_SUB_MENU,          "Spirograph drawings",   NULL,               spirographDrawingsMenu},
  {MENU_ITEM_TYPE_END_OF_MENU,       "",                      NULL,               NULL}
};


//
// menu of Spiral Drawings singles
//
MENU_ITEM sprialDrawingsMenu[] = {
  {MENU_ITEM_TYPE_SUB_MENU_HEADER,   "Spiral Drawings",       MENU_COLUMNS_2,                    singleDrawingsMenu},
  {MENU_ITEM_TYPE_COMMAND,           "Spiral",                spiralCommand,                     NULL},
  {MENU_ITEM_TYPE_COMMAND,           "Spiral HD",             spiralHDCommand,                   NULL},
  {MENU_ITEM_TYPE_COMMAND,           "Hexagons",              hexagonsCommand,                   NULL},
  {MENU_ITEM_TYPE_COMMAND,           "Triangles clipped",     trianglesClippedCommand,           NULL},
  {MENU_ITEM_TYPE_COMMAND,           "Hearts",                heartsCommand,                     NULL},
  {MENU_ITEM_TYPE_COMMAND,           "Polygon super twist",   polygonSuperTwistCommand,          NULL},
  {MENU_ITEM_TYPE_COMMAND,           "Clovers super twist",   cloversSuperTwistCommand,          NULL},
  {MENU_ITEM_TYPE_COMMAND,           "Clovers triple twist",  cloversTripleTwistClippedCommand,  NULL},
  {MENU_ITEM_TYPE_COMMAND,           "Scalops twisted",       scalopsTwistedCommand,             NULL},
  {MENU_ITEM_TYPE_COMMAND,           "Scalops triple twist",  scalopsTripleTwistCommand,         NULL},
  {MENU_ITEM_TYPE_TOGGLE,            "Spiral direction",      spiralDirectionCallback,           NULL},
  {MENU_ITEM_TYPE_END_OF_MENU,       "",                      NULL,                              NULL}
};


//
// menu of Radial Drawings singles
//
MENU_ITEM radialDrawingsMenu[] = {
  {MENU_ITEM_TYPE_SUB_MENU_HEADER,   "Radial Drawings",       MENU_COLUMNS_1,            singleDrawingsMenu},
  {MENU_ITEM_TYPE_COMMAND,           "Sun rays",              sunRaysCommand,            NULL},
  {MENU_ITEM_TYPE_COMMAND,           "Hot sun",               hotSunCommand,             NULL},
  {MENU_ITEM_TYPE_COMMAND,           "Psycho",                psychoCommand,             NULL},
  {MENU_ITEM_TYPE_COMMAND,           "Spiral rays",           spiralRaysCommand,         NULL},
  {MENU_ITEM_TYPE_END_OF_MENU,       "",                      NULL,                      NULL}
};


//
// menu of Spirograph Drawings singles
//
MENU_ITEM spirographDrawingsMenu[] = {
  {MENU_ITEM_TYPE_SUB_MENU_HEADER,   "Spirograph Drawings",   MENU_COLUMNS_1,              singleDrawingsMenu},
  {MENU_ITEM_TYPE_COMMAND,           "Loops",                 loopsCommand,                NULL},
  {MENU_ITEM_TYPE_COMMAND,           "Perimeter skirt",       perimeterSkirtCommand,       NULL},
  {MENU_ITEM_TYPE_COMMAND,           "Six blades",            sixBladeCommand,             NULL},
  {MENU_ITEM_TYPE_COMMAND,           "Narrowing spirograph",  narrowingSpirographCommand,  NULL},
  {MENU_ITEM_TYPE_END_OF_MENU,       "",                      NULL,                        NULL}
};




//
// the Settings and Tools menu
//
MENU_ITEM settingsAndToolsMenu[] = {
  {MENU_ITEM_TYPE_SUB_MENU_HEADER,   "Settings and Tools",             MENU_COLUMNS_2,                mainMenu},
  {MENU_ITEM_TYPE_COMMAND,           "Set ball speed...",              setBallSpeedCommand,           NULL},
  {MENU_ITEM_TYPE_COMMAND,           "Set lighting brightness...",     setLightingBrightnessCommand,  NULL},
  {MENU_ITEM_TYPE_COMMAND,           "Display runtime...",             displayRuntimeCommand,         NULL},
  {MENU_ITEM_TYPE_COMMAND,           "Find the ball",                  findTheBallCommand,            NULL},
  {MENU_ITEM_TYPE_COMMAND,           "Move sand inward",               moveSandInwardCommand,         NULL},
  {MENU_ITEM_TYPE_COMMAND,           "Move to home",                   homeThetaAndRCommand,          NULL},
  {MENU_ITEM_TYPE_END_OF_MENU,       "",                               NULL,                          NULL}
};



// ---------------------------------------------------------------------------------
//                              Hardware and software setup
// ---------------------------------------------------------------------------------

//
// top level setup function
//
void setup()
{
  //
  // setup the LCD and UI
  //
  ui.begin(LCD_ORIENTATION_LANDSCAPE_4PIN_RIGHT, Arial_9_Bold);

  //
  // setup the stepper driver
  //
  steppers.connectToPins(MOTOR_RADIUS_STEP_PIN, MOTOR_RADIUS_DIR_PIN, false, MOTOR_THETA_STEP_PIN, MOTOR_THETA_DIR_PIN, false);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);                       // turn off the LED

  pinMode(MOTOR_ENABLE_PIN, OUTPUT); 
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);             // disable the steppers

  pinMode(LED_LIGHTING_PIN, OUTPUT); 
  digitalWrite(LED_LIGHTING_PIN, LOW);              // turn off the LED lighting

  pinMode(MOTOR_THETA_HOME_PIN, INPUT_PULLUP);      // configure the homing pins with pull up resistors
  pinMode(MOTOR_RADIUS_HOME_PIN, INPUT_PULLUP); 
  pinMode(TEST_1_PIN, OUTPUT);

  setMicroSteppingTMC2208(MICRO_STEPPING);

  steppers.setJunctionDeviation(150 * MICRO_STEPPING);
  steppers.setAcceleration(200.0 * MICRO_STEPPING);

  //
  // setup everything else
  //
  Serial.begin(38400);
  delay(100);                       // delay needed to make the serial work
  Serial.println("Serial started");


  //
  // set the LED backlight brightness by fading up
  //
  ledBacklightBrightness = ui.readConfigurationByte(EEPROM_LED_BACKLIGHT_BRIGHTNESS, 100);
  for (int i = 0; i <= ledBacklightBrightness; i++)
  {
    analogWrite(LED_LIGHTING_PIN, i);
    delay(6);
  }


  //
  // set ball speed in mm/sec and read the current runtime
  //
  ballSpeedMMperSec = ui.readConfigurationInt(EEPROM_BALL_SPEED, defaultBallSpeedMMperSec);
  readRunningTimeFromEEPROM();


  //
  // home the R and Theta axes
  //
  homeThetaAndRCommand();
}


// ---------------------------------------------------------------------------------
//                      The Main Loop and Top Level Functions
// ---------------------------------------------------------------------------------

//
// main loop, execute the UI's menus
//
void loop()
{
  ui.displayAndExecuteMenu(mainMenu);
}


// ---------------------------------------------------------------------------------
//                                  Menu Commands
// ---------------------------------------------------------------------------------

//
// "Run forever" command
//
void runForeverCommand(void)
{ 
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                            // enable the steppers
  startMeasuringRunningTime();                                    // start the clock for measuring total runtime
 
  while(true)
  {
    if (drawSpiral(SPIRAL_OUT) == CANCEL_DRAWING)
      break;
    
    if (drawStarFlower(SPIRAL_IN) == CANCEL_DRAWING)              // This looks good before hexagons
      break;
   
    if (drawHexagons(SPIRAL_OUT) == CANCEL_DRAWING)               // This looks good after starFlower
      break;

    saveRunningTimeInEEPROM();
       
    if (drawClovers(SPIRAL_IN) == CANCEL_DRAWING)
      break;

    if (drawSquaresClipped(SPIRAL_OUT) == CANCEL_DRAWING)
      break;

    if (drawPetals(SPIRAL_IN) == CANCEL_DRAWING)                  // This has no twist
      break;

    saveRunningTimeInEEPROM();

    if (drawScalopsTwistedClipped(SPIRAL_OUT) == CANCEL_DRAWING)  // This is a good one
      break;

    if (drawMoveSandInward() == CANCEL_DRAWING)                   // From OUT to OUT, clean up outside perimeter, move a bit of the sand inward
      break;                                                     // this drawing has the same appearance as "Sun Rays"

    if (drawSpiderWeb(SPIRAL_IN) == CANCEL_DRAWING)               // this erases everything under it
      break;

    saveRunningTimeInEEPROM();

    if (drawStarsTwisted(SPIRAL_OUT) == CANCEL_DRAWING)
      break;

    if (drawHotSun(START_OUT_END_OUT) == CANCEL_DRAWING)          // From OUT to OUT, this erases everything under it
      break;
                                                                  
    if (drawNarrowingSpirograph() == CANCEL_DRAWING)              // From OUT to OUT, looks good after drawHotSun()
      break;

    saveRunningTimeInEEPROM();

    if (drawLittleCloversTwisted(SPIRAL_IN) == CANCEL_DRAWING)    // this erases everything under it
      break;

    if (drawPsycho(START_IN_END_IN) == CANCEL_DRAWING)            // From IN to IN
      break;

    if (drawCloversSuperTwist(SPIRAL_OUT) == CANCEL_DRAWING)      // Good one, looks good after after a radial drawing         
      break;

    saveRunningTimeInEEPROM();

    if (drawSpiralHD(SPIRAL_IN) == CANCEL_DRAWING)                // From OUT to IN, this erases everything under it
      break;

    if (drawSpiralRays(START_IN_END_IN) == CANCEL_DRAWING)
      break;

    if (drawHearts() == CANCEL_DRAWING)                           // From IN to OUT 
      break;

    saveRunningTimeInEEPROM();

    if (drawPerimeterSkirt() == CANCEL_DRAWING)                   // From OUT to OUT 
      break;

    if (drawScalopsTripleTwistClipped(SPIRAL_IN) == CANCEL_DRAWING)
      break;
                                                                  // 3:11 to here (at 40mm/s, format h:mm)
    if (drawTrianglesClipped(SPIRAL_OUT) == CANCEL_DRAWING)       // Better to spiral out, looks great clipped
      break;

    saveRunningTimeInEEPROM();

    if (drawLoops() == CANCEL_DRAWING)                            // From OUT to OUT, OK but not great to watch, looks interesting when done
      break;

    if (drawLittleClovers(SPIRAL_IN) == CANCEL_DRAWING) 
      break;
                       
    if (drawCloversTwisted(SPIRAL_OUT) == CANCEL_DRAWING)
      break;

    saveRunningTimeInEEPROM();

    if (drawStars() == CANCEL_DRAWING)                            // From OUT to IN, starting with a polygon, erases everything under
      break;

    if (drawSpiral(SPIRAL_OUT) == CANCEL_DRAWING)                 // From IN to OUT, Spiral needed for drawSixBlade()
      break;
    if (drawSixBlade() == CANCEL_DRAWING)                         // From OUT to OUT, this one is just OK
      break;

    saveRunningTimeInEEPROM();

    if (drawCloversTripleTwistClipped(SPIRAL_IN) == CANCEL_DRAWING) // good, looks similar to, but different from drawScalopsTripleTwistClipped()
      break;

    if (drawPolygonSuperTwist(SPIRAL_OUT) == CANCEL_DRAWING)      // this is good in or out, may want to add clipping
      break;
       
    if (drawLooseSpiral(SPIRAL_IN) == CANCEL_DRAWING)             // Return to the beginning
       break;

      saveRunningTimeInEEPROM();
  }

  stopMeasuringRunningTime();                                     // stop recording the running time
  saveRunningTimeInEEPROM();
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                           // disable the steppers
}




//
// "Spiral" command
//
void spiralCommand(void)
{ 
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                    // enable the steppers
  startMeasuringRunningTime();                            // start the clock for recording total runtime

  drawSpiral(spiralDirectionBtn);                         // draw the figure

  stopMeasuringRunningTime();                             // stop recording the running time
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                   // disable the steppers
}



//
// "Spiral with tight pitch" command
//
void spiralHDCommand(void)
{ 
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                    // enable the steppers
  startMeasuringRunningTime();                            // start the clock for recording total runtime

  drawSpiralHD(spiralDirectionBtn);                       // draw the figure

  stopMeasuringRunningTime();                             // stop recording the running time
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                   // disable the steppers
}



//
// "Trangles - Clipped" command
//
void trianglesClippedCommand(void)
{ 
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                    // enable the steppers
  startMeasuringRunningTime();                            // start the clock for recording total runtime

  drawTrianglesClipped(spiralDirectionBtn);               // draw the figure

  stopMeasuringRunningTime();                             // stop recording the running time
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                   // disable the steppers
}



//
// "Hexagons" command
//
void hexagonsCommand(void)
{ 
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                    // enable the steppers
  startMeasuringRunningTime();                            // start the clock for recording total runtime

  drawHexagons(spiralDirectionBtn);                       // draw the figure

  stopMeasuringRunningTime();                             // stop recording the running time
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                   // disable the steppers
}



//
// "Hearts" command
//
void heartsCommand(void)
{ 
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                    // enable the steppers
  startMeasuringRunningTime();                            // start the clock for recording total runtime

  drawHearts();                                           // draw the figure

  stopMeasuringRunningTime();                             // stop recording the running time
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                   // disable the steppers
}



//
// "Polygon Super Twist" command
//
void polygonSuperTwistCommand(void)
{ 
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                    // enable the steppers
  startMeasuringRunningTime();                            // start the clock for recording total runtime

  drawPolygonSuperTwist(spiralDirectionBtn);              // draw the figure

  stopMeasuringRunningTime();                             // stop recording the running time
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                   // disable the steppers
}



//
// "Clovers Super Twist" command
//
void cloversSuperTwistCommand(void)
{ 
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                    // enable the steppers
  startMeasuringRunningTime();                            // start the clock for recording total runtime

  drawCloversSuperTwist(spiralDirectionBtn);              // draw the figure

  stopMeasuringRunningTime();                             // stop recording the running time
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                   // disable the steppers
}



//
// "Clovers Triple Twist" command
//
void cloversTripleTwistClippedCommand(void)
{ 
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                    // enable the steppers
  startMeasuringRunningTime();                            // start the clock for recording total runtime

  drawCloversTripleTwistClipped(spiralDirectionBtn);      // draw the figure

  stopMeasuringRunningTime();                             // stop recording the running time
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                   // disable the steppers
}



//
// "Scalops Twisted" command
//
void scalopsTwistedCommand(void)
{ 
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                    // enable the steppers
  startMeasuringRunningTime();                            // start the clock for recording total runtime

  drawScalopsTwisted(spiralDirectionBtn);                 // draw the figure

  stopMeasuringRunningTime();                             // stop recording the running time
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                   // disable the steppers
}



//
// "Scalops Triple Twist" command
//
void scalopsTripleTwistCommand(void)
{ 
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                    // enable the steppers
  startMeasuringRunningTime();                            // start the clock for recording total runtime

  drawScalopsTripleTwist(spiralDirectionBtn);             // draw the figure

  stopMeasuringRunningTime();                             // stop recording the running time
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                   // disable the steppers
}



//
// "Sun Rays - Zen" command
//
void sunRaysCommand(void)
{ 
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                    // enable the steppers
  startMeasuringRunningTime();                            // start the clock for recording total runtime

  drawSunRays(START_IN_END_IN);                           // draw the figure

  stopMeasuringRunningTime();                             // stop recording the running time
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                   // disable the steppers
}



//
// "Hot Sun" command
//
void hotSunCommand(void)
{ 
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                    // enable the steppers
  startMeasuringRunningTime();                            // start the clock for recording total runtime

  drawHotSun(START_IN_END_IN);                            // draw the figure

  stopMeasuringRunningTime();                             // stop recording the running time
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                   // disable the steppers
}



//
// "Sun Rays - Psycho" command
//
void psychoCommand(void)
{ 
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                    // enable the steppers
  startMeasuringRunningTime();                            // start the clock for recording total runtime

  drawPsycho(START_IN_END_IN);                            // draw the figure

  stopMeasuringRunningTime();                             // stop recording the running time
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                   // disable the steppers
}



//
// "Spiral rays" command
//
void spiralRaysCommand(void)
{ 
  //
  // update the LCD screen and enable the steppers
  //
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                    // enable the steppers
  startMeasuringRunningTime();                            // start the clock for recording total runtime

  drawSpiralRays(START_IN_END_IN);                        // draw the figure

  stopMeasuringRunningTime();                             // stop recording the running time
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                   // disable the steppers
}



//
// "Loops" command
//
void loopsCommand(void)
{ 
  //
  // update the LCD screen and enable the steppers
  //
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                    // enable the steppers
  startMeasuringRunningTime();                            // start the clock for recording total runtime

  drawLoops();                                            // draw the figure

  stopMeasuringRunningTime();                             // stop recording the running time
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                   // disable the steppers
}



//
// "Perimeter Skirt" command
//
void perimeterSkirtCommand(void)
{ 
  //
  // update the LCD screen and enable the steppers
  //
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                    // enable the steppers
  startMeasuringRunningTime();                            // start the clock for recording total runtime

  drawPerimeterSkirt();                                   // draw the figure

  stopMeasuringRunningTime();                             // stop recording the running time
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                   // disable the steppers
}



//
// "Six Blade" command
//
void sixBladeCommand(void)
{ 
  //
  // update the LCD screen and enable the steppers
  //
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                    // enable the steppers
  startMeasuringRunningTime();                            // start the clock for recording total runtime

  drawSixBlade();                                         // draw the figure

  stopMeasuringRunningTime();                             // stop recording the running time
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                   // disable the steppers
}



//
// "Narrowing Spirograph" command
//
void narrowingSpirographCommand(void)
{ 
  //
  // update the LCD screen and enable the steppers
  //
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                    // enable the steppers
  startMeasuringRunningTime();                            // start the clock for recording total runtime

  drawNarrowingSpirograph();                              // draw the figure

  stopMeasuringRunningTime();                             // stop recording the running time
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                   // disable the steppers
}



//
// set the spiral direction: In or Out
//
void spiralDirectionCallback(void)
{
  //
  // check if menu is requesting that the state be changed (can have more than 2 states)
  //
  if (ui.toggleSelectNextStateFlg)
  {
    //
    // select the next state
    //
    spiralDirectionBtn = !spiralDirectionBtn;
  }
 
  //
  // send back an indication of the current state
  //
  if(spiralDirectionBtn)
    ui.toggleText = "Out";
  else
    ui.toggleText = "In";
}


// -------------------------------------- End --------------------------------------
