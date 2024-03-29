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
byte onPowerbehavior;


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
void sinewaveTwistCommand(void);
void sinewaveHDCommand(void);
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
  {MENU_ITEM_TYPE_COMMAND,           "Sinewave HD",           sinewaveHDCommand,                 NULL},
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
  {MENU_ITEM_TYPE_COMMAND,           "Settings...",                    settingsCommand,               NULL},
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
  ui.begin(LCD_CS_PIN, LCD_DC_PIN, TOUCH_CS_PIN, LCD_ORIENTATION_LANDSCAPE_4PIN_RIGHT, Arial_9_Bold);

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
  // set what happens on Power Up
  //
  onPowerbehavior = ui.readConfigurationByte(EEPROM_ON_POWERUP_BEHAVIOR, ON_POWERUP_GOTO_MAIN_MENU);

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
  //
  // check if configured for "auto run"
  //
  if (onPowerbehavior)
  {
    //
    // for "auto run", start drawing with a random block number
    //
    delay(5);
    int startingBlockNumber = (analogRead(A0) % (8 + 1));
    runDrawingsForever(startingBlockNumber);
  }

  //
  // enter the menu system, then wait for the user to run commands
  //
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
  int startingBlockNumber = 0;
  runDrawingsForever(startingBlockNumber);
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
// "sinewave twist" command
//
void sinewaveTwistCommand(void)
{ 
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                    // enable the steppers
  startMeasuringRunningTime();                            // start the clock for recording total runtime

  drawSineTwisted(spiralDirectionBtn);                    // draw the figure

  stopMeasuringRunningTime();                             // stop recording the running time
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                   // disable the steppers
}



//
// "sinewave HD" command
//
void sinewaveHDCommand(void)
{ 
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                    // enable the steppers
  startMeasuringRunningTime();                            // start the clock for recording total runtime

  drawSinewaveHD(spiralDirectionBtn);                     // draw the figure

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
