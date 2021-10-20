//      ******************************************************************
//      *                                                                *
//      *                 Settins and Tools Menu Commands                *
//      *                                                                *
//      *               Copyright (c) S. Reifel & Co,  2019              *
//      *                                                                *
//      ******************************************************************

#include <arduino.h>
#include "Constants.h"
#include "Drawings.h"
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
extern byte ledBacklightBrightness;
extern int ballSpeedMMperSec;
extern byte onPowerbehavior;


//
// forward function declarations
//


// ---------------------------------------------------------------------------------
//                                Settings Commands
// ---------------------------------------------------------------------------------


//
// Settings command: LED brightness, Ball speed, Auto startup on powerup
//
void settingsCommand(void)
{  
  int selectionValue;
  const int brightnessNumberboxY = ui.displaySpaceTopY + 38;
  const int numberBoxHeight = 26;
  const int numberBoxWidth = ui.displaySpaceWidth - 28;
  const int ballspeedSelectionBoxY = brightnessNumberboxY + numberBoxHeight + 32;
  const int selectionBoxHeight = numberBoxHeight;
  const int selectionBoxWidth = numberBoxWidth;
  const int autoStartOnPowerupSelectionBoxY = ballspeedSelectionBoxY + selectionBoxHeight + 32;
  const int ButtonsY = ui.displaySpaceBottomY - 15;
  const int buttonHeight = numberBoxHeight;
  const int buttonWidth = 100;

  
  //
  // draw title bar without a "Back" button
  //
  ui.drawTitleBar("Settings");
  ui.clearDisplaySpace();

  //
  // define and display number boxes for setting LED brightness
  //  
  NUMBER_BOX LEDBrightness_NumberBox;
  LEDBrightness_NumberBox.labelText    = "Set LED brightness:";
  LEDBrightness_NumberBox.value        = ledBacklightBrightness;
  LEDBrightness_NumberBox.minimumValue = 0;
  LEDBrightness_NumberBox.maximumValue = 255;
  LEDBrightness_NumberBox.stepAmount   = 1;
  LEDBrightness_NumberBox.centerX      = ui.displaySpaceCenterX;
  LEDBrightness_NumberBox.centerY      = brightnessNumberboxY;
  LEDBrightness_NumberBox.width        = numberBoxWidth;
  LEDBrightness_NumberBox.height       = numberBoxHeight;
  ui.drawNumberBox(LEDBrightness_NumberBox);

  
  //
  // convert the current ball speed into a "selection box" value
  //
  switch(ballSpeedMMperSec)
  {
    case 20:
      selectionValue = 0;
      break;
    case 25:
      selectionValue = 1;
      break;
    case 30:
      selectionValue = 2;
      break;
    default:
      selectionValue = 3;
      break;
  }

  //
  // define and display the "ball speed" selection box with 4 choices
  //
  SELECTION_BOX ballSpeedSelectionBox;
  ballSpeedSelectionBox.labelText       = "Set ball speed (mm/sec):";
  ballSpeedSelectionBox.value           = selectionValue;            // set default value, 0 is 1st choice
  ballSpeedSelectionBox.choice0Text     = "20";
  ballSpeedSelectionBox.choice1Text     = "25";
  ballSpeedSelectionBox.choice2Text     = "30";
  ballSpeedSelectionBox.choice3Text     = "40";
  ballSpeedSelectionBox.centerX         = ui.displaySpaceCenterX;
  ballSpeedSelectionBox.centerY         = ballspeedSelectionBoxY;
  ballSpeedSelectionBox.width           = selectionBoxWidth;
  ballSpeedSelectionBox.height          = selectionBoxHeight;
  ui.drawSelectionBox(ballSpeedSelectionBox);                       // display the Selection Box

  //
  // draw text below the selection box
  //
  ui.lcdSetCursorXY(ui.displaySpaceCenterX - 141, ballspeedSelectionBoxY - 23);
  ui.lcdPrint("Quieter");
  ui.lcdSetCursorXY(ui.displaySpaceCenterX + 139, ballspeedSelectionBoxY - 23);
  ui.lcdPrintRightJustified("Louder");

  //
  // define and display the "Auto Start on Powerup" selection box with 2 choices
  //

  SELECTION_BOX autoStartupSelectionBox;
  autoStartupSelectionBox.labelText       = "Set what happens on powerup:";
  autoStartupSelectionBox.value           = onPowerbehavior;            // set default value
  autoStartupSelectionBox.choice0Text     = "Go to main menu";
  autoStartupSelectionBox.choice1Text     = "Run the ball";
  autoStartupSelectionBox.choice2Text     = "";
  autoStartupSelectionBox.choice3Text     = "";
  autoStartupSelectionBox.centerX         = ui.displaySpaceCenterX;
  autoStartupSelectionBox.centerY         = autoStartOnPowerupSelectionBoxY;
  autoStartupSelectionBox.width           = selectionBoxWidth;
  autoStartupSelectionBox.height          = selectionBoxHeight;
  ui.drawSelectionBox(autoStartupSelectionBox);                       // display the Selection Box


  //
  // define and display "OK" and "Cancel" buttons
  //

  BUTTON okButton       = {"OK",     ui.displaySpaceCenterX - buttonWidth/2 - 8, ButtonsY, buttonWidth , buttonHeight};
  ui.drawButton(okButton);

  BUTTON cancelButton   = {"Cancel", ui.displaySpaceCenterX + buttonWidth/2 + 8, ButtonsY, buttonWidth , buttonHeight};
  ui.drawButton(cancelButton);


  //
  // process touch events
  //
  while(true)
  {
    ui.getTouchEvents();

    //
    // process touch events on the Number Boxes
    //
    if (ui.checkForNumberBoxTouched(LEDBrightness_NumberBox))
      analogWrite(LED_LIGHTING_PIN, LEDBrightness_NumberBox.value);

    
    //
    // proces touch events for the selection boxes
    //
    ui.checkForSelectionBoxTouched(ballSpeedSelectionBox);     // process "ball speed" touches
    ui.checkForSelectionBoxTouched(autoStartupSelectionBox);   // process "auto start" touches

    //
    // check if the "OK" button pressed
    //
    if (ui.checkForButtonClicked(okButton))
    {
      //
      // determine the ball speed from the selection box, then convert to mm/sec and save in EEPROM
      //
      switch(ballSpeedSelectionBox.value)
      {
        case 0:
          ballSpeedMMperSec = 20;
          break;
        case 1:
          ballSpeedMMperSec = 25;
          break;
        case 2:
          ballSpeedMMperSec = 30;
          break;
        default:
          ballSpeedMMperSec = 40;
          break;
      }
      ui.writeConfigurationInt(EEPROM_BALL_SPEED, ballSpeedMMperSec);

      //
      // save the LED brightness in EEPROM
      //
      ledBacklightBrightness = LEDBrightness_NumberBox.value;
      ui.writeConfigurationByte(EEPROM_LED_BACKLIGHT_BRIGHTNESS, ledBacklightBrightness);

      //
      // save what is configured to happen after Powerup
      //
      onPowerbehavior = autoStartupSelectionBox.value;
      ui.writeConfigurationByte(EEPROM_ON_POWERUP_BEHAVIOR, onPowerbehavior);
      return;
    }

    //
    // check if the "Cancel" button pressed
    //
    if (ui.checkForButtonClicked(cancelButton))
    {
      //
      // cancel pressed, restore originial LED brightness
      //
      analogWrite(LED_LIGHTING_PIN, ledBacklightBrightness);
      return;
    }
  }
}



////
//// set the brightness for the LEDs
////
//void setLightingBrightnessCommand(void)
//{  
//  //
//  // draw title bar without a "Back" button
//  //
//  ui.drawTitleBar("Set Lighting Brightness");
//  ui.clearDisplaySpace();
//
//  //
//  // define and display number boxes for setting LED brightness
//  //
//  const int numberBoxHeight = 40;
//  
//  NUMBER_BOX LEDBrightness_NumberBox;
//  LEDBrightness_NumberBox.labelText    = "Set brightness";
//  LEDBrightness_NumberBox.value        = ledBacklightBrightness;
//  LEDBrightness_NumberBox.minimumValue = 0;
//  LEDBrightness_NumberBox.maximumValue = 255;
//  LEDBrightness_NumberBox.stepAmount   = 1;
//  LEDBrightness_NumberBox.centerX      = ui.displaySpaceCenterX;
//  LEDBrightness_NumberBox.centerY      = ui.displaySpaceCenterY - 20;
//  LEDBrightness_NumberBox.width        = ui.displaySpaceWidth - 50;
//  LEDBrightness_NumberBox.height       = numberBoxHeight;
//  ui.drawNumberBox(LEDBrightness_NumberBox);
//
//  //
//  // define and display "OK" button
//  //
//  const int buttonHeight = numberBoxHeight;
//  const int buttonWidth = 100;
//  BUTTON okButton        = {"OK",  ui.displaySpaceCenterX,   ui.displaySpaceBottomY-35,  buttonWidth, buttonHeight};
//  ui.drawButton(okButton);
//
//
//  //
//  // process touch events
//  //
//  while(true)
//  {
//    ui.getTouchEvents();
//
//    //
//    // process touch events on the Number Boxes
//    //
//    if (ui.checkForNumberBoxTouched(LEDBrightness_NumberBox))
//    {
//      ledBacklightBrightness = LEDBrightness_NumberBox.value;
//      analogWrite(LED_LIGHTING_PIN, ledBacklightBrightness);
//    }
//
//    //
//    // check for touch events on the "OK" button
//    //
//    if (ui.checkForButtonClicked(okButton))
//    {
//      ui.writeConfigurationByte(EEPROM_LED_BACKLIGHT_BRIGHTNESS, ledBacklightBrightness);
//      return;
//    }
//  }
//}




////
//// set the ball speed (slower is more quiet)
////
//void setBallSpeedCommand(void)
//{  
//  int selectionValue;
//  
//  ui.drawTitleBar("Set the Speed of the Ball");
//  ui.clearDisplaySpace();
//
//  //
//  // convert the current ball speed into a "selection box" value
//  //
//  switch(ballSpeedMMperSec)
//  {
//    case 20:
//      selectionValue = 0;
//      break;
//    case 25:
//      selectionValue = 1;
//      break;
//    case 30:
//      selectionValue = 2;
//      break;
//    default:
//      selectionValue = 3;
//      break;
//  }
//
//
//  //
//  // define and display a selection box with 4 choices
//  //
//  const int selectionBoxY = ui.displaySpaceCenterY - 20;
//  
//  SELECTION_BOX ballSpeedSelectionBox;
//  ballSpeedSelectionBox.labelText = "Ball speed in mm/sec";
//  ballSpeedSelectionBox.value = selectionValue;            // set default value, 0 is 1st choice
//  ballSpeedSelectionBox.choice0Text = "20";
//  ballSpeedSelectionBox.choice1Text = "25";
//  ballSpeedSelectionBox.choice2Text = "30";
//  ballSpeedSelectionBox.choice3Text = "40";
//  ballSpeedSelectionBox.centerX = ui.displaySpaceCenterX;
//  ballSpeedSelectionBox.centerY = selectionBoxY;
//  ballSpeedSelectionBox.width = 250;
//  ballSpeedSelectionBox.height = 33;
//  ui.drawSelectionBox(ballSpeedSelectionBox);          // display the Selection Box
//
//  //
//  // draw text below the selection box
//  //
//  ui.lcdSetCursorXY(ui.displaySpaceCenterX - 120, selectionBoxY + 22);
//  ui.lcdPrint("Quieter");
//  ui.lcdSetCursorXY(ui.displaySpaceCenterX + 120, selectionBoxY + 22);
//  ui.lcdPrintRightJustified("Louder");
//
//  //
//  // define and display "OK" and "Cancel" buttons
//  //
//  const int ButtonsY = ui.displaySpaceBottomY - 35;
//  const int buttonHeight = 40;
//  const int buttonWidth = 100;
//
//  BUTTON okButton       = {"OK",     ui.displaySpaceCenterX - buttonWidth/2 - 8, ButtonsY, buttonWidth , buttonHeight};
//  ui.drawButton(okButton);
//
//  BUTTON cancelButton   = {"Cancel", ui.displaySpaceCenterX + buttonWidth/2 + 8, ButtonsY, buttonWidth , buttonHeight};
//  ui.drawButton(cancelButton);
//
//
//  //
//  // process touch events
//  //
//  while(true)
//  {
//    ui.getTouchEvents();
//    ui.checkForSelectionBoxTouched(ballSpeedSelectionBox);  // process SelectionBox touches
//
//    
//    //
//    // check for touch events on the "OK" button
//    //
//    if (ui.checkForButtonClicked(okButton))
//    {
//      //
//      // determine the ball speed from the selection box, then convert to mm/sec
//      //
//      switch(ballSpeedSelectionBox.value)
//      {
//        case 0:
//          ballSpeedMMperSec = 20;
//          break;
//        case 1:
//          ballSpeedMMperSec = 25;
//          break;
//        case 2:
//          ballSpeedMMperSec = 30;
//          break;
//        default:
//          ballSpeedMMperSec = 40;
//          break;
//      }
//      ui.writeConfigurationInt(EEPROM_BALL_SPEED, ballSpeedMMperSec);
//      return;
//    }
//
//    //
//    // check for touch events on the "Cancel" button
//    //
//    if (ui.checkForButtonClicked(cancelButton))
//      return;
//  }
//}

// ---------------------------------------------------------------------------------
//                            Commands in the Tools menu
// ---------------------------------------------------------------------------------

//
// "Home the Theta and R axes" command
//
void homeThetaAndRCommand(void)
{ 
  //
  // redraw the screen to let the user know it is plotting now
  //
  ui.drawTitleBar("Homing Theta and R");
  ui.clearDisplaySpace();
  ui.lcdSetCursorXY(ui.displaySpaceCenterX, ui.displaySpaceCenterY - 15);
  ui.lcdPrintCentered("Finding the home coordinate...");
  
  //
  // enable the stepper motors
  //
  digitalWrite(MOTOR_ENABLE_PIN, LOW);

  //
  // home the axes and move to the home position
  //
  homeAllAxes();

  //
  // disable the stepper motors
  //
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
}



//
// move sand from the outside in
//
void moveSandInwardCommand(void)
{ 
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                    // enable the steppers
  drawMoveSandInward();                                   // draw the figure
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                   // disable the steppers
}





//
// find the ball command (move in a spiral from the outside in)
//
void findTheBallCommand(void)
{ 
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                            // enable the steppers
  drawSpiral(SPIRAL_IN);
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                           // disable the steppers
}



//
// display total runtime
//
void displayRuntimeCommand(void)
{
  char s[40];
  
  ui.drawTitleBar("Total Accumulated Runtime");
  ui.clearDisplaySpace();

  //
  // compute the runtime in days, hours and minutes
  //
  unsigned long runtime = getTotalRunningTimeInSeconds();
  
  unsigned long secondsPerDay = 24L * 60L * 60L;
  int days = (int) (runtime / secondsPerDay); 
  runtime -= (long) days * secondsPerDay;
  
  unsigned long secondsPerHour = 60L * 60L;
  int hours = (int) (runtime / secondsPerHour); 
  runtime -= (long) hours * secondsPerHour;
  
  int minutes = (int) (runtime / 60); 
  int seconds = runtime - (long) minutes * 60;


  //
  // build a string in the format DD:HH:MM:SS
  //
  sprintf(s, "%dD : %02dH : %02dM : %02dS", days, hours, minutes, seconds);
  ui.lcdSetCursorXY(ui.displaySpaceCenterX, ui.displaySpaceCenterY - 25);
  ui.lcdPrintCentered(s);
  
  
  //
  // define and display "OK" button
  //
  const int ButtonsY = ui.displaySpaceBottomY - 35;
  const int buttonHeight = 40;
  const int buttonWidth = 100;

  BUTTON okButton       = {"OK",     ui.displaySpaceCenterX, ButtonsY, buttonWidth , buttonHeight};
  ui.drawButton(okButton);

  //
  // process touch events
  //
  while(true)
  {
    ui.getTouchEvents();
    if (ui.checkForButtonClicked(okButton))
      return;
  }
}


// ---------------------------------------------------------------------------------
//                                 Unused commands
// ---------------------------------------------------------------------------------

//
// "Test the Theta homing sensor" command
//
void testThetaHomeSensorCommand(void)
{ 
  //
  // redraw the screen to let the user know it is plotting now
  //
  ui.drawTitleBarWithBackButton("Test Theta Home Sensor");
  ui.clearDisplaySpace();
  ui.lcdSetCursorXY(ui.displaySpaceCenterX, ui.displaySpaceCenterY-10);
  ui.lcdPrintCentered("LED shows status of Theta sensor.");


  while(true)
  {
    //
    // read the home pin, then sent the LED
    //
    digitalWrite(LED_PIN, digitalRead(MOTOR_THETA_HOME_PIN));

    //
    // check if the "Back" button pressed
    //
    ui.getTouchEvents();

    if (ui.checkForBackButtonClicked())
    {
      digitalWrite(LED_PIN, LOW);
      return;
    }
  }
}



//
// "Test the Raduis homing sensor" command
//
void testRadiusHomeSensorCommand(void)
{ 
  //
  // redraw the screen to let the user know it is plotting now
  //
  ui.drawTitleBarWithBackButton("Test Raduis Home Sensor");
  ui.clearDisplaySpace();
  ui.lcdSetCursorXY(ui.displaySpaceCenterX, ui.displaySpaceCenterY-10);
  ui.lcdPrintCentered("LED shows status of Radius sensor.");


  while(true)
  {
    //
    // read the home pin, then sent the LED
    //
    digitalWrite(LED_PIN, !digitalRead(MOTOR_RADIUS_HOME_PIN));

    //
    // check if the "Back" button pressed
    //
    ui.getTouchEvents();

    if (ui.checkForBackButtonClicked())
    {
      digitalWrite(LED_PIN, LOW);
      return;
    }
  }
}


// -------------------------------------- End --------------------------------------
