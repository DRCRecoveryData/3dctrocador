// November 28, 2024

/* 3DChameleon Mk4.1 Firmware >camaleão turbo 8x

Copyright 2024 William J. Steele


Edited and modified by IGOR Henrique Darin

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
documentation files (the “Software”), to deal in the Software without restriction, including, without limitation, 
the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, 
and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions 
of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED 
TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
DEALINGS IN THE SOFTWARE.

Single Button Press Commands (Selector Pulse Count)

    1: "Extruder T0",
    2: "Extruder T1",
    3: "Extruder T2",
    4: "Extruder T3",
    5: "Extruder T4",
    6: "Extruder T5",
    7: "Extruder T6",
    8: "Extruder T7",
    9: "Load/Home T0",
    10: "Unload/Home",
    11: "Home",
    12: "Next",
    13: "Random",
    14: "Extra Pulse"
 
*/

#include <SSD1306Ascii.h> //i2C OLED
#include <SSD1306AsciiWire.h> //i2C OLED
#include <SparkFunSX1509.h> // sparkfun i/o expansion board - used for additional filament sensors as well as communications with secondary boards
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

/**
 * Made with Marlin Bitmap Converter
 * https://marlinfw.org/tools/u8glib/converter.html
 *
 * This bitmap from 128x64 pasted image
 */
#pragma once


// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET  -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define OLED_I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;

// Define the sparkfun io expansion setup
const byte SX1509_ADDRESS = 0x3E; // SX1509 I2C Address
#define SX1509_FILAMENT_0 0
#define SX1509_FILAMENT_1 1
#define SX1509_FILAMENT_2 2
#define SX1509_FILAMENT_3 3
#define SX1509_FILAMENT_4 4
#define SX1509_FILAMENT_5 5
#define SX1509_FILAMENT_6 6
#define SX1509_FILAMENT_7 7
#define SX1509_OUTPUT 8
SX1509 io;                        // Create an SX1509 object to be used throughout


// The first three (extEnable, extStep, extDir) are for the extruder motor.
#define extEnable 8
#define extStep 2
#define extDir 5
// The other three (ext2Enable, ext2Step, ext2Dir) are for the second extruder motor.
#define ext2Enable 8
#define ext2Step 4
#define ext2Dir 7
// The next three (selEnable, selStep, selDir) are for the filament selector motor.
#define selEnable 8
#define selStep 3
#define selDir 6
// The last three (trigger, s_limit, filament) are sensors to monitor the filament state.
#define trigger A3
#define s_limit A4
#define filament A5
const int counterclockwise = HIGH;
const int clockwise = !counterclockwise;
const int stepsPerRev = 200;
const int microSteps = 16;
const int speedDelay = 100;
const int defaultBackoff = 10;
Servo filamentCutter;  // Creates servo object to control a servo
int cutterPos = 0;    // Variable to store the servo position
bool reverseServo = true;
int currentExtruder = -1;
int nextExtruder = 0;
int lastExtruder = -1;
int tempExtruder = -1;
int seenCommand = 0;
int prevCommand = 0;
int loaderMode = 2;  // (0 = direct drive, 1 = loader/unloader, 2 = loader/unloader with press to cut filament)
long triggerTime = 300;
long pulseTime = (triggerTime / 2);
long distance = 10;
long unloadDistance = stepsPerRev * microSteps * distance;  // This is 10 rotations - about 10"
long loadDistance   = unloadDistance * 1.1;           // This is 11 rotations - about 11"
int address = 0;
byte value;
long idleCount = 0;
bool logoActive = false;
bool T0Loaded = false;
bool T1Loaded = false;
bool T2Loaded = false;
bool T3Loaded = false;
bool T4Loaded = false;
bool T5Loaded = false;
bool T6Loaded = false;
bool T7Loaded = false;
bool displayEnabled = false;
bool ioEnabled = false;
//int sensorEnabled = 0;
long randomNumber = 0;

void setup()
{
  Wire.begin(); // Initializes I2C
  Wire.setClock(400000L); // Sets I2C clock speed

  // Initializes the IO expander
  if (io.begin(SX1509_ADDRESS) == true)
  {
    // Configures the 8 sensor pins as input with internal pull-up resistors
    for (int i = SX1509_FILAMENT_0; i <= SX1509_FILAMENT_7; i++) {
      io.pinMode(i, INPUT_PULLUP);
    }

    // Configures the output pin as OUTPUT
    io.pinMode(SX1509_OUTPUT, OUTPUT);

    ioEnabled = true;
  }
  // Enable OLED display
  oled.begin(&Adafruit128x64, OLED_I2C_ADDRESS);

  // Wait for it to start up
  delay(50);

  // Welcome screen
  oled.setFont(Adafruit5x7);
  oled.clear(); // Clear display
  oled.println("");
  oled.println("       Welcome"); // Print a welcome message  
  oled.println("");
  oled.println("   Camaleao Turbo  "); // Print a welcome message
  oled.println("");
  oled.println("         8x ");
  delay(3000);

  displayText(0, "       Ready!");
  
  seenCommand = 0;

  // The 3 output pin sets:
  pinMode(extEnable, OUTPUT);
  pinMode(extStep, OUTPUT);
  pinMode(extDir, OUTPUT);

  pinMode(extEnable, OUTPUT);
  pinMode(ext2Step, OUTPUT);
  pinMode(ext2Dir, OUTPUT);

  pinMode(selEnable, OUTPUT);
  pinMode(selStep, OUTPUT);
  pinMode(selDir, OUTPUT);

  // Set up the button
  pinMode(trigger, INPUT_PULLUP);  // Selector

  // A little override here... we're using the two inputs as I2C instead
  pinMode(s_limit, OUTPUT);    
  pinMode(filament, OUTPUT); 

  // Lock the selector by energizing it
  digitalWrite(selEnable, HIGH);

  // Make sure filament isn't blocked by guillotine
  connectGillotine();
  cutFilament();
  disconnectGillotine();
  
  prevCommand = 0;

}

int lastLoop = 0;

void loop()
{
  static uint32_t lastTime = 0;

  seenCommand = 0;
  idleCount++;

  // Process button press
  if (digitalRead(trigger) == 0)
  {
    idleCount = 0;
    logoActive = false;
    unsigned long nextPulse;
    unsigned long pulseCount = 0;
    unsigned long commandCount = 0;

    // Keep counting (and pulsing) until button is released
    while (digitalRead(trigger) == 0)
    {
      if(pulseCount<pulseTime)
      {
        pulseCount++;
        displayCommand(pulseCount);
        if(pulseCount>1) vibrateMotor();
      }
      delay(400);  // Each pulse is 400+ milliseconds apart 
    }
    processCommand(pulseCount); // OK... execute whatever command was caught (by pulse count)
    pulseCount = 0;
  }

  // Updates the IO block, duh! No, seriously, fetches the state of the SparkFun GPIO expander
  updateIOBlock();

  // Each loop adds a 50 ms delay, which is added AFTER the command is processed before the next one can begin
  delay(50);
}

// Read the SparkFun SX1509 IO expander data
void updateIOBlock() {
  if(ioEnabled) {
    T0Loaded = io.digitalRead(SX1509_FILAMENT_0);
    T1Loaded = io.digitalRead(SX1509_FILAMENT_1);
    T2Loaded = io.digitalRead(SX1509_FILAMENT_2);
    T3Loaded = io.digitalRead(SX1509_FILAMENT_3);
    T4Loaded = io.digitalRead(SX1509_FILAMENT_4);
    T5Loaded = io.digitalRead(SX1509_FILAMENT_5);
    T6Loaded = io.digitalRead(SX1509_FILAMENT_6);
    T7Loaded = io.digitalRead(SX1509_FILAMENT_7);
  }
}



// Handles UI screen layout feedback corresponding to pulse ticks
void displayCommand(long commandCount)
{
  switch(commandCount)
  {
    case 2:
      displayText(25, "     Extruder T0");
      break;
    case 3:
      displayText(25, "     Extruder T1");
      break;
    case 4:
      displayText(25, "     Extruder T2");
      break;
    case 5:
      displayText(25, "     Extruder T3");
      break;
    case 6:
      displayText(25, "     Extruder T4");
      break;
    case 7:
      displayText(25, "     Extruder T5");
      break;
    case 8:
      displayText(25, "     Extruder T6");
      break;
    case 9:
      displayText(25, "     Extruder T7");
      break;
    case 10:
      displayText(25, "    Load/Home T0");
      break;
    case 11:
      displayText(28, "   Unload/Home");
      break;
    case 12:
      displayText(50, "       Home");
      break;  
    case 13:
      displayText(50, "       Next");
      break;
    case 14:
      displayText(40, "      Random");
      break;
    default:
      displayText(30, "    No Command");
      break;
  }
}


// Execute the specific pulse count command
void processCommand(long commandCount)
{
  switch (commandCount)
  {
  case 2: // Extruder T0
    displayText(30, "    T0 Selected");
    currentExtruder = 0;
    processMoves();
    displayText(35, "      Idle - T0");
    break;

  case 3: // Extruder T1
    displayText(30, "    T1 Selected");
    currentExtruder = 1;
    processMoves();
    displayText(35, "      Idle - T1");
    break;

  case 4: // Extruder T2
    displayText(30, "    T2 Selected");
    currentExtruder = 2;
    processMoves();
    displayText(35, "      Idle - T2");
    break;

  case 5: // Extruder T3
    displayText(30, "    T3 Selected");
    currentExtruder = 3;
    processMoves();
    displayText(35, "      Idle - T3");
    break;

  case 6: // Extrusora T4
    displayText(30, "    T4 Selected");
    currentExtruder = 4;
    processMoves();
    displayText(35, "      Idle - T4");
    break;

  case 7: // Extruder T5
    displayText(30, "    T5 Selected");
    currentExtruder = 5;
    processMoves();
    displayText(35, "      Idle - T5");
    break;

  case 8: // Extruder T6
    displayText(30, "    T6 Selected");
    currentExtruder = 6;
    processMoves();
    displayText(35, "      Idle - T6");
    break;

  case 9: // Extruder T7
    displayText(30, "    T7 Selected");
    currentExtruder = 7;
    processMoves();
    displayText(35, "      Idle - T7");
    break;

  case 10: // Load/Home T0
    displayText(40, "   Homing...");
    homeSelector();
    displayText(15, "   Press.Load T0");
    gotoExtruder(0, 0);
    if(loaderMode > 0) rotateExtruder(clockwise, loadDistance);
    if(loaderMode > 0) gotoExtruder(0, 1);
    currentExtruder = 0;
    lastExtruder = 0;
    displayText(35, "      Idle - T0");
    break;

  case 11: // Unload/Home
    displayText(30, "    Cutting...");
    connectGillotine();
    cutFilament();
    switch(lastExtruder)
    {
      case 0:
        displayText(10, "  Press.Unload T0");
        break;
      case 1:
        displayText(10, "  Press.Unload T1");
        break;
      case 2:
        displayText(10, "  Press.Unload T2");
        break;
      case 3:
        displayText(10, "  Press.Unload T3");
        break;
      case 4:
        displayText(10, "  Press.Unload T4");
        break;
      case 5:
        displayText(10, "  Press.Unload T5");
        break;
      case 6:
        displayText(10, "  Press.Unload T6");
        break;
      case 7:
        displayText(10, "  Press.Unload T7");
        break;
    }
    if(loaderMode > 0) gotoExtruder((lastExtruder == 7 ? 6 : lastExtruder + 1), lastExtruder);
    if(lastExtruder < 4)
    {
      if(loaderMode > 0) rotateExtruder(counterclockwise, unloadDistance);
    }
    else
    {
      if(loaderMode > 0) rotateExtruder(clockwise, unloadDistance);
    }
    disconnectGillotine();
    displayText(50, "        Idle");
    break;

  case 12: // Home
    displayText(40, "   Homing...");
    homeSelector();
    displayText(50, "       Home");
    break;

  case 13: // Next
    displayText(30, "    Cutting...");
    connectGillotine();
    cutFilament();
    displayText(30, " Next Tool");
    currentExtruder++;
    if(currentExtruder == 8) currentExtruder = 0;
    processMoves();
    displayText(50, "       Next");
    break;

  case 14: // Random
    displayText(30, "    Cutting...");
    connectGillotine();
    cutFilament();
    displayText(30, "   Random");

    randomNumber = random(0, 8) + 1; // 1 to 8

    for(long i = 0; i < randomNumber; i++)
    {
      currentExtruder++;
      if(currentExtruder == 8) currentExtruder = 0;
    }
    processMoves();
    displayText(50, "      Random");
    break;

  default:
    displayText(47, "       Clear");
    delay(200);
    displayText(50, "        Idle");
    break;
  }
}


// Main task function to write updating information out to the OLED screen
void displayText(int offset, String str)
{
  oled.clear();
  oled.println("");
  oled.println("   Camaleao Turbo  "); // Prints a welcome message
  
  oled.println("");
  oled.println("");
  oled.println(str);
  oled.println("");
  oled.println("");
  
  if(ioEnabled)
  {
    oled.print("     ");
    oled.print(T0Loaded ? "0 " : "+");
    oled.print(T1Loaded ? "1" : "+");
    oled.print(T2Loaded ? "2" : "+");
    oled.println(T3Loaded ? "3" : "+");

    oled.print("     ");
    oled.print(T4Loaded ? "4  " : "+  ");
    oled.print(T5Loaded ? "5  " : "+  ");
    oled.print(T6Loaded ? "6  " : "+  ");
    oled.println(T7Loaded ? "7" : "+");
  }
  else
  {
    oled.println("-  -  -  -  -  -  -  -");
    oled.println("-  -  -  -  -  -  -  -");
  }
}


// Real mechanical task processing sequences reside here
void processMoves()
{
  // Make sure we have a real extruder selected
  if(lastExtruder > -1)
  {
    // If yes, we need to cut the filament
    displayText(30, "    Cutting...");
    connectGillotine();
    cutFilament();
    // OK... then wait for the 2nd button execution sequence to unload it
    switch(lastExtruder)
    {
      case 0:
        displayText(10, " Press.Unload T0");
        break;
      case 1:
        displayText(10, " Press.Unload T1");
        break;
      case 2:
        displayText(10, " Press.Unload T2");
        break;
      case 3:
        displayText(10, " Press.Unload T3");
        break;
      case 4:
        displayText(10, " Press.Unload T4");
        break;
      case 5:
        displayText(10, " Press.Unload T5");
        break;
      case 6:
        displayText(10, " Press.Unload T6");
        break;
      case 7:
        displayText(10, " Press.Unload T7");
        break;
    } 
    // Roll over to the first if we're on the last index position
    if(loaderMode > 0) gotoExtruder((lastExtruder == 7 ? 6 : (lastExtruder + 1)), lastExtruder);
    // This determines which direction to run the motor, 0-1-4-5: counterclockwise, 2-3-6-7: clockwise
    if(lastExtruder == 0 || lastExtruder == 1 || lastExtruder == 4 || lastExtruder == 5)
    {
      if(loaderMode > 0)
      {
        if(lastExtruder < 4) rotateExtruder(counterclockwise, unloadDistance);  // Motor 1 (T0~T3)
        else rotateExtruder2(counterclockwise, unloadDistance);                 // Motor 2 (T4~T7)
      }
    }
    else
    {
      if(loaderMode > 0)
      {
        if(lastExtruder < 4) rotateExtruder(clockwise, unloadDistance);  // Motor 1
        else rotateExtruder2(clockwise, unloadDistance);                 // Motor 2
      }
    }
  }
  else
  {
    lastExtruder = 0;
  }
  disconnectGillotine();
  gotoExtruder(lastExtruder, currentExtruder);
  // OK... filament unloaded, time to load the new one... inform the user
  switch(currentExtruder)
  {
    case 0:
      displayText(15, "   Press.Load T0");
      break;
    case 1:
      displayText(15, "   Press.Load T1");
      break;
    case 2:
      displayText(15, "   Press.Load T2");
      break;
    case 3:
      displayText(15, "   Press.Load T3");
      break;
    case 4:
      displayText(15, "   Press.Load T4");
      break;
    case 5:
      displayText(15, "   Press.Load T5");
      break;
    case 6:
      displayText(15, "   Press.Load T6");
      break;
    case 7:
      displayText(15, "   Press.Load T7");
      break;
  }
  // Same logic (but inverted) for the motor direction parameters
  if(currentExtruder == 0 || currentExtruder == 1 || currentExtruder == 4 || currentExtruder == 5)
  {
    if(loaderMode > 0)
    {
      if(currentExtruder < 4) rotateExtruder(clockwise, loadDistance);  // Motor 1
      else rotateExtruder2(clockwise, loadDistance);                   // Motor 2
    }
  }
  else
  {
    if(loaderMode > 0)
    {
      if(currentExtruder < 4) rotateExtruder(counterclockwise, loadDistance);  // Motor 1
      else rotateExtruder2(counterclockwise, loadDistance);                   // Motor 2
    }
  }
  // If we are configuring loading paths, cycle alignment loops now
  if(loaderMode > 0) gotoExtruder(currentExtruder, (currentExtruder == 7 ? 6 : (currentExtruder + 1)));
  // Everyone remembers where we parked!
  lastExtruder = currentExtruder;
}

// This function calculates displacement mapping between currentCog and targetCog positions cleanly
void gotoExtruder(int currentCog, int targetCog)
{
  // Extract physical indexed position mapping coordinates within the selector structure (0 to 3)
  int currentPos = currentCog % 4;
  int targetPos = targetCog % 4;

  int newCog = targetPos - currentPos;

  // Set default rotation orientation parameters
  int newDirection = counterclockwise;

  if(newCog < 0)
  {
    newDirection = clockwise;
    newCog = -newCog;  // Invert sign value metrics
  }

  // If we are already resting on the correct tracking paths, bypass selector activations
  if(newCog > 0)
  {    
    // Increment selector tracking step increments over to the new layout
    for(int i = 0; i < newCog; i++)
    {
      rotateSelector(newDirection, (stepsPerRev / 4) * microSteps);
    }
  }
}


// Move the extruder motor in a specific direction for a specific distance (unless interrupted by manual line release actions)
void rotateExtruder(bool direction, long moveDistance)
{
  digitalWrite(extEnable, LOW);  // Lock motor holding torque
  digitalWrite(extDir, direction); // Configure travel orientation vectors
  const int fastSpeed = speedDelay/2; // Double time calculation execution adjustments
  if(loaderMode==1)
  {
    for (long x = 0; x < (moveDistance-1); x++)
    {
      digitalWrite(extStep, HIGH);
      delayMicroseconds(fastSpeed);
      digitalWrite(extStep, LOW);
      delayMicroseconds(fastSpeed);
    }
  }
  if(loaderMode==2)
  {
   while (digitalRead(trigger) != 0)
    {
      delay(50);
    }
    while (digitalRead(trigger) == 0)
    {
      digitalWrite(extStep, HIGH);
      delayMicroseconds(fastSpeed);
      digitalWrite(extStep, LOW);
      delayMicroseconds(fastSpeed);
    }
  }
  digitalWrite(extEnable, HIGH);
}

void rotateExtruder2(bool direction, long moveDistance)
{
  digitalWrite(ext2Enable, LOW);  // Lock secondary motor holding torque
  digitalWrite(ext2Dir, direction); // Define travel orientation vectors for motor 2
  const int fastSpeed = speedDelay/2; // Double time calculation execution adjustments
  if(loaderMode==1)
  {
    for (long x = 0; x < (moveDistance-1); x++)
    {
      digitalWrite(ext2Step, HIGH);
      delayMicroseconds(fastSpeed);
      digitalWrite(ext2Step, LOW);
      delayMicroseconds(fastSpeed);
    }
  }
  if(loaderMode==2)
  {
    while (digitalRead(trigger) != 0)
    {
      delay(50);
    }
    while (digitalRead(trigger) == 0)
    {
      digitalWrite(ext2Step, HIGH);
      delayMicroseconds(fastSpeed);
      digitalWrite(ext2Step, LOW);
      delayMicroseconds(fastSpeed);
    }
  }
  digitalWrite(ext2Enable, HIGH);  // De-energize secondary motor driver stages
}


// Similar to the extruder processing sequences, but traveling exactly 50 pulse increments (from a 200 base) at a time
void rotateSelector(bool direction, int moveDistance)
{
  // While we are optimizing... can we maximize velocity bounds using your magic calculations outlined above?
  
  digitalWrite(selEnable, LOW); // Engage selector holding current
  digitalWrite(selDir, direction); // Configure tracking path orientation parameters

    // Executes exactly 50 pulses to perform a full lifecycle structural rotation
    for (int x = 0; x < (moveDistance-1); x++)
    {
      digitalWrite(selStep, HIGH);
      delayMicroseconds(speedDelay);
      digitalWrite(selStep, LOW);
      delayMicroseconds(speedDelay);
    }
}

// This alternates the servo arm mechanism path layout cleanly between two predefined boundaries
void cutFilament() {
  digitalWrite(selEnable, LOW); // Kill structural stepper current constraints so full line load energy matches servo draws!
  if(reverseServo==false)
  {
    openGillotine();
    closeGillotine();
  }
  else
  {
    closeGillotine();
    openGillotine();
  }
  digitalWrite(selEnable, HIGH);
}

// Pinout assignment attachments
void connectGillotine()
{
  filamentCutter.attach(11);
}

// Disengage servo signal configurations to completely suppress mechanical buzz states during downtime
void disconnectGillotine()
{
  filamentCutter.detach();
}

// Executes rotational bounds translation cycles spanning between 135 and 180 degrees
void openGillotine()
{
    for (int pos = 135; pos <= 180; pos += 1) { // Moves from 135 degrees out to 180 degrees
    // Increments inside 1 degree tracking steps
    filamentCutter.write(pos);              // Directs servo path updates toward specific index mappings
    delayMicroseconds(25000);                       // Halts loop timing profiles allowing mechanical settling
  }
  delay(50);                       
}

// Executes inverted translation loop maps tracking from 180 indexes back into 135 boundary marks
void closeGillotine()
{
  for (int pos = 180; pos >= 135; pos -= 1) { // Inverts from 180 degrees down into 135 degrees
    filamentCutter.write(pos);              
    delayMicroseconds(25000);                       
  }
  delay(50);                       
}

// Drives physical selector alignment directly past terminal boundaries to ensure a unified physical reference point
void homeSelector()
{
  // Drive tracking clockwise into terminal mechanical constraint walls
  rotateSelector(clockwise, stepsPerRev * microSteps);

  // Translate minutely back toward Extruder 1 settings to drop line pressure stresses off the frame limits
  rotateSelector(counterclockwise, defaultBackoff * microSteps);

 currentExtruder = 0;
 lastExtruder = -2;

}

// Generates quick feedback oscillations
void vibrateMotor()
{
  // Oscillation pulse shift trace count = 1
  rotateSelector(clockwise, 2 * 16);
  rotateSelector(!clockwise, 2 * 16);
}
