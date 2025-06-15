//#include <Time.h>

// Plotclock
// cc - Original Project by Johannes Heberlein 2014
// v 1.02
// thingiverse.com/joo   wiki.fablab-nuernberg.de
// units: mm; microseconds; radians
// origin: bottom left of drawing surface
// time library see http://playground.arduino.cc/Code/time 
// RTC  library see http://playground.arduino.cc/Code/time 
//               or http://www.pjrc.com/teensy/td_libs_DS1307RTC.html  
// Change log:
// 1.01  Release by joo at https://github.com/9a/plotclock
// 1.02  Additional features implemented by Dave (https://github.com/Dave1001/):
//       - added ability to calibrate servofaktor seperately for left and right servos
//       - added code to support DS1307, DS1337 and DS3231 real time clock chips
//       - see http://www.pjrc.com/teensy/td_libs_DS1307RTC.html for how to hook up the real time clock 
// 1.03  Fixed the length bug at the servo2 angle calculation, other fixups
//
// 1.04  04/2025 Additional features implemented by Davide Gatti (www.survivalhacking.it) and renamed project to PENNINO    
//       - Ported to ESP32C3 with NTP time sync with daylight management (CEST)
//       - Easy WIFI configurator with WIFI Portal
//       - Driving an electronic blackboard, which is erased on electrical command
//       - Changed parameter for MG996R big servo
//       - Complete 3D project to easy assembly the plot clock
//       - Full comprensive english comments
//
// 1.05  05/2025 Additional features implemented by Davide Gatti (www.survivalhacking.it) and renamed project to PENNINO II    
//       - Added mode button. Short press = print time - long press = reset WIFI  
//       - Added Joystick Mode, to use PENNINO as acquisition sketchboard. You can move with a joystic a pen and via serial monitor you can see related coordinates, to be used to make your own Number style
//       - Rebuild all numbers style, now print with a digital 7 segment display look.
//       - No more Blackboard modification required. Now the erase command is done by pen that press native blackboard button (need only a soft mod, replacing metal spring button with alluminim foil)
//       - NEED TO BE POWERD BY EXTERNAL SOURCE due USB-C can't feed sufficent current to make it working fine.
//       - Changed and ricalibrated all arms lenght

// Include necessary libraries
#include <Time.h>        // Library for handling time-based operations (e.g., NTP synchronization)
#include <ESP32Servo.h>  // Library for controlling servos with ESP32
#include <WiFi.h>        // Wi-Fi connectivity library for ESP32
#include <WiFiManager.h> // Library for managing Wi-Fi connection via a portal
#include <TimeLib.h>     // Library to access system time DST functions 


// #define JOYSTICK     // Uncomment this line to enter the Joystick Mode to manually position pen to detect proper coordinates
// #define CALIBRATION  // Uncomment this line to enter the calibration mode for servo movements
// In calibration mode, the servos will continue to move at a 90-degree angle in opposite directions.
// If they do not make a 90-degree movement, variables SERVOFAKTORLEFT and SERVOFAKTORRIGHT must be changed so that the servos do not move correctly at 90 degrees.

#define WISHY 3  // Custom constant, likely used in your logic

// Constants defining servo control parameters for left and right motors
#define SERVOFAKTORLEFT 1500 // 1105  // Servo factor for the left motor to convert angle to PWM
#define SERVOFAKTORRIGHT 1090 // 890  // Servo factor for the right motor to convert angle to PWM
#define SERVOLEFTNULL 1950 // 1950    // Neutral position for the left servo motor (in microseconds)
#define SERVORIGHTNULL 715 // 815    // Neutral position for the right servo motor (in microseconds)

// Pin definitions for connected servos and other hardware
#define SERVOPINLIFT  5      // Pin for controlling the lifting servo
#define SERVOPINLEFT  6      // Pin for controlling the left servo
#define SERVOPINRIGHT 7      // Pin for controlling the right servo
#define CLEARPIN      8      // Pin used to clear a display (e.g., clearing screen)
#define UP            9      // Pin used UP
#define DOWN          10     // Pin used DOWN
#define LEFT          20     // Pin used LEFT
#define RIGHT         21     // Pin used RIGHT
#define FUNCTION      2      // Pin used FUNCTION button

// Button management definition
static uint32_t functionButtonPressStartTime = 0; // Tempo di inizio della pressione del pulsante FUNCTION
static bool functionButtonWasPressed = false;    // Stato precedente del pulsante FUNCTION
const uint32_t RESET_WIFI_HOLD_TIME = 5000;      // 5 secondi per il reset WiFi

// Constants for positioning of the servos during operation
#define ZOFF 90  // Offset for adjusting servo positions
#define LIFT0 1110 + ZOFF  // Servo position for lift zero (MOVE)
#define LIFT1 900 + ZOFF   // Servo position for lift one (WRITE)

#define LIFTSPEED 2000  // Speed of lifting action in microseconds

// Physical arm segment lengths in mm
#define L1 60.1  // Length of the first segment of the L arm (in mm)
#define L2 84.2  // Length of the second segment of the L arm (in mm)
#define L3 20.6  // Length of the third segment of the R arm (in mm)
#define L4 69.7  // Length of the fourth segment of the R arm (in mm)

// Arm offsets for the starting positions of the servos
#define O1X 30  // X-coordinate of the L servo center (in mm)
#define O1Y -55 // Y-coordinate of the L servo center (in mm)
#define O2X 88.5  // X-coordinate of the R servo center(in mm)
#define O2Y -55 // Y-coordinate of the R servo center (in mm)

struct Coords {
  uint8_t x;
  uint8_t y;
};


Coords pos[28] = {
  {21, 38}, {26, 38}, {25, 31}, {19, 31}, {15, 24}, {21, 24},   // digit1
  {29, 38}, {36, 38}, {35, 31}, {28, 31}, {25, 25}, {33, 26},   // digit2
  {43, 38}, {49, 38}, {50, 31}, {43, 31}, {42, 26}, {50, 25},   // digit3 
  {53, 40}, {59, 41}, {60, 31}, {53, 31}, {54, 25}, {62, 23},   // digit4
  {39, 34}, {38, 29},                                           // dot  
  {89, 44},                                                     // clear
  {10, 26}                                                      // home
};

// Servo initialization
int servoLift = LIFT0;         // Lift up
Servo servo1, servo2, servo3;  // Servo objects for controlling the lift, left, and right servos

// Joystick Intialization
int GX = 14; // set global joystick coordinate at home
int GY = 18; // set global joystick coordinate at home
static bool lastUpState = true;    // Inizializza a true (non premuto)
static bool lastDownState = true;  // perché i pull-up leggono HIGH quando non premuto
static bool lastLeftState = true;
static bool lastRightState = true;
static uint32_t lastJoyButtonCheck = 0;
const uint32_t JOY_DEBOUNCE_TIME = 50; // Tempo di debounce in millisecondi

// Variables for tracking last known coordinates and time
volatile double lastX = 22;   // Last X position of the arm
volatile double lastY = 37; // Last Y position of the arm
int last_min = -1;  // Last minute value for time checking to reduce unnecessary updates
int FirstTime=0;    // FirstTime management for calibration

// Structure to store local time information
struct tm timeinfo;  // Holds current time data, updated every loop iteration


void resetWiFi() {
  WiFiManager wifiManager;
  wifiManager.resetSettings();
}

void setup() 
{
#ifdef CALIBRATION
  // If calibration is enabled, do nothing special here and proceed with servo calibration
#else
  #ifdef JOYSTICK
   // If joystick is enabled, do nothing special here and proceed with joystick calibration      
  #else
    // Initialize Wi-Fi and connect using WiFiManager (automatically creates a portal to input Wi-Fi credentials)
    WiFiManager wifiManager;
    wifiManager.autoConnect("PENNINO");  // Automatically connects to Wi-Fi with the provided name "FLIPCLOCK"
  #endif
#endif

  Serial.begin(115200);  // Initialize serial communication for debugging

  // Attach servos to respective pins for control
  servo1.attach(SERVOPINLIFT);
  servo2.attach(SERVOPINLEFT);
  servo3.attach(SERVOPINRIGHT);

  // Initialize pin for clearing display
  pinMode(CLEARPIN, OUTPUT);  
  pinMode(UP, INPUT_PULLUP);  
  pinMode(DOWN, INPUT_PULLUP);  
  pinMode(LEFT, INPUT_PULLUP);  
  pinMode(RIGHT, INPUT_PULLUP);  
  pinMode(FUNCTION, INPUT_PULLUP);  
  CleanScreen();    // Clear the display at startup
  numberHD(222);    // Display home screen (or default starting display)

#ifdef CALIBRATION
  // If in calibration mode, skip NTP setup and proceed with servo calibration
#else
  #ifdef JOYSTICK
    Serial.println("JOYSTICK MODE");
    lift(0);
  #else
    // Set the correct time zone for Italy (CET/CEST) using configTime function
    int timeZoneOffset = isDST() ? 7200 : 3600;  // CEST: +2 hours (7200 seconds), CET: +1 hour (3600 seconds)

    // Set up Network Time Protocol (NTP) to synchronize system time with an external server
    configTime(timeZoneOffset, timeZoneOffset, "pool.ntp.org");  // Timezone Offset in seconds
    
    // Wait until the time is successfully synced
    while (!getLocalTime(&timeinfo)) {
      Serial.println("Waiting for NTP synchronization...");
      delay(1000);  // Retry every second until time is synchronized
    }
    Serial.println("Initialization complete. Time synchronized via NTP.");
  #endif
#endif

#ifdef CALIBRATION
  // If in calibration mode, calibrate the servos' positions
  calibrateServos();
#endif
}

void loop() 
{ 
#ifdef JOYSTICK
  // If joystick mode

    // --- Joystick Input Handling ---

   // Read the current state of joystick buttons.
   // A LOW reading means the button is pressed.
   bool isUpPressed = !digitalRead(UP);
   bool isDownPressed = !digitalRead(DOWN);
   bool isLeftPressed = !digitalRead(LEFT);
   bool isRightPressed = !digitalRead(RIGHT);

   // Check if any button is pressed
   if (isUpPressed || isDownPressed || isLeftPressed || isRightPressed) {
       // Perform the action based on which button is pressed
       if (isUpPressed) {
           GY++;
       } else if (isDownPressed) {
           GY--;
       } else if (isLeftPressed) {
           GX--;
       } else if (isRightPressed) {
           GX++;
       }

       // --- Wait for all buttons to be released before continuing ---
       // This loop blocks further execution until all joystick buttons are released.
       while (true) {
           // Re-read button states inside the blocking loop
           isUpPressed = !digitalRead(UP);
           isDownPressed = !digitalRead(DOWN);
           isLeftPressed = !digitalRead(LEFT);
           isRightPressed = !digitalRead(RIGHT);

           if (!isUpPressed && !isDownPressed && !isLeftPressed && !isRightPressed) {
               // All buttons are released, exit the blocking loop
               if (!servo1.attached()) servo1.attach(SERVOPINLIFT);
               if (!servo2.attached()) servo2.attach(SERVOPINLEFT);
               if (!servo3.attached()) servo3.attach(SERVOPINRIGHT);
               delay(100);
               drawTo(GX, GY);  // Move arm to a predefined coordinate
               Serial.print("GX: ");
               Serial.print(GX);
               Serial.print(" - GY: ");
               Serial.println(GY);
               delay(100);
//               servo1.detach();  // Detach servos to save power after operation
//               servo2.detach();
//               servo3.detach();               
               break;
           }
           yield(); // Allow other ESP32 tasks (like Wi-Fi, OTA) to run
           delay(10); // Small delay to prevent busy-waiting too aggressively
       }
   }

#else
  // oder mode
  #ifdef CALIBRATION
    // In calibration mode, attach servos if not already attached, and proceed with calibration
    if (!servo1.attached()) servo1.attach(SERVOPINLIFT);
    if (!servo2.attached()) servo2.attach(SERVOPINLEFT);
    if (!servo3.attached()) servo3.attach(SERVOPINRIGHT);
    calibrateServos();
  #else
    // Ensure servos are properly attached before usage
    if (!servo1.attached()) servo1.attach(SERVOPINLIFT);
    if (!servo2.attached()) servo2.attach(SERVOPINLEFT);
    if (!servo3.attached()) servo3.attach(SERVOPINRIGHT);

    // Update the current time by synchronizing with the NTP server
    if (!getLocalTime(&timeinfo)) return;  // If time sync fails, skip the loop iteration

    // Debugging output for current time
    Serial.print("Old time=");
    Serial.println(last_min);
    Serial.print("Current Time=");
    Serial.println(timeinfo.tm_min);

    // Only update the display and servo positions once per minute
    bool isFunctionPressed = !digitalRead(FUNCTION);
    if (isFunctionPressed) {
      functionButtonPressStartTime = millis(); // Avvia il timer
      functionButtonWasPressed= true;
    }  
    
    while (isFunctionPressed) {
        if (millis() - functionButtonPressStartTime >= RESET_WIFI_HOLD_TIME) {
            // Il pulsante è stato tenuto premuto per 5 secondi
            Serial.println("FUNCTION button held for 5 seconds! Initiating WiFi reset...");
            // DRAW a X on screen and return to home
            lift(1);  // Upper the pen to continue drawing
            drawTo(pos[11].x, pos[11].y);
            lift(0);  // Lower the pen to continue drawing
            drawTo(pos[12].x, pos[12].y);
            lift(1);  // Upper the pen to continue drawing
            drawTo(pos[7].x, pos[7].y);
            lift(0);  // Lower the pen to continue drawing
            drawTo(pos[16].x, pos[16].y);
            lift(1);  // Upper the pen to continue drawing
            drawTo(pos[27].x, pos[27].y); // home
            resetWiFi(); // Chiama la funzione per il reset WiFi
            ESP.restart();
        }
        delay(200); // debounce
        isFunctionPressed = !digitalRead(FUNCTION);
    } 
    functionButtonPressStartTime = 0; // Resetta il timer

    if ((last_min != timeinfo.tm_min) || functionButtonWasPressed){
      functionButtonWasPressed=false;
      Serial.print("Function: ");
      Serial.println(isFunctionPressed);
      lift(1);                    // Upper the pen to stop drawing

      int i = 0;
      int h = timeinfo.tm_hour;   // Get current hour
      int m = timeinfo.tm_min;    // Get current minute
      
      float factorH = 1;          // Scaling factor for hours display
      float factorM = 1.1;        // Scaling factor for minutes display
      float Ypos = 35;            // Vertical position for hour/minute display

      // Display hour digits
      while ((i + 1) * 10 <= h) i++;  // Determine the first digit of the hour
      numberHD(111);                  // Clear screen
      numberHD(i);                    // Display the first digit of the hour
      numberHU((h - i * 10));         // Display the second digit of the hour
      
      numberHD(11);  // Display the colon separator

      // Display minute digits
      i = 0;
      while ((i + 1) * 10 <= m) i++;  // Determine the first digit of the minute
      numberMD(i);                      // Display the first digit of the minute
      numberMU((m - i * 10));           // Display the second digit of the minute

      // Reset display to home screen after a brief delay
      numberHD(222);      // Return display to the default state
      last_min = timeinfo.tm_min;  // Store the current minute for future comparisons
      delay(580);       // Small delay to reduce flicker on the display
      servo1.detach();  // Detach servos to save power after operation
      servo2.detach();
      servo3.detach();
    } else {
      delay(100);  // If no update is needed, wait for 100 mS
    }
  #endif
#endif
}

// Servo calibration function: Moves servos to predefined positions to calibrate their ranges
void calibrateServos() {
  if (FirstTime==0) {  // Moves servos to predefined positions for 10 seconds
    lift(1);           // Lift the pen to drawing
    drawTo(14, 26);    // Move arm to a predefined coordinate
    lift(0);           // Lift the pen to drawing
    delay(10000);  
    FirstTime=1;
    lift(1);           // Lift the pen to drawing
  }
  drawTo(14, 26);    // Move arm to a predefined coordinate
  delay(500);        // Short delay between movements
  drawTo(64, 23);    // Move arm to another coordinate
  delay(500);        // Short delay between movements
}

// Function to draw a line from the current position to the given coordinates
void drawTo(double pX, double pY) {
  double dx, dy, c;
  int i;

  dx = pX - lastX;  // Calculate the change in X direction
  dy = pY - lastY;  // Calculate the change in Y direction
  c = floor(7 * sqrt(dx * dx + dy * dy));  // Calculate the number of steps required for the movement

  if (c < 1) c = 1;  // Ensure at least one step is taken if the movement is small

  // Draw the line in steps by interpolating between the start and end coordinates
  for (i = 0; i <= c; i++) {
    set_XY(lastX + (i * dx / c), lastY + (i * dy / c));  // Incrementally move towards the target position
  }

  lastX = pX;  // Update the current position to the new coordinates
  lastY = pY;
  delay(200);
}


// Function to set servo positions based on target X and Y coordinates
void set_XY(double Tx, double Ty) {
  delay(1);  // Small delay to avoid too rapid movements

  double dx, dy, c, a1, a2, Hx, Hy;

  dx = Tx - O1X;  // Calculate the horizontal distance to the target
  dy = Ty - O1Y;  // Calculate the vertical distance to the target
  c = sqrt(dx * dx + dy * dy);  // Calculate the total distance
  a1 = atan2(dy, dx);  // Calculate the angle for the first servo
  a2 = return_angle(L1, L2, c);  // Calculate the angle for the second servo

  // Set the position for the left servo based on the calculated angles
  servo2.writeMicroseconds(floor(((a2 + a1 - M_PI) * SERVOFAKTORLEFT) + SERVOLEFTNULL));

  // Calculate additional movements for the third servo (right servo)
  a2 = return_angle(L2, L1, c);
  Hx = Tx + L3 * cos((a1 - a2 + 2.714) + M_PI);  // Calculate the X position for right servo movement
  Hy = Ty + L3 * sin((a1 - a2 + 2.714) + M_PI);  // Calculate the Y position for right servo movement

  dx = Hx - O2X;  // Calculate the distance for the right servo
  dy = Hy - O2Y;  // Calculate the distance for the right servo
  c = sqrt(dx * dx + dy * dy);  // Calculate the total distance for the right servo
  a1 = atan2(dy, dx);  // Calculate the angle for the right servo
  a2 = return_angle(L1, L4, c);  // Calculate the inverse kinematics for the right servo

  // Set the position for the right servo based on calculated angles
  servo3.writeMicroseconds(floor(((a1 - a2) * SERVOFAKTORRIGHT) + SERVORIGHTNULL));
}

// Function to calculate the angle based on inverse kinematics for servo positions
double return_angle(double a, double b, double c) {
  return acos((a * a + c * c - b * b) / (2 * a * c));  // Calculate the angle for servo motors using the law of cosines
}

// Lift function: Controls the lifting action of the arm based on different positions
void lift(char lift) {
  if (!servo1.attached()) servo1.attach(SERVOPINLIFT);
  switch (lift) {
    case 0:  // Move to lift position 0 (default position)
      if (servoLift >= LIFT0) {
        while (servoLift >= LIFT0) {
          servoLift--;
          servo1.writeMicroseconds(servoLift);  // Decrease servo position
          delayMicroseconds(LIFTSPEED);  // Wait for the movement to complete
        }
      } else {
        while (servoLift <= LIFT0) {
          servoLift++;
          servo1.writeMicroseconds(servoLift);  // Increase servo position
          delayMicroseconds(LIFTSPEED);  // Wait for the movement to complete
        }
      }
      break;

    case 1:  // Move to lift position 1
      if (servoLift >= LIFT1) {
        while (servoLift >= LIFT1) {
          servoLift--;
          servo1.writeMicroseconds(servoLift);  // Move the servo down
          delayMicroseconds(LIFTSPEED);  // Wait for the movement to complete
        }
      } else {
        while (servoLift <= LIFT1) {
          servoLift++;
          servo1.writeMicroseconds(servoLift);  // Move the servo up
          delayMicroseconds(LIFTSPEED);  // Wait for the movement to complete
        }
      }
      break;
  }
  delay(200);
  servo1.detach();  // Detach servos to save power after operation
}

// Function to clear the screen by setting the appropriate pin high
void CleanScreen(void) {
  digitalWrite(CLEARPIN, HIGH);  // Set the CLEARPIN high to clear display
  delay(300);  // Wait for 300ms to ensure clearing is done
  digitalWrite(CLEARPIN, LOW);   // Reset CLEARPIN back to low
}

// Function to draw a counterclockwise arc (used for drawing shapes)
void bogenUZS(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = -0.05;  // Increment value for angle calculation
  float count = 0;     // Variable to track the arc's progress

  // Draw the arc in steps based on angle increment
  do {
    drawTo(sqee * radius * cos(start + count) + bx, 
           radius * sin(start + count) + by);  // Calculate the next point on the arc
    count += inkr;  // Increment the angle
  } 
  while ((start + count) > ende);  // Continue until the end angle is reached
}

// Function to draw a clockwise arc (used for drawing shapes)
void bogenGZS(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = 0.05;  // Increment value for angle calculation
  float count = 0;     // Variable to track the arc's progress

  // Draw the arc in steps based on angle increment
  do {
    drawTo(sqee * radius * cos(start + count) + bx, 
           radius * sin(start + count) + by);  // Calculate the next point on the arc
    count += inkr;  // Increment the angle
  } 
  while ((start + count) <= ende);  // Continue until the end
}


// Function to draw numbers (0-9) or special characters like dot (11), clear screen (111), home (222)
// Each case handles drawing specific shapes or symbols based on the number (num) provided as input
// For tenth of hours
void numberHD(int num) {

    // Switch statement to handle each case based on the number (num) provided
    switch (num) {

    // Case for drawing the number '0'
    case 0:
      drawTo(pos[0].x, pos[0].y);
      lift(0);  // Lower the pen to continue drawing
      drawTo(pos[1].x, pos[1].y);
      drawTo(pos[5].x, pos[5].y);
      drawTo(pos[4].x, pos[4].y);
      drawTo(pos[0].x, pos[0].y);
      lift(1);  // Lift the pen to stop drawing
      break;

    // Case for drawing the number '1'
    case 1:
      drawTo(pos[1].x, pos[1].y);
      lift(0);
      drawTo(pos[5].x, pos[5].y);
      lift(1);
      break;

    // Case for drawing the number '2'
    case 2:
      drawTo(pos[0].x, pos[0].y);
      lift(0);
      drawTo(pos[1].x, pos[1].y);
      drawTo(pos[2].x, pos[2].y);
      drawTo(pos[3].x, pos[3].y);
      drawTo(pos[4].x, pos[4].y);
      drawTo(pos[5].x, pos[5].y);
      lift(1);
      break;

    // Case for drawing the number '3'
    case 3:
      drawTo(pos[0].x, pos[0].y);
      lift(0);
      drawTo(pos[1].x, pos[1].y);
      drawTo(pos[5].x, pos[5].y);
      drawTo(pos[4].x, pos[4].y);
      lift(1);
      drawTo(pos[2].x, pos[2].y);
      lift(0);
      drawTo(pos[3].x, pos[3].y);
      lift(1);
      break;


    // Case for drawing the number '4'
    case 4:
      drawTo(pos[0].x, pos[0].y);
      lift(0);
      drawTo(pos[3].x, pos[3].y);
      drawTo(pos[2].x, pos[2].y);
      lift(1);
      drawTo(pos[1].x, pos[1].y);
      lift(0);
      drawTo(pos[5].x, pos[5].y);
      lift(1);
      break;

    // Case for drawing the number '5'
    case 5:
      drawTo(pos[1].x, pos[1].y);
      lift(0);
      drawTo(pos[0].x, pos[0].y);
      drawTo(pos[3].x, pos[3].y);
      drawTo(pos[2].x, pos[2].y);
      drawTo(pos[5].x, pos[5].y);
      drawTo(pos[4].x, pos[4].y);
      lift(1);
      break;

    // Case for drawing the number '6'
    case 6:
      drawTo(pos[1].x, pos[1].y);
      lift(0);
      drawTo(pos[0].x, pos[0].y);
      drawTo(pos[4].x, pos[4].y);
      drawTo(pos[5].x, pos[5].y);
      drawTo(pos[2].x, pos[2].y);
      drawTo(pos[3].x, pos[3].y);
      lift(1);
      break;

    // Case for drawing the number '7'
    case 7:
      drawTo(pos[0].x, pos[0].y);
      lift(0);
      drawTo(pos[1].x, pos[1].y);
      drawTo(pos[5].x, pos[5].y);
      lift(1);
      break;

    // Case for drawing the number '8'
    case 8:
      drawTo(pos[0].x, pos[0].y);
      lift(0);
      drawTo(pos[4].x, pos[4].y);
      drawTo(pos[5].x, pos[5].y);
      drawTo(pos[1].x, pos[1].y);
      drawTo(pos[0].x, pos[0].y);
      lift(1);
      drawTo(pos[2].x, pos[2].y);
      lift(0);
      drawTo(pos[3].x, pos[3].y);
      lift(1);

      lift(1);
      break;

    // Case for drawing the number '9'
    case 9:
      drawTo(pos[2].x, pos[2].y);
      lift(0);
      drawTo(pos[3].x, pos[3].y);
      drawTo(pos[0].x, pos[0].y);
      drawTo(pos[1].x, pos[1].y);
      drawTo(pos[5].x, pos[5].y);
      drawTo(pos[4].x, pos[4].y);
      lift(1);
      break;

    // Case for clearing the screen
    case 111:
      lift(1);  // Lift the pen before clearing the screen
      drawTo(pos[0].x, pos[0].y-2);
      drawTo(pos[26].x, pos[26].y-2);
      lift(0);
      drawTo(pos[26].x, pos[26].y+1);
      drawTo(pos[26].x, pos[26].y-1);
      drawTo(pos[26].x, pos[26].y+1);
      lift(1);
      CleanScreen();  // Call to a function that clears the screen (removes any drawn content)
      break;

    // Case for returning the pen to the home position (resetting drawing position)
    case 222:
      lift(1);  // Lift the pen to stop drawing
      drawTo(pos[0].x, pos[0].y);  // Move the pen to a pre-defined home position (coordinates are (0, 20.0))
      drawTo(pos[27].x, pos[27].y);  // Move the pen to a pre-defined home position (coordinates are (0, 20.0))
      break;

    // Case for drawing a dot (for '11')
    case 11:
      drawTo(pos[24].x, pos[24].y-1);
      lift(0);
      drawTo(pos[24].x, pos[24].y);
      lift(1);
      drawTo(pos[25].x, pos[25].y-1);
      lift(0);
      drawTo(pos[25].x, pos[25].y);
      lift(1);
      break;
  }
}

// Function to draw numbers (0-9) 
// Each case handles drawing specific shapes or symbols based on the number (num) provided as input
// For unit of hours
void numberHU(int num) {

    // Switch statement to handle each case based on the number (num) provided
    switch (num) {

    // Case for drawing the number '0'
    case 0:
      drawTo(pos[0+6].x, pos[0+6].y);
      lift(0);  // Lower the pen to continue drawing
      drawTo(pos[1+6].x, pos[1+6].y);
      drawTo(pos[5+6].x, pos[5+6].y);
      drawTo(pos[4+6].x, pos[4+6].y);
      drawTo(pos[0+6].x, pos[0+6].y);
      lift(1);  // Lift the pen to stop drawing
      break;

    // Case for drawing the number '1'
    case 1:
      drawTo(pos[1+6].x, pos[1+6].y);
      lift(0);
      drawTo(pos[5+6].x, pos[5+6].y);
      lift(1);
      break;

    // Case for drawing the number '2'
    case 2:
      drawTo(pos[0+6].x, pos[0+6].y);
      lift(0);
      drawTo(pos[1+6].x, pos[1+6].y);
      drawTo(pos[2+6].x, pos[2+6].y);
      drawTo(pos[3+6].x, pos[3+6].y);
      drawTo(pos[4+6].x, pos[4+6].y);
      drawTo(pos[5+6].x, pos[5+6].y);
      lift(1);
      break;

    // Case for drawing the number '3'
    case 3:
      drawTo(pos[0+6].x, pos[0+6].y);
      lift(0);
      drawTo(pos[1+6].x, pos[1+6].y);
      drawTo(pos[5+6].x, pos[5+6].y);
      drawTo(pos[4+6].x, pos[4+6].y);
      lift(1);
      drawTo(pos[2+6].x, pos[2+6].y);
      lift(0);
      drawTo(pos[3+6].x, pos[3+6].y);
      lift(1);
      break;


    // Case for drawing the number '4'
    case 4:
      drawTo(pos[0+6].x, pos[0+6].y);
      lift(0);
      drawTo(pos[3+6].x, pos[3+6].y);
      drawTo(pos[2+6].x, pos[2+6].y);
      lift(1);
      drawTo(pos[1+6].x, pos[1+6].y);
      lift(0);
      drawTo(pos[5+6].x, pos[5+6].y);
      lift(1);
      break;

    // Case for drawing the number '5'
    case 5:
      drawTo(pos[1+6].x, pos[1].y);
      lift(0);
      drawTo(pos[0+6].x, pos[0+6].y);
      drawTo(pos[3+6].x, pos[3+6].y);
      drawTo(pos[2+6].x, pos[2+6].y);
      drawTo(pos[5+6].x, pos[5+6].y);
      drawTo(pos[4+6].x, pos[4+6].y);
      lift(1);
      break;

    // Case for drawing the number '6'
    case 6:
      drawTo(pos[1+6].x, pos[1].y);
      lift(0);
      drawTo(pos[0+6].x, pos[0+6].y);
      drawTo(pos[4+6].x, pos[4+6].y);
      drawTo(pos[5+6].x, pos[5+6].y);
      drawTo(pos[2+6].x, pos[2+6].y);
      drawTo(pos[3+6].x, pos[3+6].y);
      lift(1);
      break;

    // Case for drawing the number '7'
    case 7:
      drawTo(pos[0+6].x, pos[0+6].y);
      lift(0);
      drawTo(pos[1+6].x, pos[1+6].y);
      drawTo(pos[5+6].x, pos[5+6].y);
      lift(1);
      break;

    // Case for drawing the number '8'
    case 8:
      drawTo(pos[0+6].x, pos[0+6].y);
      lift(0);
      drawTo(pos[4+6].x, pos[4+6].y);
      drawTo(pos[5+6].x, pos[5+6].y);
      drawTo(pos[1+6].x, pos[1+6].y);
      drawTo(pos[0+6].x, pos[0+6].y);
      lift(1);
      drawTo(pos[2+6].x, pos[2+6].y);
      lift(0);
      drawTo(pos[3+6].x, pos[3+6].y);
      lift(1);

      break;

    // Case for drawing the number '9'
    case 9:
      drawTo(pos[2+6].x, pos[2].y);
      lift(0);
      drawTo(pos[3+6].x, pos[3+6].y);
      drawTo(pos[0+6].x, pos[0+6].y);
      drawTo(pos[1+6].x, pos[1+6].y);
      drawTo(pos[5+6].x, pos[5+6].y);
      drawTo(pos[4+6].x, pos[4+6].y);
      lift(1);
      break;
  }
}

// Function to draw numbers (0-9) 
// Each case handles drawing specific shapes or symbols based on the number (num) provided as input
// For tenth of minutes
void numberMD(int num) {

    // Switch statement to handle each case based on the number (num) provided
    switch (num) {

    // Case for drawing the number '0'
    case 0:
      drawTo(pos[0+12].x, pos[0+12].y);
      lift(0);  // Lower the pen to continue drawing
      drawTo(pos[1+12].x, pos[1+12].y);
      drawTo(pos[5+12].x, pos[5+12].y);
      drawTo(pos[4+12].x, pos[4+12].y);
      drawTo(pos[0+12].x, pos[0+12].y);
      lift(1);  // Lift the pen to stop drawing
      break;

    // Case for drawing the number '1'
    case 1:
      drawTo(pos[1+12].x, pos[1+12].y);
      lift(0);
      drawTo(pos[5+12].x, pos[5+12].y);
      lift(1);
      break;

    // Case for drawing the number '2'
    case 2:
      drawTo(pos[0+12].x, pos[0+12].y);
      lift(0);
      drawTo(pos[1+12].x, pos[1+12].y);
      drawTo(pos[2+12].x, pos[2+12].y);
      drawTo(pos[3+12].x, pos[3+12].y);
      drawTo(pos[4+12].x, pos[4+12].y);
      drawTo(pos[5+12].x, pos[5+12].y);
      lift(1);
      break;

    // Case for drawing the number '3'
    case 3:
      drawTo(pos[0+12].x, pos[0].y);
      lift(0);
      drawTo(pos[1+12].x, pos[1+12].y);
      drawTo(pos[5+12].x, pos[5+12].y);
      drawTo(pos[4+12].x, pos[4+12].y);
      lift(1);
      drawTo(pos[2+12].x, pos[2+12].y);
      lift(0);
      drawTo(pos[3+12].x, pos[3+12].y);
      lift(1);
      break;


    // Case for drawing the number '4'
    case 4:
      drawTo(pos[0+12].x, pos[0+12].y);
      lift(0);
      drawTo(pos[3+12].x, pos[3+12].y);
      drawTo(pos[2+12].x, pos[2+12].y);
      lift(1);
      drawTo(pos[1+12].x, pos[1+12].y);
      lift(0);
      drawTo(pos[5+12].x, pos[5+12].y);
      lift(1);
      break;

    // Case for drawing the number '5'
    case 5:
      drawTo(pos[1+12].x, pos[1+12].y);
      lift(0);
      drawTo(pos[0+12].x, pos[0+12].y);
      drawTo(pos[3+12].x, pos[3+12].y);
      drawTo(pos[2+12].x, pos[2+12].y);
      drawTo(pos[5+12].x, pos[5+12].y);
      drawTo(pos[4+12].x, pos[4+12].y);
      lift(1);
      break;

    // Case for drawing the number '6'
    case 6:
      drawTo(pos[1+12].x, pos[1+12].y);
      lift(0);
      drawTo(pos[0+12].x, pos[0+12].y);
      drawTo(pos[4+12].x, pos[4+12].y);
      drawTo(pos[5+12].x, pos[5+12].y);
      drawTo(pos[2+12].x, pos[2+12].y);
      drawTo(pos[3+12].x, pos[3+12].y);
      lift(1);
      break;

    // Case for drawing the number '7'
    case 7:
      drawTo(pos[0+12].x, pos[0+12].y);
      lift(0);
      drawTo(pos[1+12].x, pos[1+12].y);
      drawTo(pos[5+12].x, pos[5+12].y);
      lift(1);
      break;

    // Case for drawing the number '8'
    case 8:
      drawTo(pos[0+12].x, pos[0+12].y);
      lift(0);
      drawTo(pos[4+12].x, pos[4+12].y);
      drawTo(pos[5+12].x, pos[5+12].y);
      drawTo(pos[1+12].x, pos[1+12].y);
      drawTo(pos[0+12].x, pos[0+12].y);
      lift(1);
      drawTo(pos[2+12].x, pos[2+12].y);
      lift(0);
      drawTo(pos[3+12].x, pos[3+12].y);
      lift(1);
      break;

    // Case for drawing the number '9'
    case 9:
      drawTo(pos[2+12].x, pos[2+12].y);
      lift(0);
      drawTo(pos[3+12].x, pos[3+12].y);
      drawTo(pos[0+12].x, pos[0+12].y);
      drawTo(pos[1+12].x, pos[1+12].y);
      drawTo(pos[5+12].x, pos[5+12].y);
      drawTo(pos[4+12].x, pos[4+12].y);
      lift(1);
      break;
  }
}


// Function to draw numbers (0-9) 
// Each case handles drawing specific shapes or symbols based on the number (num) provided as input
// For tenth of minutes
void numberMU(int num) {

    // Switch statement to handle each case based on the number (num) provided
    switch (num) {

    // Case for drawing the number '0'
    case 0:
      drawTo(pos[0+18].x, pos[0+18].y);
      lift(0);  // Lower the pen to continue drawing
      drawTo(pos[1+18].x, pos[1+18].y);
      drawTo(pos[5+18].x, pos[5+18].y);
      drawTo(pos[4+18].x, pos[4+18].y);
      drawTo(pos[0+18].x, pos[0+18].y);
      lift(1);  // Lift the pen to stop drawing
      break;

    // Case for drawing the number '1'
    case 1:
      drawTo(pos[1+18].x, pos[1+18].y);
      lift(0);
      drawTo(pos[5+18].x, pos[5+18].y);
      lift(1);
      break;

    // Case for drawing the number '2'
    case 2:
      drawTo(pos[0+18].x, pos[0+18].y);
      lift(0);
      drawTo(pos[1+18].x, pos[1+18].y);
      drawTo(pos[2+18].x, pos[2+18].y);
      drawTo(pos[3+18].x, pos[3+18].y);
      drawTo(pos[4+18].x, pos[4+18].y);
      drawTo(pos[5+18].x, pos[5+18].y);
      lift(1);
      break;

    // Case for drawing the number '3'
    case 3:
      drawTo(pos[0+18].x, pos[0+18].y);
      lift(0);
      drawTo(pos[1+18].x, pos[1+18].y);
      drawTo(pos[5+18].x, pos[5+18].y);
      drawTo(pos[4+18].x, pos[4+18].y);
      lift(1);
      drawTo(pos[2+18].x, pos[2+18].y);
      lift(0);
      drawTo(pos[3+18].x, pos[3+18].y);
      lift(1);
      break;


    // Case for drawing the number '4'
    case 4:
      drawTo(pos[0+18].x, pos[0+18].y);
      lift(0);
      drawTo(pos[3+18].x, pos[3+18].y);
      drawTo(pos[2+18].x, pos[2+18].y);
      lift(1);
      drawTo(pos[1+18].x, pos[1+18].y);
      lift(0);
      drawTo(pos[5+18].x, pos[5+18].y);
      lift(1);
      break;

    // Case for drawing the number '5'
    case 5:
      drawTo(pos[1+18].x, pos[1+18].y);
      lift(0);
      drawTo(pos[0+18].x, pos[0+18].y);
      drawTo(pos[3+18].x, pos[3+18].y);
      drawTo(pos[2+18].x, pos[2+18].y);
      drawTo(pos[5+18].x, pos[5+18].y);
      drawTo(pos[4+18].x, pos[4+18].y);
      lift(1);
      break;

    // Case for drawing the number '6'
    case 6:
      drawTo(pos[1+18].x, pos[1+18].y);
      lift(0);
      drawTo(pos[0+18].x, pos[0+18].y);
      drawTo(pos[4+18].x, pos[4+18].y);
      drawTo(pos[5+18].x, pos[5+18].y);
      drawTo(pos[2+18].x, pos[2+18].y);
      drawTo(pos[3+18].x, pos[3+18].y);
      lift(1);
      break;

    // Case for drawing the number '7'
    case 7:
      drawTo(pos[0+18].x, pos[0+18].y);
      lift(0);
      drawTo(pos[1+18].x, pos[1+18].y);
      drawTo(pos[5+18].x, pos[5+18].y);
      lift(1);
      break;

    // Case for drawing the number '8'
    case 8:
      drawTo(pos[0+18].x, pos[0+18].y);
      lift(0);
      drawTo(pos[4+18].x, pos[4+18].y);
      drawTo(pos[5+18].x, pos[5+18].y);
      drawTo(pos[1+18].x, pos[1+18].y);
      drawTo(pos[0+18].x, pos[0+18].y);
      lift(1);
      drawTo(pos[2+18].x, pos[2+18].y);
      lift(0);
      drawTo(pos[3+18].x, pos[3+18].y);
      lift(1);
      break;

    // Case for drawing the number '9'
    case 9:
      drawTo(pos[2+18].x, pos[2+18].y);
      lift(0);
      drawTo(pos[3+18].x, pos[3+18].y);
      drawTo(pos[0+18].x, pos[0+18].y);
      drawTo(pos[1+18].x, pos[1+18].y);
      drawTo(pos[5+18].x, pos[5+18].y);
      drawTo(pos[4+18].x, pos[4+18].y);
      lift(1);
      break;
  }
}


// Function to check if we are in Daylight Saving Time (DST) - CEST
bool isDST() {
  // Get current month and day from the TimeLib library
  int currentMonth = month();
  int currentDay = day();
  int currentYear = year();
  
  // DST rule for Italy:
  // - Daylight Saving Time (CEST) starts on the last Sunday of March and ends on the last Sunday of October

  // Create a tmElements_t structure to use with makeTime
  tmElements_t tmMarch = {0};
  tmMarch.Year = currentYear - 1970;  // tmElements_t stores the year as the number of years since 1970
  tmMarch.Month = 3;
  tmMarch.Day = 31;  // March 31st
  
  // Calculate the last Sunday of March
  int lastSundayOfMarch = 31 - weekday(makeTime(tmMarch)) + 1;
  
  // If the current day is on or after the last Sunday in March, DST is active
  if (currentMonth == 3 && currentDay >= lastSundayOfMarch) {
    return true;
  }

  // Create a tmElements_t structure to use with makeTime
  tmElements_t tmOctober = {0};
  tmOctober.Year = currentYear - 1970;
  tmOctober.Month = 10;
  tmOctober.Day = 31;  // October 31st
  
  // Calculate the last Sunday of October
  int lastSundayOfOctober = 31 - weekday(makeTime(tmOctober)) + 1;
  
  // If the current day is before the last Sunday in October, DST is still active
  if (currentMonth == 10 && currentDay < lastSundayOfOctober) {
    return true;
  }

  return false;
}