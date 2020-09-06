/*
   3-2-3 Sketch

   This sketch is based on the 3-2-3-Simple Sketch Shared by Kevin Holme for his 3-2-3 system.

   I have taken the sketch and modified it to work with a Rolling code Remote trigger, as an addition/alternate
   to an RC remote.
   This allows an independent system to trigger the 3-2-3 transition.  A 4 button rolling code remote is used.
   The trasnmitter receiver I used is a CHJ-8802
   https://www.ebay.com/itm/4-Channel-Rolling-Code-Remote-Receiver-and-Transmitter-CHJ-8802/163019935605?ssPageName=STRK%3AMEBIDX%3AIT&_trksid=p2057872.m2749.l2649


   One button the Remote is used to acrivate the 3-2-3 system remotely.  Without that being pressed, no other
   buttons will trigger a transition.  This works the same as Kevin's original master switch setting on the RC. (Aux1)

   Also added is the ability to receive comands on the USB Serial to manage and trigger the 3-2-3 transitions.
   Note that I use this for testing only, and if you connect the system to say MarcDuino or Stealth, you should
   use the remote commands with Caution.  The same safety command is required before the main transitions will
   be enabled, so there is some protection.  Additionally, the killswitch command will reset after 30 seconds
   so if you've not triggered the transition, we go back to safe mode.

   Sending P0 will enable/disable the transitions.  It's like a momentary switch with a 30 second timer.
   Sending P2 will try to go to a two leg stance.
   Sending P3 will try to go to a three leg stance.

   The original RC triggers are still available.  Different trigger modes are selected with the #defines below

 * ************************************* WARNING ********************************************
   If you're not hooking up an RC Transmitter, you MUST comment out ENABLE_RC_TRIGGER
   If you do not the loop will become a 2 second loop due to the code that reads the pulses
   from the RC.  Each read has a 1 second timeout and there's two reads!
   This will most likely cause an issue on the 3->2 transitions.
 * ******************************************************************************************

   The Default Pins are for a Pro Micro

   The Sabertooth Libraries can be found here:
   https://www.dimensionengineering.com/info/arduino

   We include the i2c stuff so that we can both receive commands on i2c, and also so that we can talk to
   the LED display via i2c and the two gyro/accelerometer units.  This gives us even more positioning data on
   the 2-3-2 transitions, so that we can know more about what is going on.  It may allow us to "auto restore"
   good state if things are not where we expect them to be. (That's advanced ... and TBD)

   Note that there is no need for these additional sensors.  Everything will work with just the 4 limit switches.

   The Sketch Assumes that the Limit Switches are used in NO mode (Normal Open). This means that when the Switch is
   depressed it reads LOW, and will read HIGH when open (or not pressed).

   Things that need to happen:

   When starting a transition, if the expected Limit switches don't release stop! - TBD
   Add a STOP command, so that if the safety is toggled, the sequence stops immediately - DONE
   Convert ShowTime to be a timer, instead of a counter.  Just use the counter directly. - TBD
   Check for over amperage?? - TBD
   Update the LegHappy/TiltHappy to be true when good, false when not.  Currenty it's inverted.

*/

#include "Wire.h"
#include <USBSabertooth.h>

/////
// Setup the Comms
/////

USBSabertoothSerial C;
USBSabertooth       ST(C, 128);              // Use address 128.

Stream* serialPort;

#define I2CAdress 123 // Because 323 or 232 are not valid!


/////
// Control Modes
/////

// It is recommended to only enable one of the commanding modes
// RC or Rolling Code triggers or Serial Commands, but I'm not telling you what to do!
// They do don't really co-exist currently, so don't expect things to work if you enable more than one.
//
// WARNING:  If there's no signal on the Aux1/Aux2 this routine shows the loop to one pass every 2 seconds!!!
//

/*
   PICK ONE OF THESE BELOW!
*/
//#define ENABLE_RC_TRIGGER  // If you ise RC, use this one.
#define ENABLE_ROLLING_CODE_TRIGGER  // If you use the rolling code remote, use this one.
//#define ENABLE_SERIAL_TRIGGER // If you want to trigger transitions via serial use this one.
/*
   IF YOU DIDN"T PICK ONE, NOTHING WILL HAPPEN!
*/

/*

   LCD Display

*/
#define USE_LCD_DISPLAY

#ifdef USE_LCD_DISPLAY
#include "Adafruit_RGBLCDShield.h"
// Initialise the LCD Display
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
#endif

////////////////////////////////
///////////////////////////////
// Command processing stuff //
/////////////////////////////
////////////////////////////

// Command processing stuff
// maximum number of characters in a command (63 chars since we need the null termination)
#define CMD_MAX_LENGTH 64

// memory for command string processing
char cmdString[CMD_MAX_LENGTH];

/////
// Timing Varaibles
/////
const int ReadInterval = 101;
const int DisplayInterval = 5000;
const int StanceInterval = 100;
const int ShowTimeInterval = 100;
unsigned long currentMillis = 0;      // stores the value of millis() in each iteration of loop()
unsigned long PreviousReadMillis = 0;   //
unsigned long PreviousDisplayMillis = 0;
unsigned long PreviousStanceMillis = 0;
unsigned long PreviousShowTimeMillis = 0;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
unsigned long ShowTime = 0;
unsigned long Aux1noPulseCount = 0;  // Count to make sure we have good radio comms.
unsigned long Aux2noPulseCount = 0;  // Count to make sure we have good radio comms.


/////
// Define the pins that we use.  Gives a single place to change them if desired.
/////
#define Aux1Pin A2    //Input pin from the RC reciever
#define Aux2Pin A3    //Input pin from the RC reciever
#define TiltUpPin 6   //Limit switch input pin, Grounded when closed
#define TiltDnPin 7   //Limit switch input pin, Grounded when closed
#define LegUpPin  8   //Limit switch input pin, Grounded when closed
#define LegDnPin  9   //Limit switch input pin, Grounded when closed
#define ROLLING_CODE_BUTTON_A_PIN 4 // Used as a killswitch / activate button
#define ROLLING_CODE_BUTTON_B_PIN 5 // Transition between 2 and 3 legs.
#define ROLLING_CODE_BUTTON_C_PIN 18
#define ROLLING_CODE_BUTTON_D_PIN 19

/////
// Variables to check R2 state for transitions
/////
const int ThrNumReadings = 4;      //these 5 lines are smoothing for the RC inputs
int ThrReadings[ThrNumReadings];    // the readings from the analog input
int ThrReadIndex = 0;              // the index of the current reading
int ThrTotal = 0;                  // the running total
int ThrAverage = 0;                // the average
int Aux1 = 1500;
int Aux2 = 1500;
int TiltUp;
int TiltDn;
int LegUp;
int LegDn;
int Stance;
int StanceTarget;
char stanceName[12] = "Three Legs.";
bool LegHappy;  // False if the leg is unhappy, True if it is happy
bool TiltHappy; // False if the tilt is unhappy, True if it is happy
int rollCodeA;
int rollCodeB;
int rollCodeC;
int rollCodeD;
bool enableRollCodeTransitions = false;
unsigned long rollCodeTransitionTimeout; // Used to auto disable the enable signal after a set time
bool enableCommandTransitions = false; // Used to enable transitions via Serial/i2c commands.
unsigned long commandTransitionTimeout; // Used to auto disable the enable signal after a set time
#define COMMAND_ENABLE_TIMEOUT 30000 // Default timeout of 30 seconds for performing a transition.
bool killDebugSent = false;

/////
// Variables to check commands
/////
bool serialCommandReceived = false;  // Set if we ever receive a serial command

/////
// Let's define some human friendly names for the various stances.
/////
#define TWO_LEG_STANCE 1
#define THREE_LEG_STANCE 2

/////
// Debounce for the Rolling code.
/////
#ifdef ENABLE_ROLLING_CODE_TRIGGER
#define BUTTON_DEBOUNCE_TIME 500
int buttonALastState;
int buttonBLastState;
int buttonCLastState;
int buttonDLastState;
unsigned long buttonATimeout;
unsigned long buttonBTimeout;
unsigned long buttonCTimeout;
unsigned long buttonDTimeout;
#endif

/////
// DEBUG Control
/////
#define DEBUG
#define DEBUG_VERBOSE  // Enable this to see all debug status on the Serial Monitor.

/////
// Setup Debug Print stuff
// This gives me a nice way to enable/disable debug outputs.
/////
#ifdef DEBUG
#define DEBUG_PRINT_LN(msg)  serialPort->println(msg)
#define DEBUG_PRINT(msg)  serialPort->print(msg)
#else
#define DEBUG_PRINT_LN(msg)
#define DEBUG_PRINT(msg)
#endif // DEBUG

/*

   LCD Display Settings

*/
#ifdef USE_LCD_DISPLAY
#define OFF 0x0
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7
#endif

/*
void sendCommand(uint8_t addr, char* command){

  Wire.beginTransmission(addr);
  Wire.write(command);
  Wire.endTransmission();

}
*/
/*
   Setup

   Basic pin setup to read the various sensors
   Enable the Serial communication to the Sabertooth and Tx/Rx on the Arduino
   Enable the i2c so that we can talk to the Gyro's and Screen.

*/
void setup() {

#ifdef ENABLE_RC_TRIGGER
  pinMode(Aux1Pin, INPUT);  // Set the pin type for the Enable/Disable switch
  pinMode(Aux2Pin, INPUT);  // Set the pin type for the transition joystick input.
#endif //ENABLE_RC_TRIGGER

  pinMode(TiltUpPin, INPUT_PULLUP);  // Limit Switch for body tilt (upper)
  pinMode(TiltDnPin, INPUT_PULLUP);  // Limit Switch for body tilt (lower)
  pinMode(LegUpPin,  INPUT_PULLUP);  // Limit Switch for leg lift (upper)
  pinMode(LegDnPin,  INPUT_PULLUP);  // Limit Switch for leg lift (lower)

#ifdef ENABLE_ROLLING_CODE_TRIGGER
  // Rolling Code Remote Pins
  pinMode(ROLLING_CODE_BUTTON_A_PIN, INPUT_PULLUP);  // Rolling code enable/disable pin
  pinMode(ROLLING_CODE_BUTTON_B_PIN, INPUT_PULLUP);  // Pins to trigger transitions.
  pinMode(ROLLING_CODE_BUTTON_C_PIN, INPUT_PULLUP);
  pinMode(ROLLING_CODE_BUTTON_D_PIN, INPUT_PULLUP);
#endif //ENABLE_ROLLING_CODE_TRIGGER

  SabertoothTXPinSerial.begin(9600); // 9600 is the default baud rate for Sabertooth Packet Serial.
  // The Sabertooth library uses the Serial TX pin to send data to the motor controller

  // Setup the USB Serial Port.
  Serial.begin(9600); // This is the USB Port on a Pro Micro.
  serialPort = &Serial;

  // Setup I2C
  Wire.begin(I2CAdress);                   // Start I2C Bus as Master I2C Address
  Wire.onReceive(receiveEvent);            // register event so when we receive something we jump to receiveEvent();

  //sendCommand(23, "4T14\r");

#ifdef USE_LCD_DISPLAY
  lcd.begin(16, 2);
  lcd.setBacklight(BLUE);
  lcd.setCursor(2, 0);
  lcd.print("Holme 2-3-2");
  lcd.setCursor(0, 1);
  lcd.print("by Neil H v1.0");
#endif

}

/*
   Read inputs from the RC Controller

   If ENABLE_RC_TRIGGER is defined, we will look for RC Inputs to enable transitions
   If ENABLE_RC_TRIGGER is not defined, we will never set any valid transition states in this code.

 ***************************************** WARNING ****************************************
   If there is no radio signal, or signal is lost, this could cause a faceplant.
   The Timeout to get a pule is 1 second.  Since we check 2 pins for a pulse, we could
   spend 2 seconds in this routine if there is no signal.
 ***************************************** WARNING ****************************************

*/
void ReadRC() {
#ifdef ENABLE_RC_TRIGGER
  Aux1 = pulseIn(Aux1Pin, HIGH);
  if (Aux1 == 0) {
    // pulseIn returning 0 is a special case.
    // We did not receive any pulse in the current Timeout.
    Aux1noPulseCount++;
  }
  else if (Aux1 < 500) {
    Aux1 = 1500;
    Aux1noPulseCount = 0;
  }
  else
  {
    Aux1noPulseCount = 0;
  }

  Aux2 = pulseIn(Aux2Pin, HIGH);
  if (Aux2 == 0) {
    // pulseIn returning 0 is a special case.
    // We did not receive any pulse in the current Timeout.
    Aux2noPulseCount++;
  }
  else if (Aux2 < 500) {
    Aux2 = 1500;
    Aux2noPulseCount = 0;
  }
  else
  {
    Aux2noPulseCount = 0;
  }

  if (Aux1noPulseCount == 5) {
    // This is an error condition.
    // We've had no signal from the radio in 5 seconds.
    // We only print this message once.
    DEBUG_PRINT_LN("ERROR:  No Signal on Aux1 for 5 seconds");
    return;

  }
  if (Aux2noPulseCount == 5) {
    // This is an error condition.
    // We've had no signal from the radio in 5 seconds.
    // We only print this message once.
    DEBUG_PRINT_LN("ERROR:  No Signal on Aux2 for 5 seconds");
    return;
  }

  //-----------------------------------------------------------RC Radio master switch
  // A toggle switch input from the rc reciever is 1000 when off, and 2000 when on. The following line says that if
  // the toggle switch is on (aux1)  AND the joystick (aux 2) is near the ends of the travel will anything be triggered.
  // just safer than a stick that can be bumped
  // And even then it is not a command as much as a wish.

  if ((Aux1 >= 1800) && (Aux2 <= 1100)) {
    killDebugSent = false;
    StanceTarget = THREE_LEG_STANCE; // Three Leg Stance
  }
  if ((Aux1 >= 1800) && (Aux2 >= 1800)) {
    killDebugSent = false;
    StanceTarget = TWO_LEG_STANCE; // Two Leg Stance
  }
#endif //ENABLE_RC_TRIGGER
}

/*
   ReadLimitSwitches

   This code will read the signal from the four limit switches installed in the body.
   The Limit switches are expected to be installed in NO Mode (Normal Open) so that
   when the switch is depressed, the signal will be pulled LOW.

*/
void ReadLimitSwitches() {
  TiltUp = digitalRead(TiltUpPin);
  TiltDn = digitalRead(TiltDnPin);
  LegUp = digitalRead(LegUpPin);
  LegDn = digitalRead(LegDnPin);
}

void ReadRollingCodeTrigger() {
#ifdef ENABLE_ROLLING_CODE_TRIGGER

  unsigned long now = millis();
  rollCodeA = digitalRead(ROLLING_CODE_BUTTON_A_PIN); // Used for Killswitch.
  rollCodeB = digitalRead(ROLLING_CODE_BUTTON_B_PIN);
  rollCodeC = digitalRead(ROLLING_CODE_BUTTON_C_PIN);
  rollCodeD = digitalRead(ROLLING_CODE_BUTTON_D_PIN);

  // Killswitch pressed.
  // Since the high signal lasts for several hundred milliseconds, we need to prevent checking 
  // for some small period of time.
  if (now >= buttonATimeout){
    if (rollCodeA == HIGH)
    {
      DEBUG_PRINT_LN("Button time > 500ms");
      if (!enableRollCodeTransitions) {
        enableRollCodeTransitions = true;
        killDebugSent = false;
        rollCodeTransitionTimeout = now + COMMAND_ENABLE_TIMEOUT;
        buttonATimeout = now + BUTTON_DEBOUNCE_TIME;
        lcd.setBacklight(VIOLET);
        DEBUG_PRINT_LN("Rolling Code Transmitter Transitions Enabled");
      }
      else {
        enableRollCodeTransitions = false;
        killDebugSent = false;
        lcd.setBacklight(BLUE);
        buttonATimeout = now + BUTTON_DEBOUNCE_TIME;
        DEBUG_PRINT_LN("Rolling Code Transmitter Transitions Disabled");
      }
      return;
    }
  }

  if (now >= buttonBTimeout){
    // Button B pressed.
    if (enableRollCodeTransitions && (rollCodeB == HIGH))
    {
      buttonBTimeout = now + BUTTON_DEBOUNCE_TIME;
      // Start the two to three transition.
      StanceTarget = THREE_LEG_STANCE; // Three Leg Stance
      DEBUG_PRINT_LN("Moving to Three Leg Stance.");
      return;
    }
  }
  
  if (now >= buttonCTimeout){
    // Button pressed.
    if (enableRollCodeTransitions && (rollCodeC == HIGH))
    {
      buttonCTimeout = now + BUTTON_DEBOUNCE_TIME;
      StanceTarget = TWO_LEG_STANCE; // Two Leg Stance
      DEBUG_PRINT_LN("Moving to Two Leg Stance.");
      return;
    }
  }
  
  if (now >= buttonDTimeout){
    // Button pressed.
    if (enableRollCodeTransitions && (rollCodeD == HIGH))
    {
      buttonDTimeout = now + BUTTON_DEBOUNCE_TIME;
      // Does nothing currently... Perhaps a different 2->3 mode?
      DEBUG_PRINT_LN("Button 4 Pressed");
      return;
    }
  }
#endif
}


/*

   Display

   This will output all debug Variables on the serial monitor if DEBUG_VERBOSE mode is enabled.
   The output can be helpful to verify the limit switch wiring and other inputs prior to installing
   the arduino in your droid.  For normal operation DEBUG_VERBOSE mode should be turned off.

*/
void Display() {

  // We only output this if DEBUG_VERBOSE mode is enabled.
#ifdef DEBUG_VERBOSE
  DEBUG_PRINT("Aux1 (RC Kill): ");
  DEBUG_PRINT_LN(Aux1);
  DEBUG_PRINT("Aux2 (RC Trig): ");
  DEBUG_PRINT_LN(Aux2);
  DEBUG_PRINT("Serial Enabled: ");
  DEBUG_PRINT_LN(enableCommandTransitions);
  DEBUG_PRINT("Tilt Up       : ");
  TiltUp ? DEBUG_PRINT_LN("Open") : DEBUG_PRINT_LN("Closed");
  DEBUG_PRINT("Tilt Down     : ");
  TiltDn ? DEBUG_PRINT_LN("Open") : DEBUG_PRINT_LN("Closed");
  DEBUG_PRINT("Leg Up        : ");
  LegUp ? DEBUG_PRINT_LN("Open") : DEBUG_PRINT_LN("Closed");
  DEBUG_PRINT("Leg Down      : ");
  LegDn ? DEBUG_PRINT_LN("Open") : DEBUG_PRINT_LN("Closed");
  DEBUG_PRINT("Stance        : ");
  DEBUG_PRINT(Stance); DEBUG_PRINT(": "); DEBUG_PRINT_LN(stanceName);
  DEBUG_PRINT("Stance Target : ");
  DEBUG_PRINT_LN(StanceTarget);
  DEBUG_PRINT("Leg Happy     : ");
  DEBUG_PRINT_LN(LegHappy);
  DEBUG_PRINT("Tilt Happy    : ");
  DEBUG_PRINT_LN(TiltHappy);
  DEBUG_PRINT("Show Time     : ");
  DEBUG_PRINT_LN(ShowTime);
#endif // DEBUG_VERBOSE

#ifdef USE_LCD_DISPLAY
  // Here is the code that will update the LCD Display with the Live System Status
  //lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Status:"); //7
  if (Stance <= 2) {
    lcd.setCursor(8, 0);
    // Stance is good.
    lcd.setBacklight(BLUE);
    lcd.print("OK"); // 10
    lcd.setCursor(10, 0);
    lcd.print("    "); // to end
  }
  else
  {
    lcd.setCursor(8, 0);
    // Stance is error.
    lcd.setBacklight(RED);
    lcd.print("Error");
  }
  lcd.setCursor(0, 1);
  //lcd.print("                ");
  //lcd.setCursor(0,1);
  lcd.print(Stance); // 1 character
  lcd.setCursor(1, 1);
  lcd.print(": "); // 2
  lcd.setCursor(3, 1);
  lcd.print(stanceName);


#endif
}


/*
   Actual movement commands are here,  when we send the command to move leg down, first it checks the leg down limit switch, if it is closed it
   stops the motor, sets a flag (happy) and then exits the loop, if it is open the down motor is triggered.
   all 4 work the same way
*/

/*
    MoveLegDn

    Moves the Center leg down.
*/
void MoveLegDn() {

  // Read the Pin to see where the leg is.
  LegDn = digitalRead(LegDnPin);

  // If the Limit switch is closed, we should stop the motor.
  if (LegDn == LOW) {
    ST.motor(1, 0);     // Stop.
    LegHappy = false;   // Record that we are in a good state.
    return;
  }

  // If the switch is open, then we need to move the motor until
  // the switch is closed.
  if (LegDn == HIGH) {
    ST.motor(1, 1024);  // Go forward at half power.
  }
}

/*
    MoveLegUp

    Moves the Center leg up.
*/
void MoveLegUp() {

  // Read the Pin to see where the leg is.
  LegUp = digitalRead(LegUpPin);

  // If the Limit switch is closed, we should stop the motor.
  if (LegUp == LOW) {
    ST.motor(1, 0);     // Stop.
    LegHappy = false;   // Record that we are in a good state.
    return;
  }

  // If the switch is open, then we need to move the motor until
  // the switch is closed.
  if (LegUp == HIGH) {
    ST.motor(1, -2047);  // Go backwards at full power.
  }
}

/*
    MoveTiltDn

    Rotates the body toward 18 degrees for a 3 leg stance.
*/
void MoveTiltDn() {

  // Read the Pin to see where the body tilt is.
  TiltDn = digitalRead(TiltDnPin);

  // If the Limit switch is closed, we should stop the motor.
  if (TiltDn == 0) {
    ST.motor(2, 0);     // Stop.
    TiltHappy = false;  // Record that we are in a good state.
    return;
  }

  // If the switch is open, then we need to move the motor until
  // the switch is closed.
  if (TiltDn == 1) {
    ST.motor(2, 2047);  // Go forward at full power.
  }
}

/*
    MoveTiltUp

    Rotates the body toward 0 degrees for a 2 leg stance.
*/
void MoveTiltUp() {
  // Read the Pin to see where the body tilt is.
  TiltUp = digitalRead(TiltUpPin);

  // If the Limit switch is closed, we should stop the motor.
  if (TiltUp == LOW) {
    ST.motor(2, 0);     // Stop.
    TiltHappy = false;  // Record that we are in a good state.
    return;
  }

  // If the switch is open, then we need to move the motor until
  // the switch is closed.
  if (TiltUp == HIGH) {
    ST.motor(2, -2047);  // Go forward at full power.
  }
}

/*
   displayTransition

   This function will update the LCD display (if enabled)
   during the 2-3-2 transition.

*/
void displayTransition() {
#ifdef USE_LCD_DISPLAY
  lcd.setCursor(0, 0);
  lcd.print("Status: Moving  ");
  lcd.setBacklight(GREEN);

  lcd.setCursor(0, 1);

  if (StanceTarget == TWO_LEG_STANCE) {
    lcd.print("Goto Two Legs   ");
  }
  else if (StanceTarget == THREE_LEG_STANCE) {
    lcd.print("Goto Three Legs ");
  }
#endif
}

/*
   TwoToThree

   this command to go from two legs to to three, ended up being a combo of tilt down and leg down
   with a last second check each loop on the limit switches.
   timing worked out great, by the time the tilt down needed a center foot, it was there.
*/
void TwoToThree() {

  // Read the pin positions to check if they are both down
  // before we start moving anything.
  TiltDn = digitalRead(TiltDnPin);
  LegDn = digitalRead(LegDnPin);

  DEBUG_PRINT_LN("  Moving to Three Legs  ");
  displayTransition();

  // If the leg is already down, then we are done.
  if (LegDn == LOW) {
    ST.motor(1, 0);    // Stop
    LegHappy = false;  // Record that we are in a good state.
  }
  else if (LegDn == HIGH) {
    // If the leg is not down, move the leg motor at full power.
    ST.motor(1, 1024);  // Go forward at half power.
  }

  // If the Body is already tilted, we are done.
  if (TiltDn == LOW) {
    ST.motor(2, 0);     // Stop
    TiltHappy = false;  // Record that we are in a good state.
  }
  else if (TiltDn == HIGH) {
    // If the body is not tilted, move the tilt motor at full power.
    ST.motor(2, 2047);  // Go forward at full power.
  }
}


/*
   ThreeToTwo

   going from three legs to two needed a slight adjustment. I start a timer, called show time, and use it to
   delay the center foot from retracting.

   In the future, the gyro can be used to start the trigger of the leg lift once the leg/body angle gets to
   a point where the center of mass is close enough to start the retraction.
*/

void ThreeToTwo() {

  // Read the limit switches to see where we are.
  TiltUp = digitalRead(TiltUpPin);
  LegUp = digitalRead(LegUpPin);
  TiltDn = digitalRead(TiltDnPin);

  DEBUG_PRINT_LN("  Moving to Two Legs  ");
  displayTransition();

  // First if the center leg is up, do nothing.
  if (LegUp == LOW) {
    ST.motor(1, 0);    // Stop
    LegHappy = false;  // Record that we are in a good state.
  }

  // TODO:  Convert the counters to just use a timer.
  // If leg up is open AND the timer is in the first 20 steps then lift the center leg at 25 percent speed
  // The intent here is to move the leg slowly so that it pushes the body up until we have reached the balance
  // point for two leg stance.  After that point we can pull the leg up quickly.
  if (LegUp == HIGH &&  ShowTime >= 1 && ShowTime <= 10) {
    ST.motor(1, -250);
  }

  //  If leg up is open AND the timer is over 21 steps then lift the center leg at full speed
  if (LegUp == HIGH && ShowTime >= 12) {
    ST.motor(1, -2047);
  }

  // at the same time, tilt up till the switch is closed
  if (TiltUp == LOW) {
    ST.motor(2, 0);     // Stop
    TiltHappy = false;  // Record that we are in a good state.
  }
  if (TiltUp == HIGH) {
    ST.motor(2, -2047);  // Go backward at full power.
  }
}


/*
   CheckStance

   This is simply taking all of the possibilities of the switch positions and giving them a number.
   The loop only runs when both happy flags are triggered, meaning that it does not run in the middle of
   a transition.

   At any time, including power up, the droid can run a check and come up with a number as to how he is standing.

   The Display code will output the stance number, as well as a code to tell you in a nice human readable format
   what is going on within the droid, so you don't need to physically check (or if there's a problem you can check
   to see why something is not reading correctly.

   The codes are
   L = Leg
   T = Tilt
   U = Up
   D = Down
   ? = Unknown (Neither up nor down)

   so L?TU means that the Tilt is up (two leg stance) but the Leg position is unknown.
   Leg is always reported first, Tilt second.

*/
void CheckStance() {
  // We only do this if the leg and tilt are NOT happy.
  if (LegHappy == false && TiltHappy == false) {

    // Center leg is up, and the body is straight.  This is a 2 leg Stance.
    if (LegUp == LOW && LegDn == HIGH && TiltUp == LOW && TiltDn == HIGH) {
      //Stance = 1;
      Stance = TWO_LEG_STANCE;
      strcpy(stanceName, "Two Legs. ");
      return;
    }

    // Center leg is down, Body is tilted.  This is a 3 leg stance
    if (LegUp == HIGH && LegDn == LOW && TiltUp == HIGH && TiltDn == LOW) {
      //Stance = 2;
      Stance = THREE_LEG_STANCE;
      strcpy(stanceName, "Three Legs");
      return;
    }

    // Leg is up.  Body switches are open.
    // The body is somewhere between straight and tilted.
    if (LegUp == LOW && LegDn == HIGH && TiltUp == HIGH && TiltDn == HIGH) {
      Stance = 3;
      strcpy(stanceName, "Error - LUT?");
    }

    // Leg switches are both open.  The leg is somewhere between up and down
    // Body is straight for a 2 leg stance.
    if (LegUp == HIGH && LegDn == HIGH && TiltUp == LOW && TiltDn == HIGH) {
      Stance = 4;
      strcpy(stanceName, "Error - L?TU");
    }

    // Leg is down, and the body is straight for a 2 leg stance
    // The droid is balanced on the center foot. (Probably about to fall over)
    if (LegUp == HIGH && LegDn == LOW && TiltUp == LOW && TiltDn == HIGH) {
      Stance = 4;
      strcpy(stanceName, "Error - LDTU");
    }

    // Leg is down.  Body switches are both open.
    // The body is somewhere between straight and 18 degrees
    if (LegUp == HIGH && LegDn == LOW && TiltUp == HIGH && TiltDn == HIGH) {
      Stance = 5;
      strcpy(stanceName, "Error - LDT?");
    }

    // Leg switches are both open.
    // Body is tilted for a 3 leg stance.
    if (LegUp == HIGH && LegDn == HIGH && TiltUp == HIGH && TiltDn == LOW) {
      Stance = 6;
      strcpy(stanceName, "Error - L?TD");
    }

    // All 4 limit switches are open.  No idea where we are.
    if (LegUp == HIGH && LegDn == HIGH && TiltUp == HIGH && TiltDn == HIGH) {
      Stance = 7;
      strcpy(stanceName, "Error - L?T?");
    }
  }
}

/*
   Each time through the loop this function is called.
   This checks the killswitch status, and if the killswitch is
   toggled, such that we disable the 2-3-2 system this code will
   stop all motors where they are.

   NOTE:  This could lead to a faceplant.  The expectation is that if
   the user has hit the killswitch, it's because something went wrong.
   This is here for safety.

*/
void checkKillSwitch()
{
  bool stopMotors = false;

#ifdef ENABLE_SERIAL_TRIGGER
  if (!enableCommandTransitions && serialCommandReceived)
  {
    stopMotors = true;
  }
#endif

#ifdef ENABLE_ROLLING_CODE_TRIGGER
  if (!enableRollCodeTransitions)
  {
    stopMotors = true;
  }
#endif

#ifdef ENABLE_RC_TRIGGER
  // WARNING.  IF NO RADIO IS CONNECTED, THIS WILL DELAY FOR 1 SECOND!
  Aux1 = pulseIn(Aux1Pin, HIGH);
  // A signal less than 1800 means the switch is off
  if (Aux1 < 1800)
  {
    stopMotors = true;
  }
#endif

  if (stopMotors && !killDebugSent)
  {
    DEBUG_PRINT_LN("Killswitch activated.  Stopping Motors!");
    killDebugSent = true;
    EmergencyStop();
  }
}

/*
   EmergencyStop

   If we need to stop everything, this will do it!

*/
void EmergencyStop() {
  ST.motor(1, 0);
  ST.motor(2, 0);
  LegHappy = false;
  TiltHappy = false;

  // Setting StanceTarget to 0 ensures we don't try to restart movement.
  StanceTarget = 0;

  DEBUG_PRINT_LN("Emergency Stop.");
}


/*
   Move

   This function does the checking of the requested stance (StanceTarget) and the current
   stance (Stance) based on reading the limit switches ane figuring out where we are.

   If we are in a good state, then we will trigger the relevant transition.

   This also tries to move to a known good state where it may be possible.  In certain cases
   there are "unknown" states where the requested target may be possible to make a move without
   creating a bigger mess.

   e.g. Request going to two legs, but center leg position is unknown tilt motor is up.
   In this case, we can try to lift the center leg.  We're either already in a heap, or we're
   already on two legs, and the center lift just isn't all the way up.  It's safe, or at best won't
   make anything worse!

   This is just one example, but there's a couple of cases like this where we can try to complete
   the request.

   In the rest of the cases, if we can't move safely, we wont.

*/
void Move() {

  // there is no stance target 0, so turn off your motors and do nothing.
  if (StanceTarget == 0) {
    ST.motor(1, 0);
    ST.motor(2, 0);
    LegHappy = false;
    TiltHappy = false;
    return;
  }
  // if you are told to go where you are, then do nothing
  if (StanceTarget == Stance) {
    ST.motor(1, 0);
    ST.motor(2, 0);
    LegHappy = false;
    TiltHappy = false;
    return;
  }
  // Stance 7 is bad, all 4 switches open, no idea where anything is.  do nothing.
  if (Stance == 7) {
    ST.motor(1, 0);
    ST.motor(2, 0);
    LegHappy = false;
    TiltHappy = false;
    return;
  }
  // if you are in three legs and told to go to 2
  if (StanceTarget == TWO_LEG_STANCE && Stance == THREE_LEG_STANCE) {
    LegHappy = true;
    TiltHappy = true;
    ThreeToTwo();
  }
  // This is the first of the slight unknowns, target is two legs,  look up to stance 3, the center leg is up, but the tilt is unknown.
  //You are either standing on two legs, or already in a pile on the ground. Cant hurt to try tilting up.
  if (StanceTarget == TWO_LEG_STANCE && Stance == 3) {
    TiltHappy = true;
    MoveTiltUp();
  }
  // target two legs, tilt is up, center leg unknown, Can not hurt to try and lift the leg again.
  if (StanceTarget == TWO_LEG_STANCE && Stance == 4) {
    LegHappy = true;
    MoveLegUp();
  }
  //Target is two legs, center foot is down, tilt is unknown, too risky do nothing.
  if (StanceTarget == TWO_LEG_STANCE && Stance == 5) {
    ST.motor(1, 0);
    ST.motor(2, 0);
    LegHappy = false;
    TiltHappy = false;
    return;
  }
  // target is two legs, tilt is down, center leg is unknown,  too risky, do nothing.
  if (StanceTarget == TWO_LEG_STANCE && Stance == 6) {
    ST.motor(1, 0);
    ST.motor(2, 0);
    LegHappy = false;
    TiltHappy = false;
    return;
  }
  // target is three legs, stance is two legs, run two to three.
  if (StanceTarget == THREE_LEG_STANCE && Stance == TWO_LEG_STANCE) {
    LegHappy = true;
    TiltHappy = true;
    TwoToThree();
  }
  //Target is three legs. center leg is up, tilt is unknown, safer to do nothing, Recover from stance 3 with the up command
  if (StanceTarget == THREE_LEG_STANCE && Stance == 3) {
    ST.motor(1, 0);
    ST.motor(2, 0);
    LegHappy = false;
    TiltHappy = false;
    return;
  }
  // target is three legs, but don't know where the center leg is.   Best to not try this,
  // recover from stance 4 with the up command,
  if (StanceTarget == THREE_LEG_STANCE && Stance == 4) {
    ST.motor(1, 0);
    ST.motor(2, 0);
    LegHappy = false;
    TiltHappy = false;
    return;
  }
  // Target is three legs, the center foot is down, tilt is unknownm. either on 3 legs now, or a smoking mess,
  // nothing to loose in trying to tilt down again
  if (StanceTarget == THREE_LEG_STANCE && Stance == 5) {
    TiltHappy = true;
    MoveTiltDn();
  }
  // kinda like above, Target is 3 legs, tilt is down, center leg is unknown, ......got nothing to loose.
  if (StanceTarget == THREE_LEG_STANCE && Stance == 6) {
    LegHappy = true;
    MoveLegDn();
  }
}


/*
   loop

   The main processing loop.

   Each time through we check
   1. The killswitch has not been triggered to stop everything.
   2. Read the RC or Rolling code inputs, and the limit switches if the timer has passed (101ms)
   3. Display Debug info / update the LCD display if the timer has expired (5 seconds default)
   4. Check the current stance, based on the limit switch input read above. (100ms default)
   5. Check if the killswitch timeout (Rolling code and Serial comms) has expired (30 seconds default) and disable.
   6. Move based on the inputs
   7. If we have reached the target stance, reset the Target so we don't try moving again if a switch toggles.
   8. Update the timer used to change the center leg lift speed.  (Soon to be replaced with a true timer, not a counter)

*/
void loop() {
  currentMillis = millis();  // this updates the current time each loop

  // Want to look closely at this.  I think this will reset the ShowTime every time though the loop
  // when the switch is open.  Probably not what was intended!
  if (TiltDn == LOW) { // when the tilt down switch opens, the timer starts
    ShowTime = 0;
  }

  // Regardless of time passed, we check the killswitch on every loop.
  // If the killswitch has been turned off (to kill the 2-3-2 system)
  // We will stop the motors, regardless of what is being done.
  // The assumption is that if you hit the killswitch, it was for a good reason.
  //checkKillSwitch();

  if (currentMillis - PreviousReadMillis >= ReadInterval) {
    PreviousReadMillis = currentMillis;
    ReadRC(); // Only does something if ENABLE_RC_TRIGGER is defined.
    ReadRollingCodeTrigger(); // Only does something if ENABLE_ROLLING_CODE_TRIGGER is defined.
    ReadLimitSwitches();
  }

  if (currentMillis - PreviousDisplayMillis >= DisplayInterval) {
    PreviousDisplayMillis = currentMillis;
    Display();
  }

  if (currentMillis - PreviousStanceMillis >= StanceInterval) {
    PreviousStanceMillis = currentMillis;
    CheckStance();
  }

  // Given the order of the checks here, we only check the timeout if the transitions are enabled
  // This minimises the time taken to do the check most of the time.
  if (enableCommandTransitions && (currentMillis >= commandTransitionTimeout)) {
    // We have exceeded the time to do a transition start.
    // Auto Disable the safety so we don't accidentally trigger the transition.
    enableCommandTransitions = false;
    lcd.setBacklight(BLUE);
    //DEBUG_PRINT_LN("Warning: Transition Enable Timeout reached.  Disabling Command Transitions.");
  }

  if (enableRollCodeTransitions && (currentMillis >= rollCodeTransitionTimeout)) {
    // We have exceeded the time to do a transition start.
    // Auto Disable the safety so we don't accidentally trigger the transition.
    enableCommandTransitions = false;
    lcd.setBacklight(BLUE);
    //DEBUG_PRINT_LN("Warning: Transition Enable Timeout reached.  Disabling Command Transitions.");
  }

  Move();

  // Once we have moved, check to see if we've reached the target.
  // If we have then we reset the Target, so that we don't keep
  // trying to move motors (This was a bug found in testing!)
  if (Stance == StanceTarget)
  {
    // Transition complete!
    StanceTarget = 0;
    DEBUG_PRINT_LN("Transition Complete");
  }

  // the following lines triggers my showtime timer to advance one number every 100ms.
  //I find it easier to work with a smaller number, and it is all trial and error anyway.
  if (currentMillis - PreviousShowTimeMillis >= ShowTimeInterval) {
    PreviousShowTimeMillis = currentMillis;
    ShowTime++;
    //DEBUG_PRINT("Showtime: ");DEBUG_PRINT_LN(ShowTime);
  }
}


// Wow, we get a lot of use out of this code.
// Yup here's all the Jawa Lite command processing again!

// function that executes whenever data is received from an I2C master
// this function is registered as an event, see setup()
void receiveEvent(int eventCode) {

  while (Wire.available()) {

    // New I2C handling
    // Needs to be tested, but uses the same parser as Serial!
    bool command_available;
    char ch = (char)Wire.read();

    DEBUG_PRINT("I2C Character received "); DEBUG_PRINT_LN(ch);

    command_available = buildCommand(ch, cmdString); // build command line

    if (command_available)
    {
      parseCommand(cmdString);  // interpret the command
    }
  }
}


/*
   serialEventRun

   SerialEvent occurs whenever a new data comes in the
   hardware serial RX.  This routine is run between each
   time loop() runs, so using delay inside loop can delay
   response.  Multiple bytes of data may be available.

   That's ok ... I never use delay()!

*/
void serialEventRun(void)
{
  if (serialPort->available()) serialEvent();
}

void serialEvent() {

  DEBUG_PRINT_LN("Serial In");
  bool command_available;

  while (serialPort->available()) {
    char ch = (char)serialPort->read();  // get the new byte

    // New improved command handling
    command_available = buildCommand(ch, cmdString); // build command line
    if (command_available)
    {
      parseCommand(cmdString);  // interpret the command
    }
  }
  sei();
}


////////////////////////////////////////////////////////
// Command language - JawaLite emulation
///////////////////////////////////////////////////////


////////////////////////////////
// command line builder, makes a valid command line from the input
byte buildCommand(char ch, char* output_str)
{
  static uint8_t pos = 0;
  switch (ch)
  {
    case '\r':                          // end character recognized
      output_str[pos] = '\0'; // append the end of string character
      pos = 0;      // reset buffer pointer
      return true;      // return and signal command ready
      break;
    default:        // regular character
      output_str[pos] = ch; // append the  character to the command string
      if (pos <= CMD_MAX_LENGTH - 1)pos++; // too many characters, discard them.
      break;
  }
  return false;
}

///////////////////////////////////
// command parser and switcher,
// breaks command line in pieces,
// rejects invalid ones,
// switches to the right command
void parseCommand(char* inputStr)
{
  byte hasArgument = false;
  byte hasTiming = false;
  int argument;
  int address;
  int timing;
  byte pos = 0;
  byte endArg = 0;
  byte length = strlen(inputStr);
  byte PSIPos = length;
  if (length < 2) goto beep; // not enough characters

  /*
    DEBUG_PRINT(" Here's the input string: ");
    DEBUG_PRINT_LN(inputStr);
  */

  // get the adress, one or two digits
  char addrStr[3];
  if (!isdigit(inputStr[pos])) goto beep; // invalid, first char not a digit
  addrStr[pos] = inputStr[pos];
  pos++;                            // pos=1
  if (isdigit(inputStr[pos]))         // add second digit address if it's there
  {
    addrStr[pos] = inputStr[pos];
    pos++;                            // pos=2
  }
  addrStr[pos] = '\0';                // add null terminator

  address = atoi(addrStr);       // extract the address

  // check for more
  if (!length > pos) goto beep;         // invalid, no command after address


  // special case of P commands, where it's easier to parse the string to get digits
  if (inputStr[pos] == 'P')
  {
    pos++;
    if (!length > pos) goto beep;  // no message argument
    doPcommand(address, inputStr + pos); // pass rest of string as argument
    return;                     // exit
  }

  // other commands, get the numerical argument after the command character

  pos++;                             // need to increment in order to peek ahead of command char
  if (!length > pos) {
    hasArgument = false;  // end of string reached, no arguments
    hasTiming = false;
  }
  else
  {
    for (byte i = pos; i < length; i++)
    {
      if (!isdigit(inputStr[i])) goto beep; // invalid, end of string contains non-numerial arguments
    }
    argument = atoi(inputStr + pos); // that's the numerical argument after the command character
    hasArgument = true;
  }

  // switch on command character
  switch (inputStr[pos - 1])            // 2nd or third char, should be the command char
  {
    default:
      goto beep;                        // unknown command
      break;
  }

  return;                               // normal exit

beep:                                 // error exit
  // Dont know what this does ... idnoring it for now!
  //serialPort->write(0x7);             // beep the terminal, if connected
  return;
}

////////////////////
// Command Executors

// Parameter handling for Logic settings
void doPcommand(int address, char* argument)
{
  uint8_t param = argument[0] - '0';
  char* value_array = argument + 1;
  signed long value = atol(value_array);

  /*
    DEBUG_PRINT_LN();
    DEBUG_PRINT("Command: P ");
    DEBUG_PRINT("Address: ");
    DEBUG_PRINT(address);
    DEBUG_PRINT(" Parameter: ");
    DEBUG_PRINT_LN(param);
    DEBUG_PRINT(" Value: ");
    DEBUG_PRINT_LN(value);
  */

  switch (param)
  {
    serialCommandReceived = true;
    case 0:
      if (!enableCommandTransitions)
      {
        enableCommandTransitions = true;
        killDebugSent = false;
        commandTransitionTimeout = millis() + COMMAND_ENABLE_TIMEOUT;
        lcd.setBacklight(VIOLET);
        DEBUG_PRINT_LN("Command Transitions Enabled");
      }
      else {
        enableCommandTransitions = false;
        lcd.setBacklight(BLUE);
        killDebugSent = false;
        DEBUG_PRINT_LN("Command Transitions Disabled");
      }
      break;
    case 2:
      // We don't need a value here.  Just trigger a transition.
      // We check that we are allowed to transition first.
      if (enableCommandTransitions) {
        DEBUG_PRINT_LN("Moving to Two Leg Stance.");
        StanceTarget = TWO_LEG_STANCE;
      }
      else {
        DEBUG_PRINT_LN("Command Transitions Disabled.  Ignoring the request.");
      }
      break;
    case 3:
      // We don't need a value here.  Just trigger a transition.
      // We check that we are allowed to transition first.
      if (enableCommandTransitions) {
        DEBUG_PRINT_LN("Moving to Three Leg Stance.");
        StanceTarget = THREE_LEG_STANCE;
      }
      else {
        DEBUG_PRINT_LN("Command Transitions Disabled.  Ignoring the request.");
      }
      break;
    default:
      break;
  }
}
