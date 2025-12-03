/* ************************************************************************************************* */
// UCSD ECE 5 Lab 4 Code: Line Following Robot with PID 
// V 4.0
// Last Modified 5/14/2025 by MingWei Yeoh, Korey Huynh, and Karcher Morris
/* ************************************************************************************************* */

/*
   This is code for your PID controlled line following robot.

   ******      Code Table of Contents      ******

  - Line_Follower_Code_Basic
   > Declare libraries     - declares global variables so each variable can be accessed from every function
   > Declare Pins          - where the user sets what pin everything is connected to 
   > Settings              - settings that can improve robot functionality and help to debug
   > Setup (Main)          - runs once at beginning when you press button on arduino or when you open serial monitor
   > Loop  (Main)          - loops forever calling on a series of function
   
  - Calibration 
   > Main Calibrate()      - runs calibration function calls and synchronizes calibration state with different led animations
  
  - Helper_Functions
   > setLEDs               - turns on all LEDs in the LED_Pin array on or off
   > Read Potentiometers   - reads each potentiometer
   > Read Photoresistors   - reads each photoresistor
   > Run Motors            - runs motors
   > Calculate Error       - calculate error from photoresistor readings
   > PID Turn              - takes the error and implements PID control
   > Print                 - used for printing information but should disable when not debugging because it slows down program

*/

// Include files needed
#include <L298NX2.h> // Using "L298N" library found through arduino library manager developed by Andrea Lombardo (https://github.com/AndreaLombardo/L298N)

// ************************************************************************************************* //
// ************************************************************************************************* //
// Change Robot Settings here

// Modes for our bot
enum class Mode {
    LOOP,
    SWEEP,
    RACE
};

Mode mode = Mode::RACE;

#define PRINTALLDATA        0  // Turn to 1  to prints ALL the data when changed to 1, Could be useful for debugging =)
                                // !! Turn to 0 when running robot untethered
#define NOMINALSPEED        100 // This is the base speed for both motors, can also be increased by using potentiometers

#define USEPOTENTIOMETERS   1 // Do we want to use the potentiometers, or hardcode our pid vals
                       
struct SPID {
    int s;
    int p;
    int i;
    float d;
};
SPID tuning = {0, 0, 0, 0};

// GO TO presetTunings() to change the tuning


// ************************************************************************************************* //

// ****** DECLARE PINS HERE  ****** 

// Taken from LEFT TO RIGHT of the robot ****** Orient yourself so that you are looking from the rear of the robot (photoresistors are farthest away from you, wheels are closest to you)
//                  Left Motors   Right motors 
L298NX2 DriveMotors(  42, 41, 40,      37, 39, 38);
//                 ENA, IN1, IN2, ENB, IN3, IN4

enum side {LEFT, RIGHT};

int LDR_Pin[] = {1,2,3,4,5,6,7}; // SET PINS CONNECTED TO PHOTORESISTORS // FROM LEFT TO RIGHT OF THE ROBOT, ROBOT IS ORIENTED WHERE PHOTORESISOTRS FARTHEST FROM YOU AND WHEELS ARE CLOSEST TO YOU      

// Potentiometer Pins
const int S_pin = 13; // Pin connected to Speed potentiometer
const int P_pin = 12; // Pin connected to P term potentiometer
const int I_pin = 11; // Pin connected to I term potentiometer
const int D_pin = 10; // Pin connected to D term potentiometer
                                                                 
int led_Pins[] = {46, 45};  // LEDs to indicate what part of calibration you're on and to illuminate the photoresistors

// ****** DECLARE Variables HERE  ****** 

const int Sp_range = 100;

// Variables for Calibration and Error Calculation
float Mn[20]; 
float Mx[20];
float LDRf[20];
int LDR[20];    
int rawPResistorData[20];  
int totalPhotoResistors = sizeof(LDR_Pin) / sizeof(LDR_Pin[0]);  
int numLEDs = sizeof(led_Pins) / sizeof(led_Pins[0]); 
int MxRead, MxIndex, CriteriaForMax;
int leftHighestPR, highestPResistor, rightHighestPR;
float AveRead, WeightedAve;   

// For Motor Control
int M1SpeedtoMotor, M2SpeedtoMotor;
int Turn, M1P = 0, M2P = 0;
float error, lasterror = 0, sumerror = 0;

// ************************************************************************************************* //
// setup - runs once
void setup() {
  Serial.begin(9600);                            // For serial communication set up

  for (int i = 0; i < numLEDs; i++)
    pinMode(led_Pins[i], OUTPUT);                // Initialize all LEDs to output
  
  Calibrate();                                   // Calibrate black and white sensing
  presetTuning();
  #if USEPOTENTIOMETERS
    ReadPotentiometers();                          // Read potentiometer values (Sp, P, I, & D)
  #endif

} // end setup()

// ************************************************************************************************* //
// loop - runs/loops forever
void loop() {

  setLeds(1);

  #if USEPOTENTIOMETERS
    ReadPotentiometers(); // Read potentiometers
  #endif

  ReadPhotoResistors(); // Read photoresistors 

  CalcError();          // Calculates error

  getTurn();           // Get turn output based on calculated error and pid

  generatePower();

  RunMotors();          // Runs motors
  
  #if (PRINTALLDATA)     // If PRINTALLDATA Enabled, Print all the data
    Print();      
  #endif   
  
} // end loop()


void generatePower(){
   switch(mode){
    case Mode::LOOP:
      generateLoopPower();
      break;
    case Mode::SWEEP:
      generateSweepPower();
      break;
    case Mode::RACE:
      generateRacePower();
      break;
  }
  lasterror = error;
}

void presetTuning(){
  // Change preset tunings here
  // D should be less than 1 unless u have a VERY good reason
  switch(mode){
    case Mode::LOOP:
      tuning.s = 100;
      tuning.p = 47;
      tuning.i = 0;
      tuning.d = 0.2;
      break;
    case Mode::SWEEP:
      tuning.s = 100;
      tuning.p = 47;
      tuning.i = 0;
      tuning.d = 0.2;
      break;
    case Mode::RACE:
       tuning.s = 100;
      tuning.p = 47;
      tuning.i = 0;
      tuning.d = 0.2;
      break;
  }
}

// ************************************************************************************************* //
// function to calibrate

void Calibrate() {

  int numberOfMeasurements = 20;                // set number Of Measurements to take

  CalibrateHelper(numberOfMeasurements, false); // White Calibration

  setLeds(0);                                   // Turn off LEDs to indicate user to calibrate other color
  delay(4500);
  
  CalibrateHelper(numberOfMeasurements, true);  // Black Calibration

  Serial.print("White Vals:  ");
  for (int i = 0; i < totalPhotoResistors; i++)
    Serial.print(String(Mn[i]) + " ");          // Print the White values that will be used by the robot
  Serial.println();

  Serial.print("Black Vals:  ");
  for (int i = 0; i < totalPhotoResistors; i++)
    Serial.print(String(Mx[i]) + " ");          // Print the Black values that will be used by the robot
  Serial.println();

  Serial.print("Delta Vals:  ");
  for (int i = 0; i < totalPhotoResistors; i++)
    Serial.print(String(Mx[i] - Mn[i]) + " ");  // Print the Difference between the White and Black valuess
  Serial.println();

  setLeds(1);                                   // Turn LEDs on
  delay(2000);

} // end Calibrate()

void CalibrateHelper(int numberOfMeasurements, boolean ifCalibratingBlack) {
  
  if (ifCalibratingBlack)
    Serial.println("\nCalibrating Black");
  else
    Serial.println("\nCalibrating White");
// Indicate that calibration is starting
  for (int i = 0; i < 4; i++) {
      setLeds(1); // turn the LEDs on
      delay(250); // wait
      setLeds(0); // turn the LEDs off
      delay(250); // wait 
  }
  
  setLeds(1);
  delay(250);

  for (int i = 0; i < numberOfMeasurements; i++) {
    for (int pin = 0; pin < totalPhotoResistors; pin++) {
      LDRf[pin] = LDRf[pin] + (float)analogRead(LDR_Pin[pin]);
      delay(2);
    }
    Serial.print(". ");
  }
  for (int pin = 0; pin < totalPhotoResistors; pin++) {
    if (ifCalibratingBlack) {                                   // updating cooresponding array based on if we are calibrating black or white
      Mx[pin] = round(LDRf[pin] / (float)numberOfMeasurements); // take average and store for black
    }
    else {
      Mn[pin] = round(LDRf[pin] / (float)numberOfMeasurements); // take average and store for white
    }
    LDRf[pin] = 0.0;
  }

  Serial.println(" Done!");
  setLeds(0);
  delay(250);
}

// Set all LEDs to a certain brightness
void setLeds(int x) {
  for (int i = 0; i < numLEDs; i++)
    digitalWrite(led_Pins[i], x);
}

// **********Recall your Challenge #1 Code********************************************************************** //
// function to read and map values from potentiometers
void ReadPotentiometers() {
  // Call on user-defined function to read Potentiometer values

  // Potentiometer range from -Sp_range to Sp_range.
  int SpRead = ReadPotentiometerHelper(S_pin, 0, 4095, -Sp_range, Sp_range); // We want to read a potentiometer for S_pin with resolution from 0 to 1023 
  
  int kPRead = ReadPotentiometerHelper(P_pin, 0, 4095, 0, 100); // We want to read a potentiometer for P_pin with resolution from 0 to 1023 and potentiometer range from 0 to 100.
  //kIRead = ReadPotentiometerHelper(I_pin, 0, 4095, 0, 100); // We want to read a potentiometer for I_pin with resolution from 0 to 1023 and potentiometer range from 0 to 100.
  int kDRead = ReadPotentiometerHelper(D_pin, 0, 4095, 0, 100); // We want to read a potentiometer for D_pin with resolution from 0 to 1023 and potentiometer range from 0 to 100.

  tuning.s = SpRead;
  tuning.p = kPRead;
  tuning.d = (float)kDRead * 0.01;
} // end ReadPotentiometers()

int ReadPotentiometerHelper(int pin, int min_resolution, int max_resolution, int min_potentiometer, int max_potentiometer) {
  return map(analogRead(pin), min_resolution, max_resolution, min_potentiometer, max_potentiometer); 
}

// **********Recall your Challenge #2 Code********************************************************************** //
// Function to read photo resistors and map from 0 to 100
void ReadPhotoResistors() {
  for (int i = 0; i < totalPhotoResistors; i++) { 
    rawPResistorData[i] = analogRead(LDR_Pin[i]);
    LDR[i] = map(rawPResistorData[i], Mn[i], Mx[i], 0, 100); // Mn and Mx are created from calibration Min and Max for each pin
  }    

} // end ReadPhotoResistors()


// **********Recall your Challenge #3 Code********************************************************************** //
// function to start motors using nominal speed + speed addition from potentiometer
void RunMotors() {
  M1SpeedtoMotor = min(NOMINALSPEED + tuning.s + M1P, 255); // limits speed to 255
  M2SpeedtoMotor = min(NOMINALSPEED + tuning.s + M2P, 255); // remember M1Sp & M2Sp is defined at beginning of code (default 60)
  
  runMotorAtSpeed(LEFT, -M2SpeedtoMotor); // run right motor 
  runMotorAtSpeed(RIGHT, M1SpeedtoMotor); // run left motor
} // end RunMotors()

// A function that commands a specified motor to move towards a given direction at a given speed
void runMotorAtSpeed(side _side, int speed) {
  if (_side == LEFT) {
    DriveMotors.setSpeedA(abs(speed));
    if (speed > 0)                // swap direction if speed is negative
      DriveMotors.forwardA();           // sets the direction of the motor from arguments
    else
      DriveMotors.backwardA();          // sets the direction of the motor from arguments
  }
  if (_side == RIGHT) {
    DriveMotors.setSpeedB(abs(speed));
    if (speed > 0)                // swap direction if speed is negative
      DriveMotors.backwardB();           // sets the direction of the motor from arguments
    else
      DriveMotors.forwardB();          // sets the direction of the motor from arguments
  }
}

// *************************************************************************************************
// Calculate error based on mode
void CalcError() {
  if (mode == Mode::SWEEP) {
        // ===== Mode SWEEP: use middle 3 sensors only =====
        int leftIdx   = 2;
        int centerIdx = 3;
        int rightIdx  = 4;

        int MxReadLocal = -1;
        int highestPR = centerIdx;

        for (int i = leftIdx; i <= rightIdx; i++) {
            if (LDR[i] > MxReadLocal) {
                MxReadLocal = LDR[i];
                highestPR = i;
            }
        }

        float AveReadLocal = (LDR[leftIdx] + LDR[centerIdx] + LDR[rightIdx]) / 3.0;
        float CriteriaForMax = 1.5;

        if (MxReadLocal > CriteriaForMax * AveReadLocal) {
            float numerator = LDR[leftIdx] * leftIdx + LDR[centerIdx] * centerIdx + LDR[rightIdx] * rightIdx;
            float denominator = LDR[leftIdx] + LDR[centerIdx] + LDR[rightIdx];
            WeightedAve = numerator / denominator;

            float centerPos = centerIdx;
            if (abs(WeightedAve - centerPos) > 1.5) return; // ignore false readings

            // scale it from -1 to 1 to -3 to 3
            error = (WeightedAve - centerPos) * 3;
        }
  } else {
      // ===== Modes LOOP & RACE: original 7-sensor calculation =====
      MxRead = -99;
      AveRead = 0.0;
      for (int i = 0; i < totalPhotoResistors; i++) { // This loop goes through each photoresistor and stores the photoresistor with the highest value to the variable 'highestPResistor'
        if (MxRead < LDR[i]) {
          MxRead = LDR[i];
          MxIndex = -1 * (i - 3);
          highestPResistor = (float)i;
        }
        AveRead = AveRead + (float)LDR[i] / (float)totalPhotoResistors;
      }
        
      CriteriaForMax = 1.5; 
      if (MxRead > CriteriaForMax * AveRead) { // Make sure that the highestPResistor is actually "seeing" a line. What happens if there is no line and we take the photoresistor that happens to have the highest value?

        // Next we assign variables to hold the index of the left and right Photoresistor that has the highest value, though we have to make sure that we aren't checking a Photoresistor that doesn't exist.
        // Ex: To the left of the left most photoresistor or the right of the right most photoresistor
        if (highestPResistor != 0)
          leftHighestPR = highestPResistor - 1;
        else
          leftHighestPR = highestPResistor;

        if (highestPResistor != totalPhotoResistors - 1)
          rightHighestPR = highestPResistor + 1;
        else
          rightHighestPR = highestPResistor;

        // Next we take the percentage of "line" each of our left, middle, and right photoresistors sees and then we take the average, which is our error calculation
        float numerator = (float)(LDR[leftHighestPR] * leftHighestPR) + (float)(LDR[highestPResistor] * highestPResistor) + (float)(LDR[rightHighestPR] * rightHighestPR);
        float denominator = (float)LDR[leftHighestPR] + (float)LDR[highestPResistor] + (float)LDR[rightHighestPR];

        WeightedAve = ((float)numerator) / denominator;

        // Uh, if the average error is like greater than 3, we just keep whatever error we had last
        if(abs(WeightedAve - totalPhotoResistors/2) > 3){
          return;
        }

        error = (WeightedAve - totalPhotoResistors/2);    
      }
  }
}


// ************************************************************************************************* //
// get PD values



double getTurn() {

    Turn = error * tuning.p + (error - lasterror) * tuning.d; // PID!!!!!!!!!!!!!
    return Turn;
}


// ************************************************************************************************* //
// PID Function
void generateRacePower() {

  M1P = -Turn;       // One motor becomes slower and the other faster
  M2P = Turn;
} // end PID_Turn()

// ************************************************************************************************* //
// PID Function
void generateLoopPower() {
  float absError = abs(error);

  // Linear scaling: small errors → factor ~1, large errors → factor ~2.5
  float factor = 1.0 + (absError / 3) * 1.5;

  // might need to flip...
  // we too far to the right, thus we making left turn
  if (error > 0) {
    M1P = -0.5 * Turn;     
    M2P = factor * Turn;
  } else {
    generateRacePower();
  }
  // M1P = -0.25 * Turn;       // One motor becomes slower and the other faster
  // M2P = 1.5 * Turn;


  // scale = constrain(abs(error)/maxError, 0, 1);

  // left_target_reactivity = 0.25;
  // right_target_ractivity = 1.5;

  // // If the error is HUGE, we on a turn, we can go harsh
  // if(error > 2){
  //   M1P = -0.25 * Turn;       // One motor becomes slower and the other faster
  //   M2P = 1.5 * Turn;
  // } else {
  //   generateRacePower();
  // }

} 


// ************************************************************************************************* //
// PID Function
void generateSweepPower() {
  // Lowkey does not need to be that complicated
  generateRacePower();
} 

// ************************************************************************************************* //
// function to print values of interest
void Print() {
  Serial.print(" Sp: " + String(tuning.s) + " P: " + String(tuning.p) + " I: " + String(tuning.i) + " D: " + String(tuning.d) + "  PResistor Val : "); // Prints PID settings

  for (int i = 0; i < totalPhotoResistors; i++) { // Printing the photo resistor reading values one by one
    Serial.print(LDR[i]);
    //Serial.print(rawPResistorData[i]); //Uncomment this if you would prefer to see raw photoresistor readings
    Serial.print(" ");
  }

  Serial.print(" Error: " + String(error));      // this will show the calculated error (-3 through 3)

  Serial.println("  LMotor:  " + String(M1SpeedtoMotor) + "  RMotor:  " + String(M2SpeedtoMotor));    // This prints the arduino output to each motor so you can see what the values are (0-255)
  
  // commented out LEDS because we need it to be straight to test while tethered
  // uhh, i think past me was saying how it should 
  //setLeds(0); 
  // should be 200, but imo 20ms was good update speed for frc
  // so 30 cannot be that far off
  delay(30);                                    // just here to slow down the output for easier reading. Don't comment out or else it'll slow down the processor on the arduino
  //setLeds(1); 
  // delay(100); 

} // end Print()
