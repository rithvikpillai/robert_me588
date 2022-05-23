// ME 588 - Final Code for Robotic Whack-A-Mole Competition Robot - Lovingly Dubbed 'Robert'
// Code designed by: Rithvik Pillai, Dylan Foster, Yigit Karatas, and Samvit Valluri

// *************************************************************************************** //

// Encoder Library Declarations
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

// SPI & TCS34725 Library Declarations
#include <Wire.h>
#include "Adafruit_TCS34725.h"

// Input/Display LED Pin Variable Declarations
#define Start_LED 38  // White LED to show Game Start 
#define Yellow_Select 39  // LEDs to show the Selected Target Color
#define Blue_Select 40
#define Red_Select 41
#define Yellow_Color 42 // LEDs to show the color the robot is currently over
#define Blue_Color 43
#define Red_Color 44
#define Start_Button 27 // Start Button Pin

// Input Button Declarations for Start Button & Color Select Switch
int startButton = 34;
int startButtonRead = 0;
int redButton = 35;
int blueButton = 36;
int yellowButton = 37;
int redButtonRead = 0;
int blueButtonRead = 0;
int yellowButtonRead = 0;

// Left Motor (A)
#define enA 11 // ENA is the PWM input to Left Motor (A)
#define in1 10 // This actuates the backwards direction of the Left Motor (+)
#define in2 9 // This actuates the forward direction of the Left Motor (-)

// Left Motor Position and Velocity Variable Declarations
float motorL_pos = 0;
float motorL_posPrev = 0;
float motorL_vel = 0;
float motorL_velPrev = 0;
float motorL_VFilt = 0;

// Right Motor (B)
#define enB 6 // ENB is the PWM input to the Right Motor
#define in3 8 // This actuates the forward direction of the Right Motor (+)
#define in4 7 // This actuates the backwards direction of the Right Motor (-)

// Right Motor Position and Velocity Variable Declaration
float motorR_pos = 0;
float motorR_posPrev = 0;
float motorR_vel = 0;
float motorR_velPrev = 0;
float motorR_VFilt = 0;

// Encoder Instance and Variable Declarations
Encoder leftenc(2,4);
Encoder rightenc(3,5);
volatile float motorL_encoder = 0;
volatile float motorR_encoder = 0;
long prevT = 0;

// Current and Target Encoder Ticks for Rotation PID Control
long motorL_enc_curr = 0;
long motorR_enc_curr = 0;
long enct_left = 0;
long enct_right = 0;

// PID Controller and Setpoint Input Variables from Serial Window
float integerValue = 0; // stores the incoming serial value. Max value is 65535
char incomingByte; // parses and stores each individual character one by one
int motorL_pwm = 0; // after PID computation data is stored in this variable.
int motorR_pwm = 0;

// PID Gains
float Kp_L = 0.5;
float Ki_L = 4.5;
float Kd_L = 0;

float Kp_R = 0.5;
float Ki_R = 4.5;
float Kd_R = 0;

// Error Variables for PID
float e_L = 0;
float e_int_L = 0;
float e_der_L = 0;
float e_prev_L = 0;

float e_R = 0;
float e_int_R = 0;
float e_der_R = 0;
float e_prev_R = 0;

// Target Speed for PI Control
float vt = 0;

// Finite State Machine Variables
int state = 0;
long startTime;
int turnIndex = 0;

// This array tells the robot how many lines to move through per leg of travel
int lineCount[] = {3, 1, 2, 1, 2, 1, 3, 3};
// This array tells the robot which turn to make at each end of the leg of travel
int turnDir[] = {0, 0, 1, 1, 0, 0, 0, 0}; // Left is 0, Right is 1

// Line Sensor Variables
const int line1pin = 22;
const int line2pin = 23;
const int line3pin = 24;
const int line4pin = 25;
const int line5pin = 26;
int linecount = 0;

// Stepper Motor Characteristics and Pins
#define NUMBER_OF_STEPS_PER_REV 512
#define A 30
#define B 31
#define C 32
#define D 33

// Color Sensor Characteristics and Variable Declarations
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
int desiredColor = 0;
int dispo = 0;
int colorRead = 0;

// Color Sensor Values
#define RED 100
#define YELLOW 110
#define BLUE 101

#define SERIAL

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // Declare motor control pins to be outputs
  pinMode(enA, OUTPUT); // PWM Left
  pinMode(enB, OUTPUT); // PWM Right
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(line3pin, INPUT);
  // Declare stepper motor pins to be outputs
  pinMode(A,OUTPUT);
  pinMode(B,OUTPUT);
  pinMode(C,OUTPUT);
  pinMode(D,OUTPUT);
  // Setting input/output modes for Start Button, Color Select, and LED's
  pinMode(startButton, INPUT);
  pinMode(redButton, INPUT);
  pinMode(blueButton, INPUT);
  pinMode(yellowButton, INPUT);

  pinMode(Start_LED,OUTPUT);
  pinMode(Red_Select,OUTPUT);
  pinMode(Yellow_Select,OUTPUT);
  pinMode(Blue_Select,OUTPUT);

  pinMode(Red_Color,OUTPUT);
  pinMode(Blue_Color,OUTPUT);
  pinMode(Yellow_Color,OUTPUT);
  
//  // Check to See if color sensor is connected properly (for troubleshooting)
//  if (tcs.begin()) {
//    Serial.println("Found sensor");
//  } else {
//    Serial.println("No TCS34725 found ... check your connections");
//    while (1);
//  }
  
  delay(200);
}

void loop() {
  // put your main code here, to run repeatedly:
  
//  // Read Setpoint Input (for troubleshooting)
//  while (Serial.available() > 0) {
//    integerValue = Serial.parseInt(); // stores the integerValue
//    incomingByte = Serial.read(); // stores the /n character
//    if (incomingByte == '\n') // if we receive a newline character we will continue in the loop
//      continue;
//  }

  // ********************** MEASUREMENT *********************** //
  
  // Read Feedback from Encoders
  motorL_encoder = -1*leftenc.read();
  motorR_encoder = 1*rightenc.read();

  // Convert Position to Revolutions
  motorL_pos = motorL_encoder / (230);
  motorR_pos = motorR_encoder / (230);

  // Convert to RPM
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float motorL_vel = 60*(motorL_pos - motorL_posPrev)/deltaT;
  float motorR_vel = 60*(motorR_pos - motorR_posPrev)/deltaT;
  motorL_posPrev = motorL_pos;
  motorR_posPrev = motorR_pos;
  motorL_velPrev = motorL_vel;
  motorR_velPrev = motorR_vel;
  prevT = currT;

  // Discrete Low-Pass Filter (25 Hz Cutoff)
  motorL_VFilt = 0.854*motorL_VFilt + 0.0728*motorL_vel + 0.0728*motorL_velPrev;
  motorR_VFilt = 0.854*motorR_VFilt + 0.0728*motorR_vel + 0.0728*motorR_velPrev;

  // ********************* PID Control **************************** //
  // PID Controller for Left Motor
  e_int_L = e_int_L + (e_L * deltaT);
  e_der_L = (e_L - e_prev_L)/deltaT;
  e_prev_L = e_L;
  motorL_pwm = (Kp_L * e_L) + (Ki_L * e_int_L)+ (Kd_L * e_der_L);
  motorL_drive(motorL_pwm);

  // PID Controller for Right Motor
  e_int_R = e_int_R  + (e_R * deltaT);
  e_der_R = (e_R - e_prev_R)/deltaT;
  e_prev_R = e_R;
  motorR_pwm = (Kp_R * e_R) + (Ki_R * e_int_R) + (Kd_R * e_der_R);
  motorR_drive(motorR_pwm); 

//  // Serial Plotter (for troubleshooting)
//  #ifdef SERIAL
//  Serial.print(vt);
//  Serial.print("\t");
//  Serial.print(motorL_VFilt);
//  Serial.print("\t");
//  Serial.print(motorR_VFilt);
//  Serial.print("\t");
//  Serial.println(state*100);
//  #endif
  
  // ***************** Finite State Machine ********************* //
  switch(state) {
    case 0: // Initialization State (Robot is in White Square and Will not Move Until Button is Pressed)
      startButtonRead = digitalRead(Start_Button); // Start Button is read
      resetError(); // PID Errors are Reset
      startTime = millis(); // Elapsed time is Started

      // Checks the selected Color and Outputs it to the Correct LED
      if(desiredColor == 0)
      {
        redButtonRead = digitalRead(redButton);
        blueButtonRead = digitalRead(blueButton);
        yellowButtonRead = digitalRead(yellowButton);
        Serial.println(redButtonRead);
        Serial.println(blueButtonRead);
        Serial.println(yellowButtonRead);
        
        if(redButtonRead == 1)
        {
          desiredColor = RED;
          digitalWrite(Red_Select,1);
        }
        else if(blueButtonRead == 1)
        {
          desiredColor = BLUE;
          digitalWrite(Blue_Select,1);
        }
        else if(yellowButtonRead == 1)
        {
          desiredColor = YELLOW;
          digitalWrite(Yellow_Select,1);
        }
      }
      
      // Check if the color is selected, if so, check if the start button is pressed, if so, light up the start LED and go to State 1!
      if(desiredColor != 0)
      {
        if(startButtonRead == 1)
        {
          delay(500);
          state = 1;
          digitalWrite(Start_LED,1);
        }
      }
    break;
    
    case 1: // Run straight at constant velocity until next line
      straightlineControl(); // Gain Scheduling to do Straight Line Travel
      vt = 125; // Runs at 125 rpm
      if(!digitalRead(line3pin)) { // If you detect a line, go to State 2
        state = 2;
        startTime = millis();
      }
    break;
    
    case 2: // Transition State: Stop, increment the line count, reset errors, and move on to State 3
      vt = 0;
      linecount = linecount + 1;
      state = 3;
      startTime = millis();
      resetError();
    break;
  
    case 3: // Stop and Check State:
      // Stop
      vt = 0;
      // Wait 0.5 second to allow robot to stop completely, read color sensor
      if((millis() - startTime) > 500) {
        startTime = millis();
        colorRead = currentColor();
        colorRead = currentColor();
        // If you detect the correct color, dispense one Mole-Whacker
        if ((colorRead == desiredColor))
        {
          motorL_drive(0);
          motorR_drive(0);
          dispense();
          colorRead = 0;
        }
        // After dispensing, if it is time to turn, go to State 4 (1 ft forward, then turn), if not, go back to State 1 to move to the next line
        if(linecount == lineCount[turnIndex])
        {
          state = 4;
          linecount = 0;
          startTime = millis();
          resetError();
        }else{
          state = 1;
          resetError();
        }
      }
//      // Prints off the color read (for troubleshooting)
//      Serial.println(colorRead);
    break;

    case 4: // Travel forward for 1 foot, go to state 5 to turn
      vt = 125; // 125 RPM speed
      // Controls Declaration
      straightlineControl(); // travel in a straight line

      // Travel in a straight line for 1.2 seconds (equivalent to ~1 foot), stop, store current encoder ticks, reset PID errors, and transition to State 5
      if (millis() - startTime >= 1200){
        vt = 0; // Stop
        motorL_enc_curr = motorL_encoder; // Store the current encoder ticks for each motor
        motorR_enc_curr = motorR_encoder;
        state = 5; // Transition to State 5
        startTime = millis(); // Store Current Time
        resetError(); // Reset error
      }
    break;

    case 5:
      // Distance PID Characteristics
      // Controls Declaration
      turnControl(); // Gain Scheduling to switch to precise rotation using encoder PID control 
      if (millis() - startTime >  2000){ // Transition to next state after 2 seconds have passed
        // After rotating, go back to State 1 to resume straight line travel until next line, increment the turn index
        state = 1;
        turnIndex++;
        resetError(); // Reset errors
        // If you have turned more than 8 times (completed all legs of travel)
        // Go to End State
        if(turnIndex >= 8) {
          state = 6;
          vt = 0;
          turnIndex = 0;
          digitalWrite(Start_LED,0); // Stop moving and turn off game start LED
        }
      }
    break;

    case 6: // End State
      vt = 0; // Stop Moving (END State)
      digitalWrite(Yellow_Color,1);
      digitalWrite(Red_Color,1);
      digitalWrite(Blue_Color,1);
      // Turn LED's on.
      delay(1000);
      digitalWrite(Yellow_Color,0);
      digitalWrite(Red_Color,0);
      digitalWrite(Blue_Color,0);
      delay(1000);
      // Turn LED's off.
    break;
  }
  // END OF FINITE STATE MACHINE
  
  delay(1);
}
// END OF MAIN LOOP

// ********** FUNCTIONS ********** //

// H-bridge Motor Driver - Left Motor Drive Function
void motorL_drive(int pwm){
  if (pwm > 255){
    pwm = 255;
  }
  if (pwm < -255){
    pwm = -255;
  }
  int pwm_val = (int) fabs(pwm); 
  if (pwm == 0 || vt == 0){
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }
  if (pwm > 0){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    analogWrite(enA,pwm_val);
  }
  if (pwm < 0){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    analogWrite(enA,pwm_val);
  }
}

// H-bridge Motor Driver - Right Motor Drive Function
void motorR_drive(int pwm){
  if (pwm > 255){
    pwm = 255;
  }
  if (pwm < -255){
    pwm = -255;
  }
  int pwm_val = (int) fabs(pwm);
  if (pwm == 0 || vt == 0){
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);
  }
  if (pwm > 0){
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    analogWrite(enB,pwm_val);
  }
  if (pwm < 0){
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);
    analogWrite(enB,pwm_val);
  }
}

// Write Values to Stepper Motor Pins Function
void write(int a,int b,int c,int d){
  digitalWrite(A,a);
  digitalWrite(B,b);
  digitalWrite(C,c);
  digitalWrite(D,d);
}

// Perform One Step Equivalent to ~72 degrees of rotation
void onestep(){
  write(1,0,0,0);
  delay(2);
  write(1,1,0,0);
  delay(2);
  write(0,1,0,0);
  delay(2);
  write(0,1,1,0);
  delay(2);
  write(0,0,1,0);
  delay(2);
  write(0,0,1,1);
  delay(2);
  write(0,0,0,1);
  delay(2);
  write(1,0,0,1);
  delay(2);
}

// Combine the two functions above to make this function that dispenses exactly one foam cube
void dispense(){
    int i = 0;
    while(i<104)
    {
    onestep();
    i++;
    }
}

// This function reads the current color the robot is on, and sends an integer value back to the main code
int currentColor(){
  uint16_t r, g, b, c, colorTemp, lux;
  tcs.getRawData(&r, &g, &b, &c);
  if(g>5000)
  {
    digitalWrite(Yellow_Color,0);
    digitalWrite(Red_Color,0);
    digitalWrite(Blue_Color,0);
    return 0;
  }
  else if((g > 3000))
  {
    digitalWrite(Yellow_Color,1);
    digitalWrite(Red_Color,0);
    digitalWrite(Blue_Color,0);
    return YELLOW;
  }
  else if(r > 2000)
  {
    digitalWrite(Yellow_Color,0);
    digitalWrite(Red_Color,1);
    digitalWrite(Blue_Color,0);
    return RED;
  }
  else if(b > 800)
  {
    digitalWrite(Yellow_Color,0);
    digitalWrite(Red_Color,0);
    digitalWrite(Blue_Color,1);
    return BLUE;
  }
}

// This function resets the PID error accumulation
void resetError(){
  e_L = 0;
  e_int_L = 0;
  e_der_L = 0;
  e_prev_L = 0;
  
  e_R = 0;
  e_int_R = 0;
  e_der_R = 0;
  e_prev_R = 0;
}

// This is a gain scheduling function that instructs the controller to do velocity PID control in a straight line
void straightlineControl(){
  e_L = vt - motorL_VFilt;
  e_R = vt - motorR_VFilt;
  vt = 150;
  Kp_L = 0.5;
  Ki_L = 4.5;
  Kd_L = 0.02;

  Kp_R = 0.5;
  Ki_R = 4.5;
  Kd_R = 0.02;
}

// This is a gain scheduling function that instructs the controller to do encoder PI control for precise +/- 90 degree rotation
void turnControl(){
  Kp_L = 1.5;
  Ki_L = 0.2;
  Kd_L = 0;
  Kp_R = 1.5;
  Ki_R = 0.2;
  Kd_R = 0;
  if (turnDir[turnIndex] == 0){
    enct_left = motorL_enc_curr - 190; // Target Encoder Value corresponding to +90 deg
    enct_right = motorR_enc_curr + 195;
  }else if(turnDir[turnIndex] == 1){
    enct_left = motorL_enc_curr + 190; // Target Encoder Value corresponding to -90 deg
    enct_right = motorR_enc_curr - 190;        
  }

  e_L = enct_left - motorL_encoder;
  e_R = enct_right - motorR_encoder;
}

// **************** END ********************** //
