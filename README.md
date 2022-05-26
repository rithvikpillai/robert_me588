# Search-and-Deploy-Bot
A 2W Differential Drive Robot that searches a playfield for a certain color and deploys foam cubes on the right ones! For further details, please refer to the final report in the repository. This design and implementation was completed with team members Yigit Karatas and Dylan Foster under the supervision of Dr. Laura Blumenschein at Purdue University as a part of ME 588 - Mechatronics. We also participated in a competition with 12 other teams and won <b> 3rd place! </b> Here is a link to a YouTube video showing the robot's operation: https://www.youtube.com/watch?v=hvHdn9Ghw7U

The playfield is a 8x8 ft square field divided into a white starting square and 15 color squares randomly assorted with the colors: red, blue and yellow. The objective of the project is purely academic: It is to start the robot in the white starting square, select a color (red, blue or yellow), start it, and have it automatically navigate the field to find and deploy small foam cubes on the selected color squares.

The approach taken to accomplish this task is to set a pre-defined route that covers each square exactly once. At each line between squares, the robot will stop and check for the selected color, and decides to deploy a foam cube or not depending on the color detection. For robustness and ease of editing, the pre-defined route is accomplished using a finite state machine which actuates a PID controller (controlling DC motors) and switches states according to a line sensor.

![alt text](https://github.com/rithvikpillai/Search-and-Deploy-Bot/blob/main/finalprototype.png?raw=true)

<b> Hardware </b>
- x1 Skitter Chassis Frame (from AndyMark - am-4344a) // I recommend 3D printing this frame
- x1 Caster Ball (from Andymark - am-4353)
- x1 Arduino MEGA
- x1 Breadboard
- Jumper Wires
- Line Following Sensor / 5 IR Configuration (from AndyMark - am-4341)

<i> Differential Drive Sub-system </i> 
- x2 12 V DC Motor-Encoders (from AndyMark - am-4338)
- x1 L298N H-bridge Motor Driver
- x2 10K potentiometers
- x2 PLA wheels + rubber O-rings (from AndyMark - am-4354 & am-4340) // I recommend 3D printing the wheels
- x1 12 V Li-Ion Battery, 3000 mAh (from AndyMark - am-4347)

![alt text](https://github.com/rithvikpillai/Search-and-Deploy-Bot/blob/main/driveprototype.png?raw=true)

<i> Dispenser Sub-system </i>
- x1 3D Printed 5pc Dispenser Mechanisms // Unfortunately, I do not have access to the CAD files for the mechanism but its simple enough to design with reference to the chassis and the stepper motor dimensions
- x1 28BYJ-48 Stepper Motor
- x1 ULN2003 Stepper Motor Driver
- x1 TCS34725 Color Sensor
- x1 9V battery

![alt text](https://github.com/rithvikpillai/Search-and-Deploy-Bot/blob/main/dispensercad.png?raw=true)

![alt text](https://github.com/rithvikpillai/Search-and-Deploy-Bot/blob/main/dispenserprototype.png?raw=true)

![alt text](https://github.com/rithvikpillai/Search-and-Deploy-Bot/blob/main/colorsensor.png?raw=true)

<i> Input & Lighting Sub-system </i>
- x2 Red LEDs
- x2 Blue LEDs
- x2 Yellow LEDs
- x1 White LED
- x7 220 ohm resistors
- x1 Pushbutton
- x1 3-Way DIP Switch

Please refer to the electrical schematic provided to see how all of these components are wired, and the reference photos to see how the robot is set up, although it is fairly intuitive. The total cost of our project was ~ 70 USD, although we did have many of the parts on hand from the ME department and our own personal projects, which would bring the total cost estimate to ~ 150 USD.

<b> Software </b>

The software for this project runs entirely on Arduino IDE. The drive sub-system is controlled using a PID control loop that runs in the main loop, but its targets and characteristics are modified in the finite state machine. The dispenser sub-system is controlled using the stepper motor driver pins and a TCS34725 color sensing library. The input & lighting sub-system is controlled simply using digital pins on the Arduino. The finite state machine is switched based on outputs from the line sensor as well as timers.

<b><i> Input & Display Sub-system </i></b>

The input and display sub-system is fairly simple. A 3-way DIP switch is used to select the color and depending on which color is selected, a corresponding LED will be turned on. This selected color is also stored for later use. The pushbutton is used to indicate game start. The robot will not move until this pushbutton is pressed. Debouncing may be necessary for this, but in our case, since we don't care too much about what happens after the button is pressed, it was not required. Pressing this button will also light up the white LED indicator.

<b><i> Differential Drive Sub-system </i></b>

First, the encoder readings of each motor is read using the Encoder.h library. By rotating each motor wheel manually by 1 revolution, the number of counts per revolution can be determined which allow us to determine the wheel position. Assuming that the wheel only rolls and does not slip (which is not always the case), an estimated robot position can be localized. Using a simple timer, the current velocity of each motor can be estimated. Due to the nature of encoder pulses, this introduces a fair bit of noise into the velocity estimation. As such, an averaging filter is used. This does introduce a small delay into the system which should be kept in mind during experimental testing.

With these measurements available, a simple PID control loop can be designed to achieve a target velocity or target motor position either for straight line travel or precise rotation. For optimization, the PID control algorithm runs consistently in the main loop along with the encoder feedback reading, while the finite state machine selects PID gains to achieve target velocity for straight line travel or to achieve target motor positions for precise rotation.

<b><i> Dispenser Sub-system </i></b>

The dispenser sub-system is actuated using a stepper motor + driver which is controlled by the readings of a color sensor. From the TCS34725 library, readings of illuminance, red, green, and blue values are returned from the sensor output. Based on the specific application, 'if' conditionals are created with experimentally determined color thresholds from the physical playfield. When the output of the conditionals (red, blue, or yellow) matches the color selected in the input system, this color is sent to the correct LED indicator and  the stepper motor is actuated. In this code, a program is written to output exactly one step by writing a specific order of binary codes to the stepper driver pins (1000, 1100, 0100, 0110, 0010, 0011, 0001, 1001). This program is then called 104 times to rotate the stepper motor about 72 degrees to dispense exactly one foam cube. Alternatively, I would recommend simply using a library like Stepper.h to make your life easier.

<b> <i> Finite State Machine </i> </b>

<i> State 0: Starting State </i> 

The robot will not move until a color is selected and the game start button is pressed. The robot is intended to be on the white starting square during this state. When a color is selected and the button is pressed, it will transition to State 1.

<i> State 1: Straight Line Travel Until Line </i> 

The robot will move forward by executing constant velocity PID control to each motor until a line is detected by the line sensor. Once a line is detected, it will transition to State 2.

<i> State 2 & 3: Stop & Check State </i> 

State 2 is simply a transition state where the PID errors are reset after which the system moves to State 3. In this state, the robot will stop at the line, increment the number of lines traversed, and checks the color that the color sensor is currently detecting. If the correct color is detected, the dispenser mechanism is rotated 72 degrees which will drop one foam cube. This state will also check whether a turn is required based on the number of lines traversed. If it is, it will transition to the turn states, State 4 & 5. If it is not, it will transition to the straight line travel state, State 1.

<i> State 4: Move-to-Center State </i> 

The move-to-center state is a timed state that tells the robot to execute constant velocity PID control at 125 RPM for 1.2 seconds which is equivalent to moving forward by 1 foot. This is essentially meant to prep the robot for rotation by moving from the stopped line to the center of square. It will then move to State 5.

<i> State 5: Rotation State </i>

Once the robot has moved to the center of the square, the PID gains will be shifted to achieve precise rotation. Basically instead of having the same target velocity for each motor, we have equal and opposite target motor positions for each motor corresponding to a 90 degree turn.

<i> State 6: End State </i>

Finally, there is an end state where the robot stops moving, and turns off all LED indicators. This end state is triggered when the total number of lines traversed and total number of turns are completed. The way this is achieved is by stepping through two arrays (one for lines, one for turns) during the state transitions.

The pre-defined route shown in the report can be translated for this robot to two arrays: 

lineCount[] = {3, 1, 2, 1, 2, 1, 3, 3};

turnDir[] = {L, L, R, R, L, L, L, L};

The robot will run through States 1 -> 2 -> 3 -> 1 ... <i>three</i> times. After the 3rd time, it will complete a <i>left</i> turn by going to State 4 -> 5. It will then run through States 1 -> 2 -> 3 -> 1 ... <i>one</i> time, then complete another <i>left</i> turn by doing State 4 -> 5. Then it will run States 1 -> 2 -> 3 <i>two</i> times, then complete a <i>right</i> turn with the same process. This will be repeated a total of 8 times until the pre-defined route is completed. It's a bit odd to wrap one's head around, but is in fact a very intuitive, easily modifiable, and robust method to run through a pre-defined route.

