# Search-and-Deploy-Bot
A 2W Differential Drive Robot that searches a playfield for a certain color and deploys foam cubes on the right ones! For further details, please refer to the final report in the repository. This design and implementation was completed with team members Yigit Karatas and Dylan Foster under the supervision of Dr. Laura Blumenschein at Purdue University as a part of ME 588 - Mechatronics.

The playfield is a 4x4 ft square field divided into a white starting sqaure and 15 color squares randomly assorted with the colors: red, blue and yellow. The objective of the project is purely academic: It is to start the robot in the white starting square, select a color (red, blue or yellow), start it, and have it automatically navigate the field to find and deploy small foam cubes on the selected color squares.

The approach taken to accomplish this task is to set a pre-defined route that covers each square exactly once. At each line between squares, the robot will stop and check for the selected color, and decides to deploy a foam cube or not depending on the color detection. For robustness and ease of editing, the pre-defined route is accomplished using a finite state machine which actuates a PID controller (controlling DC motors) and switches states according to a line sensor.

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

<i> Dispenser Sub-system </i>
- x1 3D Printed 5pc Dispenser Mechanism // Unfortunately, I do not have access to the CAD files for the mechanism but its simple enough to design with reference
- x1 28BYJ-48 Stepper Motor
- x1 ULN2003 Stepper Motor Driver
- x1 TCS34725 Color Sensor
- x1 9V battery

<i> Input & Lighting Sub-system </i>
- x2 Red LEDs
- x2 Blue LEDs
- x2 Yellow LEDs
- x1 White LED
- x7 220 ohm resistors
- x1 Pushbutton

<b> Software </b>

The software for this project runs entirely on Arduino IDE. The drive sub-system is controlled using a PID control loop that runs in the main loop, but its targets and characteristics are modified in the finite state machine. The dispenser sub-system is controlled using the stepper motor driver pins and a TCS34725 color sensing library. The input & lighting sub-system is controlled simply using digital pins on the Arduino. The finite state machine is switched based on outputs from the line sensor as well as timers.

For optimization, the PID control algorithm runs consistently in the main loop along with the encoder feedback reading, while the finite state machine selects PID gains for target velocity for straight line travel or for target motor positions for precise rotation.


<b> State 0: Starting State </b> 

The robot will not move until a color is selected and the game start button is pressed. The robot is intended to be on the white starting square during this state. When a color is selected and the button is pressed, it will transition to State 1.

<b> State 1: Straight Line Travel Until Line </b> 

The robot will move forward by executing constant velocity PID control to each motor until a line is detected by the line sensor. Once a line is detected, it will transition to State 2.

<b> State 2 & 3: Stop & Check State </b> 

State 2 is simply a transition state where the PID errors are reset after which the system moves to State 3. In this state, the robot will stop at the line, increment the number of lines traversed, and checks the color that the color sensor is currently detecting. If the correct color is detected, the dispenser mechanism is rotated 72 degrees which will drop one foam cube. This state will also check whether a turn is required based the number of lines traversed. If it is, it will transition to the turn states, State 4 & 5. If it is not, it will transition to the straight line travel state, State 1.

<b> State 4: Move-to-Center State </b> 

<b> State 5: Rotation State </b>

<b> State 6: End State </b>
