# Search-and-Deploy-Bot
A 2W Differential Drive Robot that searches a playfield for a certain color and deploys foam cubes on the right ones! For exact details, please refer to the final report in the repository. This design and implementation was completed with team members Yigit Karatas and Dylan Foster under the supervision of Dr. Laura Blumenschein at Purdue University as a part of ME 588 - Mechatronics.

The playfield is a 4x4 ft square field divided into a white starting sqaure and 15 color squares randomly assorted with the colors: red, blue and yellow. The objective of the project is purely academic: It is to start the robot in the white starting square, select a color (red, blue or yellow), start it, and have it automatically navigate the field to find and deploy small foam cubes on the selected color squares.

The approach taken to accomplish this task is to set a pre-defined route that covers each square exactly once. At each line between squares, the robot will stop and check for the selected color, and decides to deploy a foam cube or not depending on the color detection. For robustness and ease of editing, the pre-defined route is accomplished using a finite state machine which actuates a PID controller (controlling DC motors) and switches states according to a line sensor.

<b> Hardware </b>
- x1 Skitter Chassis Frame (from AndyMark - am-4344a) // I recommend 3D printing this frame
- x1 Caster Ball (from Andymark - am-4353)
- x1 Arduino MEGA
- x1 Breadboard
- Jumper Wires

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

<i> Input & Lighting System </i>
- x2 Red LEDs
- x2 Blue LEDs
- x2 Yellow LEDs
- x1 White LED
- x7 220 ohm resistors
- x1 Pushbutton

<b> Software </b>
State 0: The robot will not move until a color is selected and the game start button is pressed. The robot is intended to be on the white starting square during this state.

State 1: 

State 2:

State 3:

State 4:
