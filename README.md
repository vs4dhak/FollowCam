<h1>FollowCam:</h1>

The multi-rotor system shown below can track the human form autonomously. Currently the multi-rotor tracks the user only by rotation, more dynamic movement capability is being worked on at the moment. The system uses image processing in order to detect and track the human form. Once the image processing is confident that a user is in the field of vision, then the control system is activated and begins to send commands to the motors to rotate accordingly. A PID controller (Proportional, Intergral and Derivative) is used to control the amount of power going to each motor.

The system is designed by students Anish Agarwal, Victor Ojukwu, Stephen Ting, Zeki Sherif and Shiva Talwar at the University of Waterloo.

The major creation of this project was the design of a compact and light-weight embedded system which can do both image processing and real-time controls analysis. The design which we created integrated a single board computer, the Raspberry PI and two standard microcontrollers. The Raspberry PI was used for image processing and the two microcontrollers were responsible for control theory analysis and outputting to the motors. The custom designed embedded system is shown in the 4 figures below. The pictures show the two microcontrollers, the side view of the entire system, the integrated Raspberry PI and the sensor board.

The reset of the quadcopter contained stock parts consisting of Electronic Speed Controllers, Brushless DC Motors ("The Baby Beast"), a Lithium Polymer Battery, Carbon Fibre frame and 5 inch propellers. Different configurations were tried to produce the optimal design. Video demonstrations of the working system are at the bottom of the page.

![alt tag](https://raw.githubusercontent.com/veda-s4dhak/FollowCam/master/Poster.PNG?raw=true)

#Testing Footage
Video 1: https://www.youtube.com/watch?v=D9XE87QOBpg </br>
Video 2: https://www.youtube.com/watch?v=N3HvmY-s_i8 </br>
Video 3: https://www.youtube.com/watch?v=aA82wMc2FgU
