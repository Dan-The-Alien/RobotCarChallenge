# Daniel's Robot Car Challenge Source Code
## Overview
I chose to use a combination of closed loop *(controlling the car based on sensor input)* and hard-coded paths. The main loop has four stages:
1. Stage One: The ultrasonic sensor turns to the right. The robot tries to keep a distance of 12 inches from the box. Once the robot passes two corners, this stage exits to stage 2.
2. Stage Two: The ultrasonic sensor turns to the left. The robot keeps a distance of 10 inches from the box. Stage 3 begins when the robot passes four corners in stage 2.
3. Stage Three: The ultrasonic sensor turns to the right, and the robot keeps a distance of 10 inches. Stage 3 exits after 2 corners.
4. Stage Four: The robot keeps a distance of 16 inches. Once the line sensor detects a discrepency between two of its sensors, the robot stops

## Random Unorganized Thoughts
### *Why are there multiple files?*
There are multiple files because I intended to use many different files, one for bare metal control, one for helper fuctions, one for main fucntions (such as `Go(), Stop(), Test()`). However, I could not figure out how do this, so I ended up having one file with everything (the Functions.h file) and one file that has the Arduino loops (RobotCarChallenge.ino). The build folder is where I compiled the project to, and the Log folder is mostly unsued, although it contains one log that I didn't bother decoding to ASCII.

### *Why is the code so unorganized and unoptimized?*
Because I didn't feel like organizing or optimizing it. This is a one-off project, all I needed was for it to work once. No one else is collaborating on it, so I organized it the way that worked for me. You might find remenets of prior ideas in the code that I didn't remove, and again, this was simply because I didn't get around to it.

### *What is the serial port?*
I used the bluetooth module to control my car. The car used serial over bluetooth, and listens for any of these three characters `'s', 'g', 't'`. Each of these call their respective functions: Stop(), Go(), and Test(). Logs are also sent over the serial port, and are labled with either ERROR or INFO. Unfortunalty I couldn't view these logs over bluetooth, but I could test the car with it connected over usb to my computer, and I could read the logs over usb.

## Things I would have liked to do
The main improvement I need is consistency. The car is very inconsistent, sometimes it works, sometimes it doesn't. However for this particular challenge, consistency is not a consideration, as it only has to go around the track once, when you're videoing it. I would probobly start by implimenting a PID loop for the control of the car, as that would make the car more consistant. I also would have liked to be able to make the car stop in the exact center of the end box.

*More detail coming soon. Maybe. Probobly not. Look through the code if you want more detail, it's all there.*