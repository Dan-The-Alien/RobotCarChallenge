# Daniel's Robot Car Challenge Source Code
## Installation
Please follow the below steps to install this code and use it.
1. Download the code by clicking on the green "Code" button in the top right corner of the github page, then clicking "Download ZIP", or by clicking [here](https://github.com/EcoRobotics-Daniel/RobotCarChallenge/archive/master.zip).
2. Unpack the zip file, and open the "RobotCarChallenge.ino" file in your Arduino IDE
3. Unplug the bluetooth module from the car, connect the car to the computer, and upload the code.
4. Disconnect the car from the computer, reconnect the bluetooth module.
5. Download the Elegoo Ble Tool from either the [Apple App Store](https://apps.apple.com/us/app/elegoo-ble-tool/id1441940983) or the [Google Play Store](https://play.google.com/store/apps/details?id=air.com.elegoo.elegooTool&hl=en_US).
6. Open the app, tap on the "Smart Robot Car" tile, open the "DIY Control panel".
7. Edit one of the buttons by long tapping it, and setting "Button Name" to "Stop", "Message" to "s" (lowercase), and check the "Character" box. Choose whatever color you want.
8. Repeat step 7 with two other buttons, with values "Go", "g", and "Test", "t".
9. Turn on the car, and pair it with the phone.
10. Your car is now ready to run! Set it on the track, and press go.
---
## Overview
I chose to use a combination of closed loop *(controlling the car based on sensor input)* and hard-coded paths. The main loop has four stages:
1. Stage One: The ultrasonic sensor turns to the right. The robot tries to keep a distance of 12 inches from the box. Once the robot passes two corners, this stage exits to stage 2.
2. Stage Two: The ultrasonic sensor turns to the left. The robot keeps a distance of 10 inches from the box. Stage 3 begins when the robot passes four corners in stage 2.
3. Stage Three: The ultrasonic sensor turns to the right, and the robot keeps a distance of 10 inches. Stage 3 exits after 2 corners.
4. Stage Four: The robot keeps a distance of 16 inches. Once the line sensor detects a discrepency between two of its sensors, the robot stops

## Random Unorganized Thoughts
### *Why are there multiple files?*
The build folder is where I compiled the project to, and the Log folder is mostly unsued, although it contains one log that I didn't bother decoding to ASCII. The only file you need to worry about is the RobotCarChallenge.ino file.

### *Why is the code so unorganized and unoptimized?*
Because I didn't feel like organizing or optimizing it. This is a one-off project, all I needed was for it to work once. No one else was intened to see it or use it, so I organized it the way that worked for me. You might find remenets of prior ideas in the code that I didn't remove, and again, this was simply because I didn't get around to it.

### *What is the serial port?*
I used the bluetooth module to control my car. The car used serial over bluetooth, and listens for any of these three characters `'s', 'g', 't'`. Each of these call their respective functions: Stop(), Go(), and Test(). Logs are also sent over the serial port, and are labled with either ERROR or INFO. Unfortunalty I couldn't view these logs over bluetooth, but I could test the car with it connected over usb to my computer, and I could read the logs over usb.

## Things I would have liked to do
The main improvement I need is consistency. The car is very inconsistent, sometimes it works, sometimes it doesn't. However for this particular challenge, consistency is not a consideration, as it only has to go around the track once, when you're videoing it. I would probobly start by implimenting a PID loop for the control of the car, as that would make the car more consistant *(As well as faster. Hint hint.)*. I also would have liked to be able to make the car stop in the exact center of the end box.

*More detail coming soon. Maybe. Probobly not. Look through the code if you want more detail, it's all there.*