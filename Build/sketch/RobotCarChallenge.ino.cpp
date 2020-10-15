#include <Arduino.h>
#line 1 "/Users/daniel/Documents/School Work/FRC/RobotCarChallenge/RobotCarChallenge.ino"
#include <Servo.h>

#include "Functions.h"

#line 5 "/Users/daniel/Documents/School Work/FRC/RobotCarChallenge/RobotCarChallenge.ino"
void setup();
#line 9 "/Users/daniel/Documents/School Work/FRC/RobotCarChallenge/RobotCarChallenge.ino"
void loop();
#line 5 "/Users/daniel/Documents/School Work/FRC/RobotCarChallenge/RobotCarChallenge.ino"
void setup() {
    Functions.RobotInit();
}

void loop() {
    Functions.RobotExecute();
}
