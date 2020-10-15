#include <Servo.h>

#include "Functions.h"

void setup() {
    Functions.RobotInit();
}

void loop() {
    Functions.RobotExecute();
}