//*********************************************//
//There are two different type of values used in
//this project: 8 bit values, which is an integer
//between 0 and 255, and values between -1 and 1.
//-1 to 1 is much easier to do math with, so it 
//is my prefered way of dealing variable inputs.
//It is also how the FRC robots are programmed.
//********************************************//

#include <Servo.h>

    #define ENA 5 //Left Wheel Speed 8-bit Analogue
    #define ENB 6 //Right Wheel Speed 8-bit Analogue
    #define IN1 7 //Left Wheels Digital
    #define IN2 8 //Left Wheels Ditigal
    #define IN3 9 //Right Wheels Digital
    #define IN4 11 //Right Wheels Digital
    #define LED 13 //Bluetooth Module LED (Not used)

    #define LT_L !digitalRead(2) //Left line sensor port
    #define LT_M !digitalRead(4) //Middle line sensor port
    #define LT_R !digitalRead(10) //Right line sensor port

    //===CONSTANTS===//
    int kSlowestSpeed = 85; //8-bit value (This means that this value uses 8
                            // bits of data, or 8 zeros or ones. The highest 
                            // number you can make is 255, so this is a value between 0 and 255)
    double kGoLinearSpeed = 1; //The speed at which the robot drives. Is used in the Go() function
    double kGoRotationSpeed = .85; //The speed at which the robot turns. Is used in the Go() function

    //===VARIABLES===//

    int Echo = A4; // Used for ultrasonic sensor
    int Trig = A5; //Used for ultrasonic sensor
    int Stage = 1; // Initial stage for Go() function
    int distanceFromTarget; // Initializing ultrasonic sensor reading
    int emptySpaceDetection = 0; //Initial value used in Go() function
    int onTargetDetection = 0; //Initial value used in Go() function
    int cornersPassed = 0; //Initial value used in Go() function
    
    char incomingByte = 's'; //Initial car state

    boolean stop = false; //I don't remeber exactly why I have this here

    Servo ultrasonicServo; //Initialize servo

    //===DRIVEBASE CONTROLLER===//

    //Direct bare metal control of left wheels from direction and speed
    void LeftWheel8Bit(char direction, int speed){
        if (direction == 'F') {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
        } else if (direction == 'B') {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
        } else {
            Serial.println("ERROR: not a valid value for LeftWheel8Bit direction");
        }

        analogWrite(ENA, speed);
    }

    //Direct bare metal control of right wheels from direction and speed
    void RightWheel8Bit(char direction, int speed){
        if (direction == 'F') {
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
        } else if (direction == 'B') {
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
        } else {
            Serial.println("ERROR: not a valid value for RightWheel8Bit direction");
        }

        analogWrite(ENB, speed);
    }

    //Control of left wheels with range from -1 to 1
    void LeftSpeed(double speed) {
        int adjustedSpeed;
        double absSpeed = abs(speed);
        if (speed != 0) {
            adjustedSpeed = (absSpeed * (255 - kSlowestSpeed));
        } else {adjustedSpeed = 0;}
        char direction;
        if (speed >= 0) {direction = 'F';} else {direction = 'B';}

        LeftWheel8Bit(direction, adjustedSpeed);
    }

    //Control of right wheels with range from -1 to 1
    void RightSpeed(double speed) {
        int adjustedSpeed;
        double absSpeed = abs(speed);
        if (speed != 0) {
            adjustedSpeed = (absSpeed * (255 - kSlowestSpeed));
        } else {adjustedSpeed = 0;}
        char direction;
        if (speed >= 0) {direction = 'F';} else {direction = 'B';}

        RightWheel8Bit(direction, adjustedSpeed);
    }

    //Consolidates control of both sides into one method
    void DifferentialDrive(double leftSpeed, double rightSpeed) {
        LeftSpeed(leftSpeed);
        RightSpeed(rightSpeed);
    }

    //Arcacde Drive for manual control (arcade drive is where one axis controls speed, and the other rotation)
    void ArcadeDrive(double speed, double rotation) {
        double plus = speed + rotation;
        double minus = speed - rotation;
        DifferentialDrive(constrain(plus, -1, 1), constrain(minus, -1, 1));
    }

    //===ULTRASONIC CONTROLLER===//

    // Gets the distance from the ultrasonic sensor and returns it in inches
    int GetDistanceFromTarget() { 
        digitalWrite(Trig, LOW);
        delayMicroseconds(2);
        digitalWrite(Trig, HIGH);
        delayMicroseconds(20);
        digitalWrite(Trig, LOW);
        float Fdistance = pulseIn(Echo, HIGH);
        Fdistance = Fdistance / 58;
        Fdistance = Cm2In(Fdistance);
        return (int)Fdistance;
    }

    //Turns servo with ultrasonic sensor to angle from 0 (all the way to the right) to 180 (all the way to the left)
    void TurnUltrasonicSensor(int angle) {
        ultrasonicServo.write(angle);
    }

    //Turns ultrasonic sensor and then gets distance from target.
    int TurnAndGetDistanceFromTarget(int angle) {
        TurnUltrasonicSensor(angle);
        delay(360);
        return GetDistanceFromTarget();
    }

    //===LINE SENSOR===//
    //Checks to see if the right sensor detected a dark spot on the ground
    boolean RightDetectedDark() {
        return LT_R;
    }

    //Checks to see if the middle sensor detected a dark spot on the ground
    boolean MiddleDetectedDark() {
        return LT_M;
    }

    //Checks to see if the left sensor detected a dark spot on the ground
    boolean LeftDetectedDark() {
        return LT_L;
    }

    //===COMPOUND FUCNCTIONS===//
    //Keeps a set distance from the object detected by the ultrasonic sensor. Requires linear speed to follow at,
    //a rotational speed to know how fast to turn, a distance (in inches) to know how far to stay from the object,
    //and the angle that the servo is currently turned to, to know which direction it needs to turn. This function could use improvement.
    void KeepDistance(double linearSpeed, double rotationalSpeed, double distance, int servoAngle) {
        if(servoAngle >= 0 && servoAngle < 60) {
            if(GetDistanceFromTarget() > distance) {
                ArcadeDrive(linearSpeed, rotationalSpeed);
            } else if(GetDistanceFromTarget() < distance) {
                ArcadeDrive(linearSpeed, -rotationalSpeed);
            } else {
                ArcadeDrive(linearSpeed, 0);
            }
        } else if(servoAngle >= 60 && servoAngle < 120) {
            if(GetDistanceFromTarget() > distance) {
                ArcadeDrive(linearSpeed, 0);
            } else if(GetDistanceFromTarget() < distance) {
                ArcadeDrive(-linearSpeed, 0);
            } else {
                ArcadeDrive(0, 0);
            }
        } else if(servoAngle >= 120 && servoAngle <= 180) {
            if(GetDistanceFromTarget() > distance) {
                ArcadeDrive(linearSpeed, -rotationalSpeed);
            } else if(GetDistanceFromTarget() < distance) {
                ArcadeDrive(linearSpeed, rotationalSpeed);
            } else {
                ArcadeDrive(linearSpeed, 0);
            }
        } else {Serial.println("ERROR: Invalid argument passed to function KeepDistance");}
    }

    //===UTILITIES===//
    //Converts inches to centimeters
    int In2Cm(double inches) {
        return inches * 2.54;
    }
    //Converts centimeters to inches
    double Cm2In(double cm) {
        return cm/2.54;
    }


    //===TEST FUNCTIONS===//
    
    //A test I created when I needed to test if the Arcade Drive I made worked
    void ArcadeTest() {
        delay(1000);
        ArcadeDrive(1, 0);
        delay(1000);
        ArcadeDrive(1, .75);
        delay(1000);
        ArcadeDrive(1, -.75);
    }

    //A test I made to test the differential drive.
    void DifferentialTest() {
        delay(1000);
        DifferentialDrive(1, 1);
        delay(1000);
        DifferentialDrive(-1, -1);
        delay(1000);
        DifferentialDrive(1, -1);
        delay(1000);
        DifferentialDrive(-1, 1);
        delay(1000);
    }

    //===MAIN FUNCTIONS==//

    //Add things that you need to test in this function.
    void Test() {
    }


    //The main function. This is what controls the robot, and tells it where to turn.
    void Go() {
        Serial.println("Go");

        // This function operates in stages. When a stage is complete, it will increase the stage variable by one.
        if(Stage == 1) {
            Serial.println("INFO: Stage One Initiated");
            int stage1Angle = 0; //servo angle for this stage
            TurnUltrasonicSensor(stage1Angle);
            KeepDistance(kGoLinearSpeed, kGoRotationSpeed, 12, stage1Angle);

            //The rest of this stage is for determining when the stage is complete
            if(GetDistanceFromTarget() > 20) {
                //If the sensor returns a value greater than 20, the variable "emptySpaceDetection" will increase by one. 
                //I am assuming that if the sensor detects a distance greater than 20, it has passed a corner. Also, since
                //it is greater than the follow distance, the robot will be turning towards the barrier.
                emptySpaceDetection++;
            }
            if(emptySpaceDetection >= 10) {
                //If empty space was detected for at least 10 cycles, the robot starts looking for the box.
                if(GetDistanceFromTarget() > 8 && GetDistanceFromTarget() < 16) {
                    onTargetDetection++;
                }
            }
                //If empty space was found and then a target was detected, the robot assumes that it passed a corner,
                //and increases the cornerPassed variable by one. It also resets the emptySpaceDetection and onTargetDetection.
            if(emptySpaceDetection > 10 && onTargetDetection > 5) {
                cornersPassed++;
                emptySpaceDetection = 0;
                onTargetDetection = 0;
            }
                //Once two corners have been passed, the robots stars the next stage, and resets corners passed to 0
            if(cornersPassed == 2) {
                Stage++;
                cornersPassed = 0;
                Serial.println("INFO: Stage One Complete!");
            }
        } else if (Stage == 2) {
            Serial.println("INFO: Stage Two Initiated");
            int stage2Angle = 180;
            TurnUltrasonicSensor(stage2Angle);
            KeepDistance(kGoLinearSpeed, kGoRotationSpeed, 10, stage2Angle);
            if(GetDistanceFromTarget() > 20) {
                emptySpaceDetection++;
            }
            if(emptySpaceDetection >= 10) {
                if(GetDistanceFromTarget() > 8 && GetDistanceFromTarget() < 16) {
                    onTargetDetection++;
                }
            }
            if(emptySpaceDetection > 10 && onTargetDetection > 5) {
                cornersPassed++;
                emptySpaceDetection = 0;
                onTargetDetection = 0;
            }
            if(cornersPassed == 4) {
                Stage++;
                cornersPassed = 0;
                Serial.println("INFO: Stage Two Complete!");
            }
        } else if (Stage == 3) {
            Serial.println("INFO: Stage Three Initiated");
            int stage3Angle = 0;
            TurnUltrasonicSensor(stage3Angle);
            KeepDistance(kGoLinearSpeed, kGoRotationSpeed, 10, stage3Angle);
            if(GetDistanceFromTarget() > 20) {
                emptySpaceDetection++;
            }
            if(emptySpaceDetection >= 10) {
                if(GetDistanceFromTarget() > 8 && GetDistanceFromTarget() < 16) {
                    onTargetDetection++;
                }
            }
            if(emptySpaceDetection > 10 && onTargetDetection > 5) {
                cornersPassed++;
                emptySpaceDetection = 0;
                onTargetDetection = 0;
            }
            if(cornersPassed == 2) {
                Stage++;
                cornersPassed = 0;
                Serial.println("INFO: Stage Three Complete!");
            }
        } else if(Stage == 4) {
            //For this last stage, the robot is looking for a place where two of the line sensors are not equal to each other.
            //The assumption is that it will differentiate between my dark floor and the white piece of paper, and that it
            //will be at an angle, so that some sensors return true while others return false. This will add one to the stage.
            int stage4angle = 0;
            KeepDistance(kGoLinearSpeed, kGoRotationSpeed, 16, stage4angle);
            if(LeftDetectedDark() != MiddleDetectedDark() || MiddleDetectedDark() != RightDetectedDark()) {
                Stage++;
            }
        } else { 
            //Once the stage equals 5, the car will stop, as it completed the loop.
            Serial.println("INFO: Circuit Complete!");
            Stop();
        }
    }

    //How to stop the robot. It sets the wheels to 0, resets the go() function, resets the cornersPassed,
    //onTargetDetection, and emptySpaceDetection, and sets the servo to 90degrees (straight ahead)
    void Stop() {
        Serial.println("Stop");
        incomingByte = 's';
        DifferentialDrive(0, 0);
        Stage = 1;
        cornersPassed = 0;
        onTargetDetection = 0;
        emptySpaceDetection = 0;
        TurnUltrasonicSensor(90);
    }

    //Initiates the robot. This is the only function that is called by the Setup() loop
    void RobotInit() {
        Serial.begin(9600);
        ultrasonicServo.attach(3, 700, 2400);
        pinMode(LED, OUTPUT); 
        pinMode(IN1,OUTPUT);
        pinMode(IN2,OUTPUT);
        pinMode(IN3,OUTPUT);
        pinMode(IN4,OUTPUT);
        pinMode(ENA,OUTPUT);
        pinMode(ENB,OUTPUT);
        pinMode(Echo, INPUT);    
        pinMode(Trig, OUTPUT);  
        pinMode(2, INPUT);
        pinMode(4, INPUT);
        pinMode(10, INPUT);
        Serial.println("INFO: Robot Initiated!");
    }

    //Is executed every cycle. Looks to see if the serial port has sent a different value, and runs the 
    //correct function. 
    void RobotExecute() {
        if (Serial.available() >= 1) {
            char incomingByteUnfiltered = Serial.read();
            if(incomingByteUnfiltered == 't' || incomingByteUnfiltered ==  'g' || incomingByteUnfiltered == 's') {
                incomingByte = incomingByteUnfiltered;
            }
        }
        switch(incomingByte) {
            case 't': Test(); break;
            case 'g': Go(); break;
            case 's': Stop(); break;
            default: 
            Serial.println(incomingByte);
            break;
        }
    }


//===ARDUINO FUNCTIONS===//
//Arduino provided setup
void setup() {
    RobotInit();
}

//Arduino provided loop
void loop() {
    RobotExecute();
}