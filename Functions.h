class Functions {

private:

#include <Servo.h>

    #define ENA 5 //Left Wheel Speed (Maybe?) 8-bit Analogue
    #define ENB 6 //Right Wheel Speed (Maybe?) 8-bit Analogue
    #define IN1 7 //Left Wheels Digital
    #define IN2 8 //Left Wheels Ditigal
    #define IN3 9 //Right Wheels Digital
    #define IN4 11 //Right Wheels Digital
    #define LED 13 //Bluetooth Module LED

    #define LT_L !digitalRead(2)
    #define LT_M !digitalRead(4)
    #define LT_R !digitalRead(10)

    /* Constants */
    int kSlowestSpeed = 85; //8-bit
    double kGoLinearSpeed = 1;
    double kGoRotationSpeed = .85;

    int Echo = A4;
    int Trig = A5;
    int Stage = 1;
    int distanceFromTarget;
    int emptySpaceDetection = 0;
    int onTargetDetection = 0;
    int cornersPassed = 0;
    

    char incomingByte = 's';

    boolean stop = false;

    Servo ultrasonicServo;

public:
    //===DRIVEBASE CONTROLLER===//

    //Direct control of left wheels from direction and speed
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

    //Direct control of right wheels from direction and speed
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

    //Consolidates both sides into one method
    void DifferentialDrive(double leftSpeed, double rightSpeed) {
        LeftSpeed(leftSpeed);
        RightSpeed(rightSpeed);
    }

    //Arcacde Drive for manual control
    void ArcadeDrive(double speed, double rotation) {
        double plus = speed + rotation;
        double minus = speed - rotation;
        DifferentialDrive(constrain(plus, -1, 1), constrain(minus, -1, 1));
    }

    //===ULTRASONIC CONTROLLER===//

    int GetDistanceFromTarget() { //in centimeters
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

    void TurnUltrasonicSensor(int angle) {
        ultrasonicServo.write(angle);
    }

    int TurnAndGetDistanceFromTarget(int angle) {
        TurnUltrasonicSensor(angle);
        delay(360);
        return GetDistanceFromTarget();
    }

    //===LINE SENSOR===//
    boolean RightDetectedDark() {
        return LT_R;
    }

    boolean MiddleDetectedDark() {
        return LT_M;
    }

    boolean LeftDetectedDark() {
        return LT_L;
    }

    //===COMPOUND FUCNCTIONS===//
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
    int In2Cm(double inches) {
        return inches * 2.54;
    }
    double Cm2In(double cm) {
        return cm/2.54;
    }


    //===TEST FUNCTIONS===//
    void ArcadeTest() {
        delay(1000);
        ArcadeDrive(1, 0);
        delay(1000);
        ArcadeDrive(1, .75);
        delay(1000);
        ArcadeDrive(1, -.75);
    }

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
    void Test() {
    }


    //The main function. This is what controls the robot, and tells it where to turn.
    void Go() {
        Serial.println("Go");
        if(Stage == 1) {
            Serial.println("INFO: Stage One Initiated");
            int stage1Angle = 0;
            TurnUltrasonicSensor(stage1Angle);
            KeepDistance(kGoLinearSpeed, kGoRotationSpeed, 12, stage1Angle);
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
            int stage4angle = 0;
            KeepDistance(kGoLinearSpeed, kGoRotationSpeed, 16, stage4angle);
            if(LeftDetectedDark() != MiddleDetectedDark() || MiddleDetectedDark() != RightDetectedDark()) {
                Stage++;
            }
        } else { 
            Serial.println("INFO: Circuit Complete!");
            Stop();
        }
    }

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

};
Functions Functions;