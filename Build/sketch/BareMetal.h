class BareMetal
{
private:
    #include <Servo.h>

    Servo ultrasonicServer;

    #define ENA 5 //Left Wheel Speed (Maybe?) 8-bit Analogue
    #define ENB 6 //Right Wheel Speed (Maybe?) 8-bit Analogue
    #define IN1 7 //Left Wheels Digital
    #define IN2 8 //Left Wheels Ditigal
    #define IN3 9 //Right Wheels Digital
    #define IN4 11 //Right Wheels Digital
    #define LED 13 //Bluetooth Module LED

    /* Constants */
    int kSlowestSpeed = 100; //8-bit

    int Echo = A4;
    int Trig = A5;

public:
    /* DRIVEBASE CONTROLLER */

    //Direct control of right wheels from direction and speed
    void LeftWheel8Bit(char direction, int speed){
        if (direction = 'F') {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
        } else if (direction = 'B') {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
        } else {
            Serial.println("ERROR: %s not a valid value for LeftWheel8Bit direction", direction);
        }

        analogWrite(ENA, speed);
    }

    //Direct control of right wheels from direction and speed
    void RightWheel8Bit(char direction, int speed){
        if (direction = 'F') {
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
        } else if (direction = 'B') {
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
        } else {
            Serial.println("ERROR: " + direction + " not a valid value for RightWheel8Bit direction");
        }

        analogWrite(ENB, speed);
    }

    //Control of left wheels with range from -1 to 1
    void LeftSpeed(double speed) {
        int adjustedSpeed = abs(speed * (255 - kSlowestSpeed)) + kSlowestSpeed;
        char direction;
        if (speed >= 0) {direction == 'F';} else {direction = 'B';}

        LeftWheel8Bit(direction, adjustedSpeed);
    }

    //Control of left wheels with range from -1 to 1
    void RightSpeed(double speed) {
        int adjustedSpeed = abs(speed * (255 - kSlowestSpeed)) + kSlowestSpeed;
        char direction;
        if (speed >= 0) {direction == 'F';} else {direction = 'B';}

        RightWheel8Bit(direction, adjustedSpeed);
    }

    //Consolidates both sides into one method
    void DifferentialDrive(double leftSpeed, double rightSpeed) {
        LeftSpeed(leftSpeed);
        RightSpeed(rightSpeed);
    }

    //Arcacde Drive for manual control
    void ArcadeDrive(double speed, double rotation) {
        if(rotation >=0) {
            DifferentialDrive(speed, speed/rotation);
        } else {
            DifferentialDrive(speed/-rotation, speed);
        }
    }

    /* ULTRASONIC CONTROLLER */

    int GetDistanceFromTarget() {
        digitalWrite(Trig, LOW);
        delayMicroseconds(2);
        digitalWrite(Trig, HIGH);
        delayMicroseconds(20);
        digitalWrite(Trig, LOW);
        float Fdistance = pulseIn(Echo, HIGH);
        Fdistance = Fdistance / 58;
        return (int)Fdistance;
    }
};