#include <ESP32Servo.h>
#include <vector>
const int SERVO_FORWARD_STEP_ANGLE = 1;
const int SERVO_BACKWARD_STEP_ANGLE = -1;
bool flag_end = false;
bool flag_start = true;

struct ServoPins
{
    Servo servo;
    int servoPin;
    String servoName;
    int initialPosition;
};
std::vector<ServoPins> servoPins =
    {
        {Servo(), 14, "Base", 106},
        {Servo(), 15, "Shoulder", 169},
        {Servo(), 13, "Elbow", 58},
        {Servo(), 12, "Gripper", 10},
};

void init_control()
{
    pinMode(2, OUTPUT);
    for (int i = 0; i < servoPins.size(); i++)
    {
        servoPins[i].servo.attach(servoPins[i].servoPin);
        // servoPins[i].servo.write(servoPins[i].initialPosition);
    }
}

void writeServoValues(int servoIndex, int servoMoveStepSize, bool setvalue = false)
{
    int servoPosition;
    int positioncurrent;
    if (setvalue)
    {
        servoPosition = servoMoveStepSize;
    }
    else
    {
        servoPosition = servoPins[servoIndex].initialPosition;
        servoPosition = servoPosition + servoMoveStepSize;
    }
    if (servoPosition > 180 || servoPosition < 0)
    {
        return;
    }
    // if (abs(positioncurrent - servoPosition) <= 2)
    // {

    servoPins[servoIndex].servo.write(servoPosition);
    // }
    servoPins[servoIndex].initialPosition = servoPosition;
    // }
    //   delay(10);
}

// void move_servo(char data_control){
//     data_control
// }
bool control_keyboard(char data_control)
{
    digitalWrite(2, HIGH);
    if (data_control == 'a')
    {
        writeServoValues(0, SERVO_BACKWARD_STEP_ANGLE);
        return 1;
    }
    else if (data_control == 'd')
    {
        writeServoValues(0, SERVO_FORWARD_STEP_ANGLE);
        return 1;
    }

    else if (data_control == 'w')
    {
        writeServoValues(1, SERVO_BACKWARD_STEP_ANGLE);
        return 1;
    }
    else if (data_control == 's')
    {
        writeServoValues(1, SERVO_FORWARD_STEP_ANGLE);
        return 1;
    }

    else if (data_control == 'u')
    {
        writeServoValues(2, SERVO_BACKWARD_STEP_ANGLE);
        return 1;
    }
    else if (data_control == 'b')
    {
        writeServoValues(2, SERVO_FORWARD_STEP_ANGLE);
        return 1;
    }

    else if (data_control == 'l')
    {
        writeServoValues(3, SERVO_BACKWARD_STEP_ANGLE);
        return 1;
    }
    else if (data_control == 'r')
    {
        writeServoValues(3, SERVO_FORWARD_STEP_ANGLE);
        return 1;
    }
    else
    {
        return 0;
    }
    // }
}

void move_point(int start[], int end[])
{
    flag_end = false;

    for (int i = 0; i < 4; i++)
    {
        int check;
        check = start[i] - end[i];
        if (check > 0)
        {
            for (int j = start[i]; j >= end[i]; j--)
            {
                servoPins[i].servo.write(j);
                Serial.println(j);
                delay(10);
            }
            servoPins[i].initialPosition = end[i];
        }
        else if (check < 0)
        {
            for (int j = start[i]; j <= end[i]; j++)
            {
                servoPins[i].servo.write(j);
                Serial.println(j);
                delay(10);
            }
            servoPins[i].initialPosition = end[i];
        }
        else
        {
            servoPins[i].initialPosition = end[i];
            delay(10);
        }
    }
    flag_start = true;
    flag_end = true;
}