// #include <Arduino.h>

#define CHA 6
#define IN1 18
#define IN2 5
#define IN3 17
#define IN4 16

#define ENA 19
#define ENB 4
#define CHB 7


const int freq = 30000;
const int resolution = 8;

void init_motor()
{
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);
	pinMode(ENA, OUTPUT);
	pinMode(ENB, OUTPUT);
	// pinMode(LIGHT, OUTPUT);

	ledcSetup(CHA, freq, resolution);
	ledcSetup(CHB, freq, resolution);

	ledcAttachPin(ENA, CHA);
	ledcAttachPin(ENB, CHB);


}

void set_speed(int speed_1, int speed_2)
{
	ledcWrite(CHA, speed_1);
	ledcWrite(CHB, speed_2); 

}

void rotate_left()
{
	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, LOW);
	digitalWrite(IN3, HIGH);
	digitalWrite(IN4, LOW);
}

void rotate_right()
{
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, HIGH);
	digitalWrite(IN3, LOW);
	digitalWrite(IN4, HIGH);
}

void di_chuyen_lui()
{
	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, LOW);
	digitalWrite(IN3, LOW);
	digitalWrite(IN4, HIGH);
}

void di_chuyen_thang()
{
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, HIGH);
	digitalWrite(IN3, HIGH);
	digitalWrite(IN4, LOW);
}

void stop()
{
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);
	digitalWrite(IN3, LOW);
	digitalWrite(IN4, LOW);
}

void control_robot(char data_control)
{
	// digitalWrite(LIGHT, LOW);
	if (data_control == 't')
	{
		di_chuyen_thang();
		// set_speed(255,255);
	}

	else if (data_control == 'h')
	{
		rotate_left();
		// set_speed(255,255);

	}

	else if (data_control == 'f')
	{
		rotate_right();
		// set_speed(255,255);
	}
	else if (data_control == 'g')
	{
		di_chuyen_lui();
	}
	else if (data_control == 'o')
	{
		stop();
	}
	else{
		stop();
	}
}