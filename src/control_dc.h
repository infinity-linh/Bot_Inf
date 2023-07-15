// #include <Arduino.h>

#define IN1 14
#define IN2 15
#define IN3 13
#define IN4 12

#define ENA 16
#define ENB 2

int direct;

void init_dc()
{
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);
	pinMode(ENA, OUTPUT);
	pinMode(ENB, OUTPUT);
}

void set_speed(int speed_1, int speed_2)
{
	analogWrite(ENA, speed_1);
	analogWrite(ENB, speed_2);
}

void di_chuyen_thang()
{
	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, LOW);
	digitalWrite(IN3, HIGH);
	digitalWrite(IN4, LOW);
}

void di_chuyen_lui()
{
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, HIGH);
	digitalWrite(IN3, LOW);
	digitalWrite(IN4, HIGH);
}

void rotate_right()
{
	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, LOW);
	digitalWrite(IN3, LOW);
	digitalWrite(IN4, HIGH);
}

void rotate_left()
{
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, HIGH);
	digitalWrite(IN3, HIGH);
	digitalWrite(IN4, LOW);
}

void stop()
{
	analogWrite(ENA, 0);
	analogWrite(ENB, 0);
}

int control_robot(char data_control)
{
	if (data_control == 't')
	{
		set_speed(255,255);
		di_chuyen_thang();
		direct = 1;
		return 1;
	}

	else if (data_control == 'h')
	{
		set_speed(255,255);
		rotate_left();
		direct = 2;
		return 2;
		/* code */
	}

	else if (data_control == 'f')
	{
		set_speed(255,255);
		rotate_right();
		direct = 3;
		return 3;
		/* code */
	}
	else if (data_control == 'g')
	{
		set_speed(255,255);

		di_chuyen_lui();
		direct = 4;
		return 4;
		/* code */
	}
	else if (data_control == 'o')
	{
		stop();
		direct = 5;
		return 5;
	}
	return 0;
}