#include <WiFi.h>
#include <Arduino.h>
#include "lib/control_arm.h"
#include "lib/control_motor.h"
int dutyCycle = 255;

#define led 2

// const char *ssid = "101";
// const char *password = "p1016282";

const char *ssid = "DO VAN LUC";
const char *password = "0867660302";
const uint16_t port = 8090;
// const char *host = "192.168.2.107";
const char *host = "192.168.1.6";



void setup() {

  init_motor();
  init_servo();
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("\nConnecting to WiFi Network ..");

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(100);
  }
  digitalWrite(led, HIGH);
  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
  

  // testing
  Serial.print("Testing DC Motor...");
}


void brain_robot()
{

  WiFiClient client;
  if (!client.connect(host, port))
  {
    return;
  }
  // client.print("sta");
  while (client.connected())
  {
    if (client.available())
    {
      String line = client.readStringUntil('\n');

      if (line.length() < 15 && line.length() > 10)
      {
        Serial.println(line);

        String angle;
        int angles_end[4];
        int angles_start[4];

        angles_start[0] = servoPins[0].initialPosition;
        angles_start[1] = servoPins[1].initialPosition;
        angles_start[2] = servoPins[2].initialPosition;
        angles_start[3] = servoPins[3].initialPosition;
        for (int i = 0; i < line.length(); i++)
        {
          angle += line[i];
          if ((i + 1) % 3 == 0)
          {
            angles_end[i / 3] = angle.toInt();
            // servoPins[i / 3].servo.write(angles_end[i / 3]);
            angle = "";
          }
        }
        move_point(angles_start, angles_end);
      }


      else if (line.length() < 11 and line.length() > 3)
      {
        String speed;
        int speed_end[8];
        char direct = line[9];
        for (int i = 0; i < line.length(); i++)
        {
          speed += line[i];
          if ((i + 1) % 3 == 0)
          {
            speed_end[i / 3] = speed.toInt();
            // Serial.println(speed.toInt());
            speed = "";
          }
        }
        Serial.println(direct);
        control_robot(direct);
        set_speed(speed_end[0], speed_end[1]);//18, 30
        delay(speed_end[2]);
        set_speed(0, 0);//18, 30

        delay(10);

      }
    }
  }
  client.stop();
}

void loop() {
  // control_robot('t');
  // set_speed(255, 255);
  // delay(100);
  // control_robot('o');
  // set_speed(0, 0);
  // delay(10);
  brain_robot();
  // for (int i = 0; i<180 ;i++){
    // servoPins[0].servo.write(0);
  //   delay(100);
  // }
  // servoPins[1].servo.write(100);
  // servoPins[2].servo.write(58);
  // servoPins[3].servo.write(5);

}