#include <WiFi.h>
#include <Arduino.h>
// #include "control_dc.h"
#include "control_key.h"
// Replace with your own network credentials
const char *ssid = "101";
const char *password = "p1016282";
const uint16_t port = 8090;
const char *host = "192.168.2.107";
bool control = false;

void setup()
{

  Serial.begin(115200);
  pinMode(33, OUTPUT);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("\nConnecting to WiFi Network ..");

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(100);
  }
  digitalWrite(33, HIGH);
  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
  // init_dc();
  init_control();
  // set_speed(255,255);
}

void loop()
{
  // Do Nothing
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
      Serial.println(line);

      if (line.length() < 15 && line.length() > 8)
      {
        control = false;
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
      else if (line.length() <= 7 and line.length()>3)
      {
        control = true;
        char data_control = line[5];
        // Serial.println(data_control);
        if (control_keyboard(data_control) == 0)
        {
          Serial.println(line);
          // client.print("end");
        }
      }
    }
  }
  client.stop();

}
