#include "ESP8266WiFi.h"

const char* ssid = "Infidel attachments";
const char* password = "seekwhatyoucrave";
int rssiArr[10];

void setup()
{
  Serial.begin(115200);
  Serial.printf("Connecting to %s ", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println(" connected");
}



void loop() {
  for(int i=0;i<10;i++){
    rssiArr[i] = rssiArr[(i+1)%10];
  }
  rssiArr[9] = WiFi.RSSI();
  float rssiAve = 0;
  for(int i=0;i<10;i++){
      rssiAve = rssiAve+rssiArr[i];
  }

  rssiAve = rssiAve/10;
  
  Serial.println(rssiAve);
  delay(50);
  }
