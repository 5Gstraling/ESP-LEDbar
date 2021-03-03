/*
PINNEN
*/

#include <Arduino.h>
#include "WiFi.h"
#include "PubSubClient.h"               // pio lib install "knolleary/PubSubClient"

#define SSID          "NETGEAR68"
#define PWD           "excitedtuba713"

#define MQTT_SERVER   "192.168.1.2"
#define MQTT_PORT     1883

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

bool vijfG = false;                     // in het aan onze puzzel? aan indien morse klaar, uit indien 5G puzzel voltooid
bool aan = true;                        // ledbar aan? (moet uit bij ontsmetten of indien auto opgenomen)        
int RSSI = 0;                           // de afstand tot de stralingslocatie

int ledA = 13;
int ledB = 12;
int ledC = 14;
int ledD = 27;
int ledE = 26;
int ledF = 25;

void allemaalAan() {
  digitalWrite(ledA, HIGH);
  digitalWrite(ledB, HIGH);
  digitalWrite(ledC, HIGH);
  digitalWrite(ledD, HIGH);
  digitalWrite(ledE, HIGH);
  digitalWrite(ledF, HIGH);
}

void allemaalUit() {
  digitalWrite(ledA, LOW);
  digitalWrite(ledB, LOW);
  digitalWrite(ledC, LOW);
  digitalWrite(ledD, LOW);
  digitalWrite(ledE, LOW);
  digitalWrite(ledF, LOW);
}

void updateLEDs(int RSSI) {
  if (RSSI < 30) {
    allemaalAan();
  }
  else if (RSSI < 40) {
    digitalWrite(ledA, HIGH);
    digitalWrite(ledB, HIGH);
    digitalWrite(ledC, HIGH);
    digitalWrite(ledD, HIGH);
    digitalWrite(ledE, HIGH);
    digitalWrite(ledF, LOW);
  }
  else if (RSSI < 50) {
    digitalWrite(ledA, HIGH);
    digitalWrite(ledB, HIGH);
    digitalWrite(ledC, HIGH);
    digitalWrite(ledD, HIGH);
    digitalWrite(ledE, LOW);
    digitalWrite(ledF, LOW);
  }
  else if (RSSI < 60) {
    digitalWrite(ledA, HIGH);
    digitalWrite(ledB, HIGH);
    digitalWrite(ledC, HIGH);
    digitalWrite(ledD, LOW);
    digitalWrite(ledE, LOW);
    digitalWrite(ledF, LOW);
  }
  else if (RSSI < 70) {
    digitalWrite(ledA, HIGH);
    digitalWrite(ledB, HIGH);
    digitalWrite(ledC, LOW);
    digitalWrite(ledD, LOW);
    digitalWrite(ledE, LOW);
    digitalWrite(ledF, LOW);
  }
  else if (RSSI < 80) {
    digitalWrite(ledA, HIGH);
    digitalWrite(ledB, LOW);
    digitalWrite(ledC, LOW);
    digitalWrite(ledD, LOW);
    digitalWrite(ledE, LOW);
    digitalWrite(ledF, LOW);
  }
  else if (RSSI == 200) {
    Serial.println("auto wordt opgenomen");
    allemaalUit();
  }

  else {
    allemaalUit();
  }
}

void callback(char *topic, byte *message, unsigned int length);

void setup_wifi() {
  delay(10);
  Serial.println("Connecting to WiFi..");

  WiFi.begin(SSID, PWD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char *topic, byte *message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // verbinding met puzzel ontsmetting -- bij "0" moet er ontsmet worden, bij "1" kunnen de RSSIwaardes worden afgelezen

  if (String(topic) == "espOntsmet") {
    if (messageTemp == "0") {
      Serial.println("ontsmetten");
      aan = false;
      allemaalUit();
    }
    else if (messageTemp == "1") {
      aan = true;
    }
  }

  // verbinding met puzzel morsecode -- er wordt een string "1" verzonden indien de morsecode puzzel voltooid is

  if (String(topic) == "espMorse") {
    if (messageTemp == "1") {
      Serial.println("morsecode voltooid");
      vijfG = true;
    }
  }

  // verbinding met onze robot

  if (String(topic) == "rssiwaarde") {
    RSSI = messageTemp.toInt();
    if (aan && vijfG) {
      updateLEDs(RSSI); // update leds aan de hand van de RSSI waarde
    }
  }
}

void reconnect() {
  while (!client.connected()) { // loop until we're reconnected
    Serial.print("Attempting MQTT connection...");
    // attempt to connect
    if (client.connect("ESP_LEDbar")) {
      Serial.println("connected");
      client.subscribe("espMorse");
      client.subscribe("espOntsmet");
      client.subscribe("rssiwaarde");
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      // wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);                 // stel de seriële monitor in

  pinMode(ledA, OUTPUT); 
  pinMode(ledB, OUTPUT); 
  pinMode(ledC, OUTPUT); 
  pinMode(ledD, OUTPUT); 
  pinMode(ledE, OUTPUT);
  pinMode(ledF, OUTPUT);

  setup_wifi();                         // stel wi-fi verbinding in
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);  
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  client.loop();
}