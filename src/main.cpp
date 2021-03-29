#include <Arduino.h>
#include "WiFi.h"
#include "PubSubClient.h"               // pio lib install "knolleary/PubSubClient"
#include "esp_now.h"                    // voor de verbinding met de robot (gaat niet over de broker)

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
int RSSI = 0;                           // geeft de afstand tot de stralingslocatie weer

const int ledCount = 8;
int ledPins[] = {13,12,14,27,26,25,33,32};

int RSSI_1m = 26;                        // rssi waarde op 1m
double factor = 4;                       // tussen 2 en 4 (op het moment zelf bepalen in het lokaal!)

double RSSI_to_distance(int RSSI) {       // bepaal de afstand tot de stralingslocatie ahv de RSSI-waarde
  double distance = pow(10,(RSSI-RSSI_1m)/(10*factor));

  Serial.print("distance :");
  Serial.println(distance);

  return (distance);
}

int distance_to_leds(double distance) {   // bepaal het aantal LEDs dat moet branden ahv de afstand tot de stralingslocatie

  int distance2 = (int)(distance * 100);
  Serial.print("distance groter :");
  Serial.println(distance2);

  int distance3 = constrain(distance2, 50, 600);
  Serial.print("distance beperkt :");
  Serial.println(distance3);  

  return (7 - map(distance3, 50, 600, 0, 8));
}
void leds(int leds) {                    // laat gegeven aantal LEDs branden

  Serial.print("ledlevel :");
  Serial.println(leds);

  for (int thisLed = 0; thisLed < ledCount; thisLed++) {
    if (thisLed < leds) {
      digitalWrite(ledPins[thisLed], HIGH);
    }
    else {
      digitalWrite(ledPins[thisLed], LOW);
    }
  }
}

/*
VERBINDEN MET DE ROBOT VIA ESP-NOW
*/

// structure to receive data - must match the sender structure!
typedef struct struct_message {
    int i;
} struct_message;

// create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  int RSSI = myData.i;
  Serial.println(RSSI);

  // bepaal het aantal LEDs dat moet branden ahv de RSSI-waarde
  leds(distance_to_leds(RSSI_to_distance(RSSI)));
}

/* 
VERBINDEN MET DE BROKER VIA WI-FI
*/

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
      leds(0); // alle LEDs uit;
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
}

void reconnect() {
  while (!client.connected()) { // loop until we're reconnected
    Serial.print("Attempting MQTT connection...");
    // attempt to connect
    if (client.connect("ESP_LEDbar")) {
      Serial.println("connected");
      client.subscribe("espMorse");
      client.subscribe("espOntsmet");
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

  setup_wifi();                         // stel wi-fi verbinding in (voor verbinding met de broker)
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);  


  WiFi.mode(WIFI_STA);                  // set device as a wi-fi station

  if (esp_now_init() != ESP_OK) {       // init ESP-NOW
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // once ESP-NOW is successfully init, we will register for recv CB to get recv packer info
  esp_now_register_recv_cb(OnDataRecv);


  for (int thisLed = 0; thisLed < ledCount; thisLed++) {
    pinMode(ledPins[thisLed], OUTPUT);
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  client.loop();
}