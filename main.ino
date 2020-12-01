
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <NTPClient.h>

#define Relay0            D0 //Bed room light
#define Relay1            D1 //Bed room fans
#define Relay2            D2 //kitchen light
#define Relay3            D3 //kitchen window
#define Relay4            D4 //Veranda light
#define Relay5            D5 //bed room window
#define Relay6            D6 //back door
#define Relay7            D7 //main door
#define Relay8            D8 //Veranda windows
#define Relay9            D9 //sicurity system

#define WLAN_SSID       "Shan"             // SSID
#define WLAN_PASS       "098765432"        // password

//get time pre setup section

const long utcOffsetInSeconds = 3600;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

bool led;

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com" //Adafruit Server
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "sachinthanode"            // Username
#define AIO_KEY         "aio_cCnm35Higyg9xoMExCZiZLfIArCM"   // Auth Key

//WIFI CLIENT
WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Feeds names
Adafruit_MQTT_Subscribe Light0 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME"/feeds/bed_light");
Adafruit_MQTT_Subscribe Light1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Relay2");
Adafruit_MQTT_Subscribe Light2 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/kit_light");
Adafruit_MQTT_Subscribe Light3 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Relay4");
Adafruit_MQTT_Subscribe Light4 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/ven_light");
Adafruit_MQTT_Subscribe Light5 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Relay4");
Adafruit_MQTT_Subscribe Light6 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Relay4");
Adafruit_MQTT_Subscribe Light7 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Relay4");
Adafruit_MQTT_Subscribe Light8 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Relay4");
Adafruit_MQTT_Subscribe Light9 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/s_sys");
Adafruit_MQTT_Subscribe Night = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/night");
Adafruit_MQTT_Subscribe Day = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/day");
Adafruit_MQTT_Subscribe Out = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/out");
Adafruit_MQTT_Subscribe reset1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/light11");

void MQTT_connect();

void setup() {
  Serial.begin(115200);

  pinMode(Relay0, OUTPUT);
  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT);
  pinMode(Relay3, INPUT);
  pinMode(Relay4, OUTPUT);
  pinMode(Relay5, INPUT);
  pinMode(Relay6, INPUT);
  pinMode(Relay7, INPUT);
  pinMode(Relay8, INPUT);
  pinMode(Relay9, OUTPUT);

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  timeClient.begin();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  mqtt.subscribe(&Light0);
  mqtt.subscribe(&Light1);
  mqtt.subscribe(&Light2);
  mqtt.subscribe(&Light3);
  mqtt.subscribe(&Light4);
  mqtt.subscribe(&Light5);
  mqtt.subscribe(&Light6);
  mqtt.subscribe(&Light7);
  mqtt.subscribe(&Light8);
  mqtt.subscribe(&Light9);
  mqtt.subscribe(&Night);
  mqtt.subscribe(&Day);
  mqtt.subscribe(&Out);
  mqtt.subscribe(&reset1);

}

void loop() {


  MQTT_connect();
  Serial.print(digitalRead(Relay3));
  Serial.print(digitalRead(Relay5));
  Serial.print(digitalRead(Relay6));
  Serial.print(digitalRead(Relay7));
  Serial.print(digitalRead(Relay8));
  Serial.print("\n");

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(20000))) {
    if (subscription == &Night) {
      Serial.print(F("Got(night): "));
      Serial.println((char *)Night.lastread);
if(timeClient.getHours()<18)
      night_Mode();
    }
    if (subscription == &Day) {
      Serial.print(F("Got (day): "));
      Serial.println((char *)Day.lastread);
      //int Light2_State = atoi((char *)Day.lastread);
      // digitalWrite(Relay1, Light2_State);
      day_Mode();
    }
    if (subscription == &Out) {
      Serial.print(F("Got(out): "));
      Serial.println((char *)Out.lastread);
      out_Mode();
    }
    if (subscription == &reset1) {
      Serial.print(F("Got(rest): "));
      Serial.println((char *)reset1.lastread);
      reset_Mode();
    }

    if (subscription == &Light0) {
      Serial.print(F("Got: "));
      Serial.println((char *)Light0.lastread);
      int Light4_State = atoi((char *)Light0.lastread);
      digitalWrite(Relay0, Light4_State);

    }
    if (subscription == &Light2) {
      Serial.print(F("Got: "));
      Serial.println((char *)Light2.lastread);
      int Light4_State = atoi((char *)Light2.lastread);
      digitalWrite(Relay2, Light4_State);

    }
    if (subscription == &Light4) {
      Serial.print(F("Got: "));
      Serial.println((char *)Light4.lastread);
      int Light4_State = atoi((char *)Light4.lastread);
      digitalWrite(Relay4, Light4_State);

    }
    if (subscription == &Light9) {
      Serial.print(F("Got: "));
      Serial.println((char *)Light9.lastread);
      int Light4_State = atoi((char *)Light9.lastread);
      digitalWrite(Relay9, Light4_State);

    }
  }


}

void MQTT_connect() {
  int8_t ret;

  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;

  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);
    retries--;
    if (retries == 0) {
      while (1);
    }
  }
  Serial.println("MQTT Connected!");

}
void night_Mode() {
  if (digitalRead(Relay8) == 0 || digitalRead(Relay6) == 0 || digitalRead(Relay5) == 0 || digitalRead(Relay7) == 0 || digitalRead(Relay3) == 0) {
    digitalWrite(Relay1, HIGH);
  }
  else if (digitalRead(Relay8) == 1 || digitalRead(Relay6) == 1 || digitalRead(Relay5) == 1 || digitalRead(Relay7) == 1 || digitalRead(Relay3) == 1) {
    digitalWrite(Relay0, LOW);
    digitalWrite(Relay1, LOW);
    digitalWrite(Relay2, LOW);
    digitalWrite(Relay4, LOW);
    //digitalWrite(Relay5, LOW);
    digitalWrite(Relay9, HIGH);
  }
}
void day_Mode() {
  digitalWrite(Relay0, LOW);
  //  digitalWrite(Relay1, LOW);
  digitalWrite(Relay2, LOW);
  digitalWrite(Relay4, LOW);
  // digitalWrite(Relay5, HIGH);
  digitalWrite(Relay9, LOW);
}
void out_Mode() {
  if (digitalRead(Relay8) == 0 || digitalRead(Relay6) == 0 || digitalRead(Relay5) == 0 || digitalRead(Relay7) == 0 || digitalRead(Relay3) == 0) {
    digitalWrite(Relay1, HIGH);
  }
  else if (digitalRead(Relay8) == 1 || digitalRead(Relay6) == 1 || digitalRead(Relay5) == 1 || digitalRead(Relay7) == 1 || digitalRead(Relay3) == 1) {
    digitalWrite(Relay0, LOW);

    digitalWrite(Relay1, LOW);
    digitalWrite(Relay2, LOW);
    digitalWrite(Relay4, LOW);
    // digitalWrite(Relay5, LOW);
    digitalWrite(Relay9, HIGH);
  }
}
void reset_Mode() {
  digitalWrite(Relay0, LOW);
  // digitalWrite(Relay1, LOW);
  digitalWrite(Relay2, LOW);
  digitalWrite(Relay4, LOW);
  //digitalWrite(Relay5, LOW);
  digitalWrite(Relay9, LOW);
}
