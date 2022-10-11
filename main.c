#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Time.h>

//edit this for new node
#define humidity_topic "sensor/<node_name>/humidity"
#define temperature_topic "sensor/<node_name>/temperature"
#define pressure_topic "sensor/<node_name>/pressure"
#define pir_topic "sensor/<node_name>/pir"
#define ldr_topic "sensor/<node_name>/lumen"

//edit this for new node
const char* node = "<device_name>";
const char* ssid = "<wifi_name>";
const char* password = "<wifi_password>";

IPAddress ip(192, 168, x, x); //edit this for new node
IPAddress gateway(192, 168, 8, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 8, 1);

const char* mqtt_server = "<mqtt_address>";
const char* mqtt_user = "<mqtt_user>";
const char* mqtt_password = "<mqtt_pw>";

const float temperaturecalibration = 0;

bool pir_enabled = false;

int sensorPin = A0;    // input for LDR
int enable1 = 15;      // enable reading LDR
int light_ldr = 0;     // variable to store the value coming from sensor LDR
int pirPin = 1;        // input pin for PIR sensor
int pirState = LOW;    // no motion detected
int pirVal = 0;        // variable for reading the pin status
int pirStatus = 0;     // variable for the pin status

float temp;
float pressure;
float altitude;
float humidity;

String motionStatus;   // motion status text

WiFiClientSecure espClient;
PubSubClient client(espClient);
Adafruit_BME280 bme;

void bme280() {
    temp = bme.readTemperature() + temperaturecalibration;
    pressure = bme.readPressure() / 100.0F;
    humidity = bme.readHumidity();
    if (isnan(temp) || isnan(pressure) || isnan(humidity)) {
        Serial.println("Failed to read from BME280 sensor!");
        return;
    }
    client.publish(temperature_topic, String(temp).c_str(), true);
    client.publish(pressure_topic, String(pressure).c_str(), true);
    client.publish(humidity_topic, String(humidity).c_str(), true);
    yield();
}

void ldr() {
    digitalWrite(enable1, HIGH);
    light_ldr = analogRead(sensorPin);
    light_ldr = constrain(light_ldr, 300, 850);
    light_ldr = map(light_ldr, 300, 850, 0, 1023);
    digitalWrite(enable1, LOW);
    client.publish(ldr_topic, String(light_ldr).c_str(), true);
    yield();
}

void pir() {
    pirVal = digitalRead(pirPin);
    if (pirVal == LOW && pirStatus != 1) {
      motionStatus = "standby";
      client.publish(pir_topic, String(motionStatus).c_str(), true);
      pirStatus = 1;
    }

    else if (pirVal == HIGH && pirStatus != 2) {
      motionStatus = "motion detected";
      client.publish(pir_topic, String(motionStatus).c_str(), true);
      pirStatus = 2;
    }
    yield();
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect(node, mqtt_user, mqtt_password)) {
    } else {
      delay(5000);
    }
  }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.hostname(node);
    WiFi.config(ip, dns, gateway, subnet);
    if (WiFi.SSID() != ssid) {
        WiFi.begin(ssid, password);
        WiFi.persistent(true);
        WiFi.setAutoConnect(true);
        WiFi.setAutoReconnect(true);
    }
    Serial.println("");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("Booting this thing up...");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    espClient.setInsecure();
    client.setServer(mqtt_server, 8883);
    pinMode(pirPin, INPUT);
    Wire.begin();
    if(!bme.begin(0x76)) {
        Serial.print("Failed to read from BME sensor!");
        while(1);
    }
}

void loop() {
    const unsigned long time = 1 * 60 * 1000UL; //1 minute
    static unsigned long lasttime = 0 - time;
    unsigned long now = millis();

    if (!client.connected()) {
        reconnect();
    }

    client.loop();
    yield();

    if (now - lasttime >= time){
        lasttime += time;
        bme280();
        ldr();
        if (pir_enabled == true) {
            pir();
        }
    }
    delay(1000);
}
