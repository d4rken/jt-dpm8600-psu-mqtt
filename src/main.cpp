#include <../Config.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Ticker.h>
#include <Wire.h>

#include "DPM8600.h"

Ticker ticker;

const char *SSID = WIFI_SSID;
const char *PSK = WIFI_PW;
const char *MQTT_BROKER = MQTT_SERVER;
const char *MQTT_BROKER_IP = MQTT_SERVER_IP;
const char *MQTT_BROKER_PORT = MQTT_SERVER_PORT;

WiFiClient espClient;
PubSubClient client(espClient);

SoftwareSerial mySerial(D2, D1);
DPM8600 converter(1);

char buffer[256];

const long SYSTEM_STAT_INTERVAL = 30 * 1000;

bool hasInitSettings = false;
float desiredMaxCurrent = 0;
float desiredMaxVoltage = 0;
bool desiredPowerState = false;
float lastCommandUpdate = 0;
unsigned long lastReceivedUpdate = 0;
unsigned long lastMqttSetup = 0;

void mqttCallback(char *_topic, byte *payload, unsigned int length) {
    String topic = String(_topic);
    char data_str[length + 1];
    os_memcpy(data_str, payload, length);
    data_str[length] = '\0';

    Serial.println("Received topic '" + topic + "' with data '" + (String)data_str + "'");

    if (topic == "pv1/dcdc/command/power") {
        desiredPowerState = atoi(data_str);
    } else if (topic == "pv1/dcdc/command/max-current") {
        desiredMaxCurrent = atof(data_str);
    } else if (topic == "pv1/dcdc/command/max-voltage") {
        desiredMaxVoltage = atof(data_str);
    } else if (topic == "pv1/dcdc/uptime") {
        lastReceivedUpdate = atol(data_str);
    }
    lastCommandUpdate = millis();
    client.loop();
}

void tick() {
    int state = digitalRead(LED_BUILTIN);
    digitalWrite(LED_BUILTIN, !state);
}

void setupMqttSubs() {
    Serial.println("MQTT Setup Subs: Setting callback");
  
    client.setCallback(mqttCallback);
    delay(200);
    Serial.println("MQTT Setup Subs: Re-Subscribing");
    client.subscribe("pv1/dcdc/uptime");  
    client.subscribe("pv1/dcdc/command/power");
    client.subscribe("pv1/dcdc/command/max-voltage");
    client.subscribe("pv1/dcdc/command/max-current");

    lastMqttSetup = millis();
    Serial.println("MQTT Setup Subs: Done");     
}

void setup() {
    Serial.begin(74880);
    Serial.println("Setup...");

    pinMode(LED_BUILTIN, OUTPUT);
    ticker.attach(0.6, tick);

    Serial.println("Setting up WIFI" + String(SSID));
    WiFi.setAutoReconnect(true);
    WiFi.hostname("PV1-DCDC-ESP8266");
    WiFi.begin(SSID, PSK);

    client.setServer(MQTT_BROKER, (uint16_t)strtol(MQTT_BROKER_PORT, NULL, 10));

    bool success = converter.begin(&mySerial);
    Serial.println("Connected to DCDC: " + String(success));
}

char itoaBuf[64];
char dtostrfBuf[64];

void updateSystemStats() {
    long rssi = WiFi.RSSI();
    Serial.println("RSSI: " + String(rssi));
    client.publish("pv1/dcdc/rssi", itoa(rssi, itoaBuf, 10));

    unsigned long uptime = millis();
    Serial.println("Uptime: " + String(uptime));
    client.publish("pv1/dcdc/uptime", ultoa(uptime, itoaBuf, 10));
}

void processCommands() {
    Serial.println("### START: processCommands ###");

    int rawPowerState = converter.read('p');
    if (rawPowerState != -12) {
        bool currentPowerState = rawPowerState == 1;
        if (!hasInitSettings) desiredPowerState = currentPowerState;
        if (desiredPowerState != currentPowerState) {
            Serial.println("Setting power state to " + String(desiredPowerState));

            bool success = converter.write('p', desiredPowerState);
            if (success) {
                Serial.println("Toggle power success");
            } else {
                Serial.println("Toggle power failed");
            }
            delay(600);
        } else {
            Serial.println("Desired power state already set to " + String(desiredPowerState));
        }
    }

    float currentMaxCurrent = converter.read('l');
    if (currentMaxCurrent != -15) {
        Serial.println("Current max current set to " + String(currentMaxCurrent));
        if (desiredMaxCurrent > 15) desiredMaxCurrent = 15;
        if (!hasInitSettings) desiredMaxCurrent = currentMaxCurrent;

        if (desiredMaxCurrent != currentMaxCurrent) {
            bool success = converter.write('c', desiredMaxCurrent);
            if (success) {
                Serial.println("New max current set to " + String(desiredMaxCurrent));
            } else {
                Serial.println("Failed to set new max current");
            }
            delay(600);
        } else {
            Serial.println("Desired max current already set to " + String(desiredMaxCurrent));
        }
    }

    float currentMaxVoltage = converter.read('v');
    if (currentMaxVoltage != -10) {
        Serial.println("Current max voltage set to " + String(currentMaxVoltage));
        if (desiredMaxVoltage > 48) desiredMaxVoltage = 48;

        if (!hasInitSettings) desiredMaxVoltage = currentMaxVoltage;

        if (desiredMaxVoltage != currentMaxVoltage) {
            bool success = converter.write('v', desiredMaxVoltage);
            if (success) {
                Serial.println("New max voltage set to " + String(desiredMaxVoltage));
            } else {
                Serial.println("Failed to set new max voltage");
            }
            delay(600);
        } else {
            Serial.println("Desired max voltage already set to " + String(desiredMaxVoltage));
        }
    }

    float currentMode = converter.read('s');
    if (currentMode != -13) {
        Serial.println("Current mode set to " + String(currentMode));
        if (currentMode != 0) {
            bool success = converter.write('s', 0);
            if (success) {
                Serial.println("New mode set to 0");
            } else {
                Serial.println("Failed to set new mode to 0");
            }
            delay(600);
        } else {
            Serial.println("Desired mode already set to 0");
        }
    }

    hasInitSettings = true;

    Serial.println("### END: processCommands ###");
}

void updateStats() {
    Serial.println("### START: updateStats ###");

    float voltage = converter.read('v');
    Serial.println("Voltage: " + String(voltage));
    if (voltage != -10) {
        client.publish("pv1/dcdc/voltage", dtostrf(voltage, 1, 2, dtostrfBuf));
    }

    float current = converter.read('c');
    Serial.println("Current: " + String(current));
    if (current != -11) {
        client.publish("pv1/dcdc/current", dtostrf(current, 1, 3, dtostrfBuf));
    }

    float maxCurrent = converter.read('l');
    Serial.println("Max current: " + String(maxCurrent));
    if (maxCurrent != -15) {
        client.publish("pv1/dcdc/limit", dtostrf(maxCurrent, 1, 3, dtostrfBuf));
    }

    float power = converter.read('p');
    Serial.println("Power: " + String(power));
    if (power != -12) {
        client.publish("pv1/dcdc/enabled", itoa(power, itoaBuf, 10));
    }

    float temp = converter.read('t');
    Serial.println("Temp: " + String(temp));
    if (temp != -16) {
        client.publish("pv1/dcdc/temp", dtostrf(temp, 2, 2, dtostrfBuf));
    }

    Serial.println("### END: updateStats ###");
}

long lastStats = 0;
long lastCommands = 0;
long lastCommandCheck = 0;

void loop() {
    bool wifiConnected = true;
    bool wasWifiConnected = WiFi.status() == WL_CONNECTED;
    int retryWifi = 0;
    while (WiFi.status() != WL_CONNECTED) {
        Serial.println("Connecting to WiFi...");
        if (retryWifi > 10) {
            wifiConnected = false;
            break;
        } else {
            retryWifi++;
        }
        delay(500);
    }

    if (!wasWifiConnected && wifiConnected) {
        Serial.println("Reconnected, my IP is: " + WiFi.localIP().toString());
    } else if (!wifiConnected) {
        Serial.println("Failed to connect to WiFi");
    }

    bool mqttConnected = wifiConnected;
    bool wasMqttConnected = client.connected();
    int retryMqtt = 0;
    while (wifiConnected && !client.connected()) {
        Serial.println("Connecting to MQTT broker...");
        client.connect("PV1-DCDC-ESP8266");
        if (retryMqtt > 3) {
            mqttConnected = false;
            break;
        } else {
            retryMqtt++;
        }
        delay(500);
    }

    if (!mqttConnected) {
        Serial.println("Failed to connect to MQTT broker");
    } else if (!wasMqttConnected && mqttConnected) {
        Serial.println("Reconnected to MQTT broker");
        setupMqttSubs();
    }

    // If we are still connected, but no longer receiving callbacks, resubscribe.
    unsigned long uptime = millis();
    if(mqttConnected && uptime - lastReceivedUpdate > 10 * SYSTEM_STAT_INTERVAL && uptime - lastMqttSetup > 2 * SYSTEM_STAT_INTERVAL ) {
        Serial.println("Callback no longer triggers: " + String(uptime));
        client.publish("pv1/dcdc/error/callback_timeout", ultoa(uptime, itoaBuf, 10), true);
        setupMqttSubs();
    }

    ticker.detach();

    client.loop();

    if (lastCommandUpdate != lastCommandCheck && (millis() - lastCommands) > 1000) {
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("Checking for new commands...");

        processCommands();
        updateStats();

        lastCommandCheck = lastCommandUpdate;
        lastCommands = millis();

        digitalWrite(LED_BUILTIN, HIGH);
    }

    client.loop();

    if ((millis() - lastStats) > SYSTEM_STAT_INTERVAL) {
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("Updating statistics...");
        updateSystemStats();
        updateStats();

        lastStats = millis();

        digitalWrite(LED_BUILTIN, HIGH);
    }

    client.loop();
}
