#include <Adafruit_ADXL345_U.h>
#include <Adafruit_Sensor.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <NTPClient.h>
#include <NetworkUdp.h>
#include <PubSubClient.h>
#include <SimpleKalmanFilter.h>
#include <WiFi.h>
#include <Wire.h>

// --- Config start ---

const char* topic = "accel/1";

const char* ssid = "happyswing";
const char* password = "****";

const char* mqtt_server = "192.168.1.252";
const int mqtt_port = 1883;
const char* mqtt_username = "";
const char* mqtt_password = "";

const unsigned long send_interval_ms = 100;

const unsigned int measurement_buffer_size = 128;

const char* test_root_ca =
    "-----BEGIN CERTIFICATE-----\n" \ 
    "MIIDUTCCAjmgAwIBAgIJAPPYCjTmxdt/MA0GCSqGSIb3DQEBCwUAMD8xCzAJBgNV\n" \ 
    "BAYTAkNOMREwDwYDVQQIDAhoYW5nemhvdTEMMAoGA1UECgwDRU1RMQ8wDQYDVQQD\n" \ 
    "DAZSb290Q0EwHhcNMjAwNTA4MDgwNjUyWhcNMzAwNTA2MDgwNjUyWjA/MQswCQYD\n" \ 
    "VQQGEwJDTjERMA8GA1UECAwIaGFuZ3pob3UxDDAKBgNVBAoMA0VNUTEPMA0GA1UE\n" \ 
    "AwwGUm9vdENBMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAzcgVLex1\n" \ 
    "EZ9ON64EX8v+wcSjzOZpiEOsAOuSXOEN3wb8FKUxCdsGrsJYB7a5VM/Jot25Mod2\n" \ 
    "juS3OBMg6r85k2TWjdxUoUs+HiUB/pP/ARaaW6VntpAEokpij/przWMPgJnBF3Ur\n" \ 
    "MjtbLayH9hGmpQrI5c2vmHQ2reRZnSFbY+2b8SXZ+3lZZgz9+BaQYWdQWfaUWEHZ\n" \ 
    "uDaNiViVO0OT8DRjCuiDp3yYDj3iLWbTA/gDL6Tf5XuHuEwcOQUrd+h0hyIphO8D\n" \ 
    "tsrsHZ14j4AWYLk1CPA6pq1HIUvEl2rANx2lVUNv+nt64K/Mr3RnVQd9s8bK+TXQ\n" \ 
    "KGHd2Lv/PALYuwIDAQABo1AwTjAdBgNVHQ4EFgQUGBmW+iDzxctWAWxmhgdlE8Pj\n" \ 
    "EbQwHwYDVR0jBBgwFoAUGBmW+iDzxctWAWxmhgdlE8PjEbQwDAYDVR0TBAUwAwEB\n" \ 
    "/zANBgkqhkiG9w0BAQsFAAOCAQEAGbhRUjpIred4cFAFJ7bbYD9hKu/yzWPWkMRa\n" \ 
    "ErlCKHmuYsYk+5d16JQhJaFy6MGXfLgo3KV2itl0d+OWNH0U9ULXcglTxy6+njo5\n" \ 
    "CFqdUBPwN1jxhzo9yteDMKF4+AHIxbvCAJa17qcwUKR5MKNvv09C6pvQDJLzid7y\n" \ 
    "E2dkgSuggik3oa0427KvctFf8uhOV94RvEDyqvT5+pgNYZ2Yfga9pD/jjpoHEUlo\n" \ 
    "88IGU8/wJCx3Ds2yc8+oBg/ynxG8f/HmCC1ET6EHHoe2jlo8FpU/SgGtghS1YL30\n" \ 
    "IWxNsPrUP+XsZpBJy/mvOhE5QXo6Y35zDqqj8tI7AGmAWu22jg==\n" \ 
    "-----END CERTIFICATE-----\n";

// --- Config end ---

WiFiClient espClient;
PubSubClient client(espClient);

SimpleKalmanFilter kalmanAngle(10, 10, 1);

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

unsigned long lastUpdate = 0;

float measurements[measurement_buffer_size] = {0.0};

void setup() {
    Serial.begin(115200);
    // TODO: use CA cert and don't run in insecure mode
    // espClient.setCACert(test_root_ca);
    // espClient.setInsecure();

    Serial.println("HI");

    setupWiFi();
    setupOTA();
    setupMQTT();
    setupAccel();
}

void loop() {
    ArduinoOTA.handle();
    sensors_event_t event;
    accel.getEvent(&event);
    float acceleration = event.acceleration.y;

    float roll = asin(max(min(-acceleration / 9.81, 1.0), -1.0)) * 180.0 / PI;

    Serial.print(-acceleration);
    Serial.print(",");
    Serial.println(roll);

    float angle = kalmanAngle.updateEstimate(roll);

    reconnectWiFiIfLost();

    if (client.connected() && ((millis() - lastUpdate) > send_interval_ms)) {
        char payload[250];

        float rms32 = calculateRMS(32);
        float rms64 = calculateRMS(64);
        float rms128 = calculateRMS(128);
        addNewMeasurement(angle);

        char* payload_mask = "{\"ts\": 0, \"angle\": %.2f, \"rms\": %.2f, \"rms32\": %.2f, \"rms64\": %.2f, \"rms128\": %.2f}";
        snprintf(payload, sizeof(payload), payload_mask, time, angle, rms32, rms32, rms64, rms128);
        client.publish(topic, payload);
        Serial.println(payload);
        lastUpdate = millis();
    }

    client.loop();
    delay(10);
}

void addNewMeasurement(float value) {
    memcpy(measurements, &measurements[1], sizeof(measurements) - sizeof(float));
    measurements[measurement_buffer_size - 1] = value;
}

float calculateRMS(int count) {
    float rms = 0;
    for (int i = 0; i < count; i++) {
        float measurement = measurements[measurement_buffer_size - i - 1];
        rms += measurement * measurement;
    }
    return sqrt(rms / float(count));
}

void callback(char* topic, byte* payload, unsigned int length) {}

void reconnectWiFiIfLost() {
    // TODO: also reconnect MQTT
    if ((WiFi.status() != WL_CONNECTED)) {
        Serial.println("Reconnecting to WiFi...");
        WiFi.disconnect();
        WiFi.reconnect();
    }
    if (!client.connected()) {
        setupMQTT();
    }
}

void setupWiFi() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
}

void setupMQTT() {
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    while (!client.connected()) {
        Serial.println("Connecting to MQTT...");
        if (client.connect("ArduinoClient", mqtt_username, mqtt_password)) {
            Serial.println("Connected to MQTT");
        } else {
            Serial.print("MQTT Connection failed, rc=");
            Serial.print(client.state());
            Serial.println(" Retrying...");
            delay(5000);
        }
    }
}

void setupAccel() {
    if (!accel.begin()) {
        Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
        while (1);
    }
}

void setupOTA() {
    ArduinoOTA
        .onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH) {
                type = "sketch";
            } else {  // U_SPIFFS
                type = "filesystem";
            }

            // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
            Serial.println("Start updating " + type);
        })
        .onEnd([]() {
            Serial.println("\nEnd");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR) {
                Serial.println("Auth Failed");
            } else if (error == OTA_BEGIN_ERROR) {
                Serial.println("Begin Failed");
            } else if (error == OTA_CONNECT_ERROR) {
                Serial.println("Connect Failed");
            } else if (error == OTA_RECEIVE_ERROR) {
                Serial.println("Receive Failed");
            } else if (error == OTA_END_ERROR) {
                Serial.println("End Failed");
            }
        });

    ArduinoOTA.begin();
}
