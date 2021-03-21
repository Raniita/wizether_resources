/*
    WizeTher v1 - Apadrina una estacion meterologica
    Mas información en: https://wizether.wize.ranii.pro/

    wize2serial code for "gateway" of the kit

    Use with WeMos R1 D2 Mini [no new version of 8266]
*/

#include "AllWize.h"
#include "SoftwareSerial.h"
#include <ESP8266WiFi.h>
#include <AsyncMqttClient.h>
#include <Ticker.h>
#include "configuration.h"

// Payload CayenneLPP
#include "CayenneLPP.h"
#include "ArduinoJson.h"

#if not defined(ARDUINO_ARCH_ESP8266)
#error "This example is meant to run on an ESP8266 board!"
#endif

// ----------------------------
// Declarations
// ----------------------------

// MQTT
AsyncMqttClient mqtt;
Ticker mqttTimer;

// WiFi
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiTimer;

// Wize radio
AllWize allwize(RX_PIN, TX_PIN, RESET_PIN);

// ----------------------------
// Utils
// ----------------------------

// Legacy function to parse binary data
String bin2hex(uint8_t * bin, uint8_t len) {
  char b[3];
  String output = String("");
  for (uint8_t i = 0; i < len; i++) {
    sprintf(b, "%02X", bin[i]);
    output += String(b);
  }
  return output;
}

// ----------------------------
// MQTT
// ----------------------------

void mqttConnect() {
  DEBUG_SERIAL.printf("[MQTT] Connecting to %s:%d...\n", MQTT_HOST, MQTT_PORT);
  mqtt.connect();
}

void mqttOnConnect(bool sessionPresent) {
  DEBUG_SERIAL.printf("[MQTT] Connected!\n");
  ping();
}

void mqttOnDisconnect(AsyncMqttClientDisconnectReason reason) {
  DEBUG_SERIAL.printf("[MQTT] Disconnected!\n");
  if (WiFi.isConnected()) {
    mqttTimer.detach();
    mqttTimer.once(2, mqttConnect);
  }
}

void mqttSend(const char * topic, const char * payload) {
  DEBUG_SERIAL.printf("[MQTT] Sending: %s => %s\n", topic, payload);
  mqtt.publish(topic, MQTT_QOS, MQTT_RETAIN, payload);
}

void mqttSetup() {
  mqtt.onConnect(mqttOnConnect);
  mqtt.onDisconnect(mqttOnDisconnect);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
#if defined(MQTT_USER) && defined(MQTT_PASS)
  if (strlen(MQTT_USER) > 0 && strlen(MQTT_PASS) > 0) {
    mqtt.setCredentials(MQTT_USER, MQTT_PASS);
  }
#endif
}

// ----------------------------
// AllWize
// ----------------------------

void wizeSetup() {
  // Init AllWize object
  allwize.begin();
  if (!allwize.waitForReady()) {
    DEBUG_SERIAL.printf("[WIZE] Error connecting to the module, check your wiring!\n");
    while (true) delay(1);
  }

  allwize.master();
  allwize.setChannel(WIZE_CHANNEL);
  allwize.setPower(WIZE_POWER);
  allwize.setDataRate(WIZE_DATARATE);

  DEBUG_SERIAL.printf("[WIZE] Module type: %s\n", allwize.getModuleTypeName().c_str());
  DEBUG_SERIAL.printf("[WIZE] MBUS mode: 0x%2X\n", allwize.getMode());
  DEBUG_SERIAL.printf("[WIZE] Channel: %d\n", allwize.getChannel());
  DEBUG_SERIAL.printf("[WIZE] Datarate: %d (%d bps)\n", allwize.getDataRate(), allwize.getDataRateSpeed(allwize.getDataRate()));
  DEBUG_SERIAL.printf("[WIZE] Listening...\n");
}

void wizeDebugMessage(allwize_message_t message) {

  // Code to pretty-print the message
  DEBUG_SERIAL.printf(
    "[WIZE] ADDR: 0x%s, RSSI: %d, DATA: 0x%s\n",
    bin2hex(message.address, 4).c_str(),
    (int16_t) message.rssi / -2,
    bin2hex(message.data, message.len).c_str()
  );
}

void ping() {

  DynamicJsonDocument root(512);

  JsonObject gateway = root.createNestedObject("gateway");
  gateway["mid"] = allwize.getMID();
  gateway["uid"] = allwize.getUID();
  //gateway["sn"] = allwize.getSerialNumber();

  char topic[32];
  snprintf(topic, sizeof(topic), "gateway/%s%s/ping", allwize.getMID().c_str(), allwize.getUID().c_str());
  String payload;
  serializeJson(root, payload);
  mqttSend(topic, payload.c_str());

}

void wizeLoop() {
  if (allwize.available()) {
    // Get message
    allwize_message_t message = allwize.read();

    // Show it to console
    wizeDebugMessage(message);

    // Parse and send via MQTT
    if (mqtt.connected()) {
      wizeMQTTParse(message);
    }
  }
}

void wizeMQTTParse(allwize_message_t message) {

  DynamicJsonDocument root(512);

  root["app"] = message.wize_application;
  root["net"] = message.wize_network_id;
  root["uid"] = bin2hex(message.address, 4);
  root["cpt"] = message.wize_counter;

  JsonObject metadata = root.createNestedObject("metadata");
  metadata["ch"] = allwize.getChannel();
  metadata["freq"] = allwize.getFrequency(allwize.getChannel());
  metadata["dr"] = allwize.getDataRateSpeed(allwize.getDataRate());
  metadata["toa"] = 8000.0 * message.len / allwize.getDataRateSpeed(allwize.getDataRate());

  JsonObject gateway = root.createNestedObject("gateway");
  gateway["mid"] = allwize.getMID();
  gateway["uid"] = allwize.getUID();
  //gateway["sn"] = allwize.getSerialNumber();
  gateway["rssi"] = message.rssi / -2;

  root["payload"] = bin2hex(message.data, message.len);

  char topic[32];

  JsonObject fields = root.createNestedObject("fields");

  // Parse payload
  CayenneLPP payload(1);
  DynamicJsonDocument jsonBuffer(512);
  JsonArray input = jsonBuffer.createNestedArray();
  payload.decode(message.data, message.len, input);

  // Walk JsonArray
  for (JsonObject element : input) {
    JsonVariant v = element["value"];
    if (v.is<JsonObject>()) {
      for (JsonPair kv : v.as<JsonObject>()) {
        fields[kv.key()] = kv.value().as<float>();
      }
    } else {
      String name = element["name"].as<char *>();
      if (name.equals("analog_in")) {
        fields["loudness"] = element["value"].as<float>();
      } else if (name.equals("digital_in")) {
        fields["air_quality"] = element["value"].as<float>();
      } else {
        fields[element["name"].as<char *>()] = element["value"].as<float>();
      }
    }
  }

  snprintf(topic, sizeof(topic), "gateway/%s%s/uplink", allwize.getMID().c_str(), allwize.getUID().c_str());
  String output;
  serializeJson(root, output);
  //DEBUG_SERIAL.println(output);
  mqttSend(topic, output.c_str());

}

// ----------------------------
// WiFi
// ----------------------------

void wifiConnect() {
  DEBUG_SERIAL.printf("[WIFI] Connecting to %s...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void wifiOnConnect(const WiFiEventStationModeGotIP& event) {
  DEBUG_SERIAL.printf("[WIFI] Connected!\n");
  mqttConnect();
}

void wifiOnDisconnect(const WiFiEventStationModeDisconnected& event) {
  DEBUG_SERIAL.printf("[WIFI] Disconnected!\n");
  wifiTimer.detach();
  wifiTimer.once(2, wifiConnect);
}

void wifiSetup() {
  wifiConnectHandler = WiFi.onStationModeGotIP(wifiOnConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(wifiOnDisconnect);
}


// ----------------------------
// Main
// ----------------------------

void setup() {

  // Setup serial DEBUG_SERIAL
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL && millis() < 5000);

  DEBUG_SERIAL.printf("\n[WIZETHER] Wize2Serial \n");

  mqttSetup();
  wifiSetup();
  wizeSetup();

  wifiConnect();
}

void loop() {

  // Listen to messages and print on console
  wizeLoop();

  // PING
#if PING_INTERVAL
  static unsigned long last_ping = 0;
  if (millis() - last_ping > PING_INTERVAL) {
    last_ping = millis();
    ping();
  }
#endif

  delay(1);
}
