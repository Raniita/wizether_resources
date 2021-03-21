/*
   WizeTher Gateway v1 ---- Configuration file
*/

#pragma once

#define DEBUG_SERIAL Serial

// ----------------------------
// WiFi credentials
// ----------------------------
#define WIFI_SSID      ""
#define WIFI_PASSWORD  ""

// ----------------------------
// Wize Radio connection
// ----------------------------

#define RESET_PIN      14
#define RX_PIN         5
#define TX_PIN         4

// ----------------------------
// Wize configuration
// ----------------------------

#define WIZE_CHANNEL    CHANNEL_01
#define WIZE_POWER      POWER_20dBm
#define WIZE_DATARATE   DATARATE_2400bps

// ----------------------------
// MQTT configuration
// ----------------------------

#define MQTT_HOST       "84.124.211.16"       // DNS: ranii.pro
#define MQTT_PORT       1883
//#define MQTT_USER       ""
//#define MQTT_PASS       ""
#define MQTT_QOS        2
#define MQTT_RETAIN     0

// ----------------------------
// Forwarder
// ----------------------------

#define PING_INTERVAL   30000
