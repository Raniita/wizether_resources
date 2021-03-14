/*
    WizeTher v1 - Apadrina una estacion meterologica
    Mas información en: https://wizether.wize.ranii.pro/

    Sensor code

    Use with Allwize K2 board!

*/

#include "AllWize.h"
#include "CayenneLPP.h"

// Sensors
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Air_Quality_Sensor.h"

// Display OLED
#include <U8g2lib.h>
#include <Wire.h>

// ----------------------------
// Board configuration
// ----------------------------

#if not defined(ARDUINO_ALLWIZE_K2)
#error "This sktech is meant to roun on an AllWize K2 board"
#endif

#define MODULE_SERIAL           SerialWize
#define DEBUG_SERIAL            SerialUSB
#define RESET_PIN               PIN_WIZE_RESET

// ----------------------------
// Wize Configuration
// ----------------------------

#define WIZE_CHANNEL            CHANNEL_01
#define WIZE_POWER              POWER_20dBm
#define WIZE_DATARATE           DATARATE_2400bps
#define WIZE_MID                0x06FA
#define WIZE_UID                0x20212230
#define WIZE_APP_ID             0xFE
#define WIZE_NETWORK_ID         0x01

// ----------------------------
// Sensors configuration
// ----------------------------
#define PIN_LOUDNESS A1
#define PIN_AIRQUALITY A4
#define PIN_TOUCH 0

// ----------------------------
// Declarations
// ----------------------------

// Radio
AllWize allwize(&MODULE_SERIAL, RESET_PIN);
CayenneLPP payload(32);             // Revisar que significa el 32

// Sensors
Adafruit_BME280 sensor;
AirQualitySensor air_sensor(PIN_AIRQUALITY);

// Display
U8G2_SSD1306_128X64_ALT0_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// ----------------------------
// Sensors Declarations
// ----------------------------
float last_temp;
int last_hum;
float last_press;
int last_loud;
int last_air;

int act_page;

long debouncing_time = 15; //Debouncing Time in Milliseconds
volatile unsigned long last_micros;

void sensorSetup() {
  // BM280
  if (!sensor.begin(0x76)) {
    DEBUG_SERIAL.println("[SENSOR] Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  DEBUG_SERIAL.println("[SENSOR] Sensor BM280 ready!");

  // Air Quality
  if (air_sensor.init()) {
    DEBUG_SERIAL.println("[SENSOR] Air quality sensor ready!");
  } else {
    DEBUG_SERIAL.println("[SENSOR] Air quality sensor ERROR!");
  }

  // Display
  if (!u8g2.begin()) {
    DEBUG_SERIAL.println("[DISPLAY] Unable to start OLED display :S");
  }
  u8g2.enableUTF8Print();
  act_page = 0;       // Default: temperature as display page

  // Touch Sensor
  pinMode(PIN_TOUCH, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_TOUCH), debounceInterrupt, RISING);
}

// ----------------------------
// Sensors
// ----------------------------

// Returns the temperature in C (with 1 decimal)
float getTemperature() {
  float t = sensor.readTemperature();
  DEBUG_SERIAL.print("[SENSOR] Temperature (C): ");
  DEBUG_SERIAL.println(t);
  return t;
}

// Returns humidity in %
int getHumidity() {
  unsigned char h = sensor.readHumidity();
  DEBUG_SERIAL.print("[SENSOR] Humidity (%): ");
  DEBUG_SERIAL.println(h);
  return h;
}

// Returns pressure in hPa
float getPressure() {
  float p = sensor.readPressure() / 100.0;
  DEBUG_SERIAL.print("[SENSOR] Pressure (hPa): ");
  DEBUG_SERIAL.println(p);
  return p;
}

// Loudness Sensor
int getLoudness() {
  int p = analogRead(PIN_LOUDNESS);
  DEBUG_SERIAL.print("[SENSOR] Loudness: ");
  DEBUG_SERIAL.println(p);
  return p;
}

// Air Quality Sensor
int getAirQuality() {
  int quality = air_sensor.slope();
  int value = air_sensor.getValue();
  DEBUG_SERIAL.print("[SENSOR] Air quality value: ");
  DEBUG_SERIAL.print(value);

  if (quality == AirQualitySensor::FORCE_SIGNAL) {
    DEBUG_SERIAL.println(" High pollution!");
  } else if (quality == AirQualitySensor::HIGH_POLLUTION) {
    DEBUG_SERIAL.println(" High pollution!");
  } else if (quality == AirQualitySensor::LOW_POLLUTION) {
    DEBUG_SERIAL.println(" Low pollution!");
  } else if (quality == AirQualitySensor::FRESH_AIR) {
    DEBUG_SERIAL.println(" Fresh air!");
  }

  return value;
}

bool readTouch(){
  return digitalRead(PIN_TOUCH);
}

void debounceInterrupt(){
  if((long)(micros() - last_micros) >= debouncing_time * 1000) {
    interruptTouch();
    last_micros = micros();
  }
}

void interruptTouch(){
  DEBUG_SERIAL.println("[SENSOR] Touch detected!");
  DEBUG_SERIAL.print("[DISPLAY] Changing to page ");
  
  switch(act_page) {
    case 0:
      act_page = 1;
      // Change display to 1
      displayPressure(last_press);
      DEBUG_SERIAL.println(act_page);
      break;
    
    case 1:
      act_page = 2;
      // Change display to 2
      displayHumidity(last_hum);
      DEBUG_SERIAL.println(act_page);
      break;
    
    case 2:
      act_page = 3;
      // Change display to 3
      displayLoudness(last_loud);
      DEBUG_SERIAL.println(act_page);
      break;
    
    case 3:
      act_page = 4;
      // Change display to 4
      displayAirQuality(last_air);
      DEBUG_SERIAL.println(act_page);
      break;

    case 4:
      act_page = 0;
      // Change display to 0
      displayTemperature(last_temp);
      DEBUG_SERIAL.println(act_page);
      break;
  }
}

// ----------------------------
// Display functions
// ----------------------------
void displayHelloWorld() {
  int degree = 30;
  
  u8g2.clearBuffer();
  
  u8g2.setFont(u8g2_font_t0_11b_mr);
  u8g2.drawStr(0, 10, "[WizeTher]");
  u8g2.drawStr(115, 10, "v1");
  
  u8g2.setFont(u8g2_font_courB10_tr);
  u8g2.drawUTF8(0, 25, "Temp:");
  u8g2.setCursor(50, 25);
  u8g2.print(degree);
  u8g2.print("°C ");

  //u8g2.setFont(u8g2_font_ncenB08_tr);
  //u8g2.drawStr(90, 64, "Next...");

  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawUTF8(115, 64, "→");

  //u8g2.setFont(u8g2_font_unifont_t_symbols);
  //u8g2.drawUTF8(5, 20, "Snowman: ☃");
  //u8g2.setFont(u8g2_font_bitcasual_tr);
  //u8g2.print("°C");
  
  //u8g2.setFont(u8g2_font_courB10_tr);
  //u8g2.drawStr(15, 40, "Temp: 30°C");

  u8g2.sendBuffer();

  delay(100);
}

void displayBase(){
  u8g2.setFont(u8g2_font_t0_11b_mr);
  u8g2.drawStr(0, 10, "[WizeTher]");
  u8g2.drawStr(115, 10, "v1");

  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawUTF8(115, 64, "→");
}

void displayTemperature(float t){

  u8g2.clearBuffer();

  displayBase();

  u8g2.setFont(u8g2_font_courB10_tr);
  u8g2.drawUTF8(0, 25, "Temp:");
  u8g2.setCursor(50, 25);
  u8g2.print(t);
  u8g2.print("°C ");

  u8g2.sendBuffer();
  delay(200);
  
}

void displayPressure(int p){
  u8g2.clearBuffer();

  displayBase();

  u8g2.setFont(u8g2_font_courB10_tr);
  u8g2.drawUTF8(0, 25, "Press:");
  u8g2.setCursor(50, 25);
  u8g2.print(p);
  u8g2.print("hPa");

  u8g2.sendBuffer();
  delay(200);
}

void displayHumidity(float h){
  u8g2.clearBuffer();

  displayBase();

  u8g2.setFont(u8g2_font_courB10_tr);
  u8g2.drawUTF8(0, 25, "Hum:");
  u8g2.setCursor(50, 25);
  u8g2.print(h);
  u8g2.print("%");

  u8g2.sendBuffer();
  delay(200);
}

void displayLoudness(int l){
  u8g2.clearBuffer();

  displayBase();

  u8g2.setFont(u8g2_font_courB10_tr);
  u8g2.drawUTF8(0, 25, "Loud:");
  u8g2.setCursor(50, 25);
  u8g2.print(l);
  //u8g2.print("%");

  u8g2.sendBuffer();
  delay(200);
}

void displayAirQuality(int a){
  u8g2.clearBuffer();

  displayBase();

  u8g2.setFont(u8g2_font_courB10_tr);
  u8g2.drawUTF8(0, 25, "Air:");
  u8g2.setCursor(50, 25);
  u8g2.print(a);
  //u8g2.print("%");

  u8g2.sendBuffer();
  delay(200);
}

// ----------------------------
// AllWize Setup
// ----------------------------
void wizeSetup() {
  DEBUG_SERIAL.println("[WIZE] Initializing radio module");

  // Init AllWize object
  allwize.begin();
  if (!allwize.waitForReady()) {
    DEBUG_SERIAL.println("[WIZE] Error conecting to the module, check wiring!");
    while (true);
  }

  allwize.slave();
  allwize.setChannel(WIZE_CHANNEL, true);
  allwize.setPower(WIZE_POWER);
  allwize.setDataRate(WIZE_DATARATE);
  allwize.setMID(WIZE_MID);
  allwize.setUID(WIZE_UID);
  allwize.setWizeApplication(WIZE_APP_ID);
  allwize.setWizeNetworkId(WIZE_NETWORK_ID);

  DEBUG_SERIAL.println("[WIZE] Ready...");
}

// ----------------------------
// AllWize Send payload
// ----------------------------
void wizeSend(uint8_t * payload, size_t len) {
  char buffer[64];
  DEBUG_SERIAL.print("[WIZE] Sending: ");
  for (uint8_t i = 0; i < len; i++) {
    snprintf(buffer, sizeof(buffer), "%02X", payload[i]);
    DEBUG_SERIAL.print(buffer);
  }
  DEBUG_SERIAL.print("\n");

  if (!allwize.send(payload, len)) {
    DEBUG_SERIAL.println("[WIZE] Error sending message");
  }
}

// ----------------------------
// Setup
// ----------------------------
void setup() {
  // Init serial DEBUG_SERIAL
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL && millis() < 5000);
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("[WIZETHER] Sensor V1");

  // Init radio
  wizeSetup();

  // Init sensors
  sensorSetup();

}

// ----------------------------
// Loop
// ----------------------------
void loop() {
  
  // Build payload
  payload.reset();
  last_temp = getTemperature();
  payload.addTemperature(2, last_temp);
  last_hum = getHumidity();
  payload.addRelativeHumidity(3, last_hum);
  last_press = getPressure();
  payload.addBarometricPressure(4, last_press);
  last_loud = getLoudness();
  payload.addAnalogInput(5, last_loud);         // Si lo mandamos como mismo add,
  last_air = getAirQuality();
  payload.addDigitalInput(6, last_air);         // no lo detecta el GW

  // Update Display
  switch(act_page){
    case 0:
      displayTemperature(last_temp);
      break;
    case 1:
      displayPressure(last_press);
      break;
    case 2:
      displayHumidity(last_hum);
      break;
    case 3:
      displayLoudness(last_loud);
      break;
    case 4:
      displayAirQuality(last_air);
      break;
  }

  // Send the string as payload
  //noInterrupts();
  wizeSend(payload.getBuffer(), payload.getSize());
  //interrupts();

  // Delay responses for 20 seconds [may be sleep arduino]
  delay(20000);
}
