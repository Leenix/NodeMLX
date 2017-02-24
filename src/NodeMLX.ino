#include <Arduino.h>
#include "i2c_t3.h"
#include "MLX90621.h"
#include "ThermalTracker.h"
#include "SimpleTimer.h"
#include "Logging.h"
#include "ArduinoJson.h"
#include "ESP8266WiFi.h"

extern "C" {
#include "user_interface.h"
}

////////////////////////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////////////////////////
// Versioning
const char* DEVICE_NAME = "NodeMLX";
const char* VERSION = "1.0";

// Pins
const byte MLX_POWER_PIN = D1;

// Serial
const long SERIAL_BAUD = 115200;
const int LOGGER_LEVEL = LOG_LEVEL_INFOS;
const char PACKET_START = '#';
const char PACKET_END = '$';

// Thermal flow
const long THERMAL_FRAME_INTERVAL = 1000 / REFRESH_RATE;
const long THERMAL_CHECK_MOVEMENT_INTERVAL = 500;
const long BACKGROUND_CHECK_INTERVAL = 200;
const long PRINT_FRAME_INTERVAL = 1000;
const long THERMAL_PRINT_AMBIENT_INTERVAL = 5000;
// Thermal flow tracker
const int TRACKER_NUM_BACKGROUND_FRAMES = 100;
const int TRACKER_MINIMUM_DISTANCE = 150;
const int TRACKER_MINIMUM_BLOB_SIZE = 5;

////////////////////////////////////////////////////////////////////////////////
// Function Prototypes
////////////////////////////////////////////////////////////////////////////////

void start_thermal_flow();
void process_new_frame();
void print_new_movements();
void check_background();
void print_frame();


////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

SimpleTimer timer;
MLX90621 thermal_flow;
ThermalTracker tracker(TRACKER_NUM_BACKGROUND_FRAMES, TRACKER_MINIMUM_DISTANCE, TRACKER_MINIMUM_BLOB_SIZE);

// Thermal
long movements[NUM_DIRECTION_CATEGORIES];
float frame[NUM_ROWS][NUM_COLS];
bool background_building = true;

////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////

void setup() {
    /**
    * Main setup
    * Start all the sensors and do all the things
    */

    pinMode(MLX_POWER_PIN, OUTPUT);
    digitalWrite(MLX_POWER_PIN, HIGH);

    Log.Init(LOGGER_LEVEL, SERIAL_BAUD);
    Wire.begin(D2, D3);
    Log.Info("NodeMLX Starting...");

    disable_wifi();
    start_thermal_flow();
}

void loop() {
    /**
    * Main loop
    * Everything is called off timer events
    */
    timer.run();
    wdt_reset();
}

////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////
// Thermal tracking

void start_thermal_flow() {
    /**
    * Start the thermal flow sensor
    */

    thermal_flow.initialise(REFRESH_RATE * 2);

    timer.setInterval(THERMAL_FRAME_INTERVAL, process_new_frame);
    timer.setInterval(THERMAL_CHECK_MOVEMENT_INTERVAL, print_new_movements);
    timer.setTimeout(BACKGROUND_CHECK_INTERVAL, check_background);
    timer.setInterval(THERMAL_PRINT_AMBIENT_INTERVAL, print_ambient_temperature);

    Log.Info("Thermal flow started.");
}

void process_new_frame() {
    /**
    * Grab the next frame in from the MLX90621 sensor and pass it to the tracking
    * algorithm
    */

    long start_time = millis();
    thermal_flow.get_temperatures(frame);
    tracker.process_frame(frame);
    long process_time = millis() - start_time;

    Log.Debug("Blobs in frame: %d\tprocess time %l ms", tracker.get_num_last_blobs(), process_time);
}

void print_new_movements() {
    /**
    * Print any new movements that have been detected by the tracking algorithm
    */
    if (tracker.has_new_movements()) {
        tracker.get_movements(movements);
        int num_blobs = tracker.get_num_last_blobs();

        Log.Info(
            "%c{\"id\":\"thermal\",\"left\":%l,\"right\":%l,\"zero\":%l,"
            "\"blobs\":%d}%c",
            PACKET_START, movements[LEFT], movements[RIGHT], movements[NO_DIRECTION], num_blobs, PACKET_END);
    }
}

void check_background() {
    /**
    * Check if the tracking algorithm has built enough background frames.
    * This function is for debug and notification and doesn't affect the runtime
    * of the sensor.
    */

    if (tracker.finished_building_background()) {
        Log.Info(
            "Thermal background finished building.;\tThermal flow detection "
            "started.");
    } else {
        timer.setTimeout(BACKGROUND_CHECK_INTERVAL, check_background);
    }
}

void print_frame() {
    /**
    * Print the temperatures observed by the thermopile array.
    * Temperatures are only printed if the logger priority is debug or lower.
    * Temperatures are in °C.
    */
    if (Log.getLevel() == LOG_LEVEL_VERBOSE) {
        for (int i = 0; i < NUM_ROWS; i++) {
            Serial.print('[');
            for (int j = 0; j < NUM_COLS; j++) {
                Serial.print('\t');
                Serial.print(frame[i][j], 2);
            }
            Serial.println(']');
        }
        Serial.println();
    }
}

void print_ambient_temperature(){
    char temperature[8];

    float ambient = thermal_flow.get_ambient_temperature();
    dtostrf(ambient, 0, 2, temperature);
    Log.Info("%c{\"id\":\"ambient\", \"temp\":%s}%c", PACKET_START, temperature, PACKET_END);
}

////////////////////////////////////////////////////////////////////////////////
// WiFi

void disable_wifi() {
    /**
    * Turn off the wifi module to save power
    * The module must be turned back on again to use
    */
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();
    delay(1);
    Log.Info("Wifi disabled");
}
