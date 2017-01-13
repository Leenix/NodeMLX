#include <Arduino.h>
#include "i2c_t3.h"
#include "MLX90621.h"
#include "ThermalTracker.h"
#include "SimpleTimer.h"
#include "Logging.h"

////////////////////////////////////////////////////////////////////////////////
// Constants

const long SERIAL_BAUD = 115200;
const int LOGGER_LEVEL = LOG_LEVEL_VERBOSE;
const char PACKET_START = '#';
const char PACKET_END = '$';

const long FRAME_INTERVAL = 1000/REFRESH_RATE;
const long CHECK_MOVEMENTS_INTERVAL = 100;
const long BACKGROUND_CHECK_INTERVAL = 100;
const long PRINT_FRAME_INTERVAL = 1000;

const int TRACKER_NUM_BACKGROUND_FRAMES = 100;
const int TRACKER_MINIMUM_DISTANCE = 150;
const int TRACKER_MINIMUM_BLOB_SIZE = 5;

////////////////////////////////////////////////////////////////////////////////
// Variables

SimpleTimer timer;
MLX90621 sensor;
ThermalTracker tracker(TRACKER_NUM_BACKGROUND_FRAMES, TRACKER_MINIMUM_DISTANCE, TRACKER_MINIMUM_BLOB_SIZE);

long movements[NUM_DIRECTION_CATEGORIES];
float frame[NUM_ROWS][NUM_COLS];
bool background_building = true;

////////////////////////////////////////////////////////////////////////////////
// Main

void setup(){
    Log.Init(LOGGER_LEVEL, SERIAL_BAUD);
    Wire.begin(D2, D3);
    pinMode(D1, OUTPUT);
    digitalWrite(D1, HIGH);
    Log.Info("Thermal tracker started");

    sensor.initialise(REFRESH_RATE*2);
    Log.Debug("MLX90621 sensor started");

    timer.setInterval(FRAME_INTERVAL, process_new_frame);
    timer.setInterval(CHECK_MOVEMENTS_INTERVAL, print_new_movements);
    timer.setTimeout(BACKGROUND_CHECK_INTERVAL, check_background);
    timer.setInterval(PRINT_FRAME_INTERVAL, print_frame);
}

void loop(){
    timer.run();
}

////////////////////////////////////////////////////////////////////////////////
// Functions

void process_new_frame(){
    /**
    * Grab the next frame in from the MLX90621 sensor and pass it to the tracking algorithm
    */

    long start_time = millis();
    sensor.get_temperatures(frame);
    tracker.process_frame(frame);
    long process_time = millis() - start_time;

    Log.Debug("Blobs in frame: %d\tprocess time %l ms", tracker.get_num_last_blobs(), process_time);
}

void print_new_movements(){
    /**
    * Print any new movements that have been detected by the tracking algorithm
    */
    if (tracker.has_new_movements()) {
        tracker.get_movements(movements);
        int num_blobs = tracker.get_num_last_blobs();

        Log.Info("%c{\"id\":\"thermal\",\"left\":%l,\"right\":%l,\"zero\":%l,\"blobs\":%d}%c",
        PACKET_START, movements[LEFT], movements[RIGHT], movements[NO_DIRECTION], num_blobs, PACKET_END);
    }
}

void check_background(){
    /**
    * Check if the tracking algorithm has built enough background frames.
    * This function is for debug and notification and doesn't affect the runtime of the sensor.
    */

    if (tracker.finished_building_background()) {
        Log.Info("Background finished building;\tDetection started...");
    }
    else{
        timer.setTimeout(BACKGROUND_CHECK_INTERVAL, check_background);
    }
}

void print_frame(){
    if (Log.getLevel() == LOG_LEVEL_VERBOSE){
        for (int i = 0; i < NUM_ROWS; i++) {
            Serial.print('[');
            for (int j = 0; j < NUM_COLS; j++) {
                Serial.print('\t');
                Serial.print(frame[i][j], 2);
            }
            Serial.println(']');
        }
        Serial.print('\n\n');
    }
}
////////////////////////////////////////////////////////////////////////////////
