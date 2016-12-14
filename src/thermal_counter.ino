#include <Arduino.h>
#include "i2c_t3.h"
#include "MLX90621.h"
#include "ThermalTracker.h"
#include "SimpleTimer.h"
#include "Logging.h"

////////////////////////////////////////////////////////////////////////////////
// Constants

SimpleTimer timer;
MLX90621 sensor;
ThermalTracker tracker;

const long SERIAL_BAUD = 115200;
const int LOGGER_LEVEL = LOG_LEVEL_DEBUG;
const char PACKET_START = '#';
const char PACKET_END = '$';


const long FRAME_INTERVAL = 1000/REFRESH_RATE;
const long CHECK_MOVEMENTS_INTERVAL = 100;

////////////////////////////////////////////////////////////////////////////////
// Variables

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
}

void loop(){
    timer.run();
}

////////////////////////////////////////////////////////////////////////////////
// Functions

void process_new_frame(){

    long start_time = millis();
    sensor.get_temperatures(frame);
    tracker.process_frame(frame);
    long process_time = millis() - start_time;

    Log.Debug("Blobs in frame: %d\tprocess time %l ms", tracker.get_num_last_blobs(), process_time);
}

void print_new_movements(){
    if (tracker.has_new_movements()) {
        tracker.get_movements(movements);
        Log.Info("%c{\"id\":\"thermal\",\"left\":%l,\"right\":%l,\"zero\":%l}%c", PACKET_START, movements[LEFT], movements[RIGHT], movements[NO_DIRECTION], PACKET_END);
    }
}

////////////////////////////////////////////////////////////////////////////////
