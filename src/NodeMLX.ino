#include <Arduino.h>
#include "i2c_t3.h"
#include "MLX90621.h"
#include "ThermalTracker.h"
#include "SimpleTimer.h"
#include "Logging.h"
#include "SPI.h"
#include "SD.h"
#include "RTClib.h"
#include "ArduinoJson.h"
#include "ESP8266WiFi.h"
#include <ESP8266HTTPClient.h>

////////////////////////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////////////////////////
// Versioning
const char* DEVICE_NAME = "NodeMLX";
const char* VERSION = "1.1";

// Pins
const byte BUTTON_PIN = D0;
const byte LED_PIN = D1;
const byte PIR_PIN = D4;
const byte CHIP_SELECT_PIN = D8;
const byte LIGHT_SENS_PIN = A0;

// Serial
const long SERIAL_BAUD = 115200;
const int LOGGER_LEVEL = LOG_LEVEL_VERBOSE;
const char PACKET_START = '#';
const char PACKET_END = '$';

// Thermal flow
const long THERMAL_FRAME_INTERVAL = 1000 / REFRESH_RATE;
const long THERMAL_CHECK_MOVEMENT_INTERVAL = 500;
const long BACKGROUND_CHECK_INTERVAL = 200;
const long PRINT_FRAME_INTERVAL = 1000;

// Thermal flow tracker
const int TRACKER_NUM_BACKGROUND_FRAMES = 100;
const int TRACKER_MINIMUM_DISTANCE = 150;
const int TRACKER_MINIMUM_BLOB_SIZE = 5;

// Motion
const long MOTION_INITIALISATION_TIME = 10000;
const int MOTION_INITIALISATION_INTERVALS = 10;
const long MOTION_COOLDOWN_DEFAULT = 5000;
const long MOTION_CHECK_INTERVAL = 500;

// Light Sensor (LDR)
const long LIGHT_CHECK_INTERVAL = 2000;
const int LIGHT_ON_THRESHOLD = 500;

// WiFi
const char* WIFI_SSID = "Handy";
const char* WIFI_PASSWORD = "things11";
const int WIFI_DEFAULT_TIMEOUT = 2000;
const long WIFI_RECONNECT_INTERVAL = 10000;

// HTTP
const char* SERVER_ADDRESS = "www.dweet.io";
const int SERVER_PORT = 80;
const char GET_REQUEST[] = "GET /dweet/for/";
const char GET_TAIL[] = " HTTP/1.1\r\n\r\n";
const long TIMEOUT = 5000;  // TCP timeout in ms

// RTC
const long RTC_CHECK_DATE_INTERVAL = 60000;

// SD Card
const long SD_WRITE_DATA_INTERVAL = 5 * 60 * 1000;

// Button
const long BUTTON_CHECK_INTERVAL = 200;
const long BUTTON_UPLOAD_DATA_DELAY = 3000;
const long BUTTON_RESET_COUNTERS_DELAY = 6000;

// LED
const long LED_ON_PERIOD = 50;
const long LED_OFF_PERIOD = 50;

////////////////////////////////////////////////////////////////////////////////
// Data Structures
////////////////////////////////////////////////////////////////////////////////
struct PirSensor {
    bool state;
    long start_time;
    long count;
    long cooldown;
    bool is_in_cooldown;
    byte pin;
};

struct Button {
    bool state;
    long down_time;
    byte pin;
};

////////////////////////////////////////////////////////////////////////////////
// Function Prototypes
////////////////////////////////////////////////////////////////////////////////

void start_thermal_flow();
void process_new_frame();
void print_new_movements();
void check_background();
void print_frame();

PirSensor init_motion_sensor(int pin, long cooldown);
void start_pir();
void warm_up_pir_sensors();
void update_pir();

void start_wifi();
bool attempt_wifi_connection(long timeout = WIFI_DEFAULT_TIMEOUT);
void upload_data();
void assemble_data_packet(char* packet_buffer);
void add_to_array(char* buffer, char* insert);

void start_light_sensor();
void check_light_sensor();

void start_sd_card();
void append_data_to_file();

void start_rtc();
void get_datetime(char* buffer);
void get_date(char* buffer);
void check_for_date_change();
void reset_counters();

void start_button();
void check_button();

void start_indicator();
void flash();
void flash(int num_flashes);
void flash_check();
void indicator_off();
void indicator_on();

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

SimpleTimer timer;
MLX90621 thermal_flow;
ThermalTracker tracker(TRACKER_NUM_BACKGROUND_FRAMES, TRACKER_MINIMUM_DISTANCE, TRACKER_MINIMUM_BLOB_SIZE);
PirSensor motion;
Button button;
File data_file;

// RTC
RTC_DS3231 rtc;
int rtc_day = 0;

// Thermal
long movements[NUM_DIRECTION_CATEGORIES];
float frame[NUM_ROWS][NUM_COLS];
bool background_building = true;

// Light
bool light_state;

// LED
int led_flash_count;
int max_led_flash_count;

////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////

void setup() {
    /**
    * Main setup
    * Start all the sensors and do all the things
    */
    Log.Init(LOGGER_LEVEL, SERIAL_BAUD);
    Wire.begin(SDA, SCL);
    Log.Info("NodeMLX Starting...");

    start_thermal_flow();
    start_pir();
    start_light_sensor();
    start_rtc();
    start_sd_card();
    start_wifi();
    start_button();
}

void loop() {
    /**
    * Main loop
    * Everything is called off timer events
    */
    timer.run();
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

    Log.Debug("Thermal flow started.");
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

        Log.Debug(
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

////////////////////////////////////////////////////////////////////////////////
// PIR

PirSensor init_motion_sensor(int pin, long cooldown) {
    /**
    * Initialise a new PIR motion sensor
    */
    PirSensor sensor;
    sensor.cooldown = cooldown;
    sensor.is_in_cooldown = false;
    sensor.pin = pin;
    sensor.state = false;
    sensor.start_time = 0;
    sensor.count = 0;

    pinMode(sensor.pin, INPUT);

    return sensor;
}

void start_pir() {
    /**
    * Start the PIR motion sensor for motion detections
    */

    motion = init_motion_sensor(PIR_PIN, MOTION_COOLDOWN_DEFAULT);

    // Stop everything until the PIR sensor has had a chance to settle
    warm_up_pir_sensors();

    // Start up regular reads
    timer.setInterval(MOTION_CHECK_INTERVAL, update_pir);
    Log.Info(("Motion detection started."));

    update_pir();
}

void warm_up_pir_sensors() {
    /**
    * Halt the system until the PIR sensors have had a chance to start.
    * The downtime is goverened by the MOTION_INITIALISATION_TIME variable
    * Default: 10 seconds
    */
    Log.Info(("Calibrating motion sensor - wait %d ms"), MOTION_INITIALISATION_TIME);

    for (long i = 0; i < MOTION_INITIALISATION_TIME;
         i += (MOTION_INITIALISATION_TIME / MOTION_INITIALISATION_INTERVALS)) {
        Log.Debug(("Motion sensor calibration: %d ms remaining..."), (MOTION_INITIALISATION_TIME - i));
        Serial.flush();
        delay(MOTION_INITIALISATION_TIME / MOTION_INITIALISATION_INTERVALS);
    }
}

void update_pir() {
    /**
    * Check the PIR sensor for detected movement
    * The sensor will output HIGH when motion is detected.
    * Detections will hold the detection status HIGH until the cool-down has
    * lapsed (default: 2s)
    */

    // Check cooldown status
    if (motion.is_in_cooldown) {
        if (millis() - motion.start_time > motion.cooldown) {
            motion.is_in_cooldown = false;
        }
    }

    // Check for motion if not in cooldown
    if (!motion.is_in_cooldown) {
        if (digitalRead(motion.pin)) {
            if (!motion.state) {
                // If motion detected, increment the count (and do all the extra bits)
                motion.state = true;
                motion.start_time = millis();
                motion.is_in_cooldown = true;
                motion.count++;

                Log.Debug("Motion sensor triggered. Count: %l", motion.count);
            }

            else {
                if (motion.state) {
                    motion.state = false;
                }
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// WiFi

void start_wifi() {
    /**
    * Set up the WiFi connection and events.
    * If setup fails, the connection will be retried after a delay (default: 30 seconds) until it is connected.
    * If everything succeeds, regular uploads will be established
    */

    bool connected = attempt_wifi_connection();

    if (connected) {
        Log.Info("WiFi Connected!\nIP address: %s", WiFi.localIP().toString().c_str());
        timer.setInterval(5000, upload_data);

    } else {
        timer.setTimeout(WIFI_RECONNECT_INTERVAL, start_wifi);
        upload_data();
    }
}

bool attempt_wifi_connection(long timeout) {
    /**
    * Attempt to connect to the preconfigured WiFi network
    * @param timeout Max amount of time allowed to establish a connection in ms
    * @return True if WiFi connected successfully
    */
    long start_time = millis();
    Log.Info("Connecting to %s", WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Wait for connection before continuing
    while (WiFi.status() != WL_CONNECTED && (millis() - start_time) < timeout) {
        delay(200);
    }

    return (WiFi.status() == WL_CONNECTED);
}

void upload_data() {
    /**
    * Upload the gathered data to the preconfigured web server
    * In this case, the web server is dweet so the data can be displayed on freeboard.io
    * All data is uploaded using a GET request to avoid bullshit HTTP flags and junk.
    */

    WiFiClient client;
    char packet_buffer[400];

    // Attempt connection to data server
    if (client.connect(SERVER_ADDRESS, SERVER_PORT)) {
        // Send data if connection is available
        assemble_data_packet(packet_buffer);
        client.print(packet_buffer);
        flash(1);

        timer.run();
        while (client.available() > 0) {
            Serial.print(client.readString());
            timer.run();
            delay(1);
        }
    }

    else {
        Log.Error("Connection to [%s] failed", SERVER_ADDRESS);
    }

    client.stopAll();
}

void assemble_data_packet(char* packet_buffer) {
    /**
    * Put all of the data into string format for uploading.
    * Entries are in the format of '&key=value'
    *
    * @param packet_buffer Buffer to store the payload string
    */
    char entry[50];

    sprintf(packet_buffer, "%s%s?", GET_REQUEST, DEVICE_NAME);

    sprintf(entry, "&therm_left=%l", movements[LEFT]);
    add_to_array(packet_buffer, entry);

    sprintf(entry, "&therm_right=%l", movements[RIGHT]);
    add_to_array(packet_buffer, entry);

    sprintf(entry, "&therm_up=%l", movements[UP]);
    add_to_array(packet_buffer, entry);

    sprintf(entry, "&therm_right=%l", movements[DOWN]);
    add_to_array(packet_buffer, entry);

    sprintf(entry, "&therm_nodir=%l", movements[NO_DIRECTION]);
    add_to_array(packet_buffer, entry);

    sprintf(entry, "&lights_on=%T", light_state);
    add_to_array(packet_buffer, entry);

    sprintf(entry, "&pir_count=%l", motion.count);
    add_to_array(packet_buffer, entry);

    add_to_array(packet_buffer, const_cast<char*>(GET_TAIL));
}

void add_to_array(char* buffer, char* insert) {
    /**
    * Append two strings together
    * The buffer string must be longer than the insert string.
    * If not, the insert string will be clipped.
    *
    * @param buffer Buffer to append string to
    * @param insert String to append to buffer
    */
    int index = strlen(buffer);

    for (int i = 0; i < strlen(insert); i++) {
        buffer[index + i] = insert[i];
    }

    buffer[index + strlen(insert)] = '\0';
}

////////////////////////////////////////////////////////////////////////////////
// SD Card

void start_sd_card() {
    /**
    * Set up the SD card for writing
    * Regular writes are enabled if the SD card is working
    */
    bool sd_status = SD.begin(CHIP_SELECT_PIN);

    if (sd_status) {
        timer.setInterval(SD_WRITE_DATA_INTERVAL, append_data_to_file);
        append_data_to_file();
    }
}

void append_data_to_file() {
    /**
    * Write the sensor data to the SD card in JSON format
    * A timestamp is also added to the data
    */
    char filename[50];
    char temp[30];
    StaticJsonBuffer<400> json_buffer;

    // Open up the current datefile - Changes by date
    get_date(temp);
    add_to_array(filename, temp);
    sprintf(temp, "_%s.log", DEVICE_NAME);
    add_to_array(filename, temp);
    data_file = SD.open(filename, FILE_WRITE);

    // Write all the data if the file is available
    if (data_file) {
        JsonObject& entry = json_buffer.createObject();

        get_datetime(temp);
        entry["datetime"] = temp;
        entry["therm_left"] = movements[LEFT];
        entry["therm_right"] = movements[RIGHT];
        entry["therm_up"] = movements[UP];
        entry["therm_down"] = movements[DOWN];
        entry["therm_nodir"] = movements[NO_DIRECTION];
        entry["pir_count"] = motion.count;
        entry["light_status"] = light_state;

        entry.printTo(data_file);
        data_file.println();
        flash(2);
    }

    data_file.close();
}

////////////////////////////////////////////////////////////////////////////////
// Ambient light

void start_light_sensor() {
    /**
    * Start the LDR light sensor with regular reads
    */
    pinMode(LIGHT_SENS_PIN, INPUT);
    timer.setInterval(LIGHT_CHECK_INTERVAL, check_light_sensor);
}

void check_light_sensor() {
    /**
    * Check if the lights are on or off
    * The LDR sensor gives an arbitrary light level, but can at least tell if the lights are on or not.
    */

    int light_level = analogRead(LIGHT_SENS_PIN);
    light_state = light_level > LIGHT_ON_THRESHOLD;
}

////////////////////////////////////////////////////////////////////////////////
// RTC

void start_rtc() {
    /**
    * Start up the real-time clock
    */
    rtc.begin();
    timer.setInterval(RTC_CHECK_DATE_INTERVAL, check_for_date_change);
}

void get_datetime(char* buffer) {
    /** Get the datetime in string format
    * DateTime follows the standard ISO format ("YYYY-MM-DD hh:mm:ss")
    * @param buffer Character buffer to write datetime to; Must be at least 20
    * char wide.
    */
    DateTime now = rtc.now();

    // Get the DateTime into the standard, readable format
    sprintf(buffer, ("%04d-%02d-%02d %02d:%02d:%02d"), now.year(), now.month(), now.day(), now.hour(), now.minute(),
            now.second());
}

void get_date(char* buffer) {
    /**
    * Get the current date string
    * @param buffer Buffer to hold date string. Must be at least 11 characters wide
    */
    DateTime now = rtc.now();

    // Get the DateTime into the standard, readable format
    sprintf(buffer, ("%04d-%02d-%02d"), now.year(), now.month(), now.day());
}

void check_for_date_change() {
    /**
    * Check to see if the date has changed since last check
    * If the date changes, reset all the counters
    */
    DateTime now = rtc.now();
    int current_day = now.day();

    if (current_day != rtc_day) {
        rtc_day = current_day;
        reset_counters();
    }
}

void reset_counters() {
    /**
    * Reset all of the counters (usually because its a new day)
    */
    tracker.reset_movements();
    tracker.reset_background();
    motion.count = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Button

void start_button() {
    /**
    * Start monitoring the state of the button
    */
    pinMode(BUTTON_PIN, INPUT);

    button.state = LOW;
    button.down_time = 0;

    timer.setInterval(BUTTON_CHECK_INTERVAL, check_button);
}

void check_button() {
    /**
    * Keep track of how long the button is depressed for and perform actions depending on how long.
    * If the button is held down for more than 3 seconds, a packet is immediately uploaded if WiFi is connected on
    * release
    * If the button is held down for more than 6 seconds, all motion counters are reset
    */
    button.state = digitalRead(BUTTON_PIN);

    // Button down - Count how long it's down for
    if (button.state == HIGH) {
        button.down_time += BUTTON_CHECK_INTERVAL;

        if (button.down_time > BUTTON_RESET_COUNTERS_DELAY &&
            button.down_time < BUTTON_RESET_COUNTERS_DELAY + BUTTON_CHECK_INTERVAL) {
            flash(3);
        }

        else if (button.down_time > BUTTON_UPLOAD_DATA_DELAY &&
                 button.down_time < BUTTON_UPLOAD_DATA_DELAY + BUTTON_CHECK_INTERVAL) {
            flash(6);
        }

        // Button up - See if anything needs to happen
    } else {
        // Reset counters after... (default: 6 seconds)
        if (button.down_time > BUTTON_RESET_COUNTERS_DELAY) {
            reset_counters();

            // Prompt data upload after... (default: 3 seconds)
        } else if (button.down_time > BUTTON_UPLOAD_DATA_DELAY) {
            upload_data();
        }

        button.down_time = 0;
    }
}

////////////////////////////////////////////////////////////////////////////////
// LED

void start_indicator() {
    /**
    * Set up the indicator for use
    */
    pinMode(LED_PIN, OUTPUT);
    indicator_off();
}

void flash() {
    /**
    * Beep the indicator once.
    * Uses a timeout event to turn the indicator off
    */
    indicator_on();
    timer.setTimeout(LED_ON_PERIOD, flash_check);
}

void flash(int num_flashes) {
    /**
    * Beep the indicator multiple times.
    * At low indicator periods, multiple flashs sound more like a trill.
    */
    if (num_flashes > 0) {
        led_flash_count = 1;
        max_led_flash_count = num_flashes;

        flash();
    }
}

void flash_check() {
    /**
    * Callback for flash - Check if the indicator needs to flash again
    */
    indicator_off();

    if (led_flash_count < max_led_flash_count) {
        led_flash_count++;
        timer.setTimeout(LED_OFF_PERIOD, flash);
    }
}

void indicator_off() {
    /**
    * Turn the LED indicator off
    */
    digitalWrite(LED_PIN, LOW);
}

void indicator_on() {
    /**
    * Turn the LED indicator on
    */
    digitalWrite(LED_PIN, HIGH);
}
