#include <Arduino.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <RTClib.h>
#include "ArduinoJson.h"
#include "Button.h"
#include "ESP8266WiFi.h"
#include "Logging.h"
#include "MLX90621.h"
#include "PIR.h"
#include "SD.h"
#include "SPI.h"
#include "SimpleTimer.h"
#include "ThermalTracker.h"
#include "Wire.h"
#include "user_interface.h"

////////////////////////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////////////////////////
// Versioning
const char* DEVICE_NAME = "NodeMLX";
const char* NODE_MLX_VERSION = "20170825";
const bool DEBUG_ENABLED = true;

const char COMPILE_DATE[12] = __DATE__;
const char COMPILE_TIME[9] = __TIME__;

// Pins
const byte MLX_POWER_PIN = D1;
const byte BUTTON_PIN = D0;
const byte LED_PIN = D1;
const byte PIR_PIN = D4;
const byte CHIP_SELECT_PIN = D8;
const byte LIGHT_SENS_PIN = A0;

// Serial
const long SERIAL_BAUD = 115200;
const int LOGGER_LEVEL = LOG_LEVEL_INFOS;
const char PACKET_START = '#';
const char PACKET_END = '$';
const int PROCESSED_FRAME_CHECK_INTERVAL = 1000;

// Thermal flow
const int REFRESH_RATE = 32;
const long THERMAL_FRAME_INTERVAL = 1000 / REFRESH_RATE;
const long THERMAL_CHECK_MOVEMENT_INTERVAL = 500;
const long BACKGROUND_CHECK_INTERVAL = 200;
const long PRINT_FRAME_INTERVAL = 1000;
const long THERMAL_PRINT_AMBIENT_INTERVAL = 5000;

// Thermal flow tracker
const int TRACKER_NUM_BACKGROUND_FRAMES = 200;
const int TRACKER_MINIMUM_DISTANCE = 150;
const int TRACKER_MINIMUM_BLOB_SIZE = 3;

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

// HTTP Uploading
const char* SERVER_ADDRESS = "www.dweet.io";
const int UPLOAD_SERVER_PORT = 80;
const char GET_REQUEST[] = "GET /dweet/for/";
const char GET_TAIL[] = " HTTP/1.1\r\n\r\n";
const long TIMEOUT = 5000;  // TCP timeout in ms
const int TRACKED_BLOB_BUFFER_SIZE = 5;
const char* NAV_TABLE =
    "<hr><table bgcolor=\"#a4b2ec\" style=\"width:75%\"><th><a href=\"live\">Live feed</a></th><th><a "
    "href=\"average\">Averages</a></th><th><a href=\"variance\">Variances</a></th><th><a "
    "href=\"diff\">Difference</a></th><th><a href=\"active\">Active Pixels</a></th></table>";

// Server
const int SERVER_PORT = 80;
const float DEFAULT_MAX_DISPLAY_TEMPERATURE = 50.0;
const float DEFAULT_MIN_DISPLAY_TEMPERATURE = 0.0;
const int HOT_HUE = 0;
const int COLD_HUE = 240;

// RTC
const long RTC_CHECK_DATE_INTERVAL = 60000;

// SD Card
const long SD_WRITE_DATA_INTERVAL = 5 * 60 * 1000;

// Button
const long BUTTON_CHECK_INTERVAL = 200;
const long BUTTON_UPLOAD_DATA_DELAY = 3000;
const long BUTTON_RESET_COUNTERS_DELAY = 6000;
const bool BUTTON_DEBOUNCE_ENABLED = true;
const long BUTTON_DEBOUNCE_TIME = 50;

// LED
const long LED_ON_PERIOD = 50;
const long LED_OFF_PERIOD = 50;

////////////////////////////////////////////////////////////////////////////////
// Function Prototypes
////////////////////////////////////////////////////////////////////////////////

void start_thermal_flow();
void process_new_frame();
void print_new_movements();
void check_background();
void print_frame();
void print_tracked_blob(TrackedBlob blob);

void start_pir();
void update_pir();
void motion_event();

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
void button_press_event(Button& b);

void start_indicator();
void flash();
void flash(int num_flashes);
void flash_check();
void indicator_off();
void indicator_on();

void handle_root();
void handle_live();
void handle_not_found();
String generate_colour_map(float[4][16]);
String generate_temperature_table(float[4][16]);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

SimpleTimer timer;
MLX90621 thermal_flow;
ThermalTracker tracker;
PIR motion(PIR_PIN, MOTION_COOLDOWN_DEFAULT);
Button button = Button(BUTTON_PIN, BUTTON_PULLUP, BUTTON_DEBOUNCE_ENABLED, BUTTON_DEBOUNCE_TIME);
File data_file;

// Debug
int num_processed_frames;
int last_num_processed_frames;
long last_check_time;

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

// Server
MDNSResponder mdns;
ESP8266WebServer server(SERVER_PORT);
float min_display_temperature = DEFAULT_MIN_DISPLAY_TEMPERATURE;
float max_display_temperature = DEFAULT_MAX_DISPLAY_TEMPERATURE;
TrackedBlob last_blobs[TRACKED_BLOB_BUFFER_SIZE];

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

    start_thermal_flow();

    if (DEBUG_ENABLED) {
        start_wifi();
        start_server();

    } else {
        disable_wifi();
    }

    // start_pir();
    // start_light_sensor();
    // start_rtc();
    // start_sd_card();
    // start_button();

    flash(5);
}

void loop() {
    /**
    * Main loop
    * Everything is called off timer events
    */
    timer.run();
    wdt_reset();

    if (DEBUG_ENABLED) {
        server.handleClient();
    }
}

////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////
// Thermal tracking

void start_thermal_flow() {
    /**
    * Start the thermal flow sensor
    */
    thermal_flow.initialise(REFRESH_RATE);

    timer.setInterval(THERMAL_FRAME_INTERVAL, process_new_frame);
    timer.setTimeout(BACKGROUND_CHECK_INTERVAL, check_background);
    timer.setInterval(THERMAL_PRINT_AMBIENT_INTERVAL, print_ambient_temperature);

    tracker.set_tracking_start_callback(handle_tracked_start);
    tracker.set_tracking_end_callback(handle_tracked_end);

    num_processed_frames = 0;
    timer.setInterval(1000, check_frames_per_second);

    Log.Info("Thermal flow started.");
}

void handle_tracked_start(TrackedBlob blob) {
    Log.Info(
        "%c{\"id\":\"thermal\",\"type\":\"start\",\"t_id\":%d,\"size\":%d,\"start_x\":%d,\"start_y\":%d,\"temp\":%d,"
        "\"w\":%d,\"h\":%d}%c",
        PACKET_START, blob.id, blob.max_size, int(blob.start_pos[X]), int(blob.start_pos[Y]),
        int(blob._blob.average_temperature * 100), blob.max_width, blob.max_height, PACKET_END);
}

void handle_tracked_end(TrackedBlob blob) {
    Log.Info(
        "%c{\"id\":\"thermal\",\"type\":\"end\",\"t_id\":%d,\"av_diff\":%d,\"max_diff\":%d,\"time\":%d,\"frames\":%d,"
        "\"size\":%d,\"travel\":%d,\"temp\":%d,\"w\":%d,\"h\":%d}%c",
        PACKET_START, blob.id, int(blob.average_difference), int(blob.max_difference), blob.event_duration,
        blob.times_updated, blob.max_size, int(blob.travel[X] * 100), int(blob._blob.average_temperature * 100),
        blob.max_width, blob.max_height, PACKET_END);

    // Keep a list of the most recent blobs if the option is enabled
    if (DEBUG_ENABLED) {
        for (int i = TRACKED_BLOB_BUFFER_SIZE - 1; i > 0; i--) {
            // Cycle through the tracked blobs so the most recent is first
            last_blobs[i].copy(last_blobs[i - 1]);
        }
        last_blobs[0].copy(blob);
    }
}

void process_new_frame() {
    /**
    * Grab the next frame in from the MLX90621 sensor and pass it to the tracking
    * algorithm
    */

    long start_time = millis();
    thermal_flow.get_temperatures(frame, true);
    tracker.update(frame);
    long process_time = millis() - start_time;

    Log.Debug("Blobs in frame: %d\tprocess time %l ms", tracker.num_last_blobs, process_time);
    num_processed_frames++;
}

void print_new_movements() {
    /**
    * Print any new movements that have been detected by the tracking algorithm
    */
    if (tracker.has_new_movements()) {
        tracker.get_movements(movements);
        int num_blobs = tracker.num_last_blobs;

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

    if (tracker.is_background_finished()) {
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

void print_ambient_temperature() {
    char temperature[8];

    float ambient = thermal_flow.get_ambient_temperature();
    dtostrf(ambient, 0, 2, temperature);
    Log.Info("%c{\"id\":\"ambient\", \"temp\":%s}%c", PACKET_START, temperature, PACKET_END);
}

void print_tracked_blob(TrackedBlob blob) {
    Log.Info(
        "%c{\"id\":\"thermal\",\"duration\":%l,\"start\":(%d,%d),\"travel\":(%d,%d),\"frames\":%d,\"distance\":%d,"
        "\"size\":%d}%c",
        PACKET_START, blob.event_duration, blob.start_pos[0], blob.start_pos[1], blob.travel[0], blob.travel[1],
        blob.times_updated, int(blob.average_difference), blob._blob.get_size(), PACKET_END);
}

void check_frames_per_second() {
    // Just assume we're fine for the first check
    if (last_check_time == 0) {
        last_check_time = millis() + 1000;
    }

    Log.Debug("Processed frames - %d second(s): %d", PROCESSED_FRAME_CHECK_INTERVAL / 1000, num_processed_frames);
    last_num_processed_frames = (num_processed_frames * 1000) / (millis() - last_check_time);

    last_check_time = millis();
    num_processed_frames = 0;
}

////////////////////////////////////////////////////////////////////////////////
// PIR

void start_pir() {
    /**
    * Start the PIR motion sensor for motion detections
    */

    motion.start();
    motion.calibrate(MOTION_INITIALISATION_TIME);
    motion.set_event_start_callback(motion_event);

    // Start up regular reads
    timer.setInterval(MOTION_CHECK_INTERVAL, update_pir);
    Log.Info(("Motion detection started."));

    motion.update();
}

void update_pir() { motion.update(); }

void motion_event() {
    flash();
    Log.Debug("Motion event @ %d \t: %d - %d ms", millis(), motion.num_detections);
}

////////////////////////////////////////////////////////////////////////////////
// WiFi

void start_wifi() {
    /**
    * Set up the WiFi connection and events.
    * If setup fails, the connection will be retried after a delay (default: 30 seconds) until it is connected.
    * If everything succeeds, regular uploads will be established
    */

    bool connected = attempt_wifi_connection(WIFI_DEFAULT_TIMEOUT);

    if (connected) {
        Log.Info("WiFi Connected!\nIP address: %s", WiFi.localIP().toString().c_str());
        // timer.setInterval(5000, upload_data);

    } else {
        timer.setTimeout(WIFI_RECONNECT_INTERVAL, start_wifi);
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
    if (client.connect(SERVER_ADDRESS, UPLOAD_SERVER_PORT)) {
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

    sprintf(entry, "&therm_left=%d", movements[LEFT]);
    add_to_array(packet_buffer, entry);

    sprintf(entry, "&therm_right=%d", movements[RIGHT]);
    add_to_array(packet_buffer, entry);

    sprintf(entry, "&therm_up=%d", movements[UP]);
    add_to_array(packet_buffer, entry);

    sprintf(entry, "&therm_right=%d", movements[DOWN]);
    add_to_array(packet_buffer, entry);

    sprintf(entry, "&therm_nodir=%d", movements[NO_DIRECTION]);
    add_to_array(packet_buffer, entry);

    sprintf(entry, "&lights_on=%s", (light_state ? "ON" : "OFF"));
    add_to_array(packet_buffer, entry);

    sprintf(entry, "&pir_count=%d", motion.num_detections);
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
    StaticJsonBuffer<500> json_buffer;

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
        entry["pir_count"] = motion.num_detections;
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
    motion.num_detections = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Button

void start_button() {
    /**
    * Start monitoring the state of the button
    */
    button.pressHandler(button_press_event);
}

void button_press_event(Button& b) {
    // TODO - Button events
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

////////////////////////////////////////////////////////////////////////////////
// Server

void start_server() {
    /**
    * Start the web server
    */

    WiFi.softAP(DEVICE_NAME);
    if (mdns.begin(DEVICE_NAME, WiFi.localIP())) {
        Log.Info("MDNS responder started");
    }

    server.on("/", handle_root);
    server.on("/live", handle_live);
    server.on("/average", handle_average);
    server.on("/variance", handle_variance);
    server.on("/diff", handle_diff);
    server.on("/active", handle_active);
    server.onNotFound(handle_not_found);

    server.begin();
    Log.Info("HTTP server started: Local %s, Hosted %s", WiFi.localIP().toString().c_str(),
             WiFi.softAPIP().toString().c_str());
}

void handle_root() {
    /**
    * Generate the HTML for the basic web page.
    * The root page just displays basic information at this stage
    */

    String header =
        "<html><head><title> NodeMLX Info</title><style>table, th, td {border: 1px solid black;}</style><meta "
        "http-equiv=\"refresh\" content=\"1\"/></head><body bgcolor=\"#8a8f8a\">";
    String footer = "</body></html>";

    String body = "<h1>NodeMLX Thermal Tracker - Ver:";
    body += NODE_MLX_VERSION;
    body += "</h1>";

    body += "<h2> Compile time: ";
    body += COMPILE_DATE;
    body += " ";
    body += COMPILE_TIME;
    body += "</h2><hr>";

    body += "<h2> Basic Info</h2>";
    body += generate_basic_info_table();
    body += "<h2> Tracking Variables</h2>";
    body += generate_tracking_info_table();
    body += "<h2>Inter-frame Track Weightings</h2>";
    body += generate_penalty_table();
    body += "<h2> Recent Tracked Blobs</h2>";
    body += generate_last_blobs_table();
    body += "<hr>";
    body += NAV_TABLE;

    handle_server_args();

    server.send(200, "text/html", header + body + footer);
}

void handle_server_args() {
    /**
    * Change variables based on arguments passed to the server.
    */

    for (int i = 0; i < server.args(); i++) {
        if (server.argName(i) == "min") {
            min_display_temperature = server.arg(i).toFloat();
        } else if (server.argName(i) == "max") {
            max_display_temperature = server.arg(i).toFloat();
        } else if (server.argName(i) == "min_blob") {
            tracker.min_blob_size = server.arg(i).toInt();
        } else if (server.argName(i) == "avg_size") {
            tracker.running_average_size = server.arg(i).toInt();
        } else if (server.argName(i) == "max_diff") {
            tracker.max_difference_threshold = server.arg(i).toInt();
        } else if (server.argName(i) == "min_t_diff") {
            tracker.minimum_temperature_differential = server.arg(i).toFloat();
        } else if (server.argName(i) == "ap_scalar") {
            tracker.active_pixel_variance_scalar = server.arg(i).toFloat();
        } else if (server.argName(i) == "pen_pos") {
            TrackedBlob::position_penalty = server.arg(i).toFloat();
        } else if (server.argName(i) == "pen_area") {
            TrackedBlob::area_penalty = server.arg(i).toFloat();
        } else if (server.argName(i) == "pen_aratio") {
            TrackedBlob::aspect_ratio_penalty = server.arg(i).toFloat();
        } else if (server.argName(i) == "pen_dir") {
            TrackedBlob::direction_penalty = server.arg(i).toFloat();
        } else if (server.argName(i) == "pen_temp") {
            TrackedBlob::temperature_penalty = server.arg(i).toFloat();
        } else if (server.argName(i) == "max_dead") {
            tracker.max_dead_frames = server.arg(i).toInt();
        } else if (server.argName(i) == "ad_fuzz") {
            Pixel::adjacency_fuzz = server.arg(i).toInt();
        }
    }
}

String generate_basic_info_table() {
    char temp[30];
    String output = "<table bgcolor=\"#e6f4a4\" style=\"width:50%\">";
    get_datetime(temp);
    output += "<tr><th>Last update</th><td>";
    output += temp;
    output += "</td></tr>";
    output += "<tr><th>Ambient Temperature</th><td>";
    dtostrf(thermal_flow.get_ambient_temperature(), 4, 2, temp);
    output += temp;
    output += " °C</td></tr>";

    output += "<tr><th>Processed frame rate (last second)</th><td>";
    output += last_num_processed_frames;

    output += "<tr><th>Background status</th><td>";
    output += tracker.num_background_frames;
    output += "/";
    output += tracker.running_average_size;

    output += "</td></tr></table>";

    return output;
}

String generate_tracking_info_table() {
    char temp[10];
    String output =
        "<hr><table bgcolor=\"#a7f47d\" style =\"width:50%\"><tr><th>Tracking Variables</th><th>Var "
        "Name</th><th>Value</th></tr><tr>";
    output += "<td>Min blob size</td><td>min_blob</td><td>";
    output += tracker.min_blob_size;
    output += "</td></tr>";

    output += "<td>Running average frames</td><td>avg_size</td><td>";
    output += tracker.running_average_size;
    output += "</td></tr>";

    output += "<td>Max difference threshold</td><td>max_diff</td><td>";
    output += tracker.max_difference_threshold;
    output += "</td></tr>";

    output += "<td>Min temp differential</td><td>min_t_diff</td><td>";
    dtostrf(tracker.minimum_temperature_differential, 4, 2, temp);
    output += temp;
    output += "</td></tr>";

    output += "<td>Active pixel variance scalar</td><td>ap_scalar</td><td>";
    dtostrf(tracker.active_pixel_variance_scalar, 4, 2, temp);
    output += temp;
    output += "</td></tr>";

    output += "<td>Adjacency fuzz factor</td><td>ad_fuzz</td><td>";
    dtostrf(tracker.active_pixel_variance_scalar, 4, 2, temp);
    output += temp;
    output += "</td></tr>";

    output += "<td>Maximum dead frames</td><td>max_dead</td><td>";
    output += tracker.max_dead_frames;
    output += "</td></tr></table>";

    return output;
}

String generate_penalty_table() {
    char temp[10];
    String output =
        "<hr><table bgcolor=\"#eeb77d\" style=\"width:50%\"><th>Blob tracking</th><th>Var "
        "Name</th><th>Value</th></tr><tr>";
    output += "<td>Position penalty</td><td>pen_pos</td><td>";
    dtostrf(TrackedBlob::position_penalty, 4, 2, temp);
    output += temp;
    output += "</td></tr>";
    output += "<td>Area penalty</td><td>pen_area</td><td>";
    dtostrf(TrackedBlob::area_penalty, 4, 2, temp);
    output += temp;
    output += "</td></tr>";
    output += "<td>Aspect Ratio penalty</td><td>pen_aratio</td><td>";
    dtostrf(TrackedBlob::aspect_ratio_penalty, 4, 2, temp);
    output += temp;
    output += "</td></tr>";
    output += "<td>Direction penalty</td><td>pen_dir</td><td>";
    dtostrf(TrackedBlob::direction_penalty, 4, 2, temp);
    output += temp;
    output += "</td></tr>";
    output += "<td>Temperature penalty</td><td>pen_temp</td><td>";
    dtostrf(TrackedBlob::temperature_penalty, 4, 2, temp);
    output += temp;
    output += "</td></tr></table>";

    return output;
}

String generate_last_blobs_table() {
    char temp[10];
    String output =
        "<hr><table bgcolor=\"#a972b4\" style=\"width:80%\"><tr><th>Track id</th><th>Tracked frames</th><th>Max blob "
        "size</th><th>Travel</th><th>Width</th><th>Height</th><th>Temperature</th><th>A diff</th><th>P diff</th><th>AR "
        "diff</th><th>D diff</th><th>T diff</th><th>Num Dead</tr>";

    for (int i = 0; i < TRACKED_BLOB_BUFFER_SIZE; i++) {
        output += "<tr><td>";
        output += last_blobs[i].id;
        output += "</td><td>";
        output += last_blobs[i].times_updated;
        output += "</td><td>";
        output += last_blobs[i].max_size;
        output += "</td><td>";
        dtostrf(last_blobs[i].travel[X], 4, 2, temp);
        output += temp;
        output += "</td><td>";
        output += last_blobs[i].max_width;
        output += "</td><td>";
        output += last_blobs[i].max_height;
        output += "</td><td>";
        dtostrf(last_blobs[i]._blob.average_temperature, 4, 2, temp);
        output += temp;
        output += "</td><td>";
        dtostrf(last_blobs[i].average_area_difference, 4, 2, temp);
        output += temp;
        output += "</td><td>";
        dtostrf(last_blobs[i].average_position_difference, 4, 2, temp);
        output += temp;
        output += "</td><td>";
        dtostrf(last_blobs[i].average_aspect_ratio_difference, 4, 2, temp);
        output += temp;
        output += "</td><td>";
        dtostrf(last_blobs[i].average_direction_difference, 4, 2, temp);
        output += temp;
        output += "</td><td>";
        dtostrf(last_blobs[i].average_temperature_difference, 4, 2, temp);
        output += temp;
        output += "</td><td>";
        output += last_blobs[i].max_num_dead_frames;
        output += "</td></tr>";
    }

    output += "</table>";

    return output;
}

void handle_live() {
    min_display_temperature = 20;
    max_display_temperature = 50;
    String page = generate_live_view(tracker.frame);
    server.send(200, "text/html", page);
}

void handle_average() {
    min_display_temperature = 20;
    max_display_temperature = 50;
    String page = generate_live_view(tracker.pixel_averages);
    server.send(200, "text/html", page);
}

void handle_variance() {
    min_display_temperature = 0;
    max_display_temperature = 5;
    String page = generate_live_view(tracker.pixel_variance);
    server.send(200, "text/html", page);
}

void handle_diff() {
    min_display_temperature = -5;
    max_display_temperature = 5;
    float diff[4][16];

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 16; j++) {
            diff[i][j] = tracker.frame[i][j] - tracker.pixel_averages[i][j];
        }
    }

    String page = generate_live_view(diff);
    server.send(200, "text/html", page);
}

void handle_active() {
    min_display_temperature = 0;
    max_display_temperature = 1;
    float active[4][16];

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 16; j++) {
            float temp = tracker.frame[i][j];
            float average = tracker.pixel_averages[i][j];
            float variance = tracker.pixel_variance[i][j];

            float temperature_difference = absolute(average - temp);

            if (temperature_difference > (variance * 3) &&
                temperature_difference > tracker.minimum_temperature_differential) {
                active[i][j] = 1;
            } else {
                active[i][j] = 0;
            }
        }
    }

    String page = generate_live_view(active);
    server.send(200, "text/html", page);
}

String generate_live_view(float values[4][16]) {
    /**
    * Generate the html for the live page.
    * The live page contains a false-colour temperature map of the sensor output
    */

    String page = "";
    page += generate_colour_map(values);
    page += "<html><head><title> NodeMLX Live Feed</title><meta http-equiv=\"refresh\" content=\"0.25\"/></head><body>";
    page += generate_temperature_table(values);
    page += "<hr><a href=\"\\\">Back</a></body></html>";

    return page;
}

int calculate_hue(float temperature) {
    /**
    * Calculate the hue of the temperature according to the colour map
    * Hue bounds are set by COLD_HUE and HOT_HUE
    * @param temperature Temperature to map to a hue
    * @return Hue of the input temperature relative to the set bounds
    */
    int temp = constrain(temperature * 5, min_display_temperature * 5, max_display_temperature * 5);
    int hue = map(temp, min_display_temperature * 5, max_display_temperature * 5, COLD_HUE, HOT_HUE);
    return hue;
}

String generate_colour_map(float temperatures[4][16]) {
    /**
    * Generate the CSS for displaying the table colours for the thermal image
    * Table generation is handled in generate_temperature_table
    * @param temperature The temperature frame recorded by the sensor
    * @return The CSS script for displaying the table colours
    */
    String css =
        "<style type=\"text/css\">\n.thermal{color: 0xFFFFFF; border: 1px solid black; width: 100%; height: 60%}\n";

    for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 16; x++) {
            char row[50];
            int hue = calculate_hue(temperatures[y][x]);
            sprintf(row, ".r%dc%d{background-color: hsl(%d,100%,50%)}\n", y, x, hue);
            css += row;
        }
    }
    css += "</style>\n";
    return css;
}

String generate_temperature_table(float temperature[4][16]) {
    /**
    * Generate the html for displaying the recorded temperatures
    * Colour mapping is handled in generate_colour_map
    * @param temperature The temperature frame recorded by the sensor
    * @return The HTML script for displaying the table
    */
    String table = "<table class=thermal>\n";

    for (int row = 0; row < 4; row++) {
        table += "<tr>\n";

        for (int column = 0; column < 16; column++) {
            char cell[50];
            char temp[8];
            dtostrf(temperature[row][column], 0, 2, temp);
            sprintf(cell, "<th class=r%dc%d> %s </th>\n", row, column, temp);
            table += cell;
        }
        table += "</tr>\n";
    }

    table += "</table>\n";
    return table;
}

void handle_not_found() {
    /**
    * Display the 404 page for weird site requests
    */
    String message = "File Not Found\n\n";
    message += "URI: ";
    message += server.uri();
    message += "\nMethod: ";
    message += (server.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += server.args();
    message += "\n";
    for (uint8_t i = 0; i < server.args(); i++) {
        message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
    }
    server.send(404, "text/plain", message);
}
