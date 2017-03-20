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
#include "i2c_t3.h"

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
const char* WIFI_SSID = "Turbo Beemo 2.4Ghz";
const char* WIFI_PASSWORD = "tnetennba";
const int WIFI_DEFAULT_TIMEOUT = 2000;
const long WIFI_RECONNECT_INTERVAL = 10000;

// HTTP Uploading
const char* SERVER_ADDRESS = "www.dweet.io";
const int UPLOAD_SERVER_PORT = 80;
const char GET_REQUEST[] = "GET /dweet/for/";
const char GET_TAIL[] = " HTTP/1.1\r\n\r\n";
const long TIMEOUT = 5000;  // TCP timeout in ms

// Server
const int SERVER_PORT = 80;
const char RELOAD_SCRIPT[] =
    "<script>\n$(document).ready(function(){setInterval(function(){cache_clear()},500);});function "
    "cache_clear(){window.location.reload(true);}</script>";
const float MAX_DISPLAY_TEMPERATURE = 50.0;
const float MIN_DISPLAY_TEMPERATURE = 0.0;
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
ThermalTracker tracker(TRACKER_NUM_BACKGROUND_FRAMES, TRACKER_MINIMUM_DISTANCE, TRACKER_MINIMUM_BLOB_SIZE);
PIR motion(PIR_PIN, MOTION_COOLDOWN_DEFAULT);
Button button = Button(BUTTON_PIN, BUTTON_PULLUP, BUTTON_DEBOUNCE_ENABLED, BUTTON_DEBOUNCE_TIME);
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

// Server
MDNSResponder mdns;
ESP8266WebServer server(SERVER_PORT);

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

    // start_thermal_flow();
    start_pir();
    // start_light_sensor();
    // start_rtc();
    // start_sd_card();
    start_indicator();
    // start_wifi();
    // start_button();

    flash(5);
}

void loop() {
    /**
    * Main loop
    * Everything is called off timer events
    */
    timer.run();
    server.handleClient();
    button.process();
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

    sprintf(entry, "&pir_count=%l", motion.num_detections);
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

    if (mdns.begin(DEVICE_NAME, WiFi.localIP())) {
        Log.Info("MDNS responder started");
    }

    server.on("/", handle_root);
    server.on("/live", handle_live);
    server.onNotFound(handle_not_found);

    server.begin();
    Log.Info("HTTP server started: %s", WiFi.localIP().toString().c_str());
}

void handle_root() {
    /**
    * Generate the HTML for the basic web page.
    * The root page just displays basic information at this stage
    * TODO - Add a config menu to the web server to change tracking variables on the fly
    */

    String header = "<html><head><title> NodeMLX Info</title><meta http-equiv=\"refresh\" content=\"5\"/></head><body>";
    String footer = "</body></html>";
    char temp[100];

    String body = "<table style=\"width:50%\">";

    get_datetime(temp);
    body += "<tr><th>Last update</th><td>";
    body += temp;
    body += "</td></tr>";
    body += "<tr><th>Thermal left</th><tr>";
    body += tracker.movements[LEFT];
    body += "</td></tr>";
    body += "<tr><th>Thermal right</th><tr>";
    body += tracker.movements[RIGHT];
    body += "</td></tr>";
    body += "<tr><th>Thermal up</th><tr>";
    body += tracker.movements[UP];
    body += "</td></tr>";
    body += "<tr><th>Thermal down</th><tr>";
    body += tracker.movements[DOWN];
    body += "</td></tr>";
    body += "<tr><th>Thermal none</th><tr>";
    body += tracker.movements[NO_DIRECTION];
    body += "</td></tr>";
    body += "<tr><th>PIR Count</th><tr>";
    body += motion.num_detections;
    body += "</td></tr>";
    body += "<tr><th>Light state</th><tr>";
    body += light_state + "</td></tr>";
    dtostrf(thermal_flow.get_ambient_temperature(), 0, 2, temp);
    body += "<tr><th>Ambient Temperature</th><tr>";
    body += temp;
    body += " °C</td></tr>";
    body += "<hr><a href=\"live\">Live feed</a>";

    server.send(200, "text/html", header + body + footer);
}

void handle_live() {
    /**
    * Generate the html for the live page.
    * The live page contains a false-colour temperature map of the sensor output
    */
    String page = RELOAD_SCRIPT;
    page += generate_colour_map(tracker.frame);
    page += "<html><head><title> NodeMLX Live Feed</title></head><body>";
    page += generate_temperature_table(tracker.frame);
    page += "<hr><a href=\"\\\">Back</a></body></html>";

    server.send(200, "text/html", page);
}

int calculate_hue(float temperature) {
    /**
    * Calculate the hue of the temperature according to the colour map
    * Hue bounds are set by COLD_HUE and HOT_HUE
    * @param temperature Temperature to map to a hue
    * @return Hue of the input temperature relative to the set bounds
    */
    int temp = constrain(temperature * 5, MIN_DISPLAY_TEMPERATURE * 5, MAX_DISPLAY_TEMPERATURE * 5);
    int hue = map(temp, MIN_DISPLAY_TEMPERATURE * 5, MAX_DISPLAY_TEMPERATURE * 5, COLD_HUE, HOT_HUE);
    return hue;
}

String generate_colour_map(float temperatures[4][16]) {
    String css = "<style type=\"text/css\">\n.thermal{color: 0xFFFFFF}\n";

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
        table += "<td>\n";

        for (int column = 0; column < 16; column++) {
            char cell[50];
            char temp[8];
            dtostrf(temperature[row][column], 0, 2, temp);
            sprintf(cell, "<td class=r%dc%d> %s </td>\n", row, column, temp);
            table += cell;
        }
        table += "</td>\n";
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
