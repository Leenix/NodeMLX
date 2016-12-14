#include <Arduino.h>
#include "MLX90621.h"
#include "ThermalTracker.h"
#include "Logging.h"

const long SERIAL_BAUD = 115200;
const int LOGGER_LEVEL = LOG_LEVEL_DEBUG;

int num_tests = 0;
int num_passed = 0;

ThermalTracker tracker(5);

void build_background_test();
void active_pixel_test();
void pixel_adjacency_test();
void blob_detection_test();
void blob_add_pixel_test();
void track_test();

float zeros[NUM_ROWS][NUM_COLS] =  {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};

float ones[NUM_ROWS][NUM_COLS] =   {{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                                    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                                    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                                    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}};

float twos[NUM_ROWS][NUM_COLS] =   {{2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2},
                                    {2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2},
                                    {2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2},
                                    {2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2}};

float counting[NUM_ROWS][NUM_COLS] =   {{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16},
                                        {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16},
                                        {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16},
                                        {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16}};

float test_frame[NUM_ROWS][NUM_COLS] = {{5,5,5,5,5,5,5,0,0,0,0,0,0,0,0,0},
                                        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                        {0,0,0,0,0,0,0,0,0,0,0,0,2,2,0,0},
                                        {0,9,9,0,0,0,0,0,0,0,0,2,2,2,0,0}};

float blob_test_frame[NUM_ROWS][NUM_COLS] = {{5,5,5,5,0,0,8,8,0,7,7,0,0,0,1,1},
                                        {0,0,0,0,0,0,0,0,0,7,7,0,0,0,0,0},
                                        {9,9,0,3,0,4,4,0,0,0,0,0,2,2,0,0},
                                        {9,9,0,3,0,4,4,0,6,6,0,2,2,2,0,0}};

float track_test_frame_1[NUM_ROWS][NUM_COLS] = {{0,0,5,5,5,5,5,5,5,0,0,0,0,0,0,0},
                                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                {0,0,0,0,0,0,2,2,0,0,0,0,0,0,0,0},
                                                {0,0,0,0,0,0,2,2,0,0,0,0,0,0,0,0}};


float track_test_frame_2[NUM_ROWS][NUM_COLS] = {{0,0,0,0,5,5,5,5,5,5,5,0,0,0,0,0},
                                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                {0,0,2,2,0,0,0,0,0,0,0,0,0,0,0,0},
                                                {0,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0}};

float track_test_frame_3[NUM_ROWS][NUM_COLS] = {{0,0,0,0,0,0,5,5,5,5,5,5,5,0,0,0},
                                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};

float track_test_frame_4[NUM_ROWS][NUM_COLS] = {{0,0,0,0,0,0,0,0,0,0,0,5,5,5,5,5},
                                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};

////////////////////////////////////////////////////////////////////////////////
// Main
void setup(){

    Log.Init(LOGGER_LEVEL, SERIAL_BAUD);
    build_background_test();
    background_average_and_variance_test();
    active_pixel_test();
    pixel_adjacency_test();
    blob_detection_test();
    blob_add_pixel_test();
    distance_test();
    process_blob_test();
    get_num_blobs_test();
    distance_matrix_test();
    sort_tracked_blobs_test();
    track_test();
}

void loop(){
    Log.Info("\t==================================================\n\t\tTests finished. %d tests run; %d passed\n\t\t==================================================\n\n", num_tests, num_passed);
    delay(5000);

}

////////////////////////////////////////////////////////////////////////////////
// Tests

void build_test_background(){
    tracker.reset_background();
    while (!tracker.finished_building_background()){
        tracker.process_frame(zeros);
    }
}

void build_background_test(){
    num_tests++;

    Log.Info("Background build test");
    Log.Debug("Building background: %d/%d frames", tracker.num_background_frames, tracker.running_average_size);
    build_test_background();
    num_passed++;
    Log.Info("Background build test; Pass? true. %d/%d\n\n", num_passed, num_tests);
}

void background_average_and_variance_test(){
    float averages[NUM_ROWS][NUM_COLS];
    float variances[NUM_ROWS][NUM_COLS];
    num_tests++;
    bool passing = false;
    Log.Info("Background average and variance test");

    tracker.reset_background();
    tracker.process_frame(zeros);
    tracker.process_frame(ones);
    tracker.get_averages(averages);
    passing = averages[0][0] == 0.5;
    Log.Debug("New average: %d.%d\t(pass? %T)", int(averages[0][0]), int((averages[0][0] - int(averages[0][0]))*100), passing);

    tracker.process_frame(twos);
    tracker.get_averages(averages);
    if (passing) {
        passing = averages[0][0] == 1;
    }
    Log.Debug("New average: %d.%d\t(pass? %T)", int(averages[0][0]), int((averages[0][0] - int(averages[0][0]))*100), passing);

    tracker.process_frame(twos);
    tracker.get_averages(averages);
    if (passing) {
        passing = averages[0][0] == 1.25;
    }
    Log.Debug("New average: %d.%d\t(pass? %T)", int(averages[0][0]), int((averages[0][0] - int(averages[0][0]))*100), passing);

    tracker.process_frame(ones);
    tracker.get_averages(averages);
    tracker.get_variances(variances);
    if (passing) {
        passing = absolute(averages[0][0] - 1.2) < 0.1 && absolute(variances[0][0] - 0.83666) < 0.1;
    }
    Log.Debug("New average: %d.%d\tNew variance: %d.%d\t(pass? %T)", int(averages[0][0]), int((averages[0][0] - int(averages[0][0]))*100), int(variances[0][0]), int((variances[0][0] - int(variances[0][0]))*100), passing);

    if (passing) {
        num_passed++;
    }
    Log.Info("Background average and variance test: Pass? %T %d/%d\n\n", passing, num_passed, num_tests);
}

void active_pixel_test(){
    Log.Info("Active pixel test");
    num_tests++;
    bool passing = false;
    build_test_background();

    tracker.load_frame(test_frame);

    Pixel pixels[64];

    int num_pixels = tracker.get_active_pixels(pixels);
    Log.Debug("Active pixels: %d/%d", num_pixels, 14);
    passing = num_pixels==14;

    if (passing) {
        num_passed++;
    }

    Log.Info("Active pixel test: Pass? %T %d/%d\n\n", passing, num_passed, num_tests);
}

void pixel_adjacency_test(){
    Log.Info("Adjacency test");
    num_tests++;
    tracker.load_frame(test_frame);
    Pixel pixels[64];
    bool passing = false;
    int num_pixels = tracker.get_active_pixels(pixels);

    Log.Debug("Pixel 1 location: \trow: %d, col: %d", pixels[0].get_x(), pixels[0].get_y());
    Log.Debug("Pixel 2 location: \trow: %d, col: %d", pixels[1].get_x(), pixels[1].get_y());
    Log.Debug("Pixel 48 location: \trow: %d, col: %d", pixels[47].get_x(), pixels[47].get_y());
    Log.Debug("Pixel 1 & 2 adjacent?: %T", pixels[0].is_adjacent(pixels[1]));
    Log.Debug("Pixel 1 & 48 adjacent?: %T", pixels[0].is_adjacent(pixels[47]));
    passing = pixels[0].is_adjacent(pixels[1]) && !pixels[0].is_adjacent(pixels[47]);

    if (passing) {
        num_passed++;
    }
    Log.Info("Adjacency test; Pass? %T %d/%d\n\n", passing, num_passed, num_tests);
}

void blob_detection_test(){
    Log.Info("Blob detection test");
    Blob blobs[MAX_BLOBS];
    int num_blobs = 0;
    num_tests++;
    bool passing = false;

    tracker.load_frame(test_frame);
    Log.Debug("Loaded test frame");
    num_blobs = tracker.get_blobs(blobs);
    Log.Debug("Number of blobs detected: %d", num_blobs);
    passing = num_blobs == 3;

    tracker.remove_small_blobs(blobs, MINIMUM_BLOB_SIZE);
    num_blobs = tracker.get_num_blobs(blobs);
    Log.Debug("Number of blobs detected: %d", num_blobs);
    if(passing){
        passing = num_blobs == 2;
    }

    tracker.load_frame(blob_test_frame);
    Log.Debug("Loaded test frame");
    num_blobs = tracker.get_blobs(blobs);
    Log.Debug("Number of blobs detected: %d", num_blobs);
    if (passing) {
        passing = num_blobs == 9;
    }

    if (passing) {
        num_passed++;
    }

    Log.Info("Blob detection test; Pass? %T %d/%d\n\n", passing, num_passed, num_tests);
}

void blob_add_pixel_test(){
    Blob blob;
    Pixel pixel;
    num_tests++;
    bool passing = false;
    Log.Info("Blob add pixel test");

    blob.clear();
    pixel.set(1, 1, 10.0);
    blob.add_pixel(pixel);
    passing = blob.num_pixels == 1;
    if (passing) {
        passing = blob.centroid[X] == 1.0 && blob.centroid[Y] == 1.0 && blob.average_temperature == 10.0;
    }
    Log.Debug("Blob: num: %d\tcentroid: (%d,%d)\tpassing? %T", blob.num_pixels, int(blob.centroid[X]), int(blob.centroid[Y]), passing);

    pixel.set(1, 2, 20.0);
    blob.add_pixel(pixel);
    if (passing) {
        passing = blob.num_pixels == 2 && blob.centroid[X] == 1.0 && blob.centroid[Y] == 1.5 && blob.average_temperature == 15.0;
    }
    Log.Debug("Blob: num: %d\tcentroid: (%d,%d)\tpassing? %T", blob.num_pixels, int(blob.centroid[X]), int(blob.centroid[Y]), passing);

    pixel.set(1, 3, 30.0);
    blob.add_pixel(pixel);
    if (passing) {
        passing = blob.num_pixels == 3 && blob.centroid[X] == 1.0 && blob.centroid[Y] == 2.0 && blob.average_temperature == 20.0;
    }
    Log.Debug("Blob: num: %d\tcentroid: (%d,%d)\tpassing? %T", blob.num_pixels, int(blob.centroid[X]), int(blob.centroid[Y]), passing);

    pixel.set(2, 3, 40.0);
    blob.add_pixel(pixel);
    if (passing) {
        passing = blob.num_pixels == 4 && blob.centroid[X] == 1.25 && blob.centroid[Y] == 2.25 && blob.average_temperature == 25.0;
    }
    Log.Debug("Blob: num: %d\tcentroid: (%d,%d)\tpassing? %T", blob.num_pixels, int(blob.centroid[X]), int(blob.centroid[Y]), passing);

    if (passing) {
        passing = blob.width == 2 && blob.height == 3;
    }
    Log.Debug("Blob: width: %d\theight: %d\tAR(x100): %d", blob.width, blob.height, int(blob.aspect_ratio*100), passing);

    if (passing) {
        num_passed++;
    }

    Log.Info("Blob add pixel test: Pass? %T %d/%d\n\n", passing, num_passed, num_tests);
}

void distance_test(){
    TrackedBlob t_blob;
    Blob blob;
    Pixel pixel;
    num_tests++;
    bool passing = false;

    Log.Info("Distance calculation test");

    // Set up tracked blob
    blob.clear();
    pixel.set(15, 2, 30);
    blob.add_pixel(pixel);
    pixel.set(15, 3, 30);
    blob.add_pixel(pixel);
    pixel.set(15, 4, 30);
    blob.add_pixel(pixel);
    pixel.set(14, 2, 30);
    blob.add_pixel(pixel);
    pixel.set(14, 3, 30);
    blob.add_pixel(pixel);
    pixel.set(14, 4, 30);
    blob.add_pixel(pixel);
    t_blob.set(blob);
    // Centroid = (14.5, 3), Area = 6, Temperature = 30, AR = 0.66

    // Get distance to unrelated blob
    blob.clear();
    pixel.set(2, 2, 48);
    blob.add_pixel(pixel);
    pixel.set(3, 2, 48);
    blob.add_pixel(pixel);
    pixel.set(4, 2, 48);
    blob.add_pixel(pixel);
    // Centroid = (3, 2), Area = 3, Temperature = 48, AR = 3

    // Distance = 2*(14.5 - 3) + 2*(3 - 2) + 2*(6 - 3) + 10*(3 - 0.666666) + 10(48 - 30) = 234.33334
    float distance = t_blob.get_distance(blob);
    passing = (distance - 234) < 1;
    Log.Debug("Distance between unrelated blobs: %d (%T)", int(distance), passing);

    // Get distance to related blob
    blob.clear();
    pixel.set(11, 2, 30);
    blob.add_pixel(pixel);
    pixel.set(11, 3, 30);
    blob.add_pixel(pixel);
    pixel.set(11, 4, 30);
    blob.add_pixel(pixel);
    pixel.set(10, 2, 30);
    blob.add_pixel(pixel);
    pixel.set(10, 3, 30);
    blob.add_pixel(pixel);
    pixel.set(10, 4, 30);
    blob.add_pixel(pixel);
    // Centroid = (10.5, 3), Area = 6, Temperature = 30, AR = 0.66

    // Distance = 2*(14.5 - 10.5) + 2*(3 - 3) + 2*(6 - 6) + 10*(3 - 3) + 10*(30 - 30) = 8
    distance = t_blob.get_distance(blob);
    if(passing){
        passing = distance == 8.0;
    }
    Log.Debug("Distance between related blobs: %d (%T)", int(distance), passing);


    // Test out distance using predicted path
    t_blob.update_blob(blob);
    blob.clear();
    pixel.set(7, 2, 30);
    blob.add_pixel(pixel);
    pixel.set(7, 3, 30);
    blob.add_pixel(pixel);
    pixel.set(7, 4, 30);
    blob.add_pixel(pixel);
    pixel.set(6, 2, 30);
    blob.add_pixel(pixel);
    pixel.set(6, 3, 30);
    blob.add_pixel(pixel);
    pixel.set(6, 4, 30);
    blob.add_pixel(pixel);
    // Centroid = (6.5, 3), Area = 6, Temperature = 30, AR = 0.66

    // Distance = 2*(6.5 - 6.5) + 2*(3 - 3) + 2*(6 - 6) + 10*(3 - 3) + 10*(30 - 30) = 0
    distance = t_blob.get_distance(blob);
    Log.Debug("Distance between related blobs (with prediction): %d (%T)", int(distance), passing);
    if(passing){
        passing = distance == 0;
    }

    if(passing){
        num_passed++;
    }

    Log.Info("Distance calculation test: Pass? %T %d/%d\n\n", passing, num_passed, num_tests);
}

void distance_matrix_test(){
    /**
    * 3 tracked blobs; 3 New blobs
    * 2 of the new blobs will match the tracked blobs closely.
    * The last combination will not match up
    */
    TrackedBlob tracked_blobs[MAX_BLOBS];
    Blob blobs[MAX_BLOBS];
    Pixel pixel;
    float distance_matrix[MAX_BLOBS][MAX_BLOBS];
    num_tests++;
    bool passing;
    Log.Info("Distance matrix test");

    // Blob 0 matches TBlob 0
    pixel.set(2, 2, 10);
    blobs[0].add_pixel(pixel);
    pixel.set(2, 3, 10);
    blobs[0].add_pixel(pixel);
    pixel.set(3, 2, 10);
    blobs[0].add_pixel(pixel);
    pixel.set(3, 3, 10);
    blobs[0].add_pixel(pixel);
    tracked_blobs[0].set(blobs[0]);

    // Blob 1 matches TBlob 2
    pixel.set(9, 1, 20);
    blobs[1].add_pixel(pixel);
    pixel.set(9, 2, 20);
    blobs[1].add_pixel(pixel);
    tracked_blobs[2].set(blobs[1]);
    pixel.set(9, 3, 20);
    blobs[1].add_pixel(pixel);

    // Blob 2 matches nothing!
    pixel.set(15, 1, 30);
    blobs[2].add_pixel(pixel);
    pixel.set(14, 2, 30);
    blobs[2].add_pixel(pixel);
    tracked_blobs[1].set(blobs[2]);

    blobs[2].clear();
    pixel.set(7, 0, 90);
    blobs[2].add_pixel(pixel);
    pixel.set(6, 0, 90);
    blobs[2].add_pixel(pixel);

    tracker.generate_distance_matrix(tracked_blobs, blobs, distance_matrix);

    if(Log.getLevel() >= LOG_LEVEL_DEBUG){
        for (int i = 0; i < MAX_BLOBS; i++) {
            Serial.print("[");
            for (int j = 0; j < MAX_BLOBS; j++) {
                Serial.print("\t");
                Serial.print(distance_matrix[i][j], 2);
            }
            Serial.println("]");
        }
    }

    int indexes[2];
    float distance = tracker.get_lowest_distance(distance_matrix, indexes);
    passing = (indexes[0] == 0 && indexes[1] == 0 && distance == 0.0);
    Log.Debug("Tracked blob %d matches blob %d with a distance of %d (passing? %T)", indexes[0], indexes[1], int(distance), passing);
    tracker.remove_distance_row_col(indexes[0], indexes[1], distance_matrix);

    distance = tracker.get_lowest_distance(distance_matrix, indexes);
    if(passing){
        passing = (indexes[0] == 2 && indexes[1] == 1 && (distance - 4.67 < 1));
    }
    Log.Debug("Tracked blob %d matches blob %d with a distance of %d (passing? %T)", indexes[0], indexes[1], int(distance), passing);
    tracker.remove_distance_row_col(indexes[0], indexes[1], distance_matrix);

    distance = tracker.get_lowest_distance(distance_matrix, indexes);
    if (passing) {
        passing = (indexes[0] == -1 && indexes[1] == -1 && distance == 999);
    }
    Log.Debug("Tracked blob %d matches blob %d with a distance of %d (No matches left) (passing? %T)", indexes[0], indexes[1], int(distance), passing);

    if (passing) {
        num_passed++;
    }
    Log.Info("Distance matrix test: Pass? %T %d/%d\n\n", passing, num_passed, num_tests);
}

void get_num_blobs_test(){
    Blob blobs[MAX_BLOBS];
    TrackedBlob tracked_blobs[MAX_BLOBS];
    Pixel pixel;
    Blob blob;
    num_tests++;
    bool passing = false;
    Log.Info("Get num blobs test");

    for (int i = 0; i < MAX_BLOBS; i++) {
        pixel.set(i, i, i);
        blobs[i].add_pixel(pixel);
    }
    int num_blobs = tracker.get_num_blobs(blobs);
    passing = num_blobs == MAX_BLOBS;
    Log.Debug("Number of active blobs in array: %d (pass? %T)", num_blobs, passing);

    for (int i = 0; i < MAX_BLOBS/2; i++) {
        tracked_blobs[i].set(blobs[i]);
    }
    num_blobs = tracker.get_num_blobs(tracked_blobs);
    if (passing) {
        passing = num_blobs == MAX_BLOBS/2;
    }
    Log.Debug("Number of tracked blobs in array: %d (pass? %T)", num_blobs, passing);

    if (passing) {
        num_passed++;
    }
    Log.Info("Get num blobs test: Pass? %T %d/%d\n\n", passing, num_passed, num_tests);
}

void sort_tracked_blobs_test(){
    TrackedBlob tracked_blobs[MAX_BLOBS];
    Blob blob;
    Pixel pixel;
    num_tests++;
    Log.Info("Sort tracked blobs test");
    bool passing = false;

    pixel.set(0, 0, 0);
    blob.add_pixel(pixel);
    tracked_blobs[0].set(blob);

    blob.clear();
    pixel.set(1, 1, 1);
    blob.add_pixel(pixel);
    tracked_blobs[1].set(blob);

    blob.clear();
    pixel.set(2, 2, 2);
    blob.add_pixel(pixel);
    tracked_blobs[2].set(blob);

    blob.clear();
    pixel.set(3, 3, 3);
    blob.add_pixel(pixel);
    tracked_blobs[3].set(blob);

    blob.clear();
    pixel.set(4, 4, 4);
    blob.add_pixel(pixel);
    tracked_blobs[4].set(blob);

    // Initial check to make sure the numbers are good
    int num_tracked = tracker.get_num_blobs(tracked_blobs);
    int num_updated = tracker.get_num_updated_blobs(tracked_blobs);
    passing = num_tracked == 5 && num_updated == 5;
    Log.Debug("Num tracked blobs: %d\tupdated: %d\t(pass: %T)", num_tracked, num_updated, passing);

    // Scratch one off and recheck numbers
    tracked_blobs[1].reset_updated_status();
    num_tracked = tracker.get_num_blobs(tracked_blobs);
    num_updated = tracker.get_num_updated_blobs(tracked_blobs);
    if (passing) {
        passing = num_tracked == 5 && num_updated == 4;
    }
    Log.Debug("Num tracked blobs: %d\tupdated: %d\t(pass: %T)", num_tracked, num_updated, passing);


    // Check the contents of tracked blob 1 (to be removed in next pass)
    int x1 = tracked_blobs[1]._blob.centroid[X];
    int y1 = tracked_blobs[1]._blob.centroid[Y];
    float t1 = tracked_blobs[1]._blob.average_temperature;
    if (passing) {
        passing = x1 == 1 && y1 == 1;
    }
    Log.Debug("Tracked blob 1: %d, %d, %d \t(pass? %T)" , x1, y1, int(t1), passing);
    for (int i = 0; i < num_tracked; i++) {
        Log.Debug("Tracked blob %d: %d, %d\t updated? %T", i, int(tracked_blobs[i]._blob.centroid[X]), int(tracked_blobs[i]._blob.centroid[Y]), tracked_blobs[i].has_updated());
    }

    // Re-sort to remove un-updated blobs
    tracker.sort_tracked_blobs(tracked_blobs);
    num_tracked = tracker.get_num_blobs(tracked_blobs);
    num_updated = tracker.get_num_updated_blobs(tracked_blobs);
    if (passing) {
        passing = num_tracked == 4 && num_updated == 4;
    }
    Log.Debug("Num tracked blobs: %d\tupdated: %d\t(pass: %T)", num_tracked, num_updated, passing);
    for (int i = 0; i < num_tracked; i++) {
        Log.Debug("Tracked blob %d: %d, %d\t updated? %T", i, int(tracked_blobs[i]._blob.centroid[X]), int(tracked_blobs[i]._blob.centroid[Y]), tracked_blobs[i].has_updated());
    }

    // Make sure the pixel has changed
    x1 = tracked_blobs[1]._blob.centroid[X];
    y1 = tracked_blobs[1]._blob.centroid[Y];
    t1 = tracked_blobs[1]._blob.average_temperature;
    if (passing) {
        passing = x1 != 1 && y1 != 1;
    }
    Log.Debug("Tracked blob 1: %d, %d, %d \t(pass? %T)" , x1, y1, int(t1), passing);

    // Now remove the rest, but leave the blob in position 1 (it will move to 0 after the shuffle)
    tracked_blobs[0].reset_updated_status();
    tracked_blobs[2].reset_updated_status();
    tracked_blobs[3].reset_updated_status();
    num_tracked = tracker.get_num_blobs(tracked_blobs);
    num_updated = tracker.get_num_updated_blobs(tracked_blobs);
    if (passing) {
        passing = num_tracked == 4 && num_updated == 1;
    }
    Log.Debug("Num tracked blobs: %d\tupdated: %d\t(pass: %T)", num_tracked, num_updated, passing);
    for (int i = 0; i < num_tracked; i++) {
        Log.Debug("Tracked blob %d: %d, %d\t updated? %T", i, int(tracked_blobs[i]._blob.centroid[X]), int(tracked_blobs[i]._blob.centroid[Y]), tracked_blobs[i].has_updated());
    }

    // Check that the sort function can remove multiple blobs at once
    tracker.sort_tracked_blobs(tracked_blobs);
    num_tracked = tracker.get_num_blobs(tracked_blobs);
    num_updated = tracker.get_num_updated_blobs(tracked_blobs);
    if (passing) {
        passing = num_tracked == 1 && num_updated == 1;
    }
    Log.Debug("Num tracked blobs: %d\tupdated: %d\t(pass: %T)", num_tracked, num_updated, passing);

    // Check that blob 1 is empty
    x1 = tracked_blobs[1]._blob.centroid[X];
    y1 = tracked_blobs[1]._blob.centroid[Y];
    t1 = tracked_blobs[1]._blob.average_temperature;
    if (passing) {
        passing = x1 < 0 && y1 < 0;
    }
    Log.Debug("Tracked blob 1: %d, %d, %d \t(pass? %T)" , x1, y1, int(t1), passing);

    if (passing) {
        num_passed++;
    }

    Log.Info("Sort tracked blobs test: Passed? %T %d/%d\n\n", passing, num_passed, num_tests);
}

void process_blob_test(){
    TrackedBlob t_blob;
    Blob blob;
    Pixel pixel;
    long movements[5];
    bool passing = false;
    num_tests++;

    Log.Info("Tracked blob movement test");

    pixel.set(0, 0, 25.0);
    blob.add_pixel(pixel);
    pixel.set(0, 1, 25.0);
    blob.add_pixel(pixel);
    pixel.set(1, 0, 25.0);
    blob.add_pixel(pixel);
    pixel.set(1, 1, 25.0);
    blob.add_pixel(pixel);
    Log.Debug("Blob centroid: (%d, %d)", int(blob.centroid[X]), int(blob.centroid[Y]));
    t_blob.set(blob);
    Log.Debug("Tracked blob travel: \tX:%d\tY:%d", int(t_blob.get_travel(X)), int(t_blob.get_travel(Y)));

    blob.clear();
    pixel.set(4, 5, 25.0);
    blob.add_pixel(pixel);
    pixel.set(4, 6, 25.0);
    blob.add_pixel(pixel);
    pixel.set(5, 5, 25.0);
    blob.add_pixel(pixel);
    pixel.set(5, 6, 25.0);
    blob.add_pixel(pixel);
    Log.Debug("Blob centroid: (%d, %d)", int(blob.centroid[X]), int(blob.centroid[Y]));
    t_blob.update_blob(blob);
    Log.Debug("Tracked blob travel: \tX:%d\tY:%d", int(t_blob.get_travel(X)), int(t_blob.get_travel(Y)));
    passing = t_blob.get_travel(X) == 4;

    blob.clear();
    pixel.set(14, 0, 25.0);
    blob.add_pixel(pixel);
    pixel.set(15, 0, 25.0);
    blob.add_pixel(pixel);
    pixel.set(14, 1, 25.0);
    blob.add_pixel(pixel);
    pixel.set(15, 1, 25.0);
    blob.add_pixel(pixel);
    Log.Debug("Blob centroid: (%d, %d)", int(blob.centroid[X]), int(blob.centroid[Y]));
    t_blob.update_blob(blob);

    Log.Debug("Tracked blob travel: \tX:%d\tY:%d", int(t_blob.get_travel(X)), int(t_blob.get_travel(Y)));
    if(passing){
        passing = t_blob.get_travel(X) == 14;
    }

    tracker.reset_movements();
    tracker.process_blob_movements(t_blob);
    tracker.get_movements(movements);
    Log.Debug("Movements detected: \tL%l\tR%l\tU%l\tD%l\tZ%l", movements[LEFT], movements[RIGHT], movements[UP], movements[DOWN], movements[NO_DIRECTION]);

    if(passing){
        passing = movements[RIGHT] == 1;
    }

    if (passing){
        num_passed++;
    }

    Log.Info("Tracked blob movement test: Pass? %T %d/%d\n\n", passing, num_passed, num_tests);
}

void track_test(){
    TrackedBlob tracked_blobs[MAX_BLOBS];
    Blob blobs[MAX_BLOBS];
    long movements[5];

    bool passing = false;
    num_tests++;
    Log.Info("Tracking test");

    build_test_background();

    while (!tracker.finished_building_background()){
        tracker.process_frame(zeros);
        Log.Debug("Building background: %d/%d frames", tracker.num_background_frames, tracker.running_average_size);
        Serial.flush();
    }

    // Read movements to clear the has_new_movements flag
    tracker.get_movements(movements);
    tracker.reset_movements();

    // Test frame 0 contains 3 blobs - one of them is small
    tracker.load_frame(test_frame);
    tracker.get_blobs(blobs);
    int num_blobs = tracker.get_num_blobs(blobs);
    int num_tracked = tracker.get_num_blobs(tracked_blobs);
    tracker.track_blobs(blobs, tracked_blobs);
    int num_updated = tracker.get_num_updated_blobs(tracked_blobs);
    Log.Debug("Frame %d:\tBlobs: %d\tTracked: %d/%d tracked", 0, num_blobs, num_tracked, num_updated);
    for (int i = 0; i < num_updated; i++) {
        Log.Debug("Tracked blob %d: Size: %d, Temp: %d, Centroid: (%d,%d)", i, tracked_blobs[i]._blob.get_size(), int(tracked_blobs[i]._blob.average_temperature), int(tracked_blobs[i]._blob.centroid[X]),  int(tracked_blobs[i]._blob.centroid[Y]));
    }
    passing = num_blobs == 3 && num_tracked == 0 && num_updated == 3 && !tracker.has_new_movements();
    if (tracker.has_new_movements()){
        tracker.get_movements(movements);
        Log.Debug("Movements detected: \tL%l\tR%l\tU%l\tD%l\tZ%l\n", movements[LEFT], movements[RIGHT], movements[UP], movements[DOWN], movements[NO_DIRECTION]);
    }

    // Test frame 1 has 2 blobs - both are similar to the last frame's blobs (small blob missing)
    tracker.load_frame(track_test_frame_1);
    tracker.get_blobs(blobs);
    num_blobs = tracker.get_num_blobs(blobs);
    num_tracked = tracker.get_num_blobs(tracked_blobs);
    tracker.track_blobs(blobs, tracked_blobs);
    num_updated = tracker.get_num_updated_blobs(tracked_blobs);
    Log.Debug("Frame %d:\tBlobs: %d\tTracked: %d/%d tracked", 1, num_blobs, num_tracked, num_updated);
    for (int i = 0; i < num_updated; i++) {
        Log.Debug("Tracked blob %d: Size: %d, Temp: %d, Centroid: (%d,%d)", i, tracked_blobs[i]._blob.get_size(), int(tracked_blobs[i]._blob.average_temperature), int(tracked_blobs[i]._blob.centroid[X]),  int(tracked_blobs[i]._blob.centroid[Y]));
    }
    if (passing) {
        passing = num_blobs == 2 && num_tracked == 3 && num_updated == 2 && tracker.has_new_movements();
    }
    if (tracker.has_new_movements()){
        tracker.get_movements(movements);
        Log.Debug("Movements detected: \tL%l\tR%l\tU%l\tD%l\tZ%l\n", movements[LEFT], movements[RIGHT], movements[UP], movements[DOWN], movements[NO_DIRECTION]);
        if (passing) {
            passing = movements[NO_DIRECTION] == 1;
        }
    }

    // Test frame 2 has 2 blobs - none new
    tracker.load_frame(track_test_frame_2);
    tracker.get_blobs(blobs);
    num_blobs = tracker.get_num_blobs(blobs);
    num_tracked = tracker.get_num_blobs(tracked_blobs);
    tracker.track_blobs(blobs, tracked_blobs);
    num_updated = tracker.get_num_updated_blobs(tracked_blobs);
    Log.Debug("Frame %d:\tBlobs: %d\tTracked: %d/%d tracked", 2, num_blobs, num_tracked, num_updated);
    for (int i = 0; i < num_updated; i++) {
        Log.Debug("Tracked blob %d: Size: %d, Temp: %d, Centroid: (%d,%d)", i, tracked_blobs[i]._blob.get_size(), int(tracked_blobs[i]._blob.average_temperature), int(tracked_blobs[i]._blob.centroid[X]),  int(tracked_blobs[i]._blob.centroid[Y]));
    }
    if (passing) {
        passing = num_blobs == 2 && num_tracked == 2 && num_updated == 2 && !tracker.has_new_movements();
    }
    if (tracker.has_new_movements()){
        tracker.get_movements(movements);
        Log.Debug("Movements detected: \tL%l\tR%l\tU%l\tD%l\tZ%l\n", movements[LEFT], movements[RIGHT], movements[UP], movements[DOWN], movements[NO_DIRECTION]);
    }

    // Test frame 3 contains 1 blob - A blob from the last frame has left
    tracker.load_frame(track_test_frame_3);
    tracker.get_blobs(blobs);
    num_blobs = tracker.get_num_blobs(blobs);
    num_tracked = tracker.get_num_blobs(tracked_blobs);
    tracker.track_blobs(blobs, tracked_blobs);
    num_updated = tracker.get_num_updated_blobs(tracked_blobs);
    Log.Debug("Frame %d:\tBlobs: %d\tTracked: %d/%d tracked", 3, num_blobs, num_tracked, num_updated);
    for (int i = 0; i < num_updated; i++) {
        Log.Debug("Tracked blob %d: Size: %d, Temp: %d, Centroid: (%d,%d)", i, tracked_blobs[i]._blob.get_size(), int(tracked_blobs[i]._blob.average_temperature), int(tracked_blobs[i]._blob.centroid[X]),  int(tracked_blobs[i]._blob.centroid[Y]));
    }
    if (passing) {
        passing = num_blobs == 1 && num_tracked == 2 && num_updated == 1 && tracker.has_new_movements();
    }
    if (tracker.has_new_movements()){
        tracker.get_movements(movements);
        Log.Debug("Movements detected: \tL%l\tR%l\tU%l\tD%l\tZ%l\n", movements[LEFT], movements[RIGHT], movements[UP], movements[DOWN], movements[NO_DIRECTION]);
        if (passing) {
            passing = movements[LEFT] == 1;
        }
    }

    // Test frame 4 has 1 blob - none new
    tracker.load_frame(track_test_frame_4);
    tracker.get_blobs(blobs);
    num_blobs = tracker.get_num_blobs(blobs);
    num_tracked = tracker.get_num_blobs(tracked_blobs);
    tracker.track_blobs(blobs, tracked_blobs);
    num_updated = tracker.get_num_updated_blobs(tracked_blobs);
    Log.Debug("Frame %d:\tBlobs: %d\tTracked: %d/%d tracked", 4, num_blobs, num_tracked, num_updated);
    for (int i = 0; i < num_updated; i++) {
        Log.Debug("Tracked blob %d: Size: %d, Temp: %d, Centroid: (%d,%d)", i, tracked_blobs[i]._blob.get_size(), int(tracked_blobs[i]._blob.average_temperature), int(tracked_blobs[i]._blob.centroid[X]),  int(tracked_blobs[i]._blob.centroid[Y]));
    }
    if (passing) {
        passing = num_blobs == 1 && num_tracked == 1 && num_updated == 1 && !tracker.has_new_movements();
    }
    if (tracker.has_new_movements()){
        tracker.get_movements(movements);
        Log.Debug("Movements detected: \tL%l\tR%l\tU%l\tD%l\tZ%l", movements[LEFT], movements[RIGHT], movements[UP], movements[DOWN], movements[NO_DIRECTION]);
    }

    tracker.load_frame(zeros);
    tracker.get_blobs(blobs);
    num_blobs = tracker.get_num_blobs(blobs);
    num_tracked = tracker.get_num_blobs(tracked_blobs);
    tracker.track_blobs(blobs, tracked_blobs);
    num_updated = tracker.get_num_updated_blobs(tracked_blobs);
    Log.Debug("Frame %d:\tBlobs: %d\tTracked: %d/%d tracked", 4, num_blobs, num_tracked, num_updated);
    for (int i = 0; i < num_updated; i++) {
        Log.Debug("Tracked blob %d: Size: %d, Temp: %d, Centroid: (%d,%d)", i, tracked_blobs[i]._blob.get_size(), int(tracked_blobs[i]._blob.average_temperature), int(tracked_blobs[i]._blob.centroid[X]),  int(tracked_blobs[i]._blob.centroid[Y]));
    }
    if (passing) {
        passing = num_blobs == 0 && num_tracked == 1 && num_updated == 0 && tracker.has_new_movements();
    }
    if (tracker.has_new_movements()){
        tracker.get_movements(movements);
        Log.Debug("Movements detected: \tL%l\tR%l\tU%l\tD%l\tZ%l", movements[LEFT], movements[RIGHT], movements[UP], movements[DOWN], movements[NO_DIRECTION]);
        if (passing) {
            passing = movements[RIGHT] == 1;
        }
    }

    if (passing) {
        num_passed++;
    }

    Log.Info("Track test: Pass? %T %d/%d\n\n", passing, num_passed, num_tests);
}
