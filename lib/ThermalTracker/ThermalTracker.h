#include <Arduino.h>
#include <SimpleTimer.h>
#include <stdarg.h>
#include "Blob.h"
#include "Pixel.h"
#include "TrackedBlob.h"

const static char* TRACKER_VERSION = "20170606";

const int FRAME_WIDTH = 16;
const int FRAME_HEIGHT = 4;
const int MAX_BLOBS = 8;

// Default configuration
const int DEFAULT_MIN_TRAVEL_THRESHOLD = 4;
const int DEFAULT_MIN_BLOB_SIZE = 3;
const int DEFAULT_MAX_DIFFERENCE_THRESHOLD = 400;
const int DEFAULT_RUNNING_AVERAGE_SIZE = 800;
const float DEFAULT_MIN_TEMPERATURE_DIFFERENTIAL = 0.5;
const float DEFAULT_ACTIVE_PIXEL_VARIANCE_SCALAR = 4;
const unsigned int DEFAULT_MAX_DEAD_FRAMES = 4;

// Default blob tracking configuration
const float DEFAULT_POSITION_PENALTY = 2.0;
const float DEFAULT_AREA_PENALTY = 5.0;
const float DEFAULT_ASPECT_RATIO_PENALTY = 10.0;
const float DEFAULT_TEMPERATURE_PENALTY = 10.0;
const float DEFAULT_DIRECTION_PENALTY = 50.0;
const float DEFAULT_DEAD_FRAME_PENALTY = DEFAULT_MAX_DIFFERENCE_THRESHOLD / DEFAULT_MAX_DEAD_FRAMES;
const int DEFAULT_ADJACENCY_FUZZ = 1;

const int ADD_TO_BACKGROUND_DELAY = 20;
const int UNCHANGED_FRAME_DELAY = 50;

const bool INVERT_TRAVEL_DIRECTION = false;
const int NUM_DIRECTION_CATEGORIES = 5;
enum directions { LEFT = 0, RIGHT = 1, UP = 2, DOWN = 3, NO_DIRECTION = 4 };

typedef void (*event_callback)(void); /**< Callback function structure - must have no parameters. */
typedef void (*tracked_callback)(TrackedBlob blob);

class ThermalTracker {
   public:
    /**
    * Constructor - Make a new thermal tracker object with the default configuration
    * The thermal tracker uses a MLX90621 thermopile array to observe moving objects in its view.
    */
    ThermalTracker();

    /**
    * Process an input thermal frame.
    * If a background has not yet been established, the frame goes directly to the background without tracking.
    * If the background has already been built, then the frame is analysed to detect and track movement.
    * @float frame_buffer A 2D array containing the pixel temperatures from the thermopile sensor.
    */
    void update(float frame_buffer[FRAME_HEIGHT][FRAME_WIDTH]);

    /**
    * Determine if the tracker has finished build its background frames.
    * @return True if the tracker has gathered the minumum number of frames.
    */
    bool is_background_finished();

    /**
    * Reset the number of frames in the running background, forcing the tracker to recreate.
    */
    void reset_background();

    /**
    * Get the average temperatures of the background pixels.
    * @param frame_buffer A 2D array to pass the averages into. Averages in deg C
    */
    void get_averages(float frame_buffer[FRAME_HEIGHT][FRAME_WIDTH]);

    /**
    * Get the temperature variances of the background pixels.
    * @param frame_buffer A 2D array to pass the variances into. Variances in deg C.
    */
    void get_variances(float frame_buffer[FRAME_HEIGHT][FRAME_WIDTH]);

    /**
    * Get the average remote temperature of the frame in deg C
    * @return Average temperature of the frame in deg C
    */
    float get_average_frame_temperature();

    /**
    * Add the current frame to the running background
    * Both mean and variance are calculated as an incremental, weighted value
    * Note: This method is different to build_background in that the averages and variances are rolling.
    *       Pixel averages and variances are weighted and averaged out of significance as new frames are added.
    *       This results in the averages and variances to be inaccurate, but 'close enough' to function in this
    * implementation.
    */
    void add_current_frame_to_background();

    ////////////////////////////////////////////////////////////////////////////////
    // Blob detection

    /**
    * Search through the current frame to find pixel 'blobs' that appear in front of the background.
    * @param blobs A Blob array to pass the detected blobs into.
    * @return Number of detected blobs
    *
    * Psuedo:
    * - Assign every active pixel to a blob
    * - Sorting ends when there are no more pixels left inside the active pixel queue
    * - Active pixels are sent to the sort queue if they are adjacent to the currently investigated pixel
    * - After a pixel is moved to the sort queue, the active pixel queue must be resorted to remove gaps (which can be
    * performed on the fly)
    * - In the case of multiple adjcent active pixels in the queue, the first free position is indexed as the free space
    * - Subsequent non-adjacent pixels will be moved to the next free space
    * - To reiterate: the active pixel queue starts off populated with active pixels
    * - The active queue depletes as pixels are sorted until there are no more pixels left
    * - The number of pixels left in the active pixels queue is managed by the num_active_pixels variable
    * - After each sweep of the queue, the active pixels are resorted to remove gaps in the array as elements are popped
    * out
    * - The sort queue is reset with each blob that is constructed
    * - The sort queue starts with zero elements and is populated using pixels from the active pixel queue
    * - Elements are not sorted or removed from the sort queue until the blob is finalised
    * - Pixels in the sort queue are added to the blob once they have finished checking for adjacency with active
    * pixels, then are not operated on again
    */
    int get_blobs(Blob blobs[]);

    /**
    * Reset a list of blobs.
    * Useful for cleaning after inspecting a frame.
    */
    void clear_blobs(Blob blobs[MAX_BLOBS]);

    /**
    * Return the active pixels in the current frame.
    * @param active Array of Pixel objects. Active pixels are added to the array.
    * @return Number of active pixels in the array.
    */
    int get_active_pixels(Pixel pixel_buffer[]);

    /**
    * Drop any blobs that are smaller than the minimum required size.
    * Must be performed after the blobs have finished building
    * @param blobs Blob array comtaining the discovered blobs from a get_blobs call
    * @param minimum_size Minimum number of pixels a blob should have to avoid the chopping block
    */
    void remove_small_blobs(Blob blobs[]);

    /**
    * Get the number of active blobs in an array
    * @param blobs Array containing the blobs. Yup. Pretty much what it says on the label...
    * @return Number of active blobs in the array.
    */
    int get_num_blobs(Blob blobs[]);

    /**
    * Get the number of active blobs in an array
    * @param blobs Array containing the blobs. Yup. Pretty much what it says on the label...
    * @return Number of active blobs in the array.
    */
    int get_num_blobs(TrackedBlob blobs[]);

    ////////////////////////////////////////////////////////////////////////////////
    // Inter-frame tracking

    /**
    * Track blobs between frames using its characteristics to match the old with the new.
    * @param new_blobs Blobs from the latest frame
    * @param tracked_blobs Tracked blobs from the previous frame
    */
    void track_blobs(Blob new_blobs[MAX_BLOBS], TrackedBlob old_tracked_blobs[MAX_BLOBS]);

    /**
    * Sort the tracked blobs to make sure there aren't any gaps.
    * Tracked blobs that have not been updated are cleared from the list
    * @param tracked_blobs List containing the tracked blobs to be sorted
    */
    void sort_tracked_blobs(TrackedBlob tracked_blobs[MAX_BLOBS]);

    /**
    * Add any remaining, new blobs, to the tracked blob list.
    * @param new_blobs Blobs from the latest frame - may contain newly discovered blobs to be tracked
    * @param tracked_blobs  A list containing the currently-tracked blobs
    */
    void add_remaining_blobs_to_tracked(Blob new_blobs[MAX_BLOBS], TrackedBlob old_tracked_blobs[MAX_BLOBS]);

    /**
    * Get the number of blobs that have been updated in the tracked blob list
    * @param tracked_blobs List containing the tracked blobs
    * @return Number of tracked blobs that have been updated
    */
    int get_num_updated_blobs(TrackedBlob tracked_blobs[MAX_BLOBS]);

    /**
    * Get the number of blobs that have not been assigned to a tracked blob
    * @param blobs A list of blobs to be tracked
    * @return Number of blobs that are not assigned to tracked blobs
    */
    int get_num_unassigned_blobs(Blob blobs[MAX_BLOBS]);

    /**
    * Update the details of previously tracked blobs if there is a similar enough to a current blob.
    * @param new_blobs Blobs from the last frame
    * @param tracked_blobs Previously tracked blobs to be updated if there are any matches
    */
    void update_tracked_blobs(Blob new_blobs[MAX_BLOBS], TrackedBlob old_tracked_blobs[MAX_BLOBS]);

    /**
    * Generate a matrix of the differences between the tracked blobs and blobs.
    * @param tracked_blobs List containing the tracked blobs
    * @param blobs List containing the new blobs from the frame
    * @param output Matrix to store the difference values
    */
    void generate_difference_matrix(TrackedBlob tracked_blobs[MAX_BLOBS], Blob blobs[MAX_BLOBS],
                                    float output[MAX_BLOBS][MAX_BLOBS]);

    /**
    * Get the index and value of the lowest difference in the difference matrix
    * @param difference_matrix Matrix containing the difference values between the different tracked blobs and normal
    * blobs.
    * @param indexes The location of the lowest difference in the matrix.
    */
    float get_lowest_difference(float difference_matrix[MAX_BLOBS][MAX_BLOBS], int indexes[2]);

    /**
    * Remove a row and column from the difference matrix so it cannot be used in matching up blobs with tracked blobs.
    * The entire row and column are marked with a difference of 999, which is far above the maximum difference
    * threshold.
    * You'd use this function after a match has been found between a blob and a tracked blob.
    * @param row The row number (tracked blob index) to remove from the difference matrix
    * @param col The coloumn number (blob index) to remove from the difference matrix
    * @param difference_matrix A 2D matrix containing the combination of difference between tracked blobs and blobs
    */
    void remove_difference_row_col(int row, int col, float difference_matrix[MAX_BLOBS][MAX_BLOBS]);

    /**
    * Check if a dying tracked blob has travelled far enough to register a movement.
    * If a tracked blob travels over the the net minimum travel threshold.
    * @param blob Tracked blob to be processed. Contains the travel information.
    */
    void process_blob_movements(TrackedBlob blob);

    /**
    * Increment the movement of the specified direction
    * @param direction The direction to increment movements to
    */
    void add_movement(int direction);

    /**
    * Get the list of movements recorded by the tracker
    * Reading the movements clears the movement_changed_since_last_check flag.
    * Movements are recorded in the following order:
    * {left, right, up, down, no_direction}
    *
    * @param _movements List containing the total movements recorded by the tracker
    */
    void get_movements(long _movements[NUM_DIRECTION_CATEGORIES]);

    /**
    * Determine if there have been any new movements since the last check.
    * Grabbing the movements will clear the flag
    * @return True if there have been any new movements
    */
    bool has_new_movements();

    /**
    * Reset the movement list back to zeroes
    */
    void reset_movements();

    /**
    * Set the callback for when tracking starts on a new blob
    * Call without parameters to clear the callback
    *
    * @param callback Function to call when tracking starts (tracked blob object is passed)
    */
    void set_tracking_start_callback(tracked_callback callback = NULL);

    /**
    * Set the callback for when tracking ends on a previously tracked blob
    * Call without parameters to clear the callback
    *
    * @param callback Function to call when tracking ends (tracked blob object is passed)
    */
    void set_tracking_end_callback(tracked_callback callback = NULL);

    ////////////////////////////////////////////////////////////////////////////////
    // Variables

    TrackedBlob tracked_blobs[MAX_BLOBS];
    tracked_callback tracking_start_callback;
    tracked_callback tracking_end_callback;

    // Running configuration
    int running_average_size;
    int min_blob_size;
    int minimum_travel_threshold;
    int max_difference_threshold;
    float minimum_temperature_differential;
    float active_pixel_variance_scalar;
    int max_dead_frames;

    // Runtime variables
    int num_background_frames;
    float frame[FRAME_HEIGHT][FRAME_WIDTH];
    float pixel_averages[FRAME_HEIGHT][FRAME_WIDTH];
    float pixel_variance[FRAME_HEIGHT][FRAME_WIDTH];
    long movements[5];

    int num_unchanged_frames;
    int num_last_blobs;
    bool movement_changed_since_last_check;

   private:
    /**
    * Load an input frame into the buffer.
    * @param frame_buffer A 2D array containing the pixel temperatures to be added to the buffer
    */
    void load_frame(float frame_buffer[FRAME_HEIGHT][FRAME_WIDTH]);

    /**
    * Add the currently-loaded frame to the background.
    * The background forms the basis for determining which pixels have changed to indicate movement.
    * Note: This function is only meant to be run once before tracking begins.
    *       Frames are still added to the background after tracking begins using the add_frame_to_to_running_background
    * function.
    *       add_frame_to_to_running_background uses a running average and variance to operate whereas this function uses
    * a fixed population size.
    */
    void build_background();
};
