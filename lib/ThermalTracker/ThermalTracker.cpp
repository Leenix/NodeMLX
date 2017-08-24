#include "ThermalTracker.h"

////////////////////////////////////////////////////////////////////////////////
// Constructor

ThermalTracker::ThermalTracker() {
    /**
    * Constructor - Make a new thermal tracker object with the default configuration
    * The thermal tracker uses a MLX90621 thermopile array to observe moving objects in its view.
    */

    num_background_frames = 0;
    min_blob_size = DEFAULT_MIN_BLOB_SIZE;
    running_average_size = DEFAULT_RUNNING_AVERAGE_SIZE;
    minimum_travel_threshold = DEFAULT_MIN_TRAVEL_THRESHOLD;
    max_difference_threshold = DEFAULT_MAX_DIFFERENCE_THRESHOLD;
    minimum_temperature_differential = DEFAULT_MIN_TEMPERATURE_DIFFERENTIAL;
    active_pixel_variance_scalar = DEFAULT_ACTIVE_PIXEL_VARIANCE_SCALAR;
    max_dead_frames = DEFAULT_MAX_DEAD_FRAMES;

    TrackedBlob::position_penalty = DEFAULT_POSITION_PENALTY;
    TrackedBlob::area_penalty = DEFAULT_AREA_PENALTY;
    TrackedBlob::aspect_ratio_penalty = DEFAULT_ASPECT_RATIO_PENALTY;
    TrackedBlob::direction_penalty = DEFAULT_DIRECTION_PENALTY;
    TrackedBlob::temperature_penalty = DEFAULT_TEMPERATURE_PENALTY;

    Pixel::adjacency_fuzz = DEFAULT_ADJACENCY_FUZZ;
}

////////////////////////////////////////////////////////////////////////////////
// Initialisation

void ThermalTracker::update(float frame_buffer[FRAME_HEIGHT][FRAME_WIDTH]) {
    /**
    * Process an input thermal frame.
    * If a background has not yet been established, the frame goes directly to the background without tracking.
    * If the background has already been built, then the frame is analysed to detect and track movement.
    * @float frame_buffer A 2D array containing the pixel temperatures from the thermopile sensor.
    */

    load_frame(frame_buffer);

    // Has the background been built first? If not; build it!
    if (!is_background_finished()) {
        build_background();
    }

    // Background already built; go track all the things!
    else {
        bool add_frame_to_average = true;
        Blob blobs[MAX_BLOBS];

        get_blobs(blobs);
        remove_small_blobs(blobs);
        int num_blobs = get_num_blobs(blobs);

        // Activity check - don't add frames to background when there is activity
        // There is a limit to this though if the in-frame blobs stay the same for a certain amount of time (default 4
        // seconds)
        if (num_blobs > 0) {
            add_frame_to_average = false;

            num_unchanged_frames++;

            if (num_unchanged_frames > UNCHANGED_FRAME_DELAY) {
                add_frame_to_average = true;
            }
        } else {
            num_unchanged_frames = 0;
        }

        num_last_blobs = num_blobs;
        track_blobs(blobs, tracked_blobs);

        if (add_frame_to_average) {
            add_current_frame_to_background();
        }
    }
}

bool ThermalTracker::is_background_finished() {
    /**
    * Determine if the tracker has finished build its background frames.
    * @return True if the tracker has gathered the minumum number of frames.
    */
    return (num_background_frames >= running_average_size);
}

void ThermalTracker::reset_background() {
    /**
    * Reset the number of frames in the running background, forcing the tracker to recreate.
    */
    num_background_frames = 0;
}

void ThermalTracker::load_frame(float frame_buffer[FRAME_HEIGHT][FRAME_WIDTH]) {
    /**
    * Load an input frame into the buffer.
    * @param frame_buffer A 2D array containing the pixel temperatures to be added to the buffer
    */
    for (int i = 0; i < FRAME_HEIGHT; i++) {
        for (int j = 0; j < FRAME_WIDTH; j++) {
            frame[i][j] = frame_buffer[i][j];
        }
    }
}

void ThermalTracker::build_background() {
    /**
    * Add the currently-loaded frame to the background.
    * The background forms the basis for determining which pixels have changed to indicate movement.
    * Note: This function is only meant to be run once before tracking begins.
    *       Frames are still added to the background after tracking begins using the add_current_frame_to_background
    * function.
    *       add_current_frame_to_background uses a running average and variance to operate whereas this function uses
    * a fixed population size.
    */

    if (num_background_frames == 0) {
        for (int i = 0; i < FRAME_HEIGHT; i++) {
            for (int j = 0; j < FRAME_WIDTH; j++) {
                pixel_averages[i][j] = frame[i][j];
                pixel_variance[i][j] = 0;
            }
        }
    }

    else {
        // Mean the frames together to form the background and calculate variance
        for (int i = 0; i < FRAME_HEIGHT; i++) {
            for (int j = 0; j < FRAME_WIDTH; j++) {
                float temp = frame[i][j];
                float last_average = pixel_averages[i][j];

                pixel_averages[i][j] += (temp - last_average) / (num_background_frames + 1);
                pixel_variance[i][j] += (temp - pixel_averages[i][j]) * (temp - last_average);
            }
        }
    }

    num_background_frames++;

    // Calculate the standard deviation of the frames when the background has finished building
    if (num_background_frames == running_average_size) {
        // Calculate standard deviation using Welford's Method
        // See: https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
        // Also: http://jonisalonen.com/2013/deriving-welfords-method-for-computing-variance/
        for (int i = 0; i < FRAME_HEIGHT; i++) {
            for (int j = 0; j < FRAME_WIDTH; j++) {
                pixel_variance[i][j] = sqrtf(pixel_variance[i][j] / (num_background_frames - 1));
            }
        }
    }
}

void ThermalTracker::add_current_frame_to_background() {
    /**
    * Add the current frame to the running background
    * Both mean and variance are calculated as an incremental, weighted value
    * Note: This method is different to build_background in that the averages and variances are rolling.
    *       Pixel averages and variances are weighted and averaged out of significance as new frames are added.
    *       This results in the averages and variances to be inaccurate, but 'close enough' to function in this
    * implementation.
    */
    for (int i = 0; i < FRAME_HEIGHT; i++) {
        for (int j = 0; j < FRAME_WIDTH; j++) {
            float temp = frame[i][j];

            // Add the weighted average
            pixel_averages[i][j] = ((pixel_averages[i][j] * (running_average_size - 1)) + temp) / running_average_size;

            // Add the weighted variance
            float incremental_variance = absolute(temp - pixel_averages[i][j]);
            pixel_variance[i][j] =
                ((pixel_variance[i][j] * (running_average_size - 1)) + incremental_variance) / running_average_size;
        }
    }
}

void ThermalTracker::get_averages(float frame_buffer[FRAME_HEIGHT][FRAME_WIDTH]) {
    /**
    * Get the average temperatures of the background pixels.
    * @param frame_buffer A 2D array to pass the averages into. Averages in deg C
    */
    for (int i = 0; i < FRAME_HEIGHT; i++) {
        for (int j = 0; j < FRAME_WIDTH; j++) {
            frame_buffer[i][j] = pixel_averages[i][j];
        }
    }
}

void ThermalTracker::get_variances(float frame_buffer[FRAME_HEIGHT][FRAME_WIDTH]) {
    /**
    * Get the temperature variances of the background pixels.
    * @param frame_buffer A 2D array to pass the variances into. Variances in deg C.
    */
    for (int i = 0; i < FRAME_HEIGHT; i++) {
        for (int j = 0; j < FRAME_WIDTH; j++) {
            frame_buffer[i][j] = pixel_variance[i][j];
        }
    }
}

float ThermalTracker::get_average_frame_temperature() {
    /**
    * Get the average remote temperature of the frame in deg C
    * @return Average temperature of the frame in deg C
    */

    float total_temperature = 0;

    for (int i = 0; i < FRAME_HEIGHT; i++) {
        for (int j = 0; j < FRAME_WIDTH; j++) {
            total_temperature += pixel_averages[i][j];
        }
    }

    return total_temperature / (FRAME_HEIGHT * FRAME_WIDTH);
}

////////////////////////////////////////////////////////////////////////////////
// Blob detection

int ThermalTracker::get_blobs(Blob blobs[]) {
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

    Pixel active_pixels[FRAME_WIDTH * FRAME_HEIGHT];
    int num_blobs = 0;
    int vacant_index = FRAME_WIDTH * FRAME_HEIGHT + 1;
    clear_blobs(blobs);

    int num_active_pixels = get_active_pixels(active_pixels);

    // Assign every active pixel to a blob
    while ((num_active_pixels > 0) && (num_blobs < MAX_BLOBS)) {
        int num_queued_pixels = 0;
        int queue_index = 0;
        vacant_index = 0;
        Pixel sort_queue[FRAME_WIDTH * FRAME_HEIGHT];
        sort_queue[num_queued_pixels++].set(active_pixels[vacant_index].get_x(), active_pixels[vacant_index].get_y(),
                                            active_pixels[vacant_index].get_temperature());
        bool first_pixel = true;

        // Construct the current blob
        while (queue_index < num_queued_pixels) {
            // Find adjacent active pixels in the queue
            for (int i = 0; i < num_active_pixels; i++) {
                if (first_pixel) {
                    i++;
                    first_pixel = false;
                }

                // If the pixel is adjacent to the current pixel, add it to the sort queue
                if (sort_queue[queue_index].is_adjacent(active_pixels[i])) {
                    sort_queue[num_queued_pixels++].set(active_pixels[i].get_x(), active_pixels[i].get_y(),
                                                        active_pixels[i].get_temperature());
                }

                else {
                    if (vacant_index < i) {
                        // Sort the pixel to the front of the queue
                        active_pixels[vacant_index].set(active_pixels[i].get_x(), active_pixels[i].get_y(),
                                                        active_pixels[i].get_temperature());
                    }
                    vacant_index++;
                }
            }

            // Reset the number of active pixels in the queue
            // Note: This needs to be changed at the end of each search cycle; not during
            num_active_pixels = vacant_index;
            vacant_index = 0;

            // Searched finished; add the current pixel to the blob
            blobs[num_blobs].add_pixel(sort_queue[queue_index++]);
        }

        // Blob finished; add it to the current blobs and start on the next one
        num_blobs++;
    }

    return num_blobs;
}

void ThermalTracker::clear_blobs(Blob blobs[MAX_BLOBS]) {
    /**
    * Reset a list of blobs.
    * Useful for cleaning after inspecting a frame.
    */
    for (int i = 0; i < MAX_BLOBS; i++) {
        blobs[i].clear();
    }
}

int ThermalTracker::get_active_pixels(Pixel pixel_buffer[]) {
    /**
    * Return the active pixels in the current frame.
    * @param active Array of Pixel objects. Active pixels are added to the array.
    * @return Number of active pixels in the array.
    */
    int num_active = 0;

    for (int i = 0; i < FRAME_HEIGHT; i++) {
        for (int j = 0; j < FRAME_WIDTH; j++) {
            float temp = frame[i][j];
            float average = pixel_averages[i][j];
            float variance = pixel_variance[i][j];

            float temperature_difference = absolute(average - temp);

            if (temperature_difference > (variance * active_pixel_variance_scalar) &&
                temperature_difference > minimum_temperature_differential) {
                pixel_buffer[num_active++].set(j, i, temp);
            }
        }
    }

    return num_active;
}

void ThermalTracker::remove_small_blobs(Blob blobs[MAX_BLOBS]) {
    /**
    * Drop any blobs that are smaller than the minimum required size.
    * Must be performed after the blobs have finished building
    * @param blobs Blob array comtaining the discovered blobs from a get_blobs call
    * @param minimum_size Minimum number of pixels a blob should have to avoid the chopping block
    */
    int vacant_index = MAX_BLOBS + 1;

    // Pass over the blob array and pop the small ones
    for (int i = 0; i < MAX_BLOBS; i++) {
        if (blobs[i].get_size() < min_blob_size) {
            // Blob too smol; pop it out
            blobs[i].clear();

            if (i < vacant_index) {
                vacant_index = i;
            }
        } else {
            // Blob is big enough; make sure there are no gaps
            if (i > vacant_index) {
                blobs[vacant_index++].copy(blobs[i]);
                blobs[i].clear();
            }
        }
    }
}

int ThermalTracker::get_num_blobs(Blob blobs[MAX_BLOBS]) {
    /**
    * Get the number of active blobs in an array
    * @param blobs Array containing the blobs. Yup. Pretty much what it says on the label...
    * @return Number of active blobs in the array.
    */
    int num_blobs = 0;
    for (int i = 0; i < MAX_BLOBS; i++) {
        if (blobs[i].is_active()) {
            num_blobs++;
        }
    }

    return num_blobs;
}

int ThermalTracker::get_num_blobs(TrackedBlob blobs[MAX_BLOBS]) {
    /**
    * Get the number of active blobs in an array
    * @param blobs Array containing the blobs. Yup. Pretty much what it says on the label...
    * @return Number of active blobs in the array.
    */
    int num_blobs = 0;
    for (int i = 0; i < MAX_BLOBS; i++) {
        if (blobs[i].is_active()) {
            num_blobs++;
        }
    }

    return num_blobs;
}

////////////////////////////////////////////////////////////////////////////////
// Inter-frame tracking

void ThermalTracker::track_blobs(Blob new_blobs[], TrackedBlob tracked_blobs[]) {
    /**
    * Track blobs between frames using its characteristics to match the old with the new.
    * @param new_blobs Blobs from the latest frame
    * @param tracked_blobs Tracked blobs from the previous frame
    */

    int num_tracked_blobs = get_num_blobs(tracked_blobs);

    // Update any existing blobs
    if (num_tracked_blobs > 0) {
        update_tracked_blobs(new_blobs, tracked_blobs);
        sort_tracked_blobs(tracked_blobs);
    }

    // All unassigned blobs get added to the track list
    add_remaining_blobs_to_tracked(new_blobs, tracked_blobs);
}

void ThermalTracker::update_tracked_blobs(Blob new_blobs[MAX_BLOBS], TrackedBlob tracked_blobs[MAX_BLOBS]) {
    /**
    * Update the details of previously tracked blobs if there is a similar enough to a current blob.
    * @param new_blobs Blobs from the last frame
    * @param tracked_blobs Previously tracked blobs to be updated if there are any matches
    */
    for (int i = 0; i < MAX_BLOBS; i++) {
        tracked_blobs[i].reset_updated_status();
        new_blobs[i].clear_assigned();
    }

    // Create a difference matrix to show which blobs are likely the same across frames (lower difference == more likely
    // the
    // same)
    float difference_matrix[MAX_BLOBS][MAX_BLOBS]; /**< Stores the difference values between every tracked_blob/new_blob
                                                    combination */
    int indexes[2];
    generate_difference_matrix(tracked_blobs, new_blobs, difference_matrix);

    // Keep going until there are no more matches
    float difference = get_lowest_difference(difference_matrix, indexes);
    while (difference < max_difference_threshold) {
        // Update the tracked blob
        tracked_blobs[indexes[0]].update_blob(new_blobs[indexes[1]]);

        // Remove the matched up rows and columns from the difference matrix so they cannot be matched again
        remove_difference_row_col(indexes[0], indexes[1], difference_matrix);
        new_blobs[indexes[1]].set_assigned();

        // Find the next lowest difference
        difference = get_lowest_difference(difference_matrix, indexes);
    }
}

void ThermalTracker::sort_tracked_blobs(TrackedBlob tracked_blobs[]) {
    /**
    * Sort the tracked blobs to make sure there aren't any gaps.
    * Tracked blobs that have not been updated are cleared from the list
    * @param tracked_blobs List containing the tracked blobs to be sorted
    */
    // Clean up at the end - Remove gaps in the tracked blobs table and process/clear old and un-updated tracked blobs
    int free_index = MAX_BLOBS + 1;
    for (int i = 0; i < MAX_BLOBS; i++) {
        // If the blob hasn't been updated this frame, it might be dead
        if (!tracked_blobs[i].has_updated) {
            tracked_blobs[i].num_dead_frames++;
        }

        if (tracked_blobs[i].has_updated || tracked_blobs[i].num_dead_frames < max_dead_frames) {
            // Tracked blob has updated
            // Keep it in the list, but move it up if there are gaps
            if (free_index < i) {
                tracked_blobs[free_index++].copy(tracked_blobs[i]);
                tracked_blobs[i].clear();
            }
        }

        else {
            // Tracked blob not updated
            // If it was active, process the movement
            if (tracked_blobs[i].is_active()) {
                process_blob_movements(tracked_blobs[i]);
            }

            // Either way, clear it afterwards to make room for the real ones
            tracked_blobs[i].clear();
            if (free_index > i) {
                free_index = i;
            }
        }
    }
}

void ThermalTracker::generate_difference_matrix(TrackedBlob tracked_blobs[MAX_BLOBS], Blob blobs[MAX_BLOBS],
                                                float output[MAX_BLOBS][MAX_BLOBS]) {
    /**
    * Generate a matrix of the differences between the tracked blobs and blobs.
    * @param tracked_blobs List containing the tracked blobs
    * @param blobs List containing the new blobs from the frame
    * @param output Matrix to store the difference values
    */

    for (int i = 0; i < MAX_BLOBS; i++) {
        for (int j = 0; j < MAX_BLOBS; j++) {
            if (blobs[j].is_active() && tracked_blobs[i].is_active()) {
                output[i][j] = tracked_blobs[i].get_difference(blobs[j]);
            } else {
                output[i][j] = max_difference_threshold;
            }
        }
    }
}

float ThermalTracker::get_lowest_difference(float difference_matrix[MAX_BLOBS][MAX_BLOBS], int indexes[2]) {
    /**
    * Get the index and value of the lowest difference in the difference matrix
    * @param difference_matrix Matrix containing the difference values between the different tracked blobs and normal
    * blobs.
    * @param indexes The location of the lowest difference in the matrix.
    */
    float lowest = max_difference_threshold;
    int x_index = -1;
    int y_index = -1;

    // Find the value and index of the lowest value in the matrix
    for (int i = 0; i < MAX_BLOBS; i++) {
        for (int j = 0; j < MAX_BLOBS; j++) {
            float difference = difference_matrix[i][j];
            if (difference < lowest && difference < max_difference_threshold) {
                lowest = difference;
                x_index = i;
                y_index = j;
            }
        }
    }

    indexes[0] = x_index;
    indexes[1] = y_index;
    return lowest;
}

void ThermalTracker::remove_difference_row_col(int row, int col, float difference_matrix[MAX_BLOBS][MAX_BLOBS]) {
    /**
    * Remove a row and column from the difference matrix so it cannot be used in matching up blobs with tracked blobs.
    * The entire row and column are marked with a difference of max_difference_threshold, which is far above the maximum
    * difference
    * threshold.
    * You'd use this function after a match has been found between a blob and a tracked blob.
    * @param row The row number (tracked blob index) to remove from the difference matrix
    * @param col The coloumn number (blob index) to remove from the difference matrix
    * @param difference_matrix A 2D matrix containing the combination of difference between tracked blobs and blobs
    */
    for (int i = 0; i < MAX_BLOBS; i++) {
        difference_matrix[row][i] = max_difference_threshold;
        difference_matrix[i][col] = max_difference_threshold;
    }
}

void ThermalTracker::add_remaining_blobs_to_tracked(Blob new_blobs[MAX_BLOBS], TrackedBlob tracked_blobs[MAX_BLOBS]) {
    /**
    * Add any remaining, new blobs, to the tracked blob list.
    * @param new_blobs Blobs from the latest frame - may contain newly discovered blobs to be tracked
    * @param tracked_blobs  A list containing the currently-tracked blobs
    */
    int num_updated_blobs = get_num_updated_blobs(tracked_blobs);
    int num_new_blobs = get_num_blobs(new_blobs);
    int num_unassigned_blobs = get_num_unassigned_blobs(new_blobs);

    static int track_id = 0;

    // AN: Can validate numbers here: num_updated_blobs == (new_blobs - num_unassigned_blobs)

    if (num_unassigned_blobs > 0) {
        // Make sure the latest tracking weights are up to date

        int i = 0;
        while (num_unassigned_blobs > 0 && i < MAX_BLOBS) {
            if (new_blobs[i].is_active() && !new_blobs[i].is_assigned()) {
                tracked_blobs[num_updated_blobs].set(new_blobs[i], track_id++);
                new_blobs[i].set_assigned();  // Probably not necessary...
                num_unassigned_blobs--;

                // New tracking event. Do the callback if it exists
                if (tracking_start_callback) {
                    (*tracking_start_callback)(tracked_blobs[num_updated_blobs]);
                }

                num_updated_blobs++;
            }
            i++;
        }
    }
}

int ThermalTracker::get_num_updated_blobs(TrackedBlob tracked_blobs[MAX_BLOBS]) {
    /**
    * Get the number of blobs that have been updated in the tracked blob list
    * @param tracked_blobs List containing the tracked blobs
    * @return Number of tracked blobs that have been updated
    */
    int num_updated = 0;
    for (int i = 0; i < MAX_BLOBS; i++) {
        if (tracked_blobs[i].has_updated) {
            num_updated++;
        }
    }

    return num_updated;
}

int ThermalTracker::get_num_unassigned_blobs(Blob blobs[MAX_BLOBS]) {
    /**
    * Get the number of blobs that have not been assigned to a tracked blob
    * @param blobs A list of blobs to be tracked
    * @return Number of blobs that are not assigned to tracked blobs
    */
    int num_unassigned = 0;
    for (int i = 0; i < MAX_BLOBS; i++) {
        if (blobs[i].is_active() && !blobs[i].is_assigned()) {
            num_unassigned++;
        }
    }

    return num_unassigned;
}

void ThermalTracker::process_blob_movements(TrackedBlob blob) {
    /**
    * Check if a dying tracked blob has travelled far enough to register a movement.
    * If a tracked blob travels over the the net minimum travel threshold.
    * @param blob Tracked blob to be processed. Contains the travel information.
    */

    bool movement_added = false;

    // Check for horizontal movement
    if (abs(blob.get_travel(X)) > minimum_travel_threshold) {
        movement_added = true;
        if (blob.get_travel(X) < 0) {
            add_movement(LEFT);
        } else {
            add_movement(RIGHT);
        }
    }

    // Check for vertical movement
    if (abs(blob.get_travel(Y)) > minimum_travel_threshold) {
        movement_added = true;
        if (blob.get_travel(Y) > 0) {
            add_movement(UP);
        } else {
            add_movement(DOWN);
        }
    }

    // No direction! Thing disappeared in a single frame or stopped moving. Cheeky shit.
    if (!movement_added) {
        add_movement(NO_DIRECTION);
    }

    if (tracking_end_callback) {
        (*tracking_end_callback)(blob);
    }

    blob.id = 0;
}

void ThermalTracker::add_movement(int direction) {
    /**
    * Increment the movement of the specified direction
    * @param direction The direction to increment movements to
    */
    direction = constrain(direction, 0, 4);
    movements[direction]++;
    movement_changed_since_last_check = true;
}

void ThermalTracker::get_movements(long _movements[NUM_DIRECTION_CATEGORIES]) {
    /**
    * Get the list of movements recorded by the tracker
    * Reading the movements clears the movement_changed_since_last_check flag.
    * Movements are recorded in the following order:
    * {left, right, up, down, no_direction}
    *
    * @param _movements List containing the total movements recorded by the tracker
    */
    for (int i = 0; i < 5; i++) {
        _movements[i] = movements[i];
    }
    movement_changed_since_last_check = false;
}

bool ThermalTracker::has_new_movements() {
    /**
    * Determine if there have been any new movements since the last check.
    * Grabbing the movements will clear the flag
    * @return True if there have been any new movements
    */
    return movement_changed_since_last_check;
}

void ThermalTracker::reset_movements() {
    /**
    * Reset the movement list back to zeroes
    */
    for (int i = 0; i < 5; i++) {
        movements[i] = 0;
    }
}

////////////////////////////////////////////////////////////////////////////////
// Callbacks

void ThermalTracker::set_tracking_start_callback(tracked_callback callback) {
    /**
    * Set the callback for when tracking starts on a new blob
    * Call without parameters to clear the callback
    *
    * @param callback Function to call when tracking starts (tracked blob object is passed)
    */
    if (callback) {
        tracking_start_callback = callback;
    } else {
        tracking_start_callback = NULL;
    }
}

void ThermalTracker::set_tracking_end_callback(tracked_callback callback) {
    /**
    * Set the callback for when tracking ends on a previously tracked blob
    * Call without parameters to clear the callback
    *
    * @param callback Function to call when tracking ends (tracked blob object is passed)
    */
    if (callback) {
        tracking_end_callback = callback;
    } else {
        tracking_end_callback = NULL;
    }
}
