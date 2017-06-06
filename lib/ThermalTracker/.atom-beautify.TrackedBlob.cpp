#include "TrackedBlob.h"

float absolute(float f) {
    if (f < 0.0) {
        f *= -1.0;
    }
    return f;
}

float TrackedBlob::position_penalty = 0;
float TrackedBlob::area_penalty = 0;
float TrackedBlob::aspect_ratio_penalty = 0;
float TrackedBlob::temperature_penalty = 0;
float TrackedBlob::direction_penalty = 0;
int TrackedBlob::frame_width = 16;

////////////////////////////////////////////////////////////////////////////////
// Constructor

TrackedBlob::TrackedBlob() {
    /**
    * Create a new tracked blob.
    * Track blobs start off empty until they are initialised by a regular blob using the set method.
    * After that, you can update the blob that is being tracked to record its movements between frames (and thus
    * updates)
    */

    clear();
}

////////////////////////////////////////////////////////////////////////////////
// Public Methods

void TrackedBlob::clear() {
    /**
    * Clear the tracked blob's characteristics
    * The tracked blob object will be marked inactive until the blob is reinitialised.
    * Tracking information is also lost by this action.
    */
    _blob.clear();

    predicted_position[X] = -1;
    predicted_position[Y] = -1;
    travel[X] = 0;
    travel[Y] = 0;
    max_size = 0;
    reset_updated_status();
}

void TrackedBlob::set(Blob blob) {
    /**
    * Start tracking a new blob.
    * All previous tracking data (if any existed) is lost by this action.
    * @param blob Blob to start tracking
    */
    clear();
    copy_blob(blob);
    has_updated = true;
    start_pos[0] = blob.centroid[0];
    start_pos[1] = blob.centroid[1];
    times_updated = 0;
    max_size = blob.get_size();
    start_time = millis();
}

bool TrackedBlob::is_active() {
    /**
    * Determine if the tracked blob is actually tracking anything
    * @return True if a blob is being tracked
    */
    return _blob.is_active();
}

void TrackedBlob::update_blob(Blob blob) {
    /**
    * Update the tracked blob
    * Movements between the old and new blob states are recorded.
    * The next predicted position of the next blob is also calculated for future calculations
    * @param blob New blob state used to update the tracked blob
    */
    float movement[2];

    movement[X] = blob.centroid[X] - _blob.centroid[X];
    movement[Y] = blob.centroid[Y] - _blob.centroid[Y];

    predicted_position[X] = blob.centroid[X] + movement[X];
    predicted_position[Y] = blob.centroid[Y] + movement[Y];

    travel[X] += movement[X];
    travel[Y] += movement[Y];

    copy_blob(blob);

    event_duration = millis() - start_time;
    has_updated = true;
    times_updated++;

    int size = blob.get_size();
    if (size > max_size) {
        max_size = size;
    }
}

void TrackedBlob::reset_updated_status() {
    /**
    * Reset the updated status of the tracked blob.
    * Tracked blobs that have not updated will be purged at the end of frame processing.
    */
    has_updated = false;
}

void TrackedBlob::copy(TrackedBlob tblob) {
    /**
    * Overwrite the tracked blob with the information from another tracked blob.
    * Useful for shuffling tracked blobs around in arrays
    * Information from the source blob completely overwrites any currently-held information
    * @param tblob Tracked blob to copy information from
    */
    copy_blob(tblob._blob);

    predicted_position[X] = tblob.predicted_position[X];
    predicted_position[Y] = tblob.predicted_position[Y];
    travel[Y] = tblob.travel[Y];
    travel[X] = tblob.travel[X];
    has_updated = tblob.has_updated;
}

float TrackedBlob::get_travel(int axis) {
    /**
    * Get the net travel difference of the tracked blob as it moves between frames
    * @param axis The axis of travel to get the difference for
    * @return The net number of pixels the blob has moved from its original position in the specified axis
    */
    float _travel = 0;

    if (axis == X) {
        _travel = travel[X];
    } else {
        _travel = travel[Y];
    }

    return _travel;
}

float TrackedBlob::get_difference(Blob other_blob) {
    /**
    * Find out how 'different' the tracked blob is from another blob; not just how far away the blob is...
    * A low difference score between blobs means they are very similar
    * This function is used on blobs between frames to determine if the blobs originate from the same object
    * @param other_blob The second blob in the calculations. The difference factor will be between this blob and the
    * tracked blob.
    * @return The difference score between the two blobs. Unitless.
    */

    float difference_factor = 0.0;
    float position = _blob.centroid[X];
    float edge_penalty = (1 - absolute((frame_width / 2 - position)) / (frame_width / 2));

    difference_factor += calculate_position_difference(other_blob);
    difference_factor += calculate_area_difference(other_blob);
    difference_factor += calculate_aspect_ratio_difference(other_blob);

    // Soften the difference if the blob is touching the sides of the frame
    if (is_touching_side()) {
        difference_factor *= edge_penalty;
    }

    difference_factor += calculate_temperature_difference(other_blob);
    difference_factor += calculate_direction_difference(other_blob);

    return difference_factor;
}

void TrackedBlob::copy_blob(Blob blob) {
    /**
    * Copy the details from a given blob into the tracked blob.
    * @param blob Source blob to copy data from.
    */
    _blob.centroid[X] = blob.centroid[X];
    _blob.centroid[Y] = blob.centroid[Y];
    _blob.min[X] = blob.min[X];
    _blob.min[Y] = blob.min[Y];
    _blob.max[X] = blob.max[X];
    _blob.max[Y] = blob.max[Y];
    _blob.aspect_ratio = blob.aspect_ratio;
    _blob.average_temperature = blob.average_temperature;
    _blob.width = blob.width;
    _blob.height = blob.height;
    _blob.num_pixels = blob.num_pixels;
}

float TrackedBlob::calculate_position_difference(Blob other_blob) {
    float difference_factor = 0;
    if (predicted_position[X] >= 0 && predicted_position[Y] >= 0) {
        difference_factor += absolute(predicted_position[X] - other_blob.centroid[X]) * position_penalty;
        difference_factor += absolute(predicted_position[Y] - other_blob.centroid[Y]) * position_penalty;
    } else {
        difference_factor += absolute(_blob.centroid[X] - other_blob.centroid[X]) * position_penalty;
        difference_factor += absolute(_blob.centroid[Y] - other_blob.centroid[Y]) * position_penalty;
    }
    return difference_factor;
}

float TrackedBlob::calculate_area_difference(Blob other_blob) {
    float difference = absolute(_blob.num_pixels - other_blob.num_pixels) * area_penalty;
    return difference;
}

float TrackedBlob::calculate_temperature_difference(Blob other_blob) {
    return absolute(_blob.average_temperature - other_blob.average_temperature) * temperature_penalty;
}

float TrackedBlob::calculate_aspect_ratio_difference(Blob other_blob) {
    absolute(_blob.aspect_ratio - other_blob.aspect_ratio) * aspect_ratio_penalty;
}

float TrackedBlob::calculate_direction_difference(Blob other_blob) {
    /**
    * Calculate the penalty for any changes in the blobs direction of travel
    * This penalty is binary.
    * If the blob is moving in the same direction as before, no penalty is applied.
    * If the movement is different from the net travel direction of the blob, the penalty is added.
    * @return the penalty as a result of differences in travel direction
    */
    float difference = 0;

    // Establish current direction
    int latest_direction = predicted_position[X] - _blob.centroid[X];

    // Check if that direction matches the overall travel of the blob
    if ((latest_direction > 0) != (travel > 0)) {
        difference += direction_penalty;
    }

    return difference;
}

bool TrackedBlob::is_touching_side() {
    /**
    * Determine if the blob is touching the side of the frame
    * @return True if the blob is likely touching a vertical side of the frame
    */
    bool is_touching = false;

    if ((_blob.centroid[X] - _blob.width / 2) <= 1) {
        is_touching = true;
    } else if ((_blob.centroid[X] + _blob.width / 2) <= (frame_width - 1)) {
        is_touching = true;
    }

    return is_touching;
}
