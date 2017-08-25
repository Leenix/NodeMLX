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
float TrackedBlob::dead_frame_penalty = 0;

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
    total_travel[X] = 0;
    total_travel[Y] = 0;
    max_size = 0;
    max_difference = 0;
    average_difference = 0;
    max_width = 0;
    max_height = 0;
    num_dead_frames = 0;
    average_position_difference = 0;
    average_aspect_ratio_difference = 0;
    average_area_difference = 0;
    average_direction_difference = 0;
    average_temperature_difference = 0;
    max_num_dead_frames = 0;

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
    max_width = blob.width;
    max_height = blob.height;
}
void TrackedBlob::set(Blob blob, unsigned int _id) {
    id = _id;
    set(blob);
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

    event_duration = millis() - start_time;
    update_differences(blob);

    update_movements(blob);
    copy_blob(blob);
    update_geometry(blob);

    has_updated = true;
    if (num_dead_frames > max_num_dead_frames) {
        max_num_dead_frames = num_dead_frames;
    }
    num_dead_frames = 0;
    times_updated++;
}

void TrackedBlob::update_movements(Blob blob) {
    /**
    * Update the movement and travel variables from the last blob update
    */
    float movement[2];

    movement[X] = blob.centroid[X] - _blob.centroid[X];
    movement[Y] = blob.centroid[Y] - _blob.centroid[Y];

    predicted_position[X] = blob.centroid[X] + movement[X];
    predicted_position[Y] = blob.centroid[Y] + movement[Y];

    travel[X] += movement[X];
    travel[Y] += movement[Y];

    total_travel[X] += abs(movement[X]);
    total_travel[Y] += abs(movement[Y]);
}

void TrackedBlob::update_geometry(Blob blob) {
    /**
    * Update the geometry variables that need to change from the last blob update
    */
    int size = blob.get_size();
    if (size > max_size) {
        max_size = size;
    }

    if (blob.width > max_width) {
        max_width = blob.width;
    }

    if (blob.height > max_height) {
        max_height = blob.height;
    }
}

void TrackedBlob::update_differences(Blob blob) {
    /**
    * Update the difference factors from the last blob update
    * @return None
    */
    float difference = get_difference(blob);

    // Calculate average difference
    average_difference *= times_updated;
    average_difference += difference;
    average_difference /= (times_updated + 1);

    if (difference > max_difference) {
        max_difference = difference;
    }

    average_area_difference = (average_area_difference * times_updated + area_difference) / (times_updated + 1);

    average_position_difference =
        (average_position_difference * times_updated + position_difference) / (times_updated + 1);

    average_aspect_ratio_difference =
        (average_aspect_ratio_difference * times_updated + aspect_ratio_difference) / (times_updated + 1);

    average_direction_difference =
        (average_direction_difference * times_updated + direction_difference) / (times_updated + 1);

    average_temperature_difference =
        (average_temperature_difference * times_updated + temperature_difference) / (times_updated + 1);
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

    id = tblob.id;
    predicted_position[X] = tblob.predicted_position[X];
    predicted_position[Y] = tblob.predicted_position[Y];
    travel[Y] = tblob.travel[Y];
    travel[X] = tblob.travel[X];
    total_travel[Y] = tblob.total_travel[Y];
    total_travel[X] = tblob.total_travel[X];
    start_pos[X] = tblob.start_pos[X];
    start_pos[Y] = tblob.start_pos[Y];
    start_time = tblob.start_time;
    has_updated = tblob.has_updated;
    event_duration = tblob.event_duration;
    has_updated = tblob.has_updated;
    times_updated = tblob.times_updated;
    average_difference = tblob.average_difference;
    max_difference = tblob.max_difference;
    max_size = tblob.max_size;
    max_width = tblob.max_width;
    max_height = tblob.max_height;
    average_area_difference = tblob.average_area_difference;
    average_position_difference = tblob.average_position_difference;
    average_aspect_ratio_difference = tblob.average_aspect_ratio_difference;
    average_direction_difference = tblob.average_direction_difference;
    average_temperature_difference = tblob.average_temperature_difference;
    max_num_dead_frames = tblob.max_num_dead_frames;
    num_dead_frames = tblob.num_dead_frames;
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

    edge_penalty = get_edge_penalty(other_blob.centroid[X]);
    position_difference = calculate_position_difference(other_blob);
    area_difference = calculate_area_difference(other_blob);
    aspect_ratio_difference = calculate_aspect_ratio_difference(other_blob);
    temperature_difference = calculate_temperature_difference(other_blob);
    direction_difference = calculate_direction_difference(other_blob);
    dead_frame_difference = calculate_dead_frame_difference();

    // Soften the difference if the blob is touching the sides of the frame
    // Blobs close to the centre do not get much leeway
    // Blobs close to the edges are probably still forming, so the penalties are softened a bunch
    difference_factor =
        position_difference + area_difference + aspect_ratio_difference + temperature_difference + direction_difference;

    return difference_factor;
}

float TrackedBlob::get_edge_penalty(float position) {
    float edge_penalty = 1;

    if (is_touching_side()) {
        edge_penalty = (1 - absolute((frame_width / 2 - position)) / (frame_width / 2));
    }

    return edge_penalty;
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
    return difference_factor * edge_penalty;
}

float TrackedBlob::calculate_area_difference(Blob other_blob) {
    float difference = absolute(_blob.get_size() - other_blob.get_size()) * area_penalty;
    return difference * edge_penalty;
}

float TrackedBlob::calculate_temperature_difference(Blob other_blob) {
    return (absolute(_blob.average_temperature - other_blob.average_temperature) * temperature_penalty);
}

float TrackedBlob::calculate_aspect_ratio_difference(Blob other_blob) {
    float difference = (absolute(_blob.aspect_ratio - other_blob.aspect_ratio) * aspect_ratio_penalty);
    return difference * edge_penalty;
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
    if (!is_touching_side() && times_updated > 1 && (latest_direction >= 0) != (travel >= 0)) {
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

float TrackedBlob::calculate_dead_frame_difference() {
    /**
    * Calculate the penalty for dead frames.
    * A dead frame is where the blob is no longer visible or recognised in the frame.
    * The purpose behind keeping dead frames is that blobs may 'disappear' for a frame or two while an object moves
    * through the frame.
    * The more frames that the blob has been 'dead' for, the higher the total difference.
    * The penalty for this difference should be set high to avoid new blobs being mistaken for old dead frames.
    *
    * @return The penalty as a result of the number of dead frames the blob has
    */

    return num_dead_frames * dead_frame_penalty;
}
