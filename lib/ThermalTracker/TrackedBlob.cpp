#include "TrackedBlob.h"

float absolute(float f){
    if (f < 0.0){
        f *= -1.0;
    }
    return f;
}

////////////////////////////////////////////////////////////////////////////////
// Constructor

TrackedBlob::TrackedBlob(){
    /**
    * Create a new tracked blob.
    * Track blobs start off empty until they are initialised by a regular blob using the set method.
    * After that, you can update the blob that is being tracked to record its movements between frames (and thus updates)
    */
    clear();
}

////////////////////////////////////////////////////////////////////////////////
// Public Methods

void TrackedBlob::clear(){
    /**
    * Clear the tracked blob's characteristics
    * The tracked blob object will be marked inactive until the blob is reinitialised.
    * Tracking information is also lost by this action.
    */
    _blob.clear();

    _predicted_position[X] = -1;
    _predicted_position[Y] = -1;
    _travel[X] = 0;
    _travel[Y] = 0;
    reset_updated_status();
}

void TrackedBlob::set(Blob blob){
    /**
    * Start tracking a new blob.
    * All previous tracking data (if any existed) is lost by this action.
    * @param blob Blob to start tracking
    */
    clear();
    copy_blob(blob);
    _has_updated = true;
}

bool TrackedBlob::is_active(){
    /**
    * Determine if the tracked blob is actually tracking anything
    * @return True if a blob is being tracked
    */
    return _blob.is_active();
}

void TrackedBlob::update_blob(Blob blob){
    /**
    * Update the tracked blob
    * Movements between the old and new blob states are recorded.
    * The next predicted position of the next blob is also calculated for future calculations
    * @param blob New blob state used to update the tracked blob
    */
        float movement[2];

        movement[X] = blob.centroid[X] - _blob.centroid[X];
        movement[Y] = blob.centroid[Y] - _blob.centroid[Y];

        _predicted_position[X] = blob.centroid[X] + movement[X];
        _predicted_position[Y] = blob.centroid[Y] + movement[Y];

        _travel[X] += movement[X];
        _travel[Y] += movement[Y];

        copy_blob(blob);

        _has_updated = true;
}

bool TrackedBlob::has_updated(){
    /**
    * Find out if the tracked blob has updated
    * @return True if the tracked blob has updated
    */
    return _has_updated;
}

void TrackedBlob::reset_updated_status(){
    /**
    * Reset the updated status of the tracked blob.
    * Tracked blobs that have not updated will be purged at the end of frame processing.
    */
    _has_updated = false;
}

void TrackedBlob::copy(TrackedBlob tblob){
    /**
    * Overwrite the tracked blob with the information from another tracked blob.
    * Useful for shuffling tracked blobs around in arrays
    * Information from the source blob completely overwrites any currently-held information
    * @param tblob Tracked blob to copy information from
    */
    copy_blob(tblob._blob);

    _predicted_position[X] = tblob._predicted_position[X];
    _predicted_position[Y] = tblob._predicted_position[Y];
    _travel[Y] = tblob._travel[Y];
    _travel[X] = tblob._travel[X];
    _has_updated = tblob.has_updated();
}

float TrackedBlob::get_travel(int axis){
    /**
    * Get the net travel distance of the tracked blob as it moves between frames
    * @param axis The axis of travel to get the distance for
    * @return The net number of pixels the blob has moved from its original position in the specified axis
    */
    float travel = 0;

    if (axis == X){
        travel = _travel[X];
    }
    else{
        travel = _travel[Y];
    }

    return travel;
}

float TrackedBlob::get_distance(Blob other_blob){
    /**
    * Find out how 'different' the tracked blob is from another blob; not just how far away the blob is...
    * A low distance score between blobs means they are very similar
    * This function is used on blobs between frames to determine if the blobs originate from the same object
    * @param other_blob The second blob in the calculations. The distance factor will be between this blob and the tracked blob.
    * @return The distance score between the two blobs. Unitless.
    */
    float difference_factor = 0.0;

    if (_predicted_position[X] >= 0 && _predicted_position[Y] >= 0){
        difference_factor += absolute(_predicted_position[X] - other_blob.centroid[X]) * POSITION_PENALTY;
        difference_factor += absolute(_predicted_position[Y] - other_blob.centroid[Y]) * POSITION_PENALTY;
    }
    else{
        difference_factor += absolute(_blob.centroid[X] - other_blob.centroid[X]) * POSITION_PENALTY;
        difference_factor += absolute(_blob.centroid[Y] - other_blob.centroid[Y]) * POSITION_PENALTY;
    }

    difference_factor += absolute(_blob.num_pixels - other_blob.num_pixels) * AREA_PENALTY;

    difference_factor += absolute(_blob.average_temperature - other_blob.average_temperature) * TEMPERATURE_PENALTY;

    difference_factor += absolute(_blob.aspect_ratio - other_blob.aspect_ratio) * ASPECT_RATIO_PENALTY;

    return difference_factor;
}

////////////////////////////////////////////////////////////////////////////////
// Private Methods

void TrackedBlob::copy_blob(Blob blob){
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
