#ifndef TRACKED_BLOB_H
#define TRACKED_BLOB_H

#include "Blob.h"
#include "Pixel.h"

float absolute(float f);

const static char* TBLOB_VERSION = "20170606";

class TrackedBlob {
   public:
    TrackedBlob();

    /**
    * Clear the tracked blob's characteristics
    * The tracked blob object will be marked inactive until the blob is reinitialised.
    * Tracking information is also lost by this action.
    */
    void clear();

    /**
    * Start tracking a new blob.
    * All previous tracking data (if any existed) is lost by this action.
    * @param blob Blob to start tracking
    * @param _id Optional. Blob indentifying number.
    */
    void set(Blob);
    void set(Blob blob, unsigned int _id);

    /**
    * Update the tracked blob
    * Movements between the old and new blob states are recorded.
    * The next predicted position of the next blob is also calculated for future calculations
    * @param blob New blob state used to update the tracked blob
    */
    void update_blob(Blob blob);

    void update_movements(Blob blob);
    void update_geometry(Blob blob);
    void update_differences(Blob blob);

    /**
    * Get the net travel difference of the tracked blob as it moves between frames
    * @param axis The axis of travel to get the difference for
    * @return The net number of pixels the blob has moved from its original position in the specified axis
    */
    float get_travel(int axis);

    /**
    * Reset the updated status of the tracked blob.
    * Tracked blobs that have not updated will be purged at the end of frame processing.
    */
    void reset_updated_status();

    /**
    * Determine if the tracked blob is actually tracking anything
    * @return True if a blob is being tracked
    */
    bool is_active();

    float get_difference(Blob other_blob);
    float get_edge_penalty(float position);
    float calculate_position_difference(Blob other_blob);
    float calculate_area_difference(Blob other_blob);
    float calculate_temperature_difference(Blob other_blob);
    float calculate_aspect_ratio_difference(Blob other_blob);

    /**
    * Calculate the penalty for any changes in the blobs direction of travel
    * This penalty is binary.
    * If the blob is moving in the same direction as before, no penalty is applied.
    * If the movement is different from the net travel direction of the blob, the penalty is added.
    * @return the penalty as a result of differences in travel direction
    */
    float calculate_direction_difference(Blob other_blob);

    /**
    * Determine if the blob is touching the side of the frame
    * @return True if the blob is likely touching a vertical side of the frame
    */
    bool is_touching_side();

    /**
    * Overwrite the tracked blob with the information from another tracked blob.
    * Useful for shuffling tracked blobs around in arrays
    * Information from the source blob completely overwrites any currently-held information
    * @param tblob Tracked blob to copy information from
    */
    void copy(TrackedBlob tblob);

    /**
    * Copy the details from a given blob into the tracked blob.
    * @param blob Source blob to copy data from.
    */
    void copy_blob(Blob blob);

    Blob _blob;
    static float position_penalty;
    static float area_penalty;
    static float aspect_ratio_penalty;
    static float temperature_penalty;
    static float direction_penalty;
    static int frame_width;

    float predicted_position[2];
    float travel[2];
    int total_travel[2];
    long start_time;
    long event_duration;
    bool has_updated;
    int times_updated;
    float start_pos[2];
    float average_difference;
    float max_difference;
    int max_size;
    int max_width;
    int max_height;
    unsigned int id;

    float position_difference;
    float direction_difference;
    float temperature_difference;
    float aspect_ratio_difference;
    float area_difference;

    float average_position_difference;
    float average_direction_difference;
    float average_temperature_difference;
    float average_aspect_ratio_difference;
    float average_area_difference;
    float edge_penalty;
};

#endif
