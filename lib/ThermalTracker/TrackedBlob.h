#ifndef TRACKED_BLOB_H
#define TRACKED_BLOB_H

#include "Pixel.h"
#include "Blob.h"

const float POSITION_PENALTY = 2.0;
const float AREA_PENALTY = 2.0;
const float ASPECT_RATIO_PENALTY = 10.0;
const float TEMPERATURE_PENALTY = 10.0;

float absolute(float f);

class TrackedBlob{
public:
    TrackedBlob();
    void clear();
    void set(Blob);
    void update_blob(Blob blob);
    float get_travel(int axis);

    void reset_updated_status();
    bool is_active();
    bool has_updated();

    float get_distance(Blob other_blob);
    void copy(TrackedBlob tblob);

//private:
    void copy_blob(Blob blob);

    Blob _blob;
    float _predicted_position[2];
    float _travel[2];
    bool _has_updated;
};


#endif
