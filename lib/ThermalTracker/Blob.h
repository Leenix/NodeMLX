#ifndef BLOB_H
#define BLOB_H

#include "Pixel.h"

const static char* BLOB_VERSION = "20170606";

class Blob {
   public:
    /**
    * Create a new blob object.
    * Blobs are clumps of adjacent pixels that are aggregated to have generalised characteristics.
    * Blobs start off empty and inactive.
    * Feed the blob pixels to make it grow.
    */
    Blob();

    /**
    * Clear the blob configuration to set it back to blank.
    */
    void clear();

    /**
    * Add a new pixel to the blob
    * The blob will need to recalculate its shape and other aspects
    * @param pixel Pixel object to be added to the blob
    *
    * AN: There are currently no mechanisms that prevent a pixel from being added multiple times.
    *   - Probably not going to bother
    *   - Pixel objects are not actually stored. The blob just absorbs its information (as blobs do).
    */
    void add_pixel(Pixel);

    /**
    * Copy the information of another blob.
    * All previous information in the blob is overwritten.
    * Useful for moving blobs around an array.
    * @param blob Source blob to copy information from.
    */
    void copy(Blob blob);

    /**
    * Determine if the blob is actually being used.
    * A blob must have at least one pixel to be considered active.
    * @return True if the blob contains at least one pixel.
    */
    bool is_active();

    /**
    * Set the assigned flag, which indicates the blob has been assigned to a tracked blob object
    */
    void set_assigned();

    /**
    * Get the number of pixels contained in the blob
    * @return Number of pixels the blob has absorbed
    */
    int get_size();

    /**
    * Clear the state of the is_assigned flag.
    * This indicates the blob has not been assigned to a tracked blob.
    * Blobs that have not been assigned need a new tracked blob to be created for them
    */
    void clear_assigned();

    /**
    * Set the assigned flag, which indicates the blob has been assigned to a tracked blob object
    */
    bool is_assigned();

    int min[2];
    int max[2];
    float centroid[2];
    float aspect_ratio;
    float average_temperature;
    int width;
    int height;
    int num_pixels;

   private:
    /**
    * Recalculate the centroid location of the blob
    * This occurs every time a new pixel is added to the blob because the old values are invalidated.
    * @param pixel_x Column location of the new pixel
    * @param pixel_y Row location of the new pixel
    */
    void recalculate_centroid(float pixel_x, float pixel_y);

    /**
    * Recalculate the minimum and maximum bounds of the blob.
    * Secondary values including the width, height, and aspect ratio of the blob are also recalculated.
    * This occurs every time a new pixel is added to the blob because the old values are invalidated.
    * @param pixel_x Column location of the new pixel
    * @param pixel_y Row location of the new pixel
    */
    void recalculate_bounds(int x, int y);

    float total_x;
    float total_y;
    bool _is_assigned;
};

enum COORDINATES { X = 1, Y = 0 };

#endif
