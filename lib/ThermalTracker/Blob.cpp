#include "Blob.h"

////////////////////////////////////////////////////////////////////////////////
// Constructor

Blob::Blob(void){
    /**
    * Create a new blob object.
    * Blobs are clumps of adjacent pixels that are aggregated to have generalised characteristics.
    * Blobs start off empty and inactive.
    * Feed the blob pixels to make it grow.
    */
    clear();
}

////////////////////////////////////////////////////////////////////////////////
// Public Methods

void Blob::clear(){
    /**
    * Clear the blob configuration to set it back to blank.
    */
    num_pixels = 0;
    width = 0;
    height = 0;
    num_pixels = 0;
    average_temperature = 0;
    aspect_ratio = 0;
    total_x = 0;
    total_y = 0;
    centroid[X] = -1;
    centroid[Y] = -1;

    clear_assigned();
}

void Blob::add_pixel(Pixel pixel){
    /**
    * Add a new pixel to the blob
    * The blob will need to recalculate its shape and other aspects
    * @param pixel Pixel object to be added to the blob
    *
    * AN: There are currently no mechanisms that prevent a pixel from being added multiple times.
    *   - Probably not going to bother
    *   - Pixel objects are not actually stored. The blob just absorbs its information (as blobs do).
    */

    int pixel_x = pixel.get_x();
    int pixel_y = pixel.get_y();
    float pixel_temp = pixel.get_temperature();
    num_pixels++;

    average_temperature = (average_temperature * (num_pixels-1) + pixel_temp) / float(num_pixels);
    recalculate_bounds(pixel_x, pixel_y);
    recalculate_centroid(pixel_x, pixel_y);

}

void Blob::copy(Blob blob){
    /**
    * Copy the information of another blob.
    * All previous information in the blob is overwritten.
    * Useful for moving blobs around an array.
    * @param blob Source blob to copy information from.
    */

    centroid[X] = blob.centroid[X];
    centroid[Y] = blob.centroid[Y];
    min[X] = blob.min[X];
    min[Y] = blob.min[Y];
    max[X] = blob.max[X];
    max[Y] = blob.max[Y];
    aspect_ratio = blob.aspect_ratio;
    average_temperature = blob.average_temperature;
    width = blob.width;
    height = blob.height;
    num_pixels = blob.num_pixels;
}

bool Blob::is_active(){
    /**
    * Determine if the blob is actually being used.
    * A blob must have at least one pixel to be considered active.
    * @return True if the blob contains at least one pixel.
    */
    return num_pixels > 0;
}

void Blob::set_assigned(){
    /**
    * Set the assigned flag, which indicates the blob has been assigned to a tracked blob object
    */
    _is_assigned = true;
}

bool Blob::is_assigned(){
    /**
    * Determine if the blob has been assigned to a tracked blob.
    * @return True if the blob has been assigned.
    */
    return _is_assigned;
}

void Blob::clear_assigned(){
    /**
    * Clear the state of the is_assigned flag.
    * This indicates the blob has not been assigned to a tracked blob.
    * Blobs that have not been assigned need a new tracked blob to be created for them
    */
    _is_assigned = false;
}

int Blob::get_size(){
    /**
    * Get the number of pixels contained in the blob
    * @return Number of pixels the blob has absorbed
    */
    return num_pixels;
}

////////////////////////////////////////////////////////////////////////////////
// Private Methods

void Blob::recalculate_bounds(int pixel_x, int pixel_y){
    /**
    * Recalculate the minimum and maximum bounds of the blob.
    * Secondary values including the width, height, and aspect ratio of the blob are also recalculated.
    * This occurs every time a new pixel is added to the blob because the old values are invalidated.
    * @param pixel_x Column location of the new pixel
    * @param pixel_y Row location of the new pixel
    */

    // First pixel added to the blob; initialise the values
    if (num_pixels == 1){
        min[X] = pixel_x;
        min[Y] = pixel_y;
        max[X] = pixel_x;
        max[Y] = pixel_y;
    }

    // Not the first; business as usual
    else{
        if (pixel_x > max[X]){
            max[X] = pixel_x;
        }

        if (pixel_x < min[X]) {
            min[X] = pixel_x;
        }

        if (pixel_y > max[Y]) {
            max[Y] = pixel_y;
        }

        if (pixel_y < min[Y]) {
            max[Y] = pixel_y;
        }
    }

    width = (max[X] - min[X]) + 1;
    height = (max[Y] - min[Y]) + 1;
    aspect_ratio = float(width)/float(height);
}

void Blob::recalculate_centroid(float pixel_x, float pixel_y){
    /**
    * Recalculate the centroid location of the blob
    * This occurs every time a new pixel is added to the blob because the old values are invalidated.
    * @param pixel_x Column location of the new pixel
    * @param pixel_y Row location of the new pixel
    */
    total_x += pixel_x;
    total_y += pixel_y;

    centroid[X] = total_x/num_pixels;
    centroid[Y] = total_y/num_pixels;
}
