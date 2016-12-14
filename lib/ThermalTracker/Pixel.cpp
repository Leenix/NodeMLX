#include "Pixel.h"

////////////////////////////////////////////////////////////////////////////////
// Constructor

Pixel::Pixel(){
    /**
    * Create an empty pixel object
    * Pixel values need to be set using the set method rather than on construction
    * It's a lot easier to use a null constructor for lists and junk, trust me...
    */
    _x = -1;
    _y = -1;
    _temperature = -1;
}

Pixel::Pixel(int x, int y, float temperature){
    /**
    * Create a pixel object
    * @param x The column location of the pixel (should be positive)
    * @param y The row location of the pixel (should be positive)
    * @param temperature Recorded temperature of the pixel (in °C)
    *
    * AN: The temperature field could probably be changed to a standard value field for use with normal camera pixels
    *   - (As long as they're greyscale)
    */
    set(x, y, temperature);
}

////////////////////////////////////////////////////////////////////////////////
// Public methods

void Pixel::set(int x, int y, float temperature){
    /**
    * Set the pixels' values
    * @param x The column location of the pixel (should be positive)
    * @param y The row location of the pixel (should be positive)
    * @param temperature Recorded temperature of the pixel (in °C)
    *
    * AN: The temperature field could probably be changed to a standard value field for use with normal camera pixels
    *   - (As long as they're greyscale)
    */
    _x = x;
    _y = y;
    _temperature = temperature;
}

int Pixel::get_x(){
    /**
    * Get the column location of the pixel
    * @return Column location of the pixel (should be positive)
    */
    return _x;
}

int Pixel::get_y(){
    /**
    * Get the row location of the pixel
    * @return Row location of the pixel (should be positive)
    */
    return _y;
}

float Pixel::get_temperature(){
    /**
    * Get the recorded temperature of the pixel
    * @return Pixel temperature in deg C
    */
    return _temperature;
}

bool Pixel::is_adjacent(Pixel other_pixel){
    /**
    * Check if the pixel is adjacent to another pixel.
    * Diagonal adjacency also counts in this case
    * @param other_pixel A second pixel to test adjacency against
    * @return True if the other_pixel is in the surrounding 8 adjacent pixel locations. False if not adjacent.
    * AN: Pixels are not adjacent if they occupy the same location.
    */

    bool adjacent = false;

    // Pixel must be located in the frame
    if (other_pixel.get_x() >= 0 ){
        if (other_pixel.get_y() >= 0){

            // The two pixels are not adjacent if they occupy the same location
            if (other_pixel.get_x() != get_x() || other_pixel.get_y() != get_y()) {

                // Pixel must be in one of the 8 surrounding pixels
                if (abs(_x - other_pixel.get_x()) <= 1){
                    if (abs(_y - other_pixel.get_y()) <= 1){
                        adjacent = true;
                    }
                }
            }
        }
    }

    return adjacent;
}
