#ifndef BLOB_H
#define BLOB_H

#include "Pixel.h"
class Blob{
public:
    Blob();
    void copy(Blob blob);
    void clear();
    void add_pixel(Pixel);
    bool is_active();
    int get_size();
    void set_assigned();
    void clear_assigned();
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
    void recalculate_centroid(float pixel_x, float pixel_y);
    void recalculate_bounds(int x, int y);

    float total_x;
    float total_y;
    bool _is_assigned;
};

enum COORDINATES{
    X = 1,
    Y = 0
};

#endif
