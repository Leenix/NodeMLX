#ifndef PIXEL_H
#define PIXEL_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

const static char* PIXEL_VERSION = "20170606";

class Pixel {
   public:
    Pixel();
    Pixel(int x, int y, float temperature);
    void set(int x, int y, float temperature);
    bool is_adjacent(Pixel);

    int get_x();
    int get_y();
    float get_temperature();

   private:
    int _x;
    int _y;
    float _temperature;
};

#endif
