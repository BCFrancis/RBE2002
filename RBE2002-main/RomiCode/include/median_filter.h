#ifndef MEDIAN_FILTER
#define MEDIAN_FILTER

#include <Romi32U4.h>

class MedianFilter {
   private:
    int array[5] = {0};

   public:
    void Sort(int, int);
    void Init();
    int Filter(int);
};

#endif