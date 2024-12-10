#ifndef THINNING_H
#define THINNING_H
#include <stddef.h>

#define ZERO 0x00
#define ONE 0xFF
#define M 128

struct specialThinPattern
{
    unsigned char pattern[9];
};

class Thinning
{
public:
    Thinning();

    //algorithm from Zoeng and Hall 1989, ualso used by MATLAB, claims to result in 8-connected thinned lines
    bool thinningGUOandHALL(const unsigned char* image, unsigned char* output, int width, int height);
    bool spurRemoval(int iterations, unsigned char* image, int width, int height);
    bool spurRemovalSmart(int iterations, unsigned char* image, int width, int height);
    //method used to set the borders as 0
    void _setBordersAsZero(unsigned char* image, const size_t& width, const size_t& height, const size_t& channels);

    //stuff for GUO and HALL thinning algorithm
    void _grabThinPattern2(const unsigned char *image, int width, int center, specialThinPattern &p);
    int _XH(const specialThinPattern &p);//eight-connected stuff
    int _minN1N2(const specialThinPattern &p);//eight-connected stuff
    bool _testG3(const specialThinPattern &p);//odd  condition
    bool _testG3_(const specialThinPattern &p);//even condition
};

#endif // THINNING_H
