#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../../software/config/motor_axis_profile.h"

int main(void)
{
    MotorAxisProfile p, q;
    unsigned i;
    memset(&p, 0xFF, sizeof(p));
    assert(MotorAxisProfile_Load(&p, &q) && q.magic == 0);
    assert(MotorAxisProfile_AllowsPosition(&q, true, 20, 30));
    memset(&p, 0, sizeof(p));
    assert(MotorAxisProfile_Load(&p, &q));
    assert(!MotorAxisProfile_Load(NULL, &q));
    assert(!MotorAxisProfile_AllowsPosition(&p, false, 0, 0));
    assert(!MotorAxisProfile_AllowsPosition(&p, true, NAN, 0));
    assert(MotorAxisProfile_Create(&p, "roll", -1.57079633f, 1.57079633f, .785398163f));
    assert(p.crc32 == 3400232026U); /* Python zlib/Flash record golden vector. */
    assert(MotorAxisProfile_Load(&p, &q) && memcmp(&p, &q, sizeof(p)) == 0);
    assert(MotorAxisProfile_AllowsPosition(&p, true, 0, 1.48f));
    assert(!MotorAxisProfile_AllowsPosition(&p, true, 0, 1.6f));
    assert(!MotorAxisProfile_AllowsPosition(&p, true, 1.57079633f, 0));
    for (i = 0; i < sizeof(p); ++i) {
        q = p; ((unsigned char *)&q)[i] ^= 1;
        assert(!MotorAxisProfile_Load(&q, &q));
    }
    assert(MotorAxisProfile_Create(&p, "pitch", -.523598776f, .523598776f, .785398163f));
    assert(!MotorAxisProfile_AllowsPosition(&p, true, 0, .6f));
    assert(MotorAxisProfile_AllowsPosition(&p, true, 0, .436332313f));
    assert(!MotorAxisProfile_Create(&p, "other", -1, 1, 1));
    assert(!MotorAxisProfile_Create(&p, "roll", 1, -1, 1));
    assert(!MotorAxisProfile_Create(&p, "roll", -1, 1, INFINITY));
    puts("PASS axis profiles: legacy, roll/pitch, CRC corruption, limits and invalid inputs");
    return 0;
}
