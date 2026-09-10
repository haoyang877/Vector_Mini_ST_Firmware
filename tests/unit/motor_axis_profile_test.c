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
    MotorJointControlConfig config;
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
    assert(MotorAxisProfile_InitialEncoderOffsetQ15(&p, true, -60800) == 4736);
    assert(MotorAxisProfile_InitialEncoderOffsetQ15(&p, true, 60800) == -4736);
    assert(MotorAxisProfile_InitialEncoderOffsetQ15(&p, true, 1000) == 1000);
    assert(MotorAxisProfile_InitialEncoderOffsetQ15(&p, false, -60800) == -60800);
    assert(MotorAxisProfile_InitialEncoderOffsetQ15(NULL, true, -60800) == -60800);
    assert(MotorAxisProfile_InitialEncoderOffsetQ15(&p, true, 32768) == 32768);
    assert(MotorAxisProfile_InitialEncoderOffsetQ15(&p, true, -32768) == -32768);
    assert(MotorAxisProfile_InitialEncoderOffsetQ15(&p, true, 70000) == 70000);
    /* A real out-of-travel angle stays out, even after branch selection. */
    assert(!MotorAxisProfile_AllowsPosition(&p, true,
        MotorAxisProfile_InitialEncoderOffsetQ15(&p, true, -45000) *
        (6.283185307f / 65536.0f), 0.0f));
    q = p; q.minimum_position_rad = -7.0f;
    assert(MotorAxisProfile_InitialEncoderOffsetQ15(&q, true, -60800) == -60800);
    q = p; q.maximum_position_rad = NAN;
    assert(MotorAxisProfile_InitialEncoderOffsetQ15(&q, true, -60800) == -60800);
    memset(&q, 0, sizeof(q));
    assert(MotorAxisProfile_InitialEncoderOffsetQ15(&q, true, -60800) == -60800);
    assert(MotorAxisProfile_JointType(&p) == MOTOR_JOINT_ROLL);
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
    assert(MOTOR_JOINT_UNKNOWN == 0 && MOTOR_JOINT_ROLL == 1 && MOTOR_JOINT_PITCH == 2 &&
           MOTOR_JOINT_YAW == 3 && MOTOR_JOINT_WHEEL_RIGHT == 4 && MOTOR_JOINT_WHEEL_LEFT == 5);
    assert(!MotorAxisProfile_Resolve(&p, &config)); /* AXS1 keeps saved gains. */
    assert(strcmp(MotorJointType_Name(MOTOR_JOINT_WHEEL_RIGHT), "wheel_right") == 0);
    assert(strcmp(MotorJointType_Name(MOTOR_JOINT_WHEEL_LEFT), "wheel_left") == 0);
    assert(strcmp(MotorJointType_Name(99), "unknown") == 0);
    for (i = 0; i <= 5; ++i) {
        assert(MotorAxisProfile_CreateJoint(&p, i, 0, 0, 0, 0));
        assert(MotorAxisProfile_Load(&p, &q));
        assert(MotorAxisProfile_JointType(&q) == i);
        assert(!MotorAxisProfile_Resolve(&q, &config));
        assert(!MotorAxisProfile_AllowsPosition(&q, true, 0, 0));
    }
    assert(!MotorAxisProfile_CreateJoint(&p, 6, 0, 0, 0, 0));
    assert(!MotorAxisProfile_CreateJoint(&p, 3, 1, -1, 1, .5f));
    assert(!MotorAxisProfile_CreateJoint(&p, 1, 2, -1, 1, .5f));
    assert(!MotorAxisProfile_CreateJoint(&p, 1, 0, -1, 1, .5f));
    assert(!MotorAxisProfile_CreateJoint(&p, 2, 1, -.4f, .9f, .5f));
    assert(!MotorAxisProfile_CreateJoint(&p, 1, 1, -1, 1, 1));
    assert(MotorAxisProfile_CreateJoint(&p, 1, 1, -1.57079633f, 1.57079633f, .785398163f));
    assert(p.crc32 == 1552381621U); /* Python zlib AXS2 golden vector. */
    assert(MotorAxisProfile_Resolve(&p, &config));
    assert(config.position_kp == 8 && config.position_kd == 2 && config.speed_kp == .5f && config.speed_ki == 1);
    for (i = 0; i < sizeof(p); ++i) {
        q = p; ((unsigned char *)&q)[i] ^= 1;
        assert(!MotorAxisProfile_Load(&q, &q));
        assert(!MotorAxisProfile_Resolve(&q, &config));
    }
    assert(MotorAxisProfile_CreateJoint(&p, 2, 1, -.3f, .9f, .785398163f));
    assert(p.crc32 == 2092040327U);
    assert(MotorAxisProfile_Resolve(&p, &config) && config.speed_ki == 2);
    assert(MotorAxisProfile_AllowsPosition(&p, true, 0, .8f));
    assert(!MotorAxisProfile_AllowsPosition(&p, true, -.3f, 0));
    assert(!MotorAxisProfile_AllowsPosition(&p, true, 0, .91f));
    assert(MotorAxisProfile_CreateJoint(&p, 2, 1, -.1f, .3f, .2f));
    assert(MotorAxisProfile_Resolve(&p, &config)); /* narrower configured envelope */
    assert(!MotorAxisProfile_AllowsPosition(NULL, true, 0, 0));
    puts("PASS numeric joint IDs: legacy identity, disabled placeholders, versioned roll/pitch, range and CRC");
    puts("PASS axis profiles: legacy, roll/pitch, CRC corruption, limits and invalid inputs");
    return 0;
}
