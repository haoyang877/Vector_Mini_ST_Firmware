#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "position_smooth_trajectory.h"

static void prepare(PositionSmoothTrajectory *s)
{
    unsigned ticks = 0;
    while (!PositionSmooth_Prepare(s)) assert(!s->failed && ++ticks < 64);
}

static void check_joins(const PositionSmoothTrajectory *s)
{
    float joins[] = {0, s->accel_time, s->accel_time+s->cruise_time, s->duration};
    float h = 0.00001f*s->duration;
    float tr = fminf(s->accel_time, s->decel_time);
    float snap = 60*s->speed/(tr*tr*tr);
    unsigned k;
    for (k = 0; k < 4; ++k) {
        PositionSmoothSample l, r;
        PositionSmooth_Sample(s, joins[k]-h, &l);
        PositionSmooth_Sample(s, joins[k]+h, &r);
        assert(fabsf(r.position-l.position) <= 2.1f*h*s->speed+0.000002f);
        assert(fabsf(r.speed-l.speed) <= 2.1f*h*fmaxf(s->acceleration,s->deceleration)+0.000002f);
        assert(fabsf(r.acceleration-l.acceleration) <= 2.1f*h*s->jerk_limit+0.000002f);
        assert(fabsf(r.jerk-l.jerk) <= 2.1f*h*snap+0.00002f);
    }
}

int main(int argc, char **argv)
{
    const float distances[] = {0.000001f, .0011f, .005f, .087266463f,
        .34906585f, 1.48352986f, 2.96705973f, 10.0f};
    int sign, k, n;
    PositionSmoothTrajectory s;
    PositionSmoothSample o, previous;
    assert(!PositionSmooth_Begin(&s,0,NAN,1,1,1,1));
    assert(!PositionSmooth_Begin(&s,0,1,0,1,1,1));
    assert(!PositionSmooth_Begin(&s,0,1,1,1,0,1));
    assert(!PositionSmooth_Begin(&s,0,1,1,1,1,INFINITY));
    assert(PositionSmooth_Begin(&s,1,1,1,1,1,1));
    prepare(&s); assert(PositionSmooth_Advance(&s,.001f,&o));
    assert(o.position==1 && o.speed==0 && o.acceleration==0 && o.jerk==0);
    for (sign=-1; sign<=1; sign+=2) for(k=0;k<8;++k) {
        double integral = 0;
        float h;
        assert(PositionSmooth_Begin(&s,0,sign*distances[k],.785398163f,
            .785398163f,.523598776f,3.926990817f));
        prepare(&s); check_joins(&s); h=s.duration/20000;
        PositionSmooth_Sample(&s,0,&previous);
        for(n=1;n<=20000;++n) {
            PositionSmooth_Sample(&s,s.duration*((float)n/20000),&o);
            assert(isfinite(o.position) && isfinite(o.speed) && isfinite(o.acceleration) && isfinite(o.jerk));
            assert(sign*o.position >= sign*previous.position-0.000003f);
            assert(sign*o.position >= -.000003f && sign*o.position <= distances[k]+.000003f);
            assert(sign*o.speed >= -.000002f && fabsf(o.speed) <= s.speed_limit+.000002f);
            assert(sign*o.acceleration <= s.acceleration+.000002f);
            assert(sign*o.acceleration >= -s.deceleration-.000002f);
            assert(fabsf(o.jerk) <= s.jerk_limit+.00002f);
            integral += .5*(previous.speed+o.speed)*h;
            previous=o;
        }
        assert(fabs(integral-s.target)<.00001*(1+s.distance));
        assert(o.position==s.target && o.speed==0 && o.acceleration==0 && o.jerk==0);
    }
    puts("PASS quintic velocity profiles: limits, integrated displacement, C3 joins, short/cruise moves, both directions");
    if (argc==2) {
        FILE *f=fopen(argv[1],"w"); assert(f);
        assert(PositionSmooth_Begin(&s,0,1.48352986f,.785398163f,.785398163f,.523598776f,3.926990817f));prepare(&s);
        fputs("time_s,position_rad,velocity_rad_s,acceleration_rad_s2,jerk_rad_s3\n",f);
        for(n=0;n<=10000;++n) {
            float t=s.duration*(float)n/10000;
            PositionSmooth_Sample(&s,t,&o);
            fprintf(f,"%.9g,%.9g,%.9g,%.9g,%.9g\n",t,o.position,o.speed,o.acceleration,o.jerk);
        }
        assert(fclose(f)==0);
    }
    return 0;
}
