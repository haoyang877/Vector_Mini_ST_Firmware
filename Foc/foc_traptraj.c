#include "foc_traptraj.h"

#include "utils.h"
#include "hw_conf.h"

Traj_TypeDef Traj;

/**
	* @brief  Trapzoidal profile plan 
    * @param  target_position: target position (rad)
    * @param  start_position: start position (rad)
	* @param  start_velocity: start velocity (rad/s)
	* @param  Vmax: maximum velocity during  
	* @param  Amax: maximum unsigned acceleration (rad/(s¡¤s)) 
	* @param  Dmax: maximum unsigned deceleration (rad/(s¡¤s)) 
 **/
void TRAJ_plan(float position, float start_position, float start_velocity, float Vmax, float Amax, float Dmax)
{
    float distance  = position - start_position;           // Distance to travel
    float stop_dist = fast_sq(start_velocity) / (2.0f * Dmax);  // Minimum stopping distance
    float dXstop    = copy_sign(stop_dist, start_velocity); // Minimum stopping displacement
    float s         = sign_hard(distance - dXstop);        // Sign of coast velocity (if any)
    Traj.acc        = s * Amax;                            // Maximum Acceleration (signed)
    Traj.dec        = -s * Dmax;                           // Maximum Deceleration (signed)
    Traj.vel        = s * Vmax;                            // Maximum Velocity (signed)

    // If we start with a speed faster than cruising, then we need to decel instead of accel aka "double deceleration move" in the paper
    if ((s * start_velocity) > (s * Traj.vel)) {
        Traj.acc = -s * Amax;
    }

    // Time to accel/decel to/from Vr (cruise speed)
    Traj.t_acc = (Traj.vel - start_velocity) / Traj.acc;
    Traj.t_dec = -Traj.vel / Traj.dec;

    // Integral of velocity ramps over the full accel and decel times to get
    // minimum displacement required to reach cuising speed
    float dXmin = 0.5f * Traj.t_acc * (Traj.vel + start_velocity) + 0.5f * Traj.t_dec * Traj.vel;

    // Are we displacing enough to reach cruising speed?
    if (s * distance < s * dXmin) {
        // Short move (triangle profile)
        Traj.vel = s
                   * fast_sqrt(fast_max((Traj.dec * fast_sq(start_velocity) + 2.0f * Traj.acc * Traj.dec * distance)
                                    / (Traj.dec - Traj.acc),
                                0.0f));
        Traj.t_acc = fast_max(0.0f, (Traj.vel - start_velocity) / Traj.acc);
        Traj.t_dec = fast_max(0.0f, -Traj.vel / Traj.dec);
        Traj.t_vel = 0.0f;
    } else {
        // Long move (trapezoidal profile)
        Traj.t_vel = (distance - dXmin) / Traj.vel;
    }

    // Fill in the rest of the values used at evaluation-time
    Traj.t_total        = Traj.t_acc + Traj.t_vel + Traj.t_dec;
    Traj.start_position = start_position;
    Traj.start_velocity = start_velocity;
    Traj.end_position   = position;
    Traj.acc_distance   = start_position + start_velocity * Traj.t_acc
                        + 0.5f * Traj.acc * fast_sq(Traj.t_acc); // pos at end of accel phase

    Traj.tick         = 0;
    Traj.profile_done = false;
}


/**
	* @brief  Evaluate trapezoidal profile output
 **/
void TRAJ_eval(void)
{
    if (Traj.profile_done) {
        return;
    }

    Traj.tick++;
    float t = Traj.tick * Position_Ts;

    if (t < 0.0f) { // Initial Condition
        Traj.Y   = Traj.start_position;
        Traj.Yd  = Traj.start_velocity;
        Traj.Ydd = 0.0f;
    } else if (t < Traj.t_acc) { // Accelerating
        Traj.Y   = Traj.start_position + Traj.start_velocity * t + 0.5f * Traj.acc * fast_sq(t);
        Traj.Yd  = Traj.start_velocity + Traj.acc * t;
        Traj.Ydd = Traj.acc;
    } else if (t < Traj.t_acc + Traj.t_vel) { // Coasting
        Traj.Y   = Traj.acc_distance + Traj.vel * (t - Traj.t_acc);
        Traj.Yd  = Traj.vel;
        Traj.Ydd = 0.0f;
    } else if (t < Traj.t_total) { // Deceleration
        float td = t - Traj.t_total;
        Traj.Y   = Traj.end_position + 0.5f * Traj.dec * fast_sq(td);
        Traj.Yd  = Traj.dec * td;
        Traj.Ydd = Traj.dec;
    } else if (t >= Traj.t_total) { // Final Condition
        Traj.Y            = Traj.end_position;
        Traj.Yd           = 0.0f;
        Traj.Ydd          = 0.0f;
        Traj.profile_done = true;
    }
}

/**
	* @brief  Get trapezoidal profile position output
	* @retval profile position output
 **/
float TRAJ_Get_Y(void)
{
	return Traj.Y;
}

/**
	* @brief  Get trapezoidal profile velocity output
	* @retval profile velocity output
 **/
float TRAJ_Get_Yd(void)
{
	return Traj.Yd;
}
