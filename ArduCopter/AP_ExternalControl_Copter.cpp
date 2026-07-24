/*
  external control library for copter
 */


#include "AP_ExternalControl_Copter.h"
#if AP_EXTERNAL_CONTROL_ENABLED

#include "Copter.h"

/*
  set linear velocity and yaw rate. Pass NaN for yaw_rate_rads to not control yaw
  velocity is in earth frame, NED, m/s
*/
bool AP_ExternalControl_Copter::set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity_ned_ms, float yaw_rate_rads)
{
    if (!ready_for_external_control()) {
        return false;
    }
    const float checked_yaw_rate_rad = isnan(yaw_rate_rads)? 0: yaw_rate_rads;

    // Copter velocity is positive if aircraft is moving up which is opposite the incoming NED frame.
    copter.mode_guided.set_vel_NED_ms(linear_velocity_ned_ms, false, 0, !isnan(yaw_rate_rads), checked_yaw_rate_rad);
    return true;
}

/*
  set linear acceleration and yaw rate. Pass NaN for yaw_rate_rads to not control yaw
  acceleration is in earth frame, NED, m/s/s
*/
bool AP_ExternalControl_Copter::set_acceleration_and_yaw_rate(const Vector3f &linear_accel_ned_mss, float yaw_rate_rads)
{
    if (!ready_for_external_control()) {
        return false;
    }
    const float checked_yaw_rate_rad = isnan(yaw_rate_rads)? 0: yaw_rate_rads;

    copter.mode_guided.set_accel_NED_mss(linear_accel_ned_mss, false, 0, !isnan(yaw_rate_rads), checked_yaw_rate_rad, false, true);
    return true;
}

/*
  set target attitude (quaternion, earth frame NED) and normalized collective
  thrust [0,1], via GUIDED's Angle submode — bypasses AC_PosControl entirely.
*/
bool AP_ExternalControl_Copter::set_attitude_and_thrust(const Quaternion &attitude_ned, const Vector3f &ang_vel_body_rads, float thrust_norm)
{
    if (!ready_for_external_control()) {
        return false;
    }
    copter.mode_guided.set_angle(attitude_ned, ang_vel_body_rads, thrust_norm, true);
    return true;
}

bool AP_ExternalControl_Copter::set_global_position(const Location& loc)
{
    // Check if copter is ready for external control and returns false if it is not.
    if (!ready_for_external_control()) {
        return false;
    }
    return copter.set_target_location(loc);
}

bool AP_ExternalControl_Copter::ready_for_external_control()
{
    return copter.flightmode->in_guided_mode() && copter.motors->armed();
}

#endif // AP_EXTERNAL_CONTROL_ENABLED
