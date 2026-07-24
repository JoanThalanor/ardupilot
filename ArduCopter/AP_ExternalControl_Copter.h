/*
  external control library for copter
 */
#pragma once

#include <AP_ExternalControl/AP_ExternalControl.h>

#if AP_EXTERNAL_CONTROL_ENABLED

class AP_ExternalControl_Copter : public AP_ExternalControl
{
public:
    /*
      Set linear velocity and yaw rate. Pass NaN for yaw_rate_rads to not control yaw.
      Velocity is in earth frame, NED [m/s].
      Yaw is in earth frame, NED [rad/s].
     */
    bool set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity_ned_ms, float yaw_rate_rads) override WARN_IF_UNUSED;

    /*
      Set linear acceleration (earth frame NED, m/s/s) and yaw rate (rad/s).
      Pass NaN for yaw_rate_rads to not control yaw.
     */
    bool set_acceleration_and_yaw_rate(const Vector3f &linear_accel_ned_mss, float yaw_rate_rads) override WARN_IF_UNUSED;

    /*
      Sets the target global position for a loiter point.
    */
    bool set_global_position(const Location& loc) override WARN_IF_UNUSED;

    /*
      Set target attitude (quaternion, earth frame NED) and normalized
      collective thrust [0,1]. Bypasses AC_PosControl entirely.
     */
    bool set_attitude_and_thrust(const Quaternion &attitude_ned, const Vector3f &ang_vel_body_rads, float thrust_norm) override WARN_IF_UNUSED;
private:
    /*
      Return true if Copter is ready to handle external control data.
      Currently checks mode and arm states.
    */
    bool ready_for_external_control() WARN_IF_UNUSED;
};

#endif // AP_EXTERNAL_CONTROL_ENABLED
