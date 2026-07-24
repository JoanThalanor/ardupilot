#pragma once

#if AP_DDS_ENABLED
#include "ardupilot_msgs/msg/GlobalPosition.h"
#include "geometry_msgs/msg/TwistStamped.h"
#if AP_DDS_ACCEL_CTRL_ENABLED
#include "geometry_msgs/msg/AccelStamped.h"
#endif // AP_DDS_ACCEL_CTRL_ENABLED
#if AP_DDS_ATTITUDE_CTRL_ENABLED
#include "mavros_msgs/msg/AttitudeTarget.h"
#endif // AP_DDS_ATTITUDE_CTRL_ENABLED
#include <AP_Arming/AP_Arming.h>
#include <AP_Common/Location.h>

class AP_DDS_External_Control
{
public:
    // REP-147 Goal Interface Global Position Control
    // https://ros.org/reps/rep-0147.html#goal-interface
    static bool handle_global_position_control(ardupilot_msgs_msg_GlobalPosition& cmd_pos);
    static bool handle_velocity_control(geometry_msgs_msg_TwistStamped& cmd_vel);
#if AP_DDS_ACCEL_CTRL_ENABLED
    // Acceleration setpoint control — see docs/ardupilot_dds_acceleration_control.md.
    static bool handle_acceleration_control(geometry_msgs_msg_AccelStamped& cmd_accel);
#endif // AP_DDS_ACCEL_CTRL_ENABLED
#if AP_DDS_ATTITUDE_CTRL_ENABLED
    // Direct attitude+thrust setpoint control, bypassing AC_PosControl — see
    // docs/ardupilot_dds_acceleration_control.md §7.
    static bool handle_attitude_control(mavros_msgs_msg_AttitudeTarget& cmd_att);
#endif // AP_DDS_ATTITUDE_CTRL_ENABLED
    static bool arm(AP_Arming::Method method, bool do_arming_checks);
    static bool disarm(AP_Arming::Method method, bool do_disarm_checks);

private:
    static bool convert_alt_frame(const uint8_t frame_in,  Location::AltFrame& frame_out);
};
#endif // AP_DDS_ENABLED
