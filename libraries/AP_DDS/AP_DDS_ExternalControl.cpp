#include "AP_DDS_config.h"

#if AP_DDS_ENABLED

#include "AP_DDS_ExternalControl.h"
#include "AP_DDS_Frames.h"
#include <AP_AHRS/AP_AHRS.h>

#include <AP_ExternalControl/AP_ExternalControl.h>

bool AP_DDS_External_Control::handle_global_position_control(ardupilot_msgs_msg_GlobalPosition& cmd_pos)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    if (strcmp(cmd_pos.header.frame_id, MAP_FRAME) == 0) {
        // Narrow the altitude
        const int32_t alt_cm  = static_cast<int32_t>(cmd_pos.altitude * 100);

        Location::AltFrame alt_frame;
        if (!convert_alt_frame(cmd_pos.coordinate_frame, alt_frame)) {
            return false;
        }

        constexpr uint32_t MASK_POS_IGNORE =
            GlobalPosition::IGNORE_LATITUDE |
            GlobalPosition::IGNORE_LONGITUDE |
            GlobalPosition::IGNORE_ALTITUDE;

        if (!(cmd_pos.type_mask & MASK_POS_IGNORE)) {
            Location loc(cmd_pos.latitude * 1E7, cmd_pos.longitude * 1E7, alt_cm, alt_frame);
            if (!external_control->set_global_position(loc)) {
                return false; // Don't try sending other commands if this fails
            }
        }

        // TODO add velocity and accel handling

        return true;
    }

    return false;
}

bool AP_DDS_External_Control::handle_velocity_control(geometry_msgs_msg_TwistStamped& cmd_vel)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    if (strcmp(cmd_vel.header.frame_id, BASE_LINK_FRAME_ID) == 0) {
        // Convert commands from body frame (x-forward, y-left, z-up) to NED.
        Vector3f linear_velocity;
        Vector3f linear_velocity_base_link {
            float(cmd_vel.twist.linear.x),
            float(cmd_vel.twist.linear.y),
            float(-cmd_vel.twist.linear.z) };

        if (isnan(linear_velocity_base_link.y) && isnan(linear_velocity_base_link.z)) {
            // Assume it's an airspeed command so ignore the angular data.
            // While MAV_CMD_GUIDED_CHANGE_SPEED supports commands of ground speed and airspeed,
            // ROS users likely care more about airspeed control for a low level velocity control interface like this.
            return external_control->set_airspeed(linear_velocity_base_link.x);
        }

        const float yaw_rate = -cmd_vel.twist.angular.z;

        auto &ahrs = AP::ahrs();
        linear_velocity = ahrs.body_to_earth(linear_velocity_base_link);
        return external_control->set_linear_velocity_and_yaw_rate(linear_velocity, yaw_rate);
    }

    else if (strcmp(cmd_vel.header.frame_id, MAP_FRAME) == 0) {
        // Convert commands from ENU to NED frame
        Vector3f linear_velocity {
            float(cmd_vel.twist.linear.y),
            float(cmd_vel.twist.linear.x),
            float(-cmd_vel.twist.linear.z) };
        const float yaw_rate = -cmd_vel.twist.angular.z;
        return external_control->set_linear_velocity_and_yaw_rate(linear_velocity, yaw_rate);
    }

    return false;
}

#if AP_DDS_ACCEL_CTRL_ENABLED
bool AP_DDS_External_Control::handle_acceleration_control(geometry_msgs_msg_AccelStamped& cmd_accel)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    const float yaw_rate = -cmd_accel.accel.angular.z;

    if (strcmp(cmd_accel.header.frame_id, BASE_LINK_FRAME_ID) == 0) {
        // Convert commands from body frame (x-forward, y-left, z-up) to NED.
        Vector3f linear_accel_base_link {
            float(cmd_accel.accel.linear.x),
            float(cmd_accel.accel.linear.y),
            float(-cmd_accel.accel.linear.z) };

        auto &ahrs = AP::ahrs();
        const Vector3f linear_accel_ned = ahrs.body_to_earth(linear_accel_base_link);
        return external_control->set_acceleration_and_yaw_rate(linear_accel_ned, yaw_rate);
    }

    if (strcmp(cmd_accel.header.frame_id, MAP_FRAME) == 0) {
        // Convert commands from ENU to NED frame
        Vector3f linear_accel_ned {
            float(cmd_accel.accel.linear.y),
            float(cmd_accel.accel.linear.x),
            float(-cmd_accel.accel.linear.z) };
        return external_control->set_acceleration_and_yaw_rate(linear_accel_ned, yaw_rate);
    }

    return false;
}
#endif // AP_DDS_ACCEL_CTRL_ENABLED

#if AP_DDS_ATTITUDE_CTRL_ENABLED
bool AP_DDS_External_Control::handle_attitude_control(mavros_msgs_msg_AttitudeTarget& cmd_att)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    if (strcmp(cmd_att.header.frame_id, MAP_FRAME) != 0) {
        return false;
    }

    if (cmd_att.type_mask & AttitudeTarget::IGNORE_ATTITUDE) {
        // This path is specifically for attitude+thrust control — an
        // orientation-less request has nothing for us to do.
        return false;
    }

    // Inverse of AP_DDS_Client's local-pose publishing transform (NED->ENU +
    // Z-axis 90deg rotation, AP_DDS_Client.cpp's populate_pose_topic): here we
    // go ENU (ROS REP-103, as received) back to NED (ArduPilot native,
    // as ModeGuided::set_angle()/attitude_control->input_quaternion() expect).
    const Quaternion orientation_enu {
        float(cmd_att.orientation.w), float(cmd_att.orientation.x),
        float(cmd_att.orientation.y), float(cmd_att.orientation.z) };
    const Quaternion transformation_inv (sqrtF(2) * 0.5, 0, 0, -sqrtF(2) * 0.5); // inverse Z-axis 90deg rotation
    const Quaternion rotated = orientation_enu * transformation_inv;
    const Quaternion attitude_ned (rotated[0], rotated[2], rotated[1], -rotated[3]); // self-inverse ENU<->NED swap

    // body_rate: ROS REP-103 body FLU (x-forward,y-left,z-up) -> ArduPilot
    // body FRD (x-forward,y-right,z-down) — negate y and z, no axis swap.
    Vector3f ang_vel_body_rads {0.0f, 0.0f, 0.0f};
    if (!(cmd_att.type_mask & AttitudeTarget::IGNORE_ROLL_RATE)) {
        ang_vel_body_rads.x = float(cmd_att.body_rate.x);
    }
    if (!(cmd_att.type_mask & AttitudeTarget::IGNORE_PITCH_RATE)) {
        ang_vel_body_rads.y = -float(cmd_att.body_rate.y);
    }
    if (!(cmd_att.type_mask & AttitudeTarget::IGNORE_YAW_RATE)) {
        ang_vel_body_rads.z = -float(cmd_att.body_rate.z);
    }

    const float thrust_norm = (cmd_att.type_mask & AttitudeTarget::IGNORE_THRUST) ? 0.0f : float(cmd_att.thrust);

    return external_control->set_attitude_and_thrust(attitude_ned, ang_vel_body_rads, thrust_norm);
}
#endif // AP_DDS_ATTITUDE_CTRL_ENABLED

bool AP_DDS_External_Control::arm(AP_Arming::Method method, bool do_arming_checks)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    return external_control->arm(method, do_arming_checks);
}

bool AP_DDS_External_Control::disarm(AP_Arming::Method method, bool do_disarm_checks)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    return external_control->disarm(method, do_disarm_checks);
}

bool AP_DDS_External_Control::convert_alt_frame(const uint8_t frame_in,  Location::AltFrame& frame_out)
{

    // Specified in ROS REP-147; only some are supported.
    switch (frame_in) {
    case 5: // FRAME_GLOBAL_INT
        frame_out = Location::AltFrame::ABSOLUTE;
        break;
    case 6: // FRAME_GLOBAL_REL_ALT
        frame_out = Location::AltFrame::ABOVE_HOME;
        break;
    case 11: // FRAME_GLOBAL_TERRAIN_ALT
        frame_out = Location::AltFrame::ABOVE_TERRAIN;
        break;
    default:
        return false;
    }
    return true;
}


#endif // AP_DDS_ENABLED