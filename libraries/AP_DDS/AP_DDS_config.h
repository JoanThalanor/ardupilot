#pragma once

#include <AP_GPS/AP_GPS_config.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Networking/AP_Networking_Config.h>
#include <AP_VisualOdom/AP_VisualOdom_config.h>
#include <RC_Channel/RC_Channel_config.h>
#include <AP_RSSI/AP_RSSI_config.h>

#ifndef AP_DDS_ENABLED
#define AP_DDS_ENABLED 1
#endif

// UDP only on SITL for now
#ifndef AP_DDS_UDP_ENABLED
#define AP_DDS_UDP_ENABLED AP_DDS_ENABLED && AP_NETWORKING_ENABLED
#endif

#include <AP_VisualOdom/AP_VisualOdom_config.h>
#ifndef AP_DDS_VISUALODOM_ENABLED
#define AP_DDS_VISUALODOM_ENABLED HAL_VISUALODOM_ENABLED && AP_DDS_ENABLED
#endif

// Whether experimental interfaces are enabled.
#ifndef AP_DDS_EXPERIMENTAL_ENABLED
#define AP_DDS_EXPERIMENTAL_ENABLED 1
#endif

#ifndef AP_DDS_IMU_PUB_ENABLED
#define AP_DDS_IMU_PUB_ENABLED AP_DDS_EXPERIMENTAL_ENABLED
#endif

#ifndef AP_DDS_DELAY_IMU_TOPIC_MS
#define AP_DDS_DELAY_IMU_TOPIC_MS 5
#endif

#ifndef AP_DDS_TIME_PUB_ENABLED
#define AP_DDS_TIME_PUB_ENABLED 1
#endif

#ifndef AP_DDS_DELAY_TIME_TOPIC_MS
#define AP_DDS_DELAY_TIME_TOPIC_MS 10
#endif

#ifndef AP_DDS_NAVSATFIX_PUB_ENABLED
#define AP_DDS_NAVSATFIX_PUB_ENABLED AP_GPS_ENABLED
#endif

#ifndef AP_DDS_STATIC_TF_PUB_ENABLED
#define AP_DDS_STATIC_TF_PUB_ENABLED AP_GPS_ENABLED
#endif

#ifndef AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
#define AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED 1
#endif

#ifndef AP_DDS_DELAY_GPS_GLOBAL_ORIGIN_TOPIC_MS
#define AP_DDS_DELAY_GPS_GLOBAL_ORIGIN_TOPIC_MS 1000
#endif

#ifndef AP_DDS_GEOPOSE_PUB_ENABLED
#define AP_DDS_GEOPOSE_PUB_ENABLED 1
#endif

#ifndef AP_DDS_DELAY_GEO_POSE_TOPIC_MS
#define AP_DDS_DELAY_GEO_POSE_TOPIC_MS 33
#endif

#ifndef AP_DDS_LOCAL_POSE_PUB_ENABLED
#define AP_DDS_LOCAL_POSE_PUB_ENABLED 1
#endif

#ifndef AP_DDS_DELAY_LOCAL_POSE_TOPIC_MS
// TEMP diagnostic edit (2026-07-29): forcing 10ms directly in-source to verify
// whether waf's --define=AP_DDS_DELAY_LOCAL_POSE_TOPIC_MS=10 build-flag override
// was actually taking effect (measured rate stayed ~30Hz despite the configure
// log confirming the flag was passed) — REVERT TO 33 before any hardware build,
// see the real-hardware bandwidth-saturation reasoning below.
//
// Reverted to stock 33ms (2026-07-27): 10ms saturated the single shared
// reliable XRCE stream (8x512B history) on real hardware over a 921600 baud
// UART — geometry_msgs_msg_PoseStamped_serialize_topic() succeeded a handful
// of times at boot then failed on every subsequent call, forever (confirmed
// with a debug counter). Never showed up in SITL, which isn't bandwidth
// bound. See AP_DDS_Client.cpp's write_local_pose_topic() TODO.
#define AP_DDS_DELAY_LOCAL_POSE_TOPIC_MS 10
#endif

#ifndef AP_DDS_LOCAL_VEL_PUB_ENABLED
#define AP_DDS_LOCAL_VEL_PUB_ENABLED 1
#endif

#ifndef AP_DDS_DELAY_LOCAL_VELOCITY_TOPIC_MS
// See AP_DDS_DELAY_LOCAL_POSE_TOPIC_MS above — kept in lockstep with it.
#define AP_DDS_DELAY_LOCAL_VELOCITY_TOPIC_MS 10
#endif

#ifndef AP_DDS_AIRSPEED_PUB_ENABLED
#define AP_DDS_AIRSPEED_PUB_ENABLED 1
#endif

#ifndef AP_DDS_DELAY_AIRSPEED_TOPIC_MS
#define AP_DDS_DELAY_AIRSPEED_TOPIC_MS 33
#endif

#ifndef AP_DDS_RC_PUB_ENABLED
#define AP_DDS_RC_PUB_ENABLED (AP_RSSI_ENABLED && AP_RC_CHANNEL_ENABLED)
#endif

#ifndef AP_DDS_DELAY_RC_TOPIC_MS
#define AP_DDS_DELAY_RC_TOPIC_MS 100
#endif

#ifndef AP_DDS_BATTERY_STATE_PUB_ENABLED
#define AP_DDS_BATTERY_STATE_PUB_ENABLED 1
#endif

#ifndef AP_DDS_DELAY_BATTERY_STATE_TOPIC_MS
#define AP_DDS_DELAY_BATTERY_STATE_TOPIC_MS 1000
#endif

#ifndef AP_DDS_DELAY_STATUS_TOPIC_MS
#define AP_DDS_DELAY_STATUS_TOPIC_MS 100
#endif

#ifndef AP_DDS_CLOCK_PUB_ENABLED
#define AP_DDS_CLOCK_PUB_ENABLED 1
#endif

#ifndef AP_DDS_DELAY_CLOCK_TOPIC_MS
#define AP_DDS_DELAY_CLOCK_TOPIC_MS 10
#endif

#ifndef AP_DDS_CLOCK_SUB_ENABLED
#define AP_DDS_CLOCK_SUB_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef AP_DDS_GOAL_PUB_ENABLED
#define AP_DDS_GOAL_PUB_ENABLED 1
#endif

#ifndef AP_DDS_DELAY_GOAL_TOPIC_MS
#define AP_DDS_DELAY_GOAL_TOPIC_MS  200
#endif
#ifndef AP_DDS_STATUS_PUB_ENABLED
#define AP_DDS_STATUS_PUB_ENABLED 1
#endif

#ifndef AP_DDS_JOY_SUB_ENABLED
#define AP_DDS_JOY_SUB_ENABLED 1
#endif

#ifndef AP_DDS_VEL_CTRL_ENABLED
#define AP_DDS_VEL_CTRL_ENABLED 1
#endif

#ifndef AP_DDS_GLOBAL_POS_CTRL_ENABLED
#define AP_DDS_GLOBAL_POS_CTRL_ENABLED 1
#endif

// True acceleration setpoint control (REP-147 style), separate from
// AP_DDS_VEL_CTRL_ENABLED's velocity-only /ap/cmd_vel.
#ifndef AP_DDS_ACCEL_CTRL_ENABLED
#define AP_DDS_ACCEL_CTRL_ENABLED 1
#endif

// Direct attitude+thrust control (mavros_msgs/AttitudeTarget); unlike
// AP_DDS_ACCEL_CTRL_ENABLED, bypasses AC_PosControl's cascade entirely.
#ifndef AP_DDS_ATTITUDE_CTRL_ENABLED
#define AP_DDS_ATTITUDE_CTRL_ENABLED 1
#endif

#ifndef AP_DDS_DYNAMIC_TF_SUB_ENABLED
#define AP_DDS_DYNAMIC_TF_SUB_ENABLED 1
#endif

#ifndef AP_DDS_ARM_SERVER_ENABLED
#define AP_DDS_ARM_SERVER_ENABLED 1
#endif

#ifndef AP_DDS_MODE_SWITCH_SERVER_ENABLED
#define AP_DDS_MODE_SWITCH_SERVER_ENABLED 1
#endif

#ifndef AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED
#define AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED 1
#endif

#ifndef AP_DDS_PARAMETER_SERVER_ENABLED
#define AP_DDS_PARAMETER_SERVER_ENABLED 1
#endif

#ifndef AP_DDS_ARM_CHECK_SERVER_ENABLED
#define AP_DDS_ARM_CHECK_SERVER_ENABLED 1
#endif

// Whether to include Twist support
#define AP_DDS_NEEDS_TWIST AP_DDS_VEL_CTRL_ENABLED || AP_DDS_LOCAL_VEL_PUB_ENABLED

// Whether to include Transform support
#define AP_DDS_NEEDS_TRANSFORMS AP_DDS_DYNAMIC_TF_SUB_ENABLED || AP_DDS_STATIC_TF_PUB_ENABLED

// Whether DDS needs GPS
#define AP_DDS_NEEDS_GPS AP_DDS_NAVSATFIX_PUB_ENABLED || AP_DDS_STATIC_TF_PUB_ENABLED

#ifndef AP_DDS_DEFAULT_UDP_IP_ADDR
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#define AP_DDS_DEFAULT_UDP_IP_ADDR "192.168.144.2"
#else
#define AP_DDS_DEFAULT_UDP_IP_ADDR "127.0.0.1"
#endif
#endif

// Max DDS topic/service string
#ifndef AP_DDS_MAX_NAME_LEN
#define AP_DDS_MAX_NAME_LEN 128
#endif
