/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.7-dev */

#ifndef PB_LANE_PB_H_INCLUDED
#define PB_LANE_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _LaneStatus {
    LaneStatus_FORWARD = 0,
    LaneStatus_BACKWARD = 1,
    LaneStatus_STOP = 2,
    LaneStatus_BLINK = 3,
    LaneStatus_KEEP = 4
} LaneStatus;

/* Struct definitions */
typedef struct _LaneColorConfig {
    /* first byte reserved.
 second byte is the red value,
 third byte is the green value,
 fourth byte is the blue value */
    uint32_t rgb;
} LaneColorConfig;

typedef struct _LaneLengthConfig {
    float total_length_m;
    float line_length_m;
    float active_length_m;
    uint32_t line_leds_num;
    uint32_t finish_time_m;
    uint32_t head_offset; /* we could calculate the distance between each led with the above two */
} LaneLengthConfig;

typedef struct _LaneSetSpeed {
    /* in m/s */
    double speed;
} LaneSetSpeed;

typedef struct _LaneSetStatus {
    LaneStatus status;
} LaneSetStatus;

/* go through Control Characteristic via Notify/Read */
typedef struct _LaneState {
    float shift;
    float speed;
    float head;
    float tail;
    /* / the distance between the start/end of the lane and the head of trail
/ if the status is FORWARD, the head is the distance from the start
/ if the status is BACKWARD, the head is the distance from the end */
    LaneStatus status;
} LaneState;

typedef struct _PbLanePace {
    uint32_t pace_num;
    pb_size_t pace_time_count;
    float pace_time[5];
    float else_deal;
    float platform_surface_time;
    float platform_surface_range;
    float acceleration;
    float turn_time;
} PbLanePace;

/* use to config the lane
 go through Config Characteristic */
typedef struct _LaneConfig {
    pb_size_t which_msg;
    union {
        LaneLengthConfig length_cfg;
        LaneColorConfig color_cfg;
    } msg;
} LaneConfig;

/* LaneConfigRO is the read-only version of LaneConfig
 which contains all of the current config of the lane */
typedef struct _LaneConfigRO {
    bool has_length_cfg;
    LaneLengthConfig length_cfg;
    bool has_color_cfg;
    LaneColorConfig color_cfg;
} LaneConfigRO;

/* use to control the lane
 go through Control Characteristic vid Write. The lane would clear the characteristic after processing and
 replace the buffer with `LaneState` */
typedef struct _LaneControl {
    pb_size_t which_msg;
    union {
        LaneSetStatus set_status;
        LaneSetSpeed set_speed;
    } msg;
} LaneControl;


/* Helper constants for enums */
#define _LaneStatus_MIN LaneStatus_FORWARD
#define _LaneStatus_MAX LaneStatus_KEEP
#define _LaneStatus_ARRAYSIZE ((LaneStatus)(LaneStatus_KEEP+1))


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define LaneLengthConfig_init_default            {0, 0, 0, 0, 0, 0}
#define LaneColorConfig_init_default             {0}
#define LaneSetStatus_init_default               {_LaneStatus_MIN}
#define LaneSetSpeed_init_default                {0}
#define LaneState_init_default                   {0, 0, 0, 0, _LaneStatus_MIN}
#define LaneConfig_init_default                  {0, {LaneLengthConfig_init_default}}
#define LaneConfigRO_init_default                {false, LaneLengthConfig_init_default, false, LaneColorConfig_init_default}
#define LaneControl_init_default                 {0, {LaneSetStatus_init_default}}
#define PbLanePace_init_default                  {0, 0, {0, 0, 0, 0, 0}, 0, 0, 0, 0, 0}
#define LaneLengthConfig_init_zero               {0, 0, 0, 0, 0, 0}
#define LaneColorConfig_init_zero                {0}
#define LaneSetStatus_init_zero                  {_LaneStatus_MIN}
#define LaneSetSpeed_init_zero                   {0}
#define LaneState_init_zero                      {0, 0, 0, 0, _LaneStatus_MIN}
#define LaneConfig_init_zero                     {0, {LaneLengthConfig_init_zero}}
#define LaneConfigRO_init_zero                   {false, LaneLengthConfig_init_zero, false, LaneColorConfig_init_zero}
#define LaneControl_init_zero                    {0, {LaneSetStatus_init_zero}}
#define PbLanePace_init_zero                     {0, 0, {0, 0, 0, 0, 0}, 0, 0, 0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define LaneColorConfig_rgb_tag                  1
#define LaneLengthConfig_total_length_m_tag      1
#define LaneLengthConfig_line_length_m_tag       2
#define LaneLengthConfig_active_length_m_tag     3
#define LaneLengthConfig_line_leds_num_tag       4
#define LaneLengthConfig_finish_time_m_tag       5
#define LaneLengthConfig_head_offset_tag         6
#define LaneSetSpeed_speed_tag                   1
#define LaneSetStatus_status_tag                 1
#define LaneState_shift_tag                      1
#define LaneState_speed_tag                      2
#define LaneState_head_tag                       3
#define LaneState_tail_tag                       4
#define LaneState_status_tag                     5
#define PbLanePace_pace_num_tag                  1
#define PbLanePace_pace_time_tag                 2
#define PbLanePace_else_deal_tag                 3
#define PbLanePace_platform_surface_time_tag     4
#define PbLanePace_platform_surface_range_tag    5
#define PbLanePace_acceleration_tag              6
#define PbLanePace_turn_time_tag                 7
#define LaneConfig_length_cfg_tag                1
#define LaneConfig_color_cfg_tag                 2
#define LaneConfigRO_length_cfg_tag              1
#define LaneConfigRO_color_cfg_tag               2
#define LaneControl_set_status_tag               1
#define LaneControl_set_speed_tag                2

/* Struct field encoding specification for nanopb */
#define LaneLengthConfig_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    total_length_m,    1) \
X(a, STATIC,   SINGULAR, FLOAT,    line_length_m,     2) \
X(a, STATIC,   SINGULAR, FLOAT,    active_length_m,   3) \
X(a, STATIC,   SINGULAR, UINT32,   line_leds_num,     4) \
X(a, STATIC,   SINGULAR, UINT32,   finish_time_m,     5) \
X(a, STATIC,   SINGULAR, UINT32,   head_offset,       6)
#define LaneLengthConfig_CALLBACK NULL
#define LaneLengthConfig_DEFAULT NULL

#define LaneColorConfig_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   rgb,               1)
#define LaneColorConfig_CALLBACK NULL
#define LaneColorConfig_DEFAULT NULL

#define LaneSetStatus_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UENUM,    status,            1)
#define LaneSetStatus_CALLBACK NULL
#define LaneSetStatus_DEFAULT NULL

#define LaneSetSpeed_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, DOUBLE,   speed,             1)
#define LaneSetSpeed_CALLBACK NULL
#define LaneSetSpeed_DEFAULT NULL

#define LaneState_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    shift,             1) \
X(a, STATIC,   SINGULAR, FLOAT,    speed,             2) \
X(a, STATIC,   SINGULAR, FLOAT,    head,              3) \
X(a, STATIC,   SINGULAR, FLOAT,    tail,              4) \
X(a, STATIC,   SINGULAR, UENUM,    status,            5)
#define LaneState_CALLBACK NULL
#define LaneState_DEFAULT NULL

#define LaneConfig_FIELDLIST(X, a) \
X(a, STATIC,   ONEOF,    MESSAGE,  (msg,length_cfg,msg.length_cfg),   1) \
X(a, STATIC,   ONEOF,    MESSAGE,  (msg,color_cfg,msg.color_cfg),   2)
#define LaneConfig_CALLBACK NULL
#define LaneConfig_DEFAULT NULL
#define LaneConfig_msg_length_cfg_MSGTYPE LaneLengthConfig
#define LaneConfig_msg_color_cfg_MSGTYPE LaneColorConfig

#define LaneConfigRO_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  length_cfg,        1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  color_cfg,         2)
#define LaneConfigRO_CALLBACK NULL
#define LaneConfigRO_DEFAULT NULL
#define LaneConfigRO_length_cfg_MSGTYPE LaneLengthConfig
#define LaneConfigRO_color_cfg_MSGTYPE LaneColorConfig

#define LaneControl_FIELDLIST(X, a) \
X(a, STATIC,   ONEOF,    MESSAGE,  (msg,set_status,msg.set_status),   1) \
X(a, STATIC,   ONEOF,    MESSAGE,  (msg,set_speed,msg.set_speed),   2)
#define LaneControl_CALLBACK NULL
#define LaneControl_DEFAULT NULL
#define LaneControl_msg_set_status_MSGTYPE LaneSetStatus
#define LaneControl_msg_set_speed_MSGTYPE LaneSetSpeed

#define PbLanePace_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   pace_num,          1) \
X(a, STATIC,   REPEATED, FLOAT,    pace_time,         2) \
X(a, STATIC,   SINGULAR, FLOAT,    else_deal,         3) \
X(a, STATIC,   SINGULAR, FLOAT,    platform_surface_time,   4) \
X(a, STATIC,   SINGULAR, FLOAT,    platform_surface_range,   5) \
X(a, STATIC,   SINGULAR, FLOAT,    acceleration,      6) \
X(a, STATIC,   SINGULAR, FLOAT,    turn_time,         7)
#define PbLanePace_CALLBACK NULL
#define PbLanePace_DEFAULT NULL

extern const pb_msgdesc_t LaneLengthConfig_msg;
extern const pb_msgdesc_t LaneColorConfig_msg;
extern const pb_msgdesc_t LaneSetStatus_msg;
extern const pb_msgdesc_t LaneSetSpeed_msg;
extern const pb_msgdesc_t LaneState_msg;
extern const pb_msgdesc_t LaneConfig_msg;
extern const pb_msgdesc_t LaneConfigRO_msg;
extern const pb_msgdesc_t LaneControl_msg;
extern const pb_msgdesc_t PbLanePace_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define LaneLengthConfig_fields &LaneLengthConfig_msg
#define LaneColorConfig_fields &LaneColorConfig_msg
#define LaneSetStatus_fields &LaneSetStatus_msg
#define LaneSetSpeed_fields &LaneSetSpeed_msg
#define LaneState_fields &LaneState_msg
#define LaneConfig_fields &LaneConfig_msg
#define LaneConfigRO_fields &LaneConfigRO_msg
#define LaneControl_fields &LaneControl_msg
#define PbLanePace_fields &PbLanePace_msg

/* Maximum encoded size of messages (where known) */
#define LaneColorConfig_size                     6
#define LaneConfigRO_size                        43
#define LaneConfig_size                          35
#define LaneControl_size                         11
#define LaneLengthConfig_size                    33
#define LaneSetSpeed_size                        9
#define LaneSetStatus_size                       2
#define LaneState_size                           22
#define PbLanePace_size                          56

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
