/********************************************************************
* Description: tc.h
*   Discriminate-based trajectory planning
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
********************************************************************/
#ifndef TC_TYPES_H
#define TC_TYPES_H

#include <posemath.h>
#include <emcpos.h>
#include <emcmotcfg.h>

#include "spherical_arc.h"
#include "../motion/state_tag.h"

#define BLEND_DIST_FRACTION 0.5
/* values for endFlag */
typedef enum {
    TC_TERM_COND_STOP = 0,
    TC_TERM_COND_EXACT = 1,
    TC_TERM_COND_PARABOLIC = 2,
    TC_TERM_COND_TANGENT = 3
} tc_term_cond_t;

typedef enum {
    TC_LINEAR = 1,
    TC_CIRCULAR = 2,
    TC_RIGIDTAP = 3,
    TC_SPHERICAL = 4
} tc_motion_type_t;

typedef enum {
    TC_SYNC_NONE = 0,
    TC_SYNC_VELOCITY,
    TC_SYNC_POSITION
} tc_spindle_sync_t;

typedef enum {
    TC_DIR_FORWARD = 0,
    TC_DIR_REVERSE
} tc_direction_t;

#define TC_GET_PROGRESS 0
#define TC_GET_STARTPOINT 1
#define TC_GET_ENDPOINT 2

#define TC_OPTIM_UNTOUCHED 0
#define TC_OPTIM_AT_MAX 1

#define TC_ACCEL_TRAPZ 0
#define TC_ACCEL_RAMP 1

/**
 * Spiral arc length approximation by quadratic fit.
 */
typedef struct {
    double b0;                  /* 2nd order coefficient */
    double b1;                  /* 1st order coefficient */
    double total_planar_length; /* total arc length in plane */
    int spiral_in;              /* flag indicating spiral is inward,
                                   rather than outward */
} SpiralArcLengthFit;


/* structure for individual trajectory elements */

typedef struct {
    PmCartLine xyz;
    PmCartLine abc;
    PmCartLine uvw;
} PmLine9;

typedef struct {
    PmCircle xyz;
    PmCartLine abc;
    PmCartLine uvw;
    SpiralArcLengthFit fit;
} PmCircle9;

typedef struct {
    SphericalArc xyz;
    PmCartesian abc;
    PmCartesian uvw;
} Arc9;

typedef enum {
    RIGIDTAP_START,
    TAPPING, REVERSING, RETRACTION, FINAL_REVERSAL, FINAL_PLACEMENT
} RIGIDTAP_STATE;

typedef unsigned long long iomask_t; // 64 bits on both x86 and x86_64

typedef struct {
    char anychanged;
    iomask_t dio_mask;
    iomask_t aio_mask;
    signed char dios[EMCMOT_MAX_DIO];
    double aios[EMCMOT_MAX_AIO];
} syncdio_t;

typedef struct {
    PmCartLine xyz;             // original, but elongated, move down
    PmCartLine aux_xyz;         // this will be generated on the fly, for the other
                            // two moves: retraction, final placement
    PmCartesian abc;
    PmCartesian uvw;
    double reversal_target;
    double reversal_scale;
    double spindlerevs_at_reversal;
    RIGIDTAP_STATE state;
} PmRigidTap;

typedef struct {
    // ====================================================================
    // HOT DATA: Frequently accessed during optimization (single L1 cache line)
    // Fields are ordered to fit within 64 bytes for single cache line
    // ====================================================================
    struct {
        double target;           // actual segment length
        double progress;         // where are we in the segment? 0..target
        double finalvel;         // velocity to aim for at end of segment
        double finalacc;         // acceleration to aim for at end of segment
        double maxvel;           // max possible vel (feed override stops here)
        double currentvel;       // keep track of current step (vel * cycle_time)
        double currentacc;       // current acceleration for S-curve planning
        // Total: 56 bytes (fits in one 64-byte L1 cache line)
    } hot;

    // ====================================================================
    // COLD DATA: Infrequently accessed (regular top-level members)
    // ====================================================================
    double cycle_time;
    double nominal_length;
    double reqvel;           // vel requested by F word, calc'd by task
    double target_vel;       // velocity to actually track, limited by other factors
    double last_move_length; // last move length
    double term_vel;         // actual velocity at termination of segment
    double kink_vel;         // Temp storage for max velocity for tangent declaration
    double kink_accel_reduce_prev; // Accel reduction for approximate tangent
    double kink_accel_reduce;      // Accel reduction for approximate tangent
    double factor;
    double targetvel;
    double vt;
    double maxjerk;          // max jerk for S-curve motion
    double blend_maxjerk;    // max jerk during blend (set by look-ahead)
    double currentjerk;      // current jerk for S-curve planning
    double lastacc;
    double maxaccel;         // accel calc'd by task
    double acc_ratio_tan;    // ratio between normal and tangential accel
    double blend_vel;        // velocity below which we should start blending
    double tolerance;        // distance tolerance during blend
    double uu_per_rev;       // for sync, user units per rev
    double vel_at_blend_start;
    double ruckig_trajectory_time;  // current trajectory time
    double ruckig_last_maxaccel;    // max acceleration used in last planning
    double ruckig_last_maxjerk;     // max jerk used in last planning
    double ruckig_last_target_vel;  // target velocity used in last planning
    double ruckig_last_final_vel;   // final velocity used in last planning
    double ruckig_last_final_acc;   // final acceleration used in last planning
    double ruckig_last_target_pos;  // target position used in last planning
    double ruckig_last_req_pos;     // last req_pos value (for velocity control)
    double ruckig_last_feed_override; // feed override at last planning

    int id;                  // segment's serial number
    struct state_tag_t tag;  // state tag corresponding to running motion

    union {                  // describes the segment's start and end positions
        PmLine9 line;
        PmCircle9 circle;
        PmRigidTap rigidtap;
        Arc9 arc;
    } coords;

    int motion_type;         // TC_LINEAR/CIRCULAR/RIGIDTAP
    int active;              // this motion is being executed
    int canon_motion_type;   // this motion is due to which canon function?
    int term_cond;           // gcode requests continuous feed at end

    int blending_next;       // segment is being blended into following segment
    int synchronized;        // spindle sync state
    int sync_accel;          // we're accelerating up to sync with the spindle
    unsigned char enables;   // Feed scale, etc, enable bits for this move
    int atspeed;             // wait for spindle to be at-speed
    syncdio_t syncdio;       // synched DIO's for this move
    int indexer_jnum;        // which joint to unlock (locking indexer), -1 for none
    int optimization_state;  // At peak velocity during blends
    int on_final_decel;
    int blend_prev;
    int accel_mode;
    int splitting;           // segment is < 1 cycle time from end
    int remove;              // Flag to remove segment from queue
    int active_depth;        // how many segments until zero speed
    int finalized;

    int is_blending;         // Temporary status flag (reset each cycle)

    void *ruckig_planner;    // Ruckig planner handle (opaque pointer)
    int ruckig_planned;      // whether Ruckig planning completed
    int ruckig_last_use_velocity_control; // control mode (1=velocity, 0=position)
} TC_STRUCT;

#endif				/* TC_TYPES_H */
