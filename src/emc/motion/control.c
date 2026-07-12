/********************************************************************
* Description: control.c
*   emcmotController() is the main loop running at the servo cycle
*   rate. All state logic and trajectory calcs are called from here.
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* Created on:
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
********************************************************************/

#define SWITCHKINS_DEBUG
#undef  SWITCHKINS_DEBUG

#ifdef SWITCHKINS_DEBUG
#include <stdio.h>  // rtpreempt only, consolidate to stderr
#include <stddef.h> // offsetof (MCHAN MC2b status snapshot ranges)
#endif

#include <rtapi.h>
#include <rtapi_math.h>
#include <rtapi_string.h> /* memcpy, mchan_update_status() status snapshot */
#include <hal.h>
#include <posemath.h>
#include <kinematics.h>  //for kinematicsSwitchable()
#include <motion_types.h>

#include "../tp/tp.h"
#include "simple_tp.h"
#include "motion.h"
#include "motion_struct.h" /* emcmot_struct_t (MCHAN MC2b mailbox/status) */
#include "mot_priv.h"
#include "config.h"
#include "homing.h"
#include "axis.h"
#include "state_tag.h"

// Mark strings for translation, but defer translation to userspace
#define _(s) (s)

static int    ext_offset_teleop_limit = 0;
static int    ext_offset_coord_limit  = 0;
static bool   coord_cubic_active = 0;
static int    switchkins_type = 0;
/* kinematics flags */
KINEMATICS_FORWARD_FLAGS fflags = 0;
KINEMATICS_INVERSE_FLAGS iflags = 0;

/*! \todo FIXME - debugging - uncomment the following line to log changes in
   JOINT_FLAG and MOTION_FLAG */
// #define WATCH_FLAGS 1


/***********************************************************************
*                  LOCAL VARIABLE DECLARATIONS                         *
************************************************************************/

/* the (nominal) period the last time the motion handler was invoked */
static unsigned long last_period = 0;

/* servo cycle time */
static double servo_period;

extern struct emcmot_status_t *emcmotStatus;

// *pcmd_p[0] is shorthand for emcmotStatus->carte_pos_cmd.tran.x
// *pcmd_p[1] is shorthand for emcmotStatus->carte_pos_cmd.tran.y
//  etc.
static double *pcmd_p[EMCMOT_MAX_AXIS];

/***********************************************************************
*                      LOCAL FUNCTION PROTOTYPES                       *
************************************************************************/

/* the following functions are called (in this order) by the main
   controller function.  They are an attempt to break the huge
   function (originally 1600 lines) into something a little easier
   to understand.
*/

/* 'process_inputs()' is responsible for reading hardware input
   signals (from the HAL) and doing basic processing on them.  In
   the case of position feedback, that means removing backlash or
   screw error comp and calculating the following error.  For
   switches, it means debouncing them and setting flags in the
   emcmotStatus structure.
*/
static void process_inputs(void);

/* 'joint_jog_abort_all()' if either jog-stop or jog-stop-immediate
   become True while jogging then the jog will abort.
   jog-stop will stop the active jog following the associated
   acceleration values.
   jog-stop-immediate will immediately stop jogging (potentially
   causing joint following errors).
*/
static void joint_jog_abort_all(bool immediate);

/* 'do forward kins()' takes the position feedback in joint coords
   and applies the forward kinematics to it to generate feedback
   in Cartesean coordinates.  It has code to handle machines that
   don't have forward kins, and other special cases, such as when
   the joints have not been homed.
*/
static void do_forward_kins(void);

/* probe inputs need to be handled after forward kins are run, since
   cartesian feedback position is latched when the probe fires, and it
   should be based on the feedback read in on this servo cycle.
*/
static void process_probe_inputs(void);

/* 'check_for_faults()' is responsible for detecting fault conditions
   such as limit switches, amp faults, following error, etc.  It only
   checks active axes.  It is also responsible for generating an error
   message.  (Later, once I understand the cmd/status/error interface
   better, it will probably generate error codes that can be passed
   up the architecture toward the GUI - printing error messages
   directly seems a little messy)
*/
static void check_for_faults(void);

/* 'set_operating_mode()' handles transitions between the operating
   modes, which are free, coordinated, and teleop.  This stuff needs
   to be better documented.  It is basically a state machine, with
   a current state, a desired state, and rules determining when the
   state can change.  It should be rewritten as such, but for now
   it consists of code copied exactly from emc1.
*/
static void set_operating_mode(void);

/* 'handle_jjogwheels()' reads jogwheels, decides if they should be
   enabled, and if so, changes the free mode planner's target position
   when the jogwheel(s) turn.
*/
static void handle_jjogwheels(void);

/* 'do_homing_sequence()' decides what, if anything, needs to be done
    related to multi-joint homing.

   no prototype here, implemented in homing.c, proto in mot_priv.h
*/

/* 'do_homing()' looks at the home_state field of each joint struct
    to decide what, if anything, needs to be done related to homing
    the joint.  Homing is implemented as a state machine, the exact
    sequence of states depends on the machine configuration.  It
    can be as simple as immediately setting the current position to
    zero, or a it can be a multi-step process (find switch, set
    approximate zero, back off switch, find index, set final zero,
    rapid to home position), or anywhere in between.

   no prototype here, implemented in homing.c, proto in mot_priv.h
*/

/* 'get_pos_cmds()' generates the position setpoints.  This includes
   calling the trajectory planner and interpolating its outputs.
*/
static void get_pos_cmds(long period);

/* MCHAN MC10/Phase4: waiting-M (M200-M229) rendezvous engine, run once per
 * servo cycle - matches arrived channels, releases them together, and
 * applies the error+hold deadlock timeout. */
static void mchan_run_rendezvous(void);

/* MCHAN MC31: interference guard - each servo cycle, transform each channel's
 * controlled point to world coords and detect co-occupancy of the keep-out
 * zone. I2 = detection + observability pins (warn-only). */
static void mchan_run_interference(void);

/* 'compute_screw_comp()' is responsible for calculating backlash and
   lead screw error compensation.  (Leadscrew error compensation is
   a more sophisticated version that includes backlash comp.)  It uses
   the velocity in emcmotStatus->joint_vel_cmd to determine which way
   each joint is moving, and the position in emcmotStatus->joint_pos_cmd
   to determine where the joint is at.  That information is used to
   create the compensation value that is added to the joint_pos_cmd
   to create motor_pos_cmd, and is subtracted from motor_pos_fb to
   get joint_pos_fb.  (This function does not add or subtract the
   compensation value, it only computes it.)  The basic compensation
   value is in backlash_corr, however has makes step changes when
   the direction reverses.  backlash_filt is a ramped version, and
   that is the one that is later added/subtracted from the position.
*/
static void compute_screw_comp(void);

/* 'output_to_hal()' writes the handles the final stages of the
   control function.  It applies screw comp and writes the
   final motor position to the HAL (which routes it to the PID
   loop).  It also drives other HAL outputs, and it writes a
   number of internal variables to HAL parameters so they can
   be observed with halscope and halmeter.
*/
static void output_to_hal(void);

/* 'update_status()' copies assorted status information to shared
   memory (the emcmotStatus structure) so that it is available to
   higher level code.
*/
static void update_status(void);

/* MCHAN (MC2b): per-channel status snapshots for secondary stacks */
static void mchan_update_status(void);
/* MCHAN: EmcPose axis-component setter (defined near the executor) */
static void mchan_pose_set_axis(EmcPose *p, int ax, double v);

static void handle_kinematicsSwitch(void);

/***********************************************************************
*                        PUBLIC FUNCTION CODE                          *
************************************************************************/

/*
  emcmotController() runs the trajectory and interpolation calculations
  each control cycle

  This function gets called at regular intervals - therefore it does NOT
  have a loop within it!

  Inactive axes are still calculated, but the PIDs are inhibited and
  the amp enable/disable are inhibited
  */
void emcmotController(void *arg, long period)
{
    (void)arg;
    static int do_once = 1;
    if (do_once) {
        pcmd_p[0] = &(emcmotStatus->carte_pos_cmd.tran.x);
        pcmd_p[1] = &(emcmotStatus->carte_pos_cmd.tran.y);
        pcmd_p[2] = &(emcmotStatus->carte_pos_cmd.tran.z);
        pcmd_p[3] = &(emcmotStatus->carte_pos_cmd.a);
        pcmd_p[4] = &(emcmotStatus->carte_pos_cmd.b);
        pcmd_p[5] = &(emcmotStatus->carte_pos_cmd.c);
        pcmd_p[6] = &(emcmotStatus->carte_pos_cmd.u);
        pcmd_p[7] = &(emcmotStatus->carte_pos_cmd.v);
        pcmd_p[8] = &(emcmotStatus->carte_pos_cmd.w);
        do_once = 0;
    }

    static long long int last = 0;

    long long int now = rtapi_get_time();
    long int this_run = (long int)(now - last);
    *(emcmot_hal_data->last_period) = this_run;

    // we need this for next time
    last = now;


    /* calculate servo period as a double - period is in integer nsec */
    servo_period = period * 0.000000001;

    if(period != (long)last_period) {
        emcmotSetCycleTime(period);
        last_period = period;
    }

    /* increment head count to indicate work in progress */
    emcmotStatus->head++;
    /* here begins the core of the controller */

    read_homing_in_pins(ALL_JOINTS);
    handle_kinematicsSwitch();
    process_inputs();
    do_forward_kins();
    process_probe_inputs();
    check_for_faults();
    set_operating_mode();
    if (!*emcmot_hal_data->jog_inhibit) {
        handle_jjogwheels();
    }
    if (!emcmotStatus->on_soft_limit && !*emcmot_hal_data->jog_inhibit) {  // change from teleop to move off joint soft limit
        axis_handle_jogwheels(GET_MOTION_TELEOP_FLAG(), GET_MOTION_ENABLE_FLAG(), get_homing_is_active());
    }
    if (   (emcmotStatus->motion_state == EMCMOT_MOTION_FREE)
        && do_homing()) {
        /* MCHAN: only CHANNEL 0's homing session may flip the GLOBAL
         * mode on completion (legacy behavior); a secondary channel's
         * session finishing must leave the machine state alone (it
         * polluted the global mode to TELEOP otherwise - found in the
         * MC4 acceptance run) */
        if (mchan_homing_session_ch == 0) {
            switch_to_teleop_mode();
        }
    }

    /* MCHAN: tick the secondary channels' planners every cycle. Their queues
     * stay empty until the per-channel command plumbing lands (MC2+), so this
     * is a cheap no-op pass that keeps every channel's planner clock aligned
     * with the servo thread. Loop body never runs at num_channels=1. */
    for (int mchan_ch = 1; mchan_ch < motion_num_channels; mchan_ch++) {
	tpRunCycle(&emcmotInternal->chan[mchan_ch].coord_tp, period);
    }

    get_pos_cmds(period);
    mchan_run_rendezvous();	/* MCHAN MC10/Phase4: waiting-M match/release/timeout */
    mchan_run_interference();	/* MCHAN MC31: interference zone co-occupancy detect */
    compute_screw_comp();
    *(emcmot_hal_data->eoffset_active) = axis_plan_external_offsets(servo_period, GET_MOTION_ENABLE_FLAG(), get_allhomed());
    output_to_hal();
    write_homing_out_pins(ALL_JOINTS);
    update_status();
    /* here ends the core of the controller */
    emcmotStatus->heartbeat++;
    /* set tail to head, to indicate work complete */
    emcmotStatus->tail = emcmotStatus->head;
    /* MCHAN (MC2b): publish the secondary channels' status views from the
     * now-complete global status + per-channel state (no-op at 1 channel) */
    mchan_update_status();
/* end of controller function */
}

/***********************************************************************
*                         LOCAL FUNCTION CODE                          *
************************************************************************/
/* The prototypes and documentation for these functions are located
   at the top of the file in the section called "local function
   prototypes"
*/

static bool joint_jog_is_active(void) {
    int jno;
    for (jno = 0; jno < EMCMOT_MAX_JOINTS; jno++) {
        if ( (&joints[jno])->kb_jjog_active || (&joints[jno])->wheel_jjog_active) {
            return 1;
        }
    }
    return 0;
}

static void handle_kinematicsSwitch(void) {
    int joint_num;
    int hal_switchkins_type = 0;

    if (!kinematicsSwitchable()) return;
    hal_switchkins_type = (int)*emcmot_hal_data->switchkins_type;
    if (switchkins_type == hal_switchkins_type) return;

    switchkins_type = hal_switchkins_type;

    emcmot_joint_t *jointKinsSwitch;
    double joint_posKinsSwitch[EMCMOT_MAX_JOINTS] = {0,};
    /* copy joint position feedback to local array */
    for (joint_num = 0; joint_num < emcmotConfig->numJoints; joint_num++) {
        /* point to joint struct */
        jointKinsSwitch = &joints[joint_num];
        /* copy feedback */
        joint_posKinsSwitch[joint_num] = jointKinsSwitch->pos_cmd;
    }

    if (kinematicsSwitch(switchkins_type)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"kinematicsSwitch() FAIL<%f>\n",
                        *emcmot_hal_data->switchkins_type);
        SET_MOTION_ERROR_FLAG(1);  // abort
        return; // no updates for abort
    }

    KINEMATICS_FORWARD_FLAGS tmpFFlags = fflags;
    KINEMATICS_INVERSE_FLAGS tmpIFlags = iflags;
#ifdef SWITCHKINS_DEBUG
    double beforePose[EMCMOT_MAX_AXIS];
    int anum;
    for (anum = 0; anum < EMCMOT_MAX_AXIS; anum++) {
        beforePose[anum] = *pcmd_p[anum];
    }
#endif
    kinematicsForward(joint_posKinsSwitch,
                      &emcmotStatus->carte_pos_cmd,
                      &tmpFFlags, &tmpIFlags);
#ifdef SWITCHKINS_DEBUG
    fprintf(stderr,"kswitch type=%d (%s:%d)\n",switchkins_type,__FUNCTION__,__LINE__);
    for (anum = 0; anum < EMCMOT_MAX_AXIS; anum++) {
        fprintf(stderr,"anum=%d before:%8.3g after:%8.3g delta=%8.3g\n"
               ,anum,beforePose[anum],*pcmd_p[anum],*pcmd_p[anum]-beforePose[anum]);
    }
#endif
    axis_apply_ext_offsets_to_carte_pos(-1, pcmd_p);
    tpSetPos(&emcmotInternal->chan[0].coord_tp, &emcmotStatus->carte_pos_cmd);
} //handle_kinematicsSwitch()

/* MCHAN MC32: resolve a channel's EFFECTIVE per-channel feed controls,
 * honouring sync groups (motion.N.feed-group >= 0 couples channels):
 *   - feed-hold is OR'd across the group (any member holds -> the whole
 *     group holds: a stop on one synchronized head stops the set)
 *   - the feed override is taken from the group AUTHORITY = the lowest-
 *     numbered member (Fanuq exclusive-authority: one knob governs the
 *     group; members' own override pins are ignored while grouped).
 * Ungrouped (feed-group < 0) = pure MC5 per-channel behaviour.
 * Note: this couples OVERRIDE and HOLD, not the toolpaths themselves
 * (program synchronisation is the waiting-M / phase-4 work). */
static void mchan_feed_controls(int ch, int *hold, int *ov_enable, double *ov)
{
    emcmot_hal_data_t *h = emcmot_hal_data;
    int g = *h->mchan[ch].feed_group;
    *hold = *h->mchan[ch].feed_hold ? 1 : 0;
    if (g < 0) {
	*ov_enable = *h->mchan[ch].feed_override_enable ? 1 : 0;
	*ov = *h->mchan[ch].feed_override;
	return;
    }
    int auth = ch, m;
    for (m = 0; m < motion_num_channels; m++) {
	if (m == ch) continue;
	if (*h->mchan[m].feed_group == g) {
	    if (*h->mchan[m].feed_hold) *hold = 1;
	    if (m < auth) auth = m;
	}
    }
    *ov_enable = *h->mchan[auth].feed_override_enable ? 1 : 0;
    *ov = *h->mchan[auth].feed_override;
}

static void process_inputs(void)
{
    int joint_num, spindle_num;
    double abs_ferror, scale;
    joint_hal_t *joint_data;
    emcmot_joint_t *joint;
    unsigned char enables;
    /* read spindle angle (for threading, etc) */
    for (spindle_num = 0; spindle_num < emcmotConfig->numSpindles; spindle_num++){
		emcmotStatus->spindle_status[spindle_num].spindleRevs =
				*emcmot_hal_data->spindle[spindle_num].spindle_revs;
		emcmotStatus->spindle_status[spindle_num].spindleSpeedIn =
				*emcmot_hal_data->spindle[spindle_num].spindle_speed_in;
		emcmotStatus->spindle_status[spindle_num].at_speed =
				*emcmot_hal_data->spindle[spindle_num].spindle_is_atspeed;
    }
    /* compute net feed and spindle scale factors */
    /* MCHAN MC22: composed PER CHANNEL into each channel TP. Channel 0 keeps
     * the exact legacy composition (motion_state / global motionType /
     * adaptive-feed with ch0-TP reverse-run coupling) and mirrors to the
     * legacy emcmotStatus fields. Secondary channels are coord-only by
     * design: their enables come from their own TP (queued while motion is
     * in flight), rapid-vs-feed from their own executing motion type, the
     * GLOBAL feed-hold / feed-inhibit pins still apply to them (D5 global
     * floor), and the adaptive-feed pin is ch0-only until per-channel
     * motion.N.* pins exist (MC7) - it couples to a single TP's reverse-run. */
    {
	int mchan_ch;
	for (mchan_ch = 0; mchan_ch < motion_num_channels; mchan_ch++) {
	    TP_STRUCT *ctp = &emcmotInternal->chan[mchan_ch].coord_tp;
	    unsigned char ch_enables;
	    if (mchan_ch == 0) {
		if ( emcmotStatus->motion_state == EMCMOT_MOTION_COORD ) {
		    /* use the enables that were queued with the current move */
		    ch_enables = ctp->enables_queued;
		} else {
		    /* use the enables that are in effect right now */
		    ch_enables = ctp->enables_new;
		}
	    } else {
		/* secondary: queued enables while its queue is in flight */
		if (!tpIsDone(ctp) || tpQueueDepth(ctp) > 0) {
		    ch_enables = ctp->enables_queued;
		} else {
		    ch_enables = ctp->enables_new;
		}
	    }
	    /* feed scaling first:  feed_scale, adaptive_feed, and feed_hold */
	    scale = 1.0;
	    if (mchan_ch == 0) {
		if (   (emcmotStatus->motion_state != EMCMOT_MOTION_FREE)
		    && (ch_enables & FS_ENABLED) ) {
		    if (emcmotStatus->motionType == EMC_MOTION_TYPE_TRAVERSE) {
			scale *= ctp->rapid_scale;
		    } else {
			scale *= ctp->feed_scale;
		    }
		}
	    } else if (ch_enables & FS_ENABLED) {
		/* secondary channels have no FREE state; use their own
		 * executing motion type for the rapid-vs-feed choice */
		if (tpGetMotionType(ctp) == EMC_MOTION_TYPE_TRAVERSE) {
		    scale *= ctp->rapid_scale;
		} else {
		    scale *= ctp->feed_scale;
		}
	    }
	    if ( (mchan_ch == 0) && (ch_enables & AF_ENABLED) ) {
		/* read and clamp adaptive feed HAL pin (ch0-only, see above) */
		double adaptive_feed_in = *emcmot_hal_data->adaptive_feed;
		// Clip range to +/- MAX_FEED_OVERRIDE from the [DISPLAY] section of the ini file
		if (adaptive_feed_in > emcmotConfig->maxFeedScale) {
		    adaptive_feed_in = emcmotConfig->maxFeedScale;
		} else if (adaptive_feed_in < -emcmotConfig->maxFeedScale) {
		    adaptive_feed_in = -emcmotConfig->maxFeedScale;
		}
		// Handle case of negative adaptive feed
		// Actual scale factor is always positive by default
		double adaptive_feed_out = fabs(adaptive_feed_in);
		// Case 1: positive to negative direction change
		if ( adaptive_feed_in < 0.0 && emcmotInternal->chan[0].coord_tp.reverse_run == TC_DIR_FORWARD) {
		    // User commands feed in reverse direction, but we're not running in reverse yet
		    if (tpSetRunDir(&emcmotInternal->chan[0].coord_tp, TC_DIR_REVERSE) != TP_ERR_OK) {
			// Need to decelerate to a stop first
			adaptive_feed_out = 0.0;
		    }
		} else if (adaptive_feed_in > 0.0 && emcmotInternal->chan[0].coord_tp.reverse_run == TC_DIR_REVERSE ) {
		    // User commands feed in forward direction, but we're running in reverse
		    if (tpSetRunDir(&emcmotInternal->chan[0].coord_tp, TC_DIR_FORWARD) != TP_ERR_OK) {
			// Need to decelerate to a stop first
			adaptive_feed_out = 0.0;
		    }
		}
		//Otherwise, if direction and sign match, we're ok
		scale *= adaptive_feed_out;
	    }
	    /* MCHAN MC5/MC32: this channel's effective per-channel feed-hold +
	     * override, resolved through any sync group it belongs to. */
	    {
		int mc_hold, mc_oven; double mc_ov;
		mchan_feed_controls(mchan_ch, &mc_hold, &mc_oven, &mc_ov);
		if ( ch_enables & FH_ENABLED ) {
		    /* feed hold HAL pin (global pin = all channels, D5) */
		    if ( *emcmot_hal_data->feed_hold ) {
			scale = 0;
		    }
		    /* MCHAN MC5: per-channel feed-hold (motion.N.feed-hold) - a
		     * hardware feed-hold button for THIS head only (MC32: OR'd
		     * across its sync group); maskable like the global one so
		     * it will not break a tap/thread. */
		    if ( mc_hold ) {
			scale = 0;
		    }
		}
		/*non maskable (except during spinndle synch move) feed hold inhibit pin */
		if ( ch_enables & *emcmot_hal_data->feed_inhibit ) {
		    scale = 0;
		}
		/* MCHAN MC5: per-channel feed override (motion.N.feed-override),
		 * an operator pot for THIS head (MC32: from the group authority
		 * when grouped). Applied only when enabled (unwired = stock);
		 * multiplies on top of the GUI/NML feed scale; clamped to
		 * [DISPLAY]MAX_FEED_OVERRIDE. */
		if ( mc_oven ) {
		    double ov = mc_ov;
		    if ( ov < 0.0 ) ov = 0.0;
		    if ( ov > emcmotConfig->maxFeedScale ) ov = emcmotConfig->maxFeedScale;
		    scale *= ov;
		}
	    }
	    /* MCHAN MC31 I3: protective stop - a keep-out-zone co-occupant
	     * (handover permit off) holds at zero feed until it clears or is
	     * permitted (flag set by mchan_run_interference last cycle). */
	    if ( emcmotInternal->chan[mchan_ch].interfere_stop ) {
		scale = 0;
	    }
	    /* save the resulting combined scale factor for this channel */
	    ctp->net_feed_scale = scale;
	}
	/* channel 0 mirrors to the legacy status fields (GUI/status view and
	 * the jog/free-mode consumers elsewhere in this file) */
	emcmotStatus->net_feed_scale = emcmotInternal->chan[0].coord_tp.net_feed_scale;
	emcmotStatus->enables_queued = emcmotInternal->chan[0].coord_tp.enables_queued;
	/* leave 'enables' (used by the spindle-scale section below) on the
	 * legacy ch0 semantics */
	if ( emcmotStatus->motion_state == EMCMOT_MOTION_COORD ) {
	    enables = emcmotInternal->chan[0].coord_tp.enables_queued;
	} else {
	    enables = emcmotInternal->chan[0].coord_tp.enables_new;
	}
	scale = emcmotStatus->net_feed_scale;
    }

    /* now do spindle scaling */
    for (spindle_num=0; spindle_num < emcmotConfig->numSpindles; spindle_num++){
		scale = 1.0;
		if ( enables & SS_ENABLED ) {
			scale *= emcmotStatus->spindle_status[spindle_num].scale;
		}
		/*non maskable (except during spindle synch move) spindle inhibit pin */
		if ( enables & *emcmot_hal_data->spindle[spindle_num].spindle_inhibit ) {
			scale = 0;
		}
		/* save the resulting combined scale factor */
		emcmotStatus->spindle_status[spindle_num].net_scale = scale;
    }

    /* read and process per-joint inputs */
    for (joint_num = 0; joint_num < ALL_JOINTS ; joint_num++) {
	/* point to joint HAL data */
	joint_data = &(emcmot_hal_data->joint[joint_num]);
	/* point to joint data */
	joint = &joints[joint_num];
	if (!GET_JOINT_ACTIVE_FLAG(joint)) {
	    /* if joint is not active, skip it */
	    continue;
	}
	/* copy data from HAL to joint structure */
	joint->motor_pos_fb = *(joint_data->motor_pos_fb);
	/* calculate pos_fb */
	if (( get_homing_at_index_search_wait(joint_num) ) &&
	    ( get_index_enable(joint_num) == 0 )) {
	    /* special case - we're homing the joint, and it just
	       hit the index.  The encoder count might have made a
	       step change.  The homing code will correct for it
	       later, so we ignore motor_pos_fb and set pos_fb
	       to match the commanded value instead. */
	    joint->pos_fb = joint->pos_cmd;
	} else {
	    /* normal case: subtract backlash comp and motor offset */
	    joint->pos_fb = joint->motor_pos_fb -
		(joint->backlash_filt + joint->motor_offset);
	}
	/* calculate following error */
	if ( IS_EXTRA_JOINT(joint_num) && get_homed(joint_num) ) {
	    joint->ferror = 0; // not relevant for homed extrajoints
	} else {
	    joint->ferror = joint->pos_cmd - joint->pos_fb;
	}
	abs_ferror = fabs(joint->ferror);
	/* update maximum ferror if needed */
	if (abs_ferror > joint->ferror_high_mark) {
	    joint->ferror_high_mark = abs_ferror;
	}

	/* calculate following error limit */
	if (joint->vel_limit > 0.0) {
	    joint->ferror_limit =
		joint->max_ferror * fabs(joint->vel_cmd) / joint->vel_limit;
	} else {
	    joint->ferror_limit = 0;
	}
	if (joint->ferror_limit < joint->min_ferror) {
	    joint->ferror_limit = joint->min_ferror;
	}
	/* update following error flag */
	if (abs_ferror > joint->ferror_limit) {
	    SET_JOINT_FERROR_FLAG(joint, 1);
	} else {
	    SET_JOINT_FERROR_FLAG(joint, 0);
	}

	/* read limit switches */
	if (*(joint_data->pos_lim_sw)) {
	    SET_JOINT_PHL_FLAG(joint, 1);
	} else {
	    SET_JOINT_PHL_FLAG(joint, 0);
	}
	if (*(joint_data->neg_lim_sw)) {
	    SET_JOINT_NHL_FLAG(joint, 1);
	} else {
	    SET_JOINT_NHL_FLAG(joint, 0);
	}
	joint->on_pos_limit = GET_JOINT_PHL_FLAG(joint);
	joint->on_neg_limit = GET_JOINT_NHL_FLAG(joint);
	/* read amp fault input */
	if (*(joint_data->amp_fault)) {
	    SET_JOINT_FAULT_FLAG(joint, 1);
	} else {
	    SET_JOINT_FAULT_FLAG(joint, 0);
	}
    }

    // a fault was signalled during a spindle-orient in progress
    // signal error, and cancel the orient
    for (spindle_num = 0; spindle_num < emcmotConfig->numSpindles; spindle_num++){
        if(*(emcmot_hal_data->spindle[spindle_num].spindle_amp_fault)){
            emcmotStatus->spindle_status[spindle_num].fault = 1;
        }else{
            emcmotStatus->spindle_status[spindle_num].fault = 0;
        }
		if (*(emcmot_hal_data->spindle[spindle_num].spindle_orient)) {
			if (*(emcmot_hal_data->spindle[spindle_num].spindle_orient_fault)) {
				emcmotStatus->spindle_status[spindle_num].orient_state = EMCMOT_ORIENT_FAULTED;
				*(emcmot_hal_data->spindle[spindle_num].spindle_orient) = 0;
				emcmotStatus->spindle_status[spindle_num].orient_fault =
						*(emcmot_hal_data->spindle[spindle_num].spindle_orient_fault);
				reportError(_("fault %d during orient in progress"),
						emcmotStatus->spindle_status[spindle_num].orient_fault);
				emcmotStatus->commandStatus = EMCMOT_COMMAND_INVALID_COMMAND;
				tpAbort(&emcmotInternal->chan[0].coord_tp);
				SET_MOTION_ERROR_FLAG(1);
			} else if (*(emcmot_hal_data->spindle[spindle_num].spindle_is_oriented)) {
				*(emcmot_hal_data->spindle[spindle_num].spindle_orient) = 0;
				*(emcmot_hal_data->spindle[spindle_num].spindle_locked) = 1;
				emcmotStatus->spindle_status[spindle_num].locked = 1;
				emcmotStatus->spindle_status[spindle_num].brake = 1;
				emcmotStatus->spindle_status[spindle_num].orient_state = EMCMOT_ORIENT_COMPLETE;
				rtapi_print_msg(RTAPI_MSG_DBG, "SPINDLE_ORIENT complete, spindle locked");
			}
		}
    }
    // if jog in progress stop the jog if requested
    if (enables & *(emcmot_hal_data->jog_is_active) && (*(emcmot_hal_data->jog_stop) || *(emcmot_hal_data->jog_stop_immediate))) {
        joint_jog_abort_all(*(emcmot_hal_data->jog_stop_immediate));
        axis_jog_abort_all(*(emcmot_hal_data->jog_stop_immediate));
        if (*(emcmot_hal_data->jog_stop_immediate)) {
          reportError("Jog aborted by jog-stop-immediate");
        } else {
          reportError("Jog aborted by jog-stop");
        }
    }
}

static void joint_jog_abort_all(bool immediate)
{
    int jNum;
    emcmot_joint_t *joint;
    for (jNum = 0; jNum < NO_OF_KINS_JOINTS; jNum++) {
        joint = &joints[jNum];
        joint->free_tp.enable = 0;
        joint->kb_jjog_active = 0;
        joint->wheel_jjog_active = 0;
        if (immediate) {
          joint->free_tp.curr_vel = 0.0;
        }
    }
}

static void do_forward_kins(void)
{
/* there are four possibilities for kinType:

   IDENTITY: Both forward and inverse kins are available, and they
   can used without an initial guess, even if one or more joints
   are not homed.  In this case, we apply the forward kins to the
   joint->pos_fb to produce carte_pos_fb, and if all axes are homed
   we set carte_pos_fb_ok to 1 to indicate that the feedback data
   is good.

   BOTH: Both forward and inverse kins are available, but the forward
   kins need an initial guess, and/or the kins require all joints to
   be homed before they work properly.  Here we must tread carefully.
   IF all the joints have been homed, we apply the forward kins to
   the joint->pos_fb to produce carte_pos_fb, and set carte_pos_fb_ok
   to indicate that the feedback is good.  We use the previous value
   of carte_pos_fb as the initial guess.  If all joints have not been
   homed, we don't call the kinematics, instead we set carte_pos_fb to
   the cartesean coordinates of home, as stored in the global worldHome,
   and we set carte_fb_ok to 0 to indicate that the feedback is invalid.
\todo  FIXME - maybe setting to home isn't the right thing to do.  We need
   it to be set to home eventually, (right before the first attempt to
   run the kins), but that doesn't mean we should say we're at home
   when we're not.

   INVERSE_ONLY: Only inverse kinematics are available, forward
   kinematics cannot be used.  So we have to fake it, the question is
   simply "what way of faking it is best".  In free mode, or if all
   axes have not been homed, the feedback position is unknown.  If
   we are in teleop or coord mode, or if we are in free mode and all
   axes are homed, and haven't been moved since they were homed, then
   we set carte_pos_fb to carte_pos_cmd, and set carte_pos_fb_ok to 1.
   If we are in free mode, and any joint is not homed, or any joint has
   moved since it was homed, we leave cart_pos_fb alone, and set
   carte_pos_fb_ok to 0.

   FORWARD_ONLY: Only forward kinematics are available, inverse kins
   cannot be used.  This exists for completeness only, since EMC won't
   work without inverse kinematics.

*/

/*! \todo FIXME FIXME FIXME - need to put a rate divider in here, run it
   at the traj rate */

    double joint_pos[EMCMOT_MAX_JOINTS] = {0,};
    int joint_num, result;
    emcmot_joint_t *joint;

    /* copy joint position feedback to local array */
    for (joint_num = 0; joint_num < NO_OF_KINS_JOINTS; joint_num++) {
	/* point to joint struct */
	joint = &joints[joint_num];
	/* copy feedback */
	joint_pos[joint_num] = joint->pos_fb;
    }
    switch (emcmotConfig->kinType) {

    case KINEMATICS_IDENTITY:
	kinematicsForward(joint_pos, &emcmotStatus->carte_pos_fb, &fflags,
	    &iflags);
	if (get_allhomed()) {
	    emcmotStatus->carte_pos_fb_ok = 1;
	} else {
	    emcmotStatus->carte_pos_fb_ok = 0;
	}
	break;

    case KINEMATICS_BOTH:
	if (get_allhomed()) {
	    /* is previous value suitable for use as initial guess? */
	    if (!emcmotStatus->carte_pos_fb_ok) {
		/* no, use home position as initial guess */
		emcmotStatus->carte_pos_fb = emcmotStatus->world_home;
	    }
	    /* calculate Cartesean position feedback from joint pos fb */
	    result =
		kinematicsForward(joint_pos, &emcmotStatus->carte_pos_fb,
		&fflags, &iflags);
	    /* check to make sure kinematics converged */
	    if (result < 0) {
		/* error during kinematics calculations */
		emcmotStatus->carte_pos_fb_ok = 0;
	    } else {
		/* it worked! */
		emcmotStatus->carte_pos_fb_ok = 1;
	    }
	} else {
	    emcmotStatus->carte_pos_fb_ok = 0;
	}
	break;

    case KINEMATICS_INVERSE_ONLY:

	if ((GET_MOTION_COORD_FLAG()) || (GET_MOTION_TELEOP_FLAG())) {
	    /* use Cartesean position command as feedback value */
	    emcmotStatus->carte_pos_fb = emcmotStatus->carte_pos_cmd;
	    emcmotStatus->carte_pos_fb_ok = 1;
	} else {
	    emcmotStatus->carte_pos_fb_ok = 0;
	}
	break;

    default:
	emcmotStatus->carte_pos_fb_ok = 0;
	break;
    }
}

static void process_probe_inputs(void)
{
    static int old_probeVal = 0;
    unsigned char probe_type = emcmotStatus->probe_type;

    // don't error
    char probe_suppress = probe_type & 1;

    // trigger when the probe clears, instead of the usual case of triggering when it trips
    char probe_whenclears = !!(probe_type & 2);

    /* read probe input */
    emcmotStatus->probeVal = !!*(emcmot_hal_data->probe_input);
    if (emcmotStatus->probing) {
        /* MCHAN MC25: stop/measure the channel that actually issued the probe
         * (mutual exclusion guarantees exactly one). num_channels==1 ->
         * probe_owner==0 = the legacy chan[0] path. */
        TP_STRUCT *probe_tp = &emcmotInternal->chan[emcmotInternal->probe_owner].coord_tp;
        /* check if the probe has been tripped */
        if (emcmotStatus->probeVal ^ probe_whenclears) {
            /* remember the current position */
            emcmotStatus->probedPos = emcmotStatus->carte_pos_fb;
            /* stop! */
            emcmotStatus->probing = 0;
            emcmotStatus->probeTripped = 1;
            tpAbort(probe_tp);
        /* check if the probe hasn't tripped, but the move finished */
        } else if (GET_MOTION_INPOS_FLAG() && tpQueueDepth(probe_tp) == 0) {
            /* we are already stopped, but we need to remember the current
               position here, because it will still be queried */
            emcmotStatus->probedPos = emcmotStatus->carte_pos_fb;
            emcmotStatus->probing = 0;
            if (probe_suppress) {
                emcmotStatus->probeTripped = 0;
            } else if(probe_whenclears) {
                reportError(_("G38.4 move finished without breaking contact."));
                SET_MOTION_ERROR_FLAG(1);
            } else {
                reportError(_("G38.2 move finished without making contact."));
                SET_MOTION_ERROR_FLAG(1);
            }
        }
    } else if (!old_probeVal && emcmotStatus->probeVal) {
        // not probing, but we have a rising edge on the probe.
        // this could be expensive if we don't stop.

        if(!GET_MOTION_INPOS_FLAG() && tpQueueDepth(&emcmotInternal->chan[0].coord_tp)) {
            // running an command
            if (emcmotStatus->motionType != EMC_MOTION_TYPE_PROBING) {
                tpAbort(&emcmotInternal->chan[0].coord_tp);
                reportError(_("Probe tripped during non-probe move."));
                SET_MOTION_ERROR_FLAG(1);
            }
        } else {
            // not running a command
            int i;
            int aborted = 0;

            for(i=0; i<NO_OF_KINS_JOINTS; i++) {
                emcmot_joint_t *joint = &joints[i];

                if (!GET_JOINT_ACTIVE_FLAG(joint)) {
                    /* if joint is not active, skip it */
                    continue;
                }

                // inhibit_probe_home_error is set by [TRAJ]->NO_PROBE_HOME_ERROR in the ini file
                if (!emcmotConfig->inhibit_probe_home_error) {
                    // abort any homing
                    if(get_homing(i)) {
                        do_cancel_homing(i);
                        aborted=1;
                    }
                }

                // inhibit_probe_jog_error is set by [TRAJ]->NO_PROBE_JOG_ERROR in the ini file
                if (!emcmotConfig->inhibit_probe_jog_error) {
                    // abort any joint jogs
                    if(joint->free_tp.enable == 1) {
                        joint->free_tp.enable = 0;
                        // since homing uses free_tp, this protection of aborted
                        // is needed so the user gets the correct error.
                        if(!aborted) aborted=2;
                    }
                }
            }
            if (!emcmotConfig->inhibit_probe_jog_error) {
                if (axis_jog_abort_all(1)) {
                    aborted = 3;
                }
            }

            if(aborted == 1) {
                reportError(_("Probe tripped during homing motion."));
            }

            if(aborted == 2) {
                reportError(_("Probe tripped during a joint jog."));
            }
            if(aborted == 3) {
                reportError(_("Probe tripped during a coordinate jog."));
            }
        }
    }
    old_probeVal = emcmotStatus->probeVal;
}

static void check_for_faults(void)
{
    int joint_num, spindle_num, error_num;
    emcmot_joint_t *joint;
    int neg_limit_override, pos_limit_override;

    /* check for various global fault conditions */
    /* only check enable input if running */
    if ( GET_MOTION_ENABLE_FLAG() != 0 ) {
	if ( *(emcmot_hal_data->enable) == 0 ) {
	    reportError(_("motion stopped by enable input"));
	    emcmotInternal->enabling = 0;
	}
    }
    /* check for spindle ampfifier errors */
    for (spindle_num = 0; spindle_num < emcmotConfig->numSpindles; spindle_num++){
        if(emcmotStatus->spindle_status[spindle_num].fault && GET_MOTION_ENABLE_FLAG()){
            reportError(_("spindle %d amplifier fault"), spindle_num);
            emcmotInternal->enabling = 0;
        }
    }
    /* check for various joint fault conditions */
    for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
	/* point to joint data */
	joint = &joints[joint_num];
	/* only check active, enabled axes */
	if ( GET_JOINT_ACTIVE_FLAG(joint) && GET_JOINT_ENABLE_FLAG(joint) ) {
	    /* are any limits for this joint overridden? */
	    neg_limit_override = emcmotStatus->overrideLimitMask & ( 1 << (joint_num*2));
	    pos_limit_override = emcmotStatus->overrideLimitMask & ( 2 << (joint_num*2));
	    /* check for hard limits */
	    if ((GET_JOINT_PHL_FLAG(joint) && ! pos_limit_override ) ||
		(GET_JOINT_NHL_FLAG(joint) && ! neg_limit_override )) {
		/* joint is on limit switch, should we trip? */
		if (get_homing(joint_num)) {
		    /* no, ignore limits */
		} else {
		    /* trip on limits */
		    if (!GET_JOINT_ERROR_FLAG(joint)) {
			/* report the error just this once */
			reportError(_("joint %d on limit switch error"),
			    joint_num);
		    }
		    SET_JOINT_ERROR_FLAG(joint, 1);
		    emcmotInternal->enabling = 0;
		}
	    }
	    /* check for amp fault */
	    if (GET_JOINT_FAULT_FLAG(joint)) {
		/* joint is faulted, trip */
		if (!GET_JOINT_ERROR_FLAG(joint)) {
		    /* report the error just this once */
		    reportError(_("joint %d amplifier fault"), joint_num);
		}
		SET_JOINT_ERROR_FLAG(joint, 1);
		emcmotInternal->enabling = 0;
	    }
	    /* check for excessive following error */
	    if (GET_JOINT_FERROR_FLAG(joint)) {
		if (!GET_JOINT_ERROR_FLAG(joint)) {
		    /* report the error just this once */
		    reportError(_("joint %d following error"), joint_num);
		}
		SET_JOINT_ERROR_FLAG(joint, 1);
		emcmotInternal->enabling = 0;
	    }
	/* end of if JOINT_ACTIVE_FLAG(joint) */
	}
    /* end of check for joint faults loop */
    }

    /* Check Miscellaneous faults */
    for (error_num=0; error_num < emcmotConfig->numMiscError; error_num++){
      if(emcmotStatus->misc_error[error_num] && GET_MOTION_ENABLE_FLAG()) {
        reportError(_("Motion Stopped by misc error %d"), error_num);
        emcmotInternal->enabling = 0;
      }
    }
}

static void set_operating_mode(void)
{
    int joint_num;
    emcmot_joint_t *joint;
    double positions[EMCMOT_MAX_JOINTS];

    /* check for disabling */
    if (!emcmotInternal->enabling && GET_MOTION_ENABLE_FLAG()) {
	/* clear out the motion emcmotInternal->chan[0].coord_tp and interpolators */
	tpClear(&emcmotInternal->chan[0].coord_tp);
	for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
	    /* point to joint data */
	    joint = &joints[joint_num];
	    /* disable free mode planner */
	    joint->free_tp.enable = 0;
	    joint->free_tp.curr_vel = 0.0;
        joint->free_tp.curr_acc = 0.0;
	    /* drain coord mode interpolators */
	    cubicDrain(&(joint->cubic));
	    if (GET_JOINT_ACTIVE_FLAG(joint)) {
		SET_JOINT_INPOS_FLAG(joint, 1);
		SET_JOINT_ENABLE_FLAG(joint, 0);
		do_cancel_homing(joint_num);
	    }
	    /* don't clear the joint error flag, since that may signify why
	       we just went into disabled state */
	}

    axis_jog_abort_all(1);

	/* MCHAN: machine disable is the global stop floor (D5) - abort the
	 * secondary channels' planners too (their joints just froze; their
	 * queues must not resume on re-enable) */
	for (int mch = 1; mch < motion_num_channels; mch++) {
	    tpClear(&emcmotInternal->chan[mch].coord_tp);
	}

	SET_MOTION_ENABLE_FLAG(0);
	/* don't clear the motion error flag, since that may signify why we
	   just went into disabled state */
    }

    /* check for emcmotInternal->enabling */
    if (emcmotInternal->enabling && !GET_MOTION_ENABLE_FLAG()) {
        if (*(emcmot_hal_data->eoffset_limited)) {
            reportError("Note: Motion enabled after reaching a coordinate "
                        "soft limit with active external offsets");
            *(emcmot_hal_data->eoffset_limited) = 0;
        }
        axis_initialize_external_offsets();
        tpSetPos(&emcmotInternal->chan[0].coord_tp, &emcmotStatus->carte_pos_cmd);
	for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
	    /* point to joint data */
	    joint = &joints[joint_num];
	    joint->free_tp.curr_pos = joint->pos_cmd;
	    if (GET_JOINT_ACTIVE_FLAG(joint)) {
		SET_JOINT_ENABLE_FLAG(joint, 1);
		do_cancel_homing(joint_num);
	    }
	    /* clear any outstanding joint errors when going into enabled
	       state */
	    SET_JOINT_ERROR_FLAG(joint, 0);
	}
	if ( !GET_MOTION_ENABLE_FLAG() ) {
            if (GET_MOTION_TELEOP_FLAG()) {
                axis_sync_teleop_tp_to_carte_pos(0, pcmd_p);
            }
	}
	/* MCHAN: resync each secondary channel's planner to its mapped
	 * joints' commanded positions (mirror of channel 0's tpSetPos
	 * above) so the first move after re-enable starts where the
	 * joints actually are */
	for (int mch = 1; mch < motion_num_channels; mch++) {
	    emcmot_channel_t *c = &emcmotInternal->chan[mch];
	    EmcPose cpose;
	    int any = 0;
	    ZERO_EMC_POSE(cpose);
	    for (int ax = 0; ax < EMCMOT_MAX_AXIS; ax++) {
		int jn = c->axis_to_joint[ax];
		if (jn < 0) continue;
		mchan_pose_set_axis(&cpose, ax, joints[jn].pos_cmd);
		any = 1;
	    }
	    if (any) tpSetPos(&c->coord_tp, &cpose);
	}
	SET_MOTION_ENABLE_FLAG(1);
	/* clear any outstanding motion errors when going into enabled state */
	SET_MOTION_ERROR_FLAG(0);
    }

    /* check for entering teleop mode */
    if (emcmotInternal->teleoperating && !GET_MOTION_TELEOP_FLAG()) {
	if (GET_MOTION_INPOS_FLAG()) {

	    /* update coordinated emcmotInternal->chan[0].coord_tp position */
	    tpSetPos(&emcmotInternal->chan[0].coord_tp, &emcmotStatus->carte_pos_cmd);
	    /* drain the cubics so they'll synch up */
	    for (joint_num = 0; joint_num < EMCMOT_MAX_JOINTS; joint_num++) {
		if (joint_num < NO_OF_KINS_JOINTS) {
		/* point to joint data */
		    joint = &joints[joint_num];
		    if (coord_cubic_active && *(emcmot_hal_data->eoffset_active)) {
		        //skip
		    } else if (emcmotInternal->joint_owner[joint_num] != 0) {
		        /* MCHAN: never drain a secondary channel's joint -
		         * its stream is live mid-motion (found at phase-2
		         * bring-up: ch0 mode changes wiped ch1's joints) */
		    } else {
		        cubicDrain(&(joint->cubic));
		    }
		    positions[joint_num] = joint->coarse_pos;
		} else {
		    positions[joint_num] = 0;
		}
	    }
	    coord_cubic_active = 0;
	    /* Initialize things to do when starting teleop mode. */
	    SET_MOTION_TELEOP_FLAG(1);
	    SET_MOTION_COORD_FLAG(0);
	    SET_MOTION_ERROR_FLAG(0);

            kinematicsForward(positions, &emcmotStatus->carte_pos_cmd, &fflags, &iflags);
            // entering teleop (INPOS), remove ext offsets
            axis_sync_teleop_tp_to_carte_pos(-1, pcmd_p);
	} else {
	    /* not in position-- don't honor mode change */
	    emcmotInternal->teleoperating = 0;
	}
    } else {
	if (GET_MOTION_INPOS_FLAG()) {
	    if (!emcmotInternal->teleoperating && GET_MOTION_TELEOP_FLAG()) {
		SET_MOTION_TELEOP_FLAG(0);
		if (!emcmotInternal->coordinating) {
		    for (joint_num = 0; joint_num < NO_OF_KINS_JOINTS; joint_num++) {
			/* point to joint data */
			joint = &joints[joint_num];
			/* update free planner positions */
			joint->free_tp.curr_pos = joint->pos_cmd;
		    }
		}
	    }
	}

	/* check for entering coordinated mode */
	if (emcmotInternal->coordinating && !GET_MOTION_COORD_FLAG()) {
	    if (GET_MOTION_INPOS_FLAG()) {
		/* preset traj planner to current position */

                // subtract at coord mode start
                axis_apply_ext_offsets_to_carte_pos(-1, pcmd_p);

		tpSetPos(&emcmotInternal->chan[0].coord_tp, &emcmotStatus->carte_pos_cmd);
		/* drain the cubics so they'll synch up */
		for (joint_num = 0; joint_num < NO_OF_KINS_JOINTS; joint_num++) {
		    /* MCHAN: a secondary channel's joints are NOT channel
		     * 0's to drain - their streams may be live mid-motion
		     * (found at phase-2 bring-up: every ch0 COORD entry
		     * wiped ch1's in-flight interpolators) */
		    if (emcmotInternal->joint_owner[joint_num] != 0) continue;
		    /* point to joint data */
		    joint = &joints[joint_num];
		    cubicDrain(&(joint->cubic));
		}
		/* clear the override limits flags */
		emcmotInternal->overriding = 0;
		emcmotStatus->overrideLimitMask = 0;
		SET_MOTION_COORD_FLAG(1);
		SET_MOTION_TELEOP_FLAG(0);
		SET_MOTION_ERROR_FLAG(0);
	    } else {
		/* not in position-- don't honor mode change */
		emcmotInternal->coordinating = 0;
	    }
	}

	/* check entering free space mode */
	if (!emcmotInternal->coordinating && GET_MOTION_COORD_FLAG()) {
	    if (GET_MOTION_INPOS_FLAG()) {
		for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
		    /* point to joint data */
		    joint = &joints[joint_num];
		    /* set joint planner curr_pos to current location */
		    joint->free_tp.curr_pos = joint->pos_cmd;
		    /* but it can stay disabled until a move is required */
		    joint->free_tp.enable = 0;
		}
		SET_MOTION_COORD_FLAG(0);
		SET_MOTION_TELEOP_FLAG(0);
		SET_MOTION_ERROR_FLAG(0);
	    } else {
		/* not in position-- don't honor mode change */
		emcmotInternal->coordinating = 1;
	    }
	}
    }
    /*! \todo FIXME - this code is temporary - eventually this function will be
       cleaned up and simplified, and 'motion_state' will become the master
       for this info, instead of having to gather it from several flags */
    if (!GET_MOTION_ENABLE_FLAG()) {
	emcmotStatus->motion_state = EMCMOT_MOTION_DISABLED;
    } else if (GET_MOTION_TELEOP_FLAG()) {
	emcmotStatus->motion_state = EMCMOT_MOTION_TELEOP;
    } else if (GET_MOTION_COORD_FLAG()) {
	emcmotStatus->motion_state = EMCMOT_MOTION_COORD;
    } else {
	emcmotStatus->motion_state = EMCMOT_MOTION_FREE;
    }
} //set_operating_mode

static void handle_jjogwheels(void)
{
    int joint_num;
    emcmot_joint_t *joint;
    joint_hal_t *joint_data;
    int new_jjog_counts, delta;
    double distance, pos, stop_dist;
    static int first_pass = 1;	/* used to set initial conditions */

    for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
        double jaccel_limit;
	/* point to joint data */
	joint_data = &(emcmot_hal_data->joint[joint_num]);
	joint = &joints[joint_num];
	if (!GET_JOINT_ACTIVE_FLAG(joint)) {
	    /* if joint is not active, skip it */
	    continue;
	}

        // disallow accel bogus fractions
        if (    (*(joint_data->jjog_accel_fraction) > 1)
             || (*(joint_data->jjog_accel_fraction) < 0) ) {
            jaccel_limit = joint->acc_limit;
        } else {
            jaccel_limit = (*(joint_data->jjog_accel_fraction)) * joint->acc_limit;
        }
	/* get counts from jogwheel */
	new_jjog_counts = *(joint_data->jjog_counts);
	delta = new_jjog_counts - joint->old_jjog_counts;
	/* save value for next time */
	joint->old_jjog_counts = new_jjog_counts;
	/* initialization complete */
	if ( first_pass ) {
	    continue;
	}
	/* did the wheel move? */
	if ( delta == 0 ) {
	    /* no, nothing to do */
	    continue;
	}
        if (GET_MOTION_TELEOP_FLAG()) {
            joint->free_tp.enable = 0;
            return;
        }
	/* must be in free mode and enabled */
	if (GET_MOTION_COORD_FLAG()) {
	    continue;
	}
	if (!GET_MOTION_ENABLE_FLAG()) {
	    continue;
	}
	/* the jogwheel input for this joint must be enabled */
	if ( *(joint_data->jjog_enable) == 0 ) {
	    continue;
	}
	/* must not be homing */
	if (get_homing_is_active() ) {
	    continue;
	}
	/* must not be doing a keyboard jog */
	if (joint->kb_jjog_active) {
	    continue;
	}
	if (emcmotStatus->net_feed_scale < 0.0001 ) {
	    /* don't jog if feedhold is on or if feed override is zero */
	    break;
	}
        if (get_home_needs_unlock_first(joint_num) ) {
            reportError("Can't wheel jog locking joint_num=%d",joint_num);
            continue;
        }
        if (get_home_is_synchronized(joint_num)) {
            if (emcmotConfig->kinType == KINEMATICS_IDENTITY) {
                rtapi_print_msg(RTAPI_MSG_ERR,
                "Homing is REQUIRED to wheel jog requested coordinate\n"
                "because joint (%d) home_sequence is synchronized (%d)\n"
                ,joint_num,get_home_sequence(joint_num) );
            } else {
                rtapi_print_msg(RTAPI_MSG_ERR,
                "Cannot wheel jog joint %d because home_sequence synchronized (%d)\n"
                ,joint_num,get_home_sequence(joint_num) );
            }
            continue;
        }
	/* calculate distance to jog */
	distance = delta * *(joint_data->jjog_scale);
	/* check for joint already on hard limit */
	if (distance > 0.0 && GET_JOINT_PHL_FLAG(joint)) {
	    continue;
	}
	if (distance < 0.0 && GET_JOINT_NHL_FLAG(joint)) {
	    continue;
	}
	/* calc target position for jog */
	pos = joint->free_tp.pos_cmd + distance;
	/* don't jog past limits */
	refresh_jog_limits(joint,joint_num);
	if (pos > joint->max_jog_limit) {
	    continue;
	}
	if (pos < joint->min_jog_limit) {
	    continue;
	}
	/* The default is to move exactly as far as the wheel commands,
	   even if that move takes much longer than the wheel movement
	   that commanded it.  Some folks prefer that the move stop as
	   soon as the wheel does, even if that means not moving the
	   commanded distance.  Velocity mode is for those folks.  If
	   the command is faster than the machine can track, excess
	   command is simply dropped. */
	if ( *(joint_data->jjog_vel_mode) ) {
            double v = joint->vel_limit * emcmotStatus->net_feed_scale;
	    /* compute stopping distance at max speed */
	    stop_dist = v * v / ( 2 * jaccel_limit);
	    /* if commanded position leads the actual position by more
	       than stopping distance, discard excess command */
	    if ( pos > joint->pos_cmd + stop_dist ) {
		pos = joint->pos_cmd + stop_dist;
	    } else if ( pos < joint->pos_cmd - stop_dist ) {
		pos = joint->pos_cmd - stop_dist;
	    }
	}
        /* set target position and use full velocity and accel */
        joint->free_tp.pos_cmd = pos;
        joint->free_tp.max_vel = joint->vel_limit;
        joint->free_tp.max_acc = jaccel_limit;
        joint->free_tp.max_jerk = joint->jerk_limit;
	/* lock out other jog sources */
	joint->wheel_jjog_active = 1;
        /* and let it go */
        joint->free_tp.enable = 1;
	SET_JOINT_ERROR_FLAG(joint, 0);
	/* clear joint homed flag(s) if we don't have forward kins.
	   Otherwise, a transition into coordinated mode will incorrectly
	   assume the homed position. Do all if they've all been moved
	   since homing, otherwise just do this one */
	clearHomes(joint_num);
    }

    // done with initialization, do the whole thing from now on
    first_pass = 0;
}

/* MCHAN (MC3-lite): axis component (0=X..8=W) of an EmcPose */
static double mchan_pose_axis(EmcPose const *p, int ax)
{
    switch (ax) {
    case 0: return p->tran.x;
    case 1: return p->tran.y;
    case 2: return p->tran.z;
    case 3: return p->a;
    case 4: return p->b;
    case 5: return p->c;
    case 6: return p->u;
    case 7: return p->v;
    case 8: return p->w;
    }
    return 0.0;
}

/* MCHAN (MC2b): set axis component (0=X..8=W) of an EmcPose */
static void mchan_pose_set_axis(EmcPose *p, int ax, double v)
{
    switch (ax) {
    case 0: p->tran.x = v; break;
    case 1: p->tran.y = v; break;
    case 2: p->tran.z = v; break;
    case 3: p->a = v; break;
    case 4: p->b = v; break;
    case 5: p->c = v; break;
    case 6: p->u = v; break;
    case 7: p->v = v; break;
    case 8: p->w = v; break;
    }
}

/* MCHAN (MC3-lite): run the secondary channels' coordinated pipelines.
 * Each secondary channel is coord-only: its planner output drives exactly
 * the joints it has mapped (and which it therefore owns - those joints are
 * excluded from channel 0's free/teleop/coord handling). A channel with no
 * mapped joints just keeps its planner clock ticking. Mirrors the channel-0
 * COORD pattern: fill the cubic interpolators from the TP as needed, then
 * interpolate. Loop body never runs at num_channels=1. */
/* MCHAN MC10/Phase4: waiting-M (M200-M229) rendezvous engine.
 *
 * Each channel that reaches a waiting-M parks (queue drained by the queue-
 * buster) and motion records its arrival (chan[].waitm_num). This runs every
 * servo cycle: for each waiting channel it computes its participant set (the
 * P-word mask, or all configured channels if none), and when EVERY
 * participant is simultaneously arrived at the SAME number with a consistent
 * mask it releases them all in the SAME cycle (waitm_released=1). A partner
 * arrived at a DIFFERENT number/mask is a Fanuq-160 mismatch -> error+hold.
 * A partner that never arrives within motion.waitm-timeout -> a ONE-SHOT
 * "chN waiting for chM @M2xx" error, then HOLD (stay armed so a late partner
 * still releases - the error+hold policy). estop/disable clears everything.
 * Channel-scoped status + HAL observability pins are published at the end.
 * No-op cost at 1 channel / when nobody is waiting (a few comparisons). */
static void mchan_run_rendezvous(void)
{
    int ch, m;
    double tmo = *(emcmot_hal_data->waitm_timeout);
    int all_mask = (1 << motion_num_channels) - 1;

    if (!GET_MOTION_ENABLE_FLAG()) {
	/* estop / machine off: drop every pending rendezvous so a stale
	 * arrival can't phantom-match when the machine comes back. */
	for (ch = 0; ch < motion_num_channels; ch++) {
	    emcmot_channel_t *c = &emcmotInternal->chan[ch];
	    c->waitm_num = -1; c->waitm_released = 0;
	    c->waitm_reported = 0; c->waitm_blockers = 0;
	}
    } else {
	for (ch = 0; ch < motion_num_channels; ch++) {
	    emcmot_channel_t *c = &emcmotInternal->chan[ch];
	    if (c->waitm_num < 0 || c->waitm_released) continue;	/* not waiting */
	    int mask = c->waitm_mask ? c->waitm_mask : all_mask;
	    int all_arrived = 1, blockers = 0, mismatch = 0, mm_ch = -1;
	    for (m = 0; m < motion_num_channels; m++) {
		if (m == ch || !((mask >> m) & 1)) continue;
		emcmot_channel_t *o = &emcmotInternal->chan[m];
		int omask = o->waitm_mask ? o->waitm_mask : all_mask;
		if (o->waitm_num < 0 || o->waitm_released) {
		    all_arrived = 0; blockers |= (1 << m);	/* not (yet) here */
		} else if (o->waitm_num != c->waitm_num || omask != mask) {
		    mismatch = 1; mm_ch = m;			/* Fanuq-160 */
		}
	    }
	    c->waitm_blockers = blockers;
	    if (mismatch) {
		if (!c->waitm_reported) {
		    mchan_active_channel = ch;
		    reportError(_("ch%d: mismatch waiting-M @M%d (ch%d at a different M-number/mask) - holding"),
			ch, c->waitm_num, mm_ch);
		    mchan_active_channel = 0;
		    c->waitm_reported = 1;
		}
		continue;					/* error + hold */
	    }
	    if (all_arrived) {
		for (m = 0; m < motion_num_channels; m++)
		    if ((mask >> m) & 1)
			emcmotInternal->chan[m].waitm_released = 1;	/* same cycle */
		continue;
	    }
	    c->waitm_t0 += servo_period;			/* elapsed wait (s) */
	    if (tmo > 0.0 && c->waitm_t0 > tmo && !c->waitm_reported) {
		int b = -1;
		for (m = 0; m < motion_num_channels; m++)
		    if ((blockers >> m) & 1) { b = m; break; }
		mchan_active_channel = ch;
		reportError(_("ch%d: waiting for ch%d @M%d (timeout %.0fs) - holding"),
		    ch, b, c->waitm_num, tmo);
		mchan_active_channel = 0;
		c->waitm_reported = 1;	/* one-shot; STAY armed (late partner still releases) */
	    }
	}
    }

    /* publish channel-scoped status + HAL observability */
    for (ch = 0; ch < motion_num_channels; ch++) {
	emcmot_channel_t *c = &emcmotInternal->chan[ch];
	int waiting = (c->waitm_num >= 0 && !c->waitm_released);
	*(emcmot_hal_data->mchan[ch].waitm_waiting)  = waiting;
	*(emcmot_hal_data->mchan[ch].waitm_number)   = waiting ? c->waitm_num : -1;
	*(emcmot_hal_data->mchan[ch].waitm_blockers) = c->waitm_blockers;
	if (ch == 0) {
	    emcmotStatus->waitm_num      = c->waitm_num;
	    emcmotStatus->waitm_released = c->waitm_released;
	    emcmotStatus->waitm_blockers = c->waitm_blockers;
	}
    }
}

/* MCHAN MC31 (I2): interference detection. Each servo cycle, transform every
 * channel's controlled point (its coord-TP cartesian) into the shared WORLD
 * frame (origin + rot*carte, frame from [CHANNEL]ORIGIN/ORIENT) and test it
 * against the declared keep-out zone. If two or more channels are inside the
 * zone at once, raise motion.interfere-active and flag each co-occupant on
 * motion.N.interfere-hold. I2 is WARN-ONLY (no motion change); I3 will turn
 * the flag into a protective feed-hold. No-op (all pins low) unless a zone is
 * configured and >=2 channels exist (D7). */
static void mchan_run_interference(void)
{
    int ch;
    int inzone[EMCMOT_MAX_CHANNELS];
    int count = 0;

    if (!emcmotInternal->interfere_zone_set || motion_num_channels < 2) {
	*(emcmot_hal_data->interfere_active) = 0;
	for (ch = 0; ch < motion_num_channels; ch++)
	    *(emcmot_hal_data->mchan[ch].interfere_hold) = 0;
	return;
    }

    const double *z = emcmotInternal->interfere_zone;	/* xmin xmax ymin ymax zmin zmax */
    for (ch = 0; ch < motion_num_channels; ch++) {
	emcmot_channel_t *c = &emcmotInternal->chan[ch];
	EmcPose p;
	tpGetPos(&c->coord_tp, &p);			/* this channel's controlled point */
	double lx = p.tran.x, ly = p.tran.y, lz = p.tran.z;
	double wx = c->origin[0] + c->rot[0][0]*lx + c->rot[0][1]*ly + c->rot[0][2]*lz;
	double wy = c->origin[1] + c->rot[1][0]*lx + c->rot[1][1]*ly + c->rot[1][2]*lz;
	double wz = c->origin[2] + c->rot[2][0]*lx + c->rot[2][1]*ly + c->rot[2][2]*lz;
	inzone[ch] = (wx >= z[0] && wx <= z[1] &&
		      wy >= z[2] && wy <= z[3] &&
		      wz >= z[4] && wz <= z[5]) ? 1 : 0;
	if (inzone[ch]) count++;
    }

    int active = (count >= 2);
    int allow = *(emcmot_hal_data->interfere_allow) ? 1 : 0;
    static int reported = 0;	/* one-shot error latch (rising edge of a stop) */
    *(emcmot_hal_data->interfere_active) = active;
    for (ch = 0; ch < motion_num_channels; ch++) {
	int hold = (active && inzone[ch]) ? 1 : 0;
	*(emcmot_hal_data->mchan[ch].interfere_hold) = hold;
	/* I3: protective stop unless the handover permit is on. The feed-scale
	 * loop (process_inputs) forces this channel's net feed to 0 next cycle.
	 * Both co-occupants stop short of contact (zone carries decel margin);
	 * the operator jogs one out / or the handover permit is asserted. */
	emcmotInternal->chan[ch].interfere_stop = (hold && !allow) ? 1 : 0;
    }
    if (active && !allow) {
	if (!reported) {
	    /* name the first two co-occupants */
	    int a = -1, b = -1;
	    for (ch = 0; ch < motion_num_channels; ch++)
		if (inzone[ch]) { if (a < 0) a = ch; else if (b < 0) { b = ch; break; } }
	    reportError(_("interference: ch%d and ch%d both in keep-out zone - protective stop (jog one clear, or assert motion.interfere-allow for a sanctioned handover)"),
		a, b);
	    reported = 1;
	}
    } else {
	reported = 0;	/* re-arm the one-shot once clear / permitted */
    }
}

static void mchan_run_secondary(long period)
{
    /* MCHAN D-MC4: a SECONDARY channel's homing session must progress
     * even when the GLOBAL state is not FREE (HOMING_INTERLOCK=own lets
     * channel 0 keep running). The legacy do_homing() call only fires in
     * the global FREE state; this one covers the rest. Called
     * UNCONDITIONALLY (not gated on homing-active): a freshly requested
     * sequence is not "active" until do_homing() runs it once -
     * chicken-and-egg found in the MC4 acceptance. Idle cost is a switch
     * on HOME_SEQUENCE_IDLE. The permit mask limits the engine to the
     * session's joints; the owned joints' homing moves execute through
     * this executor's FREE branch below. */
    if (emcmotStatus->motion_state != EMCMOT_MOTION_FREE) {
	do_homing();
    }
    for (int ch = 1; ch < motion_num_channels; ch++) {
	emcmot_channel_t *c = &emcmotInternal->chan[ch];
	int ref_jn = -1;
	for (int ax = 0; ax < EMCMOT_MAX_AXIS; ax++) {
	    if (c->axis_to_joint[ax] >= 0) {
		ref_jn = c->axis_to_joint[ax];
		break;
	    }
	}
	if (ref_jn < 0) {
	    /* nothing mapped: keep the planner clock aligned */
	    tpRunCycle(&c->coord_tp, period);
	    continue;
	}
	/* MCHAN S1: while ANY owned joint is homing, this channel's joints
	 * are driven by the homing FSM through free_tp - the stock homing
	 * pattern - NOT the channel TP. A homing session requires the
	 * channel idle (no program), so the channel TP is parked; yielding
	 * the whole channel to the free/homing drive for the cycle is safe
	 * and mirrors how channel 0 homes in FREE mode. Without it the COORD
	 * cubic path below overwrote the multi-cycle index-homing move every
	 * tick and the home-state aborted (16 -> 0) before index-enable ever
	 * armed. Immediate homing (search=latch=0) hid the bug by completing
	 * in a single tick. */
	int mchan_homing_now = 0;
	for (int ax = 0; ax < EMCMOT_MAX_AXIS; ax++) {
	    int jn = c->axis_to_joint[ax];
	    if (jn >= 0 && get_homing(jn)) { mchan_homing_now = 1; break; }
	}
	/* MC3 (D-MC3-5): the channel's OWN mode decides how its joints
	 * are driven. FREE/TELEOP = per-joint jog planners (legacy free-
	 * mode pattern, no cubic); COORD = the channel TP below. The
	 * machine-disable case never reaches here (executor is enable-
	 * gated; D5 floor holds the joints). */
	if (mchan_homing_now ||
	    c->virt_state == EMCMOT_MOTION_FREE ||
	    c->virt_state == EMCMOT_MOTION_TELEOP) {
	    for (int ax = 0; ax < EMCMOT_MAX_AXIS; ax++) {
		int jn = c->axis_to_joint[ax];
		if (jn < 0) continue;
		emcmot_joint_t *j = &joints[jn];
		j->free_tp.max_jerk = j->jerk_limit;
		/* the stock FREE executor skips owned joints, so ITS acc-limit
		 * housekeeping never runs for them - without this the homing
		 * FSM's seek move (free_tp target+enable set, max_acc never
		 * initialized = 0) freezes at zero velocity forever */
		j->free_tp.max_acc = j->acc_limit;
		simple_tp_update(&(j->free_tp), servo_period);
		j->jerk_cmd = j->free_tp.curr_jerk;
		j->pos_cmd = j->free_tp.curr_pos;
		j->vel_cmd = j->free_tp.curr_vel;
		j->acc_cmd = 0.0;
		j->coarse_pos = j->free_tp.curr_pos;
		if (!j->free_tp.active) {
		    j->kb_jjog_active = 0;
		}
	    }
	    continue;
	}
	while (cubicNeedNextPoint(&(joints[ref_jn].cubic))) {
	    EmcPose pos;
	    tpRunCycle(&c->coord_tp, period);
	    tpGetPos(&c->coord_tp, &pos);
	    for (int ax = 0; ax < EMCMOT_MAX_AXIS; ax++) {
		int jn = c->axis_to_joint[ax];
		if (jn < 0) continue;
		joints[jn].coarse_pos = mchan_pose_axis(&pos, ax);
		cubicAddPoint(&(joints[jn].cubic), joints[jn].coarse_pos);
	    }
	}
	for (int ax = 0; ax < EMCMOT_MAX_AXIS; ax++) {
	    int jn = c->axis_to_joint[ax];
	    if (jn < 0) continue;
	    joints[jn].pos_cmd = cubicInterpolate(&(joints[jn].cubic), 0,
		&(joints[jn].vel_cmd), &(joints[jn].acc_cmd), &(joints[jn].jerk_cmd));
	}
    }
}

/* MCHAN (MC2b): fill each secondary channel's status snapshot. Runs at the
 * very end of the controller cycle, after the global status is complete.
 *
 * H7 TEAR FIX (found by the user dragging ch0's feed slider - ch0 values
 * flashed in ch1's GUI): the snapshot is now BUILT in a motmod-PRIVATE
 * staging buffer (global copy + channel overlays where no reader can see
 * them) and PUBLISHED to shmem in one short head/tail-protected window.
 * The old code overlaid in place: a reader landing between the global
 * copy and the overlay consumed raw channel-0 values, and the legacy
 * "head==tail inside one copy" check cannot detect a writer that starts
 * AND finishes inside the reader's copy (proven: 3580 undetected tears /
 * 44M reads with mc2c-tear). The reader side (usrmotintf) now does a real
 * seqlock check to close that half. Never runs at num_channels=1 (D7). */

/* MCHAN: compound velocity of THIS channel's own owned joints while they
 * are being driven through free_tp (jogging or homing - coord_tp is NOT
 * involved). Needed because both the legacy global compound-velocity calc
 * below and each channel's coord_tp.current_vel are blind to free_tp
 * motion outside their own scope: the global calc summed ALL_JOINTS with
 * no ownership filter (so a secondary channel's jog/home velocity bled
 * into channel 0's DRO), and a channel's own coord_tp.current_vel is
 * always 0 during a pure jog/home (coord_tp isn't the one moving) - so a
 * secondary channel's OWN DRO never showed its jog/home velocity at all.
 * Found by the user: "ch1 DRO velocity doesn't work; during homing ch1's
 * velocity shows on ch0's DRO instead." */
static double mchan_channel_free_vel(int ch)
{
    double v2 = 0.0;
    for (int j = 0; j < ALL_JOINTS; j++) {
	if (emcmotInternal->joint_owner[j] == ch
	    && GET_JOINT_ACTIVE_FLAG(&joints[j])
	    && joints[j].free_tp.active) {
	    v2 += joints[j].vel_cmd * joints[j].vel_cmd;
	}
    }
    return (v2 > 0.0) ? sqrt(v2) : 0.0;
}

static void mchan_update_status(void)
{
    static emcmot_status_t st;	/* staging - private to motmod */
    for (int ch = 1; ch < motion_num_channels; ch++) {
	emcmot_channel_t *c = &emcmotInternal->chan[ch];
	TP_STRUCT *tp = &c->coord_tp;
	emcmot_chan_mailbox_t *mb = &emcmotStruct->mchan_cmd[ch];
	emcmot_status_t *cs = &emcmotStruct->mchan_status[ch];
	const size_t off_body = offsetof(emcmot_status_t, commandEcho);
	const size_t off_tail = offsetof(emcmot_status_t, tail);
	const size_t off_post = offsetof(emcmot_status_t, external_offsets_applied);
	int enabled = GET_MOTION_ENABLE_FLAG();
	int inpos;
	unsigned char h;

	/* ---- build the channel's view in private staging ---- */
	memcpy(&st, emcmotStatus, sizeof(emcmot_status_t));

	/* command handshake: this channel's mailbox echo */
	st.commandEcho = mb->commandEcho;
	st.commandNumEcho = mb->commandNumEcho;
	st.commandStatus = mb->commandStatus;

	/* MC10/Phase4: this channel's waiting-M view (so its task/GUI sees
	 * its OWN rendezvous state, not channel 0's) */
	st.waitm_num      = c->waitm_num;
	st.waitm_released = c->waitm_released;
	st.waitm_blockers = c->waitm_blockers;

	/* channel-scoped traj state from the channel's TP (MC19/21/22/28) */
	st.feed_scale = tp->feed_scale;
	st.rapid_scale = tp->rapid_scale;
	st.net_feed_scale = tp->net_feed_scale;
	st.enables_new = tp->enables_new;
	st.enables_queued = tp->enables_queued;
	st.planner_type = tp->planner_type;
	st.distance_to_go = tp->distance_to_go;
	st.dtg = tp->dtg;
	/* coord_tp velocity for program-driven motion; this channel's OWN
	 * free_tp velocity (jog/home) when that's what's actually moving -
	 * a channel is never doing both at once, so this is unambiguous. */
	{
	    double fv = mchan_channel_free_vel(ch);
	    st.current_vel = (fv > 0.0) ? fv : tp->current_vel;
	}
	st.requested_vel = tp->requested_vel;
	st.current_acc = tp->current_acc;
	st.current_jerk = tp->current_jerk;
	st.current_dir = tp->current_dir;
	st.spindleSync = tp->spindleSync;
	st.tcqlen = tp->tcqlen;
	st.tag = tp->execTag;
	st.vel = tp->vMax;
	st.acc = tp->aMax;
	st.id = tpGetExecId(tp);
	st.depth = tpQueueDepth(tp);
	st.activeDepth = tpActiveDepth(tp);
	st.queueFull = tcqFull(&tp->queue);
	st.motionType = tpGetMotionType(tp);
	st.paused = tp->pausing;
	st.tool_offset = c->tool_offset;

	/* channel pose: commanded from its TP while in COORD; outside COORD
	 * (FREE/TELEOP jog or homing drive the joints through free_tp while
	 * the TP is parked) compose it from the mapped joints' commanded
	 * positions, mirroring the fb loop below. Without this the
	 * commanded-position DRO (POSITION_FEEDBACK=COMMANDED) freezes at
	 * the TP's stale pose during secondary-channel jogs and homing
	 * (found by the world-jog validation: joint moved 18mm, actual
	 * tracked, commanded stayed 0.000). COORD re-entry already
	 * tpSetPos()s from the joints (zero-jump rule), so switching back
	 * is seamless. */
	if (c->virt_state == EMCMOT_MOTION_COORD) {
	    tpGetPos(tp, &st.carte_pos_cmd);
	} else {
	    ZERO_EMC_POSE(st.carte_pos_cmd);
	    for (int ax = 0; ax < EMCMOT_MAX_AXIS; ax++) {
		int jn = c->axis_to_joint[ax];
		if (jn >= 0)
		    mchan_pose_set_axis(&st.carte_pos_cmd, ax, joints[jn].pos_cmd);
	    }
	}
	st.carte_pos_cmd_ok = 1;
	ZERO_EMC_POSE(st.carte_pos_fb);
	for (int ax = 0; ax < EMCMOT_MAX_AXIS; ax++) {
	    int jn = c->axis_to_joint[ax];
	    if (jn >= 0)
		mchan_pose_set_axis(&st.carte_pos_fb, ax, joints[jn].pos_fb);
	}
	st.carte_pos_fb_ok = 1;

	/* virtual mode (recorded by the MC28 gate) + per-channel flags.
	 * inpos = this channel's planner idle; enable/estop = global floor */
	inpos = (!tpIsMoving(tp) && tpQueueDepth(tp) == 0);
	st.motion_state = !enabled ? EMCMOT_MOTION_DISABLED :
	    (c->virt_state == EMCMOT_MOTION_DISABLED) ? EMCMOT_MOTION_FREE :
	    c->virt_state;
	st.motionFlag = 0;
	if (enabled)
	    st.motionFlag |= EMCMOT_MOTION_ENABLE_BIT;
	if (inpos)
	    st.motionFlag |= EMCMOT_MOTION_INPOS_BIT;
	if (c->virt_state == EMCMOT_MOTION_COORD)
	    st.motionFlag |= EMCMOT_MOTION_COORD_BIT;
	else if (c->virt_state == EMCMOT_MOTION_TELEOP)
	    st.motionFlag |= EMCMOT_MOTION_TELEOP_BIT;

	/* not this channel's: ch0's jog machinery and probe (MC25 day-1
	 * refusal) must not leak into the channel's task decisions */
	st.overrideLimitMask = 0;
	st.jogging_active = 0;
	st.probing = 0;
	st.probeTripped = 0;

	/* ---- publish: head -> body (tail byte skipped) -> tail ---- */
	h = (unsigned char)(cs->head + 1);
	cs->head = h;
	memcpy((char *)cs + off_body, (char *)&st + off_body,
	       off_tail - off_body);
	memcpy((char *)cs + off_post, (char *)&st + off_post,
	       sizeof(emcmot_status_t) - off_post);
	cs->tail = h;
    }
}

static void get_pos_cmds(long period)
{
    int joint_num, result;
    emcmot_joint_t *joint;
    double positions[EMCMOT_MAX_JOINTS];
    double vel_lim;

    /* used in teleop mode to compute the max accell requested */
    int onlimit = 0;
    int joint_limit[EMCMOT_MAX_JOINTS][2];

    /* copy joint position feedback to local array */
    for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
	/* point to joint struct */
	joint = &joints[joint_num];
	/* copy coarse command */
	positions[joint_num] = joint->coarse_pos;
    }
    /* if less than a full complement of joints, zero out the rest */
    while ( joint_num < EMCMOT_MAX_JOINTS ) {
        positions[joint_num++] = 0.0;
    }

    /* RUN MOTION CALCULATIONS: */

    /* MCHAN (MC3-lite): secondary channels execute whenever the machine is
     * enabled, independent of channel 0's motion state. When disabled, the
     * DISABLED case below holds ALL joints (including secondary-owned ones)
     * at feedback - the global-stop floor (D5). */
    if (GET_MOTION_ENABLE_FLAG()) {
	mchan_run_secondary(period);
    }

    /* run traj planner code depending on the state */
    switch ( emcmotStatus->motion_state) {
    case EMCMOT_MOTION_FREE:
	/* in free mode, each joint is planned independently */
	/* initial value for flag, if needed it will be cleared below */
	SET_MOTION_INPOS_FLAG(1);
	for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
	    /* point to joint struct */
	    joint = &joints[joint_num];
	    if (!GET_JOINT_ACTIVE_FLAG(joint)) {
	        /* if joint is not active, skip it */
	        continue;
            }
            // extra joint is not managed herein after homing:
            if (IS_EXTRA_JOINT(joint_num) && get_homed(joint_num)) continue;
	    /* MCHAN: joints owned by a secondary channel are driven by that
	     * channel's planner (mchan_run_secondary), not free-planned here */
	    if (emcmotInternal->joint_owner[joint_num] != 0) continue;

	    if(joint->acc_limit > emcmotStatus->acc)
		joint->acc_limit = emcmotStatus->acc;
        if(joint->jerk_limit > emcmotStatus->jerk)
        joint->jerk_limit = emcmotStatus->jerk;
	    /* compute joint velocity limit */
            if (   (emcmotStatus->motion_state != EMCMOT_MOTION_FREE)
                && get_home_is_idle(joint_num) ) {
                /* velocity limit = joint limit * global scale factor */
                /* the global factor is used for feedrate override */
                vel_lim = joint->vel_limit * emcmotStatus->net_feed_scale;
                /* must not be greater than the joint physical limit */
                if (vel_lim > joint->vel_limit) {
                    vel_lim = joint->vel_limit;
                }
                /* set vel limit in free TP */
               if (vel_lim < joint->free_tp.max_vel)
                   joint->free_tp.max_vel = vel_lim;
            } else {
                /* except if homing, when we set free_tp max vel in do_homing */
            }
            /* set acc limit in free TP */
            /* execute free TP */
            if (joint->wheel_jjog_active) {
                double jaccel_limit;
                joint_hal_t *joint_data;
                joint_data = &(emcmot_hal_data->joint[joint_num]);
                if (    (*(joint_data->jjog_accel_fraction) > 1)
                     || (*(joint_data->jjog_accel_fraction) < 0) ) {
                     jaccel_limit = joint->acc_limit;
                } else {
                   jaccel_limit = (*(joint_data->jjog_accel_fraction)) * joint->acc_limit;
                }
                joint->free_tp.max_acc = jaccel_limit;
            } else {
                joint->free_tp.max_acc = joint->acc_limit;
            }
            joint->free_tp.max_jerk = joint->jerk_limit;
            simple_tp_update(&(joint->free_tp), servo_period );
            /* copy free TP output to pos_cmd and coarse_pos */
            joint->jerk_cmd = joint->free_tp.curr_jerk;
            joint->pos_cmd = joint->free_tp.curr_pos;
            joint->vel_cmd = joint->free_tp.curr_vel;
            //no acceleration output form simple_tp, but the pin will
            //still show the acceleration from the interpolation.
            //it's delayed, but that's ok during jogging or homing.
            joint->acc_cmd = 0.0;
            joint->coarse_pos = joint->free_tp.curr_pos;
            /* update joint status flag and overall status flag */
            if ( joint->free_tp.active ) {
		/* active TP means we're moving, so not in position */
		SET_JOINT_INPOS_FLAG(joint, 0);
		SET_MOTION_INPOS_FLAG(0);
		/* is any limit disabled for this move? */
		if ( emcmotStatus->overrideLimitMask ) {
                    emcmotInternal->overriding = 1;
		}
            } else {
		SET_JOINT_INPOS_FLAG(joint, 1);
		/* joint has stopped, so any outstanding jogs are done */
		joint->kb_jjog_active = 0;
		joint->wheel_jjog_active = 0;
            }
	}//for loop for joints
	/* if overriding is true and we're in position, the jog
	   is complete, and the limits should be re-enabled */
	if ( (emcmotInternal->overriding ) && ( GET_MOTION_INPOS_FLAG() ) ) {
	    emcmotStatus->overrideLimitMask = 0;
	    emcmotInternal->overriding = 0;
	}
	/*! \todo FIXME - this should run at the traj rate */
	switch (emcmotConfig->kinType) {

	case KINEMATICS_IDENTITY:
	    kinematicsForward(positions, &emcmotStatus->carte_pos_cmd, &fflags, &iflags);
	    if (get_allhomed()) {
		emcmotStatus->carte_pos_cmd_ok = 1;
	    } else {
		emcmotStatus->carte_pos_cmd_ok = 0;
	    }
	    break;

	case KINEMATICS_BOTH:
	    if (get_allhomed()) {
		/* is previous value suitable for use as initial guess? */
		if (!emcmotStatus->carte_pos_cmd_ok) {
		    /* no, use home position as initial guess */
		    emcmotStatus->carte_pos_cmd = emcmotStatus->world_home;
		}
		/* calculate Cartesean position command from joint coarse pos cmd */
		result =
		    kinematicsForward(positions, &emcmotStatus->carte_pos_cmd, &fflags, &iflags);
		/* check to make sure kinematics converged */
		if (result < 0) {
		    /* error during kinematics calculations */
		    emcmotStatus->carte_pos_cmd_ok = 0;
		} else {
		    /* it worked! */
		    emcmotStatus->carte_pos_cmd_ok = 1;
		}
	    } else {
		emcmotStatus->carte_pos_cmd_ok = 0;
	    }
	    break;

	case KINEMATICS_INVERSE_ONLY:
	    emcmotStatus->carte_pos_cmd_ok = 0;
	    break;

	default:
	    emcmotStatus->carte_pos_cmd_ok = 0;
	    break;
	}
        /* end of FREE mode */
	break;

    case EMCMOT_MOTION_COORD:
        axis_jog_abort_all(1);

	/* check joint 0 to see if the interpolators are empty */
	coord_cubic_active = 1;
	{
	/* MCHAN (MC1): accumulate this cycle's coordinated-TP cost; the loop
	 * can iterate more than once while the interpolators fill. */
	long long tp_ns_acc = 0;
	while (cubicNeedNextPoint(&(joints[0].cubic))) {
	    /* they're empty, pull next point(s) off Cartesian planner */
	    /* run coordinated trajectory planning cycle */

	    long long tp_t0 = rtapi_get_time();
	    tpRunCycle(&emcmotInternal->chan[0].coord_tp, period);
            /* get new commanded traj pos */
            tpGetPos(&emcmotInternal->chan[0].coord_tp, &emcmotStatus->carte_pos_cmd);
	    tp_ns_acc += rtapi_get_time() - tp_t0;

            if (axis_update_coord_with_bound(pcmd_p, servo_period)) {
                ext_offset_coord_limit = 1;
            } else {
                ext_offset_coord_limit = 0;
            }

	    /* OUTPUT KINEMATICS - convert to joints in local array */
	    result = kinematicsInverse(&emcmotStatus->carte_pos_cmd, positions,
		&iflags, &fflags);
	    if(result == 0)
	    {
		/* copy to joint structures and spline them up */
		for (joint_num = 0; joint_num < NO_OF_KINS_JOINTS; joint_num++) {
		    /* MCHAN: secondary-owned joints are splined by their
		     * channel's planner, not by channel 0's kins output */
		    if (emcmotInternal->joint_owner[joint_num] != 0) continue;
		    if(!isfinite(positions[joint_num]))
		    {
                       reportError(_("kinematicsInverse gave non-finite joint location on joint %d"),
                           joint_num);
                       SET_MOTION_ERROR_FLAG(1);
                       emcmotInternal->enabling = 0;
                       break;
		    }
		    /* point to joint struct */
		    joint = &joints[joint_num];
		    joint->coarse_pos = positions[joint_num];
		    /* spline joints up-- note that we may be adding points
		       that fail soft limits, but we'll abort at the end of
		       this cycle so it doesn't really matter */
		    cubicAddPoint(&(joint->cubic), joint->coarse_pos);
		}
	    }
	    else
	    {
	       reportError(_("kinematicsInverse failed"));
	       SET_MOTION_ERROR_FLAG(1);
	       emcmotInternal->enabling = 0;
	       break;
	    }

	    /* END OF OUTPUT KINS */
	} // while
	/* MCHAN (MC1): publish this cycle's TP cost */
	*(emcmot_hal_data->tp_time_last) = (hal_s32_t)tp_ns_acc;
	if ((hal_s32_t)tp_ns_acc > *(emcmot_hal_data->tp_time_max))
	    *(emcmot_hal_data->tp_time_max) = (hal_s32_t)tp_ns_acc;
	}
	/* there is data in the interpolators */
	/* run interpolation */
	for (joint_num = 0; joint_num < NO_OF_KINS_JOINTS; joint_num++) {
	    /* MCHAN: secondary-owned joints interpolate in mchan_run_secondary */
	    if (emcmotInternal->joint_owner[joint_num] != 0) continue;
	    /* point to joint struct */
	    joint = &joints[joint_num];
	    /* interpolate to get new position and velocity */
		joint->pos_cmd = cubicInterpolate(&(joint->cubic), 0, &(joint->vel_cmd), &(joint->acc_cmd),  &(joint->jerk_cmd));
	}

	/* Use accurate jerk values from TP output for identity kinematics only.
	 * For KINEMATICS_BOTH (non-trivial joint mapping), joint indices don't
	 * necessarily correspond to XYZ axes, so keep cubic interpolator values.
	 */
	if (emcmotStatus->planner_type == 1
	    && emcmotConfig->kinType == KINEMATICS_IDENTITY) {
	    double path_jerk = emcmotStatus->current_jerk;
	    PmCartesian dir = emcmotStatus->current_dir;
	    if (NO_OF_KINS_JOINTS >= 1) joints[0].jerk_cmd = path_jerk * dir.x;
	    if (NO_OF_KINS_JOINTS >= 2) joints[1].jerk_cmd = path_jerk * dir.y;
	    if (NO_OF_KINS_JOINTS >= 3) joints[2].jerk_cmd = path_jerk * dir.z;
	}

	/* report motion status */
	SET_MOTION_INPOS_FLAG(0);
	if (tpIsDone(&emcmotInternal->chan[0].coord_tp)) {
	    SET_MOTION_INPOS_FLAG(1);
	}
	break;

    case EMCMOT_MOTION_TELEOP:
        ext_offset_teleop_limit = axis_calc_motion(servo_period);
        if (!ext_offset_teleop_limit) {
            ext_offset_coord_limit = 0; //in case was set in prior coord motion
        }

        axis_sync_carte_pos_to_teleop_tp(+1, pcmd_p); // teleop

	if ( axis_jog_is_active() ) {
	    /* is any limit disabled for this move? */
	    if ( emcmotStatus->overrideLimitMask ) {
		emcmotInternal->overriding = 1;
	    }
	}

	/* the next position then gets run through the inverse kins,
	    to compute the next positions of the joints */

	/* OUTPUT KINEMATICS - convert to joints in local array */
	result = kinematicsInverse(&emcmotStatus->carte_pos_cmd, positions, &iflags, &fflags);

	/* copy to joint structures and spline them up */
	if(result == 0)
	{
	    for (joint_num = 0; joint_num < NO_OF_KINS_JOINTS; joint_num++) {
		/* MCHAN: secondary-owned joints are driven by their channel's
		 * planner, not by channel 0's teleop kins output */
		if (emcmotInternal->joint_owner[joint_num] != 0) continue;
		if(!isfinite(positions[joint_num]))
		{
		   reportError(_("kinematicsInverse gave non-finite joint location on joint %d"),
		         joint_num);
		   SET_MOTION_ERROR_FLAG(1);
		   emcmotInternal->enabling = 0;
		   break;
		}
		/* point to joint struct */
		joint = &joints[joint_num];
		joint->coarse_pos = positions[joint_num];
		/* spline joints up-- note that we may be adding points
		       that fail soft limits, but we'll abort at the end of
		       this cycle so it doesn't really matter */
		cubicAddPoint(&(joint->cubic), joint->coarse_pos);
		/* interpolate to get new position and velocity */
		joint->pos_cmd = cubicInterpolate(&(joint->cubic), 0, &(joint->vel_cmd), &(joint->acc_cmd),  &(joint->jerk_cmd));
	    }
	}
	else
	{
	   reportError(_("kinematicsInverse failed"));
	   SET_MOTION_ERROR_FLAG(1);
	   emcmotInternal->enabling = 0;
	   break;
	}


	/* END OF OUTPUT KINS */

	/* if overriding is true and the jog is complete, the limits should be re-enabled */
	if ( ( emcmotInternal->overriding ) && ( !axis_jog_is_active() ) ) {
	    emcmotStatus->overrideLimitMask = 0;
	    emcmotInternal->overriding = 0;
	}

	/* end of teleop mode */

	break;

    case EMCMOT_MOTION_DISABLED:
	/* set position commands to match feedbacks, this avoids
	   disturbances and/or following errors when enabling */
	emcmotStatus->carte_pos_cmd = emcmotStatus->carte_pos_fb;
	for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
	    /* point to joint struct */
	    joint = &joints[joint_num];
	    /* save old command */
	    joint->pos_cmd = joint->pos_fb;
	    /* set joint velocity and acceleration to zero */
	    joint->vel_cmd = 0.0;
	    joint->acc_cmd = 0.0;
	}

	break;
    default:
	break;
    }
    /* check command against soft limits */
    /* This is a backup check, it should be impossible to command
	a move outside the soft limits.  However there is at least
	two cases that isn't caught upstream:
	1) if an arc has both endpoints inside the limits, but the curve extends outside,
	2) if homing params are wrong then after homing joint pos_cmd are outside,
	the upstream checks will pass it.
    */
    for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
	/* point to joint data */
	joint = &joints[joint_num];
	
	/* Zero values */
	joint_limit[joint_num][0] = 0;
	joint_limit[joint_num][1] = 0;
	
	/* skip inactive or unhomed axes */
	if ((!GET_JOINT_ACTIVE_FLAG(joint)) || (!get_homed(joint_num))) {
	    continue;
        }

	/* check for soft limits */
	if (joint->pos_cmd > joint->max_pos_limit + 0.000000000001) {
	    joint_limit[joint_num][1] = 1;
            onlimit = 1;
        }
        else if (joint->pos_cmd < joint->min_pos_limit - 0.000000000001) {
	    joint_limit[joint_num][0] = 1;
            onlimit = 1;
        }
    }
    if ( onlimit ) {
	if ( ! emcmotStatus->on_soft_limit ) {
        /* Unexpectedly hit a joint soft limit.
        ** Possible causes:
        **  1) a joint positional limit was reduced by an INI halpin
        **     (like ini.N.max_limit) -- undetected by trajectory planning
        **     including simple_tp
        **  2) issues like https://github.com/LinuxCNC/linuxcnc/issues/80
        **  3) kins module misbehavior
        **  4) poorly tuned servo motion (not detected by ferror settings)
        **
        ** Non-identity kins can often be switched to joint mode to recover
        ** using the '$' shortcut provided by the gui.
        ** Guis may not provide a means to recover for identity kins except
        ** by unhoming/jogging/rehoming.  (For trivkins, using kinstype=both
        ** can be used as a workaround).
        **
        */
	    for (joint_num = 0; joint_num < emcmotConfig->numJoints; joint_num++) {
	        if (joint_limit[joint_num][0] == 1) {
                    joint = &joints[joint_num];
                    reportError(_("Exceeded NEGATIVE soft limit (%.5f) on joint %d\n"),
                                  joint->min_pos_limit, joint_num);
                    if (emcmotConfig->kinType == KINEMATICS_IDENTITY) {
                        reportError(_("Joint must be unhomed, jogged into limits, rehomed"));
                    } else {
                        reportError(_("Hint: switch to joint mode to jog off soft limit"));
                    }
                } else if (joint_limit[joint_num][1] == 1) {
                    joint = &joints[joint_num];
                    reportError(_("Exceeded POSITIVE soft limit (%.5f) on joint %d\n"),
                                  joint->max_pos_limit,joint_num);
                    if (emcmotConfig->kinType == KINEMATICS_IDENTITY) {
                        reportError(_("Joint must be unhomed, jogged into limits, rehomed"));
                    } else {
                        reportError(_("Hint: switch to joint mode to jog off soft limit"));
                    }
                }
	    }
	    SET_MOTION_ERROR_FLAG(1);
	    emcmotStatus->on_soft_limit = 1;
	}
    } else {
	emcmotStatus->on_soft_limit = 0;
    }
    if (   emcmotInternal->teleoperating
        && GET_MOTION_TELEOP_FLAG()
        && emcmotStatus->on_soft_limit ) {
        SET_MOTION_ERROR_FLAG(1);
        axis_jog_abort_all(1);
    }
    if (ext_offset_teleop_limit || ext_offset_coord_limit) {
        *(emcmot_hal_data->eoffset_limited) = 1;
    } else {
        *(emcmot_hal_data->eoffset_limited) = 0;
    }
} // get_pos_cmds()

/* NOTES:  These notes are just my understanding of how things work.

There are seven sets of position information.

1) emcmotStatus->carte_pos_cmd
2) emcmotStatus->joints[n].coarse_pos
3) emcmotStatus->joints[n].pos_cmd
4) emcmotStatus->joints[n].motor_pos_cmd
5) emcmotStatus->joints[n].motor_pos_fb
6) emcmotStatus->joints[n].pos_fb
7) emcmotStatus->carte_pos_fb

Their exact contents and meaning are as follows:

1) This is the desired position, in Cartesean coordinates.  It is
   updated at the traj rate, not the servo rate.
   In coord mode, it is determined by the traj planner
   In teleop mode, it is determined by the traj planner?
   In free mode, it is not used, since free mode motion takes
     place in joint space, not cartesean space.  It may be
     displayed by the GUI however, so it is updated by
     applying forward kins to (2), unless forward kins are
     not available, in which case it is copied from (7).

2) This is the desired position, in joint coordinates, but
   before interpolation.  It is updated at the traj rate, not
   the servo rate..
   In coord mode, it is generated by applying inverse kins to (1)
   In teleop mode, it is generated by applying inverse kins to (1)
   In free mode, it is not used, since the free mode planner generates
     a new (3) position every servo period without interpolation.
     However, it is used indirectly by GUIs, so it is copied from (3).

3) This is the desired position, in joint coords, after interpolation.
   A new set of these coords is generated every servo period.
   In coord mode, it is generated from (2) by the interpolator.
   In teleop mode, it is generated from (2) by the interpolator.
   In free mode, it is generated by the simple free mode traj planner.

4) This is the desired position, in motor coords.  Motor coords are
   generated by adding backlash compensation, lead screw error
   compensation, and offset (for homing) to (3).
   It is generated the same way regardless of the mode, and is the
   output to the PID loop or other position loop.

5) This is the actual position, in motor coords.  It is the input from
   encoders or other feedback device (or from virtual encoders on open
   loop machines).  It is "generated" by reading the feedback device.

6) This is the actual position, in joint coordinates.  It is generated
   by subtracting offset, lead screw error compensation, and backlash
   compensation from (5).  It is generated the same way regardless of
   the operating mode.

7) This is the actual position, in Cartesean coordinates.  It is updated
   at the traj rate, not the servo rate.
   OLD VERSION:
   In the old version, there are four sets of code to generate actualPos.
   One for each operating mode, and one for when motion is disabled.
   The code for coord and teleop modes is identical.  The code for free
   mode is somewhat different, in particular to deal with the case where
   one or more axes are not homed.  The disabled code is quite similar,
   but not identical, to the coord mode code.  In general, the code
   calculates actualPos by applying the forward kins to (6).  However,
   where forward kins are not available, actualPos is either copied
   from (1) (assumes no following error), or simply left alone.
   These special cases are handled differently for each operating mode.
   NEW VERSION:
   I would like to both simplify and relocate this.  As far as I can
   tell, actualPos should _always_ be the best estimate of the actual
   machine position in Cartesean coordinates.  So it should always be
   calculated the same way.
   In addition to always using the same code to calculate actualPos,
   I want to move that code.  It is really a feedback calculation, and
   as such it belongs with the rest of the feedback calculations early
   in control.c, not as part of the output generation code as it is now.
   Ideally, actualPos would always be calculated by applying forward
   kinematics to (6).  However, forward kinematics may not be available,
   or they may be unusable because one or more axes aren't homed.  In
   that case, the options are: A) fake it by copying (1), or B) admit
   that we don't really know the Cartesean coordinates, and simply
   don't update actualPos.  Whatever approach is used, I can see no
   reason not to do it the same way regardless of the operating mode.
   I would propose the following:  If there are forward kins, use them,
   unless they don't work because of unhomed axes or other problems,
   in which case do (B).  If no forward kins, do (A), since otherwise
   actualPos would _never_ get updated.

*/

static void compute_screw_comp(void)
{
    int joint_num;
    emcmot_joint_t *joint;
    emcmot_comp_t *comp;
    double dpos;
    double a_max, v_max, v, s_to_go, ds_stop, ds_vel, ds_acc, dv_acc;


    /* compute the correction */
    for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
        /* point to joint struct */
        joint = &joints[joint_num];
	if (!GET_JOINT_ACTIVE_FLAG(joint)) {
	    /* if joint is not active, skip it */
	    continue;
	}
	/* point to compensation data */
	comp = &(joint->comp);
	if ( comp->entries > 0 ) {
	    /* there is data in the comp table, use it */
	    /* first make sure we're in the right spot in the table */
	    while ( joint->pos_cmd < comp->entry->nominal ) {
		comp->entry--;
	    }
	    while ( joint->pos_cmd >= (comp->entry+1)->nominal ) {
		comp->entry++;
	    }
	    /* now interpolate */
	    dpos = joint->pos_cmd - comp->entry->nominal;
	    if (joint->vel_cmd > 0.0) {
	        /* moving "up". apply forward screw comp */
		joint->backlash_corr = comp->entry->fwd_trim +
					comp->entry->fwd_slope * dpos;
	    } else if (joint->vel_cmd < 0.0) {
	        /* moving "down". apply reverse screw comp */
		joint->backlash_corr = comp->entry->rev_trim +
					comp->entry->rev_slope * dpos;
	    } else {
		/* not moving, use whatever was there before */
	    }
	} else {
	    /* no compensation data, just use +/- 1/2 of backlash */
	    /** FIXME: this can actually be removed - if the user space code
		sends a single compensation entry with any nominal value,
		and with fwd_trim = +0.5 times the backlash value, and
		rev_trim = -0.5 times backlash, the above screw comp code
		will give exactly the same result as this code. */
	    /* determine which way the compensation should be applied */
	    if (joint->vel_cmd > 0.0) {
	        /* moving "up". apply positive backlash comp */
		joint->backlash_corr = 0.5 * joint->backlash;
	    } else if (joint->vel_cmd < 0.0) {
	        /* moving "down". apply negative backlash comp */
		joint->backlash_corr = -0.5 * joint->backlash;
	    } else {
		/* not moving, use whatever was there before */
	    }
	}
	/* at this point, the correction has been computed, but
	   the value may make abrupt jumps on direction reversal */
    /*
     * 07/09/2005 - S-curve implementation by Bas Laarhoven
     *
     * Implementation:
     *   Generate a ramped velocity profile for backlash or screw error comp.
     *   The velocity is ramped up to the maximum speed setting (if possible),
     *   using the maximum acceleration setting.
     *   At the end, the speed is ramped dowm using the same acceleration.
     *   The algorithm keeps looking ahead. Depending on the distance to go,
     *   the speed is increased, kept constant or decreased.
     *
     * Limitations:
     *   Since the compensation adds up to the normal movement, total
     *   acceleration and total velocity may exceed maximum settings!
     *   Currently this is limited to 150% by implementation.
     *   To fix this, the calculations in get_pos_cmd should include
     *   information from the backlash correction. This makes things
     *   rather complicated and it might be better to implement the
     *   backlash compensation at another place to prevent this kind
     *   of interaction.
     *   More testing under different circumstances will show if this
     *   needs a more complicate solution.
     *   For now this implementation seems to generate smoother
     *   movements and less following errors than the original code.
     */

	/* Limit maximum acceleration and velocity 'overshoot'
	 * to 150% of the maximum settings.
	 * The TP and backlash shouldn't use more than 100%
	 * (together) but this requires some interaction that
	 * isn't implemented yet.
	 */
        v_max = 0.5 * joint->vel_limit * emcmotStatus->net_feed_scale;
        a_max = 0.5 * joint->acc_limit;
        v = joint->backlash_vel;
        if (joint->backlash_corr >= joint->backlash_filt) {
            s_to_go = joint->backlash_corr - joint->backlash_filt; /* abs val */
            if (s_to_go > 0) {
                // off target, need to move
                ds_vel  = v * servo_period;           /* abs val */
                dv_acc  = a_max * servo_period;       /* abs val */
                ds_stop = 0.5 * (v + dv_acc) *
		                (v + dv_acc) / a_max; /* abs val */
                if (s_to_go <= ds_stop + ds_vel) {
                    // ramp down
                    if (v > dv_acc) {
                        // decellerate one period
                        ds_acc = 0.5 * dv_acc * servo_period; /* abs val */
                        joint->backlash_vel  -= dv_acc;
                        joint->backlash_filt += ds_vel - ds_acc;
                    } else {
                        // last step to target
                        joint->backlash_vel  = 0.0;
                        joint->backlash_filt = joint->backlash_corr;
                    }
                } else {
                    if (v + dv_acc > v_max) {
                        dv_acc = v_max - v;                /* abs val */
                    }
                    ds_acc  = 0.5 * dv_acc * servo_period; /* abs val */
                    ds_stop = 0.5 * (v + dv_acc) *
                                    (v + dv_acc) / a_max;  /* abs val */
                    if (s_to_go > ds_stop + ds_vel + ds_acc) {
                        // ramp up
                       joint->backlash_vel  += dv_acc;
                       joint->backlash_filt += ds_vel + ds_acc;
                    } else {
                       // constant velocity
                       joint->backlash_filt += ds_vel;
                    }
                }
            } else if (s_to_go < 0) {
                // safely handle overshoot (should not occur)
               joint->backlash_vel = 0.0;
               joint->backlash_filt = joint->backlash_corr;
            }
        } else {  /* joint->backlash_corr < 0.0 */
            s_to_go = joint->backlash_filt - joint->backlash_corr; /* abs val */
            if (s_to_go > 0) {
                // off target, need to move
                ds_vel  = -v * servo_period;          /* abs val */
                dv_acc  = a_max * servo_period;       /* abs val */
                ds_stop = 0.5 * (v - dv_acc) *
			        (v - dv_acc) / a_max; /* abs val */
                if (s_to_go <= ds_stop + ds_vel) {
                    // ramp down
                    if (-v > dv_acc) {
                        // decellerate one period
                        ds_acc = 0.5 * dv_acc * servo_period; /* abs val */
                        joint->backlash_vel  += dv_acc;   /* decrease */
                        joint->backlash_filt -= ds_vel - ds_acc;
                    } else {
                        // last step to target
                        joint->backlash_vel = 0.0;
                        joint->backlash_filt = joint->backlash_corr;
                    }
                } else {
                    if (-v + dv_acc > v_max) {
                        dv_acc = v_max + v;               /* abs val */
                    }
                    ds_acc = 0.5 * dv_acc * servo_period; /* abs val */
                    ds_stop = 0.5 * (v - dv_acc) *
                                    (v - dv_acc) / a_max; /* abs val */
                    if (s_to_go > ds_stop + ds_vel + ds_acc) {
                        // ramp up
                        joint->backlash_vel  -= dv_acc;   /* increase */
                        joint->backlash_filt -= ds_vel + ds_acc;
                    } else {
                        // constant velocity
                        joint->backlash_filt -= ds_vel;
                    }
                }
            } else if (s_to_go < 0) {
                // safely handle overshoot (should not occur)
                joint->backlash_vel = 0.0;
                joint->backlash_filt = joint->backlash_corr;
            }
        }
        /* backlash (and motor offset) will be applied to output later */
        /* end of joint loop */
    }
}

/*! \todo FIXME - once the HAL refactor is done so that metadata isn't stored
   in shared memory, I want to seriously consider moving some of the
   structures into the HAL memory block.  This will eliminate most of
   this useless copying, and make nearly everything accessible to
   halscope and halmeter for debugging.
*/

static void output_to_hal(void)
{
    int joint_num, spindle_num;
    double inch_mult;
    emcmot_joint_t *joint;
    joint_hal_t *joint_data;
    static int old_motion_index[EMCMOT_MAX_SPINDLES] = {0};
    static int old_hal_index[EMCMOT_MAX_SPINDLES] = {0};

    /* output machine info to HAL for scoping, etc */
    *(emcmot_hal_data->motion_enabled) = GET_MOTION_ENABLE_FLAG();
    *(emcmot_hal_data->in_position) = GET_MOTION_INPOS_FLAG();
    *(emcmot_hal_data->coord_mode) = GET_MOTION_COORD_FLAG();
    *(emcmot_hal_data->teleop_mode) = GET_MOTION_TELEOP_FLAG();
    *(emcmot_hal_data->coord_error) = GET_MOTION_ERROR_FLAG();
    *(emcmot_hal_data->on_soft_limit) = emcmotStatus->on_soft_limit;

    /* Performance Metadata */
    *(emcmot_hal_data->interp_feedrate)         = emcmotStatus->tag.fields_float[GM_FIELD_FLOAT_FEED];

    /* Line and Motion Type (Casting to int for s32 HAL pins) */
    *(emcmot_hal_data->interp_line_number)      = (int)emcmotStatus->tag.fields[GM_FIELD_LINE_NUMBER];
    *(emcmot_hal_data->interp_motion_type)      = (int)emcmotStatus->tag.fields[GM_FIELD_MOTION_MODE];
    *(emcmot_hal_data->iscircle)                = (hal_bit_t)((emcmotStatus->tag.packed_flags & (1UL << GM_FLAG_IS_CIRCLE)) != 0);
    switch (emcmotStatus->motionType) {
        case EMC_MOTION_TYPE_FEED: //fall thru
        case EMC_MOTION_TYPE_ARC:
            if (emcmotStatus->tag.packed_flags & 1 << GM_FLAG_UNITS) {
                inch_mult = 1;
            } else {
                inch_mult = 1 / 25.4;
            }
            *(emcmot_hal_data->feed_upm) = emcmotStatus->tag.fields_float[GM_FIELD_FLOAT_FEED]
                                         * emcmotStatus->net_feed_scale;
            *(emcmot_hal_data->feed_inches_per_minute) = *emcmot_hal_data->feed_upm * inch_mult;
            *(emcmot_hal_data->feed_inches_per_second) = *emcmot_hal_data->feed_inches_per_minute / 60;
            *(emcmot_hal_data->feed_mm_per_minute) = *emcmot_hal_data->feed_inches_per_minute * 25.4;
            *(emcmot_hal_data->feed_mm_per_second) = *emcmot_hal_data->feed_mm_per_minute / 60;
            break;
        default:
            *(emcmot_hal_data->feed_upm) = 0;
            *(emcmot_hal_data->feed_inches_per_minute) = 0;
            *(emcmot_hal_data->feed_inches_per_second) = 0;
            *(emcmot_hal_data->feed_mm_per_minute) = 0;
            *(emcmot_hal_data->feed_mm_per_second) = 0;
    }

    for (spindle_num = 0; spindle_num < emcmotConfig->numSpindles; spindle_num++){
        double speed;
		if(emcmotStatus->spindle_status[spindle_num].css_factor) {
			double denom = fabs(emcmotStatus->spindle_status[spindle_num].xoffset
								- emcmotStatus->carte_pos_cmd.tran.x);
			double maxpositive;
			if(denom > 0) speed = emcmotStatus->spindle_status[spindle_num].css_factor / denom;
			else speed = emcmotStatus->spindle_status[spindle_num].speed;

			speed = speed * emcmotStatus->spindle_status[spindle_num].net_scale;
				maxpositive = fabs(emcmotStatus->spindle_status[spindle_num].speed);
				// cap speed to G96 D...
				if(speed < -maxpositive)
					speed = -maxpositive;
				if(speed > maxpositive)
					speed = maxpositive;
		} else {
			speed = emcmotStatus->spindle_status[spindle_num].speed *
					emcmotStatus->spindle_status[spindle_num].net_scale;
		}

        // Limit to spindle velocity limits
        if (speed > 0){
            if (speed > emcmotStatus->spindle_status[spindle_num].max_pos_speed) {
                speed = emcmotStatus->spindle_status[spindle_num].max_pos_speed;
            } else if (speed < emcmotStatus->spindle_status[spindle_num].min_pos_speed) {
                speed = emcmotStatus->spindle_status[spindle_num].min_pos_speed;
            }
        } else if (speed < 0) {
            if (speed < emcmotStatus->spindle_status[spindle_num].min_neg_speed) {
                speed = emcmotStatus->spindle_status[spindle_num].min_neg_speed;
            } else if (speed > emcmotStatus->spindle_status[spindle_num].max_neg_speed) {
                speed = emcmotStatus->spindle_status[spindle_num].max_neg_speed;
            }
        }

	*(emcmot_hal_data->spindle[spindle_num].spindle_speed_out) = speed;
	*(emcmot_hal_data->spindle[spindle_num].spindle_speed_out_rps) = speed/60.;
	*(emcmot_hal_data->spindle[spindle_num].spindle_speed_out_abs) = fabs(speed);
	*(emcmot_hal_data->spindle[spindle_num].spindle_speed_out_rps_abs) = fabs(speed / 60);
	*(emcmot_hal_data->spindle[spindle_num].spindle_on) = 
        ((emcmotStatus->spindle_status[spindle_num].state) !=0) ? 1 : 0;
	*(emcmot_hal_data->spindle[spindle_num].spindle_forward) = (speed > 0) ? 1 : 0;
	*(emcmot_hal_data->spindle[spindle_num].spindle_reverse) = (speed < 0) ? 1 : 0;
	*(emcmot_hal_data->spindle[spindle_num].spindle_brake) =
		    (emcmotStatus->spindle_status[spindle_num].brake != 0) ? 1 : 0;
        // What is this for? Docs don't say
        *(emcmot_hal_data->spindle[spindle_num].spindle_speed_cmd_rps) =
				emcmotStatus->spindle_status[spindle_num].speed / 60.;
    }

    *(emcmot_hal_data->program_line) = emcmotStatus->id;
    *(emcmot_hal_data->tp_reverse) = emcmotStatus->reverse_run;
    *(emcmot_hal_data->motion_type) = emcmotStatus->motionType;
    *(emcmot_hal_data->distance_to_go) = emcmotStatus->distance_to_go;
    if(GET_MOTION_COORD_FLAG()) {
        *(emcmot_hal_data->current_vel) = emcmotStatus->current_vel;
        *(emcmot_hal_data->requested_vel) = emcmotStatus->requested_vel;
    } else if (GET_MOTION_TELEOP_FLAG()) {
        emcmotStatus->current_vel = (*emcmot_hal_data->current_vel) = axis_get_compound_velocity();
        *(emcmot_hal_data->requested_vel) = 0.0;
    } else {
        /* MCHAN: legacy/global view = channel 0's own perspective - scope
         * to joints channel 0 owns, so a secondary channel's jog/home
         * velocity doesn't bleed into this (channel 0's) DRO. */
        emcmotStatus->current_vel = (*emcmot_hal_data->current_vel) = mchan_channel_free_vel(0);
        *(emcmot_hal_data->requested_vel) = 0.0;
    }

    /* MCHAN MC5: per-channel motion outputs (motion.N.is-moving / .current-vel).
     * is-moving = the channel's coord TP is running OR any joint it owns is
     * running a free/jog/homing move (so ch0 jogs/teleop count too).
     * current-vel = the channel coord TP velocity, or this channel's own
     * free_tp (jog/home) velocity when THAT's what's actually moving. */
    {
	int ch, j;
	for (ch = 0; ch < motion_num_channels; ch++) {
	    TP_STRUCT *ctp = &emcmotInternal->chan[ch].coord_tp;
	    int mv = tpIsMoving(ctp);
	    if (!mv) {
		for (j = 0; j < ALL_JOINTS; j++) {
		    if (emcmotInternal->joint_owner[j] == ch
			&& joints[j].free_tp.active) { mv = 1; break; }
		}
	    }
	    *(emcmot_hal_data->mchan[ch].is_moving) = mv;
	    {
		double fv = mchan_channel_free_vel(ch);
		*(emcmot_hal_data->mchan[ch].current_vel) =
		    (fv > 0.0) ? fv : ctp->current_vel;
	    }
	    /* MC7: per-channel run-status feedback (mirrors the global motion.*
	     * pins but for THIS channel's coord_tp). in-position = at rest with
	     * nothing queued. At num_channels=1, motion.0.* matches the legacy
	     * machine view (D7). */
	    *(emcmot_hal_data->mchan[ch].in_position) =
		(!mv && tpQueueDepth(ctp) == 0) ? 1 : 0;
	    *(emcmot_hal_data->mchan[ch].program_line) = tpGetExecId(ctp);
	    *(emcmot_hal_data->mchan[ch].distance_to_go) = ctp->distance_to_go;
	}
    }

    /* These params can be used to examine any internal variable. */
    /* Change the following lines to assign the variable you want to observe
       to one of the debug parameters.  You can also comment out these lines
       and copy elsewhere if you want to observe an automatic variable that
       isn't in scope here. */
    emcmot_hal_data->debug_bit_0 = joints[1].free_tp.active;
    emcmot_hal_data->debug_bit_1 = emcmotStatus->enables_new & AF_ENABLED;
    emcmot_hal_data->debug_float_0 = emcmotStatus->spindle_status[0].speed;
    emcmot_hal_data->debug_float_1 = emcmotStatus->spindleSync;
    emcmot_hal_data->debug_float_2 = emcmotStatus->vel;
    emcmot_hal_data->debug_float_3 = emcmotStatus->spindle_status[0].net_scale;
    emcmot_hal_data->debug_s32_0 = emcmotStatus->overrideLimitMask;
    emcmot_hal_data->debug_s32_1 = emcmotStatus->tcqlen;

    /* two way handshaking for the spindle encoder */
    for (spindle_num = 0; spindle_num < emcmotConfig->numSpindles; spindle_num++){
		if(emcmotStatus->spindle_status[spindle_num].spindle_index_enable
				&& !old_motion_index[spindle_num]) {
			*emcmot_hal_data->spindle[spindle_num].spindle_index_enable = 1;
			rtapi_print_msg(RTAPI_MSG_DBG, "setting index-enable on spindle %d\n", spindle_num);
		}

		if(!*emcmot_hal_data->spindle[spindle_num].spindle_index_enable
				&& old_hal_index[spindle_num]) {
			emcmotStatus->spindle_status[spindle_num].spindle_index_enable = 0;
		}

		old_motion_index[spindle_num] =
				emcmotStatus->spindle_status[spindle_num].spindle_index_enable;
		old_hal_index[spindle_num] =
				*emcmot_hal_data->spindle[spindle_num].spindle_index_enable;
    }

    *(emcmot_hal_data->tooloffset_x) = emcmotStatus->tool_offset.tran.x;
    *(emcmot_hal_data->tooloffset_y) = emcmotStatus->tool_offset.tran.y;
    *(emcmot_hal_data->tooloffset_z) = emcmotStatus->tool_offset.tran.z;
    *(emcmot_hal_data->tooloffset_a) = emcmotStatus->tool_offset.a;
    *(emcmot_hal_data->tooloffset_b) = emcmotStatus->tool_offset.b;
    *(emcmot_hal_data->tooloffset_c) = emcmotStatus->tool_offset.c;
    *(emcmot_hal_data->tooloffset_u) = emcmotStatus->tool_offset.u;
    *(emcmot_hal_data->tooloffset_v) = emcmotStatus->tool_offset.v;
    *(emcmot_hal_data->tooloffset_w) = emcmotStatus->tool_offset.w;

    /* output joint info to HAL for scoping, etc */
    for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
	/* point to joint struct */
	joint = &joints[joint_num];
	joint_data = &(emcmot_hal_data->joint[joint_num]);

	/* apply backlash and motor offset to output */
	joint->motor_pos_cmd =
	    joint->pos_cmd + joint->backlash_filt + joint->motor_offset;
	/* point to HAL data */
	/* write to HAL pins */
	*(joint_data->motor_offset) = joint->motor_offset;
	*(joint_data->motor_pos_cmd) = joint->motor_pos_cmd;
	*(joint_data->joint_pos_cmd) = joint->pos_cmd;
	*(joint_data->joint_pos_fb) = joint->pos_fb;
	*(joint_data->amp_enable) = GET_JOINT_ENABLE_FLAG(joint);

	*(joint_data->coarse_pos_cmd) = joint->coarse_pos;
	*(joint_data->joint_vel_cmd) = joint->vel_cmd;
	*(joint_data->joint_acc_cmd) = joint->acc_cmd;
    *(joint_data->joint_jerk_cmd) = joint->jerk_cmd;
	*(joint_data->backlash_corr) = joint->backlash_corr;
	*(joint_data->backlash_filt) = joint->backlash_filt;
	*(joint_data->backlash_vel) = joint->backlash_vel;
	*(joint_data->f_error) = joint->ferror;
	*(joint_data->f_error_lim) = joint->ferror_limit;

	*(joint_data->free_pos_cmd) = joint->free_tp.pos_cmd;
	*(joint_data->free_vel_lim) = joint->free_tp.max_vel;
	*(joint_data->free_tp_enable) = joint->free_tp.enable;
	*(joint_data->kb_jjog_active) = joint->kb_jjog_active;
	*(joint_data->wheel_jjog_active) = joint->wheel_jjog_active;

	*(joint_data->active) = GET_JOINT_ACTIVE_FLAG(joint);
	*(joint_data->in_position) = GET_JOINT_INPOS_FLAG(joint);
	*(joint_data->error) = GET_JOINT_ERROR_FLAG(joint);
	*(joint_data->phl) = GET_JOINT_PHL_FLAG(joint);
	*(joint_data->nhl) = GET_JOINT_NHL_FLAG(joint);
	*(joint_data->f_errored) = GET_JOINT_FERROR_FLAG(joint);
	*(joint_data->faulted) = GET_JOINT_FAULT_FLAG(joint);

        // conditionally remove outstanding requests to unlock rotaries:
        if  ( !GET_MOTION_ENABLE_FLAG() && (joint_is_lockable(joint_num))) {
             *(joint_data->unlock) = 0;
        }

	if (IS_EXTRA_JOINT(joint_num) && get_homed(joint_num)) {
	    // passthru posthome_cmd with motor_offset
	    // to hal pin: joint.N.motor-pos-cmd
	    extrajoint_hal_t *ejoint_data;
	    int e = joint_num - NO_OF_KINS_JOINTS;
	    ejoint_data = &(emcmot_hal_data->ejoint[e]);
	    *(joint_data->motor_pos_cmd) = *(ejoint_data->posthome_cmd)
	                                 + joint->motor_offset;
	    continue;
	}
    } // for joint_num

    axis_output_to_hal(pcmd_p);

    *(emcmot_hal_data->jog_is_active) = axis_jog_is_active() || joint_jog_is_active();

}

static void update_status(void)
{
    int joint_num, axis_num, dio, aio, misc_error;
    emcmot_joint_t *joint;
    emcmot_joint_status_t *joint_status;
    emcmot_axis_status_t *axis_status;
#ifdef WATCH_FLAGS
    static int old_joint_flags[8];
    static int old_motion_flag;
#endif

    /* copy status info from private joint structure to status
       struct in shared memory */
    for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
	/* point to joint data */
	joint = &joints[joint_num];
	/* point to joint status */
	joint_status = &(emcmotStatus->joint_status[joint_num]);
	/* copy stuff */
#ifdef WATCH_FLAGS
	/*! \todo FIXME - this is for debugging */
	if ( old_joint_flags[joint_num] != joint->flag ) {
	    rtapi_print ( "Joint %d flag %04X -> %04X\n", joint_num, old_joint_flags[joint_num], joint->flag );
	    old_joint_flags[joint_num] = joint->flag;
	}
#endif
	joint_status->flag = joint->flag;
	if(!(joint_status->homing && !get_homing(joint_num) && get_homing_is_active())) {
		// Prevent race condition.
		// (See also emc/motion/homing.c: base_write_homing_out_pins())
		// The homing status variable turns false before get_homing_is_active()
		// turns false. This means that a new homing command on a joint might
		// fail due to the homing state machine being active while all joints
		// already are in the 'not homing' state.
		// Solution:
		// Do not update the homing status when going from homing --> not homing
		// and the state machine is still active. The homing status deassertion
		// must be delayed until the state machine is done.
		joint_status->homing = get_homing(joint_num);
	}
	joint_status->homed  = get_homed(joint_num);
	joint_status->pos_cmd = joint->pos_cmd;
	joint_status->pos_fb = joint->pos_fb;
	joint_status->vel_cmd = joint->vel_cmd;
	joint_status->acc_cmd = joint->acc_cmd;
	joint_status->ferror = joint->ferror;
	joint_status->ferror_high_mark = joint->ferror_high_mark;
	joint_status->backlash = joint->backlash;
	joint_status->max_pos_limit = joint->max_pos_limit;
	joint_status->min_pos_limit = joint->min_pos_limit;
	joint_status->min_ferror = joint->min_ferror;
	joint_status->max_ferror = joint->max_ferror;
    }
    if (get_allhomed()) {
        *emcmot_hal_data->is_all_homed = 1;
    } else {
        *emcmot_hal_data->is_all_homed = 0;
    }


    for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++) {
        /* point to axis status */
        axis_status = &(emcmotStatus->axis_status[axis_num]);

        axis_status->teleop_vel_cmd = axis_get_teleop_vel_cmd(axis_num);
        axis_status->max_pos_limit = axis_get_max_pos_limit(axis_num);
        axis_status->min_pos_limit = axis_get_min_pos_limit(axis_num);
    }
    emcmotStatus->eoffset_pose.tran.x = axis_get_ext_offset_curr_pos(0);
    emcmotStatus->eoffset_pose.tran.y = axis_get_ext_offset_curr_pos(1);
    emcmotStatus->eoffset_pose.tran.z = axis_get_ext_offset_curr_pos(2);
    emcmotStatus->eoffset_pose.a      = axis_get_ext_offset_curr_pos(3);
    emcmotStatus->eoffset_pose.b      = axis_get_ext_offset_curr_pos(4);
    emcmotStatus->eoffset_pose.c      = axis_get_ext_offset_curr_pos(5);
    emcmotStatus->eoffset_pose.u      = axis_get_ext_offset_curr_pos(6);
    emcmotStatus->eoffset_pose.v      = axis_get_ext_offset_curr_pos(7);
    emcmotStatus->eoffset_pose.w      = axis_get_ext_offset_curr_pos(8);

    emcmotStatus->external_offsets_applied = *(emcmot_hal_data->eoffset_active);

    for (dio = 0; dio < emcmotConfig->numDIO; dio++) {
	emcmotStatus->synch_di[dio] = *(emcmot_hal_data->synch_di[dio]);
	emcmotStatus->synch_do[dio] = *(emcmot_hal_data->synch_do[dio]);
    }

    for (aio = 0; aio < emcmotConfig->numAIO; aio++) {
	emcmotStatus->analog_input[aio] = *(emcmot_hal_data->analog_input[aio]);
	emcmotStatus->analog_output[aio] = *(emcmot_hal_data->analog_output[aio]);
    }

    for (misc_error=0; misc_error < emcmotConfig->numMiscError; misc_error++){
      emcmotStatus->misc_error[misc_error] = *(emcmot_hal_data->misc_error[misc_error]);
    }

    emcmotStatus->jogging_active = *(emcmot_hal_data->jog_is_active);

    /*! \todo FIXME - the rest of this function is stuff that was apparently
       dropped in the initial move from emcmot.c to control.c.  I
       don't know how much is still needed, and how much is baggage.
    */

    /* motion emcmotInternal->chan[0].coord_tp status */
    emcmotStatus->depth = tpQueueDepth(&emcmotInternal->chan[0].coord_tp);
    emcmotStatus->activeDepth = tpActiveDepth(&emcmotInternal->chan[0].coord_tp);
    emcmotStatus->id = tpGetExecId(&emcmotInternal->chan[0].coord_tp);
    //KLUDGE add an API call for this
    emcmotStatus->reverse_run = emcmotInternal->chan[0].coord_tp.reverse_run;
    emcmotStatus->tag = tpGetExecTag(&emcmotInternal->chan[0].coord_tp);
    emcmotStatus->motionType = tpGetMotionType(&emcmotInternal->chan[0].coord_tp);
    emcmotStatus->queueFull = tcqFull(&emcmotInternal->chan[0].coord_tp.queue);

    /* check to see if we should pause in order to implement
       single emcmotStatus->stepping */

    if (emcmotStatus->stepping && emcmotInternal->idForStep != emcmotStatus->id) {
      tpPause(&emcmotInternal->chan[0].coord_tp);
      emcmotStatus->stepping = 0;
      emcmotStatus->paused = 1;
    }
    // State Tags handling
    // Get the current executing trajectory component (the "Source of Truth")
    /* Update the HAL Output Pins from the active tag */
    // Line and Motion Type
    *(emcmot_hal_data->interp_line_number) = (int)emcmotStatus->tag.fields[GM_FIELD_LINE_NUMBER];

    // Performance Metadata
    *(emcmot_hal_data->interp_feedrate) = emcmotStatus->tag.fields_float[GM_FIELD_FLOAT_FEED];

    // Geometric Metadata
    *(emcmot_hal_data->interp_arc_radius) = emcmotStatus->tag.fields_float[GM_FIELD_FLOAT_ARC_RADIUS];
    *(emcmot_hal_data->interp_arc_center_x) = emcmotStatus->tag.fields_float[GM_FIELD_FLOAT_ARC_CENTER_X];
    *(emcmot_hal_data->interp_arc_center_y) = emcmotStatus->tag.fields_float[GM_FIELD_FLOAT_ARC_CENTER_Y];
    *(emcmot_hal_data->interp_arc_center_z) = emcmotStatus->tag.fields_float[GM_FIELD_FLOAT_ARC_CENTER_Z];

    // Get the current motion type from the tag (1=G1, 2=G2, 3=G3)
    int motion_type = (int)emcmotStatus->tag.fields[GM_FIELD_MOTION_MODE];
    if (motion_type == 10 || motion_type == 0) {
        /* --- G1/G0 STATIC HEADING --- */
        // For linear moves, the heading doesn't change during the segment.
        *(emcmot_hal_data->interp_straight_heading) = emcmotStatus->tag.fields_float[GM_FIELD_FLOAT_STRAIGHT_HEADING];
    }
    else if (motion_type == 20 || motion_type == 30) {
        /* --- G2/G3: DYNAMIC ARC HEADING --- */

        // 1. Get Static Center from Tag
        double cx = emcmotStatus->tag.fields_float[GM_FIELD_FLOAT_ARC_CENTER_X];
        double cy = emcmotStatus->tag.fields_float[GM_FIELD_FLOAT_ARC_CENTER_Y];
        double cz = emcmotStatus->tag.fields_float[GM_FIELD_FLOAT_ARC_CENTER_Z];

        // 2. Get Real-Time Feedback Deltas
        double dx = emcmotStatus->carte_pos_fb.tran.x - cx;
        double dy = emcmotStatus->carte_pos_fb.tran.y - cy;
        double dz = emcmotStatus->carte_pos_fb.tran.z - cz;

        // 3. Determine Plane and Radial Angle
        int plane = emcmotStatus->tag.fields[GM_FIELD_PLANE];
        double angle_rad = 0.0; // Initialize to prevent "uninitialized" error

        if (plane == 170) {      // XY: X is Horizontal, Y is Verradiustical
            angle_rad = atan2(dy, dx);
        }
        else if (plane == 180) { // XZ: Z is Horizontal, X is Vertical
            angle_rad = atan2(dx, dz);
        }
        else if (plane == 190) { // YZ: Y is Horizontal, Z is Vertical
            angle_rad = atan2(dz, dy);
        }
        // Optional: add an else here for a default plane if 170/180/190 aren't found

        // 4. Calculate Normal Heading (Tool-to-Center)
        double normal_deg = (angle_rad * (180.0 / M_PI)) + 180.0;
        while (normal_deg < 0) normal_deg += 360.0;
        while (normal_deg >= 360.0) normal_deg -= 360.0;
        *(emcmot_hal_data->interp_normal_heading) = normal_deg;

        // 5. Calculate Tangent Heading (Direction of Travel)
        double tangent_rad = (motion_type == 30) ? (angle_rad + (M_PI / 2.0)) : (angle_rad - (M_PI / 2.0));
        double heading_deg = tangent_rad * (180.0 / M_PI);

        // 6. Final Normalization and Assignment
        while (heading_deg < 0) heading_deg += 360.0;
        while (heading_deg >= 360.0) heading_deg -= 360.0;

        if (emcmot_hal_data->interp_straight_heading) {
        *(emcmot_hal_data->interp_straight_heading) = heading_deg;
        }
    }
#ifdef WATCH_FLAGS
    /*! \todo FIXME - this is for debugging */
    if ( old_motion_flag != emcmotStatus->motionFlag ) {
	rtapi_print ( "Motion flag %04X -> %04X\n", old_motion_flag, emcmotStatus->motionFlag );
	old_motion_flag = emcmotStatus->motionFlag;
    }
#endif
}
