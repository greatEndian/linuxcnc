/********************************************************************
* Description: motion_struct.h
*   A data structure used in only a few places
*
* Author:
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved
********************************************************************/

#ifndef MOTION_STRUCT_H
#define MOTION_STRUCT_H

#include <rtapi_mutex.h>


/* MCHAN: command mailbox of a secondary motion channel (ch >= 1). Each
 * channel's task stack writes commands into its own mailbox and polls its
 * own echo, mirroring the legacy command/echo handshake exactly. Channel 0
 * keeps using the historic command/status fields, so single-channel layout
 * and behavior are untouched. */
    typedef struct emcmot_chan_mailbox_t {
	rtapi_mutex_t mutex;	/* protects `command` (task-side write lock) */
	struct emcmot_command_t command;
	cmd_code_t commandEcho;		/* echo of input command */
	int commandNumEcho;		/* echo of input command number */
	cmd_status_t commandStatus;	/* result of most recent command */
    } emcmot_chan_mailbox_t;

/* big comm structure, for upper memory */
    typedef struct emcmot_struct_t {
        rtapi_mutex_t command_mutex;  // Used to protect access to `command`.
        struct emcmot_command_t command;   /* struct used to pass commands/data from Task to Motion */

	struct emcmot_status_t status;	/* Struct used to store RT status */
	struct emcmot_config_t config;	/* Struct used to store RT config */
	struct emcmot_error_t error;	/* ring buffer for error messages */
	struct emcmot_internal_t internal;	/* Struct used to store RT status and debug
				   data - 2nd largest block */
	/* MCHAN: secondary-channel command mailboxes, appended at the end so
	 * all historic member offsets are unchanged. [0] is unused (channel 0
	 * uses the legacy command/echo fields above). */
	emcmot_chan_mailbox_t mchan_cmd[EMCMOT_MAX_CHANNELS];
	/* MCHAN MC2b: per-channel status snapshots, one full status struct
	 * per secondary channel, filled by motion at the end of every servo
	 * cycle (global snapshot + channel-field overlay) using the same
	 * head/tail split-read protocol as the legacy status. A secondary
	 * stack's usrmotReadEmcmotStatus() reads its channel's block here;
	 * channel 0 keeps the legacy `status` field above. [0] unused. */
	struct emcmot_status_t mchan_status[EMCMOT_MAX_CHANNELS];
    } emcmot_struct_t;


#endif // MOTION_STRUCT_H
