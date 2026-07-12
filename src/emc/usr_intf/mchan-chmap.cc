/* MCHAN phase-2: send a channel's axis-letter -> global-joint map to motion.
 * Reads [EMCMOT]MOTION_CHANNEL (via usrmotIniLoad) and [CHANNEL]MAP from the
 * channel's INI and sends EMCMOT_SET_CHANNEL_AXIS_MAP per pair through the
 * channel's own mailbox. Run by the launcher BEFORE the channel's task
 * stack starts (the map also claims joint ownership, MC6/D6).
 *
 *   [CHANNEL]
 *   MAP = X:3 Y:4 Z:5
 *
 * usage: mchan-chmap <chN.ini>   (rc 0 = all pairs accepted)
 *        mchan-chmap --machine <master.ini>   (machine-global settings only)
 *
 * Built by src/emc/usr_intf/Submakefile into ../bin/mchan-chmap, alongside
 * the other userspace tools - no separate manual build step. The launcher
 * (scripts/linuxcnc[.in]) finds it on $PATH via [MCHAN]CHMAP, defaulting to
 * the bare name "mchan-chmap" when the INI doesn't override it.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include "motion.h"
#include "usrmotintf.h"
#include "emc/ini/inifile.hh"
using linuxcnc::IniFile;

static const char axis_letters[] = "XYZABCUVW";

/* MC18/launcher robustness: at session start the FIRST axis-map command can
 * race the servo thread coming up and time out (even within the 1 s
 * EMCMOT_COMM_TIMEOUT), which made mchan-chmap reject one pair and the
 * launcher abort the whole stack - the intermittent "command timeout
 * (seq: 1)" boot failure. The map is idempotent, so retry a timed-out /
 * comm-error command for a few seconds before giving up. A real rejection
 * (EMCMOT_COMM_ERROR_COMMAND - e.g. ownership conflict) is NOT retried. */
static int write_cmd_retry(emcmot_command_t *cmd)
{
    const int max_tries = 60;          /* ~6 s worst case */
    int r = 0;
    for (int t = 0; t < max_tries; t++) {
        r = usrmotWriteEmcmotCommand(cmd);
        if (r == EMCMOT_COMM_OK) return r;
        if (r == EMCMOT_COMM_ERROR_COMMAND) return r;  /* genuine refusal */
        usleep(100000);                /* 100 ms, then retry the transient */
    }
    return r;
}

int main(int argc, char **argv)
{
    /* --machine <master.ini>: send ONLY machine-global settings (the
     * interference zone) - no per-channel map/ownership. The zone lives in
     * the master INI, which the per-channel invocations are never given, so
     * without this the collision guard is never armed at boot. */
    int machine_mode = 0;
    const char *ini_path = NULL;
    if (argc >= 3 && strcmp(argv[1], "--machine") == 0) {
        machine_mode = 1;
        ini_path = argv[2];
    } else if (argc >= 2) {
        ini_path = argv[1];
    } else {
        fprintf(stderr, "usage: %s <chN.ini>   |   %s --machine <master.ini>\n",
                argv[0], argv[0]);
        return 2;
    }

    IniFile inifile(ini_path);
    if (!inifile) {
        fprintf(stderr, "mchan-chmap: cannot open INI %s\n", ini_path);
        return 2;
    }
    char buf[256];
    buf[0] = 0;
    if (!machine_mode) {
        auto map = inifile.findString("MAP", "CHANNEL");
        if (!map) {
            fprintf(stderr, "mchan-chmap: no [CHANNEL]MAP in %s - nothing to do\n", ini_path);
            return 0;
        }
        snprintf(buf, sizeof(buf), "%s", map->c_str());
    }

    if (usrmotIniLoad(ini_path) != 0) { fprintf(stderr, "mchan-chmap: usrmotIniLoad failed\n"); return 2; }
    if (usrmotInit((char*)"chmap") != 0) { fprintf(stderr, "mchan-chmap: usrmotInit failed (is motion running?)\n"); return 2; }

    int rc = 0, npairs = 0;
    for (char *tok = machine_mode ? NULL : strtok(buf, " \t,");
         tok; tok = strtok(NULL, " \t,")) {
        char letter = toupper(tok[0]);
        const char *p = strchr(axis_letters, letter);
        if (!p || tok[1] != ':') {
            fprintf(stderr, "mchan-chmap: bad MAP token '%s' (want LETTER:JOINT)\n", tok);
            rc = 1; continue;
        }
        int axis = (int)(p - axis_letters);
        int joint = atoi(tok + 2);

        emcmot_command_t cmd;
        memset(&cmd, 0, sizeof(cmd));
        cmd.command = EMCMOT_SET_CHANNEL_AXIS_MAP;
        cmd.axis = axis;
        cmd.joint = joint;
        int r = write_cmd_retry(&cmd);
        printf("mchan-chmap: %c -> joint %d : %s\n", letter, joint,
               r == 0 ? "accepted" : "REJECTED");
        if (r != 0) rc = 1;
        npairs++;
    }
    if (!machine_mode && !npairs) rc = 1;

    // MC26b: claim this channel's spindle (ownership) from [CHANNEL]SPINDLE.
    // Absent = this channel claims no spindle (channel 0 owns it by default).
    if (!machine_mode)
    if (auto sp = inifile.findInt("SPINDLE", "CHANNEL")) {
        emcmot_command_t cmd;
        memset(&cmd, 0, sizeof(cmd));
        cmd.command = EMCMOT_SET_CHANNEL_SPINDLE;
        cmd.spindle = *sp;
        int r = write_cmd_retry(&cmd);
        printf("mchan-chmap: own spindle %d : %s\n", *sp, r == 0 ? "accepted" : "REJECTED");
        if (r != 0) rc = 1;
    }

    // MC31: this channel's WORLD frame from [CHANNEL]ORIGIN/ORIENT (same values
    // the preview uses), so the interference guard transforms its controlled
    // point to world coords. Sent whenever ORIGIN or ORIENT is present.
    if (!machine_mode)
    {
        auto orig = inifile.findString("ORIGIN", "CHANNEL");
        auto ornt = inifile.findString("ORIENT", "CHANNEL");
        if (orig || ornt) {
            emcmot_command_t cmd;
            memset(&cmd, 0, sizeof(cmd));
            cmd.command = EMCMOT_SET_CHANNEL_FRAME;
            if (orig) sscanf(orig->c_str(), "%lf %lf %lf",
                             &cmd.frame_origin[0], &cmd.frame_origin[1], &cmd.frame_origin[2]);
            if (ornt) sscanf(ornt->c_str(), "%lf %lf %lf",
                             &cmd.frame_orient[0], &cmd.frame_orient[1], &cmd.frame_orient[2]);
            int r = write_cmd_retry(&cmd);
            printf("mchan-chmap: world frame O=(%.1f,%.1f,%.1f) R=(%.1f,%.1f,%.1f) : %s\n",
                   cmd.frame_origin[0], cmd.frame_origin[1], cmd.frame_origin[2],
                   cmd.frame_orient[0], cmd.frame_orient[1], cmd.frame_orient[2],
                   r == 0 ? "accepted" : "REJECTED");
            if (r != 0) rc = 1;
        }
    }

    // MC27: this channel's digital/analog I/O index window from
    // [CHANNEL]DIO_RANGE / AIO_RANGE = "base count" (Fanuq per-path PMC area).
    // Writes (M62-M65 / M67-M68) outside the window are refused by motion so
    // one head's I/O cannot clobber another's. Omit (or count 0) = unrestricted.
    if (!machine_mode)
    {
        auto dr = inifile.findString("DIO_RANGE", "CHANNEL");
        auto ar = inifile.findString("AIO_RANGE", "CHANNEL");
        if (dr || ar) {
            emcmot_command_t cmd;
            memset(&cmd, 0, sizeof(cmd));
            cmd.command = EMCMOT_SET_CHANNEL_IO_RANGE;
            if (dr) sscanf(dr->c_str(), "%d %d", &cmd.io_dio_base, &cmd.io_dio_count);
            if (ar) sscanf(ar->c_str(), "%d %d", &cmd.io_aio_base, &cmd.io_aio_count);
            int r = write_cmd_retry(&cmd);
            printf("mchan-chmap: I/O range dio[%d,+%d) aio[%d,+%d) : %s\n",
                   cmd.io_dio_base, cmd.io_dio_count, cmd.io_aio_base, cmd.io_aio_count,
                   r == 0 ? "accepted" : "REJECTED");
            if (r != 0) rc = 1;
        }
    }

    // MC31: world keep-out / interference zone from [MCHAN]INTERFERE_ZONE
    // (master INI only): "xmin xmax ymin ymax zmin zmax" (mm). Two+ channels
    // co-occupying it -> protective stop (enforcement = later increment).
    if (auto zs = inifile.findString("INTERFERE_ZONE", "MCHAN")) {
        emcmot_command_t cmd;
        memset(&cmd, 0, sizeof(cmd));
        cmd.command = EMCMOT_SET_INTERFERE_ZONE;
        int n = sscanf(zs->c_str(), "%lf %lf %lf %lf %lf %lf",
                       &cmd.zone[0], &cmd.zone[1], &cmd.zone[2],
                       &cmd.zone[3], &cmd.zone[4], &cmd.zone[5]);
        if (n == 6) {
            int r = write_cmd_retry(&cmd);
            printf("mchan-chmap: interfere zone x[%.0f,%.0f] y[%.0f,%.0f] z[%.0f,%.0f] : %s\n",
                   cmd.zone[0], cmd.zone[1], cmd.zone[2], cmd.zone[3], cmd.zone[4], cmd.zone[5],
                   r == 0 ? "accepted" : "REJECTED");
            if (r != 0) rc = 1;
        } else {
            fprintf(stderr, "mchan-chmap: [MCHAN]INTERFERE_ZONE needs 6 numbers "
                            "(xmin xmax ymin ymax zmin zmax), got %d\n", n);
            rc = 1;
        }
    }
    return rc;
}
