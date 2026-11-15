// Self-recovering lane jobs: feed with retries, unload with re-push, and
// the at-extruder buffer check.
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#ifndef __TURTLE_JOBS_H
#define __TURTLE_JOBS_H

int jobs_handle_cmd(const char *cmd, char *line);
void jobs_cfg(const char *line);
int jobs_busy(int lane);
int jobs_owns_motion(int lane);
int jobs_any_active(void);
void jobs_cancel(int lane);
// Ends a lane's job with a failed job_done carrying code.
void jobs_abort(int lane, const char *code);
const char *jobs_name(int lane);

#endif // jobs.h
