// Shared helpers for command handlers (proto.c, motion.c).
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#ifndef __TURTLE_PROTO_H
#define __TURTLE_PROTO_H

#include "json.h"

struct reply {
    struct jw w;
    char buf[512];
};
// Starts a reply object, echoing the request's "id".
void reply_begin(struct reply *r, const char *line);
void reply_send(struct reply *r);
void reply_ok(const char *line);
void reply_err(const char *line, const char *err);
// Reads "lane" (1..4) as 0-based; sends an error reply and returns 0 if bad.
int get_lane(const char *line, int *lane);
// Switch id by name ("hub", "prep1", ...), or -1.
int sw_lookup(const char *name);

// Implemented by motion.c: extra commands and status fields.
int proto_handle_extra(const char *cmd, char *line);
void status_extra(struct jw *w);

#endif // proto.h
