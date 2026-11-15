// Tiny flat-JSON reader and writer for the BoxTurtle line protocol.
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#ifndef __TURTLE_JSON_H
#define __TURTLE_JSON_H

#include <stdint.h>

// Reading. Lines are one flat object, e.g. {"id":3,"cmd":"move","mm":-12.5}.
// Nested values are skipped over, never parsed.
// Each getter returns 1 if the key was found with the right type.
int json_get_str(const char *line, const char *key, char *out, int outlen);
// Numbers are returned scaled by 1000 ("12.5" -> 12500), so the
// firmware never needs floating point.
int json_get_milli(const char *line, const char *key, int32_t *out);
int json_get_int(const char *line, const char *key, int32_t *out);
int json_get_bool(const char *line, const char *key, int *out);

// Writing into a caller buffer. jw_* calls never overflow; if the buffer
// fills, jw.ok goes 0 and the line must not be sent.
struct jw {
    char *buf;
    int len, size, ok, first;
};
void jw_begin(struct jw *w, char *buf, int size);
void jw_str(struct jw *w, const char *key, const char *val);
void jw_int(struct jw *w, const char *key, int32_t val);
void jw_milli(struct jw *w, const char *key, int32_t milli);
void jw_bool(struct jw *w, const char *key, int val);
// Raw value, e.g. an array the caller built: jw_raw(w, "lanes", "[1,0]")
void jw_raw(struct jw *w, const char *key, const char *raw);
// Closes the object and adds "\n". Returns the length or 0 on overflow.
int jw_end(struct jw *w);

#endif // json.h
