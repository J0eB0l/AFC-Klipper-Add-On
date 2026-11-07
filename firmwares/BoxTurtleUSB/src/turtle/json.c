// Tiny flat-JSON reader and writer for the BoxTurtle line protocol.
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h>
#include "json.h"

static const char *
skip_ws(const char *p)
{
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')
        p++;
    return p;
}

// p points at '"'. Returns the char after the closing quote, or 0.
static const char *
skip_string(const char *p)
{
    p++;
    while (*p && *p != '"') {
        if (*p == '\\' && p[1])
            p++;
        p++;
    }
    return *p == '"' ? p + 1 : 0;
}

// Skips one value of any type. Returns the char after it, or 0.
static const char *
skip_value(const char *p)
{
    p = skip_ws(p);
    if (*p == '"')
        return skip_string(p);
    if (*p == '{' || *p == '[') {
        int depth = 0;
        while (*p) {
            if (*p == '"') {
                p = skip_string(p);
                if (!p)
                    return 0;
                continue;
            }
            if (*p == '{' || *p == '[')
                depth++;
            else if (*p == '}' || *p == ']') {
                depth--;
                if (!depth)
                    return p + 1;
            }
            p++;
        }
        return 0;
    }
    while (*p && *p != ',' && *p != '}' && *p != ']' && *p != ' ')
        p++;
    return p;
}

// Returns a pointer to the value of a top level key, or 0.
static const char *
find_key(const char *line, const char *key)
{
    const char *p = skip_ws(line);
    if (*p != '{')
        return 0;
    p++;
    int klen = strlen(key);
    for (;;) {
        p = skip_ws(p);
        if (*p != '"')
            return 0;
        const char *kstart = p + 1;
        const char *kend = skip_string(p);
        if (!kend)
            return 0;
        int match = (kend - 1 - kstart == klen
                     && !memcmp(kstart, key, klen));
        p = skip_ws(kend);
        if (*p != ':')
            return 0;
        p = skip_ws(p + 1);
        if (match)
            return p;
        p = skip_value(p);
        if (!p)
            return 0;
        p = skip_ws(p);
        if (*p != ',')
            return 0;
        p++;
    }
}

int
json_get_str(const char *line, const char *key, char *out, int outlen)
{
    const char *p = find_key(line, key);
    if (!p || *p != '"' || outlen <= 0)
        return 0;
    p++;
    int n = 0;
    while (*p && *p != '"') {
        char c = *p++;
        if (c == '\\') {
            c = *p++;
            if (c == 'n')
                c = '\n';
            else if (c == 't')
                c = '\t';
            else if (!c)
                return 0;
        }
        if (n < outlen - 1)
            out[n++] = c;
    }
    out[n] = 0;
    return *p == '"';
}

int
json_get_milli(const char *line, const char *key, int32_t *out)
{
    const char *p = find_key(line, key);
    if (!p)
        return 0;
    int neg = 0;
    if (*p == '-') {
        neg = 1;
        p++;
    }
    if (*p < '0' || *p > '9')
        return 0;
    int64_t whole = 0;
    while (*p >= '0' && *p <= '9') {
        whole = whole * 10 + (*p++ - '0');
        if (whole > 2000000)
            return 0;
    }
    int32_t frac = 0, scale = 100;
    if (*p == '.') {
        p++;
        while (*p >= '0' && *p <= '9') {
            frac += (*p++ - '0') * scale;
            scale /= 10;
        }
    }
    if (*p == 'e' || *p == 'E')
        return 0;
    int32_t v = whole * 1000 + frac;
    *out = neg ? -v : v;
    return 1;
}

int
json_get_int(const char *line, const char *key, int32_t *out)
{
    int32_t m;
    if (!json_get_milli(line, key, &m))
        return 0;
    *out = m / 1000;
    return 1;
}

int
json_get_bool(const char *line, const char *key, int *out)
{
    const char *p = find_key(line, key);
    if (!p)
        return 0;
    if (!strncmp(p, "true", 4)) {
        *out = 1;
        return 1;
    }
    if (!strncmp(p, "false", 5)) {
        *out = 0;
        return 1;
    }
    if (*p == '0' || *p == '1') {
        *out = *p == '1';
        return 1;
    }
    return 0;
}

/****************************************************************
 * Writer
 ****************************************************************/

static void
put(struct jw *w, const char *s, int n)
{
    if (!w->ok)
        return;
    if (w->len + n >= w->size) {
        w->ok = 0;
        return;
    }
    memcpy(&w->buf[w->len], s, n);
    w->len += n;
    w->buf[w->len] = 0;
}

static void
puts_(struct jw *w, const char *s)
{
    put(w, s, strlen(s));
}

static void
key(struct jw *w, const char *k)
{
    if (!w->first)
        put(w, ",", 1);
    w->first = 0;
    put(w, "\"", 1);
    puts_(w, k);
    put(w, "\":", 2);
}

void
jw_begin(struct jw *w, char *buf, int size)
{
    w->buf = buf;
    w->size = size;
    w->len = 0;
    w->ok = 1;
    w->first = 1;
    put(w, "{", 1);
}

void
jw_str(struct jw *w, const char *k, const char *val)
{
    key(w, k);
    put(w, "\"", 1);
    for (; *val; val++) {
        char c = *val;
        if (c == '"' || c == '\\') {
            put(w, "\\", 1);
            put(w, &c, 1);
        } else if (c == '\n') {
            put(w, "\\n", 2);
        } else if ((unsigned char)c >= 0x20) {
            put(w, &c, 1);
        }
    }
    put(w, "\"", 1);
}

static void
put_uint(struct jw *w, uint32_t v)
{
    char tmp[12];
    int n = 0;
    do {
        tmp[n++] = '0' + v % 10;
        v /= 10;
    } while (v);
    while (n)
        put(w, &tmp[--n], 1);
}

void
jw_int(struct jw *w, const char *k, int32_t val)
{
    key(w, k);
    if (val < 0) {
        put(w, "-", 1);
        put_uint(w, -(uint32_t)val);
    } else {
        put_uint(w, val);
    }
}

void
jw_milli(struct jw *w, const char *k, int32_t milli)
{
    key(w, k);
    uint32_t a = milli;
    if (milli < 0) {
        put(w, "-", 1);
        a = -(uint32_t)milli;
    }
    put_uint(w, a / 1000);
    uint32_t f = a % 1000;
    if (f) {
        char d[4] = {'.', '0' + f / 100, '0' + f / 10 % 10, '0' + f % 10};
        int n = 4;
        while (d[n - 1] == '0')
            n--;
        put(w, d, n);
    }
}

void
jw_bool(struct jw *w, const char *k, int val)
{
    key(w, k);
    puts_(w, val ? "true" : "false");
}

void
jw_raw(struct jw *w, const char *k, const char *raw)
{
    key(w, k);
    puts_(w, raw);
}

int
jw_end(struct jw *w)
{
    put(w, "}\n", 2);
    return w->ok ? w->len : 0;
}
