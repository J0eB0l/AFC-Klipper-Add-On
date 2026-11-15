// BridgeHub link key: the handshake lines and their checks. See hub.c for
// how a connection goes through them.
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdio.h>
#include <string.h>

#include "mbedtls/md.h"

#include "linkkey.h"

static const char CLAIM[] = "{\"cmd\":\"linkkey\"";

bool linkkey_valid(const char *key) {
    size_t n = strlen(key);
    if (n < 1 || n > LINKKEY_MAX) return false;
    for (size_t i = 0; i < n; i++)
        if (key[i] < 0x21 || key[i] > 0x7e || key[i] == '"' || key[i] == '\\')
            return false;
    return true;
}

bool linkkey_json_str(const char *line, const char *name, char *out, size_t len) {
    const char *p = strstr(line, name);
    if (!p) return false;
    p += strlen(name);
    while (*p == ' ' || *p == ':') p++;
    if (*p++ != '"') return false;
    size_t n = 0;
    while (*p && *p != '"') {
        if (*p == '\\' || n + 1 >= len) return false;
        out[n++] = *p++;
    }
    out[n] = '\0';
    return *p == '"';
}

void linkkey_greeting(bool keyed, const uint8_t *nonce, char *out, size_t len) {
    if (!keyed) {
        snprintf(out, len, "{\"evt\":\"auth\",\"open\":1}\n");
        return;
    }
    char hex[2 * LINKKEY_NONCE_LEN + 1];
    for (int i = 0; i < LINKKEY_NONCE_LEN; i++)
        snprintf(hex + 2 * i, 3, "%02x", nonce[i]);
    snprintf(out, len, "{\"evt\":\"auth\",\"nonce\":\"%s\"}\n", hex);
}

bool linkkey_maybe_claim(const char *buf, size_t n) {
    size_t m = n < sizeof CLAIM - 1 ? n : sizeof CLAIM - 1;
    return strncmp(buf, CLAIM, m) == 0;
}

/**
 * :param c: a character
 * :return int: its value as a hex digit, or -1
 */
static int hexval(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    c |= 0x20;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/**
 * Checks an answer to the challenge in constant time.
 *
 * :param key: the hub's key
 * :param nonce: the challenge
 * :param hex: the 64 hex digits the client sent
 * :return bool: true if they are the first 32 bytes of HMAC-SHA512(key, nonce)
 */
static bool mac_ok(const char *key, const uint8_t *nonce, const char *hex) {
    uint8_t want[64];
    if (strlen(hex) != 2 * LINKKEY_MAC_LEN) return false;
    if (mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA512),
                        (const unsigned char *)key, strlen(key), nonce,
                        LINKKEY_NONCE_LEN, want) != 0)
        return false;
    uint8_t diff = 0;
    for (int i = 0; i < LINKKEY_MAC_LEN; i++) {
        int hi = hexval(hex[2 * i]), lo = hexval(hex[2 * i + 1]);
        if (hi < 0 || lo < 0) return false;
        diff |= (uint8_t)((hi << 4 | lo) ^ want[i]);
    }
    return diff == 0;
}

int linkkey_judge(const char *key, const uint8_t *nonce, const char *line,
                  char *newkey) {
    if (key[0]) {
        char cmd[8], mac[2 * LINKKEY_MAC_LEN + 1];
        if (linkkey_json_str(line, "\"cmd\"", cmd, sizeof cmd)
                && strcmp(cmd, "auth") == 0
                && linkkey_json_str(line, "\"mac\"", mac, sizeof mac)
                && mac_ok(key, nonce, mac))
            return LINKKEY_OK;
        return LINKKEY_BAD;
    }
    if (strncmp(line, CLAIM, sizeof CLAIM - 1) == 0
            && linkkey_json_str(line, "\"key\"", newkey, LINKKEY_MAX + 1)
            && linkkey_valid(newkey))
        return LINKKEY_CLAIM;
    return LINKKEY_BAD;
}
