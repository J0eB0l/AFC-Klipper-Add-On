// BridgeHub link key: the handshake lines and their checks (linkkey.c).
// Pure functions, so the host test (test/) runs them off the board.
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LINKKEY_MAX       64      // longest key
#define LINKKEY_NONCE_LEN 16      // challenge bytes
#define LINKKEY_MAC_LEN   32      // HMAC-SHA512 bytes the client sends back
#define LINKKEY_LINE_MAX  160     // longest handshake line

enum {
    LINKKEY_BAD = 0,              // wrong answer or a malformed claim
    LINKKEY_OK,                   // the answer proves the key
    LINKKEY_CLAIM,                // an open hub's claim, with a usable key
};

/**
 * A key the host may set: 1 to 64 printable characters, no space, quote or
 * backslash (so it fits a JSON string as is).
 *
 * :param key: the key
 * :return bool: true if it is one
 */
bool linkkey_valid(const char *key);

/**
 * Copies the string value of a field out of a one-line JSON object.
 *
 * :param line: the line
 * :param name: the field name, quotes included (such as "\"mac\"")
 * :param out: where the value goes
 * :param len: size of out
 * :return bool: true if a value that fits was found
 */
bool linkkey_json_str(const char *line, const char *name, char *out, size_t len);

/**
 * Writes the greeting for a new connection: the challenge on a keyed hub,
 * the open notice on an open one. Newline included.
 *
 * :param keyed: the hub has a key
 * :param nonce: LINKKEY_NONCE_LEN random bytes (keyed only)
 * :param out: where the line goes, at least 64 bytes
 * :param len: size of out
 */
void linkkey_greeting(bool keyed, const uint8_t *nonce, char *out, size_t len);

/**
 * Whether the bytes an open hub got first may still turn out to be a claim
 * line, so it should wait for the rest before passing them to the unit.
 *
 * :param buf: the bytes so far
 * :param n: how many
 * :return bool: true while they match the start of a claim
 */
bool linkkey_maybe_claim(const char *buf, size_t n);

/**
 * Judges a new connection's first line (without its newline). An answer may
 * have spaces between its fields; a claim starts exactly {"cmd":"linkkey".
 *
 * :param key: the hub's key, "" on an open hub
 * :param nonce: the challenge sent (keyed only)
 * :param line: the line
 * :param newkey: on LINKKEY_CLAIM, the key to save (LINKKEY_MAX + 1 bytes)
 * :return int: LINKKEY_OK, LINKKEY_CLAIM or LINKKEY_BAD
 */
int linkkey_judge(const char *key, const uint8_t *nonce, const char *line,
                  char *newkey);
