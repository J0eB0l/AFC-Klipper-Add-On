// Host test for the BridgeHub link key handshake (main/linkkey.c):
// make && ./test_linkkey
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdio.h>
#include <string.h>

#include <openssl/evp.h>
#include <openssl/hmac.h>

#include "mbedtls/md.h"
#include "linkkey.h"

// The mbedtls calls linkkey.c makes, done with OpenSSL.
const mbedtls_md_info_t *mbedtls_md_info_from_type(mbedtls_md_type_t type) {
    return type == MBEDTLS_MD_SHA512 ? (const mbedtls_md_info_t *)EVP_sha512() : NULL;
}

int mbedtls_md_hmac(const mbedtls_md_info_t *info, const unsigned char *key,
                    size_t keylen, const unsigned char *input, size_t ilen,
                    unsigned char *output) {
    unsigned int len = 0;
    return HMAC((const EVP_MD *)info, key, (int)keylen, input, ilen, output, &len)
           ? 0 : -1;
}

static int fails, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("  FAIL: "); \
    printf(__VA_ARGS__); printf("\n"); } } while (0)

static const char KEY[] = "0123456789abcdef0123456789abcdef";
// First 32 bytes of HMAC-SHA512(KEY, bytes 0..15), from Python's hmac.
static const char MAC[] =
    "84aa390f3f68d40d3b8f2b5ee2ad3517c13e31649284a43e867a8fa5974500bc";

static void t_valid(void) {
    char longest[LINKKEY_MAX + 2];
    memset(longest, 'k', sizeof longest);
    longest[LINKKEY_MAX] = '\0';
    CHECK(linkkey_valid(KEY), "hex key");
    CHECK(linkkey_valid(longest), "64 characters");
    longest[LINKKEY_MAX] = 'k';
    longest[LINKKEY_MAX + 1] = '\0';
    CHECK(!linkkey_valid(longest), "65 characters");
    CHECK(!linkkey_valid(""), "empty");
    CHECK(!linkkey_valid("a b"), "space");
    CHECK(!linkkey_valid("a\"b"), "quote");
    CHECK(!linkkey_valid("a\\b"), "backslash");
    CHECK(!linkkey_valid("a\x7f"), "control character");
}

static void t_json(void) {
    char out[8];
    CHECK(linkkey_json_str("{\"cmd\":\"auth\",\"mac\":\"ab12\"}", "\"mac\"", out,
                           sizeof out) && !strcmp(out, "ab12"), "plain value");
    CHECK(linkkey_json_str("{\"mac\" : \"x\"}", "\"mac\"", out, sizeof out)
          && !strcmp(out, "x"), "spaces around the colon");
    CHECK(!linkkey_json_str("{\"mac\":\"a\\\"b\"}", "\"mac\"", out, sizeof out),
          "escapes refused");
    CHECK(!linkkey_json_str("{\"mac\":\"12345678\"}", "\"mac\"", out, sizeof out),
          "too long for out");
    CHECK(!linkkey_json_str("{\"mac\":1}", "\"mac\"", out, sizeof out), "not a string");
    CHECK(!linkkey_json_str("{\"mac\":\"ab", "\"mac\"", out, sizeof out), "unterminated");
    CHECK(!linkkey_json_str("{\"key\":\"ab\"}", "\"mac\"", out, sizeof out), "missing");
}

static void t_greeting(void) {
    uint8_t nonce[LINKKEY_NONCE_LEN];
    char line[80];
    for (int i = 0; i < LINKKEY_NONCE_LEN; i++) nonce[i] = (uint8_t)i;
    linkkey_greeting(false, nonce, line, sizeof line);
    CHECK(!strcmp(line, "{\"evt\":\"auth\",\"open\":1}\n"), "open: %s", line);
    linkkey_greeting(true, nonce, line, sizeof line);
    CHECK(!strcmp(line, "{\"evt\":\"auth\",\"nonce\":"
                        "\"000102030405060708090a0b0c0d0e0f\"}\n"), "keyed: %s", line);
}

static void t_maybe_claim(void) {
    CHECK(linkkey_maybe_claim("", 0), "nothing yet");
    CHECK(linkkey_maybe_claim("{\"cmd\"", 6), "start of a claim");
    CHECK(linkkey_maybe_claim("{\"cmd\":\"linkkey\",\"key\":\"k\"}", 27), "whole claim");
    CHECK(!linkkey_maybe_claim("{\"cmd\":\"info\"}", 14), "a unit command");
    CHECK(!linkkey_maybe_claim("{\"id\":1,\"cmd\":\"linkkey\"}", 24), "claim not first");
    CHECK(!linkkey_maybe_claim("\xff\xaa", 2), "ACE frame");
}

static void t_judge(void) {
    uint8_t nonce[LINKKEY_NONCE_LEN];
    char line[LINKKEY_LINE_MAX], newkey[LINKKEY_MAX + 1], upper[sizeof MAC];
    for (int i = 0; i < LINKKEY_NONCE_LEN; i++) nonce[i] = (uint8_t)i;
    snprintf(line, sizeof line, "{\"cmd\":\"auth\",\"mac\":\"%s\"}", MAC);
    CHECK(linkkey_judge(KEY, nonce, line, newkey) == LINKKEY_OK, "right answer");
    snprintf(line, sizeof line, "{\"cmd\": \"auth\", \"mac\": \"%s\"}", MAC);
    CHECK(linkkey_judge(KEY, nonce, line, newkey) == LINKKEY_OK, "answer with spaces");
    for (size_t i = 0; i < sizeof MAC; i++)
        upper[i] = (MAC[i] >= 'a' && MAC[i] <= 'f') ? MAC[i] - 32 : MAC[i];
    snprintf(line, sizeof line, "{\"cmd\":\"auth\",\"mac\":\"%s\"}", upper);
    CHECK(linkkey_judge(KEY, nonce, line, newkey) == LINKKEY_OK, "upper case hex");
    nonce[0] = 1;
    snprintf(line, sizeof line, "{\"cmd\":\"auth\",\"mac\":\"%s\"}", MAC);
    CHECK(linkkey_judge(KEY, nonce, line, newkey) == LINKKEY_BAD, "other nonce");
    nonce[0] = 0;
    CHECK(linkkey_judge("another key", nonce, line, newkey) == LINKKEY_BAD, "other key");
    snprintf(line, sizeof line, "{\"cmd\":\"auth\",\"mac\":\"%.62s\"}", MAC);
    CHECK(linkkey_judge(KEY, nonce, line, newkey) == LINKKEY_BAD, "short answer");
    snprintf(line, sizeof line, "{\"cmd\":\"auth\",\"mac\":\"%.62szz\"}", MAC);
    CHECK(linkkey_judge(KEY, nonce, line, newkey) == LINKKEY_BAD, "not hex");
    snprintf(line, sizeof line, "{\"cmd\":\"info\",\"mac\":\"%s\"}", MAC);
    CHECK(linkkey_judge(KEY, nonce, line, newkey) == LINKKEY_BAD, "not an auth line");
    snprintf(line, sizeof line, "{\"cmd\":\"authx\",\"mac\":\"%s\"}", MAC);
    CHECK(linkkey_judge(KEY, nonce, line, newkey) == LINKKEY_BAD, "longer cmd");
    snprintf(line, sizeof line, "{\"mac\":\"%s\"}", MAC);
    CHECK(linkkey_judge(KEY, nonce, line, newkey) == LINKKEY_BAD, "no cmd");
    CHECK(linkkey_judge(KEY, nonce, "{\"cmd\":\"linkkey\",\"key\":\"new\"}", newkey)
          == LINKKEY_BAD, "a keyed hub takes no claim");
    // Open hub
    CHECK(linkkey_judge("", nonce, "{\"cmd\":\"linkkey\",\"key\":\"abc123\"}", newkey)
          == LINKKEY_CLAIM && !strcmp(newkey, "abc123"), "claim");
    CHECK(linkkey_judge("", nonce, "{\"cmd\":\"linkkey\",\"key\":\"a b\"}", newkey)
          == LINKKEY_BAD, "claim with a bad key");
    CHECK(linkkey_judge("", nonce, "{\"cmd\":\"linkkey\"}", newkey) == LINKKEY_BAD,
          "claim without a key");
    snprintf(line, sizeof line, "{\"cmd\":\"auth\",\"mac\":\"%s\"}", MAC);
    CHECK(linkkey_judge("", nonce, line, newkey) == LINKKEY_BAD, "answer on an open hub");
}

int main(void) {
    t_valid();
    t_json();
    t_greeting();
    t_maybe_claim();
    t_judge();
    printf("%d/%d checks passed\n", checks - fails, checks);
    return fails != 0;
}
