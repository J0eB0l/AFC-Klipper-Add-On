// Host stand-in for ESP-IDF's mbedtls/md.h: just what linkkey.c uses,
// backed by OpenSSL (test only).
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#pragma once

#include <stddef.h>

typedef enum { MBEDTLS_MD_SHA512 = 1 } mbedtls_md_type_t;
typedef struct mbedtls_md_info_t mbedtls_md_info_t;

const mbedtls_md_info_t *mbedtls_md_info_from_type(mbedtls_md_type_t type);
int mbedtls_md_hmac(const mbedtls_md_info_t *info, const unsigned char *key,
                    size_t keylen, const unsigned char *input, size_t ilen,
                    unsigned char *output);
