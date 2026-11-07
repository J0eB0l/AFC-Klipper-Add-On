// Host-side tests for src/turtle/json.c: cc -I../src/turtle test_json.c ../src/turtle/json.c
#include <stdio.h>
#include <string.h>
#include "json.h"

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

int main(void)
{
    const char *l = "{\"id\":7,\"cmd\":\"move\",\"lane\":2,\"mm\":-12.5,\"x\":{\"a\":[1,\"}\"]},\"s\":\"a\\\"b\",\"on\":true,\"off\":false,\"f\":0.125}";
    char s[32]; int32_t v; int b;
    CHECK(json_get_str(l, "cmd", s, sizeof s) && !strcmp(s, "move"));
    CHECK(json_get_int(l, "id", &v) && v == 7);
    CHECK(json_get_milli(l, "mm", &v) && v == -12500);
    CHECK(json_get_milli(l, "f", &v) && v == 125);
    CHECK(json_get_int(l, "lane", &v) && v == 2);
    CHECK(json_get_str(l, "s", s, sizeof s) && !strcmp(s, "a\"b"));
    CHECK(json_get_bool(l, "on", &b) && b == 1);
    CHECK(json_get_bool(l, "off", &b) && b == 0);
    CHECK(!json_get_int(l, "a", &v));          // nested keys are not top level
    CHECK(!json_get_int(l, "nope", &v));
    CHECK(!json_get_str(l, "id", s, sizeof s)); // wrong type
    CHECK(!json_get_int("garbage", "id", &v));
    CHECK(!json_get_int("{\"id\":", "id", &v));
    CHECK(json_get_str("{\"cmd\":\"averyveryverylongcommandname\"}", "cmd", s, 8) && !strcmp(s, "averyve"));

    char buf[128]; struct jw w;
    jw_begin(&w, buf, sizeof buf);
    jw_int(&w, "id", -3); jw_str(&w, "m", "q\"\n"); jw_milli(&w, "mm", -12500);
    jw_milli(&w, "z", 7); jw_bool(&w, "ok", 1); jw_raw(&w, "a", "[1]");
    int n = jw_end(&w);
    CHECK(n > 0 && !strcmp(buf, "{\"id\":-3,\"m\":\"q\\\"\\n\",\"mm\":-12.5,\"z\":0.007,\"ok\":true,\"a\":[1]}\n"));
    jw_begin(&w, buf, 10); jw_str(&w, "long", "overflowing"); CHECK(jw_end(&w) == 0);
    printf(fails ? "%d failures\n" : "all json tests passed\n", fails);
    return fails != 0;
}
