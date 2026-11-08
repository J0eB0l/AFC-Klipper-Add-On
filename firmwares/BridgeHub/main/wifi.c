// BridgeHub WiFi.
//
// With a network saved in NVS the hub joins it and keeps rejoining. With
// none, it raises an open setup hotspot, BridgeHub-XXXX, with a page at
// http://192.168.4.1: pick the network, type its password, and the hub tries
// it with the hotspot still up, shows the address it got, saves the network
// and restarts onto it. The page polls for the result, since joining a
// network on another channel moves the hotspot and the phone may drop off it
// for a moment.
//
// Holding BOOT for 10 s forgets the network and the link key (hub.c) and
// restarts into setup.
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs.h"

#include "wifi.h"

#define NVS_NS        "bridgehub"
#define BOOT_GPIO     GPIO_NUM_0
#define BOOT_HOLD_MS  10000
#define JOIN_WAIT_MS  20000
#define JOIN_TRIES    2
#define RESTART_MS    15000     // after a join: time to read the address
#define MAX_NETS      20
#define HUB_PORT      4001      // hub.c's TCP port, AFC's default for a hub

#define EV_GOT_IP     BIT0
#define EV_FAILED     BIT1

enum { JOIN_IDLE, JOIN_BUSY, JOIN_DONE, JOIN_FAILED };

static EventGroupHandle_t s_ev;
static bool s_setup;                    // hotspot up: a failed join is reported
static volatile int s_state = JOIN_IDLE;
static volatile int s_reason;           // the last disconnect's reason
static char s_name[24];                 // BridgeHub-XXXX
static char s_ip[16];                   // our address on the network, or ""
static char s_ssid[33], s_pass[65];     // the network being joined
static char s_nets[MAX_NETS][33];       // networks seen when setup started
static int s_nnets;

/**
 * Follows the station: rejoins a saved network when it drops, and reports
 * the outcome of a join during setup.
 */
static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        if (!s_setup) esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *d = data;
        s_reason = d->reason;
        s_ip[0] = '\0';
        if (s_setup) {
            xEventGroupSetBits(s_ev, EV_FAILED);
        } else {
            printf("{\"evt\":\"wifi\",\"lost\":%d}\n", d->reason);
            esp_wifi_connect();
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *g = data;
        snprintf(s_ip, sizeof s_ip, IPSTR, IP2STR(&g->ip_info.ip));
        printf("{\"evt\":\"wifi\",\"ip\":\"%s\",\"port\":%d}\n", s_ip, HUB_PORT);
        xEventGroupSetBits(s_ev, EV_GOT_IP);
    }
}

/**
 * Reads the saved network.
 *
 * :return bool: true if one is saved
 */
static bool load_net(char *ssid, size_t ssid_len, char *pass, size_t pass_len) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    bool ok = nvs_get_str(h, "ssid", ssid, &ssid_len) == ESP_OK &&
              nvs_get_str(h, "pass", pass, &pass_len) == ESP_OK;
    nvs_close(h);
    return ok && ssid[0] != '\0';
}

static void save_net(const char *ssid, const char *pass) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_str(h, "ssid", ssid);
    nvs_set_str(h, "pass", pass);
    nvs_commit(h);
    nvs_close(h);
}

static void forget_net(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
}

/**
 * Restarts into setup, forgetting the network and the link key (all of the
 * hub's NVS), once BOOT has been held for BOOT_HOLD_MS.
 */
static void boot_task(void *arg) {
    (void)arg;
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << BOOT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io);
    int held = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (gpio_get_level(BOOT_GPIO) != 0) {
            held = 0;
            continue;
        }
        held += 100;
        if (held == 1000)
            printf("{\"evt\":\"boot\",\"msg\":\"keep holding to forget the network and key\"}\n");
        if (held >= BOOT_HOLD_MS) {
            forget_net();
            printf("{\"evt\":\"boot\",\"msg\":\"network and key forgotten, restarting\"}\n");
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_restart();
        }
    }
}

static int hexval(char c) {
    return c <= '9' ? c - '0' : tolower((unsigned char)c) - 'a' + 10;
}

/**
 * Copies one field of a urlencoded form body into out, decoded.
 */
static void form_field(const char *body, const char *key, char *out, size_t len) {
    size_t klen = strlen(key);
    out[0] = '\0';
    for (const char *p = body; p != NULL && *p != '\0'; p = strchr(p, '&'), p = p ? p + 1 : p) {
        if (strncmp(p, key, klen) != 0 || p[klen] != '=') continue;
        p += klen + 1;
        size_t n = 0;
        while (*p != '\0' && *p != '&' && n + 1 < len) {
            char c = *p++;
            if (c == '+') {
                c = ' ';
            } else if (c == '%' && isxdigit((unsigned char)p[0]) &&
                       isxdigit((unsigned char)p[1])) {
                c = (char)(hexval(p[0]) << 4 | hexval(p[1]));
                p += 2;
            }
            out[n++] = c;
        }
        out[n] = '\0';
        return;
    }
}

/**
 * Copies text into out with the HTML specials escaped.
 */
static void html_escape(const char *in, char *out, size_t len) {
    size_t n = 0;
    for (; *in != '\0' && n + 7 < len; in++) {
        const char *r = NULL;
        switch (*in) {
        case '&': r = "&amp;"; break;
        case '<': r = "&lt;"; break;
        case '>': r = "&gt;"; break;
        case '"': r = "&quot;"; break;
        case '\'': r = "&#39;"; break;
        }
        if (r != NULL) {
            size_t k = strlen(r);
            memcpy(out + n, r, k);
            n += k;
        } else {
            out[n++] = *in;
        }
    }
    out[n] = '\0';
}

static const char *why(int reason) {
    switch (reason) {
    case WIFI_REASON_NO_AP_FOUND:
    case WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY:
    case WIFI_REASON_NO_AP_FOUND_IN_RSSI_THRESHOLD:
        return "the network was not found";
    case WIFI_REASON_AUTH_FAIL:
    case WIFI_REASON_AUTH_EXPIRE:
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
        return "the password was not accepted";
    default:
        return "the network did not answer";
    }
}

static void send_head(httpd_req_t *req, int refresh) {
    char buf[96];
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr_chunk(req,
        "<!doctype html><meta charset=utf-8>"
        "<meta name=viewport content='width=device-width,initial-scale=1'>"
        "<title>BridgeHub</title><style>body{font-family:sans-serif;"
        "max-width:28em;margin:2em auto;padding:0 1em}input,button{font-size:1em;"
        "padding:.4em;width:100%;box-sizing:border-box}code{font-size:1.1em}</style>");
    if (refresh > 0) {
        snprintf(buf, sizeof buf, "<meta http-equiv=refresh content='%d;url=/status'>", refresh);
        httpd_resp_sendstr_chunk(req, buf);
    }
    snprintf(buf, sizeof buf, "<h2>%s</h2>", s_name);
    httpd_resp_sendstr_chunk(req, buf);
}

static void send_form(httpd_req_t *req, const char *note) {
    char esc[200];
    send_head(req, 0);
    if (note != NULL) {
        httpd_resp_sendstr_chunk(req, "<p><b>");
        httpd_resp_sendstr_chunk(req, note);
        httpd_resp_sendstr_chunk(req, "</b>");
    }
    httpd_resp_sendstr_chunk(req,
        "<p>Pick the network for the hub to join.<form method=post action=/join>"
        "<p>Network<br><input name=ssid list=nets maxlength=32 required autocomplete=off>"
        "<datalist id=nets>");
    for (int i = 0; i < s_nnets; i++) {
        html_escape(s_nets[i], esc, sizeof esc);
        httpd_resp_sendstr_chunk(req, "<option value=\"");
        httpd_resp_sendstr_chunk(req, esc);
        httpd_resp_sendstr_chunk(req, "\">");
    }
    httpd_resp_sendstr_chunk(req,
        "</datalist><p>Password<br><input name=pass type=password maxlength=63>"
        "<p><button>Join</button></form>");
    httpd_resp_sendstr_chunk(req, NULL);
}

/**
 * Tries the network from the form while the hotspot stays up; on success
 * saves it and restarts onto it.
 */
static void join_task(void *arg) {
    (void)arg;
    wifi_config_t sta = { 0 };
    strlcpy((char *)sta.sta.ssid, s_ssid, sizeof sta.sta.ssid);
    strlcpy((char *)sta.sta.password, s_pass, sizeof sta.sta.password);
    esp_wifi_set_config(WIFI_IF_STA, &sta);
    EventBits_t got = 0;
    for (int i = 0; i < JOIN_TRIES && !(got & EV_GOT_IP); i++) {
        xEventGroupClearBits(s_ev, EV_GOT_IP | EV_FAILED);
        s_reason = 0;
        esp_wifi_connect();
        got = xEventGroupWaitBits(s_ev, EV_GOT_IP | EV_FAILED, pdTRUE, pdFALSE,
                                  pdMS_TO_TICKS(JOIN_WAIT_MS));
    }
    if (got & EV_GOT_IP) {
        save_net(s_ssid, s_pass);
        s_state = JOIN_DONE;
        vTaskDelay(pdMS_TO_TICKS(RESTART_MS));
        esp_restart();
    }
    esp_wifi_disconnect();
    s_state = JOIN_FAILED;
    vTaskDelete(NULL);
}

static esp_err_t page_root(httpd_req_t *req) {
    send_form(req, NULL);
    return ESP_OK;
}

static esp_err_t page_join(httpd_req_t *req) {
    char body[256];
    int len = req->content_len < sizeof body - 1 ? (int)req->content_len : (int)sizeof body - 1;
    int got = 0;
    while (got < len) {
        int n = httpd_req_recv(req, body + got, len - got);
        if (n <= 0) return ESP_FAIL;
        got += n;
    }
    body[got] = '\0';
    if (s_state != JOIN_BUSY && s_state != JOIN_DONE) {
        form_field(body, "ssid", s_ssid, sizeof s_ssid);
        form_field(body, "pass", s_pass, sizeof s_pass);
        if (s_ssid[0] == '\0') {
            send_form(req, "Type or pick a network.");
            return ESP_OK;
        }
        s_state = JOIN_BUSY;
        xTaskCreate(join_task, "join", 4096, NULL, 5, NULL);
    }
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/status");
    httpd_resp_sendstr(req, "");
    return ESP_OK;
}

static esp_err_t page_status(httpd_req_t *req) {
    char esc[200], buf[512];
    html_escape(s_ssid, esc, sizeof esc);
    if (s_state == JOIN_BUSY) {
        send_head(req, 2);
        snprintf(buf, sizeof buf, "<p>Joining <b>%s</b>... This page refreshes on its "
                 "own. If your phone leaves the hotspot, join it again.", esc);
        httpd_resp_sendstr_chunk(req, buf);
        httpd_resp_sendstr_chunk(req, NULL);
    } else if (s_state == JOIN_DONE) {
        send_head(req, 0);
        snprintf(buf, sizeof buf, "<p>Joined <b>%s</b>. The hub's address is "
                 "<b>%s</b>.<p>In AFC:<p><code>serial_port: tcp://%s</code>", esc,
                 s_ip, s_ip);
        httpd_resp_sendstr_chunk(req, buf);
        httpd_resp_sendstr_chunk(req,
            "<p>Reserve this address for the hub in your router, so it stays the "
            "same. The hub restarts onto your network in a few seconds and this "
            "hotspot goes away.");
        httpd_resp_sendstr_chunk(req, NULL);
    } else if (s_state == JOIN_FAILED) {
        snprintf(buf, sizeof buf, "Could not join %s: %s.", esc, why(s_reason));
        send_form(req, buf);
    } else {
        send_form(req, NULL);
    }
    return ESP_OK;
}

static int by_rssi(const void *a, const void *b) {
    return ((const wifi_ap_record_t *)b)->rssi - ((const wifi_ap_record_t *)a)->rssi;
}

/**
 * Lists the networks in range, strongest first, once, for the setup page.
 */
static void scan_nets(void) {
    if (esp_wifi_scan_start(NULL, true) != ESP_OK) return;
    uint16_t n = 0;
    esp_wifi_scan_get_ap_num(&n);
    wifi_ap_record_t *recs = calloc(n ? n : 1, sizeof *recs);
    if (recs == NULL) {
        esp_wifi_clear_ap_list();
        return;
    }
    esp_wifi_scan_get_ap_records(&n, recs);
    qsort(recs, n, sizeof *recs, by_rssi);
    for (int i = 0; i < n && s_nnets < MAX_NETS; i++) {
        const char *ssid = (const char *)recs[i].ssid;
        bool seen = ssid[0] == '\0';
        for (int j = 0; j < s_nnets && !seen; j++) seen = strcmp(s_nets[j], ssid) == 0;
        if (!seen) strlcpy(s_nets[s_nnets++], ssid, sizeof s_nets[0]);
    }
    free(recs);
}

static void start_setup(void) {
    wifi_config_t ap = { 0 };
    strlcpy((char *)ap.ap.ssid, s_name, sizeof ap.ap.ssid);
    ap.ap.ssid_len = strlen(s_name);
    ap.ap.channel = 1;
    ap.ap.authmode = WIFI_AUTH_OPEN;
    ap.ap.max_connection = 4;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
    scan_nets();
    httpd_handle_t server = NULL;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 6144;
    ESP_ERROR_CHECK(httpd_start(&server, &cfg));
    const httpd_uri_t pages[] = {
        { .uri = "/", .method = HTTP_GET, .handler = page_root },
        { .uri = "/join", .method = HTTP_POST, .handler = page_join },
        { .uri = "/status", .method = HTTP_GET, .handler = page_status },
    };
    for (size_t i = 0; i < sizeof pages / sizeof pages[0]; i++)
        httpd_register_uri_handler(server, &pages[i]);
    printf("{\"evt\":\"setup\",\"hotspot\":\"%s\",\"page\":\"http://192.168.4.1\"}\n", s_name);
}

void wifi_start(void) {
    uint8_t mac[6];
    char host[24], pass[65] = "";
    wifi_config_t sta = { 0 };
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_name, sizeof s_name, "BridgeHub-%02X%02X", mac[4], mac[5]);
    snprintf(host, sizeof host, "bridgehub-%02x%02x", mac[4], mac[5]);
    s_ev = xEventGroupCreate();
    s_setup = !load_net((char *)sta.sta.ssid, sizeof sta.sta.ssid, pass, sizeof pass);
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(netif, host);
    if (s_setup) esp_netif_create_default_wifi_ap();
    const wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL));
    xTaskCreate(boot_task, "boot_btn", 2560, NULL, 2, NULL);
    printf("{\"evt\":\"start\",\"name\":\"%s\",\"setup\":%d}\n", s_name, s_setup ? 1 : 0);
    if (s_setup) {
        start_setup();
        return;
    }
    strlcpy((char *)sta.sta.password, pass, sizeof sta.sta.password);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
    ESP_ERROR_CHECK(esp_wifi_start());
    // No modem sleep: it would hold the unit's bytes back by up to a beacon
    // interval.
    esp_wifi_set_ps(WIFI_PS_NONE);
}
