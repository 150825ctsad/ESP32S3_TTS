/******************************************************************************
 * wifi_cfg.c  -- Full-featured Wi-Fi provisioning (xiaozhi-web compatible)
 *
 * Endpoints:
 *   GET  /                  → softap.html
 *   GET  /scan              → {"support_5g":false,"aps":[{ssid,rssi,authmode}]}
 *   POST /submit            → {"ssid","password"} → {"success":bool,"error":""}
 *   GET  /saved/list        → ["ssid1","ssid2",…]
 *   GET  /saved/delete?index=N
 *   GET  /saved/set_default?index=N
 *   GET  /done.html         → success page
 *   GET  /advanced/config   → advanced settings JSON
 *   POST /advanced/submit   → save advanced config
 *   404                     → 302 redirect to /
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_mac.h"
#include "nvs_flash.h"

#include "lwip/sockets.h"

#include "cJSON.h"
#include "wifi_cfg.h"
#include "mqtt_cfg.h"

/* ================================================================ */
/*  Embedded files (CMake EMBED_TXTFILES)                            */
/* ================================================================ */
extern const char softap_html_start[] asm("_binary_softap_html_start");
extern const char softap_html_end[]   asm("_binary_softap_html_end");
extern const char done_html_start[]   asm("_binary_done_html_start");
extern const char done_html_end[]     asm("_binary_done_html_end");

/* ================================================================ */
/*  Constants                                                        */
/* ================================================================ */
#define TAG                     "WIFI_CFG"

/* ---- NVS ---- */
#define NVS_NS                  "wifi_cfg"
#define NVS_KEY_COUNT           "saved_cnt"
#define NVS_KEY_SSID_FMT        "ssid_%d"
#define NVS_KEY_PASS_FMT        "pass_%d"
#define NVS_KEY_BSSID_FMT       "bssid_%d"
#define NVS_KEY_MAX_TX_PWR      "max_tx_pwr"
#define NVS_KEY_REM_BSSID       "rem_bssid"
#define MAX_SAVED               10

/* ---- SoftAP ---- */
#define AP_SSID_PREFIX          "ESP"
#define AP_PASSWORD             ""
#define AP_MAX_CONN             4
#define AP_IP                   "192.168.4.1"

/* ---- STA connect ---- */
#define STA_TIMEOUT_MS          15000

/* ---- DNS ---- */
#define DNS_PORT                53
#define DNS_BUF_LEN             512

/* ================================================================ */
/*  Globals                                                          */
/* ================================================================ */
static SemaphoreHandle_t s_ip_sem = NULL;
static httpd_handle_t    s_httpd  = NULL;
static volatile bool     s_connecting = false;  /* guard against duplicate /submit */

/* ================================================================ */
/*  tiny helpers                                                     */
/* ================================================================ */

static bool memiszero(const uint8_t *p, size_t n) {
    for (size_t i = 0; i < n; i++) if (p[i]) return false;
    return true;
}

/* parse ?index=N from a query string; returns -1 on failure */
static int parse_index(const char *uri)
{
    const char *q = strchr(uri, '?');
    if (!q) return -1;
    q++; /* skip '?' */
    if (strncmp(q, "index=", 6) != 0) return -1;
    return atoi(q + 6);
}

/* ---- send a tiny JSON response ---- */
static void send_json(httpd_req_t *req, const char *json)
{
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_send(req, json, strlen(json));
}

/* ---- send HTML with Connection: close ---- */
static void send_html(httpd_req_t *req, const char *start, const char *end)
{
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, start, end - start);
}

/* ================================================================ */
/*  NVS — saved Wi‑Fi networks (multi)                              */
/* ================================================================ */

static uint8_t nvs_saved_count(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return 0;
    uint8_t v = 0;
    nvs_get_u8(h, NVS_KEY_COUNT, &v);
    nvs_close(h);
    return v > MAX_SAVED ? MAX_SAVED : v;
}

static esp_err_t nvs_saved_count_set(uint8_t n)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    e = nvs_set_u8(h, NVS_KEY_COUNT, n > MAX_SAVED ? MAX_SAVED : n);
    e |= nvs_commit(h);
    nvs_close(h);
    return e;
}

/* read i-th saved SSID & password; index 0 is the "default" (tried first) */
static bool nvs_saved_get(int i, char *ssid, size_t ssid_sz,
                          char *pass, size_t pass_sz, uint8_t *bssid)
{
    char key[16];
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    snprintf(key, sizeof(key), NVS_KEY_SSID_FMT, i);
    size_t len = ssid_sz;
    esp_err_t e = nvs_get_str(h, key, ssid, &len);
    snprintf(key, sizeof(key), NVS_KEY_PASS_FMT, i);
    len = pass_sz;
    e |= nvs_get_str(h, key, pass, &len);
    if (bssid) {
        snprintf(key, sizeof(key), NVS_KEY_BSSID_FMT, i);
        len = 6;
        if (nvs_get_blob(h, key, bssid, &len) != ESP_OK) memset(bssid, 0, 6);
    }
    nvs_close(h);
    return (e == ESP_OK);
}

/* push a new SSID/password to the front (index 0), evicting last if full */
static esp_err_t nvs_saved_push_front(const char *ssid, const char *pass,
                                      const uint8_t *bssid)
{
    uint8_t n = nvs_saved_count();
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;

    /* shift existing entries backwards (drop last if at capacity) */
    int start = (n >= MAX_SAVED) ? MAX_SAVED - 1 : n;
    for (int i = start; i > 0; i--) {
        char k1[16], k2[16], k3[16];
        snprintf(k1, sizeof(k1), NVS_KEY_SSID_FMT, i);
        snprintf(k2, sizeof(k2), NVS_KEY_PASS_FMT, i);
        snprintf(k3, sizeof(k3), NVS_KEY_BSSID_FMT, i);
        size_t len = 64; char buf[64];
        if (nvs_get_str(h, k1, buf, &len) == ESP_OK) nvs_set_str(h, k1, buf);
        len = 64;
        if (nvs_get_str(h, k2, buf, &len) == ESP_OK) nvs_set_str(h, k2, buf);
        len = 6;
        uint8_t b[6];
        if (nvs_get_blob(h, k3, b, &len) == ESP_OK) nvs_set_blob(h, k3, b, 6);
    }
    /* write new entry at index 0 */
    char key[16];
    snprintf(key, sizeof(key), NVS_KEY_SSID_FMT, 0);
    nvs_set_str(h, key, ssid);
    snprintf(key, sizeof(key), NVS_KEY_PASS_FMT, 0);
    nvs_set_str(h, key, pass);
    if (bssid) {
        snprintf(key, sizeof(key), NVS_KEY_BSSID_FMT, 0);
        nvs_set_blob(h, key, bssid, 6);
    }
    if (n < MAX_SAVED) n++;
    nvs_set_u8(h, NVS_KEY_COUNT, n);
    e = nvs_commit(h);
    nvs_close(h);
    return e;
}

/* delete i-th entry; collapse later entries */
static esp_err_t nvs_saved_delete(int idx)
{
    uint8_t n = nvs_saved_count();
    if (idx < 0 || idx >= n) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;

    for (int i = idx; i < n - 1; i++) {
        char k1[16], k2[16], k3[16];
        snprintf(k1, sizeof(k1), NVS_KEY_SSID_FMT, i + 1);
        snprintf(k2, sizeof(k2), NVS_KEY_PASS_FMT, i + 1);
        snprintf(k3, sizeof(k3), NVS_KEY_BSSID_FMT, i + 1);
        size_t l = 64; char b[64];
        if (nvs_get_str(h, k1, b, &l) == ESP_OK) { snprintf(k1, sizeof(k1), NVS_KEY_SSID_FMT, i); nvs_set_str(h, k1, b); }
        l = 64;
        if (nvs_get_str(h, k2, b, &l) == ESP_OK) { snprintf(k2, sizeof(k2), NVS_KEY_PASS_FMT, i); nvs_set_str(h, k2, b); }
        l = 6; uint8_t mb[6];
        if (nvs_get_blob(h, k3, mb, &l) == ESP_OK) { snprintf(k3, sizeof(k3), NVS_KEY_BSSID_FMT, i); nvs_set_blob(h, k3, mb, 6); }
    }
    n--;
    nvs_set_u8(h, NVS_KEY_COUNT, n);
    e = nvs_commit(h);
    nvs_close(h);
    return e;
}

/* move idx to front */
static esp_err_t nvs_saved_set_default(int idx)
{
    char ssid[64], pass[64]; uint8_t bssid[6] = {0};
    if (!nvs_saved_get(idx, ssid, sizeof(ssid), pass, sizeof(pass), bssid))
        return ESP_ERR_NOT_FOUND;
    nvs_saved_delete(idx);
    return nvs_saved_push_front(ssid, pass, bssid);
}

/* erase all saved */
static void nvs_saved_clear(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
}

/* ================================================================ */
/*  NVS — advanced config                                           */
/* ================================================================ */

static int8_t nvs_adv_get_i8(const char *key, int8_t def)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return def;
    int8_t v = def;
    nvs_get_i8(h, key, &v);
    nvs_close(h);
    return v;
}

static uint8_t nvs_adv_get_u8(const char *key, uint8_t def)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return def;
    uint8_t v = def;
    nvs_get_u8(h, key, &v);
    nvs_close(h);
    return v;
}

static void nvs_adv_set_i8(const char *key, int8_t v)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i8(h, key, v);
    nvs_commit(h);
    nvs_close(h);
}

static void nvs_adv_set_u8(const char *key, uint8_t v)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, key, v);
    nvs_commit(h);
    nvs_close(h);
}

/* ================================================================ */
/*  DNS captive portal                                              */
/* ================================================================ */
static void dns_task(void *arg)
{
    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s < 0) { vTaskDelete(NULL); return; }
    struct sockaddr_in a = {.sin_family=AF_INET,.sin_port=htons(DNS_PORT),.sin_addr.s_addr=htonl(INADDR_ANY)};
    bind(s, (struct sockaddr*)&a, sizeof(a));
    char rx[DNS_BUF_LEN], tx[DNS_BUF_LEN];
    while (1) {
        struct sockaddr_in cl; socklen_t cll = sizeof(cl);
        int rl = recvfrom(s, rx, sizeof(rx), 0, (struct sockaddr*)&cl, &cll);
        if (rl < 12) continue;
        memset(tx, 0, sizeof(tx));
        memcpy(tx, rx, 2);
        tx[2]=0x81; tx[3]=0x80; tx[4]=0; tx[5]=1; tx[6]=0; tx[7]=1;
        int ql = rl - 12;
        memcpy(tx+12, rx+12, ql);
        int off = 12 + ql;
        tx[off+0]=0xC0; tx[off+1]=0x0C; tx[off+2]=0; tx[off+3]=1;
        tx[off+4]=0; tx[off+5]=1; tx[off+8]=0; tx[off+9]=60;
        tx[off+10]=0; tx[off+11]=4; tx[off+12]=192; tx[off+13]=168; tx[off+14]=4; tx[off+15]=1;
        sendto(s, tx, off+16, 0, (struct sockaddr*)&cl, cll);
    }
}

/* ================================================================ */
/*  Reboot task (delayed, so HTTP response flushes first)            */
/* ================================================================ */
static void reboot_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

/* ================================================================ */
/*  HTTP handlers                                                    */
/* ================================================================ */

/* GET / */
static esp_err_t h_root(httpd_req_t *req) {
    send_html(req, softap_html_start, softap_html_end);
    return ESP_OK;
}

/* GET /done.html */
static esp_err_t h_done(httpd_req_t *req) {
    send_html(req, done_html_start, done_html_end);
    return ESP_OK;
}

/* 404 → redirect to / */
static esp_err_t h_404(httpd_req_t *req, httpd_err_code_t err) {
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* GET /scan */
static esp_err_t h_scan(httpd_req_t *req) {
    wifi_scan_config_t sc = {.show_hidden=false,.scan_type=WIFI_SCAN_TYPE_ACTIVE,
                             .scan_time={.active={.min=100,.max=300}}};
    esp_wifi_scan_start(&sc, true);
    uint16_t n = 0;
    esp_wifi_scan_get_ap_num(&n);
    wifi_ap_record_t *ap = calloc(n, sizeof(wifi_ap_record_t));
    esp_wifi_scan_get_ap_records(&n, ap);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "support_5g", false);  /* ESP32S3: 2.4G only */
    cJSON *aps = cJSON_AddArrayToObject(root, "aps");
    for (int i = 0; i < n; i++) {
        if (ap[i].ssid[0] == '\0') continue;
        cJSON *o = cJSON_CreateObject();
        cJSON_AddStringToObject(o, "ssid", (const char *)ap[i].ssid);
        cJSON_AddNumberToObject(o, "rssi", ap[i].rssi);
        cJSON_AddNumberToObject(o, "authmode", ap[i].authmode);
        cJSON_AddItemToArray(aps, o);
    }
    free(ap);
    char *js = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    send_json(req, js);
    free(js);
    return ESP_OK;
}

/* GET /saved/list */
static esp_err_t h_saved_list(httpd_req_t *req) {
    uint8_t n = nvs_saved_count();
    cJSON *arr = cJSON_CreateArray();
    for (int i = 0; i < n; i++) {
        char ssid[64] = {0}, pass[64] = {0};
        if (nvs_saved_get(i, ssid, sizeof(ssid), pass, sizeof(pass), NULL))
            cJSON_AddItemToArray(arr, cJSON_CreateString(ssid));
    }
    char *js = cJSON_PrintUnformatted(arr);
    cJSON_Delete(arr);
    send_json(req, js);
    free(js);
    return ESP_OK;
}

/* GET /saved/delete?index=N */
static esp_err_t h_saved_delete(httpd_req_t *req) {
    int idx = parse_index(req->uri);
    if (idx < 0 || idx >= nvs_saved_count()) {
        send_json(req, "{\"success\":false,\"error\":\"invalid index\"}");
        return ESP_OK;
    }
    nvs_saved_delete(idx);
    send_json(req, "{\"success\":true}");
    return ESP_OK;
}

/* GET /saved/set_default?index=N */
static esp_err_t h_saved_set_default(httpd_req_t *req) {
    int idx = parse_index(req->uri);
    if (idx < 0 || idx >= nvs_saved_count()) {
        send_json(req, "{\"success\":false,\"error\":\"invalid index\"}");
        return ESP_OK;
    }
    nvs_saved_set_default(idx);
    send_json(req, "{\"success\":true}");
    return ESP_OK;
}

/* POST /submit  {ssid, password}  →  connect STA + save  */
static esp_err_t h_submit(httpd_req_t *req) {
    if (s_connecting) {
        send_json(req, "{\"success\":false,\"error\":\"Already connecting\"}");
        return ESP_OK;
    }

    /* read body */
    char buf[512] = {0};
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) { send_json(req, "{\"success\":false,\"error\":\"Empty body\"}"); return ESP_OK; }
    buf[len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) { send_json(req, "{\"success\":false,\"error\":\"Invalid JSON\"}"); return ESP_OK; }
    cJSON *jssid = cJSON_GetObjectItem(root, "ssid");
    cJSON *jpass = cJSON_GetObjectItem(root, "password");
    if (!cJSON_IsString(jssid) || jssid->valuestring[0] == '\0') {
        cJSON_Delete(root);
        send_json(req, "{\"success\":false,\"error\":\"SSID required\"}");
        return ESP_OK;
    }
    char ssid[64], pass[64];
    strncpy(ssid, jssid->valuestring, sizeof(ssid) - 1);
    ssid[sizeof(ssid)-1] = '\0';
    strncpy(pass, cJSON_IsString(jpass) ? jpass->valuestring : "", sizeof(pass) - 1);
    pass[sizeof(pass)-1] = '\0';
    cJSON_Delete(root);

    ESP_LOGI(TAG, "Submit: SSID=\"%s\"", ssid);
    s_connecting = true;

    /* configure STA (wifi already running in APSTA, just swap target) */
    wifi_config_t cfg = {0};
    strncpy((char *)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid) - 1);
    strncpy((char *)cfg.sta.password, pass, sizeof(cfg.sta.password) - 1);
    cfg.sta.threshold.authmode = (pass[0]=='\0') ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    cfg.sta.pmf_cfg.capable  = true;
    cfg.sta.pmf_cfg.required = false;

    if (nvs_adv_get_u8(NVS_KEY_REM_BSSID, 0)) {
        uint8_t b[6]; bool found = false;
        uint8_t cnt = nvs_saved_count();
        for (int i = 0; i < cnt; i++) {
            char s[64], p[64];
            if (nvs_saved_get(i, s, sizeof(s), p, sizeof(p), b)
                && strcmp(s, ssid) == 0 && !memiszero(b, 6)) {
                memcpy(cfg.sta.bssid, b, 6); cfg.sta.bssid_set = true; found = true; break;
            }
        }
        if (!found) cfg.sta.bssid_set = false;
    }

    /* drain stale semaphore, then try to connect */
    xSemaphoreTake(s_ip_sem, 0);
    esp_wifi_disconnect();
    esp_wifi_set_config(ESP_IF_WIFI_STA, &cfg);
    esp_wifi_connect();

    bool ok = (xSemaphoreTake(s_ip_sem, pdMS_TO_TICKS(STA_TIMEOUT_MS)) == pdTRUE);

    char resp[128];
    if (ok) {
        /* save to NVS */
        uint8_t bssid[6] = {0};
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
            memcpy(bssid, ap_info.bssid, 6);
        nvs_saved_push_front(ssid, pass, bssid);

        snprintf(resp, sizeof(resp), "{\"success\":true}");
        send_json(req, resp);

        /* reboot after a short delay */
        xTaskCreatePinnedToCore(reboot_task, "reboot", 2048, NULL, 1, NULL, 0);
    } else {
        esp_wifi_disconnect();
        s_connecting = false;
        snprintf(resp, sizeof(resp), "{\"success\":false,\"error\":\"Connection timeout\"}");
        send_json(req, resp);
    }
    return ESP_OK;
}

/* GET /advanced/config */
static esp_err_t h_adv_get(httpd_req_t *req) {
    cJSON *o = cJSON_CreateObject();
    cJSON_AddBoolToObject(o, "show_ota_config", false);
    cJSON_AddBoolToObject(o, "show_sleep_config", false);
    cJSON_AddStringToObject(o, "ota_url", "");
    cJSON_AddNumberToObject(o, "max_tx_power", nvs_adv_get_i8(NVS_KEY_MAX_TX_PWR, 80));
    cJSON_AddBoolToObject(o, "remember_bssid", (bool)nvs_adv_get_u8(NVS_KEY_REM_BSSID, 0));
    cJSON_AddBoolToObject(o, "sleep_mode", false);
    char *js = cJSON_PrintUnformatted(o);
    cJSON_Delete(o);
    send_json(req, js);
    free(js);
    return ESP_OK;
}

/* POST /advanced/submit */
static esp_err_t h_adv_submit(httpd_req_t *req) {
    char buf[512] = {0};
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) { send_json(req, "{\"success\":false,\"error\":\"Empty\"}"); return ESP_OK; }
    buf[len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) { send_json(req, "{\"success\":false,\"error\":\"Invalid JSON\"}"); return ESP_OK; }

    cJSON *j = cJSON_GetObjectItem(root, "max_tx_power");
    if (cJSON_IsNumber(j)) {
        int8_t p = (int8_t)j->valueint;
        nvs_adv_set_i8(NVS_KEY_MAX_TX_PWR, p);
        esp_wifi_set_max_tx_power(p);
    }
    j = cJSON_GetObjectItem(root, "remember_bssid");
    if (cJSON_IsBool(j)) nvs_adv_set_u8(NVS_KEY_REM_BSSID, j->valueint ? 1 : 0);
    /* ota_url and sleep_mode are intentionally ignored */

    cJSON_Delete(root);
    send_json(req, "{\"success\":true}");
    return ESP_OK;
}

/* ================================================================ */
/*  HTTP server                                                      */
/* ================================================================ */
static void http_start(void)
{
    httpd_config_t c = HTTPD_DEFAULT_CONFIG();
    c.max_uri_handlers   = 16;
    c.max_open_sockets   = 13;
    c.lru_purge_enable   = true;
    c.stack_size         = 8192;
    if (httpd_start(&s_httpd, &c) != ESP_OK) { ESP_LOGE(TAG, "httpd fail"); return; }

    const httpd_uri_t uri_root       = {"/",                 HTTP_GET,  h_root,               NULL};
    const httpd_uri_t uri_done       = {"/done.html",        HTTP_GET,  h_done,               NULL};
    const httpd_uri_t uri_scan       = {"/scan",             HTTP_GET,  h_scan,               NULL};
    const httpd_uri_t uri_submit     = {"/submit",           HTTP_POST, h_submit,             NULL};
    const httpd_uri_t uri_saved_list = {"/saved/list",       HTTP_GET,  h_saved_list,         NULL};
    const httpd_uri_t uri_saved_del  = {"/saved/delete",     HTTP_GET,  h_saved_delete,       NULL};
    const httpd_uri_t uri_saved_def  = {"/saved/set_default",HTTP_GET,  h_saved_set_default,  NULL};
    const httpd_uri_t uri_adv_get    = {"/advanced/config",  HTTP_GET,  h_adv_get,            NULL};
    const httpd_uri_t uri_adv_submit = {"/advanced/submit",  HTTP_POST, h_adv_submit,         NULL};

    httpd_register_uri_handler(s_httpd, &uri_root);
    httpd_register_uri_handler(s_httpd, &uri_done);
    httpd_register_uri_handler(s_httpd, &uri_scan);
    httpd_register_uri_handler(s_httpd, &uri_submit);
    httpd_register_uri_handler(s_httpd, &uri_saved_list);
    httpd_register_uri_handler(s_httpd, &uri_saved_del);
    httpd_register_uri_handler(s_httpd, &uri_saved_def);
    httpd_register_uri_handler(s_httpd, &uri_adv_get);
    httpd_register_uri_handler(s_httpd, &uri_adv_submit);

    httpd_register_err_handler(s_httpd, HTTPD_404_NOT_FOUND, h_404);
    ESP_LOGI(TAG, "HTTP server ready at http://%s", AP_IP);
}

/* ================================================================ */
/*  WiFi event handler                                               */
/* ================================================================ */
static void ev_handler(void *arg, esp_event_base_t base,
                       int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_STA_START: esp_wifi_connect(); break;
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "Connected: %.*s",
                     ((wifi_event_sta_connected_t*)data)->ssid_len,
                     ((wifi_event_sta_connected_t*)data)->ssid);
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGW(TAG, "Disconnected: %d",
                     ((wifi_event_sta_disconnected_t*)data)->reason);
            break;
        case WIFI_EVENT_AP_STACONNECTED:
            ESP_LOGI(TAG, "AP client " MACSTR,
                     MAC2STR(((wifi_event_ap_staconnected_t*)data)->mac));
            break;
        case WIFI_EVENT_AP_STADISCONNECTED:
            ESP_LOGI(TAG, "AP client left");
            break;
        default: break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        if (s_ip_sem) xSemaphoreGive(s_ip_sem);
        xTaskCreatePinnedToCore(mqtt_task, "mqtt_task", 4096, NULL, 5, NULL, 1);
    }
}

/* ================================================================ */
/*  Build SoftAP config (shared by boot path and provisioning)       */
/* ================================================================ */
static void build_ap_config(wifi_config_t *apc, char *ssid_out, size_t ssid_sz)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(ssid_out, ssid_sz, "%s%02X%02X", AP_SSID_PREFIX, mac[4], mac[5]);

    memset(apc, 0, sizeof(*apc));
    apc->ap.channel        = 1;
    apc->ap.max_connection = AP_MAX_CONN;
    apc->ap.pmf_cfg.required = false;
    strncpy((char *)apc->ap.ssid, ssid_out, sizeof(apc->ap.ssid) - 1);
    if (AP_PASSWORD[0]) {
        apc->ap.authmode = WIFI_AUTH_WPA2_PSK;
        strncpy((char *)apc->ap.password, AP_PASSWORD, sizeof(apc->ap.password) - 1);
    } else {
        apc->ap.authmode = WIFI_AUTH_OPEN;
    }
}

/* ================================================================ */
/*  STA try  -- set STA config and wait for IP; wifi already running */
/* ================================================================ */
static bool sta_try(const char *ssid, const char *pass, const uint8_t *bssid)
{
    ESP_LOGI(TAG, "Trying: \"%s\"", ssid);
    wifi_config_t cfg = {0};
    strncpy((char *)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid) - 1);
    strncpy((char *)cfg.sta.password, pass, sizeof(cfg.sta.password) - 1);
    cfg.sta.threshold.authmode = (pass[0]=='\0') ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    cfg.sta.pmf_cfg.capable  = true;
    cfg.sta.pmf_cfg.required = false;
    if (bssid && !memiszero(bssid, 6)) {
        memcpy(cfg.sta.bssid, bssid, 6);
        cfg.sta.bssid_set = true;
    }

    esp_wifi_disconnect();
    esp_wifi_set_config(ESP_IF_WIFI_STA, &cfg);
    esp_wifi_connect();

    bool ok = (xSemaphoreTake(s_ip_sem, pdMS_TO_TICKS(STA_TIMEOUT_MS)) == pdTRUE);
    if (!ok) esp_wifi_disconnect();
    return ok;
}

/* ================================================================ */
/*  SoftAP provisioning  (wifi already running in APSTA)             */
/* ================================================================ */
static void start_provisioning(void)
{
    ESP_LOGI(TAG, "Starting HTTP + DNS for provisioning...");
    http_start();
    xTaskCreatePinnedToCore(dns_task, "dns", 3*1024, NULL, 5, NULL, 0);
    while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}

/* ================================================================ */
/*  Public API                                                       */
/* ================================================================ */

void wifi_init(void)
{
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NOT_FOUND) { nvs_flash_erase(); r = nvs_flash_init(); }
    ESP_ERROR_CHECK(r);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t wc = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wc));
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &ev_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ev_handler, NULL);

    int8_t txp = nvs_adv_get_i8(NVS_KEY_MAX_TX_PWR, 80);
    esp_wifi_set_max_tx_power(txp);

    s_ip_sem = xSemaphoreCreateBinary();

    /* ---- always start with AP + STA both active ---- */
    wifi_config_t ap_cfg;
    char ap_ssid[32];
    build_ap_config(&ap_cfg, ap_ssid, sizeof(ap_ssid));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "APSTA: AP=\"%s\"", ap_ssid);

    /* ---- try saved credentials ---- */
    uint8_t n = nvs_saved_count();
    for (int i = 0; i < n; i++) {
        char ssid[64], pass[64]; uint8_t bssid[6];
        if (nvs_saved_get(i, ssid, sizeof(ssid), pass, sizeof(pass), bssid)) {
            bool use_bssid = nvs_adv_get_u8(NVS_KEY_REM_BSSID, 0) && !memiszero(bssid, 6);
            if (sta_try(ssid, pass, use_bssid ? bssid : NULL)) {
                ESP_LOGI(TAG, "Wi-Fi ready (STA), disabling AP...");
                esp_wifi_set_mode(WIFI_MODE_STA);
                return;
            }
        }
    }

    ESP_LOGW(TAG, "No saved credentials — staying in provisioning");
    start_provisioning();
}