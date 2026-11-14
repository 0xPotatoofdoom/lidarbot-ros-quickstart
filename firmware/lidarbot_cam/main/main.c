/*
 * lidarbot-cam: open firmware for Yahboom's ESP32-S3 Wi-Fi camera module (GC2145 sensor).
 *
 * Serves the same endpoints as Yahboom's firmware, so the ROS camera driver keeps working:
 *   http://<ip>:81/stream     MJPEG stream
 *   http://<ip>/capture       one JPEG
 *   http://<ip>/status        JSON: settings, measured fps, Wi-Fi signal
 *   http://<ip>/control?var=<name>&val=<n>
 *
 * The GC2145 has no JPEG encoder, so frames come in as YUV422 and are compressed here.
 * Wi-Fi uses the network already saved in NVS by Yahboom's firmware; nothing is compiled in.
 * Pin numbers were read from Yahboom's firmware (docs/06-camera.md).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_http_server.h"
#include "lwip/sockets.h"
#include "driver/uart.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_jpeg_enc.h"

#define FW_VERSION "0.1.0"
static const char *TAG = "lidarbot_cam";

/* Camera pins (Yahboom ESP32-S3 module) */
#define PIN_PWDN 48
#define PIN_RESET 47
#define PIN_XCLK 15
#define PIN_SDA 4
#define PIN_SCL 5
#define PIN_D7 16
#define PIN_D6 17
#define PIN_D5 18
#define PIN_D4 9
#define PIN_D3 11
#define PIN_D2 8
#define PIN_D1 12
#define PIN_D0 10
#define PIN_VSYNC 6
#define PIN_HREF 7
#define PIN_PCLK 13

/* VGA is the largest that works: two 600 KB YUV422 buffers in PSRAM. SVGA fails to get its
 * DMA buffer in internal RAM (measured 2026-11-13). */
#define MAX_FRAMESIZE FRAMESIZE_VGA
#define DEFAULT_FRAMESIZE FRAMESIZE_VGA

static SemaphoreHandle_t s_cam_lock;
static bool s_cam_ok = false;
static int s_xclk_mhz = 15;
static int s_quality = 12;              /* esp32-camera convention: 0 best .. 63 worst */
static framesize_t s_framesize = DEFAULT_FRAMESIZE;
static pixformat_t s_pixformat = PIXFORMAT_YUV422;
/* Checked against a known scene on our module (2026-11-13): mirror on, vertical flip off
 * gives an upright, unmirrored image. */
static int s_hmirror = 1;
static int s_vflip = 0;

/* Stream statistics, for /status */
static volatile float s_fps = 0;
static volatile uint32_t s_jpeg_bytes = 0;
static volatile float s_encode_ms = 0;
static volatile float s_grab_ms = 0;          /* time waiting for the sensor's next frame */
static volatile uint32_t s_grab_fail = 0;

/* Last 4 KB of log output, served at /log (opening the serial port resets the module). */
#define LOG_RING 4096
static char s_log[LOG_RING];
static size_t s_log_pos = 0;
static portMUX_TYPE s_log_mux = portMUX_INITIALIZER_UNLOCKED;
static vprintf_like_t s_uart_vprintf;

static int log_to_ring(const char *fmt, va_list args)
{
    char line[160];
    va_list copy;
    va_copy(copy, args);
    int n = vsnprintf(line, sizeof(line), fmt, copy);
    va_end(copy);
    if (n > 0) {
        if (n > (int)sizeof(line) - 1) {
            n = sizeof(line) - 1;
        }
        portENTER_CRITICAL(&s_log_mux);
        for (int i = 0; i < n; i++) {
            s_log[s_log_pos] = line[i];
            s_log_pos = (s_log_pos + 1) % LOG_RING;
        }
        portEXIT_CRITICAL(&s_log_mux);
    }
    return s_uart_vprintf(fmt, args);
}

static esp_ip4_addr_t s_ip;
static volatile bool s_connected = false;

/* ---------- camera ---------- */

static esp_err_t camera_start(framesize_t fs)
{
    camera_config_t c = {
        .pin_pwdn = PIN_PWDN, .pin_reset = PIN_RESET, .pin_xclk = PIN_XCLK,
        .pin_sccb_sda = PIN_SDA, .pin_sccb_scl = PIN_SCL,
        .pin_d7 = PIN_D7, .pin_d6 = PIN_D6, .pin_d5 = PIN_D5, .pin_d4 = PIN_D4,
        .pin_d3 = PIN_D3, .pin_d2 = PIN_D2, .pin_d1 = PIN_D1, .pin_d0 = PIN_D0,
        .pin_vsync = PIN_VSYNC, .pin_href = PIN_HREF, .pin_pclk = PIN_PCLK,
        .xclk_freq_hz = s_xclk_mhz * 1000000,
        .ledc_timer = LEDC_TIMER_0, .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = s_pixformat,
        .frame_size = fs,
        .jpeg_quality = 12,
        .fb_count = fs <= FRAMESIZE_VGA ? 2 : 1,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = fs <= FRAMESIZE_VGA ? CAMERA_GRAB_LATEST : CAMERA_GRAB_WHEN_EMPTY,
        .sccb_i2c_port = 0,
    };
    esp_err_t err = esp_camera_init(&c);
    if (err == ESP_OK) {
        s_framesize = fs;
        sensor_t *s = esp_camera_sensor_get();
        if (s && s->set_hmirror && s->set_vflip) {
            s->set_hmirror(s, s_hmirror);
            s->set_vflip(s, s_vflip);
        }
        ESP_LOGI(TAG, "camera ok: sensor PID 0x%x, %ux%u, xclk %d MHz", s ? s->id.PID : 0,
                 resolution[fs].width, resolution[fs].height, s_xclk_mhz);
    } else {
        ESP_LOGE(TAG, "camera init failed: %s", esp_err_to_name(err));
    }
    s_cam_ok = (err == ESP_OK);
    return err;
}

/* Restart the camera with a new frame size (buffers are sized at init). Caller holds the lock. */
static esp_err_t camera_restart(framesize_t fs)
{
    framesize_t old = s_framesize;
    esp_camera_deinit();
    esp_err_t err = camera_start(fs);
    if (err != ESP_OK && fs != old) {
        camera_start(old);
    }
    return err;
}

/* Espressif's SIMD JPEG encoder for YUV422 frames; reopened when the size or quality changes. */
static jpeg_enc_handle_t s_enc = NULL;
static int s_enc_w = 0, s_enc_h = 0, s_enc_q = 0;
static volatile int s_encoder = 0;            /* 1 = esp_new_jpeg, 0 = frame2jpg, for /status */

static bool fast_jpeg(camera_fb_t *fb, int q, uint8_t **out, size_t *len)
{
    if (fb->format != PIXFORMAT_YUV422) {
        return false;
    }
    if (!s_enc || s_enc_w != fb->width || s_enc_h != fb->height || s_enc_q != q) {
        if (s_enc) {
            jpeg_enc_close(s_enc);
            s_enc = NULL;
        }
        jpeg_enc_config_t cfg = DEFAULT_JPEG_ENC_CONFIG();
        cfg.width = fb->width;
        cfg.height = fb->height;
        cfg.src_type = JPEG_PIXEL_FORMAT_YCbYCr;
        cfg.subsampling = JPEG_SUBSAMPLE_422;
        cfg.quality = q;
        if (jpeg_enc_open(&cfg, &s_enc) != JPEG_ERR_OK) {
            s_enc = NULL;
            return false;
        }
        s_enc_w = fb->width;
        s_enc_h = fb->height;
        s_enc_q = q;
    }
    size_t cap = fb->width * fb->height / 2 + 4096;       /* generous for q <= 95 */
    uint8_t *buf = heap_caps_aligned_alloc(16, cap, MALLOC_CAP_SPIRAM);
    if (!buf) {
        return false;
    }
    int out_len = 0;
    if (jpeg_enc_process(s_enc, fb->buf, fb->len, buf, cap, &out_len) != JPEG_ERR_OK || out_len <= 0) {
        heap_caps_free(buf);
        return false;
    }
    *out = buf;
    *len = out_len;
    return true;
}

/* Grab the newest frame and compress it. The caller frees *out (heap_caps_free/free). */
static bool grab_jpeg(uint8_t **out, size_t *len)
{
    bool ok = false;
    xSemaphoreTake(s_cam_lock, portMAX_DELAY);
    if (s_cam_ok) {
        int64_t g0 = esp_timer_get_time();
        camera_fb_t *fb = esp_camera_fb_get();
        s_grab_ms = (esp_timer_get_time() - g0) / 1000.0f;
        if (!fb) {
            s_grab_fail++;
        }
        if (fb) {
            int64_t t0 = esp_timer_get_time();
            if (fb->format == PIXFORMAT_JPEG) {
                *out = malloc(fb->len);
                if (*out) {
                    memcpy(*out, fb->buf, fb->len);
                    *len = fb->len;
                    ok = true;
                }
            } else {
                int q = 100 - s_quality;           /* 12 -> JPEG quality 88 */
                q = q < 10 ? 10 : q;
                ok = fast_jpeg(fb, q, out, len);
                s_encoder = ok ? 1 : 0;
                if (!ok) {
                    ok = frame2jpg(fb, q, out, len);
                }
            }
            s_encode_ms = (esp_timer_get_time() - t0) / 1000.0f;
            esp_camera_fb_return(fb);
        }
    }
    xSemaphoreGive(s_cam_lock);
    return ok;
}

/* ---------- HTTP ---------- */

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %lld\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req)
{
    char part[192];
    httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    int one = 1;                                    /* send each part now, don't wait for ACKs */
    setsockopt(httpd_req_to_sockfd(req), IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    int64_t window_start = esp_timer_get_time();
    int frames = 0, failures = 0;
    while (true) {
        uint8_t *jpg = NULL;
        size_t len = 0;
        if (!grab_jpeg(&jpg, &len)) {
            if (++failures > 50) {
                return ESP_FAIL;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        failures = 0;
        /* boundary and part header in one write */
        int n = snprintf(part, sizeof(part), "%s", STREAM_BOUNDARY);
        n += snprintf(part + n, sizeof(part) - n, STREAM_PART, (unsigned)len, (long long)esp_timer_get_time());
        esp_err_t r = httpd_resp_send_chunk(req, part, n);
        if (r == ESP_OK) {
            r = httpd_resp_send_chunk(req, (const char *)jpg, len);
        }
        heap_caps_free(jpg);
        if (r != ESP_OK) {
            break;                                  /* client went away */
        }
        s_jpeg_bytes = len;
        frames++;
        int64_t now = esp_timer_get_time();
        if (now - window_start >= 2000000) {
            s_fps = frames * 1e6f / (now - window_start);
            frames = 0;
            window_start = now;
        }
    }
    return ESP_OK;
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    uint8_t *jpg = NULL;
    size_t len = 0;
    if (!grab_jpeg(&jpg, &len)) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t r = httpd_resp_send(req, (const char *)jpg, len);
    heap_caps_free(jpg);
    return r;
}

static esp_err_t status_handler(httpd_req_t *req)
{
    char json[1024];
    sensor_t *s = s_cam_ok ? esp_camera_sensor_get() : NULL;
    camera_status_t st = {0};
    if (s) {
        st = s->status;
    }
    wifi_ap_record_t ap;
    int rssi = esp_wifi_sta_get_ap_info(&ap) == ESP_OK ? ap.rssi : 0;
    int n = snprintf(json, sizeof(json),
        "{\"board\":\"lidarbot-cam\",\"fw\":\"" FW_VERSION "\",\"camera_ok\":%d,\"sensor_pid\":%d,"
        "\"xclk\":%d,\"pixformat\":%d,\"framesize\":%d,\"width\":%u,\"height\":%u,\"quality\":%d,"
        "\"brightness\":%d,\"contrast\":%d,\"saturation\":%d,\"sharpness\":%d,\"special_effect\":%d,"
        "\"awb\":%u,\"awb_gain\":%u,\"wb_mode\":%u,\"aec\":%u,\"aec2\":%u,\"ae_level\":%d,\"aec_value\":%u,"
        "\"agc\":%u,\"agc_gain\":%u,\"gainceiling\":%u,\"hmirror\":%u,\"vflip\":%u,\"dcw\":%u,\"colorbar\":%u,"
        "\"encoder\":%d,\"fps\":%.1f,\"jpeg_bytes\":%lu,\"encode_ms\":%.1f,\"grab_ms\":%.1f,\"grab_fail\":%lu,\"rssi\":%d,"
        "\"psram_free\":%u,\"heap_free\":%u,\"uptime_s\":%lld}",
        s_cam_ok, s ? s->id.PID : 0, s_xclk_mhz, (int)s_pixformat, (int)s_framesize,
        resolution[s_framesize].width, resolution[s_framesize].height, s_quality,
        st.brightness, st.contrast, st.saturation, st.sharpness, st.special_effect,
        st.awb, st.awb_gain, st.wb_mode, st.aec, st.aec2, st.ae_level, st.aec_value,
        st.agc, st.agc_gain, st.gainceiling, st.hmirror, st.vflip, st.dcw, st.colorbar,
        s_encoder, s_fps, (unsigned long)s_jpeg_bytes, s_encode_ms, s_grab_ms, (unsigned long)s_grab_fail, rssi,
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
        (long long)(esp_timer_get_time() / 1000000));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, n);
}

/* Call a sensor setter if the GC2145 driver implements it; -1 if it doesn't. */
#define SENSOR_SET(fn, v) ((s && s->fn) ? s->fn(s, (v)) : -1)

static esp_err_t control_handler(httpd_req_t *req)
{
    char query[128], var[32], val[16];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "var", var, sizeof(var)) != ESP_OK ||
        httpd_query_key_value(query, "val", val, sizeof(val)) != ESP_OK) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    int v = atoi(val);
    int res = -1;
    xSemaphoreTake(s_cam_lock, portMAX_DELAY);
    sensor_t *s = s_cam_ok ? esp_camera_sensor_get() : NULL;
    if (!strcmp(var, "framesize")) {
        if (v >= 0 && v <= MAX_FRAMESIZE) {
            res = (v == s_framesize) ? 0 : (camera_restart((framesize_t)v) == ESP_OK ? 0 : -1);
        }
    } else if (!strcmp(var, "pixformat")) {
        /* What the sensor sends before JPEG: 0 = RGB565, 1 = YUV422 */
        if (v == PIXFORMAT_RGB565 || v == PIXFORMAT_YUV422) {
            s_pixformat = (pixformat_t)v;
            res = camera_restart(s_framesize) == ESP_OK ? 0 : -1;
        }
    } else if (!strcmp(var, "xclk")) {
        if (v >= 8 && v <= 24) {
            s_xclk_mhz = v;
            res = camera_restart(s_framesize) == ESP_OK ? 0 : -1;
        }
    } else if (!strcmp(var, "quality")) {
        if (v >= 0 && v <= 63) {
            s_quality = v;
            res = 0;
        }
    } else if (!strcmp(var, "brightness")) res = SENSOR_SET(set_brightness, v);
    else if (!strcmp(var, "contrast")) res = SENSOR_SET(set_contrast, v);
    else if (!strcmp(var, "saturation")) res = SENSOR_SET(set_saturation, v);
    else if (!strcmp(var, "sharpness")) res = SENSOR_SET(set_sharpness, v);
    else if (!strcmp(var, "special_effect")) res = SENSOR_SET(set_special_effect, v);
    else if (!strcmp(var, "awb")) res = SENSOR_SET(set_whitebal, v);
    else if (!strcmp(var, "awb_gain")) res = SENSOR_SET(set_awb_gain, v);
    else if (!strcmp(var, "wb_mode")) res = SENSOR_SET(set_wb_mode, v);
    else if (!strcmp(var, "aec")) res = SENSOR_SET(set_exposure_ctrl, v);
    else if (!strcmp(var, "aec2")) res = SENSOR_SET(set_aec2, v);
    else if (!strcmp(var, "ae_level")) res = SENSOR_SET(set_ae_level, v);
    else if (!strcmp(var, "aec_value")) res = SENSOR_SET(set_aec_value, v);
    else if (!strcmp(var, "agc")) res = SENSOR_SET(set_gain_ctrl, v);
    else if (!strcmp(var, "agc_gain")) res = SENSOR_SET(set_agc_gain, v);
    else if (!strcmp(var, "gainceiling")) res = SENSOR_SET(set_gainceiling, (gainceiling_t)v);
    else if (!strcmp(var, "hmirror")) { res = SENSOR_SET(set_hmirror, v); if (res == 0) s_hmirror = v; }
    else if (!strcmp(var, "vflip")) { res = SENSOR_SET(set_vflip, v); if (res == 0) s_vflip = v; }
    else if (!strcmp(var, "dcw")) res = SENSOR_SET(set_dcw, v);
    else if (!strcmp(var, "colorbar")) res = SENSOR_SET(set_colorbar, v);
    xSemaphoreGive(s_cam_lock);
    if (res != 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t log_handler(httpd_req_t *req)
{
    static char out[LOG_RING];
    portENTER_CRITICAL(&s_log_mux);
    size_t tail = LOG_RING - s_log_pos;
    memcpy(out, s_log + s_log_pos, tail);
    memcpy(out + tail, s_log, s_log_pos);
    portEXIT_CRITICAL(&s_log_mux);
    size_t start = 0;
    while (start < LOG_RING && out[start] == 0) {
        start++;
    }
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, out + start, LOG_RING - start);
}

static esp_err_t index_handler(httpd_req_t *req)
{
    static const char *page =
        "<!doctype html><html><head><meta name=viewport content='width=device-width'>"
        "<title>lidarbot-cam</title></head><body style='font-family:sans-serif'>"
        "<h3>lidarbot-cam " FW_VERSION "</h3>"
        "<p><a href='/status'>status</a> | <a href='/capture'>capture</a></p>"
        "<img id=s style='max-width:100%'>"
        "<script>document.getElementById('s').src=location.protocol+'//'+location.hostname+':81/stream'</script>"
        "</body></html>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

static void start_servers(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.ctrl_port = 32768;
    cfg.max_uri_handlers = 8;
    httpd_handle_t web = NULL;
    if (httpd_start(&web, &cfg) == ESP_OK) {
        httpd_uri_t uris[] = {
            {.uri = "/", .method = HTTP_GET, .handler = index_handler},
            {.uri = "/status", .method = HTTP_GET, .handler = status_handler},
            {.uri = "/control", .method = HTTP_GET, .handler = control_handler},
            {.uri = "/capture", .method = HTTP_GET, .handler = capture_handler},
            {.uri = "/log", .method = HTTP_GET, .handler = log_handler},
        };
        for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
            httpd_register_uri_handler(web, &uris[i]);
        }
    }
    cfg.server_port = 81;
    cfg.ctrl_port = 32769;
    cfg.stack_size = 8192;
    httpd_handle_t stream = NULL;
    if (httpd_start(&stream, &cfg) == ESP_OK) {
        httpd_uri_t u = {.uri = "/stream", .method = HTTP_GET, .handler = stream_handler};
        httpd_register_uri_handler(stream, &u);
    }
}

/* ---------- Wi-Fi ---------- */

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_connected = false;
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_ip = ((ip_event_got_ip_t *)data)->ip_info.ip;
        s_connected = true;
        ESP_LOGI(TAG, "network up, ip " IPSTR, IP2STR(&s_ip));
    }
}

/* If the Wi-Fi stack has no saved network, fall back to the one Yahboom's firmware saved. */
static void load_yahboom_network(wifi_config_t *wc)
{
    nvs_handle_t h;
    if (nvs_open("YAHBOOM", NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    size_t n = sizeof(wc->sta.ssid);
    if (nvs_get_blob(h, "WIFI_SSID", wc->sta.ssid, &n) == ESP_OK) {
        n = sizeof(wc->sta.password);
        nvs_get_blob(h, "WIFI_PASSWD", wc->sta.password, &n);
    }
    nvs_close(h);
}

static void wifi_start(void)
{
    esp_netif_t *sta = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(sta, "lidarbot-cam");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));    /* no hotspot */
    wifi_config_t wc = {0};
    esp_wifi_get_config(WIFI_IF_STA, &wc);
    if (wc.sta.ssid[0] == 0) {
        load_yahboom_network(&wc);
        if (wc.sta.ssid[0] != 0) {
            esp_wifi_set_config(WIFI_IF_STA, &wc);
        }
    }
    ESP_LOGI(TAG, "saved network: %s", wc.sta.ssid[0] ? "yes" : "none (set it with sta_ssid:/sta_pd: on the serial port)");
    memset(&wc, 0, sizeof(wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);                        /* power save adds 100+ ms of latency */
}

/* ---------- serial commands (Yahboom-compatible subset) ---------- */

static void handle_command(char *cmd)
{
    char *sep = strpbrk(cmd, ":,.");
    const char *arg = sep ? sep + 1 : "";
    if (sep) {
        *sep = 0;
    }
    for (char *p = cmd; *p; p++) {
        *p = tolower((unsigned char)*p);
    }
    static char pending_ssid[33];
    char reply[64];
    if (!strcmp(cmd, "wifi_ver")) {
        snprintf(reply, sizeof(reply), "YAHBOOM Board Ver:lidarbot-cam_" FW_VERSION "\r\n");
    } else if (!strcmp(cmd, "sta_ip")) {
        if (s_connected) {
            snprintf(reply, sizeof(reply), "sta_ip:" IPSTR "\r\n", IP2STR(&s_ip));
        } else {
            snprintf(reply, sizeof(reply), "sta_ip:null\r\n");
        }
    } else if (!strcmp(cmd, "ap_ip")) {
        snprintf(reply, sizeof(reply), "ap_ip:null\r\n");
    } else if (!strcmp(cmd, "wifi_mode")) {
        snprintf(reply, sizeof(reply), "OK\r\n");          /* always station-only */
    } else if (!strcmp(cmd, "sta_ssid")) {
        strlcpy(pending_ssid, arg, sizeof(pending_ssid));
        snprintf(reply, sizeof(reply), pending_ssid[0] ? "OK\r\n" : "fail,ssid is null!\r\n");
    } else if (!strcmp(cmd, "sta_pd")) {
        wifi_config_t wc = {0};
        esp_wifi_get_config(WIFI_IF_STA, &wc);
        if (pending_ssid[0]) {
            strlcpy((char *)wc.sta.ssid, pending_ssid, sizeof(wc.sta.ssid));
        }
        strlcpy((char *)wc.sta.password, arg, sizeof(wc.sta.password));
        esp_wifi_disconnect();
        esp_wifi_set_config(WIFI_IF_STA, &wc);
        esp_wifi_connect();
        memset(&wc, 0, sizeof(wc));
        snprintf(reply, sizeof(reply), "OK\r\n");
    } else {
        return;
    }
    uart_write_bytes(UART_NUM_0, reply, strlen(reply));
}

static void console_task(void *arg)
{
    uart_driver_install(UART_NUM_0, 512, 0, 0, NULL, 0);
    char buf[128];
    size_t len = 0;
    while (true) {
        uint8_t c;
        int n = uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(60));
        if (n == 1 && len < sizeof(buf) - 1 && c != '\r' && c != '\n') {
            buf[len++] = (char)c;
        } else if (len > 0 && (n == 0 || c == '\r' || c == '\n')) {
            buf[len] = 0;                                  /* idle gap or newline ends a command */
            handle_command(buf);
            memset(buf, 0, sizeof(buf));
            len = 0;
        }
    }
}

void app_main(void)
{
    s_uart_vprintf = esp_log_set_vprintf(log_to_ring);
    esp_log_level_set("wifi", ESP_LOG_WARN);               /* the Wi-Fi driver logs the network name */
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        /* Never erase: nvs holds the Wi-Fi settings. */
        ESP_LOGE(TAG, "nvs init failed: %s", esp_err_to_name(err));
    }
    s_cam_lock = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_start();
    if (camera_start(DEFAULT_FRAMESIZE) != ESP_OK) {
        s_pixformat = PIXFORMAT_RGB565;
        camera_start(FRAMESIZE_QVGA);
    }
    start_servers();
    xTaskCreate(console_task, "console", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "lidarbot-cam %s ready", FW_VERSION);
}
