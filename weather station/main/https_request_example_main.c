/*
 * HTTPS GET Example using plain Mbed TLS sockets
 *
 * Contacts the howsmyssl.com API via TLS v1.2 and reads a JSON
 * response.
 *
 * Adapted from the ssl_client1 example in Mbed TLS.
 *
 * SPDX-FileCopyrightText: The Mbed TLS Contributors
 *
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "protocol_examples_common.h"
#include "esp_sntp.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "esp_tls.h"
#include "sdkconfig.h"
#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE && CONFIG_EXAMPLE_USING_ESP_TLS_MBEDTLS
#include "esp_crt_bundle.h"
#endif
#include "time_sync.h"

/* Constants that aren't configurable in menuconfig */
#define S_WEB_SERVER "wttr.in"
// #define WEB_PORT "443"

#define SERVER_URL_MAX_SZ 256

static const char *HTTPS_TAG = "example";

static const char *REQUEST_TEMPLATE = "GET /%s?Adt0m HTTP/1.1\r\n"
                                      "Host: "S_WEB_SERVER"\r\n"
                                      "User-Agent: esp-idf/1.0 esp32\r\n"
                                      "\r\n";

extern const uint8_t server_root_cert_pem_start[] asm("_binary_server_root_cert_pem_start");
extern const uint8_t server_root_cert_pem_end[]   asm("_binary_server_root_cert_pem_end");

static char* https_get_request(esp_tls_cfg_t cfg, const char *WEB_SERVER_URL, const char *REQUEST)
{
    char buf[512];
    int ret, len;
    char *response = NULL;
    int headers_end = 0;
    int total_len = 0;
    bool headers_received = false;

    esp_tls_t *tls = esp_tls_init();
    if (!tls) {
        ESP_LOGE(HTTPS_TAG, "Failed to allocate esp_tls handle!");
        goto exit;
    }

    if (esp_tls_conn_http_new_sync(WEB_SERVER_URL, &cfg, tls) == 1) {
        ESP_LOGI(HTTPS_TAG, "Connection established...");
    } else {
        ESP_LOGE(HTTPS_TAG, "Connection failed...");
        int esp_tls_code = 0, esp_tls_flags = 0;
        esp_tls_error_handle_t tls_e = NULL;
        esp_tls_get_error_handle(tls, &tls_e);
        /* Try to get TLS stack level error and certificate failure flags, if any */
        ret = esp_tls_get_and_clear_last_error(tls_e, &esp_tls_code, &esp_tls_flags);
        if (ret == ESP_OK) {
            ESP_LOGE(HTTPS_TAG, "TLS error = -0x%x, TLS flags = -0x%x", esp_tls_code, esp_tls_flags);
        }
        goto cleanup;
    }

    size_t written_bytes = 0;
    do {
        ret = esp_tls_conn_write(tls,
                                 REQUEST + written_bytes,
                                 strlen(REQUEST) - written_bytes);
        if (ret >= 0) {
            ESP_LOGI(HTTPS_TAG, "%d bytes written", ret);
            written_bytes += ret;
        } else if (ret != ESP_TLS_ERR_SSL_WANT_READ  && ret != ESP_TLS_ERR_SSL_WANT_WRITE) {
            ESP_LOGE(HTTPS_TAG, "esp_tls_conn_write  returned: [0x%02X](%s)", ret, esp_err_to_name(ret));
            goto cleanup;
        }
    } while (written_bytes < strlen(REQUEST));

    ESP_LOGI(HTTPS_TAG, "Reading HTTP response...");
    do {
        len = sizeof(buf) - 1;
        memset(buf, 0x00, sizeof(buf));
        ret = esp_tls_conn_read(tls, (char *)buf, len);

        if (ret == ESP_TLS_ERR_SSL_WANT_WRITE  || ret == ESP_TLS_ERR_SSL_WANT_READ) {
            continue;
        } else if (ret < 0) {
            ESP_LOGE(HTTPS_TAG, "esp_tls_conn_read  returned [-0x%02X](%s)", -ret, esp_err_to_name(ret));
            break;
        } else if (ret == 0) {
            ESP_LOGI(HTTPS_TAG, "connection closed");
            break;
        }

        len = ret;
        ESP_LOGD(HTTPS_TAG, "%d bytes read", len);

        if (!headers_received) {
            char *headers_end_ptr = strstr(buf, "\r\n\r\n");
            if (headers_end_ptr) {
                headers_end = headers_end_ptr - buf + 4;
                headers_received = true;
                total_len = len - headers_end;
                response = malloc(total_len + 1);
                if (response) {
                    memcpy(response, buf + headers_end, total_len);
                    response[total_len] = '\0';
                }
            }
        } else {
            total_len += len;
            response = realloc(response, total_len + 1);
            if (response) {
                memcpy(response + strlen(response), buf, len);
                response[total_len] = '\0';
            }
        }
    } while (!headers_received || ret > 0);

cleanup:
    esp_tls_conn_destroy(tls);
exit:
    return response;
}

static char* https_get_request_using_crt_bundle(const char *location)
{
    char url[SERVER_URL_MAX_SZ];
    char request[512];
    
    snprintf(url, sizeof(url), "https://%s/%s?format=%%t&m", S_WEB_SERVER, location);
    snprintf(request, sizeof(request), REQUEST_TEMPLATE, location);

    ESP_LOGI(HTTPS_TAG, "https_request using crt bundle");
    esp_tls_cfg_t cfg = {
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    return https_get_request(cfg, url, request);
}

static char* https_get_request_using_cacert_buf(const char *location)
{
    char url[SERVER_URL_MAX_SZ];
    char request[512];
    
    snprintf(url, sizeof(url), "https://%s/%s?format=%%t&m", S_WEB_SERVER, location);
    snprintf(request, sizeof(request), REQUEST_TEMPLATE, location);

    ESP_LOGI(HTTPS_TAG, "https_request using cacert_buf");
    esp_tls_cfg_t cfg = {
        .cacert_buf = (const unsigned char *) server_root_cert_pem_start,
        .cacert_bytes = server_root_cert_pem_end - server_root_cert_pem_start,
    };
    return https_get_request(cfg, url, request);
}

static char* https_request_task(const char *location)
{
    ESP_LOGI(HTTPS_TAG, "Start https_request example for location: %s", location);


    // char *response = https_get_request_using_cacert_buf(location);
    char *response = https_get_request_using_crt_bundle(location);


    ESP_LOGI(HTTPS_TAG, "Finish https_request example");
    return response;
}

// void app_main(void)
// {
//     ESP_ERROR_CHECK(nvs_flash_init());
//     ESP_ERROR_CHECK(esp_netif_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());

//     /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
//      * Read "Establishing Wi-Fi or Ethernet Connection" section in
//      * examples/protocols/README.md for more information about this function.
//      */
//     ESP_ERROR_CHECK(example_connect());

//     // char *location = "Santa+Cruz"; // Change this to the desired location
//     char *weather_info = https_request_task(location);

//     printf("%s\n", weather_info);
// }
