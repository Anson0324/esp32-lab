/* HTTP GET Example using plain POSIX sockets

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "sdkconfig.h"

/* HTTP GET Example using plain POSIX sockets

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "sdkconfig.h"

/* Constants that aren't configurable in menuconfig */
#define WEB_SERVER "169.233.174.202"
#define WEB_PORT "1234"
#define WEB_PATH "/location"

static const char *HTTP_TAG = "example";

static const char *REQUEST = "GET " WEB_PATH " HTTP/1.0\r\n"
    "Host: " WEB_SERVER ":" WEB_PORT "\r\n"
    "User-Agent: esp-idf/1.0 esp32\r\n"
    "\r\n";

static char* http_get_task(void)
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[128];
    char response_buf[512];
    int response_len = 0;

    int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

    if (err != 0 || res == NULL) {
        ESP_LOGE(HTTP_TAG, "DNS lookup failed err=%d res=%p", err, res);
        return NULL;
    }

    addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
    ESP_LOGI(HTTP_TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

    s = socket(res->ai_family, res->ai_socktype, 0);
    if (s < 0) {
        ESP_LOGE(HTTP_TAG, "... Failed to allocate socket.");
        freeaddrinfo(res);
        return NULL;
    }
    ESP_LOGI(HTTP_TAG, "... allocated socket");

    if (connect(s, res->ai_addr, res->ai_addrlen) != 0) {
        ESP_LOGE(HTTP_TAG, "... socket connect failed errno=%d", errno);
        close(s);
        freeaddrinfo(res);
        return NULL;
    }

    ESP_LOGI(HTTP_TAG, "... connected");
    freeaddrinfo(res);

    if (write(s, REQUEST, strlen(REQUEST)) < 0) {
        ESP_LOGE(HTTP_TAG, "... socket send failed");
        close(s);
        return NULL;
    }
    ESP_LOGI(HTTP_TAG, "... socket send success");

    struct timeval receiving_timeout;
    receiving_timeout.tv_sec = 5;
    receiving_timeout.tv_usec = 0;
    if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
            sizeof(receiving_timeout)) < 0) {
        ESP_LOGE(HTTP_TAG, "... failed to set socket receiving timeout");
        close(s);
        return NULL;
    }
    ESP_LOGI(HTTP_TAG, "... set socket receiving timeout success");

    /* Read HTTP response */
    response_len = 0;
    do {
        bzero(recv_buf, sizeof(recv_buf));
        r = read(s, recv_buf, sizeof(recv_buf) - 1);
        if (r > 0) {
            if (response_len + r < sizeof(response_buf) - 1) {
                memcpy(response_buf + response_len, recv_buf, r);
                response_len += r;
                response_buf[response_len] = '\0';
            } else {
                ESP_LOGW(HTTP_TAG, "Response buffer overflow");
            }
        }
    } while (r > 0);

    if (response_len > 0) {
        char *body = strstr(response_buf, "\r\n\r\n");
        if (body) {
            body += 4; // Move past the header-body separator
            ESP_LOGI(HTTP_TAG, "Extracted location: %s", body);
            // Allocate memory for the body and copy it
            char *body_copy = malloc(strlen(body) + 1);
            if (body_copy) {
                strcpy(body_copy, body);
            }
            close(s); // Close socket before returning
            return body_copy;
        } else {
            ESP_LOGW(HTTP_TAG, "Body not found in response");
        }
    }

    ESP_LOGI(HTTP_TAG, "... done reading from socket. Last read return=%d errno=%d.", r, errno);
    close(s);
    return NULL;
}



// void app_main(void)
// {
//     ESP_ERROR_CHECK( nvs_flash_init() );
//     ESP_ERROR_CHECK(esp_netif_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());

//     /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
//      * Read "Establishing Wi-Fi or Ethernet Connection" section in
//      * examples/protocols/README.md for more information about this function.
//      */
//     ESP_ERROR_CHECK(example_connect());

//     char* body = http_get_task();
    
//     printf("%s\n", body);
// }