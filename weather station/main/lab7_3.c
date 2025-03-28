#include "http_request_example_main.c"
#include "https_request_example_main.c"
#include "post.c"


void app_main(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    i2c_master_init();

    while(1){

        char* location = http_get_task();
        
        // printf("%s\n", location);

        char *weather_info = https_request_task(location);

        printf("%s\n", weather_info);

        temp_task(weather_info);

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second before next cycle
    }
}

//change IP in http & post

