#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <bmp280.h>
#include <string.h>
#include <esp_sleep.h>
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#define SDA_GPIO 21
#define SCL_GPIO 22

typedef struct test_struct
{
    float temp;
    float humidity;
    float pressure;
} test_struct;

uint8_t broadcastAddress[] = {0x5C, 0xCF, 0x7F, 0x1B, 0xE0, 0x89};

static void wifi_init(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    //ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void bmp280_test()
{
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    bmp280_t dev;
    memset(&dev, 0, sizeof(bmp280_t));

    ESP_ERROR_CHECK(bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_0, 0, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(bmp280_init(&dev, &params));

    bool bme280p = dev.id == BME280_CHIP_ID;
    printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

    //float pressure, temperature, humidity;
    test_struct test;

    vTaskDelay(500 / portTICK_PERIOD_MS);
    if (bmp280_read_float(&dev, &test.temp, &test.pressure, &test.humidity) != ESP_OK)
    {
        printf("Temperature/pressure reading failed\n");
    }
    else
    {
        printf("Pressure: %.2f Pa, Temperature: %.2f C", test.pressure, test.temp);
        printf(", Humidity: %.2f\n", test.humidity);

        esp_now_send(broadcastAddress, (uint8_t *)&test, sizeof(test_struct));
    }

    esp_deep_sleep(5 * 1000000);
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    //xTaskCreatePinnedToCore(bmp280_test, "bmp280_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    wifi_init();

    esp_now_peer_info_t peerInfo;

    // 5C:CF:7F:1B:E0:89
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));

    bmp280_test();
}
