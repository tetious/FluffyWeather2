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
#include <ina219.h>
#include <esp_log.h>
#include <esp_pm.h>
#include "../../shared.h"

#define SDA_GPIO 21
#define SCL_GPIO 22

const static char *TAG = "FluffyWeather";

uint8_t broadcastAddress[] = {0x5C, 0xCF, 0x7F, 0x1B, 0xE0, 0x89};

ina219_t i_sensor;
bmp280_t bmp_sensor;

void master_init()
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

    esp_pm_config_esp32_t pm_config = {
        .max_freq_mhz = 80,
        .min_freq_mhz = 40,
        .light_sleep_enable = true};
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
}

static void wifi_init(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    //ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(esp_now_peer_info_t));

    // 5C:CF:7F:1B:E0:89
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
    esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
}

void init_bmp280()
{
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    memset(&bmp_sensor, 0, sizeof(bmp280_t));

    ESP_ERROR_CHECK(bmp280_init_desc(&bmp_sensor, BMP280_I2C_ADDRESS_0, 0, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(bmp280_init(&bmp_sensor, &params));

    bool bme280p = bmp_sensor.id == BME280_CHIP_ID;
    printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");
}

void measure_weather(data_struct *data)
{
    if (bmp280_read_float(&bmp_sensor, &data->temp, &data->pressure, &data->humidity) != ESP_OK)
    {
        printf("Temperature/pressure reading failed\n");
    }
    else
    {
        printf("Pressure: %.2f Pa, Temperature: %.2f C, Humidity: %.2f\n", data->pressure, data->temp, data->humidity);
    }
}

void init_ina()
{
    memset(&i_sensor, 0, sizeof(ina219_t));
    ESP_ERROR_CHECK(ina219_init_desc(&i_sensor, INA219_ADDR_GND_GND, 0, SDA_GPIO, SCL_GPIO));
    ESP_LOGI(TAG, "Initializing INA219");
    ESP_ERROR_CHECK(ina219_init(&i_sensor));
    ESP_ERROR_CHECK(ina219_configure(&i_sensor, INA219_BUS_RANGE_16V, INA219_GAIN_1,
                                     INA219_RES_12BIT_64S, INA219_RES_12BIT_64S, INA219_MODE_CONT_SHUNT_BUS));
    ESP_ERROR_CHECK(ina219_calibrate(&i_sensor, 0.5, 0.1));
}

void measure_current(data_struct *data)
{
    ESP_ERROR_CHECK(ina219_get_bus_voltage(&i_sensor, &data->v_batt));
    ESP_ERROR_CHECK(ina219_get_current(&i_sensor, &data->i_batt));

    printf("v_batt: %.04f V, i_batt: %.04f mA\n", data->v_batt, data->i_batt * 1000);
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    master_init();
    wifi_init();
    init_bmp280();
    init_ina();

    // give the bmp time to settle
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    data_struct data;

    while (true)
    {
        measure_current(&data);
        measure_weather(&data);
        ESP_ERROR_CHECK(esp_wifi_start());
        esp_now_send(broadcastAddress, (uint8_t *)&data, sizeof(data_struct));
        ESP_ERROR_CHECK(esp_wifi_stop());
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
