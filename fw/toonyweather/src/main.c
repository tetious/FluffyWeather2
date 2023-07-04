#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <bme680.h>
#include <string.h>
#include <esp_sleep.h>
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include <ina3221.h>
#include <esp_log.h>
#include <esp_pm.h>
#include "driver/gpio.h"
#include "../../shared.h"
#include <driver/adc.h>

#define SDA_GPIO 15
#define SCL_GPIO 2

#define WIND_DIR_PIN 25
#define WIND_SPD_PIN 32
#define RAIN_PIN 12
#define STAT1_PIN 26
#define STAT2_PIN 27

const static char *TAG = "FluffyWeather";

static xQueueHandle gpio_evt_queue = NULL;
volatile TickType_t last_hit[40] = {};
uint8_t broadcastAddress[] = {0x5C, 0xCF, 0x7F, 0x1B, 0xE0, 0x89};

ina3221_t i_sensor = {
    .shunt = {100, 100, 100}, // shunt values are 100 mOhm for each channel
    .config.config_register = INA3221_DEFAULT_CONFIG,
    .mask.mask_register = INA3221_DEFAULT_MASK};
bme680_t bmp_sensor;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    TickType_t now = xTaskGetTickCountFromISR();
    uint32_t gpio_num = (uint32_t)arg;
    if (now - last_hit[gpio_num] > 15)
    {
        xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
        last_hit[gpio_num] = now;
    }
}

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
    gpio_evt_queue = xQueueCreate(50, sizeof(uint32_t));

    esp_pm_config_esp32_t pm_config = {
        .max_freq_mhz = 80,
        .min_freq_mhz = 40,
        .light_sleep_enable = false};
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));

    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = ((1ULL << STAT1_PIN) |
                            (1ULL << STAT2_PIN));
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.pull_up_en = true;
    io_conf.pull_down_en = false;
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << WIND_DIR_PIN);
    io_conf.pull_up_en = false;
    io_conf.pull_down_en = false;
    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = ((1ULL << WIND_SPD_PIN) |
                            (1ULL << RAIN_PIN));
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pull_up_en = true;
    io_conf.pull_down_en = false;
    gpio_config(&io_conf);

    adc2_config_channel_atten(ADC2_CHANNEL_8, ADC_ATTEN_DB_11);

    gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LOWMED);
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());

    gpio_isr_handler_add(WIND_SPD_PIN, gpio_isr_handler, (void *)WIND_SPD_PIN);
    gpio_isr_handler_add(RAIN_PIN, gpio_isr_handler, (void *)RAIN_PIN);
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

uint32_t duration;

void init_bme680()
{
    memset(&bmp_sensor, 0, sizeof(bme680_t));

    ESP_ERROR_CHECK(bme680_init_desc(&bmp_sensor, BME680_I2C_ADDR_0, 0, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(bme680_init_sensor(&bmp_sensor));
    bme680_set_oversampling_rates(&bmp_sensor, BME680_OSR_8X, BME680_OSR_4X, BME680_OSR_2X);
    bme680_set_filter_size(&bmp_sensor, BME680_IIR_SIZE_3);

    //bme680_set_heater_profile(&bmp_sensor, 0, 200, 100);
    //bme680_use_heater_profile(&bmp_sensor, 0);

    //bme680_set_ambient_temperature(&bmp_sensor, 25);

    bme680_get_measurement_duration(&bmp_sensor, &duration);
    ESP_LOGI(TAG, "duration: %i", duration);
}

void measure_weather(data_struct *data)
{
    bme680_values_float_t values;

    if (bme680_force_measurement(&bmp_sensor) == ESP_OK)
    {
        vTaskDelay(duration);

        if (bme680_get_results_float(&bmp_sensor, &values) == ESP_OK)
        {
            data->pressure = values.pressure;
            data->temp = values.temperature;
            data->humidity = values.humidity;
            data->gas_resistance = values.gas_resistance;
            printf("BME680 Sensor: %.2f °C, %.2f %%, %.2f hPa, %.2f Ohm\n",
                   values.temperature, values.humidity, values.pressure, values.gas_resistance);
        }
    }
}

void init_ina()
{
    ESP_ERROR_CHECK(ina3221_init_desc(&i_sensor, INA3221_I2C_ADDR_GND, 0, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(ina3221_set_options(&i_sensor, true, true, true));
    ESP_ERROR_CHECK(ina3221_enable_channel(&i_sensor, true, true, true));
    ESP_ERROR_CHECK(ina3221_set_average(&i_sensor, INA3221_AVG_512));
    ESP_ERROR_CHECK(ina3221_set_bus_conversion_time(&i_sensor, INA3221_CT_1100));
    ESP_ERROR_CHECK(ina3221_set_shunt_conversion_time(&i_sensor, INA3221_CT_1100));
}

void measure_current(data_struct *data)
{
    float shunt_voltage;
    ESP_ERROR_CHECK(ina3221_get_bus_voltage(&i_sensor, 0, &data->v_solar));
    ESP_ERROR_CHECK(ina3221_get_shunt_value(&i_sensor, 0, &shunt_voltage, &data->i_solar));
    printf("SOLAR: %.04f V, %.04f mA\n", data->v_solar, data->i_solar);
    ESP_ERROR_CHECK(ina3221_get_bus_voltage(&i_sensor, 1, &data->v_batt));
    ESP_ERROR_CHECK(ina3221_get_shunt_value(&i_sensor, 1, &shunt_voltage, &data->i_batt));
    printf("BATT: %.04f V, %.04f mA\n", data->v_batt, data->i_batt);
    ESP_ERROR_CHECK(ina3221_get_bus_voltage(&i_sensor, 2, &data->v_load));
    ESP_ERROR_CHECK(ina3221_get_shunt_value(&i_sensor, 2, &shunt_voltage, &data->i_load));
    printf("LOAD: %.04f V, %.04f mA\n", data->v_load, data->i_load);
}

void task(void *parameter)
{
    ESP_LOGI(TAG, "TASK core: %i", xPortGetCoreID());

    while (true)
    {
        data_struct data = {};

        ESP_ERROR_CHECK(gpio_wakeup_enable(WIND_SPD_PIN, GPIO_INTR_LOW_LEVEL));
        ESP_ERROR_CHECK(gpio_wakeup_enable(RAIN_PIN, GPIO_INTR_LOW_LEVEL));

        measure_current(&data);
        measure_weather(&data);

        adc2_get_raw(ADC2_CHANNEL_8, ADC_WIDTH_12Bit, &data.raw_wind_dir);
        data.uptime = esp_timer_get_time();
        
        uint32_t io_num = 0;
        while (xQueueReceive(gpio_evt_queue, &io_num, 0))
        {
            if (io_num == RAIN_PIN)
                data.rain_count++;
            if (io_num == WIND_SPD_PIN)
                data.wind_speed_count++;
        }

        printf("r: %i, ws: %i, wd:%i\n", data.rain_count, data.wind_speed_count, data.raw_wind_dir);
        printf("PINS: s1:%i, s2:%i\n", gpio_get_level(STAT1_PIN), gpio_get_level(STAT2_PIN));
        printf("Uptime: %lli\n", data.uptime);

        ESP_ERROR_CHECK(esp_wifi_start());
        esp_now_send(broadcastAddress, (uint8_t *)&data, sizeof(data_struct));
        ESP_ERROR_CHECK(esp_wifi_stop());

        //vTaskDelay(pdMS_TO_TICKS(5000));
        int64_t wakeup_time = esp_timer_get_time() + (5 * 1000000);
        esp_sleep_enable_timer_wakeup(wakeup_time - esp_timer_get_time());
        esp_light_sleep_start();
        // // if a sensor wakes us, just sleep again until the timer goes off.
        while (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_TIMER)
        {
            vTaskDelay(100);
            if (gpio_get_level(WIND_SPD_PIN) == 0)
            {
                ESP_ERROR_CHECK(gpio_wakeup_enable(WIND_SPD_PIN, GPIO_INTR_HIGH_LEVEL));
            }
            else
            {
                ESP_ERROR_CHECK(gpio_wakeup_enable(WIND_SPD_PIN, GPIO_INTR_LOW_LEVEL));
            }
            if (gpio_get_level(RAIN_PIN) == 0)
            {
                ESP_ERROR_CHECK(gpio_wakeup_enable(RAIN_PIN, GPIO_INTR_HIGH_LEVEL));
            }
            else
            {
                ESP_ERROR_CHECK(gpio_wakeup_enable(RAIN_PIN, GPIO_INTR_LOW_LEVEL));
            }
            int64_t time_left_to_sleep = wakeup_time - esp_timer_get_time();
            if(time_left_to_sleep >= 0)
            {
                ESP_LOGI(TAG, "Sleeping for %lli more us.", time_left_to_sleep);
                esp_sleep_enable_timer_wakeup(time_left_to_sleep);
                esp_light_sleep_start();
            } else {
                break;
            }
        }

    }
}

void app_main()
{
    ESP_LOGI(TAG, "MAIN core: %i", xPortGetCoreID());

    ESP_ERROR_CHECK(i2cdev_init());
    master_init();
    wifi_init();
    init_bme680();
    init_ina();

    // give the bmp time to settle
    vTaskDelay(pdMS_TO_TICKS(5000));

    TaskHandle_t main_task;
    xTaskCreatePinnedToCore(task, "main", 10000, NULL, 0, &main_task, 1);
}
