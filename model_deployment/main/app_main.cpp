/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "dl_tool.hpp"
#include "model_define.hpp"
#include "i2c_bus.h"
#include "mpu6050.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO 6
#define I2C_MASTER_SDA_IO 7
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000

i2c_bus_handle_t i2c_bus = NULL;
mpu6050_handle_t mpu6050 = NULL;

int input_height = 80;
int input_width = 3;
int input_channel = 1;
int input_exponent = -13;
float acc_xyz[240] = {0};
int index_acc = 0;

static const char *TAG = "MQTT_EXAMPLE";

void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

extern "C" void app_main(void)
{
    // config I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .clk_flags = 0,
    };
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);
    mpu6050 = mpu6050_create(i2c_bus, MPU6050_I2C_ADDRESS);
    uint8_t mp6050_deviceid;
    mpu6050_acce_value_t acce;

    mpu6050_get_deviceid(mpu6050, &mp6050_deviceid);
    printf("MPU6050 Device ID: %d\n", mp6050_deviceid);
    mpu6050_set_acce_fs(mpu6050, ACCE_FS_4G);

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    // esp_log_level_set("*", ESP_LOG_INFO);
    // esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    // esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    // esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    // esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    // esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    // esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    // start mqtt
    esp_mqtt_client_config_t mqtt_cfg = {
        // .uri = CONFIG_BROKER_URL,
        .uri = "mqtt://45.80.181.181",
        .username = "forback",
        .password = "forback2024"};
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);

    while (true)
    {
        for (int i = 0; i < 80; i++)
        {
            mpu6050_get_acce(mpu6050, &acce);
            acc_xyz[index_acc] = acce.acce_x;
            index_acc++;
            acc_xyz[index_acc] = acce.acce_y;
            index_acc++;
            acc_xyz[index_acc] = acce.acce_z;
            index_acc++;
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }

        index_acc = 0;
        int16_t *model_input = (int16_t *)dl::tool::malloc_aligned_prefer(input_height * input_width * input_channel, sizeof(int16_t *));
        for (int i = 0; i < input_height * input_width * input_channel; i++)
        {
            float normalized_input = acc_xyz[i] / 1.0;
            model_input[i] = (int16_t)DL_CLIP(normalized_input * (1 << -input_exponent), -32768, 32767);
        }

        Tensor<int16_t> input;
        input.set_element((int16_t *)model_input).set_exponent(input_exponent).set_shape({input_height, input_width, input_channel}).set_auto_free(false);
        ACTIVITY model;
        dl::tool::Latency latency;
        latency.start();
        model.forward(input);
        latency.end();
        latency.print("\nActivity model", "forward");
        float *score = model.l6.get_output().get_element_ptr();
        float max_score = score[0];
        int max_index = 0;
        for (size_t i = 0; i < 6; i++)
        {
            printf("%f, ", score[i] * 100);
            if (score[i] > max_score)
            {
                max_score = score[i];
                max_index = i;
            }
        }
        printf("\n");

        switch (max_index)
        {
        case 0:
            esp_mqtt_client_publish(client, "/joki-despro/activity", "Downstairs", 0, 0, 0);
            printf("0: Downstairs");
            break;
        case 1:
            esp_mqtt_client_publish(client, "/joki-despro/activity", "Jogging", 0, 0, 0);
            printf("1: Jogging");
            break;
        case 2:
            esp_mqtt_client_publish(client, "/joki-despro/activity", "Sitting", 0, 0, 0);
            printf("2: Sitting");
            break;
        case 3:
            esp_mqtt_client_publish(client, "/joki-despro/activity", "Standing", 0, 0, 0);
            printf("3: Standing");
            break;
        case 4:
            esp_mqtt_client_publish(client, "/joki-despro/activity", "Upstairs", 0, 0, 0);
            printf("4: Upstairs");
            break;
        case 5:
            esp_mqtt_client_publish(client, "/joki-despro/activity", "Walking", 0, 0, 0);
            printf("5: Walking");
            break;
        default:
            printf("No result");
        }
        printf("\n");
    }
}
