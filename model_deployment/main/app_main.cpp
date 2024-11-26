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


#include "esp_spi_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "PulseSensorPlayground.h"

// For GPS
#define GPS_UART_NUM UART_NUM_2
#define GPS_TX_PIN (GPIO_NUM_17)
#define GPS_RX_PIN (GPIO_NUM_16)
#define BUF_SIZE (1024)

// For Act
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

typedef struct ActivityData
{
    int activity_index;
    char activity_label[50];
} ActData;

static QueueHandle_t activityQueue;

void uart_init()
{
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(GPS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(GPS_UART_NUM, &uart_config);
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

void free_and_log(void *ptr)
{
    if (ptr != NULL)
    {
        // Free the allocated memory
        vPortFree(ptr);
        ESP_LOGI(TAG, "Free heap size: %d bytes", esp_get_free_heap_size());
    }
    else
    {
        ESP_LOGW(TAG, "Attempted to free a NULL pointer");
    }
}

void activity_detection_task(void *pvParameters)
{
    bool was_sitting = false;
    while (true)
    {
        int16_t *model_input = (int16_t *)dl::tool::malloc_aligned_prefer(input_height * input_width * input_channel, sizeof(int16_t *));
        Tensor<int16_t> input;
        index_acc = 0;

        for (int i = 0; i < input_height * input_width * input_channel; i++)
        {
            float normalized_input = acc_xyz[i] / 1.0;
            model_input[i] = (int16_t)DL_CLIP(normalized_input * (1 << -input_exponent), -32768, 32767);
        }

        input.set_element((int16_t *)model_input)
            .set_exponent(input_exponent)
            .set_shape({input_height, input_width, input_channel})
            .set_auto_free(false);

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
            if (score[i] > max_score)
            {
                max_score = score[i];
                max_index = i;
            }
        }

        ActData *ptr = (ActData *)pvPortMalloc(sizeof(ActData)); // Allocate memory for ActData
        if (ptr == NULL)
        {
            ESP_LOGE(TAG, "Failed to allocate memory for activity data");
            continue;
        }

        ptr->activity_index = max_index;

        switch (max_index)
        {
        case 0:
            strncpy(ptr->activity_label, "Downstairs", sizeof(ptr->activity_label));
            break;
        case 1:
            strncpy(ptr->activity_label, "Jogging", sizeof(ptr->activity_label));
            break;
        case 2:
            strncpy(ptr->activity_label, "Sitting", sizeof(ptr->activity_label));
            was_sitting = true;
            break;
        case 3:
            strncpy(ptr->activity_label, "Standing", sizeof(ptr->activity_label));
            was_sitting = false;
            break;
        case 4:
            strncpy(ptr->activity_label, "Upstairs", sizeof(ptr->activity_label));
            was_sitting = false;
            break;
        case 5:
            strncpy(ptr->activity_label, "Walking", sizeof(ptr->activity_label));
            was_sitting = false;
            break;
        default:
            strncpy(ptr->activity_label, "No Result", sizeof(ptr->activity_label));
            was_sitting = false;
            break;
        }

        if (uxQueueSpacesAvailable(activityQueue) > 0) // Check if the queue has space
        {
            if (xQueueSend(activityQueue, &ptr, portMAX_DELAY) != pdPASS)
            {
                ESP_LOGE(TAG, "Failed to send activity data to queue");
                free_and_log(ptr); // Free the allocated memory if sending to the queue fails
            }
            else
            {
                ESP_LOGI(TAG, "Sent activity data to queue: %s", ptr->activity_label);
            }
        }
        else
        {
            ESP_LOGW(TAG, "Queue full. Dropping activity data: %s", ptr->activity_label);
            free_and_log(ptr);
        }

        if (was_sitting && max_index == 0) // Detected a fall from sitting to lying down
        {
            ActData *fell = (ActData *)pvPortMalloc(sizeof(ActData)); // Allocate memory for ActData
            if (fell == NULL)
            {
                ESP_LOGE(TAG, "Failed to allocate memory for activity data");
                continue;
            }
            fell->activity_index = 7;
            strncpy(fell->activity_label, "Falling Detected", sizeof(fell->activity_label));

            if (xQueueSend(activityQueue, &fell, portMAX_DELAY) != pdPASS)
            {
                ESP_LOGE(TAG, "Failed to send activity data to queue");
                free_and_log(fell); // Free the allocated memory if sending to the queue fails
            }
            else
            {
                ESP_LOGI(TAG, "Sent activity data to queue: %s", fell->activity_label);
            }
        }

        // Free the memory
        dl::tool::free_aligned(model_input);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void ActivityMQTTTask(void *pvParameters)
{
    ActData *ptr;
    char topic[30];
    while (true)
    {
        esp_mqtt_client_config_t mqtt_cfg = {
            .uri = "mqtt://45.80.181.181",
            .username = "forback",
            .password = "forback2024"};
        esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
        esp_mqtt_client_start(client);
        
        if (xQueueReceive(activityQueue, &ptr, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            char *data = ptr->activity_label;
            ESP_LOGI(TAG, "Sending data to MQTT from Task: %s", data);
            if (ptr->activity_index == 7)
            {
                strcpy(topic, "/joki-despro/fall");
            }
            else
            {
                strcpy(topic, "/joki-despro/activity");
            }
            // Send data to MQTT
            int msg_id = esp_mqtt_client_publish(client, topic, data, 0, 1, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            esp_mqtt_client_destroy(client);
            free_and_log(ptr);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Error handling for task
    // vTaskDelete(NULL);
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    activityQueue = xQueueCreate(10, (sizeof(ActData)));
    
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://45.80.181.181",
        .username = "forback",
        .password = "forback2024"};
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);

    BaseType_t ActivityTaskCreation = xTaskCreate(
        activity_detection_task,
        "ActivityDetectionTask",
        8192,
        (void *)client,
        5,
        NULL);

    if (ActivityTaskCreation == pdPASS)
    {
        printf("Task 'ActivityDetectionTask' successfully created.\n");
    }
    else
    {
        printf("Failed to create task 'ActivityDetectionTask'.\n");
    }

    BaseType_t mqttTaskCreation = xTaskCreate(
        ActivityMQTTTask,
        "ActivityMQTTTask",
        8192,
        (void *)client,
        5,
        NULL);

    if (mqttTaskCreation == pdPASS)
    {
        ESP_LOGI(TAG, "Task 'ActivityMQTTTask' successfully created.");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to create task 'ActivityMQTTTask'.");
        return;
    }
}
