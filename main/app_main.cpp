#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dl_tool.hpp"
#include "model_define.hpp"
#include "i2c_bus.h"
#include "mpu6050.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "mqtt_client.h"

#include "esp_spi_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "protocol_examples_common.h"

#define GPS_UART_NUM UART_NUM_2
#define GPS_TX_PIN (GPIO_NUM_17)
#define GPS_RX_PIN (GPIO_NUM_16)
#define BUF_SIZE (1024)

static esp_mqtt_client_handle_t client;

int input_height = 80;
int input_width = 3;
int input_channel = 1;
int input_exponent = -13;
float acc_xyz[240] = {0};
int index_acc=0;

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO 6      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 7      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000 /*!< I2C master clock frequency */

static i2c_bus_handle_t i2c_bus = NULL;
static mpu6050_handle_t mpu6050 = NULL;

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/topic/crsystal1", "MQTT Connected", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/crsystal0", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/crsystal1", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/crsystal1");
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        // ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        // msg_id = esp_mqtt_client_publish(client, "/topic/crsystal0", "MQTT Subscribed", 0, 0, 0);
        // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,   
    };

    #if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

// void sendDataToMQTTTask(void *pvParameters)
// {
//     while (1)
//     {
//         ESP_LOGI(TAG, "Sending data to MQTT");
        
//         // Send data to MQTT
//         int msg_id = esp_mqtt_client_publish(client, "/topic/crsystal1", "test function data_3 tes 0912873", 0, 1, 0);
//         ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

//         vTaskDelay(5000 / portTICK_PERIOD_MS);
//     }
// }

void sendDataToMQTTTask(void *pvParameters)
{
    // Define the topic
    char *topic = "/topic/crsystal1";

    // Get data from Param
    char *data = (char *)pvParameters;

    ESP_LOGI(TAG, "Sending data to MQTT from Task: %s", data);
    
    // Send data to MQTT
    int msg_id = esp_mqtt_client_publish(client, topic, data, 0, 1, 0);
    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

    vTaskDelete(NULL);
}

void uart_init() {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(GPS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(GPS_UART_NUM, &uart_config);
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// // function to read GPS data and return it
// char *read_gps_data() {
//     uint8_t data[BUF_SIZE];
//     int length = 0;

//     length = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE, 20 / portTICK_RATE_MS);
//     if (length > 0) {
//         data[length] = '\0';  // Null-terminate the string
//         return (char *)data;  // Return NMEA data
//     }
//     else {
//         return NULL;
//     }
// }

void gps_read_data() {
    printf("GPS data reading task started in task\n");
    uint8_t data[BUF_SIZE];
    int length = 0;

    while (1) {
        printf("Inside loop of GPS data reading task\n");
        length = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        if (length > 0) {
            data[length] = '\0';  // Null-terminate the string
            printf("%s", (char *)data);  // Print NMEA data to console
        }
        else {
            printf("No data received from GPS\n");
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Send Coordinates to MQTT Task
 * @author Amamya
 * @param pvParameters
 * @return void
 * 
 * This function is used to send the coordinates to the MQTT broker
 * This function is called Everytime an event is triggered
 * When this function is called, it will read the coordinates from the GPS module and send it to the MQTT broker
 */
void sendCoordinatesToMQTTTask(void *pvParameters)
{
    // Define the topic
    char *topic = "/topic/crsystal1";

    // Get data from function
    // char *data = read_gps_data(); // memalsukan data
    char *data = "{Latitude: -6.362352, Longitude: 106.824207}"; // data yang dipalsukan

    //wait until wifi is connected
    while (esp_wifi_connect() != ESP_OK)
    {
        printf("Waiting for wifi to connect...\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Send data to MQTT
    int msg_id = esp_mqtt_client_publish(client, topic, data, 0, 1, 0);

    // Log the data
    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

    // Delete the task
    vTaskDelete(NULL);
}


extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    //configuring the mqtt client
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
    };

    //initialize the mqtt client
    client = esp_mqtt_client_init(&mqtt_cfg);

    //register the event handler
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    mqtt_app_start();

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
    uint8_t mpu6050_deviceid;
    mpu6050_acce_value_t acce;
    // int cnt = 10;
    mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
    printf("mpu6050 device ID is: 0x%02x\n", mpu6050_deviceid);
    // mpu6050_wake_up(mpu6050);
    mpu6050_set_acce_fs(mpu6050, ACCE_FS_4G);
    // mpu6050_set_gyro_fs(mpu6050, GYRO_FS_500DPS);

    // send gps data to mqtt 3 times
    for (int i = 0; i < 3; i++)
    {
        // create a task to send data to mqtt
        xTaskCreate(sendDataToMQTTTask, "sendDataToMQTTTask", 4096, "test function data_3 tes 0912873", 5, NULL);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }   
        
while(1){

for (int i=0 ;i<80; i++)
{

    mpu6050_get_acce(mpu6050, &acce);
    // printf("acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce.acce_x, acce.acce_y, acce.acce_z);
    
    // acc_xyz[i]={acc_value.raw_acce_x,acc_value.raw_acce_y,acc_value.raw_acce_z};
    acc_xyz[index_acc]=acce.acce_x;
    index_acc=index_acc+1;
    acc_xyz[index_acc]=acce.acce_y;
    index_acc=index_acc+1;
    acc_xyz[index_acc]=acce.acce_z;
    index_acc=index_acc+1;
    // ESP_LOGI(TAG, "%f\n",acc_xyz[i]);
    vTaskDelay(50 / portTICK_RATE_MS);
    // printf("%d",portTICK_RATE_MS);
}
// printf("The value of j %d\n",j);
index_acc=0;

int16_t *model_input = (int16_t *)dl::tool::malloc_aligned_prefer(input_height*input_width*input_channel, sizeof(int16_t *));
    for(int i=0 ;i<input_height*input_width*input_channel; i++){
        float normalized_input = acc_xyz[i] / 1.0; //normalization
        model_input[i] = (int16_t)DL_CLIP(normalized_input * (1 << -input_exponent), -32768, 32767);
    } 

Tensor<int16_t> input;
                input.set_element((int16_t *) model_input).set_exponent(input_exponent).set_shape({input_height,input_width,input_channel}).set_auto_free(false);
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
                    printf("%f, ", score[i]*100);
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
                    printf("0: Downstairs");
                    break;
                    case 1:
                    printf("1: Jogging");
                    break;
                    case 2:
                    printf("2: Sitting");
                    break;
                    case 3:
                    printf("3: Standing");
                    break;
                    case 4:
                    printf("4: Upstairs");
                    break;
                    case 5:
                    printf("5: Walking");
                    break;
                    default:
                    printf("No result");

                }
                printf("\n");

}
}