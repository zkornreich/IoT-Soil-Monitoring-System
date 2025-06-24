/*
Zack Kornreich IoT Final Project

Using an ESP32-C6 to read from the F304 SHT3X soil moisture and temperature sensor, 
and transmitting that data via MQTT to the MQTT broker ran on a raspberry pi on the 
same network. A graphical interface may be accessed by any device on the shared network
via the node-RED dashboard hosted on the same Raspberry Pi.
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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "sht3x.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_pm.h"

#define INIT_DELAY 100                          // Delay for initializations
#define SLEEP_DELAY 10000                       // Sleep duration between readings in ms (10s)

#define MQTT_PUB_TEMP "esp32/temperature"       // MQTT Topic for Temperature
#define MQTT_PUB_HUM "esp32/humidity"           // MQTT Topic for Humidity
#define MQTT_URL "mqtt://192.168.40.180:1883"   // MQTT Broker IP/Port
#define EXAMPLE_ESP_WIFI_SSID "Edu Is Roaming"  // Wi-Fi Network SSID
#define EXAMPLE_ESP_WIFI_PASS "strh6116"        // Wi_Fi Network Password
#define MAX_RETRY 10                            // Maximum Wi-Fi retry attempts after losing connection

//Tags for debug logs
static const char *SENSORS_TAG = "sensors";
static const char *TAG = "MQTT_EXAMPLE";

char scale = SCALE_FAHRENHEIT;                  // Configure temperature readings to fahrenheit
static int retry_cnt = 0;                       // Initialize Wi-Fi connection retry counter

// I2C Initialization for SHT3x sensor communication
void i2c_master_init()
{
	i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, i2c_config.mode,
                    I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));

    esp_log_level_set(SENSORS_TAG, ESP_LOG_INFO);

    #if defined(SENSORS_SCALE_F)
    scale = SCALE_FAHRENHEIT;
    #elif defined(SENSORS_SCALE_K)
    scale = SCALE_KELVIN;
    #endif

}

// MQTT initialization and configuration
uint32_t MQTT_CONNECTED = 0;                        // Initialize MQTT connection state to not-connected
static void mqtt_app_start(void);                   // Declare function to be implemented later for MQTT start
                                                        // Declaration necessary as it is called by the WiFi Event Handler 
                                                        //after a succesful internet connection is made

// Wi-Fi event handler responds to changes in connection status to either attempt a connection, start MQTT services, or respond to disconnection
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                    int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        esp_wifi_connect();
        ESP_LOGI(TAG, "Trying to connect with Wi-Fi\n");
        break;

    case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGI(TAG, "Wi-Fi connected\n");
        break;

    case IP_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip: startibg MQTT Client\n");
        mqtt_app_start();
        break;

    case WIFI_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG, "disconnected: Retrying Wi-Fi\n");
        if (retry_cnt++ < MAX_RETRY)
        {
            esp_wifi_connect();
        }
        else
            ESP_LOGI(TAG, "Max Retry Failed: Wi-Fi Connection\n");
        break;

    default:
        break;
    }
}

// Initializes Wi-Fi chip on the ESP32-C6 using a Wi-Fi configuration and declaring a Wi-Fi handler to execute connections
void wifi_init(void)
{
    esp_event_loop_create_default();
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    esp_netif_init();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *  The function will monitor and respond to chanes in MQTT connection status.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "Event dispatched from event loop base=%s, event_id=%" PRId32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        MQTT_CONNECTED = 1;
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        MQTT_CONNECTED = 0;
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

// The MQTT client handler maintains the connection to the specified broker
esp_mqtt_client_handle_t client = NULL;

// This function starts up the MQTT connection to the Raspberry Pi MQTT broker.
    // This function is called after a Wi-Fi connection has been successfully established.
static void mqtt_app_start(void)
{
    ESP_LOGI(TAG, "STARTING MQTT");
	esp_mqtt_client_config_t mqttConfig = {
    .broker.address.uri = MQTT_URL };

    client = esp_mqtt_client_init(&mqttConfig);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

/* The Freertos library is used to register tasks that run in the background.
 * The Publisher_Task serves as our main loop for the project. There is an initial delay 
 * before starting the loop to allow for Wi-Fi and MQTT connections to configure. Then the loop
 * starts with sending a signal to start the sensor. The structure to read from the values is 
 * declared, and a delay is added let the sensor transmit new readings. We then read the measurements
 * from the expected registers and verify that there was no error with measurement reading. We then
 * load our local variables with the values from memory, and check scale requirements. We log the 
 * sensor data for debugging, and put the sensor back into the low power idle state.
 * 
 * The float values for humidity and temperature data are then translated into strings and published
 * to their respective MQTT Topics if the MQTT client handler indicates that we are connected to our
 * Raspberry Pi Broker.
 * 
 * Lastly, we enter a low power state before sleeping for 10 seconds. First, we save the processor 
 * configuration state. We then set the processor state into a low power mode with dramatically decreased
 * clock speeds enabling light sleep mode, and delay the task for 10 seconds. Then, the processor will
 * return to it's initial processor configuration, and the loop will repeat.
*/
void Publisher_Task(void *params)
{
    vTaskDelay(INIT_DELAY / portTICK_PERIOD_MS);
    while (true)
    {
        sht3x_start_periodic_measurement();

        sht3x_sensors_values_t sensors_values = {
            .temperature = 0x00,
            .humidity = 0x00
        };
        vTaskDelay(INIT_DELAY / portTICK_PERIOD_MS);

        if(sht3x_read_measurement(&sensors_values) != ESP_OK) {
            ESP_LOGE(SENSORS_TAG, "Sensors read measurement error!");
        }
        vTaskDelay(INIT_DELAY / portTICK_PERIOD_MS);

        float temperature = sensors_values.temperature;
        float humidity = sensors_values.humidity;

        #if defined(SENSORS_SCALE_F)
        temperature = FAHRENHEIT(temperature);
        #elif defined(SENSORS_SCALE_K)
        temperature = KELVIN(temperature);
        #endif

        ESP_LOG_BUFFER_HEX_LEVEL(SENSORS_TAG, &sensors_values, sizeof(sensors_values), ESP_LOG_DEBUG);

        sht3x_stop_periodic_measurement();

        ESP_LOGI(SENSORS_TAG, "Temperature %2.1f °%c - Humidity %2.1f%%", temperature, scale, humidity);

        char tempString[12];
        sprintf(tempString, "%.2f degC", temperature);

        char humidityString[12];
        sprintf(humidityString, "%.2f %%", humidity);
        
        if (MQTT_CONNECTED)
        {
            ESP_LOGI("PUB_TASK", "MQTT CONNECTED - Sending data");
            vTaskDelay(10 / portTICK_PERIOD_MS);
            esp_mqtt_client_publish(client, MQTT_PUB_TEMP, tempString, 0, 0, 0);
            esp_mqtt_client_publish(client, MQTT_PUB_HUM, humidityString, 0, 0, 0);
        } else {
            ESP_LOGI("PUB_TASK", "MQTT NOT CONNECTED...");
        }
                
        esp_pm_config_t power_management_disabled;
        esp_pm_get_configuration(&power_management_disabled);
        esp_pm_config_t power_management_enabled = {
            .max_freq_mhz = 160, // ref: the esp32-c6 datasheet https://www.espressif.com/sites/default/files/documentation/esp32-c6-wroom-1_wroom-1u_datasheet_en.pdf
            .min_freq_mhz = 10,  // ref: Espressive's itwt example: https://github.com/espressif/esp-idf/tree/903af13e847cd301e476d8b16b4ee1c21b30b5c6/examples/wifi/itwt
            .light_sleep_enable = true
        };
        ESP_LOGI("SLEEP_TASK", "SLEEP START");
        esp_pm_configure(&power_management_enabled);
        vTaskDelay(SLEEP_DELAY / portTICK_PERIOD_MS);
        esp_pm_configure(&power_management_disabled);
        ESP_LOGI("SLEEP_TASK", "SLEEP WAKE");
    }
}

// This is the main function that runs on processor powering up. Memory, Wi-Fi,
// and I2c is initialized, and the main project task is created to let run.
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    wifi_init();
    i2c_master_init();
    xTaskCreate(Publisher_Task, "Publisher_Task", 1024 * 5, NULL, 5, NULL);
}
