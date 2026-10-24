#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "common/mavlink.h"
#include "imu.h"

// ============================================================
// CONFIGURATION
// ============================================================

#define MAV_UART        UART_NUM_0
#define MAV_BAUD_RATE   115200

#define MAV_SYSTEM_ID   1
#define MAV_COMPONENT_ID MAV_COMP_ID_AUTOPILOT1

#define NVS_NAMESPACE   "fc_params"

static const char *TAG = "FC";
// Current flight-controller system state
static uint8_t fc_system_state = MAV_STATE_BOOT;

// ============================================================
// PARAMETER STRUCTURE
// ============================================================

typedef struct
{
    char name[17];
    float value;

} parameter_t;


// ============================================================
// PARAMETERS
// ============================================================

static parameter_t parameters[] =
{
    {"SYSID_THISMAV",  1.0f},
    {"COMPASS_ENABLE", 1.0f},
    {"INS_ENABLE",     1.0f},
    {"FRAME_CLASS",    1.0f},
    {"FRAME_TYPE",     1.0f},
    {"AHRS_ENABLE",    1.0f},
};

#define PARAM_COUNT (sizeof(parameters) / sizeof(parameters[0]))


// ============================================================
// NVS INITIALIZATION
// ============================================================

static void init_nvs(void)
{
    esp_err_t err = nvs_flash_init();

    /*
     * NVS partition may need to be erased if:
     * - there are no free pages
     * - NVS version has changed
     */
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS needs erase. Erasing...");

        ESP_ERROR_CHECK(nvs_flash_erase());

        err = nvs_flash_init();
    }

    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "NVS initialized");
}


// ============================================================
// LOAD PARAMETERS FROM NVS
// ============================================================

static void load_parameters_from_nvs(void)
{
    nvs_handle_t handle;

    esp_err_t err = nvs_open(
        NVS_NAMESPACE,
        NVS_READONLY,
        &handle
    );

    if (err != ESP_OK)
    {
        ESP_LOGW(
            TAG,
            "Could not open NVS for reading: %s",
            esp_err_to_name(err)
        );

        return;
    }


    for (uint16_t i = 0; i < PARAM_COUNT; i++)
    {
        float saved_value = 0.0f;

        size_t size = sizeof(float);

        err = nvs_get_blob(
            handle,
            parameters[i].name,
            &saved_value,
            &size
        );


        if (err == ESP_OK && size == sizeof(float))
        {
            parameters[i].value = saved_value;

            ESP_LOGI(
                TAG,
                "Loaded %s = %.3f from NVS",
                parameters[i].name,
                parameters[i].value
            );
        }
        else if (err == ESP_ERR_NVS_NOT_FOUND)
        {
            ESP_LOGI(
                TAG,
                "No saved value for %s, using default %.3f",
                parameters[i].name,
                parameters[i].value
            );
        }
        else
        {
            ESP_LOGW(
                TAG,
                "Error loading %s: %s",
                parameters[i].name,
                esp_err_to_name(err)
            );
        }
    }


    nvs_close(handle);
}


// ============================================================
// SAVE ONE PARAMETER TO NVS
// ============================================================

static esp_err_t save_parameter_to_nvs(uint16_t index)
{
    if (index >= PARAM_COUNT)
    {
        return ESP_ERR_INVALID_ARG;
    }


    nvs_handle_t handle;

    esp_err_t err = nvs_open(
        NVS_NAMESPACE,
        NVS_READWRITE,
        &handle
    );

    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "NVS open failed: %s",
            esp_err_to_name(err)
        );

        return err;
    }


    // Store the float value
    err = nvs_set_blob(
        handle,
        parameters[index].name,
        &parameters[index].value,
        sizeof(float)
    );

    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "NVS set failed for %s: %s",
            parameters[index].name,
            esp_err_to_name(err)
        );

        nvs_close(handle);

        return err;
    }


    // IMPORTANT:
    // NVS changes are not guaranteed to be stored
    // in flash until nvs_commit() is called.

    err = nvs_commit(handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "NVS commit failed for %s: %s",
            parameters[index].name,
            esp_err_to_name(err)
        );

        nvs_close(handle);

        return err;
    }


    ESP_LOGI(
        TAG,
        "Saved %s = %.3f to NVS",
        parameters[index].name,
        parameters[index].value
    );


    nvs_close(handle);

    return ESP_OK;
}


// ============================================================
// FIND PARAMETER
// ============================================================

static int find_parameter(const char *name)
{
    for (uint16_t i = 0; i < PARAM_COUNT; i++)
    {
        if (strcmp(parameters[i].name, name) == 0)
        {
            return i;
        }
    }

    return -1;
}


// ============================================================
// SEND ONE PARAMETER
// ============================================================

static void send_parameter(uint16_t index)
{
    if (index >= PARAM_COUNT)
    {
        return;
    }


    mavlink_message_t msg;

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];


    mavlink_msg_param_value_pack(
        MAV_SYSTEM_ID,
        MAV_COMPONENT_ID,

        &msg,

        parameters[index].name,

        parameters[index].value,

        MAV_PARAM_TYPE_REAL32,

        PARAM_COUNT,

        index
    );


    uint16_t length = mavlink_msg_to_send_buffer(
        buffer,
        &msg
    );


    uart_write_bytes(
        MAV_UART,
        buffer,
        length
    );
}


// ============================================================
// SEND ALL PARAMETERS
// ============================================================

static void send_all_parameters(void)
{
    ESP_LOGI(
        TAG,
        "Sending %d parameters",
        (int)PARAM_COUNT
    );


    for (uint16_t i = 0; i < PARAM_COUNT; i++)
    {
        send_parameter(i);

        /*
         * Small delay so Mission Planner has enough
         * time to receive the parameter stream.
         */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


// ============================================================
// SEND HEARTBEAT
// ============================================================

static void send_heartbeat(void)
{
    mavlink_message_t msg;

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];


    mavlink_msg_heartbeat_pack(
        MAV_SYSTEM_ID,
        MAV_COMPONENT_ID,

        &msg,

        MAV_TYPE_QUADROTOR,

        MAV_AUTOPILOT_ARDUPILOTMEGA,

        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,

        0,
        fc_system_state
    );


    uint16_t length = mavlink_msg_to_send_buffer(
        buffer,
        &msg
    );


    uart_write_bytes(
        MAV_UART,
        buffer,
        length
    );
}


// ============================================================
// PROCESS MAVLINK MESSAGE
// ============================================================

static void process_mavlink_message(
    mavlink_message_t *msg
)
{
    switch (msg->msgid)
    {

        // ====================================================
        // PARAM_REQUEST_LIST
        // ====================================================

        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        {
            mavlink_param_request_list_t request;

            mavlink_msg_param_request_list_decode(
                msg,
                &request
            );


            /*
             * Target system:
             *
             * 0 = broadcast
             * MAV_SYSTEM_ID = this FC
             */

            if (request.target_system == 0 ||
                request.target_system == MAV_SYSTEM_ID)
            {
                ESP_LOGI(
                    TAG,
                    "PARAM_REQUEST_LIST received"
                );

                send_all_parameters();
            }

            break;
        }


        // ====================================================
        // PARAM_REQUEST_READ
        // ====================================================

        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        {
            mavlink_param_request_read_t request;

            mavlink_msg_param_request_read_decode(
                msg,
                &request
            );


            if (request.target_system != 0 &&
                request.target_system != MAV_SYSTEM_ID)
            {
                break;
            }


            /*
             * If param_index >= 0:
             * Mission Planner requested parameter by index.
             */

            if (request.param_index >= 0 &&
                request.param_index < PARAM_COUNT)
            {
                ESP_LOGI(
                    TAG,
                    "PARAM_REQUEST_READ index %d",
                    request.param_index
                );

                send_parameter(request.param_index);
            }


            /*
             * Otherwise Mission Planner requested
             * parameter by name.
             */

            else
            {
                char name[17];

                memcpy(
                    name,
                    request.param_id,
                    16
                );

                name[16] = '\0';


                int index = find_parameter(name);


                if (index >= 0)
                {
                    ESP_LOGI(
                        TAG,
                        "PARAM_REQUEST_READ %s",
                        name
                    );

                    send_parameter(index);
                }
                else
                {
                    ESP_LOGW(
                        TAG,
                        "Parameter not found: %s",
                        name
                    );
                }
            }

            break;
        }


        // ====================================================
        // PARAM_SET
        // ====================================================

        case MAVLINK_MSG_ID_PARAM_SET:
        {
            mavlink_param_set_t request;

            mavlink_msg_param_set_decode(
                msg,
                &request
            );


            /*
             * Check target system.
             */

            if (request.target_system != MAV_SYSTEM_ID)
            {
                break;
            }


            /*
             * Extract parameter name.
             */

            char name[17];

            memcpy(
                name,
                request.param_id,
                16
            );

            name[16] = '\0';


            /*
             * Find parameter.
             */

            int index = find_parameter(name);


            if (index < 0)
            {
                ESP_LOGW(
                    TAG,
                    "PARAM_SET: unknown parameter %s",
                    name
                );

                break;
            }


            // =================================================
            // UPDATE RAM
            // =================================================

            parameters[index].value =
                request.param_value;


            ESP_LOGI(
                TAG,
                "PARAM_SET: %s = %.3f",
                parameters[index].name,
                parameters[index].value
            );


            // =================================================
            // SAVE TO NVS
            // =================================================

            esp_err_t save_result =
                save_parameter_to_nvs(index);


            if (save_result != ESP_OK)
            {
                ESP_LOGE(
                    TAG,
                    "WARNING: parameter changed in RAM "
                    "but NVS save failed!"
                );
            }


            // =================================================
            // SEND UPDATED PARAMETER BACK
            // =================================================

            send_parameter(index);


            break;
        }


        default:
        {
            /*
             * Other MAVLink messages are ignored for now.
             */

            break;
        }
    }
}


// ============================================================
// RECEIVE MAVLINK
// ============================================================

static void receive_mavlink(void)
{
    uint8_t byte;

    mavlink_message_t msg;

    mavlink_status_t status;


    /*
     * Read all available UART bytes.
     */

    while (uart_read_bytes(
        MAV_UART,
        &byte,
        1,
        0
    ) > 0)
    {

        /*
         * Feed each byte into MAVLink parser.
         */

        if (mavlink_parse_char(
            MAVLINK_COMM_0,
            byte,
            &msg,
            &status
        ))
        {
            /*
             * Complete MAVLink packet received.
             */

            process_mavlink_message(&msg);
        }
    }
}


// ============================================================
// UART INITIALIZATION
// ============================================================

static void init_uart(void)
{
    const uart_config_t uart_config =
    {
        .baud_rate = MAV_BAUD_RATE,

        .data_bits = UART_DATA_8_BITS,

        .parity = UART_PARITY_DISABLE,

        .stop_bits = UART_STOP_BITS_1,

        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,

        .source_clk = UART_SCLK_DEFAULT,
    };


    ESP_ERROR_CHECK(
        uart_param_config(
            MAV_UART,
            &uart_config
        )
    );


    ESP_ERROR_CHECK(
        uart_driver_install(
            MAV_UART,
            4096,
            4096,
            0,
            NULL,
            0
        )
    );


    ESP_LOGI(
        TAG,
        "UART initialized at %d baud",
        MAV_BAUD_RATE
    );
}


// ============================================================
// MAIN
// ============================================================

void app_main(void)
{
    ESP_LOGI(
        TAG,
        "========================================"
    );

    ESP_LOGI(
        TAG,
        "ESP32 Flight Controller Starting"
    );

    ESP_LOGI(
        TAG,
        "========================================"
    );


    // ========================================================
    // INITIALIZE NVS
    // ========================================================

    init_nvs();


    // ========================================================
    // LOAD SAVED PARAMETERS
    // ========================================================

    load_parameters_from_nvs();


    // ========================================================
    // INITIALIZE UART
    // ========================================================

    init_uart();

    vTaskDelay(pdMS_TO_TICKS(500));

    // Basic software initialization completed.
    // Sensors and flight-control logic are not implemented yet.
    fc_system_state = MAV_STATE_STANDBY;

    ESP_LOGI(
        TAG,
        "Basic initialization complete; state = STANDBY"
    );
    esp_err_t imu_status = imu_init();

    if (imu_status == ESP_OK) {
        ESP_LOGI(TAG, "IMU initialization successful");
    } else {
        ESP_LOGE(TAG, "IMU initialization failed: %s",
                esp_err_to_name(imu_status));
    }

    send_heartbeat();


    // ========================================================
    // HEARTBEAT TIMER
    // ========================================================

    uint32_t heartbeat_timer = 0;


    // ========================================================
    // MAIN FC LOOP
    // ========================================================

    while (1)
    {
        imu_raw_data_t imu_data;

    if (imu_status == ESP_OK) {
        if (imu_read(&imu_data) == ESP_OK) {
            ESP_LOGI(TAG,
                    "ACC: X=%d Y=%d Z=%d | GYRO: X=%d Y=%d Z=%d",
                    imu_data.accel_x,
                    imu_data.accel_y,
                    imu_data.accel_z,
                    imu_data.gyro_x,
                    imu_data.gyro_y,
                    imu_data.gyro_z);
        }
}
        // ----------------------------------------------------
        // RECEIVE MAVLINK
        // ----------------------------------------------------

        receive_mavlink();


        // ----------------------------------------------------
        // HEARTBEAT EVERY 1 SECOND
        // ----------------------------------------------------

        heartbeat_timer++;


        if (heartbeat_timer >= 10)
        {
            send_heartbeat();

            heartbeat_timer = 0;
        }


        // ----------------------------------------------------
        // LOOP DELAY
        // ----------------------------------------------------

        vTaskDelay(
            pdMS_TO_TICKS(100)
        );
    }
}