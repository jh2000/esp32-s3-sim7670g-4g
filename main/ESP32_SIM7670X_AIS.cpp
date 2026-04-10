#include <cstdlib> // For std::calloc
#include <iostream>
#include <type_traits>
#include <fstream>
#include <string>
#include <vector>
#include <regex>
#include <iomanip>
#include <cmath>
#include <stdio.h>
#include <inttypes.h>
#include <errno.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/temperature_sensor.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <at_3gpp_ts_27_007.h> // Modifiziert für at_get_common_string; Nun wieder vanilla kompatibel.
#include <esp_log.h>
#include <esp_app_format.h>
#include <esp_camera.h>
#include <esp_check.h>
#include <esp_event.h>
#include <esp_flash_partitions.h>
#include <esp_heap_caps.h>
#include <esp_http_client.h>
#include <esp_netif.h>
#include <esp_netif_sntp.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_psram.h>
#include <esp_sleep.h>
#include <esp_sntp.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp_tls.h>
#include "esp_crt_bundle.h"
#include <iot_usbh_rndis.h>
#include <iot_usbh_ecm.h>
#include <iot_eth.h>
#include <iot_eth_netif_glue.h>
#include <iot_usbh_cdc.h>
// #include "nvs_flash.h"
#include <neopixel.h>
#include "nmea_parser.h"
#include <onewire_bus.h>
#include <cJSON.h>
#include "AIS.h"
#include <ds18b20.h>
#include <max17048.h>

#ifndef CONFIG_SYSCONFIG_GPS
#define CONFIG_SYSCONFIG_GPS 0
#endif
#ifndef CONFIG_SYSCONFIG_CAM
#define CONFIG_SYSCONFIG_CAM 0
#endif
#ifndef CONFIG_SYSCONFIG_ADSB
#define CONFIG_SYSCONFIG_ADSB 0
#endif
#ifndef CONFIG_SYSCONFIG_ADSB_SQ
#define CONFIG_SYSCONFIG_ADSB_SQ 0
#endif
#ifndef CONFIG_SYSCONFIG_ADSB_EXTSQ
#define CONFIG_SYSCONFIG_ADSB_EXTSQ 0
#endif
#ifndef CONFIG_SYSCONFIG_ADSB_MODEAC
#define CONFIG_SYSCONFIG_ADSB_MODEAC 0
#endif
#ifndef CONFIG_SYSCONFIG_AIS
#define CONFIG_SYSCONFIG_AIS 0
#endif
#ifndef CONFIG_SYSCONFIG_RGB
#define CONFIG_SYSCONFIG_RGB 0
#endif
#ifndef CONFIG_SYSCONFIG_DS18B20
#define CONFIG_SYSCONFIG_DS18B20 0
#endif

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define radians(deg) ((deg) * DEG_TO_RAD)

#define PIXEL_COUNT 1

#define MAX17048_I2C_ADDRESS 0x36

#define EVENT_GOT_IP_BIT (BIT0)
#define EVENT_AT_READY_BIT (BIT1)

#define TIME_ZONE (+1)   // Berlin Time
#define YEAR_BASE (2000) // date in GPS starts from 2000

typedef struct
{
    const char *command;
    char *string;
    size_t len;
} at_common_string_t;
/*
GPIO0   ESP_BOOT "Taster 1"
GPIO1   BAT_ADC via R86 "NC" --> Onewire DS18B20
GPIO2   AIS_RESET
GPIO3   AIS_RX
GPIO4   TFCard CMD Pin --> ADSB_RX
GPIO5   TFCard CLK Pin --> ADSB_RTS
GPIO6   TFCard D0 Pin  --> ADSB_RESET
GPIO7   Kamera Y2
GPIO8   Kamera Y3
GPIO9   Kamera Y4
GPIO10  Kamera Y5
GPIO11  Kamera Y6
GPIO12  Kamera Y7
GPIO13  Kamera Y8
GPIO14  Kamera Y9
GPIO15  I2C Bus, Kamera SIO_DAT und MAX17048G SDA
GPIO16  I2C Bus, Kamera SIO_CLK und MAX17048G SCL
GPIO17  UART RX, an GSM TX (GPS/NMEA)
GPIO18  UART TX, an GSM RX (GPS/NMEA)
GPIO19  USB Neg. an GSM CDC
GPIO20  USB Pos. an GSM CDC
GPIO21  BAT_SET Parallelschalter (Modem ein/aus)
GPIO22  ?
GPIO23  ?
GPIO24  ?
GPIO25  ?
GPIO26  NC / PSRAM
GPIO27  NC / PSRAM
GPIO28  NC / PSRAM
GPIO29  NC / PSRAM
GPIO30  NC / PSRAM
GPIO31  NC / PSRAM
GPIO32  NC / PSRAM
GPIO33  NC / PSRAM
GPIO34  NC / PSRAM
GPIO35  NC / PSRAM
GPIO36  NC / PSRAM
GPIO37  NC / PSRAM
GPIO38  WS2812B (RGB-Led)
GPIO39  Kamera XCLK
GPIO40  UART an GSM RI
GPIO41  Kamera HRef
GPIO42  Kamera VSync
GPIO43  UART0 TX // Konsolenoutput
GPIO44  UART0 RX (vom UART nicht gelesen) --> Kamera Power
GPIO45  UART an GSM DTR
GPIO46  Kamera PCLK
*/

uart_port_t AIS_UART = UART_NUM_0;
uart_port_t ADSB_UART = UART_NUM_2;

#define Y2_GPIO_NUM 7
#define Y3_GPIO_NUM 8
#define Y4_GPIO_NUM 9
#define Y5_GPIO_NUM 10
#define Y6_GPIO_NUM 11
#define Y7_GPIO_NUM 12
#define Y8_GPIO_NUM 13
#define Y9_GPIO_NUM 14
#define XCLK_GPIO_NUM 39
#define HREF_GPIO_NUM 41
#define VSYNC_GPIO_NUM 42
#define PCLK_GPIO_NUM 46

#define TASK_CORE_RUN 1

#define BUFFSIZE 1024
#define HASH_LEN 32 /* SHA-256 digest length */
#define ONEWIRE_MAX_DS18B20 4

esp_app_desc_t running_app_info;
char hash_print[HASH_LEN * 2 + 1];
int ds18b20_device_num = 0;
float temperature[ONEWIRE_MAX_DS18B20] = {-99.0};
ds18b20_device_handle_t ds18b20s[ONEWIRE_MAX_DS18B20];
onewire_device_address_t ds18b20_address[ONEWIRE_MAX_DS18B20];

/* Root cert for howsmyssl.com, taken from server_root_cert.pem

   The PEM file was extracted from the output of this command:
   openssl s_client -showcerts -connect www.howsmyssl.com:443 </dev/null

   The CA root cert is the last cert given in the chain of certs.

   To embed it in the app binary, the PEM file is named
   in the component.mk COMPONENT_EMBED_TXTFILES variable.
*/
extern const uint8_t server_root_cert_pem_start[] asm("_binary_server_root_cert_pem_start");
extern const uint8_t server_root_cert_pem_end[] asm("_binary_server_root_cert_pem_end");

temperature_sensor_handle_t temp_handle = NULL;
temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
double esp_temp = -99.00;

bool task_bat_moni_kill = 0;
bool task_ds18b20_kill = 0;
bool task_rgb_kill = 0;
bool task_gps_kill = 0;
bool task_ais_kill = 0;
bool task_adsb_kill = 0;
bool task_cam_kill = 0;

struct
{
    uint32_t main = 4096;
    uint32_t bat_moni = 8192;
    uint32_t ds18b20 = 8192;
    uint32_t rgb = 2048;
    uint32_t ais = 8192;
    uint32_t adsb = 8192;
    uint32_t cam = 16384;
    uint32_t gps = 8192;
    uint32_t config = 16384;
    uint32_t ota = 16384;
} TaskStack;

TaskHandle_t Task_main;
TaskHandle_t Task_bat_moni;
TaskHandle_t Task_ds18b20;
TaskHandle_t Task_rgb;
TaskHandle_t Task_ais;
TaskHandle_t Task_adsb;
TaskHandle_t Task_gps;
TaskHandle_t Task_cam;
TaskHandle_t Task_config;
TaskHandle_t Task_OTA;
bool init_boot = 0;
int ota_running = 0;
static const char *TAG = "ESP32_4G_MODULE";
int log_msgs = 0;
bool restartnet = false;
int failcounter = 0;

bool gps_shutdown = true;

struct
{
    uint32_t total_in = 0;
    uint32_t total_out = 0;
    uint32_t total_sq = 0;
    uint32_t total_extsq = 0;
    uint32_t total_modeac = 0;
} adsb_msgs;

struct
{
    uint32_t total_in = 0;
    uint32_t total_out = 0;
    uint32_t filt_mindist = 0;
    uint32_t filt_other = 0;
} ais_msgs;

// uint32_t ais_msgs = 0;
// uint32_t adsb_msgs = 0;
// uint32_t adsb_msgs_total = 0;
enum loglevel_t
{
    LOG_NONE = 0,
    LOG_ERROR = 1,
    LOG_WARN = 2,
    LOG_INFO = 3,
    LOG_DEBUG = 4
};

typedef struct
{
    loglevel_t loglevel;
    const char *logip;
    in_port_t logport;
    bool firmware;
    bool firmware_force;
    bool gps;
    int gps_batlevel;
    bool cam;
    int caminterval;
    int cam_jpegquality;
    framesize_t cam_framesize;
    int cam_batlevel;
    bool adsb;
    const char *adsbip;
    in_port_t adsbport;
    bool adsb_sq;
    bool adsb_extsq;
    bool adsb_modeac;
    int adsb_batlevel;
    bool ais;
    double aismindist;
    const char *aisip;
    in_port_t aisport;
    int ais_batlevel;
    bool led;
    int config_update;
    bool ds18b20;
    int ds18b20_interval;
    int gsm_batlevel;
} sys_config_t;

sys_config_t sys_config = {
    .loglevel = (loglevel_t)CONFIG_SYSCONFIG_LOGLEVEL,
    .logip = CONFIG_SYSCONFIG_LOGIP,
    .logport = CONFIG_SYSCONFIG_LOGPORT,
    .firmware = false,
    .firmware_force = false,
    .gps = CONFIG_SYSCONFIG_GPS,
    .gps_batlevel = CONFIG_SYSCONFIG_GPS_BATLEVEL,
    .cam = CONFIG_SYSCONFIG_CAM,
    .caminterval = CONFIG_SYSCONFIG_CAM_INT,
    .cam_jpegquality = CONFIG_SYSCONFIG_CAM_QUAL,
    .cam_framesize = FRAMESIZE_UXGA,
    .cam_batlevel = CONFIG_SYSCONFIG_CAM_BATLEVEL,
    .adsb = CONFIG_SYSCONFIG_ADSB,
    .adsbip = CONFIG_SYSCONFIG_ADSBIP,
    .adsbport = CONFIG_SYSCONFIG_ADSBPORT,
    .adsb_sq = CONFIG_SYSCONFIG_ADSB_SQ,
    .adsb_extsq = CONFIG_SYSCONFIG_ADSB_EXTSQ,
    .adsb_modeac = CONFIG_SYSCONFIG_ADSB_MODEAC,
    .adsb_batlevel = CONFIG_SYSCONFIG_ADSB_BATLEVEL,
    .ais = CONFIG_SYSCONFIG_AIS,
    .aismindist = CONFIG_SYSCONFIG_AISMINDIST,
    .aisip = CONFIG_SYSCONFIG_AISIP,
    .aisport = CONFIG_SYSCONFIG_AISPORT,
    .ais_batlevel = CONFIG_SYSCONFIG_AIS_BATLEVEL,
    .led = CONFIG_SYSCONFIG_LED,
    .config_update = CONFIG_SYSCONFIG_UPDATEINT,
    .ds18b20 = CONFIG_SYSCONFIG_DS18B20,
    .ds18b20_interval = CONFIG_SYSCONFIG_DS18B20_INT,
    .gsm_batlevel = CONFIG_SYSCONFIG_GSM_BATLEVEL};

static camera_config_t camera_config = {
    .pin_pwdn = -1,            // NC
    .pin_reset = -1,           // NC
    .pin_xclk = XCLK_GPIO_NUM, // GPIO39
    .pin_sccb_sda = -1,
    .pin_sccb_scl = -1,

    .pin_d7 = Y9_GPIO_NUM,       // GPIO14
    .pin_d6 = Y8_GPIO_NUM,       // GPIO13
    .pin_d5 = Y7_GPIO_NUM,       // GPIO12
    .pin_d4 = Y6_GPIO_NUM,       // GPIO11
    .pin_d3 = Y5_GPIO_NUM,       // GPIO10
    .pin_d2 = Y4_GPIO_NUM,       // GPIO9
    .pin_d1 = Y3_GPIO_NUM,       // GPIO8
    .pin_d0 = Y2_GPIO_NUM,       // GPIO7
    .pin_vsync = VSYNC_GPIO_NUM, // GPIO42
    .pin_href = HREF_GPIO_NUM,   // GPIO41
    .pin_pclk = PCLK_GPIO_NUM,   // GPIO46

    .xclk_freq_hz = 20000000, // 20000000
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,             // YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = sys_config.cam_framesize,     // FRAMESIZE_UXGA,     // 1600x1200,    // QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.
    .jpeg_quality = sys_config.cam_jpegquality, // 0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 2,                              // When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .grab_mode = CAMERA_GRAB_LATEST,            // CAMERA_GRAB_LATEST. Sets when buffers should be filled
    .sccb_i2c_port = I2C_NUM_0};

time_t now;
struct tm timeinfo;
int log_sock = 0;
int ais_sock = 0;
int adsb_sock = 0;
char strftime_buf[64];
struct sockaddr_in log_dest_addr;
struct sockaddr_in ais_dest_addr;
struct sockaddr_in adsb_dest_addr;

gps_t *gps;
float voltage = 0, percent = 0, crate = 0;

char ais_lat_str[20];
char ais_lon_str[20];
double ais_lat = 0;
double ais_lon = 0;

uint8_t fw_sha_256[HASH_LEN] = {0};

/*an ota data write buffer ready to write to the flash*/
static char ota_write_data[BUFFSIZE + 1] = {0};

uint8_t wheel = 0;

tNeopixelContext neopixel;
QueueHandle_t ais_uart_queue;
QueueHandle_t adsb_uart_queue;

i2c_bus_handle_t i2c_bus = NULL;
max17048_handle_t max17048 = NULL;

i2c_config_t i2c_conf = {
    i2c_conf.mode = I2C_MODE_MASTER,
    i2c_conf.sda_io_num = GPIO_NUM_15,
    i2c_conf.scl_io_num = GPIO_NUM_16,
    i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE,
    i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE,
    i2c_conf.master.clk_speed = 400000,
    i2c_conf.clk_flags = 0};

static EventGroupHandle_t s_event_group;
#ifdef CONFIG_MODE_RNDIS
static iot_eth_driver_t *s_rndis_eth_driver = NULL;
static esp_netif_t *s_rndis_netif = NULL;
#else
static iot_eth_driver_t *s_ecm_eth_driver = NULL;
static esp_netif_t *s_ecm_netif = NULL;
#endif

esp_http_client_config_t _server_config = {
    .url = CONFIG_UPLOAD_SERVER_URL,
    .method = HTTP_METHOD_POST,
    .timeout_ms = -1,
    .buffer_size = 1024,
    .buffer_size_tx = 1024,
    .is_async = false,
    .use_global_ca_store = true,
    .skip_cert_common_name_check = true,
};
esp_http_client_handle_t _server_client = NULL;

iot_eth_handle_t eth_handle = NULL;

typedef struct
{
    usbh_cdc_port_handle_t cdc_port; /*!< CDC port handle */
    at_handle_t at_handle;           /*!< AT command parser handle */
} at_ctx_t;

at_ctx_t g_at_ctx = {0};

double ais_dist = 0;
double ais_dist_min = 999999999;
double ais_dist_max = 0;
 static void task_main(void *pvParameters);
 static void task_bat_moni(void *pvParameters);
static void task_ds18b20(void *pvParameters);
 static void task_rgb(void *pvParameters);
 static void task_ais(void *pvParameters);
 static void task_adsb(void *pvParameters);
 static void task_gps(void *pvParameters);
 static void task_cam(void *pvParameters);
 static void task_config(void *pvParameters);
 static void task_ota(void *pvParameter);
uint32_t UDP_LOG(const char *tag, loglevel_t loglevel, char *msg, ...);

double distanceBetween(double lat1, double long1, double lat2, double long2)
{
    // returns distance in meters between two positions, both specified
    // as signed decimal-degrees latitude and longitude. Uses great-circle
    // distance computation for hypothetical sphere of radius 6372795 meters.
    // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
    // Courtesy of Maarten Lamers
    double delta = radians(long1 - long2);
    double sdlong = sin(delta);
    double cdlong = cos(delta);
    lat1 = radians(lat1);
    lat2 = radians(lat2);
    double slat1 = sin(lat1);
    double clat1 = cos(lat1);
    double slat2 = sin(lat2);
    double clat2 = cos(lat2);
    delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
    delta = delta * delta;
    delta += clat2 * sdlong * clat2 * sdlong;
    delta = sqrt(delta);
    double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
    delta = atan2(delta, denom);
    return delta * 6372795;
}

static void deep_sleep_register_rtc_timer_wakeup(void)
{
    const int wakeup_time_sec = 60;
    ESP_LOGE(TAG, "Enabling timer wakeup, %ds\n", wakeup_time_sec);
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000));
}

void ota_confirm()
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK)
    {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
        {
            esp_ota_mark_app_valid_cancel_rollback();
            UDP_LOG(TAG, LOG_INFO, "Firmware update successfull... keeping flash.");
        }
    }
}
void vTaskGetRunTimeStats2(char *pcWriteBuffer)
{
    TaskStatus_t *pxTaskStatusArray;
    UBaseType_t uxArraySize, x;
    configRUN_TIME_COUNTER_TYPE ulTotalTime, ulStatsAsPercentage;

    /*
     * PLEASE NOTE:
     *
     * This function is provided for convenience only, and is used by many
     * of the demo applications.  Do not consider it to be part of the
     * scheduler.
     *
     * vTaskGetRunTimeStats() calls uxTaskGetSystemState(), then formats part
     * of the uxTaskGetSystemState() output into a human readable table that
     * displays the amount of time each task has spent in the Running state
     * in both absolute and percentage terms.
     *
     * vTaskGetRunTimeStats() has a dependency on the sprintf() C library
     * function that might bloat the code size, use a lot of stack, and
     * provide different results on different platforms.  An alternative,
     * tiny, third party, and limited functionality implementation of
     * sprintf() is provided in many of the FreeRTOS/Demo sub-directories in
     * a file called printf-stdarg.c (note printf-stdarg.c does not provide
     * a full snprintf() implementation!).
     *
     * It is recommended that production systems call uxTaskGetSystemState()
     * directly to get access to raw stats data, rather than indirectly
     * through a call to vTaskGetRunTimeStats().
     */

    /* Make sure the write buffer does not contain a string. */
    *pcWriteBuffer = (char)0x00;

    /* Take a snapshot of the number of tasks in case it changes while this
     * function is executing. */
    uxArraySize = uxTaskGetNumberOfTasks();

    /* Allocate an array index for each task.  NOTE!  If
     * configSUPPORT_DYNAMIC_ALLOCATION is set to 0 then pvPortMalloc() will
     * equate to NULL. */
    pxTaskStatusArray = (TaskStatus_t *)pvPortMalloc(uxArraySize * sizeof(TaskStatus_t)); /*lint !e9079 All values returned by pvPortMalloc() have at least the alignment required by the MCU's stack and this allocation allocates a struct that has the alignment requirements of a pointer. */

    if (pxTaskStatusArray != NULL)
    {
        /* Generate the (binary) data. */
        uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalTime);

        /* For percentage calculations. */
        ulTotalTime /= 100UL;

        /* Avoid divide by zero errors. */
        if (ulTotalTime > 0UL)
        {
            /* Create a human readable table from the binary data. */
            for (x = 0; x < uxArraySize; x++)
            {
                /* What percentage of the total run time has the task used?
                 * This will always be rounded down to the nearest integer.
                 * ulTotalRunTime has already been divided by 100. */
                ulStatsAsPercentage = pxTaskStatusArray[x].ulRunTimeCounter / ulTotalTime;

                /* Write the task name to the string, padding with
                 * spaces so it can be printed in tabular form more
                 * easily. */
                //     pcWriteBuffer = pxTaskStatusArray[ x ].pcTaskName;
                UBaseType_t high = uxTaskGetStackHighWaterMark(pxTaskStatusArray[x].xHandle);

                if (ulStatsAsPercentage > 0UL)
                {
#ifdef portLU_PRINTF_SPECIFIER_REQUIRED
                    {
                        sprintf(pcWriteBuffer, "%s\t%lu\t\t%lu%%\t\t%i\r\n", pxTaskStatusArray[x].pcTaskName, pxTaskStatusArray[x].ulRunTimeCounter, ulStatsAsPercentage, high);
                    }
#else
                    {
                        /* sizeof( int ) == sizeof( long ) so a smaller
                         * printf() library can be used. */
                        sprintf(pcWriteBuffer, "%s\t%u\t\t%u%%\t\t%i\r\n", pxTaskStatusArray[x].pcTaskName, (unsigned int)pxTaskStatusArray[x].ulRunTimeCounter, (unsigned int)ulStatsAsPercentage, high); /*lint !e586 sprintf() allowed as this is compiled with many compilers and this is a utility function only - not part of the core kernel implementation. */
                    }
#endif
                }
                else
                {
/* If the percentage is zero here then the task has
 * consumed less than 1% of the total run time. */
#ifdef portLU_PRINTF_SPECIFIER_REQUIRED
                    {
                        sprintf(pcWriteBuffer, "%s\t%lu\t\t<1%%\t\t%i\r\n", pxTaskStatusArray[x].pcTaskName, pxTaskStatusArray[x].ulRunTimeCounter, high);
                    }
#else
                    {
                        /* sizeof( int ) == sizeof( long ) so a smaller
                         * printf() library can be used. */
                        sprintf(pcWriteBuffer, "%s\t%u\t\t<1%%\t\t%i\r\n", pxTaskStatusArray[x].pcTaskName, (unsigned int)pxTaskStatusArray[x].ulRunTimeCounter, high); /*lint !e586 sprintf() allowed as this is compiled with many compilers and this is a utility function only - not part of the core kernel implementation. */
                    }
#endif
                }

                pcWriteBuffer += strlen(pcWriteBuffer); /*lint !e9016 Pointer arithmetic ok on char pointers especially as in this case where it best denotes the intent of the code. */
            }
        }

        /* Free the array again.  NOTE!  If configSUPPORT_DYNAMIC_ALLOCATION
         * is 0 then vPortFree() will be #defined to nothing. */
        vPortFree(pxTaskStatusArray);
    }
}

uint32_t UDP_LOG(const char *tag, loglevel_t loglevel, char *msg, ...)
{
    int err = 0;
    if (loglevel <= sys_config.loglevel)
    {
        if (log_sock < 0)
        {
            ESP_LOGE(tag, "Log Socket not initialized!");
        }
        else
        {
            va_list param;
            char udp_msg[900];
            char udp_msg_out[1024];
            va_start(param, msg);
            vsnprintf(udp_msg, sizeof(udp_msg), msg, param);
            va_end(param);

            char level_symbol[] = "NEWID";
            snprintf(udp_msg_out, sizeof(udp_msg_out), "%05i (%s) (%c)%s: %s\n\r", log_msgs, strftime_buf, level_symbol[loglevel], tag, udp_msg);
            log_msgs++;
            err = sendto(log_sock, udp_msg_out, strlen(udp_msg_out), 0, (struct sockaddr *)&log_dest_addr, sizeof(log_dest_addr));
            if (err < 0)
            {
                ESP_LOGE(tag, "Error occurred during sending: errno %d", errno);
            }
        }
    }
    return err;
};

uint32_t ais_msg(char *msg, const char *TASK_TAG)
{
    int err = 0;
    if (ais_sock <= 0)
    {
        UDP_LOG(TASK_TAG, LOG_ERROR, "AIS Socket not initalized");
    }
    else
    {
        err = sendto(ais_sock, msg, strlen(msg), 0, (struct sockaddr *)&ais_dest_addr, sizeof(ais_dest_addr));
        if (err < 0)
        {
            UDP_LOG(TASK_TAG, LOG_ERROR, "Error occurred during sending: errno %d", errno);
        }
    }
    return err;
};

uint32_t adsb_msg(char *msg, const char *TASK_TAG)
{
    int err = 0;
    if (adsb_sock <= 0)
    {
        UDP_LOG(TASK_TAG, LOG_ERROR, "ADSB Socket not initialized");
    }
    else
    {

        err = sendto(adsb_sock, msg, strlen(msg), 0, (struct sockaddr *)&adsb_dest_addr, sizeof(adsb_dest_addr));
        if (err < 0)
        {
            UDP_LOG(TASK_TAG, LOG_ERROR, "Error occurred during sending: errno %d", errno);
        }
    }
    return err;
};

esp_err_t max17048_refresh() {

   int status = 0;
            //  max17048_exit_hibernation_mode(max17048);
            status += max17048_get_cell_voltage(max17048, &voltage);
            status += max17048_get_cell_percent(max17048, &percent);
            status += max17048_get_charge_rate(max17048, &crate);
            return status; 
}

void gps_enable()
{
    if (gps_shutdown == true)
    {

        // char str[64] = {0};
        // at_common_string_t common_str2 = {.command = "AT+CGNSSPWR=1", .string = str, .len = sizeof(str)};
        // modem_at_send_command(g_at_ctx.at_handle, common.str2.command,500,NULL, g_at_ctx)

        // at_get_common_string(g_at_ctx.at_handle, &common_str2);
        UDP_LOG(TAG, LOG_INFO, "CGNSSPWR: %i", at_send_command_response_ok(g_at_ctx.at_handle, "AT+CGNSSPWR=1"));
        // str[0] = '\0'; // clear the string buffer
        // at_common_string_t common_str3 = {.command = "AT+CGNSSTST=1", .string = str, .len = sizeof(str)};
        // at_get_common_string(g_at_ctx.at_handle, &common_str3);

        UDP_LOG(TAG, LOG_INFO, "CGNSSTST: %i", at_send_command_response_ok(g_at_ctx.at_handle, "AT+CGNSSTST=1"));
        // str[0] = '\0'; // clear the string buffer
        // at_common_string_t common_str4 = {.command = "AT+CGNSSPORTSWITCH=0,1", .string = str, .len = sizeof(str)};
        // at_get_common_string(g_at_ctx.at_handle, &common_str4);
        UDP_LOG(TAG, LOG_INFO, "CGNSSPORTSWITCH: %i", at_send_command_response_ok(g_at_ctx.at_handle, "AT+CGNSSPORTSWITCH=0,1"));
        // str[0] = '\0'; // clear the string buffer
        gps_shutdown = false;
    }
}
void gps_disable()
{
    if (gps_shutdown == false)
    {
        // char str[64] = {0};
        // at_common_string_t common_str2 = {.command = "AT+CGNSSPWR=0", .string = str, .len = sizeof(str)};
        // at_get_common_string(g_at_ctx.at_handle, &common_str2);
        // UDP_LOG(TAG, LOG_INFO, "CGNSSPWR: %s", str);
        UDP_LOG(TAG, LOG_INFO, "CGNSSPWR: %i", at_send_command_response_ok(g_at_ctx.at_handle, "AT+CGNSSPWR=0"));
        // str[0] = '\0'; // clear the string buffer
        // at_common_string_t common_str3 = {.command = "AT+CGNSSTST=0", .string = str, .len = sizeof(str)};
        // at_get_common_string(g_at_ctx.at_handle, &common_str3);
        // UDP_LOG(TAG, LOG_INFO, "CGNSSTST: %s", str);
        UDP_LOG(TAG, LOG_INFO, "CGNSSTST: %i", at_send_command_response_ok(g_at_ctx.at_handle, "AT+CGNSSTST=0"));
        // str[0] = '\0'; // clear the string buffer
        gps_shutdown = true;
    }
}
void ais_enable()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_AIS_RESET_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    if (uart_is_driver_installed(AIS_UART))
    {
        uart_flush(AIS_UART);
    }
    gpio_set_level((gpio_num_t)CONFIG_AIS_RESET_PIN, 1);
}

void ais_disable()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_AIS_RESET_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)CONFIG_AIS_RESET_PIN, 0);
    if (uart_is_driver_installed(AIS_UART))
    {
        uart_flush(AIS_UART);
    }
}
void cam_enable()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_CAM_POWER_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)CONFIG_CAM_POWER_PIN, 1);
}

void cam_disable()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_CAM_POWER_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)CONFIG_CAM_POWER_PIN, 0);
}
void gsm_enable()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_GSM_POWER_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)CONFIG_GSM_POWER_PIN, 1);
}

void gsm_disable()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_GSM_POWER_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)CONFIG_GSM_POWER_PIN, 0);
}
void adsb_enable()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_ADSB_RESET_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    if (uart_is_driver_installed(ADSB_UART))
    {
        uart_flush(ADSB_UART);
    }
    gpio_set_level((gpio_num_t)CONFIG_ADSB_RESET_PIN, 1);
}

void adsb_disable()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_ADSB_RESET_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)CONFIG_ADSB_RESET_PIN, 0);
    if (uart_is_driver_installed(ADSB_UART))
    {
        uart_flush(ADSB_UART);
    }
}

void server_sendJpeg(camera_fb_t *fb)
{
    uint8_t *jpeg = fb->buf;
    size_t jpegSize = fb->len;
    char jpegSizeStr[7] = "";

    snprintf(jpegSizeStr, sizeof(jpegSizeStr), "%u", jpegSize);
    UDP_LOG(TAG, LOG_DEBUG, "Image Upload begin, size %i", fb->len);
    UDP_LOG(TAG, LOG_DEBUG, "Image height: %i", fb->height);
    UDP_LOG(TAG, LOG_DEBUG, "Image width: %i", fb->width);

    _server_client = esp_http_client_init(&_server_config);
    esp_err_t err = esp_http_client_open(_server_client, -1);
    if (err != ESP_OK)
    {
        UDP_LOG(TAG, LOG_ERROR, "Failed to open HTTP connection: %s FC: %i IHS: %i MFH: %i", esp_err_to_name(err), failcounter, esp_get_free_internal_heap_size(), esp_get_minimum_free_heap_size());

        failcounter++;
        return;
    }
    uint32_t offset = 0;
    static unsigned int chunksize;

    int written_length = 0;
    int total_length = 0;
    char length_string[11] = "";
    chunksize = 1024;
    char string[chunksize + 1] = {};

    while (offset < jpegSize)
    {
        if (jpegSize - offset < 100)
            chunksize = jpegSize - offset;
        //    UDP_LOG(TAG, LOG_INFO, "Chunk: %u of %u in hex: %X", offset, jpegSize, chunksize);
        bzero(length_string, sizeof(length_string));
        bzero(string, sizeof(string));
        memcpy(string, (const char *)jpeg + offset, chunksize);

        snprintf(length_string, sizeof(length_string), "%X", chunksize);
        written_length = esp_http_client_write(_server_client, length_string, strlen(length_string));
        if (written_length == -1)
        {
            UDP_LOG(TAG, LOG_ERROR, "Upload Status: %u", esp_http_client_get_status_code(_server_client));
            esp_http_client_close(_server_client);
            return;
        }
        else
        {
            total_length += written_length;
        }

        written_length = esp_http_client_write(_server_client, "\r\n", 2);
        if (written_length == -1)
        {
            UDP_LOG(TAG, LOG_ERROR, "Upload Status: %u", esp_http_client_get_status_code(_server_client));
            esp_http_client_close(_server_client);
            return;
        }
        total_length += written_length;
        written_length = esp_http_client_write(_server_client, string, chunksize);

        if (written_length == -1 || esp_http_client_get_status_code(_server_client) > 203)
        {
            UDP_LOG(TAG, LOG_ERROR, "Upload Status: %u", esp_http_client_get_status_code(_server_client));
            esp_http_client_close(_server_client);
            return;
        }
        else
        {

            total_length += written_length;
            written_length = esp_http_client_write(_server_client, "\r\n", 2);
            if (written_length == -1)
            {
                UDP_LOG(TAG, LOG_ERROR, "Upload Status: %u", esp_http_client_get_status_code(_server_client));
                esp_http_client_close(_server_client);
                return;
            }
            total_length += written_length;
        }
        offset += chunksize;
    }
    written_length = esp_http_client_write(_server_client, "0\r\n\r\n", 5);
    total_length += written_length;

    UDP_LOG(TAG, LOG_DEBUG, "Upload Status: %u", esp_http_client_get_status_code(_server_client));

    esp_http_client_close(_server_client);
    esp_http_client_cleanup(_server_client);
    UDP_LOG(TAG, LOG_DEBUG, "Image Upload end");
}
double toDouble(float floatValue, int precision)
{
    // Convert the float to a double
    double convertedDouble = static_cast<double>(floatValue);

    // Format the double to two decimal places using stringstream
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision) << convertedDouble;

    // Convert the formatted string back to a double
    double result = std::stod(ss.str());

    return result;
}
void get_temp()
{
    if (temp_handle != NULL)
    {
        temperature_sensor_enable(temp_handle);
        float esp_temp_lcl;
        temperature_sensor_get_celsius(temp_handle, &esp_temp_lcl);
        esp_temp = toDouble(esp_temp_lcl, 2);
        temperature_sensor_disable(temp_handle);
    }
}

char *JSON_Types(int type)
{
    if (type == cJSON_Invalid)
        return ("cJSON_Invalid");
    if (type == cJSON_False)
        return ("cJSON_False");
    if (type == cJSON_True)
        return ("cJSON_True");
    if (type == cJSON_NULL)
        return ("cJSON_NULL");
    if (type == cJSON_Number)
        return ("cJSON_Number");
    if (type == cJSON_String)
        return ("cJSON_String");
    if (type == cJSON_Array)
        return ("cJSON_Array");
    if (type == cJSON_Object)
        return ("cJSON_Object");
    if (type == cJSON_Raw)
        return ("cJSON_Raw");
    return NULL;
}

void JSON_Parse(const cJSON *const root)
{
    cJSON *current_element = NULL;
    cJSON_ArrayForEach(current_element, root)
    {
        if (current_element->string)
        {
            char *string = current_element->string;
            // ESP_LOGI(TAG, "[%s]", string);
            if (strcmp(string, "firmware") == 0)
            {
                ota_confirm();
            }

            if (strcmp(string, "firmware") == 0 && cJSON_IsTrue(current_element))
            {
                sys_config.led = true;
                if (eTaskGetState(Task_rgb) == eSuspended)
                {
                    vTaskResume(Task_rgb);
                }
                if (Task_OTA == NULL || eTaskGetState(Task_OTA) == eDeleted)
                {
                    UDP_LOG(TAG, LOG_INFO, "(re)starting OTA Task: %i", xTaskCreatePinnedToCore(task_ota, "ota_task", TaskStack.ota, NULL, 5, &Task_OTA, TASK_CORE_RUN));
                }
                else
                {
                    UDP_LOG(TAG, LOG_INFO, "OTA Task already running %i", eTaskGetState(Task_OTA));
                }
            }
            else if (strcmp(string, "init_boot") == 0 && cJSON_IsTrue(current_element))
            {
                init_boot = true;
            }
            else if (strcmp(string, "force_firmware") == 0)
            {
                sys_config.firmware_force = cJSON_IsTrue(current_element);
            }
            else if (strcmp(string, "cam") == 0)
            {
                sys_config.cam = cJSON_IsTrue(current_element);
                if (cJSON_IsTrue(current_element)) // && percent >= sys_config.cam_batlevel)
                {
                    cam_enable();
                    if (Task_cam == NULL)
                    {
                        ESP_LOGI(TAG, "Erstelle CAM Task");
                        xTaskCreatePinnedToCore(task_cam, "task_cam", TaskStack.cam, NULL, 5, &Task_cam, TASK_CORE_RUN);
                    }
                    else
                    {
                        if (eTaskGetState(Task_cam) == eSuspended)
                        {
                            ESP_LOGI(TAG, "Resume CAM Task");
                            vTaskResume(Task_cam);
                        }
                    }
                }
            }
            else if (strcmp(string, "restore") == 0 && cJSON_IsTrue(current_element))
            {
                esp_ota_mark_app_invalid_rollback_and_reboot();
            }
            else if (strcmp(string, "reboot") == 0 && cJSON_IsTrue(current_element))
            {
                esp_restart();
            }
            else if (strcmp(string, "stats") == 0 && cJSON_IsTrue(current_element))
            {
                // char buff[2048];
                //  vTaskGetRunTimeStats2(buff);
                UDP_LOG(TAG, LOG_DEBUG, "Memusage:\nADSB: %i\nAIS: %i\nMainTask: %i\nDS18B20: %i\nBatMoni: %i\nCam: %i\nConf: %i\nGPS: %i\nRGB: %i\nOTA: %i",
                        TaskStack.adsb - uxTaskGetStackHighWaterMark(Task_adsb),
                        TaskStack.ais - uxTaskGetStackHighWaterMark(Task_ais),
                        TaskStack.main - uxTaskGetStackHighWaterMark(Task_main),
                        TaskStack.ds18b20 - uxTaskGetStackHighWaterMark(Task_ds18b20),
                        TaskStack.bat_moni - uxTaskGetStackHighWaterMark(Task_bat_moni),
                        TaskStack.cam - uxTaskGetStackHighWaterMark(Task_cam),
                        TaskStack.config - uxTaskGetStackHighWaterMark(Task_config),
                        TaskStack.gps - uxTaskGetStackHighWaterMark(Task_gps),
                        TaskStack.rgb - uxTaskGetStackHighWaterMark(Task_rgb),
                        TaskStack.ota - uxTaskGetStackHighWaterMark(Task_OTA));
            }
            else if (strcmp(string, "ds18b20") == 0)
            {
                sys_config.ds18b20 = cJSON_IsTrue(current_element);
                if (cJSON_IsTrue(current_element))
                {
                    if (Task_ds18b20 == NULL)
                    {
                        ESP_LOGI(TAG, "Erstelle DS18B20 Task");
                        xTaskCreatePinnedToCore(task_ds18b20, "task_ds18b20", TaskStack.ds18b20, NULL, 5, &Task_ds18b20, TASK_CORE_RUN);
                    }
                    else
                    {
                        if (eTaskGetState(Task_ds18b20) == eSuspended)
                        {
                            vTaskResume(Task_ds18b20);
                        }
                    }
                }
            }
            else if (strcmp(string, "gps") == 0)
            {
                if (sys_config.gps != cJSON_IsTrue(current_element))
                {
                    if (cJSON_IsTrue(current_element))
                    {
                        gps_enable();
                    }
                    else
                    {
                        gps_disable();
                    }
                }

                sys_config.gps = cJSON_IsTrue(current_element);
                if (cJSON_IsTrue(current_element))
                {
                    if (Task_gps == NULL)
                    {
                        ESP_LOGI(TAG, "Erstelle GPS Task");
                        xTaskCreatePinnedToCore(task_gps, "task_gps", TaskStack.gps, NULL, 5, &Task_gps, TASK_CORE_RUN);
                    }
                    else
                    {
                        if (eTaskGetState(Task_gps) == eSuspended)
                        {
                            vTaskResume(Task_gps);
                        }
                    }
                }
            }
            else if (strcmp(string, "gps_batlevel") == 0)
            {
                if (current_element->valueint >= 0 && current_element->valueint <= 100)
                { // Zwischen 0% und 100%
                    sys_config.gps_batlevel = current_element->valueint;
                }
            }
            else if (strcmp(string, "ais") == 0)
            {
                if (sys_config.ais != cJSON_IsTrue(current_element))
                {
                    if (cJSON_IsTrue(current_element))
                    {
                        ais_enable();
                    }
                    else
                    {
                        ais_disable();
                    }
                }
                sys_config.ais = cJSON_IsTrue(current_element);
                if (cJSON_IsTrue(current_element))
                {
                    if (Task_ais == NULL)
                    {
                        ESP_LOGI(TAG, "Erstelle AIS Task");
                        xTaskCreatePinnedToCore(task_ais, "task_ais", TaskStack.ais, NULL, 5, &Task_ais, TASK_CORE_RUN);
                    }
                    else
                    {
                        if (eTaskGetState(Task_ais) == eSuspended)
                        {
                            vTaskResume(Task_ais);
                        }
                    }
                }
            }
            else if (strcmp(string, "adsb") == 0)
            {
                if (sys_config.adsb != cJSON_IsTrue(current_element))
                {
                    if (cJSON_IsTrue(current_element))
                    {
                        adsb_enable();
                    }
                    else
                    {
                        adsb_disable();
                    }
                }

                sys_config.adsb = cJSON_IsTrue(current_element);
                if (cJSON_IsTrue(current_element))
                {
                    if (Task_adsb == NULL)
                    {
                        ESP_LOGI(TAG, "Erstelle ADSB Task");
                        xTaskCreatePinnedToCore(task_adsb, "task_adsb", TaskStack.adsb, NULL, 5, &Task_adsb, TASK_CORE_RUN);
                    }
                    else
                    {
                        if (eTaskGetState(Task_adsb) == eSuspended)
                        {
                            vTaskResume(Task_adsb);
                        }
                    }
                }
            }
            else if (strcmp(string, "adsb_sq") == 0)
            {
                sys_config.adsb_sq = cJSON_IsTrue(current_element);
            }
            else if (strcmp(string, "adsb_extsq") == 0)
            {
                sys_config.adsb_extsq = cJSON_IsTrue(current_element);
            }
            else if (strcmp(string, "adsb_modeac") == 0)
            {
                sys_config.adsb_modeac = cJSON_IsTrue(current_element);
            }
            else if (strcmp(string, "led") == 0)
            {
                sys_config.led = cJSON_IsTrue(current_element);
                if (cJSON_IsTrue(current_element))
                {
                    if (Task_rgb == NULL)
                    {
                        ESP_LOGI(TAG, "Erstelle LED Task");
                        xTaskCreatePinnedToCore(task_rgb, "task_rgb", 2048, NULL, 5, &Task_rgb, TASK_CORE_RUN);
                    }
                    else
                    {
                        if (eTaskGetState(Task_rgb) == eSuspended)
                        {
                            vTaskResume(Task_rgb);
                        }
                    }
                }
            }
            else if (strcmp(string, "loglevel") == 0)
            {
                if (current_element->valueint >= LOG_NONE && current_element->valueint <= LOG_DEBUG)
                {
                    //             if (static_cast<std::underlying_type<loglevel_t>::type>(current_element->valueint) >= 0 &&
                    //              static_cast<std::underlying_type<loglevel_t>::type>(current_element->valueint) <= static_cast<std::underlying_type<loglevel_t>::type>(loglevel_t::LOG_DEBUG)) {
                    sys_config.loglevel = (loglevel_t)current_element->valueint;
                }
            }
            else if (strcmp(string, "logport") == 0)
            {
                sys_config.logport = current_element->valueint;
            }
            else if (strcmp(string, "adsbport") == 0)
            {
                sys_config.adsbport = current_element->valueint;
            }
            else if (strcmp(string, "adsb_batlevel") == 0)
            {
                if (current_element->valueint >= 0 && current_element->valueint <= 100)
                { // Zwischen 0% und 100%
                    sys_config.adsb_batlevel = current_element->valueint;
                }
            }
            else if (strcmp(string, "aismindist") == 0)
            {
                sys_config.aismindist = current_element->valueint;
            }
            else if (strcmp(string, "aisport") == 0)
            {
                sys_config.aisport = current_element->valueint;
            }
            else if (strcmp(string, "ais_batlevel") == 0)
            {
                if (current_element->valueint >= 0 && current_element->valueint <= 100)
                { // Zwischen 0% und 100%
                    sys_config.ais_batlevel = current_element->valueint;
                }
            }
            else if (strcmp(string, "config_refresh") == 0)
            {
                if (current_element->valueint >= 5000 && current_element->valueint <= 600000)
                { // Zwischen 5 sek und 10 Min
                    sys_config.config_update = current_element->valueint;
                }
            }
            else if (strcmp(string, "cam_interval") == 0)
            {
                if (current_element->valueint >= 5000 && current_element->valueint <= 600000)
                { // Zwischen 5 sek und 10 Min
                    sys_config.caminterval = current_element->valueint;
                }
            }
            else if (strcmp(string, "cam_quality") == 0)
            {
                if (current_element->valueint >= 0 && current_element->valueint <= 63)
                { // Zwischen 0 und 63
                    if (sys_config.cam_jpegquality != current_element->valueint)
                    {
                        sys_config.cam_jpegquality = current_element->valueint;
                    }
                }
            }
            else if (strcmp(string, "cam_framesize") == 0)
            {
                if (current_element->valueint >= 0 && current_element->valueint <= 63)
                { // Zwischen 0 und 63
                    if (sys_config.cam_framesize != current_element->valueint)
                    {
                        sys_config.cam_framesize = (framesize_t)current_element->valueint;
                    }
                }
            }
            else if (strcmp(string, "cam_lens") == 0)
            {
                sensor_t *s = esp_camera_sensor_get();
                if (s != NULL)
                {
                    s->set_lenc(s, cJSON_IsTrue(current_element));
                }
                //    if (s != NULL) s->set_reg(s, 0x109, 0x10, cJSON_IsTrue(current_element) ? 0x10 : 0x00);
            }
            else if (strcmp(string, "cam_batlevel") == 0)
            {
                if (current_element->valueint >= 0 && current_element->valueint <= 100)
                { // Zwischen 0% und 100%
                    sys_config.cam_batlevel = current_element->valueint;
                }
            }
            else if (strcmp(string, "gsm_batlevel") == 0)
            {
                if (current_element->valueint >= 0 && current_element->valueint <= 100)
                { // Zwischen 0% und 100%
                    sys_config.gsm_batlevel = current_element->valueint;
                }
            }
            else if (strcmp(string, "gsm_disable") == 0 && cJSON_IsTrue(current_element))
            {
                UDP_LOG(TAG, LOG_ERROR, "Disabling GSM!");
                gsm_disable(); // Crasht irgendwas mit anschließendem reboot.
            }

            //  if (string != NULL) free(string);
        }
        /*      if (cJSON_IsInvalid(current_element))
              {
                  ESP_LOGI(TAG, "Invalid");
              }
              else if (cJSON_IsFalse(current_element))
              {
                  ESP_LOGI(TAG, "False");
              }
              else if (cJSON_IsTrue(current_element))
              {
                  ESP_LOGI(TAG, "True");
              }
              else if (cJSON_IsNull(current_element))
              {
                  ESP_LOGI(TAG, "Null");
              }
              else if (cJSON_IsNumber(current_element))
              {
                  int valueint = current_element->valueint;
                  double valuedouble = current_element->valuedouble;
                  ESP_LOGI(TAG, "int=%d double=%f", valueint, valuedouble);
              }
              else if (cJSON_IsString(current_element))
              {
                  const char *valuestring = current_element->valuestring;
                  ESP_LOGI(TAG, "%s", valuestring);
              }
              else if (cJSON_IsArray(current_element))
              {
                  // UDP_LOG(TAG, LOG_INFO, "Array");
                  JSON_Parse(current_element);
              }
              else if (cJSON_IsObject(current_element))
              {
                  // UDP_LOG(TAG, LOG_INFO, "Object");
                  JSON_Parse(current_element);
              }
              else if (cJSON_IsRaw(current_element))
              {
                  ESP_LOGI(TAG, "Raw(Not support)");
              }
                  */
    }
}

#define MAX_HTTP_OUTPUT_BUFFER 2048
void update_config(const char *TASK_TAG)
{
    int free_mem = esp_get_free_internal_heap_size();
    ESP_LOGI(TAG, "Begin Free mem: %i", free_mem);
    //  char output_buffer[MAX_HTTP_OUTPUT_BUFFER + 1] = {0}; // Buffer to store response of http request
    char *output_buffer = (char *)malloc(MAX_HTTP_OUTPUT_BUFFER);
 //   ESP_LOGI(TAG, "Mid   Free mem: %i diff: %i", esp_get_free_internal_heap_size(), free_mem - esp_get_free_internal_heap_size());

    static esp_http_client_config_t http_config = {
        .url = CONFIG_SYNC_SERVER_URL,
        .method = HTTP_METHOD_POST,
        //        .cert_pem = (const unsigned char *) server_root_cert_pem_start,
        //        .cert_len = server_root_cert_pem_end - server_root_cert_pem_start,
        .use_global_ca_store = true,
        .skip_cert_common_name_check = true,
    };

    //  UDP_LOG(TAG, LOG_INFO, "HTTP native request =>");
    esp_http_client_handle_t client = esp_http_client_init(&http_config);

    cJSON *post_data = cJSON_CreateObject();
    cJSON_AddItemToObject(post_data, "version", cJSON_CreateString(running_app_info.version));
    cJSON_AddItemToObject(post_data, "sha256", cJSON_CreateString(hash_print));

  //  ESP_LOGI(TAG, "Mid   Free mem: %i diff: %i", esp_get_free_internal_heap_size(), free_mem - esp_get_free_internal_heap_size());
    cJSON_AddItemToObject(post_data, "bat_pct", cJSON_CreateNumber(toDouble(percent, 2)));
  //  ESP_LOGI(TAG, "Mid   Free mem: %i diff: %i", esp_get_free_internal_heap_size(), free_mem - esp_get_free_internal_heap_size());
    cJSON_AddItemToObject(post_data, "bat_volt", cJSON_CreateNumber(toDouble(voltage, 2)));
  //  ESP_LOGI(TAG, "Mid   Free mem: %i diff: %i", esp_get_free_internal_heap_size(), free_mem - esp_get_free_internal_heap_size());
    cJSON_AddItemToObject(post_data, "bat_rate", cJSON_CreateNumber(toDouble(crate, 3)));
  //  ESP_LOGI(TAG, "Mid   Free mem: %i diff: %i", esp_get_free_internal_heap_size(), free_mem - esp_get_free_internal_heap_size());

    if (init_boot == false)
    {
        cJSON_AddItemToObject(post_data, "init_boot", cJSON_CreateNumber(true));
        
    }
esp_reset_reason_t reset_reason = esp_reset_reason();
if (reset_reason != ESP_RST_SW) {
cJSON_AddItemToObject(post_data, "reset_reason", cJSON_CreateNumber(reset_reason));
}
    cJSON_AddItemToObject(post_data, "time", cJSON_CreateString(strftime_buf));
    cJSON_AddItemToObject(post_data, "uptime", cJSON_CreateNumber(esp_timer_get_time() / 1000000));
    get_temp();
    cJSON_AddItemToObject(post_data, "esp_temp", cJSON_CreateNumber(esp_temp));
//    cJSON_AddItemToObject(post_data, "esp_heap_free", cJSON_CreateNumber(esp_get_free_heap_size()));
    cJSON_AddItemToObject(post_data, "esp_heap_int", cJSON_CreateNumber(esp_get_free_internal_heap_size()));
//    cJSON_AddItemToObject(post_data, "esp_heap_min", cJSON_CreateNumber(esp_get_minimum_free_heap_size()));
    esp_modem_at_csq_t result;
    esp_err_t err = at_cmd_get_signal_quality(g_at_ctx.at_handle, &result);
    if (err == ESP_OK)
    {
        cJSON_AddItemToObject(post_data, "rssi", cJSON_CreateNumber(result.rssi));
        //                UDP_LOG(TAG, LOG_INFO, "Signal quality rssi: %d", result.rssi);
    }
    if (gps != NULL)
    {
        cJSON_AddItemToObject(post_data, "gps_lat", cJSON_CreateNumber(toDouble(gps->latitude, 5)));
        cJSON_AddItemToObject(post_data, "gps_lon", cJSON_CreateNumber(toDouble(gps->longitude, 5)));
        cJSON_AddItemToObject(post_data, "gps_sats", cJSON_CreateNumber(gps->sats_in_use));
        cJSON_AddItemToObject(post_data, "gps_alt", cJSON_CreateNumber(toDouble(gps->altitude, 2)));
        cJSON_AddItemToObject(post_data, "gps_speed", cJSON_CreateNumber(toDouble(gps->speed, 2)));
    }
    if (ais_msgs.total_in > 0 && sys_config.ais)
    {
        cJSON_AddItemToObject(post_data, "ais_msgs_in", cJSON_CreateNumber(ais_msgs.total_in));
        cJSON_AddItemToObject(post_data, "ais_msgs_out", cJSON_CreateNumber(ais_msgs.total_out));
        cJSON_AddItemToObject(post_data, "ais_msgs_mindist", cJSON_CreateNumber(ais_msgs.filt_mindist));
        cJSON_AddItemToObject(post_data, "ais_msgs_other", cJSON_CreateNumber(ais_msgs.filt_other));
        cJSON_AddItemToObject(post_data, "ais_lat", cJSON_CreateNumber(ais_lat));
        cJSON_AddItemToObject(post_data, "ais_lon", cJSON_CreateNumber(ais_lon));
        cJSON_AddItemToObject(post_data, "ais_dist", cJSON_CreateNumber(ais_dist));
        cJSON_AddItemToObject(post_data, "ais_dist_min", cJSON_CreateNumber(ais_dist_min));
        cJSON_AddItemToObject(post_data, "ais_dist_max", cJSON_CreateNumber(ais_dist_max));
    }
    if (adsb_msgs.total_in > 0 && sys_config.adsb)
    {
        cJSON_AddItemToObject(post_data, "adsb_msgs_in", cJSON_CreateNumber(adsb_msgs.total_in));
        cJSON_AddItemToObject(post_data, "adsb_msgs_out", cJSON_CreateNumber(adsb_msgs.total_out));
        cJSON_AddItemToObject(post_data, "adsb_msgs_sq", cJSON_CreateNumber(adsb_msgs.total_sq));
        cJSON_AddItemToObject(post_data, "adsb_msgs_extsq", cJSON_CreateNumber(adsb_msgs.total_extsq));
        cJSON_AddItemToObject(post_data, "adsb_msgs_modeac", cJSON_CreateNumber(adsb_msgs.total_modeac));
    }
    if (ds18b20_device_num > 0 && sys_config.ds18b20 == 1)
    {
        for (int i = 0; i < ds18b20_device_num; i++)
        {
            char ds18b20_tmp[25] = "";
            sprintf(ds18b20_tmp, "DS18B20_%016llX", ds18b20_address[i]);
            cJSON_AddItemToObject(post_data, ds18b20_tmp, cJSON_CreateNumber(temperature[i]));
        }
    }
 //   ESP_LOGI(TAG, "Mid1  Free mem: %i diff: %i", esp_get_free_internal_heap_size(), free_mem - esp_get_free_internal_heap_size());
    char *post_data_out = cJSON_PrintUnformatted(post_data);
 //  ESP_LOGI(TAG, "Mid1  Free mem: %i diff: %i", esp_get_free_internal_heap_size(), free_mem - esp_get_free_internal_heap_size());

    cJSON_Delete(post_data);
 //   ESP_LOGI(TAG, "Mid   Free mem: %i diff: %i", esp_get_free_internal_heap_size(), free_mem - esp_get_free_internal_heap_size());
    if (post_data_out == NULL)
        UDP_LOG(TASK_TAG, LOG_ERROR, "ERROR formatted");

    UDP_LOG(TASK_TAG, LOG_DEBUG, "POST-DATA: %s", post_data_out);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    err = esp_http_client_open(client, strlen(post_data_out));
    if (err != ESP_OK)
    {
        UDP_LOG(TASK_TAG, LOG_ERROR, "Failed to open HTTP connection: %s FC: %i, IHS: %i MFH: %i", esp_err_to_name(err), failcounter, esp_get_free_internal_heap_size(), esp_get_minimum_free_heap_size());
        failcounter++;
        if (failcounter > 3)
        {
            restartnet = true;
            failcounter = 0;
            UDP_LOG(TASK_TAG, LOG_ERROR, "Network defect, restart flag set FC: %i", failcounter);
            gsm_disable();
            vTaskDelay(pdMS_TO_TICKS(1000));
            gsm_enable();
            return;
        }
        if (failcounter > 10)
        {
            UDP_LOG(TASK_TAG, LOG_ERROR, "Network defect, restarting FC: %i", failcounter);
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_restart();
        }
    }
    else
    {
        esp_http_client_write(client, post_data_out, strlen(post_data_out));

        int content_length = esp_http_client_fetch_headers(client);
        if (content_length < 0)
        {
            UDP_LOG(TASK_TAG, LOG_ERROR, "HTTP client fetch headers failed");
        }
        else
        {
            int data_read = esp_http_client_read_response(client, output_buffer, MAX_HTTP_OUTPUT_BUFFER);
            output_buffer[data_read] = '\0';
            if (data_read >= 0)
            {
                int http_status = esp_http_client_get_status_code(client);
                if (http_status == 200)
                {
                    UDP_LOG(TASK_TAG, LOG_DEBUG, "Buffer: %s", output_buffer);
                    UDP_LOG(TASK_TAG, LOG_DEBUG, "HTTP GET Retval = %d, Status = %d, content_length = %" PRId64, data_read,
                            http_status,
                            esp_http_client_get_content_length(client));
                    // ESP_LOG_BUFFER_HEX(TAG, output_buffer, data_read);
                    cJSON *root = cJSON_Parse(output_buffer);
   //                 ESP_LOGI(TAG, "Mid   Free mem: %i diff: %i", esp_get_free_internal_heap_size(), free_mem - esp_get_free_internal_heap_size());
                    JSON_Parse(root);
   //                 ESP_LOGI(TAG, "Mid   Free mem: %i diff: %i", esp_get_free_internal_heap_size(), free_mem - esp_get_free_internal_heap_size());

                    cJSON_Delete(root);
                }
                else
                {
                    UDP_LOG(TASK_TAG, LOG_ERROR, "HTTP Status Error: %i", http_status);
                }
            }
            else
            {
                UDP_LOG(TASK_TAG, LOG_ERROR, "Failed to read response");
            }
        }
    }
    cJSON_free(post_data_out);
    free(output_buffer);
    esp_http_client_close(client);

    ESP_LOGI(TAG, "End   Free mem: %i diff: %i", esp_get_free_internal_heap_size(), free_mem - esp_get_free_internal_heap_size());
}

esp_err_t camera_init()
{

    // initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        UDP_LOG(TAG, LOG_ERROR, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}

esp_err_t camera_capture(const char *TASK_TAG)
{

    if (esp_camera_sensor_get() == NULL)
    {
        esp_camera_init(&camera_config);
    }
    if (sys_config.cam_jpegquality != camera_config.jpeg_quality || sys_config.cam_framesize != camera_config.frame_size)
    {
        camera_config.jpeg_quality = sys_config.cam_jpegquality;
        camera_config.frame_size = sys_config.cam_framesize;
        esp_camera_deinit();
        if (ESP_OK == esp_camera_init(&camera_config))
        {
            UDP_LOG(TASK_TAG, LOG_INFO, "Camera reconfigured");
        }
        else
        {
            UDP_LOG(TASK_TAG, LOG_ERROR, "Camera reconfiguration error");
            return ESP_FAIL;
        }
    }
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
    {
        UDP_LOG(TASK_TAG, LOG_ERROR, "Camera Capture Failed");
        esp_camera_deinit();
        return ESP_FAIL;
    }
    server_sendJpeg(fb);
    esp_camera_fb_return(fb); // printf("CTotal PSRAM: %d bytes\n", psram_size);
    return ESP_OK;
}

static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case GPS_UPDATE:
        if (gps == NULL)
            gps = static_cast<gps_t *>(std::calloc(1, sizeof(gps_t)));
        if (gps != NULL)
        {
            free(gps);
            gps = static_cast<gps_t *>(std::calloc(1, sizeof(gps_t)));
        }
        memcpy(gps, event_data, sizeof(gps_t));
        break;
    case GPS_UNKNOWN:
        /* print unknown statements */
        UDP_LOG("GPS_EVENT", LOG_WARN, "Unknown statement:%s", (char *)event_data);
        break;
    default:
        break;
    }
}

static esp_err_t _at_send_cmd(const char *command, size_t length, void *usr_data)
{
    at_ctx_t *at_ctx = (at_ctx_t *)usr_data;
    return usbh_cdc_write_bytes(at_ctx->cdc_port, (const uint8_t *)command, length, pdMS_TO_TICKS(500));
}

static void _at_port_closed_cb(usbh_cdc_port_handle_t cdc_port_handle, void *arg)
{
    at_ctx_t *at_ctx = (at_ctx_t *)arg;
    ESP_LOGI(TAG, "AT port closed");
    at_ctx->cdc_port = NULL;

    if (at_ctx->at_handle)
    {
        modem_at_stop(at_ctx->at_handle);
        modem_at_parser_destroy(at_ctx->at_handle);
        at_ctx->at_handle = NULL;
    }
}

static void _at_recv_data_cb(usbh_cdc_port_handle_t cdc_port_handle, void *arg)
{
    at_ctx_t *at_ctx = (at_ctx_t *)arg;
    size_t length = 0;
    usbh_cdc_get_rx_buffer_size(cdc_port_handle, &length);
    char *buffer;
    size_t buffer_remain;
    modem_at_get_response_buffer(at_ctx->at_handle, &buffer, &buffer_remain);
    if (buffer_remain < length)
    {
        length = buffer_remain;
        UDP_LOG(TAG, LOG_ERROR, "data size is too big, truncated to %zu", length);
    }
    usbh_cdc_read_bytes(cdc_port_handle, (uint8_t *)buffer, &length, 0);
    // Parse the AT command response
    modem_at_write_response_done(at_ctx->at_handle, length);
}

#ifdef CONFIG_MODE_RNDIS
static esp_err_t at_init()
{
    ESP_LOGI(TAG, "AT init");
    // Open a CDC port for AT command
    usbh_cdc_port_handle_t _port = usb_rndis_get_cdc_port_handle(s_rndis_eth_driver);
    usb_device_handle_t _dev_hdl = NULL;
    ESP_ERROR_CHECK(usbh_cdc_get_dev_handle(_port, &_dev_hdl));
    usb_device_info_t device_info;
    ESP_ERROR_CHECK(usb_host_device_info(_dev_hdl, &device_info));
    usbh_cdc_port_config_t cdc_port_config = {
        .dev_addr = device_info.dev_addr,
        .itf_num = 2,
        .in_transfer_buffer_size = 1024,  // 512
        .out_transfer_buffer_size = 1024, // 512
        .cbs = {
            .notif_cb = NULL,
            .recv_data = _at_recv_data_cb,
            .closed = _at_port_closed_cb,
            .user_data = &g_at_ctx,
        },
        .flags = USBH_CDC_FLAGS_DISABLE_NOTIFICATION};

    ESP_RETURN_ON_ERROR(usbh_cdc_port_open(&cdc_port_config, &g_at_ctx.cdc_port), TAG, "Failed to open CDC port");

    // init the AT command parser
    modem_at_config_t at_config = {
        .send_buffer_length = 256,
        .recv_buffer_length = 256,
        .io = {
            .send_cmd = _at_send_cmd,
            .usr_data = &g_at_ctx,
        }};
    g_at_ctx.at_handle = modem_at_parser_create(&at_config);
    ESP_ERROR_CHECK(g_at_ctx.at_handle != NULL ? ESP_OK : ESP_FAIL);

    return modem_at_start(g_at_ctx.at_handle);
}
#else
static esp_err_t at_init()
{
    ESP_LOGI(TAG, "AT ECM init");
    // Open a CDC port for AT command
    usbh_cdc_port_handle_t _port = usb_ecm_get_cdc_port_handle(s_ecm_eth_driver);
    usb_device_handle_t _dev_hdl = NULL;
    ESP_ERROR_CHECK(usbh_cdc_get_dev_handle(_port, &_dev_hdl));
    usb_device_info_t device_info;
    ESP_ERROR_CHECK(usb_host_device_info(_dev_hdl, &device_info));
    usbh_cdc_port_config_t cdc_port_config = {
        .dev_addr = device_info.dev_addr,
        .itf_num = 2,
        .in_transfer_buffer_size = 1024,
        .out_transfer_buffer_size = 1024,
        .cbs = {
            .closed = _at_port_closed_cb,
            .recv_data = _at_recv_data_cb,
            .notif_cb = NULL,
            .user_data = &g_at_ctx,
        },
    };
    ESP_RETURN_ON_ERROR(usbh_cdc_port_open(&cdc_port_config, &g_at_ctx.cdc_port), TAG, "Failed to open CDC port");

    // init the AT command parser
    modem_at_config_t at_config = {
        .send_buffer_length = 1024,
        .recv_buffer_length = 1024,
        .io = {
            .send_cmd = _at_send_cmd,
            .usr_data = &g_at_ctx,
        }};
    g_at_ctx.at_handle = modem_at_parser_create(&at_config);
    ESP_ERROR_CHECK(g_at_ctx.at_handle != NULL ? ESP_OK : ESP_FAIL);

    return modem_at_start(g_at_ctx.at_handle);
}
#endif

static void iot_event_handle(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == IOT_ETH_EVENT)
    {
        switch (event_id)
        {
        case IOT_ETH_EVENT_START:
            ESP_LOGI(TAG, "IOT_ETH_EVENT_START");
            break;
        case IOT_ETH_EVENT_STOP:
            ESP_LOGE(TAG, "IOT_ETH_EVENT_STOP");
            break;
        case IOT_ETH_EVENT_CONNECTED:
            ESP_LOGI(TAG, "IOT_ETH_EVENT_CONNECTED");
            if (at_init() == ESP_OK)
            {
                xEventGroupSetBits(s_event_group, EVENT_AT_READY_BIT);
            }
            break;
        case IOT_ETH_EVENT_DISCONNECTED:
            ESP_LOGE(TAG, "IOT_ETH_EVENT_DISCONNECTED");
            xEventGroupClearBits(s_event_group, EVENT_GOT_IP_BIT);
            break;
        default:
            ESP_LOGE(TAG, "IOT_ETH_EVENT_UNKNOWN");
            break;
        }
    }
    else if (event_base == IP_EVENT)
    {
        ESP_LOGI(TAG, "GOT_IP");
        xEventGroupSetBits(s_event_group, EVENT_GOT_IP_BIT);
    }
}
#ifdef CONFIG_MODE_RNDIS
static void install_rndis(uint16_t idVendor, uint16_t idProduct, const char *netif_name)
{
    esp_err_t ret = ESP_OK;

    iot_eth_netif_glue_handle_t glue = NULL;

    usb_device_match_id_t *dev_match_id = static_cast<usb_device_match_id_t *> std::calloc(2, sizeof(usb_device_match_id_t));
    if (dev_match_id == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for device match ID");
        return;
    }
    dev_match_id[0].match_flags = USB_DEVICE_ID_MATCH_VID_PID;
    dev_match_id[0].idVendor = idVendor;
    dev_match_id[0].idProduct = idProduct;
    memset(&dev_match_id[1], 0, sizeof(usb_device_match_id_t)); // end of list
    iot_usbh_rndis_config_t rndis_cfg = {
        .match_id_list = dev_match_id,
    };
    ret = iot_eth_new_usb_rndis(&rndis_cfg, &s_rndis_eth_driver);
    if (ret != ESP_OK || s_rndis_eth_driver == NULL)
    {
        ESP_LOGE(TAG, "Failed to create USB RNDIS driver");
        free(dev_match_id);
        return;
    }
    // Note: dev_match_id is now managed by the driver, don't free it here

    iot_eth_config_t eth_cfg = {
        .driver = s_rndis_eth_driver,
        .stack_input = NULL,
    };
    ret = iot_eth_install(&eth_cfg, &eth_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install USB RNDIS driver");
        return;
    }

    esp_netif_inherent_config_t _inherent_eth_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
    _inherent_eth_config.if_key = netif_name;
    _inherent_eth_config.if_desc = netif_name;
    esp_netif_config_t netif_cfg = {
        .base = &_inherent_eth_config,
        .driver = NULL,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH,
    };
    s_rndis_netif = esp_netif_new(&netif_cfg);
    if (s_rndis_netif == NULL)
    {
        ESP_LOGE(TAG, "Failed to create network interface");
        return;
    }

    glue = iot_eth_new_netif_glue(eth_handle);
    if (glue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create netif glue");
        return;
    }
    esp_netif_attach(s_rndis_netif, glue);
    iot_eth_start(eth_handle);
}
#else
static void install_ecm(uint16_t idVendor, uint16_t idProduct, const char *netif_name)
{
    esp_err_t ret = ESP_OK;
    iot_eth_handle_t eth_handle = NULL;
    iot_eth_netif_glue_handle_t glue = NULL;

    usb_device_match_id_t *dev_match_id = static_cast<usb_device_match_id_t *>(std::calloc(2, sizeof(usb_device_match_id_t)));
    if (dev_match_id == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for device match ID");
        return;
    }
    dev_match_id[0].match_flags = USB_DEVICE_ID_MATCH_VID_PID;
    dev_match_id[0].idVendor = idVendor;
    dev_match_id[0].idProduct = idProduct;
    memset(&dev_match_id[1], 0, sizeof(usb_device_match_id_t)); // end of list
    iot_usbh_ecm_config_t ecm_cfg = {
        .match_id_list = dev_match_id,
    };

    ret = iot_eth_new_usb_ecm(&ecm_cfg, &s_ecm_eth_driver);
    if (ret != ESP_OK || s_ecm_eth_driver == NULL)
    {
        ESP_LOGE(TAG, "Failed to create USB ECM driver");
        free(dev_match_id);
        return;
    }
    // Note: dev_match_id is now managed by the driver, don't free it here

    iot_eth_config_t eth_cfg = {
        .driver = s_ecm_eth_driver,
        .stack_input = NULL,
    };
    ret = iot_eth_install(&eth_cfg, &eth_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install USB ECM driver");
        return;
    }

    esp_netif_inherent_config_t _inherent_eth_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
    _inherent_eth_config.if_key = netif_name;
    _inherent_eth_config.if_desc = netif_name;
    esp_netif_config_t netif_cfg = {
        .base = &_inherent_eth_config,
        .driver = NULL,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH,
    };
    s_ecm_netif = esp_netif_new(&netif_cfg);
    if (s_ecm_netif == NULL)
    {
        ESP_LOGE(TAG, "Failed to create network interface");
        return;
    }

    glue = iot_eth_new_netif_glue(eth_handle);
    if (glue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create netif glue");
        return;
    }
    esp_netif_attach(s_ecm_netif, glue);
    iot_eth_start(eth_handle);
}
#endif
static void http_cleanup(esp_http_client_handle_t client)
{
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}
static void task_fatal_error(void)
{
    ota_running = false;
    if (Task_bat_moni != NULL && eTaskGetState(Task_bat_moni) == eSuspended)
        vTaskResume(Task_bat_moni);
    if (Task_rgb != NULL && eTaskGetState(Task_rgb) == eSuspended)
        vTaskResume(Task_rgb);
    if (Task_ais != NULL && eTaskGetState(Task_ais) == eSuspended)
        vTaskResume(Task_ais);
    if (Task_adsb != NULL && eTaskGetState(Task_adsb) == eSuspended)
        vTaskResume(Task_adsb);
    if (Task_gps != NULL && eTaskGetState(Task_gps) == eSuspended)
        vTaskResume(Task_gps);
    if (Task_cam != NULL && eTaskGetState(Task_cam) == eSuspended)
        vTaskResume(Task_cam);

    UDP_LOG(TAG, LOG_ERROR, "Exiting task due to fatal error...");
    (void)vTaskDelete(NULL);
}

 static void task_ota(void *pvParameter)
{
    static const char *TASK_TAG = "TASK_OTA";
    esp_err_t err;
    /* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_partition = NULL;

    UDP_LOG(TASK_TAG, LOG_INFO, "Starting OTA task");
    ota_running = true;
    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();

    if (Task_bat_moni != NULL)
        vTaskSuspend(Task_bat_moni);
    // if (Task_rgb != NULL)
    //     vTaskSuspend(Task_rgb);
    if (Task_ais != NULL)
        vTaskSuspend(Task_ais);
    if (Task_adsb != NULL)
        vTaskSuspend(Task_adsb);
    if (Task_gps != NULL)
        vTaskSuspend(Task_gps);
    if (Task_cam != NULL)
        vTaskSuspend(Task_cam);
    //    if (Task_config != NULL) vTaskSuspend(Task_config);

    if (configured != running)
    {
        UDP_LOG(TASK_TAG, LOG_WARN, "Configured OTA boot partition at offset 0x%08" PRIx32 ", but running from offset 0x%08" PRIx32,
                configured->address, running->address);
        UDP_LOG(TASK_TAG, LOG_WARN, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    UDP_LOG(TASK_TAG, LOG_INFO, "Running partition type %d subtype %d (offset 0x%08" PRIx32 ")",
            running->type, running->subtype, running->address);

    esp_http_client_config_t fw_config = {
        .url = CONFIG_FIRMWARE_UPG_URL,
        // .cert_pem = (char *)server_cert_pem_start,
        .timeout_ms = CONFIG_OTA_RECV_TIMEOUT,
        .use_global_ca_store = true,
        .skip_cert_common_name_check = true,
        .keep_alive_enable = false,
    };

    esp_http_client_handle_t client = esp_http_client_init(&fw_config);
    if (client == NULL)
    {
        UDP_LOG(TASK_TAG, LOG_ERROR, "Failed to initialise HTTP connection");
        task_fatal_error();
    }
    err = esp_http_client_open(client, 0);
    if (err != ESP_OK)
    {
        UDP_LOG(TASK_TAG, LOG_ERROR, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        task_fatal_error();
    }
    esp_http_client_fetch_headers(client);
    int http_status = esp_http_client_get_status_code(client);
    UDP_LOG(TASK_TAG, LOG_INFO, "HTTP Status: %u", http_status);
    if (http_status != 200)
    {
        task_fatal_error();
    }

    update_partition = esp_ota_get_next_update_partition(NULL);
    assert(update_partition != NULL);
    UDP_LOG(TASK_TAG, LOG_INFO, "Writing to partition subtype %d at offset 0x%" PRIx32,
            update_partition->subtype, update_partition->address);

    int binary_file_length = 0;
    /*deal with all receive packet*/
    bool image_header_was_checked = false;
    while (1)
    {
        int data_read = esp_http_client_read(client, ota_write_data, BUFFSIZE);
        if (data_read < 0)
        {
            UDP_LOG(TASK_TAG, LOG_ERROR, "Error: data read error");
            http_cleanup(client);
            task_fatal_error();
        }
        else if (data_read > 0)
        {
            if (image_header_was_checked == false)
            {
                //    esp_app_desc_t new_app_info;
                if (data_read > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))
                {
                    image_header_was_checked = true;

                    err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
                    if (err != ESP_OK)
                    {
                        UDP_LOG(TASK_TAG, LOG_ERROR, "esp_ota_begin failed (%s)", esp_err_to_name(err));
                        http_cleanup(client);
                        esp_ota_abort(update_handle);
                        task_fatal_error();
                    }
                    UDP_LOG(TASK_TAG, LOG_INFO, "esp_ota_begin succeeded");
                }
                else
                {
                    UDP_LOG(TASK_TAG, LOG_ERROR, "received package is not fit len");
                    http_cleanup(client);
                    esp_ota_abort(update_handle);
                    task_fatal_error();
                }
            }
            err = esp_ota_write(update_handle, (const void *)ota_write_data, data_read);
            if (err != ESP_OK)
            {
                http_cleanup(client);
                esp_ota_abort(update_handle);
                task_fatal_error();
            }
            binary_file_length += data_read;
        }
        else if (data_read == 0)
        {
            /*
             * As esp_http_client_read never returns negative error code, we rely on
             * `errno` to check for underlying transport connectivity closure if any
             */
            if (errno == ECONNRESET || errno == ENOTCONN)
            {
                UDP_LOG(TASK_TAG, LOG_ERROR, "Connection closed, errno = %d", errno);
                break;
            }
            if (esp_http_client_is_complete_data_received(client) == true)
            {
                UDP_LOG(TASK_TAG, LOG_INFO, "Connection closed");
                break;
            }
            if (client == NULL)
            {
                UDP_LOG(TASK_TAG, LOG_ERROR, "Connection aborted");
                task_fatal_error();
            }
        }
    }
    UDP_LOG(TASK_TAG, LOG_INFO, "Total Write binary data length: %d", binary_file_length);
    if (esp_http_client_is_complete_data_received(client) != true)
    {
        UDP_LOG(TASK_TAG, LOG_ERROR, "Error in receiving complete file");
        http_cleanup(client);
        esp_ota_abort(update_handle);
        task_fatal_error();
    }

    err = esp_ota_end(update_handle);
    if (err != ESP_OK)
    {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED)
        {
            UDP_LOG(TASK_TAG, LOG_ERROR, "Image validation failed, image is corrupted");
        }
        else
        {
            UDP_LOG(TASK_TAG, LOG_ERROR, "esp_ota_end failed (%s)!", esp_err_to_name(err));
        }
        http_cleanup(client);
        task_fatal_error();
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK)
    {
        UDP_LOG(TASK_TAG, LOG_ERROR, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        http_cleanup(client);
        task_fatal_error();
    }
    ota_running = 2;
    UDP_LOG(TASK_TAG, LOG_INFO, "Prepare to restart system!");
    vTaskDelay(pdMS_TO_TICKS(5000));
    esp_restart();
}

 void task_cam(void *pvParameter)
{
    static const char *TASK_TAG = "TASK_CAM";
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(sys_config.caminterval)); // Erst warten, dann starten.
        if (sys_config.cam == 0)
        {
            cam_disable();
            vTaskSuspend(NULL);
        }
        if (sys_config.cam == true)
        {
            if (percent >= sys_config.cam_batlevel)
            {                                                              // Prüfe akkustand
                if (gpio_get_level((gpio_num_t)CONFIG_CAM_POWER_PIN) == 0) // Strom der Kamera einschalten
                {
                    cam_enable();
                }
                if (camera_capture(TASK_TAG) == ESP_FAIL)
                {
                    //                vTaskDelete(NULL);
                    cam_disable();
                    vTaskSuspend(NULL);
                }
            }
            else
            {                                        // Akkustand zu gering
                if (esp_camera_sensor_get() != NULL) // Prüfe ob Kamera bereits deinitialisiert wurde.
                {
                    esp_camera_deinit();
                }
                if (gpio_get_level((gpio_num_t)CONFIG_CAM_POWER_PIN) == 1) // Strom der Kamera abschalten
                {
                    cam_disable();
                }
            }
        }
    }
}
 void task_config(void *pvParameter)
{
    static const char *TASK_TAG = "TASK_CONFIG";
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(sys_config.config_update));
        update_config(TASK_TAG);
    }
}

bool startsWith(char *str, const char *prefix)
{
    std::string s = str;
    if (s.starts_with(prefix))
        return true;
    else
        return false;
}
double conv_degree(long min4, char *frac)
{
    double pos = min4 / 600000.0;
    sprintf(frac, "%.5f", pos);
    return pos;
}

std::string AisTrim(char *str)
{
    const std::regex separator{","};
    using Iter = std::sregex_token_iterator;
    std::string line{str};

    std::vector part(Iter(line.begin(), line.end(), separator, -1), {});

    if (part.size() > 5)
    {
        return part[5];
    }
    else
    {
        return "";
    }
}

 void task_adsb(void *)
{
    static const char *TASK_TAG = "TASK_ADSB";
    const uart_config_t adsb_cfg = {
        .baud_rate = 3000000, //   921600
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS, // UART_HW_FLOWCTRL_CTS_RTS, //UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT};

    ESP_ERROR_CHECK(uart_driver_install(ADSB_UART, 1024, 0, 40, &adsb_uart_queue, ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(uart_set_pin(ADSB_UART, UART_PIN_NO_CHANGE, CONFIG_ADSB_RX_PIN,
                                 CONFIG_ADSB_RTS_PIN, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_param_config(ADSB_UART, &adsb_cfg));
    uart_enable_pattern_det_baud_intr(ADSB_UART, '\n', 1, 9, 0, 0);
    uart_pattern_queue_reset(ADSB_UART, 40);
    uart_event_t event;
    char *data = (char *)malloc(BUFFSIZE);
    char *data_old = (char *)malloc(BUFFSIZE);
    while (true)
    {
        if (xQueueReceive(adsb_uart_queue, (void *)&event, pdMS_TO_TICKS(1000)))
        {
            if (event.type == UART_PATTERN_DET)
            {
                int pos = uart_pattern_pop_pos(ADSB_UART);
                if (pos != -1)
                {
                    // Read data up to the pattern
                    int len = uart_read_bytes(ADSB_UART, data, pos, 100 / portTICK_PERIOD_MS); // 8, 18, 32
                    data[len] = '\0';
                    if ((len == 32 && sys_config.adsb_extsq == true) || (len == 18 && sys_config.adsb_sq == true) || (len == 8 && sys_config.adsb_modeac == true))
                    {
                        if (strcmp(data, data_old) != 0)
                        {
                            strcpy(data_old, data);
                            adsb_msgs.total_out++;
                            adsb_msg(data, TASK_TAG);
                        }
                    }
                    if (len == 32)
                        adsb_msgs.total_extsq++;
                    if (len == 18)
                        adsb_msgs.total_sq++;
                    if (len == 8)
                        adsb_msgs.total_modeac++;

                    adsb_msgs.total_in++;
                }
                else
                {
                    uart_flush(ADSB_UART);
                }
            }
        }
        if (percent <= sys_config.adsb_batlevel)
        {
            while (percent <= sys_config.adsb_batlevel)
            {
                adsb_disable();
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            if (sys_config.adsb == 1)
                adsb_enable();
        }
        if (sys_config.adsb == 0)
            vTaskSuspend(NULL);
        if (task_adsb_kill == true)
        {
            ESP_LOGI(TASK_TAG, "Task killed");
            vTaskDelete(NULL);
        }
    }
}

 void task_ais(void *)
{
    static const char *TASK_TAG = "TASK_AIS";
    const uart_config_t ais_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT};

    ESP_ERROR_CHECK(uart_driver_install(AIS_UART, 1024, 0, 40, &ais_uart_queue, ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(uart_param_config(AIS_UART, &ais_cfg));
    ESP_ERROR_CHECK(uart_set_pin(AIS_UART, GPIO_NUM_43, CONFIG_AIS_RX_PIN, // 43 für Konsolenoutput von ESP_LOG*
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    uart_enable_pattern_det_baud_intr(AIS_UART, '\n', 1, 9, 0, 0);
    uart_pattern_queue_reset(AIS_UART, 40);
    uart_event_t event;
    char *data = (char *)malloc(BUFFSIZE);

    while (true)
    {
        if (xQueueReceive(ais_uart_queue, (void *)&event, pdMS_TO_TICKS(1000)))
        {
            if (event.type == UART_PATTERN_DET)
            {
                int pos = uart_pattern_pop_pos(AIS_UART);
                if (pos != -1)
                {
                    // Read data up to the pattern
                    int len = uart_read_bytes(AIS_UART, data, pos, 100 / portTICK_PERIOD_MS);
                    data[len] = '\0';
                    std::string line{data};
                    //    data = "!AIVDM,1,1,,A,15MgK45P3@G?fl0E`JbR0OwT0@MS,0*4E\0";

                    if (line.find_first_of("$!") != std::string::npos)
                    {
                        std::string nmea = line.substr(line.find_first_of("$!"));
                        ais_msgs.total_in++;
                        std::string aispart;
                        aispart = AisTrim(data);
                        if (aispart != "")
                        {
                            //      ESP_LOGI(TASK_TAG, "AisTrim: %s", aispart.c_str());
                        }
                        else
                        {
                            ESP_LOGE(TASK_TAG, "AisTrim returned false");
                        }
                        AIS ais_data(aispart.c_str());
                        ais_lat = conv_degree(ais_data.get_latitude(), ais_lat_str);
                        ais_lon = conv_degree(ais_data.get_longitude(), ais_lon_str);
                        //  ESP_LOGE("AIS", "Lat: %f Lon: %f", ais_lat, ais_lon);
                        ais_dist = 0.0;
                        if (gps != NULL && gps->latitude != 0 && gps->longitude != 0 && ais_lat != 0 && ais_lon != 0)
                        {
                            ais_dist = distanceBetween(ais_lat, ais_lon, gps->latitude, gps->longitude);
                            if (ais_dist > ais_dist_max)
                                ais_dist_max = ais_dist;
                            if (ais_dist < ais_dist_min)
                                ais_dist_min = ais_dist;
                            UDP_LOG(TASK_TAG, LOG_DEBUG, "Dist: %f Min: %f Max: %f", ais_dist, ais_dist_min, ais_dist_max);
                        }
                        if (ais_dist >= sys_config.aismindist)
                        {
                            ais_msgs.total_out++;
                            ais_msg((char *)nmea.c_str(), TASK_TAG);
                        }
                        else
                        {
                            ais_msgs.filt_mindist++;
                        }
                    }
                    else
                    {

                        UDP_LOG(TASK_TAG, LOG_INFO, "Unknown data: %s", data);
                        ESP_LOG_BUFFER_HEX(TASK_TAG, data, len);
                        ais_msgs.filt_other++;
                    }
                }
                else
                {
                    UDP_LOG(TASK_TAG, LOG_ERROR, "pos != -1 error");
                    uart_flush(AIS_UART);
                }
            }
        }
        if (percent <= sys_config.ais_batlevel)
        {
            while (percent <= sys_config.ais_batlevel)
            {
                ais_disable();
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            if (sys_config.ais == 1)
                ais_enable();
        }

        if (sys_config.ais == 0)
            vTaskSuspend(NULL);
        if (task_ais_kill == true)
        {
            vTaskDelete(NULL);
        }
    }
}
 void task_gps(void *)
{
    static const char *TASK_TAG = "TASK_GPS";

    while (true)
    {
        if (gps != NULL)
        {
            if (sys_config.gps == true)
            {
                if (percent <= sys_config.gps_batlevel)
                {
                    gps_disable();
                }
                else
                {
                    gps_enable();
                    UDP_LOG(TASK_TAG, LOG_DEBUG, "%d/%d/%d %d:%d:%d => \r\n"
                                                 "\t\t\t\t\t\tlatitude   = %.05f°N\r\n"
                                                 "\t\t\t\t\t\tlongitude = %.05f°E\r\n"
                                                 "\t\t\t\t\t\taltitude   = %.02fm\r\n"
                                                 "\t\t\t\t\t\tspeed      = %fm/s\r\n"
                                                 "\t\t\t\t\t\tvalid      = %i\r\n"
                                                 "\t\t\t\t\t\tfix        = %i\r\n"
                                                 "\t\t\t\t\t\tsats   = %i\r\n"
                                                 "\t\t\t\t\t\tfix_mode = %i",
                            gps->date.year + YEAR_BASE, gps->date.month, gps->date.day,
                            gps->tim.hour + TIME_ZONE, gps->tim.minute, gps->tim.second,
                            gps->latitude, gps->longitude, gps->altitude, gps->speed,
                            gps->valid, gps->fix, gps->sats_in_use, gps->fix_mode);
                }
            }
        }

        if (task_gps_kill == true)
        {
            vTaskDelete(NULL);
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
        if (sys_config.gps == 0)
            gps_disable();
        vTaskSuspend(NULL);
    }
}

 void task_bat_moni(void *pvParameter)
{
    static const char *TASK_TAG = "TASK_BAT_MONI";
    while (true)
    {
        if (max17048 != NULL)
        {
            int status = 0;
            //  max17048_exit_hibernation_mode(max17048);
            //status += max17048_get_cell_voltage(max17048, &voltage);
            //status += max17048_get_cell_percent(max17048, &percent);
            //status += max17048_get_charge_rate(max17048, &crate);
            status = max17048_refresh();
            //  max17048_enter_hibernation_mode(max17048);
            //         float batteryLevel = read_bat();
            UDP_LOG(TASK_TAG, LOG_DEBUG, "Batterylevel: %f Batteryvoltage: %f CRate: %f Status: %i", percent, voltage, toDouble(crate, 3), status);
        }
        else
        {
            UDP_LOG(TASK_TAG, LOG_ERROR, "MAX17048 not initialized");
            percent = 88.88; // Set fake Bat level to prevent shutdown if max17048 fails.
        }
        if (task_bat_moni_kill == true)
        {
            vTaskDelete(NULL);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void task_ds18b20(void *pvParameters)
{
    static const char *TASK_TAG = "TASK_DS18B20";

    // initialize the xLastWakeTime variable with the current time.
    // TickType_t last_wake_time = xTaskGetTickCount();
    onewire_bus_handle_t bus = NULL;
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = CONFIG_ONEWIRE_GPIO_NUM,
        .flags = {
            .en_pull_up = true, // enable the internal pull-up resistor in case the external device didn't have one
        }};
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10, // 1byte ROM command + 8byte ROM number + 1byte device command
    };

    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_onewire_device;
    esp_err_t search_result = ESP_OK;

    // create 1-wire device iterator, which is used for device search
    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    UDP_LOG(TASK_TAG, LOG_INFO, "Device iterator created, start searching...");
    do
    {
        search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
        if (search_result == ESP_OK)
        { // found a new device, let's check if we can upgrade it to a DS18B20
            ds18b20_config_t ds_cfg = {};
            onewire_device_address_t address;
            // check if the device is a DS18B20, if so, return the ds18b20 handle
            if (ds18b20_new_device_from_enumeration(&next_onewire_device, &ds_cfg, &ds18b20s[ds18b20_device_num]) == ESP_OK)
            {
                ds18b20_get_device_address(ds18b20s[ds18b20_device_num], &address);
                ds18b20_address[ds18b20_device_num] = address;
                UDP_LOG(TASK_TAG, LOG_INFO, "Found a DS18B20[%d], address: %016llX", ds18b20_device_num, address);
                ds18b20_device_num++;
            }
            else
            {
                UDP_LOG(TASK_TAG, LOG_WARN, "Found an unknown device, address: %016llX", next_onewire_device.address);
            }
        }
    } while (search_result != ESP_ERR_NOT_FOUND);
    ESP_ERROR_CHECK(onewire_del_device_iter(iter));
    UDP_LOG(TASK_TAG, LOG_INFO, "Searching done, %d DS18B20 device(s) found", ds18b20_device_num);

    while (true)
    {
        if (sys_config.ds18b20 == 0)
            vTaskSuspend(NULL);
        vTaskDelay(pdMS_TO_TICKS(sys_config.ds18b20_interval));

        // trigger temperature conversion for all sensors on the bus
        if (ds18b20_trigger_temperature_conversion_for_all(bus) == 0)
        {
            for (int i = 0; i < ds18b20_device_num; i++)
            {
                if (ds18b20_get_temperature(ds18b20s[i], &temperature[i]) == 0)
                {
                    // ds18b20_get_device_address(ds18b20s[i], &address);
                    UDP_LOG(TASK_TAG, LOG_DEBUG, "temperature read from DS18B20[%016llX]: %.2fC", ds18b20_address[i], temperature[i]);
                }
            }
        }
        else
            break;
    }

    UDP_LOG(TASK_TAG, LOG_WARN, "Task ending");

    vTaskDelete(NULL);
}

inline uint32_t Wheel(uint8_t WheelPos, int ota_run)
{
    WheelPos = 255 - WheelPos;
    if (ota_run == 1)
    { // Rot Grün während FW runtergeladen wird
        if ((WheelPos > 0 && WheelPos <= 63) || (WheelPos > 127 && WheelPos <= 192))
            return NP_RGB(0, 255, 0);
        else
            return NP_RGB(255, 0, 0);
    }
    else if (ota_run == 2)
    { // Grün bis zum reboot.
        return NP_RGB(0, 255, 0);
    }
    else
    {
        if (WheelPos < 85)
        {
            return NP_RGB((255 - WheelPos * 3) / 2, 0, (WheelPos * 3) / 2);
        }
        if (WheelPos < 170)
        {
            WheelPos -= 85;
            return NP_RGB(0, (WheelPos * 3) / 2, (255 - WheelPos * 3) / 2);
        }
        WheelPos -= 170;
        return NP_RGB((WheelPos * 3) / 2, (255 - WheelPos * 3) / 2, 0);
    }
}
 void task_rgb(void *pvParameter)
{
    while (true)
    {
        tNeopixel subpixel;
        if (sys_config.led == true)
        {
            wheel++;
            subpixel.index = 0;
            subpixel.rgb = Wheel(wheel, ota_running);
            tNeopixel pixel[] = {subpixel};
            neopixel_SetPixel(neopixel, pixel, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        else
        {

            subpixel.index = 0;
            subpixel.rgb = 0;
            tNeopixel pixel[] = {subpixel};
            neopixel_SetPixel(neopixel, pixel, 1);
            vTaskSuspend(NULL);
        }
    }
}

 void task_main(void *pvParameter)
{
    uint32_t uptime = 0;
    while (true)
    {
        uptime++;
        time(&now);
        localtime_r(&now, &timeinfo);
        strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (uptime > 3600 * 24)
        {
            UDP_LOG(TAG, LOG_ERROR, "24h reboot");
            vTaskDelay(pdMS_TO_TICKS(5000));
            esp_restart();
        }
        if (percent != 0.00 && percent <= CONFIG_SYSCONFIG_BOOT_BATLEVEL) { // Wenn Restlevel zu gering, rebooten und im Bootloop auf Akku warten. 
            UDP_LOG(TAG, LOG_ERROR, "Low Power, rebooting.");
            vTaskDelay(pdMS_TO_TICKS(5000));
            gsm_disable();
            gps_disable();
            ais_disable();
            adsb_disable();
            esp_restart();
        }
    }
}

static void print_sha256(const uint8_t *image_hash, const char *label)
{
    char hash_print[HASH_LEN * 2 + 1];
    hash_print[HASH_LEN * 2] = 0;
    for (int i = 0; i < HASH_LEN; ++i)
    {
        sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
    }
    ESP_LOGI(TAG, "%s: %s", label, hash_print);
}

extern "C" void app_main(void)
{
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();
    ESP_LOGI(TAG, "CAM disable...");
    cam_disable();
    ESP_LOGI(TAG, "AIS disable...");
    ais_disable();
    ESP_LOGI(TAG, "ADSB disable...");
    adsb_disable();
    ESP_LOGI(TAG, "GSM disable...");
    gsm_disable();
    i2c_bus = i2c_bus_create(I2C_NUM_0, &i2c_conf);
    max17048 = max17048_create(i2c_bus, MAX17048_I2C_ADDR_DEFAULT);
    if (max17048 != NULL)
    {
        max17048_refresh();
        //max17048_get_cell_voltage(max17048, &voltage);
        ESP_LOGI(TAG, "Voltage: %f", voltage);
        //max17048_get_cell_percent(max17048, &percent);
        ESP_LOGI(TAG, "Percent: %f", percent);
        //max17048_get_charge_rate(max17048, &crate);
        ESP_LOGI(TAG, "C-Rate: %f", crate);

        while (percent < CONFIG_SYSCONFIG_BOOT_BATLEVEL)
        {
            if (max17048_refresh() > 0) break;
            ESP_LOGE(TAG, "Bat Low, waiting...");
            //max17048_get_cell_voltage(max17048, &voltage);
            ESP_LOGI(TAG, "Voltage: %f", voltage);
            //max17048_get_cell_percent(max17048, &percent);
            ESP_LOGI(TAG, "Percent: %f", percent);
            //max17048_get_charge_rate(max17048, &crate);
            ESP_LOGI(TAG, "C-Rate: %f", crate);
            deep_sleep_register_rtc_timer_wakeup();
            esp_deep_sleep_start();
        }
    }
    else
    {
        percent = 88.88; // MAX17048 failed.
    }

    ESP_ERROR_CHECK(esp_netif_init());

    log_dest_addr.sin_addr.s_addr = inet_addr(sys_config.logip);
    log_dest_addr.sin_family = AF_INET;
    log_dest_addr.sin_port = htons(sys_config.logport);
    ais_dest_addr.sin_addr.s_addr = inet_addr(sys_config.aisip);
    ais_dest_addr.sin_family = AF_INET;
    ais_dest_addr.sin_port = htons(sys_config.aisport);
    adsb_dest_addr.sin_addr.s_addr = inet_addr(sys_config.adsbip);
    adsb_dest_addr.sin_family = AF_INET;
    adsb_dest_addr.sin_port = htons(sys_config.adsbport);
    log_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    ais_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    adsb_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (log_sock < 0 || ais_sock < 0 || adsb_sock < 0)
    {
        ESP_LOGE(TAG, "Error creating socket: %d", errno);
    }
    ESP_ERROR_CHECK(esp_tls_init_global_ca_store());
    esp_tls_set_global_ca_store(server_root_cert_pem_start, server_root_cert_pem_end - server_root_cert_pem_start);
    /* Initialize default TCP/IP stack */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    s_event_group = xEventGroupCreate();
    // ESP_RETURN_VOID_ON_FALSE(s_event_group != NULL, TAG, "Failed to create event group");
    esp_event_handler_register(IOT_ETH_EVENT, ESP_EVENT_ANY_ID, iot_event_handle, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, iot_event_handle, NULL);

    // install usbh cdc driver
    usbh_cdc_driver_config_t cdc_config = {
        .task_stack_size = 1024 * 4,
        .task_priority = configMAX_PRIORITIES - 1,
        .task_coreid = 0,
        .skip_init_usb_host_driver = false,
    };
    ESP_ERROR_CHECK(usbh_cdc_driver_install(&cdc_config));

#ifndef CONFIG_ESP_HTTP_CLIENT_ENABLE_HTTPS
    if (startsWith(CONFIG_FIRMWARE_UPG_URL, "https") || startsWith(CONFIG_SYNC_SERVER_URL, "https") || startsWith(CONFIG_UPLOAD_SERVER_URL, "https"))
    {
        ESP_LOGE(TAG, "https for CONFIG_*-URL, but HTTPS disabled! Won't work!!!");
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
#endif
#ifndef CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP
    if (startsWith(CONFIG_FIRMWARE_UPG_URL, "http:"))
    {
        ESP_LOGE(TAG, "https for CONFIG_FIRMWARE_UPG_URL required, but HTTP provided! Firmware update won't work!!! Change URL or enable CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP=y");
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
#endif

#ifdef CONFIG_MODE_RNDIS
    install_rndis(USB_DEVICE_VENDOR_ANY, USB_DEVICE_PRODUCT_ANY, "USB RNDIS0");
#else
    install_ecm(USB_DEVICE_VENDOR_ANY, USB_DEVICE_PRODUCT_ANY, "USB ECM0");
#endif
    vTaskDelay(pdMS_TO_TICKS(100));
    gsm_enable();
    xEventGroupWaitBits(s_event_group, EVENT_GOT_IP_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (ESP_OK == temperature_sensor_install(&temp_sensor_config, &temp_handle))
    {
        get_temp();
        ESP_LOGI(TAG, "int. Temp Sensor OK, %f°C", esp_temp);
    }
    else
    {
        ESP_LOGE(TAG, "int. Temp Sensor fail!");
    }

    uint8_t sha_256[HASH_LEN] = {0};
    esp_partition_t partition;

    // get sha256 digest for the partition table
    partition.address = ESP_PARTITION_TABLE_OFFSET;
    partition.size = ESP_PARTITION_TABLE_MAX_LEN;
    partition.type = ESP_PARTITION_TYPE_DATA;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for the partition table: ");

    // get sha256 digest for bootloader
    partition.address = ESP_BOOTLOADER_OFFSET;
    partition.size = ESP_PARTITION_TABLE_OFFSET;
    partition.type = ESP_PARTITION_TYPE_APP;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for bootloader: ");

    // get sha256 digest for running partition
    esp_partition_get_sha256(esp_ota_get_running_partition(), fw_sha_256);
    print_sha256(fw_sha_256, "SHA-256 for current firmware: ");

    esp_ota_get_partition_description(esp_ota_get_running_partition(), &running_app_info);
    hash_print[HASH_LEN * 2] = 0;
    for (int i = 0; i < HASH_LEN; ++i)
    {
        sprintf(&hash_print[i * 2], "%02x", fw_sha_256[i]);
    }

    neopixel = neopixel_Init(PIXEL_COUNT, CONFIG_NEOPIXEL_PIN);

    if (NULL == neopixel)
    {
        UDP_LOG(TAG, LOG_ERROR, "[%s] Neopixel Initialization failed\n", __func__);
    }

    ESP_LOGI(TAG, "SNTP init...");
    esp_sntp_config_t sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&sntp_config);

    nmea_parser_config_t nmea_config = {
        .uart = {
            .uart_port = UART_NUM_1,
            .rx_pin = CONFIG_NMEA_PARSER_UART_RXD,
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .event_queue_size = 16}};
    /* init NMEA parser library */
    ESP_LOGI(TAG, "NMEA init...");
    nmea_parser_handle_t nmea_hdl = nmea_parser_init(&nmea_config);
    /* register event handler for NMEA parser library */
    nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);

    xTaskCreatePinnedToCore(task_main, "task_main", TaskStack.main, NULL, 1, &Task_main, TASK_CORE_RUN);
    ESP_LOGI(TAG, "Starting tasks...");
    xTaskCreatePinnedToCore(task_bat_moni, "task_bat_moni", TaskStack.bat_moni, NULL, 1, &Task_bat_moni, TASK_CORE_RUN);
    xTaskCreatePinnedToCore(task_rgb, "task_rgb", TaskStack.rgb, NULL, 1, &Task_rgb, TASK_CORE_RUN);
    //    xTaskCreatePinnedToCore(task_ais, "task_ais", 16384, NULL, 5, &Task_ais,TASK_CORE_RUN);
    //    xTaskCreatePinnedToCore(task_adsb, "task_adsb", 16384, NULL, 5, &Task_adsb,TASK_CORE_RUN);
    //    xTaskCreatePinnedToCore(task_gps, "task_gps", 16384, NULL, 5, &Task_gps,TASK_CORE_RUN);
    //    xTaskCreatePinnedToCore(task_cam, "task_cam", 16384, NULL, 2, &Task_cam,TASK_CORE_RUN);
    xTaskCreatePinnedToCore(task_config, "task_config", TaskStack.config, NULL, 6, &Task_config, TASK_CORE_RUN);

    while (1)
    {
        EventBits_t bits = xEventGroupWaitBits(s_event_group, EVENT_AT_READY_BIT | EVENT_GOT_IP_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
        if (bits & EVENT_AT_READY_BIT)
        {
            at_cmd_set_echo(g_at_ctx.at_handle, true);

            esp_modem_at_csq_t result;
            esp_err_t err = at_cmd_get_signal_quality(g_at_ctx.at_handle, &result);
            if (err == ESP_OK)
            {
                UDP_LOG(TAG, LOG_INFO, "Signal quality rssi: %d", result.rssi);
            }

            // at_common_string_t common_str = {.command = "AT+DIALMODE=0", .string = str, .len = sizeof(str)};
            // at_get_common_string(g_at_ctx.at_handle, &common_str);
            ESP_LOGI(TAG, "DIALMODE: %i", at_send_command_response_ok(g_at_ctx.at_handle, "AT+DIALMODE=0"));
        }
        if (bits & EVENT_GOT_IP_BIT)
        {
            vTaskDelay(pdMS_TO_TICKS(1000)); // Wait a bit for DNS to be ready
                                             //  test_query_public_ip();          // Query public IP via HTTP
            ESP_LOGI(TAG, "Bereit...");
            // Print modem information
            char str[1024] = {0};
            at_cmd_get_manufacturer_id(g_at_ctx.at_handle, str, sizeof(str));
            UDP_LOG(TAG, LOG_INFO, "Modem manufacturer ID: %s", str);
            str[0] = '\0'; // clear the string buffer
            at_cmd_get_module_id(g_at_ctx.at_handle, str, sizeof(str));
            UDP_LOG(TAG, LOG_INFO, "Modem module ID: %s", str);
            str[0] = '\0'; // clear the string buffer
            at_cmd_get_revision_id(g_at_ctx.at_handle, str, sizeof(str));
            UDP_LOG(TAG, LOG_INFO, "Modem revision ID: %s", str);
            str[0] = '\0'; // clear the string buffer
            at_cmd_get_pdp_context(g_at_ctx.at_handle, str, sizeof(str));
            UDP_LOG(TAG, LOG_INFO, "Modem PDP context: %s", str);
            str[0] = '\0'; // clear the string buffer
            UDP_LOG(TAG, LOG_INFO, "Bereit...");
            return;
        }
    }
}