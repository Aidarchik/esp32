#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_hidd_api.h"
#include "esp_bt_hid.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "soc/uart_reg.h"

/* Настройки UART */
#define UART_NUM UART_NUM_0        // Используем UART0 (GPIO1 - TX, GPIO3 - RX)
#define BUF_SIZE (1024)
#define RD_BYTES (2)               // Читаем 2 байта (dx и dy) за раз

/* Настройки Bluetooth */
#define HID_DEVICE_NAME "ESP32_HID_Mouse"  // Имя Bluetooth-устройства

/* Дескриптор HID-мыши */
static const uint8_t hid_mouse_report_descriptor[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Buttons)
    0x19, 0x01,        //     Usage Minimum (Button 1)
    0x29, 0x03,        //     Usage Maximum (Button 3)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x03,        //     Report Count (3)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data, Variable, Absolute)
    0x95, 0x01,        //     Report Count (1)
    0x75, 0x05,        //     Report Size (5)
    0x81, 0x01,        //     Input (Constant) for padding
    0x05, 0x01,        //     Usage Page (Generic Desktop)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x02,        //     Report Count (2)
    0x81, 0x06,        //     Input (Data, Variable, Relative)
    0xC0,              //   End Collection
    0xC0               // End Collection
};

static const esp_hidd_app_param_t app_param = {
    .name = "ESP32 Mouse",
    .description = "HID Mouse",
    .provider = "ESP32",
    .subclass = ESP_HID_CLASS_MIC,
    .desc_list = hid_mouse_report_descriptor,
    .desc_list_len = sizeof(hid_mouse_report_descriptor)
};

static const esp_hidd_qos_param_t both_qos = {
    .service_type = 0x01,          // Best Effort
    .token_rate = 0,
    .token_bucket_size = 0,
    .peak_bandwidth = 0,
    .latency = 0,
    .delay_variation = 0
};

/* Глобальные переменные */
static bool connected = false;
static esp_bd_addr_t peer_addr;    // Адрес подключенного устройства
static esp_hidd_dev_t *hid_dev = NULL;
static const char *TAG = "HID_Mouse";

/* Прототипы функций */
static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);
static void gap_event_handler(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void uart_init(void);
static void hid_mouse_task(void *pvParameters);

/* Обработчик событий HID */
static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch (event) {
    case ESP_HIDD_INIT_EVT:
        ESP_LOGI(TAG, "HID профиль инициализирован");
        // Регистрируем HID-приложение после инициализации
        esp_bt_hid_device_register_app(&app_param, &both_qos, &both_qos);
        break;
    case ESP_HIDD_REGISTER_APP_EVT:
        ESP_LOGI(TAG, "HID приложение зарегистрировано");
        // Делаем устройство обнаруживаемым
        esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
        break;
    case ESP_HIDD_OPEN_EVT:
        if (param->open.status == ESP_HIDD_SUCCESS) {
            connected = true;
            memcpy(peer_addr, param->open.remote_bda, ESP_BD_ADDR_LEN);
            ESP_LOGI(TAG, "Подключено к: %02x:%02x:%02x:%02x:%02x:%02x",
                     peer_addr[0], peer_addr[1], peer_addr[2],
                     peer_addr[3], peer_addr[4], peer_addr[5]);
        }
        break;
    case ESP_HIDD_CLOSE_EVT:
        connected = false;
        ESP_LOGI(TAG, "Отключено");
        // Снова делаем обнаруживаемым для нового подключения
        esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
        break;
    case ESP_HIDD_SEND_REPORT_EVT:
        // Отчет отправлен
        break;
    default:
        break;
    }
}

/* Обработчик событий GAP */
static void gap_event_handler(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Аутентификация успешна");
        } else {
            ESP_LOGE(TAG, "Ошибка аутентификации");
        }
        break;
    case ESP_BT_GAP_PIN_REQ_EVT:
        // Для устройств, требующих PIN-код (обычно не требуется для HID)
        esp_bt_pin_code_t pin_code = {0};
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        break;
    default:
        break;
    }
}

/* Инициализация UART */
static void uart_init(void)
{
    // Базовая инициализация UART через драйвер (совместима с ESP-IDF 3.1.0)
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .use_ref_tick = false
    };
    
    // Настройка параметров UART
    uart_param_config(UART_NUM, &uart_config);
    
    // Установка пинов UART (UART0 по умолчанию использует GPIO1-TX, GPIO3-RX)
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    // Установка драйвера UART с буферами
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    
    ESP_LOGI(TAG, "UART инициализирован");
}

/* Задача для чтения данных мыши и отправки через HID */
static void hid_mouse_task(void *pvParameters)
{
    uint8_t data[RD_BYTES];
    uint8_t mouse_report[4] = {0}; // [buttons, dx, dy, wheel]
    
    while (1) {
        if (connected) {
            // Чтение данных из UART
            int len = uart_read_bytes(UART_NUM, data, RD_BYTES, 20 / portTICK_PERIOD_MS);
            
            if (len == RD_BYTES) {
                // Формируем отчет мыши: [кнопки, перемещение X, перемещение Y, колесо]
                mouse_report[0] = 0;  // Кнопки не нажаты
                mouse_report[1] = data[0]; // dx (относительное перемещение по X)
                mouse_report[2] = data[1]; // dy (относительное перемещение по Y)
                mouse_report[3] = 0;  // Колесо
                
                // Отправка отчета мыши
                esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 
                                            ESP_HIDD_REPORT_ID_MOUSE, 
                                            4, mouse_report);
                
                ESP_LOGI(TAG, "Отправлено перемещение: dx=%d, dy=%d", data[0], data[1]);
            }
        } else {
            // Если не подключено, ждем перед следующей проверкой
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }
}

/* Главная функция приложения */
void app_main(void)
{
    ESP_LOGI(TAG, "Запуск HID-мыши...");
    
    // Инициализация NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Инициализация Bluetooth контроллера
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    
    // Инициализация Bluedroid
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    
    // Регистрация обработчиков
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_bt_hid_device_register_callback(hidd_event_callback));
    
    // Установка имени устройства
    ESP_ERROR_CHECK(esp_bt_dev_set_device_name(HID_DEVICE_NAME));
    
    // Инициализация HID-устройства
    ESP_ERROR_CHECK(esp_bt_hid_device_init());
    
    // Инициализация UART
    uart_init();
    
    // Создание задачи для работы HID-мыши
    xTaskCreate(hid_mouse_task, "hid_mouse_task", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Инициализация завершена. Ожидание подключения...");
}