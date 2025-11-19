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
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_hid_gap.h"
#include "esp_hidd.h"

/* Настройки UART */
#define UART_NUM UART_NUM_0
#define BUF_SIZE (1024)
#define RD_BYTES (2)

/* Настройки Bluetooth */
#define HID_DEVICE_NAME "ESP32_HID_Mouse"

/* Глобальные переменные */
static bool connected = false;
static esp_bd_addr_t peer_addr;
static esp_hidd_dev_t *hid_dev = NULL;
static const char *TAG = "HID_Mouse";

/* Дескриптор HID-мыши */
static const uint8_t hid_mouse_report_map[] = {
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

/* Конфигурация HID-устройства */
static const esp_hidd_app_param_t app_param = {
    .name = "ESP32 Mouse",
    .description = "HID Mouse",
    .provider = "ESP32",
    .subclass = ESP_HID_CLASS_MIC,
};

static const esp_hidd_qos_param_t both_qos = {
    .service_type = 0x01,
    .token_rate = 0,
    .token_bucket_size = 0,
    .peak_bandwidth = 0,
    .latency = 0,
    .delay_variation = 0
};

/* Обработчик событий HID */
static void hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;

    switch (event) {
        case ESP_HIDD_START_EVENT:
            ESP_LOGI(TAG, "HID начал работу");
            break;
            
        case ESP_HIDD_CONNECT_EVENT:
            connected = true;
            memcpy(peer_addr, param->connect.remote_bda, ESP_BD_ADDR_LEN);
            ESP_LOGI(TAG, "Подключено к: %02x:%02x:%02x:%02x:%02x:%02x",
                     peer_addr[0], peer_addr[1], peer_addr[2],
                     peer_addr[3], peer_addr[4], peer_addr[5]);
            break;
            
        case ESP_HIDD_PROTOCOL_MODE_EVENT:
            ESP_LOGI(TAG, "Протокольный режим: %s", 
                     param->protocol_mode.protocol_mode ? "BOOT" : "REPORT");
            break;
            
        case ESP_HIDD_DISCONNECT_EVENT:
            connected = false;
            ESP_LOGI(TAG, "Отключено");
            // Снова делаем обнаруживаемым для нового подключения
            esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
            break;
            
        case ESP_HIDD_SEND_REPORT_EVENT:
            if (param->send_report.status == ESP_HIDD_REPORT_SEND_SUCCESS) {
                ESP_LOGI(TAG, "Отчет отправлен успешно");
            }
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
                ESP_LOGE(TAG, "Ошибка аутентификации: %d", param->auth_cmpl.stat);
            }
            break;
            
        case ESP_BT_GAP_PIN_REQ_EVT:
            // Для устройств, требующих PIN-код
            ESP_LOGI(TAG, "Требуется PIN-код");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
            break;
            
        case ESP_BT_GAP_CFM_REQ_EVT:
            // Подтверждение числового сравнения
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;
            
        default:
            break;
    }
}

/* Инициализация UART */
static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Настройка параметров UART
    uart_param_config(UART_NUM, &uart_config);
    
    // Установка пинов UART
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    // Установка драйвера UART
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    
    ESP_LOGI(TAG, "UART инициализирован на скорости 115200");
}

/* Задача для чтения данных мыши и отправки через HID */
static void hid_mouse_task(void *pvParameters)
{
    uint8_t data[RD_BYTES];
    uint8_t mouse_report[4] = {0}; // [buttons, dx, dy, wheel]
    
    ESP_LOGI(TAG, "Задача HID-мыши запущена");
    
    while (1) {
        if (connected && hid_dev != NULL) {
            // Чтение данных из UART
            int len = uart_read_bytes(UART_NUM, data, RD_BYTES, 20 / portTICK_PERIOD_MS);
            
            if (len == RD_BYTES) {
                // Формируем отчет мыши
                mouse_report[0] = 0x00;  // Кнопки не нажаты
                mouse_report[1] = data[0]; // dx
                mouse_report[2] = data[1]; // dy  
                mouse_report[3] = 0x00;  // Колесо
                
                // Отправка отчета мыши
                esp_hidd_dev_report_send(hid_dev, ESP_HIDD_REPORT_TYPE_INTRDATA, 
                                        ESP_HIDD_REPORT_ID_MOUSE, 
                                        4, mouse_report);
                
                ESP_LOGI(TAG, "Отправлено перемещение: dx=%d, dy=%d", 
                         (int8_t)data[0], (int8_t)data[1]);
            } else if (len > 0) {
                ESP_LOGW(TAG, "Прочитано неверное количество байт: %d", len);
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
    ESP_LOGI(TAG, "Запуск HID-мыши на ESP-IDF v5.3.1...");
    
    // Инициализация NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
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
    
    // Установка имени устройства
    ESP_ERROR_CHECK(esp_bt_dev_set_device_name(HID_DEVICE_NAME));
    
    // Инициализация HID-устройства
    esp_hidd_dev_config_t hid_config = {
        .device_type = ESP_HIDD_CLASSIC_BT,
        .mode = ESP_HIDD_MODE_REPORT,
        .name = HID_DEVICE_NAME,
        .description = "ESP32 HID Mouse",
        .provider = "ESP32",
        .subclass = ESP_HID_CLASS_MIC,
        .report_map_size = sizeof(hid_mouse_report_map),
        .report_map_data = hid_mouse_report_map,
    };
    
    hid_dev = esp_hidd_dev_init(&hid_config);
    if (hid_dev == NULL) {
        ESP_LOGE(TAG, "Ошибка инициализации HID-устройства");
        return;
    }
    
    // Регистрация callback для HID событий
    ESP_ERROR_CHECK(esp_hidd_dev_register_callback(hid_dev, hidd_event_callback, NULL));
    
    // Запуск HID-устройства
    ESP_ERROR_CHECK(esp_hidd_dev_start(hid_dev));
    
    // Установка режима обнаружения
    esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
    
    // Инициализация UART
    uart_init();
    
    // Создание задачи для работы HID-мыши
    xTaskCreate(hid_mouse_task, "hid_mouse_task", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Инициализация завершена. Ожидание подключения...");
    ESP_LOGI(TAG, "Имя устройства: %s", HID_DEVICE_NAME);
}