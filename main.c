#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_hidd.h"
#include "esp_rom_uart.h"
#include "esp_log.h"

static const char *TAG = "HID_MOUSE";
static uint16_t hid_conn_id = 0;

// Callback HID события
void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event){
        case ESP_HIDD_EVENT_OPEN:
            hid_conn_id = param->open.conn_id;
            ESP_LOGI(TAG, "Device connected");
            break;
        case ESP_HIDD_EVENT_CLOSE:
            hid_conn_id = 0;
            ESP_LOGI(TAG, "Device disconnected");
            break;
        default:
            break;
    }
}

// Функция для чтения байта из UART через ROM API
int uart_read_byte() {
    while (esp_rom_uart_rx_one_char(UART_NUM_0, (uint8_t*)NULL) == 0) {
        ; // ждём байт
    }
    uint8_t byte;
    esp_rom_uart_rx_one_char(UART_NUM_0, &byte);
    return byte;
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    // Инициализация Bluetooth Classic HID
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_hidd_register_callbacks(&hidd_event_callback));
    ESP_ERROR_CHECK(esp_hidd_init());

    ESP_LOGI(TAG, "HID ready. Waiting for connection...");

    while(1){
        // читаем два байта: dx и dy
        int dx = uart_read_byte();
        int dy = uart_read_byte();

        if(hid_conn_id != 0){
            esp_hidd_send_mouse_value(hid_conn_id, 0, (int8_t)dx, (int8_t)dy, 0, 0);
            ESP_LOGI(TAG, "Move dx=%d dy=%d", dx, dy);
        }
    }
}
