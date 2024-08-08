#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "driver/spi_master.h"
#include "freertos/semphr.h"
#include "esp_lcd_ili9341.h"

#define LCD_HOST               SPI2_HOST
#define LCD_H_RES              (320)
#define LCD_V_RES              (240)
#define LCD_BIT_PER_PIXEL      (16)

#define PIN_NUM_LCD_CS         (GPIO_NUM_15)
#define PIN_NUM_LCD_PCLK       (GPIO_NUM_18)
// mosi
#define PIN_NUM_LCD_DATA0      (GPIO_NUM_23) 
#define PIN_NUM_LCD_DC         (GPIO_NUM_2)
#define PIN_NUM_LCD_RST        (GPIO_NUM_4)
#define PIN_NUM_BK_LIGHT       (2)

static const char *TAG = "esp-hello-world";
static esp_lcd_panel_handle_t lcd_handle = NULL;
static SemaphoreHandle_t refresh_finish = NULL;


IRAM_ATTR static bool test_notify_refresh_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    BaseType_t need_yield = pdFALSE;

    xSemaphoreGiveFromISR(refresh_finish, &need_yield);
    return (need_yield == pdTRUE);
}

static esp_err_t init_lcd(lv_display_t *disp) {
    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "Initialize SPI bus");
    const spi_bus_config_t bus_config = ILI9341_PANEL_BUS_SPI_CONFIG(PIN_NUM_LCD_PCLK, PIN_NUM_LCD_DATA0,
                                                                     LCD_H_RES * 80 * (sizeof(uint16_t) / 2));
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &bus_config, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = ILI9341_PANEL_IO_SPI_CONFIG(PIN_NUM_LCD_CS, PIN_NUM_LCD_DC,
                                                                                test_notify_refresh_ready, disp);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));
    ESP_LOGI(TAG, "Install ili9341 panel driver");
    const esp_lcd_panel_dev_config_t panel_config = {
      .reset_gpio_num = PIN_NUM_LCD_RST,
      .rgb_endian = LCD_RGB_ENDIAN_BGR,
      .bits_per_pixel = LCD_BIT_PER_PIXEL,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &lcd_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_handle));  
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_handle, true, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_handle, true));
    
    ESP_LOGI(TAG, "Installed ili9341 panel driver");  

    return ret;
}

static void draw_bitmap(esp_lcd_panel_handle_t panel_handle)
{
    refresh_finish = xSemaphoreCreateBinary();

    uint16_t row_line = LCD_V_RES / LCD_BIT_PER_PIXEL;
    uint8_t byte_per_pixel = LCD_BIT_PER_PIXEL / 8;
    uint8_t *color = (uint8_t *)heap_caps_calloc(1, row_line * LCD_H_RES * byte_per_pixel, MALLOC_CAP_DMA);

    for (int j = 0; j < LCD_BIT_PER_PIXEL; j++) {
        for (int i = 0; i < row_line * LCD_H_RES; i++) {
            for (int k = 0; k < byte_per_pixel; k++) {
                color[i * byte_per_pixel + k] = (SPI_SWAP_DATA_TX(BIT(j), LCD_BIT_PER_PIXEL) >> (k * 8)) & 0xff;
            }
        }
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, j * row_line, LCD_H_RES, (j + 1) * row_line, color));
        xSemaphoreTake(refresh_finish, portMAX_DELAY);
    }
    free(color);
    vSemaphoreDelete(refresh_finish);
}

void app_main(void)
{
    printf("ESP32 Hello World!\n");
    /* LCD HW initialization */
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    ESP_ERROR_CHECK(gpio_set_level(PIN_NUM_BK_LIGHT, 1));
    ESP_ERROR_CHECK(init_lcd(NULL));
    draw_bitmap(lcd_handle);
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_reset_pin(PIN_NUM_BK_LIGHT);

    ESP_LOGI(TAG, "Display Hello World!");
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
