#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_ili9341.h"
#include "esp_lcd_touch_xpt2046.h"
#include "tree.h"

/* LCD settings */
#define LCD_HOST             (SPI2_HOST)
#define LCD_TOUCH_HOST       (SPI3_HOST)
#define LCD_H_RES            (240)
#define LCD_V_RES            (320)
#define LCD_BIT_PER_PIXEL    (16)

/* pins */
#define PIN_LCD_CS           (GPIO_NUM_15)
#define PIN_LCD_RST          (GPIO_NUM_4)
#define PIN_LCD_DC           (GPIO_NUM_2)
#define PIN_LCD_MOSI         (GPIO_NUM_13)
#define PIN_LCD_SCK          (GPIO_NUM_14)
#define PIN_LCD_LED          (GPIO_NUM_21)
#define PIN_LCD_MISO         (GPIO_NUM_12)
#define PIN_LCD_TOUCH_CLK    (GPIO_NUM_25)
#define PIN_LCD_TOUCH_CS     (GPIO_NUM_33)
#define PIN_LCD_TOUCH_DIN    (GPIO_NUM_32)
#define PIN_LCD_TOUCH_OUT    (GPIO_NUM_39)
#define PIN_LCD_TOUCH_IRQ    (GPIO_NUM_36)


static const char *TAG = "esp-hello-world";
static esp_lcd_panel_handle_t lcd_panel = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_touch_handle_t touch_handle = NULL;

/* lvgl */
static lv_display_t *lvgl_disp = NULL;
static lv_indev_t *lvgl_touch_indev = NULL;

/* tree, lvgl custom events */
#define MAX_TREE_HEIGHT 2
#define MAX_STACK_SIZE 3 * MAX_TREE_HEIGHT
#define MAX_NUM_DIGITS 9
static uint32_t CALC_EVENT_EVAL;
Arvore *arv = NULL;

static esp_err_t init_lcd() {
    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "Initialize SPI bus");

    const spi_bus_config_t bus_config = {                                                           
        .sclk_io_num = PIN_LCD_SCK,
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = PIN_LCD_MISO,
        .quadhd_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .max_transfer_sz = LCD_H_RES * 80 * (LCD_BIT_PER_PIXEL / 2),
    };
    
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &bus_config, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "Install panel IO");

    const esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = PIN_LCD_CS,
        .dc_gpio_num = PIN_LCD_DC,
        .spi_mode = 0,
        .pclk_hz = 40 * 1000 * 1000,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));
    ESP_LOGI(TAG, "Install ili9341 panel driver");
    const esp_lcd_panel_dev_config_t panel_config = {
      .reset_gpio_num = PIN_LCD_RST,
      .rgb_endian = LCD_RGB_ENDIAN_BGR,
      .bits_per_pixel = LCD_BIT_PER_PIXEL,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_panel));  
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_panel, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_panel, true));    
    ESP_LOGI(TAG, "Installed ili9341 panel driver");    

    ESP_LOGI(TAG, "Install XPT2046 touch controller");    
    esp_lcd_panel_io_handle_t touch_io_handle = NULL;
    esp_lcd_panel_io_spi_config_t touch_io_config = {                                                   
        .cs_gpio_num = PIN_LCD_TOUCH_CS,
        .dc_gpio_num = GPIO_NUM_NC,
        .spi_mode = 0,
        .pclk_hz = 2 * 1000 * 1000,// ESP_LCD_TOUCH_SPI_CLOCK_HZ
        .trans_queue_depth = 3,
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .flags = {
	  .dc_low_on_data = 0,
	  .octal_mode = 0,
	  .sio_mode = 0,
	  .lsb_first = 0,
	  .cs_high_active = 0
	}
    };

    static const int SPI_MAX_TRANSFER_SIZE = 32768;
    const spi_bus_config_t buscfg_touch = {
      .mosi_io_num = PIN_LCD_TOUCH_DIN,
      .miso_io_num = PIN_LCD_TOUCH_OUT,
      .sclk_io_num = PIN_LCD_TOUCH_CLK,
      .quadwp_io_num = GPIO_NUM_NC,
      .quadhd_io_num = GPIO_NUM_NC,
      .data4_io_num = GPIO_NUM_NC,
      .data5_io_num = GPIO_NUM_NC,
      .data6_io_num = GPIO_NUM_NC,
      .data7_io_num = GPIO_NUM_NC,
      .max_transfer_sz = SPI_MAX_TRANSFER_SIZE,
      .flags = SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI | SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS,
      .isr_cpu_id = INTR_CPU_ID_AUTO,
      .intr_flags = ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM
    };

    esp_lcd_touch_config_t touch_config = {
      .x_max = LCD_H_RES,
      .y_max = LCD_V_RES,
      .rst_gpio_num = PIN_LCD_RST,
      .int_gpio_num = GPIO_NUM_NC, //PIN_LCD_TOUCH_IRQ,
      .flags = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = true,
        },
    };
    
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_TOUCH_HOST, &buscfg_touch, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_TOUCH_HOST, &touch_io_config, &touch_io_handle));
    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(touch_io_handle, &touch_config, &touch_handle));
    ESP_LOGI(TAG, "Installed touch controller XPT2046");
    
    return ret;
}

static esp_err_t app_lvgl_init(void)
{
    /* Initialize LVGL */
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 4,         /* LVGL task priority */
        .task_stack = 4096,         /* LVGL task stack size */
        .task_affinity = -1,        /* LVGL task pinned to core (-1 is no affinity) */
        .task_max_sleep_ms = 500,   /* Maximum sleep in LVGL task */
        .timer_period_ms = 2        /* LVGL timer tick period in ms */
    };
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port initialization failed");

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = lcd_panel,
        .buffer_size = LCD_H_RES * LCD_V_RES / 10,
        .double_buffer = 1,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
            .swap_bytes = true,
        }
    };
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);

    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = lvgl_disp,
        .handle = touch_handle,
    };
    lvgl_touch_indev = lvgl_port_add_touch(&touch_cfg);

    return ESP_OK;
}


/* uint32_t get_num() { */
/*     const uint32_t num = (uint32_t)strtol(num_buffer, NULL, 10); */
/*     /\* reset number digits buffer and cursor *\/ */
/*     memset(num_buffer, 0, MAX_NUM_DIGITS); */
/*     num_cursor = 0; */
/*     return num; */
/* } */


static bool is_operator(char p) {
  return (p == '+') || (p == '-') || (p == '*') || (p == '/');
}

static uint8_t op_peso(char op) {
  switch (op) {
  case '+':
  case '-': return 2;
  case '*':
  case '/': return 3;
  }
  return 2;
}

/* shunting yard RPN */
static char** create_stack(const char *txt) {

  
  char num_buffer[MAX_NUM_DIGITS] = {0};
  char op_stack[MAX_STACK_SIZE][1] = {[0][0]=0};
  char data_stack[MAX_STACK_SIZE][MAX_NUM_DIGITS] = {[0][0]=0};
  uint8_t num_cursor = 0;
  uint8_t op_counter = 0;
  uint8_t data_counter = 0;
  
  printf("2txt: %s\n", txt);
  
  for (uint8_t i = 0; txt[i] != '\0'; i++) {
    if (is_operator(txt[i])) {
      printf("i: %c\n", txt[i]);
      if (op_counter > 0) {
	char op = op_stack[op_counter-1][0]; /* top stack op */
	printf("op: %c\n", op);
	uint8_t npeso = op_peso(txt[i]); /* next stack op */
	if (npeso > op_peso(op)) {
	  op_stack[op_counter++][0] = txt[i];
	} else {
	  /* op_stack[op_counter-1][0] = '\0'; */
	  while (npeso < op_peso(op)) {
	    op_counter--;
	    data_stack[data_counter++][0] = op; /* push do top op com peso maior */
	    if (op_counter == 0) break;
	    op_stack[op_counter][0] = op_stack[op_counter-1][0];
	    op_stack[op_counter-1][0] = '\0';
	    op = op_stack[op_counter][0]; /* pop do top op da stack */
	  }
	}
      } else {
	printf("chamo op %c\n", txt[i]);
	op_stack[op_counter++][0] = txt[i];
      }
    } else {
      while(txt[i] != '\0' && !is_operator(txt[i])) {
	printf("char: %c\n", txt[i]);
	num_buffer[num_cursor++] = txt[i];
	i++;
      }
      printf("num_buffer: %s\n", num_buffer);
      strncpy(data_stack[data_counter], num_buffer, MAX_NUM_DIGITS);
      memset(num_buffer, 0, MAX_NUM_DIGITS);
      num_cursor = 0;
      data_counter++;
      i--;
    }
  }

  printf("op stack\n");
  printf("strlen: %d\n", op_counter);
  for (uint8_t i = 0; i <= op_counter; ++i) {
    printf("%c\n", op_stack[i][0]);
  }
  printf("------------------\n");
  
  printf("data stack\n");
  printf("strlen: %d\n", data_counter);
  for (uint8_t i = 0; i < data_counter; ++i) {
    if (strlen(data_stack[i]) == 1) printf("%c\n", data_stack[i][0]);
    else printf("%s\n", data_stack[i]);
  }
  printf("------------------\n");

  return NULL;
}

/* calculator arit exprs */
static void calc_eval_event_handler(lv_event_t *e) {
    lv_obj_t * ta = lv_event_get_user_data(e);
    const char *txt = lv_textarea_get_text(ta);
    printf("txt: %s\n", txt);
    printf("\n\n");
    create_stack(txt);
    //lv_textarea_set_text(ta, ""); /* set result on the screen */
}

static void btnm_event_handler(lv_event_t *e) {
    lv_obj_t * obj = lv_event_get_target(e);
    lv_obj_t * ta = lv_event_get_user_data(e);
    const char *c_txt = lv_buttonmatrix_get_button_text(obj, lv_buttonmatrix_get_selected_button(obj));

    if (lv_strcmp(c_txt, LV_SYMBOL_BACKSPACE) == 0) {
      lv_textarea_delete_char(ta);
      return;
    } else if (lv_strcmp(c_txt, "=") == 0) {
      lv_obj_send_event(ta, CALC_EVENT_EVAL, NULL); /* evaluate */
      return;
    }
    lv_textarea_add_text(ta, c_txt);
}

void create_calc() {
    lv_obj_t * ta = lv_textarea_create(lv_screen_active());
    lv_textarea_set_one_line(ta, true);
    lv_obj_align(ta, LV_ALIGN_TOP_MID, 10, 10);
    /* calc events */
    lv_obj_add_event_cb(ta, calc_eval_event_handler, CALC_EVENT_EVAL, ta);
    lv_obj_add_state(ta, LV_STATE_FOCUSED); /*To be sure the cursor is visible*/

    static const char * btnm_map[] = {"1", "2", "3", "+", "\n",
                                      "4", "5", "6", "-", "\n",
                                      "7", "8", "9", "*", "\n",
				      "0", LV_SYMBOL_BACKSPACE, "=", "/", ""
                                     };

    lv_obj_t * btnm = lv_buttonmatrix_create(lv_screen_active());
    lv_obj_set_style_bg_color(btnm, lv_color_hex(0x249ee8), LV_PART_ITEMS);
    lv_obj_set_size(btnm, 250, 200);
    lv_obj_align(btnm, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_add_event_cb(btnm, btnm_event_handler, LV_EVENT_VALUE_CHANGED, ta);
    lv_obj_remove_flag(btnm, LV_OBJ_FLAG_CLICK_FOCUSABLE); /*To keep the text area focused on button clicks*/
    lv_buttonmatrix_set_map(btnm, btnm_map);
}

/* void create_hello_world(lv_obj_t* scr) { */
/*     lv_obj_t *label = lv_label_create(scr); */
/*     lv_obj_set_width(label, LCD_H_RES); */
/*     lv_obj_set_style_bg_color(scr, lv_color_hex(0xe1bdff), LV_PART_MAIN); */
/*     lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0); */
/*     lv_label_set_text(label, "esp32 Hello world!"); */
/*     lv_obj_align(label, LV_ALIGN_CENTER, 0, -30); */
/* } */

void app_main(void)
{
    printf("ESP32 Hello World!\n");
    /* LCD HW initialization */
    gpio_config_t io_config = {
        .mode = GPIO_MODE_OUTPUT,
	.pin_bit_mask = (1ULL << PIN_LCD_LED)
    };
    
    ESP_ERROR_CHECK(gpio_config(&io_config));
    ESP_ERROR_CHECK(gpio_set_level(PIN_LCD_LED, 1));
    ESP_ERROR_CHECK(init_lcd());
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "Display Hello World!");
    ESP_ERROR_CHECK(app_lvgl_init());

    /* calc events */
    CALC_EVENT_EVAL = lv_event_register_id();
    lvgl_port_lock(0);

    create_calc();
    //create_hello_world(scr);
    lvgl_port_unlock();

    ESP_ERROR_CHECK(esp_lcd_touch_read_data(touch_handle));
    ESP_LOGI(TAG, "Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(300));
    }
}
