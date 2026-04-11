#ifndef LCD_H
#define LCD_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t lcd_init_and_print(const char *text);
esp_err_t lcd_print_message(const char *text);

#ifdef __cplusplus
}
#endif

#endif