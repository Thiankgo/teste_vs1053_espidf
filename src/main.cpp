#include <stdio.h>
#include <string>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mp3_2x_NB.h"
#include "vs1053.h"

#define AUDIO_MISO 37
#define AUDIO_MOSI 35
#define AUDIO_CLK 36
#define AUDIO_DC (gpio_num_t)41
#define AUDIO_CS (gpio_num_t)40
#define AUDIO_RESET (gpio_num_t)42
#define DREQ (gpio_num_t)39

void println(const char *message) {
  printf("%s\n", message);
}

void print(const char *message) {
  printf("%s", message);
}

VS1053_t *_audio = new VS1053_t(AUDIO_CLK,AUDIO_MOSI,AUDIO_MISO,AUDIO_CS,AUDIO_DC, DREQ, AUDIO_RESET);

extern "C" {
void app_main(void);
}

void app_main() {
  gpio_set_direction(GPIO_NUM_45, GPIO_MODE_OUTPUT);
  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };

  uart_param_config(UART_NUM_0, &uart_config);
  uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);

  while (!uart_is_driver_installed(UART_NUM_0)) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  println("init");

  gpio_set_level(GPIO_NUM_45, 1);
  vTaskDelay(pdMS_TO_TICKS(20));
  gpio_set_level(GPIO_NUM_45, 0);

  _audio->begin();
  _audio->switchToMp3Mode();
  _audio->setVolume(80);


  gpio_set_level(GPIO_NUM_45, 1);
  vTaskDelay(pdMS_TO_TICKS(20));
  gpio_set_level(GPIO_NUM_45, 0);

  while (1) {
		_audio->playChunk((uint8_t *)casa_mp3, casa_mp3_len);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}