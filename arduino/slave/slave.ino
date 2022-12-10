#include <stdio.h>
#include <stdarg.h>
#include <limits.h>

#include "FreeRTOSConfig.h"

#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include <semphr.h>

#include "opentherm/transport.h"

#include "arduino_data_structures.h"
#include "arduino_opentherm_io.h"

using namespace OpenTherm;

static const unsigned slave_in = 3, slave_out = 5;

static FILE uartf = {0};
static SemaphoreHandle_t log_mtx = NULL;
static StaticSemaphore_t log_mtx_state;

static int uart_putchar(char c, FILE *stream)
{
  Serial.write(c);
  return 0;
}

void vlog(const char *fmt, va_list args) {
  if (log_mtx)
    xSemaphoreTake(log_mtx, portMAX_DELAY);
  printf("[%010lu] ", micros());
  vprintf(fmt, args);
  printf("\n");
  fflush(stdout);
  if (log_mtx)
    xSemaphoreGive(log_mtx);
}

void log(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vlog(fmt, args);
  va_end(args);
}


static OpenTherm::Slave<ArduinoTimer, ArduinoSemaphore, ArduinoTime, ArduinoIO, ArduinoQueue, 2> slave({.rx=slave_in, .tx=slave_out});

void rx_task(void *) {
  slave.rx_forever(nullptr, [](bool v){ digitalWrite(LED_BUILTIN, v ? HIGH : LOW); } );
}

static StaticTask_t rx_task_buf;
static StackType_t rx_task_stack[128];

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  log_mtx = xSemaphoreCreateMutexStatic(&log_mtx_state);
  xSemaphoreGive(log_mtx);

  Serial.begin(115200);
  while (!Serial);

  fdev_setup_stream(&uartf, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &uartf;

  log("Slave starting... ");

  xTaskCreateStatic(rx_task, "rx_task", 128, NULL, 1, rx_task_stack, &rx_task_buf);

  vTaskStartScheduler();
}

void loop() {}

extern "C" {
  void vAssertCalled(const char* file, int line) {
    log("%s:%d: ASSERTION FAILED", file, line);
    fflush(stdout);
    // taskDISABLE_INTERRUPTS();
    for( ;; );
  }
}
