#include <stdio.h>
#include <stdarg.h>
#include <limits.h>

#include "FreeRTOSConfig.h"

#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include <semphr.h>

#include <opentherm/transport.h>
#include <opentherm/application.h>

#include "arduino_opentherm_ds.h"

using namespace OpenTherm;

ArduinoIO* io = NULL;
Application::IDMeta Application::idmeta[256] = {0};

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

class SlaveApp : public Application {
public:
  SlaveApp(const Pins &pins) : Application(device), device(pins) {
    device.set_frame_callback(Application::sprocess, this);

    sconfig_smemberid = (uint16_t)0x0300;
    ot_version_master = 0.0f;
    ot_version_slave = 2.2f;
  }

  virtual ~SlaveApp() = default;

  virtual void on_read(uint8_t data_id, uint16_t data_value = 0x0000) override {
    ::log("Read-Data(%d, %04x)", data_id, data_value);
    Application::on_read(data_id, data_value);
  }

  virtual void on_write(uint8_t data_id, uint16_t data_value) override {
    ::log("Write-Data(%d, %04x)", data_id, data_value);
    Application::on_write(data_id, data_value);
  }

  virtual void on_invalid_data(uint8_t data_id, uint16_t data_value) override {
    ::log("Invalid-Data(%d, %04x)", data_id, data_value);
    Application::on_invalid_data(data_id, data_value);
  }

  virtual void run() override {
    device.rx_forever([](bool v){ digitalWrite(LED_BUILTIN, v ? HIGH : LOW); } );
  }

protected:
  Slave<ArduinoTimer, ArduinoSemaphore, ArduinoTime, ArduinoQueue, ArduinoIO, 2> device;
};

static SlaveApp app({.rx=3, .tx=5, .owned=true});

static void rx_task(void *) { app.run(); }

static StaticTask_t rx_task_buf;
static StackType_t rx_task_stack[128];

void setup()
{
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  log_mtx = xSemaphoreCreateMutexStatic(&log_mtx_state);
  xSemaphoreGive(log_mtx);

  fdev_setup_stream(&uartf, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &uartf;

  log("Slave starting... ");
  log("sizeof(app)=%d", sizeof(SlaveApp));

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
