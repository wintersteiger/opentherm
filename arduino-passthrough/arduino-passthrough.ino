#include <stdio.h>
#include <stdarg.h>
#include <limits.h>

#include "FreeRTOSConfig.h"

#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include <semphr.h>

#include "opentherm/transport.h"

using namespace OpenTherm;

static const unsigned master_in = 2, master_out = 4;
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

class ArduinoTimer : public OpenTherm::Timer {
public:
  ArduinoTimer() = default;

  ArduinoTimer(uint64_t delay_us, uint64_t period_us, callback_t ftick,
               callback_t fstop = nullptr, void *data = nullptr)
      : OpenTherm::Timer(delay_us, period_us, ftick, fstop, data) {
    if (!(t = xTimerCreateStatic("timer", pdMS_TO_TICKS(period_us)/1000, pdTRUE, this, pcb, &state)))
      log("xTimerCreate failed");
  }

  virtual ~ArduinoTimer() = default;

  virtual void start(uint64_t delay_us = 0) override {
    if (xTimerStart(t, 0) == pdFAIL)
      log("xTimerStart failed");
  }

  virtual void stop(bool run_fstop = true) override {
    if (xTimerStop(t, 0) == pdFAIL)
      log("xTimerStop failed");
  }

protected:
  TimerHandle_t t;
  StaticTimer_t state;

  static void pcb(TimerHandle_t xTimer) {
    ArduinoTimer *at = static_cast<ArduinoTimer *>(pvTimerGetTimerID(xTimer));
    return at->ftick(at, at->data);
  };
};

class ArduinoSemaphore : public OpenTherm::BinarySemaphore {
public:
  ArduinoSemaphore() : OpenTherm::BinarySemaphore() {
    if (!(s = xSemaphoreCreateBinaryStatic(&state)))
      log("xSemaphoreCreateCounting failed");
    xSemaphoreGive(s);
  }
  virtual ~ArduinoSemaphore() = default;

  virtual void acquire_blocking() override { xSemaphoreTake(s, portMAX_DELAY); }
  virtual bool acquire_timeout(uint64_t us) override { return xSemaphoreTake(s, pdMS_TO_TICKS(us)/1000); }
  virtual void release() override { xSemaphoreGive(s); }
  virtual bool try_acquire() override { return xSemaphoreTake(s, 0); }

protected:
  SemaphoreHandle_t s;
  StaticSemaphore_t state;
};

class ArduinoTime : public OpenTherm::Time {
public:
  ArduinoTime() = default;
  virtual ~ArduinoTime() = default;

  virtual uint64_t get_us() const override { return micros(); }
  virtual void sleep_us(uint64_t us) const override { vTaskDelay(pdMS_TO_TICKS(us)/1000); }
};

class ArduinoIO;

static ArduinoIO* io;

class ArduinoIO : public OpenTherm::IO {
public:
  ArduinoIO(unsigned pin_rx = slave_in, unsigned pin_tx = slave_out, bool own_pins = false) :
    pin_rx(pin_rx), pin_tx(pin_tx), rx_state(IDLE), own_pins(own_pins)
  {
    if (own_pins) {
      pinMode(pin_rx, INPUT_PULLUP);
      pinMode(pin_tx, OUTPUT);
    }
    ::io = this;
  }

  virtual ~ArduinoIO() = default;

  virtual void log(const char *fmt, ...) override {
    va_list args;
    va_start(args, fmt);
    ::vlog(fmt, args);
    va_end(args);
  }

  using IO::send;

  void send_bit(bool v) const {
    digitalWrite(pin_tx, v ? HIGH : LOW);
    delayMicroseconds(500);
    digitalWrite(pin_tx, v ? LOW : HIGH);
    delayMicroseconds(500);
  }

  virtual void send(const Frame &f) override {
    send_bit(true);
    uint32_t x = (uint32_t)f;
    for (size_t i = 0; i < 32; i++) {
      send_bit((x & 0x80000000) != 0);
      x <<= 1;
    }
    send_bit(true);
  }

  virtual Frame get_blocking() override {
    waiting_task = xTaskGetCurrentTaskHandle();
    if (own_pins)
      attachInterrupt(digitalPinToInterrupt(pin_rx), internal_isr, CHANGE);
    while (rx_state != COMPLETE)
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (own_pins)
      detachInterrupt(digitalPinToInterrupt(pin_rx));
    rx_state = IDLE;
    return Frame(frame);
  }

  // Without pin ownership, we rely on other ISRs to call isr().
  void isr() { rx_callback(this); }

protected:
  const unsigned pin_rx, pin_tx;
  const bool own_pins;

  enum RXState { IDLE, START, DATA, STOP, COMPLETE };
  volatile RXState rx_state = IDLE;
  volatile uint32_t frame = 0;
  volatile unsigned long prev_time = 0;
  volatile TaskHandle_t waiting_task;

  static void internal_isr() { rx_callback(::io); }

  static void rx_callback(ArduinoIO *io) {
    bool rising = digitalRead(io->pin_rx);
    unsigned long time = micros();
    signed long delta = time - io->prev_time;

    if (delta < 0)
      delta += ULONG_MAX;

    switch (io->rx_state) {
      case IDLE:
        if (rising) {
          io->rx_state = START;
          io->prev_time = time;
        }
        break;
      case START:
        if (delta > 750) {
          ::log("frame dropped in START: %d", delta);
          io->rx_state = IDLE;
        }
        else if (!rising) {
          io->frame = 0x01;
          io->rx_state = DATA;
        }
        else
          io->rx_state = IDLE;
        io->prev_time = time;
        break;
      case DATA:
        if (delta > 1500) {
          ::log("frame dropped in DATA: %08x", io->frame);
          io->rx_state = IDLE;
          io->prev_time = time;
        }
        else if (delta >= 750) {
          bool is_last = (io->frame & 0x80000000) != 0;
          io->frame = (io->frame << 1) | (rising ? 0 : 1);
          if (is_last)
            io->rx_state = STOP;
          io->prev_time = time;
        }
        break;
      case STOP:
        if (delta > 1500 || rising) {
          ::log("frame dropped in STOP: %08x", io->frame);
          io->rx_state = IDLE;
        }
        else
          io->rx_state = COMPLETE;
        io->prev_time = time;
        break;
      default:
        ::log("unknown RX state: %d", io->rx_state);
        io->rx_state = IDLE;
        io->prev_time = time;
    }

    if (io->rx_state == IDLE || io->rx_state == COMPLETE) {
      if (io->waiting_task)
        vTaskNotifyGiveFromISR(io->waiting_task, NULL);
    }
  }
};

template <typename T>
class ArduinoQueue : public OpenTherm::Queue<T> {
public:
  static const constexpr size_t queue_size = 2;

  ArduinoQueue(size_t size = queue_size) : OpenTherm::Queue<T>(size) {
    if (!(q = xQueueCreateStatic(size, sizeof(T), storage, &state)))
      log("xQueueCreate failed");
  }
  virtual ~ArduinoQueue() { vQueueDelete(q); }

  virtual bool full() const override { return xQueueIsQueueFullFromISR(q); }
  virtual bool empty() const override { return xQueueIsQueueEmptyFromISR(q); }
  virtual void add(const T &data) override { xQueueSend(q, &data, portMAX_DELAY); }
  virtual bool try_add(const T &data) override { xQueueSend(q, &data, 0); }
  virtual T remove() override { T r; xQueueReceive(q, &r, portMAX_DELAY); return r; }
  virtual bool try_remove(T &t) override { return xQueueReceive(q, &t, 0);}
  virtual size_t level() const override { return uxQueueMessagesWaiting(q); }

protected:
  QueueHandle_t q;
  StaticQueue_t state;
  uint8_t storage[queue_size * sizeof(T)];
};

class Listener : public Device<ArduinoTimer, ArduinoSemaphore, ArduinoTime, ArduinoIO, ArduinoQueue, 2>
{
  using DeviceT = Device<ArduinoTimer, ArduinoSemaphore, ArduinoTime, ArduinoIO, ArduinoQueue, 2>;
public:
  Listener(const Pins &pins, const char* name) : DeviceT(pins), name(name) {}
  virtual ~Listener() = default;

  virtual void start() override {}

  virtual void process(const Frame &f) override {
    io.log("%s: %08x", name, (uint32_t)f);
  }

protected:
  const char *name;
};

static Listener slave_listener({.rx=slave_in, .tx=slave_out}, "S");
static Listener master_listener({.rx=master_in, .tx=master_out}, "M");

void slave2master_isr() {
  digitalWrite(master_out, digitalRead(slave_in));
  slave_listener.io.isr();
}

void master2slave_isr() {
  digitalWrite(slave_out, digitalRead(master_in));
  master_listener.io.isr();
}

void slave_rx_task(void *) {
  slave_listener.rx_forever(nullptr, [](bool v){ digitalWrite(LED_BUILTIN, v ? HIGH : LOW); } );
}

void master_rx_task(void *) {
  master_listener.rx_forever(nullptr, [](bool v){} );
}

static StaticTask_t slave_rx_task_buf;
static StackType_t slave_rx_task_stack[128];

static StaticTask_t master_rx_task_buf;
static StackType_t master_rx_task_stack[128];

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  log_mtx = xSemaphoreCreateMutexStatic(&log_mtx_state);
  xSemaphoreGive(log_mtx);

  pinMode(master_in, INPUT_PULLUP);
  pinMode(master_out, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(master_in), master2slave_isr, CHANGE);

  pinMode(slave_in, INPUT_PULLUP);
  pinMode(slave_out, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(slave_in), slave2master_isr, CHANGE);

  Serial.begin(115200);
  while (!Serial);

  fdev_setup_stream(&uartf, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &uartf;

  log("Passthrough starting... ");

  log("sizeof(...): listener=%u timer=%u semaphore=%u", sizeof(decltype(slave_listener)), sizeof(ArduinoTimer), sizeof(ArduinoSemaphore));

  xTaskCreateStatic(slave_rx_task, "slave_rx_task", 128, NULL, 1, slave_rx_task_stack, &slave_rx_task_buf);
  xTaskCreateStatic(master_rx_task, "master_rx_task", 128, NULL, 1, master_rx_task_stack, &master_rx_task_buf);

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
