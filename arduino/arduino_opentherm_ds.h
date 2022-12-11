#ifndef _ARDUINO_DATA_STRUCTURES_H_
#define _ARDUINO_DATA_STRUCTURES_H_

#include <opentherm/transport.h>

extern void log(const char*, ...);
extern void vlog(const char *fmt, va_list args);

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

class ArduinoIO;
extern ArduinoIO* io;

class ArduinoIO : public OpenTherm::IO {
public:
  ArduinoIO(const OpenTherm::Pins &pins) :
    OpenTherm::IO(pins),
    rx_state(IDLE)
  {
    if (pins.owned) {
      pinMode(pins.rx, INPUT_PULLUP);
      pinMode(pins.tx, OUTPUT);
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
    // Slave adapter inverts, apparently.
    digitalWrite(pins.tx, !v ? HIGH : LOW);
    delayMicroseconds(487);
    digitalWrite(pins.tx, !v ? LOW : HIGH);
    delayMicroseconds(487);
  }

  virtual void send(const OpenTherm::Frame &f) override {
    uint32_t x = (uint32_t)f;
    send_bit(true);
    for (size_t i = 0; i < 32; i++) {
      send_bit((x & 0x80000000) != 0);
      x <<= 1;
    }
    send_bit(true);
  }

  virtual OpenTherm::Frame get_blocking() override {
    waiting_task = xTaskGetCurrentTaskHandle();
    if (pins.owned)
      attachInterrupt(digitalPinToInterrupt(pins.rx), internal_isr, CHANGE);
    while (rx_state != COMPLETE)
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (pins.owned)
      detachInterrupt(digitalPinToInterrupt(pins.rx));
    rx_state = IDLE;
    return OpenTherm::Frame(frame);
  }

  // Without pin ownership, we rely on other ISRs to call isr().
  void isr() { rx_callback(this); }

protected:
  enum RXState { IDLE, START, DATA, STOP, COMPLETE };
  volatile RXState rx_state = IDLE;
  volatile uint32_t frame = 0;
  volatile unsigned long prev_time = 0;
  volatile TaskHandle_t waiting_task;

  static void internal_isr() { rx_callback(::io); }

  static void rx_callback(ArduinoIO *io) {
    bool rising = digitalRead(io->pins.rx);
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
        if (delta > 1500 || (delta > 750 && rising) || (delta <= 750 && !rising)) {
          ::log("frame dropped in STOP: %08x", io->frame);
          io->rx_state = IDLE;
        }
        else
          io->rx_state = COMPLETE;
        io->prev_time = time;
        break;
      case COMPLETE:
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

#endif // _ARDUINO_DATA_STRUCTURES_H_
