#ifndef _ARDUINO_DATA_STRUCTURES_H_
#define _ARDUINO_DATA_STRUCTURES_H_

extern void log(const char*, ...);

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

#endif // _ARDUINO_DATA_STRUCTURES_H_
