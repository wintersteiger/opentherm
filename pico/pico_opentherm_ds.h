#ifndef _PICO_OPENTHERM_DS_H_
#define _PICO_OPENTHERM_DS_H_

#include <hardware/pio.h>
#include <hardware/timer.h>
#include <pico/util/queue.h>

#include <opentherm/transport.h>

#include "opentherm.pio.h"

extern void vllog(const char *fmt, va_list args);
extern void llog(const char *fmt, ...);

class PicoTimer : public OpenTherm::Timer {
public:
  PicoTimer() = default;

  PicoTimer(uint64_t delay_us, uint64_t period_us, callback_t ftick,
            callback_t fstop = nullptr, void *data = nullptr)
      : Timer(delay_us, period_us, ftick, fstop, data) {
    if (!alarm_pool)
      alarm_pool = alarm_pool_create(2, 16);
  }

  virtual ~PicoTimer() {
    if (running)
      stop(true);
  }

  virtual void start(uint64_t delay_us = 0) override {
    stop(false);

    this->delay_us = delay_us;
    running = true;

    if (delay_us == 0) {
      if (!alarm_pool_add_repeating_timer_us(alarm_pool, -period_us, pcb, this,
                                             &t))
        llog("No alarm slots available!");
    } else {
      alarm_id =
          alarm_pool_add_alarm_in_us(alarm_pool, delay_us, acb, this, true);
    }
  }

  virtual void stop(bool run_fstop = true) override {
    running = false;
    cancel_repeating_timer(&t);
    if (alarm_id != -1) {
      alarm_pool_cancel_alarm(alarm_pool, alarm_id);
      alarm_id = -1;
    }
    if (run_fstop && fstop)
      fstop(this, data);
  }

protected:
  bool running = false;
  static alarm_pool_t *alarm_pool;
  repeating_timer_t t = {0};
  volatile alarm_id_t alarm_id = -1;

  static bool pcb(repeating_timer_t *rt) {
    PicoTimer *pt = static_cast<PicoTimer *>(rt->user_data);
    return pt->ftick(pt, pt->data);
  };

  static int64_t acb(alarm_id_t id, void *data) {
    PicoTimer *pt = static_cast<PicoTimer *>(data);
    if (pt->alarm_id != id)
      return 0;
    alarm_pool_cancel_alarm(alarm_pool, id);
    pt->alarm_id = -1;
    if (pt->period_us != 0) {
      alarm_pool_add_repeating_timer_us(alarm_pool, -pt->period_us, pcb, pt,
                                        &pt->t);
    } else
      pt->running = false;
    pt->ftick(pt, pt->data);
    return 0;
  }
};

class PicoSemaphore : public OpenTherm::BinarySemaphore {
public:
  PicoSemaphore() : OpenTherm::BinarySemaphore() { sem_init(&sem, 1, 1); }
  virtual ~PicoSemaphore() = default;
  virtual void acquire_blocking() override { sem_acquire_blocking(&sem); }
  virtual bool try_acquire() override { return sem_try_acquire(&sem); }
  virtual bool release() override { return sem_release(&sem); }
  virtual bool acquire_timeout(uint64_t us) override {
    return sem_acquire_timeout_us(&sem, us);
  }

protected:
  semaphore_t sem;
};

template <typename T> class PicoQueue : public OpenTherm::Queue<T> {
public:
  PicoQueue(uint size) : OpenTherm::Queue<T>(size) {
    queue_init(&q, sizeof(T), size);
  }
  virtual ~PicoQueue() { queue_free(&q); };
  virtual bool full() const override {
    return queue_is_full(const_cast<queue_t *>(&q));
  }
  virtual bool empty() const override {
    return queue_is_empty(const_cast<queue_t *>(&q));
  }
  virtual void add(const T &data) override { queue_add_blocking(&q, &data); }
  virtual bool try_add(const T &data) override {
    return queue_try_add(&q, &data);
  }
  virtual T remove() override {
    T r;
    queue_remove_blocking(&q, &r);
    return r;
  }
  virtual bool try_remove(T &t) override { return queue_try_remove(&q, &t); }
  virtual size_t level() const override {
    return queue_get_level(const_cast<queue_t *>(&q));
  }

protected:
  queue_t q;
};

class PicoTime : public OpenTherm::Time {
public:
  PicoTime() : Time() {}
  virtual ~PicoTime() = default;
  virtual uint64_t get_us() const override { return time_us_64(); }
  virtual void sleep_us(uint64_t us) const override {
    return ::busy_wait_us(us);
  }
};

class PicoIO : public OpenTherm::IO {
public:
  PicoIO(const OpenTherm::Pins &pins) : OpenTherm::IO(pins) {
    sm_tx = pio_claim_unused_sm(pio, true);
    uint offset_tx = pio_add_program(pio, &opentherm_tx_inv_program);
    opentherm_tx_inv_init(pio, sm_tx, offset_tx, pins.tx);

    sm_rx = pio_claim_unused_sm(pio, true);
    uint offset_rx = pio_add_program(pio, &opentherm_rx_program);
    opentherm_rx_init(pio, sm_rx, offset_rx, pins.rx);
  }

  virtual ~PicoIO() = default;

  virtual void send(const OpenTherm::Frame &f) override {
    pio_sm_put(pio0, sm_tx, (uint32_t)f);
  }

  using IO::send;

  virtual OpenTherm::Frame get_blocking() override {
    return pio_sm_get_blocking(pio, sm_rx);
  }

  virtual void log(const char *fmt, ...) override {
    va_list args;
    va_start(args, fmt);
    ::vllog(fmt, args);
    va_end(args);
  }

  float clk_div() const { return pio0->sm[sm_tx].clkdiv / 1e6; }

protected:
  PIO pio = pio0;
  unsigned sm_tx, sm_rx;
};

#endif
