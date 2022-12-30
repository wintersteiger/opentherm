// CM Wintersteiger, 2022

#ifndef _OPENTHERM_DS_H_
#define _OPENTHERM_DS_H_

#include <stdint.h>
#include <string.h>

namespace OpenTherm {

class BinarySemaphore {
public:
  BinarySemaphore() {}
  virtual ~BinarySemaphore() = default;

  virtual void acquire_blocking() = 0;
  virtual bool acquire_timeout(uint64_t us) = 0;
  virtual bool release() = 0;
  virtual bool try_acquire() = 0;
};

template <typename T> class Queue {
public:
  Queue(size_t size) {}
  virtual ~Queue() = default;
  virtual bool full() const = 0;
  virtual bool empty() const = 0;
  virtual void add(const T &data) = 0;
  virtual bool try_add(const T &data) = 0;
  virtual T remove() = 0;
  virtual bool try_remove(T &t) = 0;
  virtual size_t level() const = 0;
};

class Timer {
public:
  typedef bool (*callback_t)(Timer *, void *);

  Timer() = default;

  Timer(uint64_t delay_us, uint64_t period_us, callback_t ftick,
        callback_t fstop = nullptr, void *data = nullptr)
      : delay_us(delay_us), period_us(period_us), ftick(ftick), fstop(fstop),
        data(data) {}

  ~Timer() = default;

  virtual void start(uint64_t delay_us = 0) = 0;
  virtual void stop(bool run_fstop = true) = 0;
  virtual bool tick() {
    if (!ftick)
      return false;
    return ftick(this, data);
  }

protected:
  uint64_t delay_us = 0;
  uint64_t period_us = 0;
  bool (*ftick)(Timer *, void *) = nullptr;
  bool (*fstop)(Timer *, void *) = nullptr;
  void *data = nullptr;
};

class Time {
public:
  Time() = default;
  virtual ~Time() = default;
  virtual uint64_t get_us() const = 0;
  virtual void sleep_us(uint64_t) const = 0;
};

}

#endif // _OPENTHERM_DS_H_