// CM Wintersteiger, 2022

#ifndef _OPENTHERM_H_
#define _OPENTHERM_H_

#include <inttypes.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// OpenTherm 2.2 transport layer

#ifndef PRIu64
#define PRIu64 "%llu"
#endif

namespace OpenTherm {

class Frame;

class BinarySemaphore {
public:
  BinarySemaphore() {}
  virtual ~BinarySemaphore() = default;

  virtual void acquire_blocking() = 0;
  virtual bool acquire_timeout(uint64_t us) = 0;
  virtual void release() = 0;
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
  virtual void tick() {
    if (ftick)
      ftick(this, data);
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

class IO {
public:
  IO() = default;
  virtual ~IO() = default;
  virtual void send(const Frame &) = 0;
  virtual void send(Frame &&f) { send(f); }
  virtual Frame get_blocking() = 0;
  virtual void log(const char *fmt, ...) = 0;
};

enum MsgType : uint8_t {
  // Master-to-Slave
  ReadData = 0b000,
  WriteData = 0b001,
  InvalidData = 0b010,
  // 0b011 reserved
  // Slave-to-Master
  ReadACK = 0b100,
  WriteACK = 0b101,
  DataInvalid = 0b110,
  UnknownDataID = 0b111
};

struct Frame {
  uint32_t data = 0;

  Frame() = default;

  Frame(uint32_t data) : data(data) {}

  Frame(MsgType msg_type, uint8_t id, uint16_t value) {
    data = (static_cast<uint8_t>(msg_type) << 28) | (id << 16) | value;
  }

  MsgType msg_type() const { return static_cast<MsgType>((data >> 28) & 0x07); }
  uint8_t id() const { return (data >> 16) & 0xFF; }
  uint16_t value() const { return data & 0xFFFF; }
  uint8_t data_byte_1() const { return (data >> 8) & 0xFF; }
  uint8_t data_byte_2() const { return data & 0xFF; }

  bool parity() const {
    bool r = false;
    uint32_t t = data & 0x7FFFFFFF;
    while (t != 0) {
      if (t % 2)
        r = !r;
      t >>= 1;
    }
    return r;
  }

  bool parity_ok() const {
    return parity() == (data >> 31);
  }

  operator uint32_t() const { return (data & 0x70FFFFFF) | parity() << 31; }
};

using RequestID = uint64_t;
static constexpr RequestID NoRequestID = UINT64_MAX;

enum class RequestStatus { OK = 0, TIMED_OUT, SKIPPED, ERROR, PENDING };

struct Request {
  Request(RequestID id = NoRequestID, bool skip_if_busy = false, Frame f = {0},
          void (*callback)(RequestStatus, const Frame &) = nullptr)
      : id(id), skip_if_busy(skip_if_busy), f(f), callback(callback) {}
  virtual ~Request() = default;

  RequestID id = NoRequestID;
  bool skip_if_busy = false;
  Frame f = {0};
  void (*callback)(RequestStatus, const Frame &) = nullptr;
};

struct Response {
  RequestID id = NoRequestID;
  Frame f = {0};
  RequestStatus status = RequestStatus::ERROR;
};

struct Pins {
  unsigned rx;
  unsigned tx;
};

class DeviceBase {
public:
  DeviceBase() = default;
  virtual ~DeviceBase() = default;
};

template <typename TimerType, typename SemaphoreType, typename TimeType,
          typename IOType, template <typename> typename QueueType,
          size_t max_concurrent_requests = 16>
class Device : public DeviceBase {
public:
  Device(const Pins &pins) :
    io(pins.rx, pins.tx),
    rx_frame_count(0),
    next_request_id(0),
    requests(max_concurrent_requests)
  {}

  virtual ~Device(){};

  uint8_t status = 0x00;
  SemaphoreType tx_sem, rx_sem;
  TimeType time;
  IOType io;
  Frame rx_last;
  uint64_t rx_frame_count = 0;
  RequestID next_request_id = 0;
  QueueType<Request> requests;
  Response responses[max_concurrent_requests];

  unsigned num_outstanding_requests() const { return requests.level(); }

  virtual void start() = 0;
  virtual void process(const Frame &f) = 0;

  Frame rx_once() __attribute__((noinline)) {
    rx_sem.acquire_blocking();
    rx_last = io.get_blocking();
    rx_frame_count++;
    rx_sem.release();
    return rx_last;
  }

  void rx_forever(void (*callback)(const Frame &) = nullptr,
                  void (*blink)(bool) = nullptr) __attribute__((noinline)) {
    while (true) {
      Frame f = rx_once();
      if (blink)
        blink(true);
      process(f);
      if (callback)
        callback(f);
      if (blink)
        blink(false);
    }
  }

  bool is_finished(uint64_t rid) {
    Response &r = responses[rid % max_concurrent_requests];
    return r.status != RequestStatus::PENDING;
  }

  RequestStatus get(uint64_t rid, Frame &f) {
    Response &r = responses[rid % max_concurrent_requests];
    if (r.id != rid)
      io.log("warning: outdated response id %" PRIu64, rid);
    if (r.status != RequestStatus::PENDING)
      f = r.f;
    return r.status;
  }

  RequestID tx(Frame f, bool skip_if_busy = false,
               void (*callback)(RequestStatus, const Frame &) = nullptr)
      __attribute__((noinline)) {
    if (next_request_id == NoRequestID)
      next_request_id++;
    Request req(next_request_id++, skip_if_busy, f, callback);
    Response &rsp = responses[req.id % max_concurrent_requests];
    rsp.id = req.id;
    rsp.status = RequestStatus::PENDING;
    if (!requests.try_add(req)) {
      rsp.status = RequestStatus::ERROR;
      return NoRequestID;
    } else
      return req.id;
  }

  void tx_forever() {
    Frame reply;
    while (true) {
      auto req = requests.remove();

      Response &r = responses[req.id % max_concurrent_requests];
      r.id = req.id;
      if (!converse(req, r.f))
        r.status = req.skip_if_busy ? RequestStatus::SKIPPED
                                    : RequestStatus::TIMED_OUT;
      else
        r.status = RequestStatus::OK;

      if (req.callback) {
        io.log("tx_forever: callback for rid=%lu", req.id);
        req.callback(r.status, r.f);
      }
    }
  }
};

template <typename TimerType, typename SemaphoreType, typename TimeType,
          typename IOType, template <typename> typename QueueType,
          size_t max_concurrent_requests = 16>
class Master : public Device<TimerType, SemaphoreType, TimeType, IOType, QueueType, max_concurrent_requests>
{
  using DeviceT = Device<TimerType, SemaphoreType, TimeType, IOType, QueueType, max_concurrent_requests>;
  using DeviceT::io;
  using DeviceT::tx_sem;
  using DeviceT::rx_sem;
  using DeviceT::rx_last;
  using DeviceT::time;

public:
  using DeviceT::rx_once;
  using DeviceT::rx_forever;
  using DeviceT::tx_forever;
  using DeviceT::tx;

  Master(const Pins &pins) : DeviceT(pins) {
      master_timer = TimerType(
          0, 1000000,
          [](Timer *, void *data) {
            DeviceT *d = (DeviceT*)data;
            d->tx(Frame(OpenTherm::MsgType::ReadData, 0x00, d->status << 8),
                  true);
            return true;
          },
          nullptr, this);

      plus_check_timer = TimerType(
          0, 20000000,
          [](Timer *timer, void *data) {
            timer->stop();
            return true;
          },
          [](Timer *timer, void *data) {
            DeviceT *d = (DeviceT*)data;
            if (d->rx_frame_count == 0) {
              d->io.log(
                  "No OpenTherm/plus reply after startup; the slave does not "
                  "seem to support OpenTherm/plus.");
#ifdef NDEBUG
              d->io.log("OpenTherm/- not supported; giving up.");
              d->master_timer.stop();
#else
              d->io.log("Not giving up, though.");
#endif
            } else
              d->io.log("OpenTherm/plus ok (%" PRIu64 ").", d->rx_frame_count);
            return true;
          },
          this);

      tx_sem_release_timer = TimerType(
          0, 0,
          [](Timer *timer, void *data) {
            DeviceT *d = (DeviceT *)data;
            timer->stop(false);
            d->tx_sem.release();
            return true;
          },
          nullptr, this);
    }

  virtual ~Master() = default;

  virtual void start() override {
      master_timer.start();
      plus_check_timer.start();
  }

  bool converse(const Request &req, Frame &reply) __attribute__((noinline)) {
    volatile uint64_t frame_time = 0;
    volatile bool acquired = false;

    if (req.skip_if_busy && !tx_sem.try_acquire())
      return false;

    master_timer.stop(false);

    {
      if (!req.skip_if_busy)
        tx_sem.acquire_blocking();

      uint64_t start_time = time.get_us();
      io.send(req.f);
      // After 34 ms frame time, at least 20ms before we can expect a reply.
      // Max 1.15 sec between sends.
      acquired = rx_sem.acquire_timeout((1150 - 34) * 1000);
      frame_time = time.get_us() - start_time;

      if (acquired) {
        reply = rx_last;
        rx_sem.release();
        // At least 100 ms before the next send.
        tx_sem_release_timer.start(100000);
        frame_time += 100000;
      } else
        tx_sem.release();
    }

    uint64_t next_master = 0;
    if (acquired && frame_time < 1000000)
      next_master = 1000000 - frame_time;
    else {
      if (acquired)
        io.log("TX/RX (%" PRIu64 "): took too long (%" PRIu64 "us)", req.id,
               frame_time);
      else
        io.log("TX/RX (%" PRIu64 "): timed out", req.id);
      master_timer.tick();
    }
    master_timer.start(next_master);

    return acquired;
  }

  virtual void process(const Frame &f) override
  {
    if (f.parity_ok()) {
      io.log("Slave: process %08x", (uint32_t)f);
      switch (f.msg_type()) {
        case ReadACK:
          if (f.id() == 0) {
            auto nos = f.data_byte_2();
            if (nos != slave_status) {
              io.log("New slave status: %02x", nos);
              slave_status = nos;
            }
          }
          break;
        case WriteACK: break;
        case DataInvalid: break;
        case UnknownDataID: break;
        default:
          io.log("unexpected message type: %d", f.msg_type());
      }
    }
  }

protected:
  uint8_t slave_status = 0x00;
  TimerType master_timer, plus_check_timer, tx_sem_release_timer;
};

template <typename TimerType, typename SemaphoreType, typename TimeType,
          typename IOType, template <typename> typename QueueType,
          size_t max_concurrent_requests = 16>
class Slave
    : public Device<TimerType, SemaphoreType, TimeType, IOType, QueueType, max_concurrent_requests>
{
  using DeviceT = Device<TimerType, SemaphoreType, TimeType, IOType, QueueType, max_concurrent_requests>;
public:
  using DeviceT::rx_once;
  using DeviceT::rx_forever;
  using DeviceT::tx_forever;
  using DeviceT::tx;
  using DeviceT::io;

  Slave(const Pins &pins)
      : Device<TimerType, SemaphoreType, TimeType, IOType, QueueType, max_concurrent_requests>(pins) {}
  virtual ~Slave() = default;

  virtual void start() override {}

  virtual void process(const Frame &f) override {
    if (f.parity_ok()) {
      io.log("Slave: process %08x", (uint32_t)f);
      switch (f.msg_type()) {
        case ReadData: break;
        case WriteData: break;
        case InvalidData: break;
        default:
          io.log("unexpected message type: %d", f.msg_type());
      }
    }
  }

protected:
  uint8_t master_status;
};

} // namespace OpenTherm

#endif // _OPENTHERM_H_
