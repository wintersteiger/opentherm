#ifndef _ARDUINO_OPENTHERM_IO_H_
#define _ARDUINO_OPENTHERM_IO_H_

#include <stdio.h>
#include <stdarg.h>

#include <opentherm/transport.h>

extern void vlog(const char *fmt, va_list args);

class ArduinoIO;

static ArduinoIO* io;

class ArduinoIO : public OpenTherm::IO {
public:
  ArduinoIO(unsigned pin_rx, unsigned pin_tx, bool own_pins = false) :
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

  virtual void send(const OpenTherm::Frame &f) override {
    send_bit(true);
    uint32_t x = (uint32_t)f;
    for (size_t i = 0; i < 32; i++) {
      send_bit((x & 0x80000000) != 0);
      x <<= 1;
    }
    send_bit(true);
  }

  virtual OpenTherm::Frame get_blocking() override {
    waiting_task = xTaskGetCurrentTaskHandle();
    if (own_pins)
      attachInterrupt(digitalPinToInterrupt(pin_rx), internal_isr, CHANGE);
    while (rx_state != COMPLETE)
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (own_pins)
      detachInterrupt(digitalPinToInterrupt(pin_rx));
    rx_state = IDLE;
    return OpenTherm::Frame(frame);
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

#endif // _ARDUINO_OPENTHERM_IO_H_