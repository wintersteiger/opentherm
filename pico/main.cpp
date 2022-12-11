#include <boards/pico_w.h>

#include <hardware/adc.h>
#include <hardware/clocks.h>
#include <hardware/pll.h>
#include <pico/cyw43_arch.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <pico/unique_id.h>

#include <lwip/apps/fs.h>
#include <lwip/apps/httpd.h>
#include <lwip/apps/mqtt.h>
#include <lwip/tcpip.h>

#include <opentherm/transport.h>
#include <opentherm/application.h>

#include "pico_opentherm_ds.h"

using namespace OpenTherm;

alarm_pool_t *PicoTimer::alarm_pool = NULL;
Application::IDMeta Application::idmeta[256] = {0};

auto_init_mutex(log_mtx);

void vlog(const char *fmt, va_list args) {
  mutex_enter_blocking(&log_mtx);
  printf("[%010llu] ", time_us_64());
  vprintf(fmt, args);
  printf("\n");
  fflush(stdout);
  mutex_exit(&log_mtx);
}

void log(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vlog(fmt, args);
  va_end(args);
}

float get_core_temperature() {
  static const float conversion_factor = 3.3f / (1 << 12);
  adc_select_input(4);
  uint16_t traw = adc_read();
  float v = traw * conversion_factor;
  return 27.0 - (v - 0.706) / 0.001721;
}

static mqtt_client_t *mqtt_client = NULL;
static struct mqtt_connect_client_info_t mqtt_client_info;
static const char *mqtt_topic = "OpenThermOstat";
static const char *mqtt_cmd_in_topic = "OpenThermOstat/command/in";
static const char *mqtt_cmd_out_topic = "OpenThermOstat/command/out";
auto_init_mutex(mqtt_mtx);

bool mosq_cmd_triggered = false;
char mosq_cmd_hex[9];
size_t mosq_data_arrived = 0;

static Master<PicoTimer, PicoSemaphore, PicoTime, PicoQueue, PicoIO> otdev({.rx = 27, .tx = 26, .owned = true});

class MyApp : public Application
{
public:
  MyApp(const Pins &pins) : Application(otdev)  {
    otdev.set_frame_callback(Application::sprocess, this);

    sconfig_smemberid = (uint16_t)0x0000;
    ot_version_master = 2.2f;
    ot_version_slave = 0.0f;
  }

  virtual ~MyApp() = default;

  virtual void run() override {}

  virtual void on_read_ack(uint8_t data_id, uint16_t data_value) override {
    ::log("Read-Ack(%d, %04x)", data_id, data_value);
    Application::on_read_ack(data_id, data_value);
    slave_data[data_id] = data_value;
  }

  virtual void on_write_ack(uint8_t data_id, uint16_t data_value) override {
    ::log("Write-Ack(%d, %04x)", data_id, data_value);
    Application::on_write_ack(data_id, data_value);
    slave_data[data_id] = data_value;
  }

  virtual void on_data_invalid(uint8_t data_id, uint16_t data_value) override {
    ::log("Data-Invalid(%d, %04x)", data_id, data_value);
    Application::on_data_invalid(data_id, data_value);
  }

  virtual void on_unknown_data_id(uint8_t data_id, uint16_t data_value) override {
    ::log("Unknown-DataID(%d, %04x)", data_id, data_value);
    Application::on_unknown_data_id(data_id, data_value);
  }

  virtual RequestID tx(const Frame & f, bool skip_if_busy = false, void (*callback)(RequestStatus, const Frame &) = nullptr) {
    return otdev.tx(f, skip_if_busy, callback);
  }

protected:
  uint16_t slave_data[256] = {0};
};

MyApp app({.rx = 27, .tx = 26, .owned = true});

static void mosq_publish_cb(void *arg, const char *topic, u32_t tot_len) {
  // log("mqtt_incoming_publish_cb_t: %d", tot_len);
  if (strcmp(topic, mqtt_cmd_in_topic) == 0) {
    if (mosq_cmd_triggered) {
      log("error: busy with previous command");
      return;
    }
    mosq_cmd_triggered = true;
    mosq_data_arrived = 0;
  }
}

static void mosq_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
  // log("mqtt_incoming_data_cb_t: %d", len);
  if (mosq_cmd_triggered) {
    for (uint16_t i = 0; i < len; i++) {
      mosq_cmd_hex[mosq_data_arrived++] = data[i];
      if (mosq_data_arrived >= 8) {
        mosq_cmd_hex[mosq_data_arrived] = 0;
        break;
      }
    }
    if (mosq_data_arrived >= 8) {
      uint32_t req_msg;
      if (sscanf(mosq_cmd_hex, "%08lx", &req_msg) != 1) {
        log("error: invalid input; not a hex-encoded OpenTherm frame");
        return;
      }
      Frame rsp;
      RequestID rid =
          app.tx(req_msg, false, [](RequestStatus s, const Frame &f) {
            char rsp[9] = "";
            snprintf(rsp, sizeof(rsp), "%08lx", (uint32_t)f);
            mutex_enter_blocking(&mqtt_mtx);
            if (mqtt_client && mqtt_client_is_connected(mqtt_client))
              mqtt_publish(
                  mqtt_client, mqtt_cmd_out_topic, rsp, 8, 0, 0,
                  [](void *arg, err_t err) {
                    if (err != ERR_OK)
                      log("mqtt_publish failed: %d", err);
                  },
                  NULL);
            else
              log("error: mqtt client not connected");
            mutex_exit(&mqtt_mtx);
          });
      log("mosq_data_cb: rid=%llu", rid);
      if (rid == NoRequestID)
        log("error: failure to submit request (queue full)");
    }

    mosq_cmd_triggered = false;
    mosq_data_arrived = 0;
  }
}

void mqtt_init() {
  mqtt_client_info.client_id = "OpenThermOstat";
  mqtt_client_info.client_user = "monitor";
  mqtt_client_info.client_pass = "UirHdEAPDxiOCB5939VQ";

  ip_addr_t ip;
  if (ipaddr_aton("192.168.0.41", &ip) != 1) {
    log("ipaddr_aton failed");
    return;
  }

  mutex_enter_blocking(&mqtt_mtx);

  {
    u16_t port = 1883;
    mqtt_connection_cb_t cb = [](mqtt_client_t *client, void *arg,
                                 mqtt_connection_status_t status) {
      log("mqtt connection status: %d", status);
      if (status == MQTT_CONNECT_DISCONNECTED ||
          status == MQTT_CONNECT_TIMEOUT) {
        log("mqtt reconnecting");
        mutex_enter_blocking(&mqtt_mtx);
        {
          if (mqtt_client)
            mqtt_client_free(mqtt_client);
          mqtt_client = NULL;
          mqtt_init();
        }
        mutex_exit(&mqtt_mtx);
      }
    };

    void *arg = NULL;
    mqtt_client = mqtt_client_new();

    if (!mqtt_client) {
      log("failed to allocate new mqtt client");
      mutex_exit(&mqtt_mtx);
      return;
    }

    err_t r =
        mqtt_client_connect(mqtt_client, &ip, port, cb, arg, &mqtt_client_info);
    if (r != ERR_OK) {
      log("mqtt_client_connect failed: %d", r);
      mutex_exit(&mqtt_mtx);
      return;
    }

    r = mqtt_subscribe(
        mqtt_client, mqtt_cmd_in_topic, 0,
        [](void *arg, err_t err) { log("mqtt mqtt_subscribe: %d", err); },
        NULL);
    if (r != ERR_OK) {
      log("mqtt_subscribe failed: %d", r);
      mutex_exit(&mqtt_mtx);
      return;
    }

    mqtt_set_inpub_callback(mqtt_client, mosq_publish_cb, mosq_data_cb, arg);
  }

  mutex_exit(&mqtt_mtx);
}

static bool wifi_link_down = true;

PicoTimer wifi_link_timer(0, 1000000, [](OpenTherm::Timer *, void *) {
  int ws = cyw43_wifi_link_status(&cyw43_state, 0);
  int ts = cyw43_tcpip_link_status(&cyw43_state, 0);
  if (wifi_link_down)
    log("Wifi link status: wifi=%d tcpip=%d", ws, ts);
  if (ws <= 0) {
    ws = cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, WIFI_AUTH);
    wifi_link_down = true;
  }
  if (wifi_link_down && ts == CYW43_LINK_UP) {
    log("Wifi link up; ip=%s", ipaddr_ntoa(&cyw43_state.netif[0].ip_addr));
    wifi_link_down = false;
    mqtt_init();
  }
  return true;
});

bool wifi_connect() {
  if (cyw43_arch_init_with_country(CYW43_COUNTRY_UK)) {
    log("wifi init failed.");
    return false;
  }

  cyw43_wifi_pm(&cyw43_state, CYW43_AGGRESSIVE_PM);
  // gpio_put(CYW43_PIN_WL_REG_ON, 0);
  // cyw43_arch_gpio_put(1, 0);

  log("Connecting to '%s'", WIFI_SSID);
  cyw43_arch_enable_sta_mode();
  wifi_link_timer.start();

  return true;
}

extern "C" {
#define MAX_RESPONSE_SIZE 512

struct FSCustomData {
  uint64_t request_id = -1;
  const char *status = NULL;
  char lbuf[MAX_RESPONSE_SIZE];
  size_t lbuf_sz = 0;
};

static int make_http_response(char *state, FSCustomData *data) {
  auto &sz = data->lbuf_sz;
  auto *status = data->status;

  if (!status && data->request_id != NoRequestID) {
    Frame response;
    switch (otdev.get(data->request_id, response)) {
    case RequestStatus::OK:
      sz = snprintf(data->lbuf, MAX_RESPONSE_SIZE, "%08lx", (uint32_t)response);
      status = "200";
      break;
    case RequestStatus::TIMED_OUT:
      sz = snprintf(data->lbuf, MAX_RESPONSE_SIZE, "timed out");
      status = "408";
      break;
    default:
      sz = snprintf(data->lbuf, MAX_RESPONSE_SIZE,
                    "request failed with status %d", (int)status);
      status = "500";
    }
  }

  return snprintf(state, MAX_RESPONSE_SIZE,
                  "HTTP/1.1 %s OK\ncontent-length: %d\n\n%s\n", status, sz + 1,
                  data->lbuf);
}

int fs_open_custom(struct fs_file *file, const char *name) {
  if (strcmp(name, "/ot.ssi") == 0 || strcmp(name, "/otget.ssi") == 0 ||
      strcmp(name, "/temp.ssi") == 0) {

    memset(file, 0, sizeof(struct fs_file));
    file->flags = FS_FILE_FLAGS_CUSTOM | FS_FILE_FLAGS_SSI;
    file->state = fs_state_init(file, name);
    file->len = 0;
    file->index = 0;

    auto data = (FSCustomData *)mem_malloc(sizeof(FSCustomData));
    data->request_id = NoRequestID;
    data->status = NULL;
    data->lbuf_sz = 0;

    file->pextension = data;

    return 1;
  }
  return 0;
}

void fs_close_custom(struct fs_file *file) {
  if (file && file->pextension) {
    mem_free(file->pextension);
    file->pextension = NULL;
  }
}

void *fs_state_init(struct fs_file *file, const char *name) {
  void *r = mem_calloc(MAX_RESPONSE_SIZE, 1);
  if (!r)
    log("FS state allocation failed");
  return r;
}

void fs_state_free(struct fs_file *file, void *state) {
  if (state)
    mem_free(state);
  file->state = NULL;
}

u8_t fs_canread_custom(struct fs_file *file) {
  auto data = (FSCustomData *)file->pextension;
  if (data == NULL)
    return 1;
  bool ready = true;
  if (data->request_id != NoRequestID)
    ready = otdev.is_finished(data->request_id);
  if (ready && file->len == 0)
    file->len = make_http_response((char *)file->state, data);
  return ready ? 1 : 0;
}

u8_t fs_wait_read_custom(struct fs_file *file, fs_wait_cb callback_fn,
                         void *callback_arg) {
  // log("fs_wait_read_custom cb=%p", callback_fn);
  return 1;
}

int fs_read_async_custom(struct fs_file *file, char *buffer, int count,
                         fs_wait_cb callback_fn, void *callback_arg) {
  const char *from = (const char *)file->state + file->index;
  int rem = file->len - file->index;
  int len = count > rem ? rem : count;
  memcpy(buffer, from, len);
  file->index += count;
  return len;
}

void httpd_cgi_handler(struct fs_file *file, const char *uri, int num_params,
                       char **keys, char **values, void *state) {
  // log("httpd_cgi_handler: %s", uri);
  if (state != NULL) {
    auto data = (FSCustomData *)file->pextension;
    if (data == NULL) {
      data->lbuf_sz =
          snprintf(data->lbuf, sizeof(data->lbuf), "missing extension data");
      data->status = "500";
    } else {
      if (strcmp(uri, "/ot.ssi") == 0) {
        bool found_msg = false;
        for (int i = 0; i < num_params; i++) {
          if (strcmp(keys[i], "msg") == 0) {
            uint32_t msg = 0;
            found_msg = true;
            if (sscanf(values[i], "%08lx", &msg) != 1) {
              data->lbuf_sz =
                  snprintf(data->lbuf, sizeof(data->lbuf),
                           "'%s' is not a hex-encoded uint32_t", values[i]);
              data->status = "400";
            } else {
              log("OpenTherm request: msg=%08x", msg);
              data->request_id = otdev.tx(Frame(msg));
              if (data->request_id == NoRequestID) {
                data->lbuf_sz = snprintf(data->lbuf, sizeof(data->lbuf),
                                         "too many requests (%u)",
                                         otdev.num_outstanding_requests());
                data->status = "429";
              }
            }
          }
        }
        if (!found_msg) {
          data->lbuf_sz = snprintf(data->lbuf, sizeof(data->lbuf),
                                   "missing parameter 'msg'");
          data->status = "400";
        }
      } else if (strcmp(uri, "/temp.ssi") == 0) {
        data->lbuf_sz = snprintf(data->lbuf, sizeof(data->lbuf), "%2.3f",
                                 get_core_temperature());
        data->status = "200";
      } else
        data->status = "404";
    }
  }
}
}

void core1_main() {
  // pio_interrupt_clear(pio0, 0);
  // pio_set_irq0_source_enabled(pio0, pis_interrupt0, true);
  // irq_set_exclusive_handler(PIO0_IRQ_0, []() {
  //   if (pio_interrupt_get(pio0, 0)) {
  //     pio_interrupt_clear(pio0, 0);
  //     ::log("irq: %08x; dropped frame", pio0->irq);
  //   }
  // });
  // irq_set_enabled(PIO0_IRQ_0, true);

  otdev.rx_forever(
      [](const Frame &f) {
        char payload[9];
        int len = snprintf(payload, sizeof(payload), "%08lx", (uint32_t)f);
        mutex_enter_blocking(&mqtt_mtx);
        if (mqtt_client && mqtt_client_is_connected(mqtt_client))
          mqtt_publish(mqtt_client, mqtt_topic, payload, len, 0, 0, NULL, NULL);
        mutex_exit(&mqtt_mtx);
      },
      [](bool on) { cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on ? 1 : 0); });
}

int main() {
  stdio_init_all();
  sleep_ms(1000);

  log("OpenThermOstat starting...");

  char board_id[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
  pico_get_unique_board_id_string(board_id, sizeof(board_id));
  log("Board ID: %s", board_id);

  // clocks_init();
  // uint vco = PICO_PLL_VCO_MIN_FREQ_MHZ * 1e6, pd1=7, pd2=7;
  // uint vco = 1000 * 1e6, pd1 = 4, pd2 = 4;
  // check_sys_clock_khz(20000, &vco, &pd1, &pd2);
  // if (check_sys_clock_khz(20000, &vco, &pd1, &pd2))
  // set_sys_clock_pll(vco, pd1, pd2);

  log("Running at %.2f MHz", clock_get_hz(clk_sys) / 1e6);
  // log("(vco=%d pd1=%d pd2=%d)", vco, pd1, pd2);

  adc_init();
  adc_set_temp_sensor_enabled(true);

  if (!wifi_connect())
    log("Could not connect to wifi.");

  multicore_launch_core1(core1_main);
  otdev.start();

  httpd_init();

  otdev.tx_forever();

  return 0;
}
